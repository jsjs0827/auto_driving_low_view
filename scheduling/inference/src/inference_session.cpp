#include "inference_session_api.hpp"
#include "perception_sdk.h"
#include "dag_pipeline.hpp"
#include "visualization.hpp"
#ifdef R29_WITH_ORT
#include "cpu_stages.hpp"
#endif
#ifdef PSDK_INTERNAL_TESTS
#include "sdk_test_support.hpp"
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

struct psdk_engine {
    std::mutex mutex;
    std::unique_ptr<r25::Pipeline> pipeline;
    demo_vis::Style style;
    bool detection_enabled=true, segmentation_enabled=true;
};
struct psdk_result {
    r25::Output output;
    std::vector<psdk_detection> detections;
    demo_vis::Style style;
    uint32_t width=0,height=0;
    perception::FrameInfo frame;
    psdk_timings timings{};
};

namespace {
using ai_node_percep_demo::DnnConfig;
constexpr uint32_t config_invalid=2001, image_invalid=1002, init_failed=1005;
constexpr uint32_t inference_failed=1011, result_invalid=1012, output_failed=2002, cleanup_failed=2006;
using Clock=std::chrono::steady_clock;
// Preserved exact-input destination caches are process-global. Model lifecycle
// also invalidates them, so SDK entry points must serialize create/process/close.
std::mutex sdk_lifecycle_mutex;
double elapsed(Clock::time_point begin) {return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();}
template<std::size_t N> void copyText(char (&out)[N],const char* text) noexcept {
    if(!text){out[0]=0;return;}
    std::size_t n=0;while(n+1<N && text[n]){out[n]=text[n];++n;}out[n]=0;
}
uint32_t putStatus(psdk_status* out,uint32_t code=0,int native=0,
                   perception::FrameInfo frame={},const char* feature="",const char* task="",const char* detail="") noexcept {
    if(out){std::memset(out,0,sizeof(*out));out->struct_size=sizeof(*out);out->abi_version=PSDK_ABI_VERSION;
        out->code=code;out->native_code=native;out->frame_id=frame.id;out->timestamp_ns=frame.timestamp_ns;
        copyText(out->feature,feature);copyText(out->task,task);copyText(out->detail,detail);}
    return code;
}
uint32_t putStatus(psdk_status* out,const r28::Status& value) noexcept {
    return putStatus(out,value.code,value.native_code,value.frame,value.feature.c_str(),value.task.c_str(),value.detail.c_str());
}
uint32_t caught(psdk_status* out,const std::exception& error,uint32_t fallback,
                const char* feature,perception::FrameInfo frame={}) noexcept {
    if(const auto* failure=dynamic_cast<const r28::Failure*>(&error)){
        const auto& s=failure->status();
        const auto f=(s.frame.id||s.frame.timestamp_ns)?s.frame:frame;
        return putStatus(out,s.code?s.code:fallback,s.native_code,f,s.feature.empty()?feature:s.feature.c_str(),s.task.c_str(),s.detail.c_str());
    }
    if(const auto* typed=dynamic_cast<const perception::PerceptionException*>(&error)){
        const auto& s=typed->info();const auto f=(s.frame.id||s.frame.timestamp_ns)?s.frame:frame;
        const auto code=static_cast<uint32_t>(s.code);
        return putStatus(out,code?code:fallback,0,f,s.component.empty()?feature:s.component.c_str(),"",s.detail.c_str());
    }
    const auto* system=dynamic_cast<const std::system_error*>(&error);
    return putStatus(out,fallback,system?system->code().value():0,frame,feature,"",error.what());
}
[[noreturn]] void invalid(const std::string& message){throw std::invalid_argument(message);}
bool abi(uint32_t size,uint32_t version,std::size_t expected) noexcept {return size==expected&&version==PSDK_ABI_VERSION;}
std::string trim(const std::string& value){const auto a=value.find_first_not_of(" \t\r\n");if(a==std::string::npos)return {};return value.substr(a,value.find_last_not_of(" \t\r\n")-a+1);}
int number(const std::string& value,const char* name,int lo,int hi){
    std::size_t count=0;int result=0;try{result=std::stoi(value,&count);}catch(...){invalid(std::string("Invalid integer: ")+name);}
    if(count!=value.size()||result<lo||result>hi)invalid(std::string("Invalid integer range: ")+name);return result;
}
float scalar(const std::string& value,const char* name,float lo,float hi,bool allow_zero){
    std::size_t count=0;float result=0;try{result=std::stof(value,&count);}catch(...){invalid(std::string("Invalid float: ")+name);}
    if(count!=value.size()||!std::isfinite(result)||result>hi||(allow_zero?result<lo:result<=lo))invalid(std::string("Invalid float range: ")+name);return result;
}
std::vector<std::string> names(const std::string& csv){
    std::vector<std::string> out;std::size_t begin=0;
    for(;;){const auto end=csv.find(',',begin);auto value=trim(csv.substr(begin,end==std::string::npos?end:end-begin));
        if(value.empty())invalid("Empty class name in profile");out.push_back(std::move(value));if(end==std::string::npos)break;begin=end+1;}
    return out;
}
DnnConfig loadOriginal(const char* path){
    const std::string file=path?path:"profiles/original.cfg";if(file.empty())invalid("original_profile must not be empty");
    std::ifstream input(std::filesystem::u8path(file),std::ios::binary);if(!input)invalid("Cannot open original profile: "+file);
    input.seekg(0,std::ios::end);const auto bytes=input.tellg();if(bytes<0||bytes>1024*1024)invalid("Original profile exceeds 1 MiB");input.seekg(0);
    std::map<std::string,std::string> fields;std::string line;
    const std::set<std::string> required={"det_model","seg_model","det_classes","det_names","score_threshold","nms_threshold","reg_bins","nms_top_k","seg_names","parallel_det_seg","seg_align_corners","seg_argmax_mode","det_resize_type","seg_resize_type","det_width","det_height","seg_width","seg_height"};
    while(std::getline(input,line)){
        line=trim(line);if(line.empty()||line[0]=='#')continue;
        if(line.find('\0')!=std::string::npos)invalid("NUL in original profile");
        const auto equals=line.find('=');if(equals==std::string::npos)invalid("Profile line requires key=value");
        const auto key=trim(line.substr(0,equals)),value=trim(line.substr(equals+1));
        if((required.count(key)==0&&key!="keep_classes")||!fields.emplace(key,value).second)invalid("Unknown/duplicate original profile field: "+key);
    }
    if(input.bad())invalid("Cannot read original profile: "+file);
    for(const auto& key:required)if(fields.count(key)==0||fields.at(key).empty())invalid("Missing original profile field: "+key);
    const auto integer=[&](const char* key,int lo,int hi){return number(fields.at(key),key,lo,hi);};
    if(integer("det_resize_type",0,1)!=1||integer("seg_resize_type",0,1)!=0||integer("seg_align_corners",0,1)!=1||fields.at("seg_argmax_mode")!="bilinear")invalid("Original letterbox, stretch and align-corners bilinear segmentation are required");
    if(integer("det_width",1,8192)!=864||integer("det_height",1,8192)!=480||integer("seg_width",1,8192)!=832||integer("seg_height",1,8192)!=480)invalid("Original model geometry must be detection 864x480 and segmentation 832x480");
    if(integer("parallel_det_seg",0,1)!=1)invalid("Shared SDK requires the original same-frame parallel DAG");
    DnnConfig cfg;cfg.enabled=true;cfg.parallel_det_seg=true;cfg.execution_schedule="legacy";
    // Profiles remain portable when launched from another working directory.
    // Existence is checked by the enabled model backend, not this config loader.
    const auto modelPath=[&](const char* key){
        auto value=std::filesystem::u8path(fields.at(key));
        if(value.is_relative())value=std::filesystem::absolute(std::filesystem::u8path(file)).parent_path()/value;
        return value.lexically_normal().u8string();
    };
    cfg.detect.model_path=modelPath("det_model");cfg.seg.model_path=modelPath("seg_model");
    cfg.detect.classes_num=integer("det_classes",1,256);cfg.detect.class_names=fields.at("det_names");
    if(int(names(cfg.detect.class_names).size())!=cfg.detect.classes_num)invalid("Detector class_names count differs from det_classes");
    cfg.detect.score_threshold=scalar(fields.at("score_threshold"),"score_threshold",0,1,false);
    if(cfg.detect.score_threshold>=1)invalid("score_threshold must be less than one");
    cfg.detect.nms_threshold=scalar(fields.at("nms_threshold"),"nms_threshold",0,1,true);
    cfg.detect.reg=integer("reg_bins",2,64);cfg.detect.nms_top_k=integer("nms_top_k",1,10000);cfg.detect.resize_type=1;
    // Match the original worker: keep the DnnDetectConfig default filter unless
    // the caller explicitly supplied a keep_classes field. Never clear it implicitly.
    if(fields.count("keep_classes"))cfg.detect.keep_classes=fields.at("keep_classes");
    cfg.seg.class_names=fields.at("seg_names");if(names(cfg.seg.class_names).size()>256)invalid("Too many segmentation classes");
    cfg.seg.resize_type=0;cfg.seg.align_corners=true;cfg.seg.argmax_mode="bilinear";
    return cfg;
}
perception::ImageView validateFrame(const psdk_frame* frame){
    const auto bad=[&](const char* text){throw perception::PerceptionException(perception::ErrorCode::CameraDataInvalid,text,"sdk.input");};
    if(!frame||!abi(frame->struct_size,frame->abi_version,sizeof(*frame)))bad("Invalid frame ABI/struct_size");
    if(frame->format!=PSDK_FORMAT_NV12&&frame->format!=PSDK_FORMAT_I420)bad("Only explicit NV12 or I420 is accepted");
    if(!frame->data||!frame->width||!frame->height||frame->width>8192||frame->height>8192||frame->width%2||frame->height%2||frame->stride%2||frame->stride<frame->width||frame->stride>16U*1024U*1024U)bad("Invalid YUV420 pointer, even geometry or stride");
    const uint64_t base=uint64_t(frame->stride)*frame->height;
    if(base>std::numeric_limits<uint64_t>::max()-base/2)bad("Image storage extent overflow");
    const uint64_t needed=base+base/2;
    if(frame->bytes<needed||frame->bytes>std::numeric_limits<std::size_t>::max()||needed>std::numeric_limits<std::size_t>::max())bad("Short or unrepresentable YUV420 buffer");
    return {frame->data,static_cast<std::size_t>(frame->bytes),frame->stride,static_cast<int>(frame->width),static_cast<int>(frame->height),frame->format==PSDK_FORMAT_NV12?perception::PixelFormat::NV12:perception::PixelFormat::I420};
}
std::unique_ptr<psdk_engine> createEngine(const psdk_config& config,std::shared_ptr<r25::Stages> injected={}){
    if(!abi(config.struct_size,config.abi_version,sizeof(config))||config.reserved_flags)invalid("Invalid config ABI/struct_size/reserved_flags");
    if(config.backend!=PSDK_BACKEND_BPU&&config.backend!=PSDK_BACKEND_CPU_ORT)invalid("Unknown inference backend");
    if(config.backend!=PSDK_BACKEND_CPU_ORT&&config.cpu_profile)invalid("cpu_profile requires the CPU backend");
    if(config.dag_profile&&!config.dag_profile[0])invalid("dag_profile must not be empty");
    if(config.detection_model&&!config.detection_model[0])invalid("detection_model must not be empty");
    if(config.segmentation_model&&!config.segmentation_model[0])invalid("segmentation_model must not be empty");
    const auto inference_graph=perception::runtime::loadPerceptionEngineConfig(config.dag_profile?config.dag_profile:"profiles/dag_exact.json");
    std::array<std::set<std::string>,2> lane_features;
    for(const auto& task:inference_graph.tasks)if(task.enabled)for(const auto& feature:task.features) {
        for(int lane=0;lane<2;++lane) {
            const std::string prefix=lane?"ai.exact.seg.":"ai.exact.det.";
            if(feature.compare(0,prefix.size(),prefix)==0)lane_features[lane].insert(feature.substr(prefix.size()));
        }
    }
    for(const auto& selected:lane_features)if(!selected.empty()&&selected!=std::set<std::string>{"preprocess","backend","postprocess"})
        invalid("An enabled inference lane requires preprocess, backend and postprocess");
    const bool detect=!lane_features[0].empty(),segment=!lane_features[1].empty();
    if(!detect&&!segment)invalid("SDK inference graph must enable detection or segmentation");
    auto cfg=loadOriginal(config.original_profile);
    cfg.detect.enabled=detect;cfg.seg.enabled=segment;
    std::shared_ptr<r25::Stages> stages=std::move(injected);
    if(!stages){
        if(config.backend==PSDK_BACKEND_BPU){
#ifdef R20_WITH_BPU
            if(config.detection_model)cfg.detect.model_path=config.detection_model;
            if(config.segmentation_model)cfg.seg.model_path=config.segmentation_model;
            try{stages=std::make_shared<r25::NativeStages>(cfg);}
            catch(const std::exception& e){throw r28::Failure({init_failed,0,"sdk.bpu.init",{},e.what(),{}});}
#else
            throw r28::Failure({config_invalid,0,"sdk.backend",{},"BPU backend is unavailable in this build; cross-build with R20_WITH_BPU=ON",{}});
#endif
        }else{
#ifdef R29_WITH_ORT
            if(!config.cpu_profile||!config.cpu_profile[0])invalid("CPU backend requires an explicit CPU ONNX profile");
            auto cpu=r29::loadCpuConfig(config.cpu_profile);cfg=cpu.dnn;
            cfg.detect.enabled=detect;cfg.seg.enabled=segment;
            if(config.detection_model)cfg.detect.model_path=config.detection_model;
            if(config.segmentation_model)cfg.seg.model_path=config.segmentation_model;
            cpu.dnn=cfg;r29::validateCpuConfig(cpu);
            stages=std::make_shared<r29::CpuStages>(cpu);
#else
            throw r28::Failure({config_invalid,0,"sdk.backend",{},"CPU backend is unavailable in this build; configure R29_WITH_ORT=ON and ORT_ROOT",{}});
#endif
        }
        if((detect&&(stages->width(0)!=864||stages->height(0)!=480))||(segment&&(stages->width(1)!=832||stages->height(1)!=480)))invalid("Loaded models violate the original input geometry");
    }
    auto out=std::make_unique<psdk_engine>();out->style.detection_names=names(cfg.detect.class_names);out->style.segmentation_names=names(cfg.seg.class_names);
    out->detection_enabled=detect;out->segmentation_enabled=segment;
    out->pipeline=std::make_unique<r25::Pipeline>(config.dag_profile?config.dag_profile:"profiles/dag_exact.json",std::move(stages));
    return out;
}
}

namespace perception::sdk_internal {
uint32_t impl_psdk_abi_version(void){return PSDK_ABI_VERSION;}
uint32_t impl_psdk_build_capabilities(void){
    uint32_t out=PSDK_CAP_PNG;
#ifdef R20_WITH_BPU
    out|=PSDK_CAP_BPU;
#endif
#ifdef R29_WITH_ORT
    out|=PSDK_CAP_CPU_ORT;
#endif
    return out;
}
uint32_t impl_psdk_create(const psdk_config* config,psdk_engine** engine,psdk_status* status){
    if(engine)*engine=nullptr;
    try{if(!config||!engine)invalid("config and engine output are required");std::lock_guard<std::mutex> gate(sdk_lifecycle_mutex);auto value=createEngine(*config);*engine=value.release();return putStatus(status);}
    catch(const std::exception& e){return caught(status,e,config_invalid,"sdk.create");}
    catch(...){return putStatus(status,init_failed,0,{},"sdk.create","","Unknown exception");}
}
uint32_t impl_psdk_process(psdk_engine* engine,const psdk_frame* frame,psdk_result** result,psdk_status* status){
    if(result)*result=nullptr;
    const auto identity=frame&&abi(frame->struct_size,frame->abi_version,sizeof(*frame))?perception::FrameInfo{frame->frame_id,frame->timestamp_ns}:perception::FrameInfo{};
    try{
        if(!engine||!result)invalid("engine and result output are required");
        const auto image=validateFrame(frame);std::lock_guard<std::mutex> gate(sdk_lifecycle_mutex);std::lock_guard<std::mutex> lock(engine->mutex);const auto begin=Clock::now();
        auto value=std::make_unique<psdk_result>();r28::Status failure;r25::RunOptions options;
        options.optimized=true;options.variant=r22::Variant::CompactDirect;
        options.run_detection=engine->detection_enabled;options.run_segmentation=engine->segmentation_enabled;
        const auto code=engine->pipeline->tryRun(image,identity,options,value->output,failure);if(code)return putStatus(status,failure);
        value->width=frame->width;value->height=frame->height;value->frame=identity;value->style=engine->style;
        if(engine->segmentation_enabled&&value->output.mask().size()!=std::size_t(frame->width)*frame->height)throw r28::Failure({result_invalid,0,"sdk.result",{},"Segmentation result does not cover the source frame",identity});
        value->detections.reserve(value->output.det.size());
        for(const auto& det:value->output.det){psdk_detection copy{};copy.class_id=det.class_id;copy.confidence=det.confidence;
            copy.x1=det.x1;copy.y1=det.y1;copy.x2=det.x2;copy.y2=det.y2;copyText(copy.class_name,det.class_name.c_str());value->detections.push_back(copy);}
        const auto& out=value->output;const auto feature=[&](int i){return out.features[i].end_ms-out.features[i].start_ms;};
        value->timings={out.pack_ms,feature(1),out.det_infer_ms,feature(3),feature(4),out.seg_infer_ms,feature(6),out.graph_ms,elapsed(begin)};
        *result=value.release();return putStatus(status,0,0,identity);
    }catch(const std::exception& e){return caught(status,e,inference_failed,"sdk.process",identity);}
    catch(...){return putStatus(status,inference_failed,0,identity,"sdk.process","","Unknown exception");}
}
uint32_t impl_psdk_result_get_view(const psdk_result* result,psdk_result_view* view,psdk_status* status){
    try{
        if(!result||!view||!abi(view->struct_size,view->abi_version,sizeof(*view)))invalid("Invalid result/view ABI/struct_size");
        psdk_result_view value{};value.struct_size=sizeof(value);value.abi_version=PSDK_ABI_VERSION;value.width=result->width;value.height=result->height;
        value.frame_id=result->frame.id;value.timestamp_ns=result->frame.timestamp_ns;value.detections=result->detections.empty()?nullptr:result->detections.data();value.detection_count=result->detections.size();
        value.segmentation_mask=result->output.mask().data();value.segmentation_bytes=result->output.mask().size();value.timings=result->timings;
        *view=value;return putStatus(status,0,0,result->frame);
    }catch(const std::exception& e){return caught(status,e,result_invalid,"sdk.result");}
    catch(...){return putStatus(status,result_invalid,0,{},"sdk.result","","Unknown exception");}
}
uint32_t impl_psdk_save_png(const psdk_result* result,const psdk_frame* frame,const char* path,psdk_status* status){
    try{
        if(!result||!path||!path[0])invalid("result and nonempty output path are required");const auto image=validateFrame(frame);
        if(frame->frame_id!=result->frame.id||frame->timestamp_ns!=result->frame.timestamp_ns||frame->width!=result->width||frame->height!=result->height)invalid("Overlay input and result frame identity/geometry differ");
        std::vector<uint8_t> rgb;demo_vis::convertRgbInto(image,rgb);
        demo_vis::overlayRgb(rgb,static_cast<int>(result->width),static_cast<int>(result->height),result->output.frame.detection.get(),result->output.frame.segmentation.get(),result->style);
        demo_vis::saveRgbPng(std::filesystem::u8path(path),rgb,static_cast<int>(result->width),static_cast<int>(result->height));
        return putStatus(status,0,0,result->frame);
    }catch(const std::exception& e){return caught(status,e,output_failed,"sdk.output",result?result->frame:perception::FrameInfo{});}
    catch(...){return putStatus(status,output_failed,0,{},"sdk.output","","Unknown exception");}
}
void impl_psdk_result_release(psdk_result* result){try{delete result;}catch(...){}}
uint32_t impl_psdk_destroy(psdk_engine** engine,psdk_status* status){
    try{
        if(!engine)invalid("engine pointer is required");std::lock_guard<std::mutex> gate(sdk_lifecycle_mutex);std::unique_ptr<psdk_engine> value(*engine);*engine=nullptr;if(!value)return putStatus(status);
        r28::Status failure;uint32_t code=0;{std::lock_guard<std::mutex> lock(value->mutex);code=value->pipeline->tryClose(failure);}
        value.reset();return code?putStatus(status,failure):putStatus(status);
    }catch(const std::exception& e){return caught(status,e,cleanup_failed,"sdk.destroy");}
    catch(...){return putStatus(status,cleanup_failed,0,{},"sdk.destroy","","Unknown exception");}
}
}

#ifdef PSDK_INTERNAL_TESTS
namespace psdk_test {
std::pair<std::string,std::string> originalModelPaths(const char* path) {
    const auto config=loadOriginal(path);
    return {config.detect.model_path,config.seg.model_path};
}
uint32_t create(const psdk_config* config,std::shared_ptr<r25::Stages> stages,psdk_engine** engine,psdk_status* status) noexcept {
    if(engine)*engine=nullptr;
    try{if(!config||!engine||!stages)invalid("Test config, injected stages and engine output required");std::lock_guard<std::mutex> gate(sdk_lifecycle_mutex);auto value=createEngine(*config,std::move(stages));*engine=value.release();return putStatus(status);}
    catch(const std::exception& e){return caught(status,e,config_invalid,"sdk.test.create");}
    catch(...){return putStatus(status,init_failed,0,{},"sdk.test.create","","Unknown exception");}
}
}
#endif
