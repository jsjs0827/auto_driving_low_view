#include "dag_pipeline.hpp"
#include "background_drain.hpp"
#include "original_input_contract.hpp"
#include "seg_post.hpp"
#include "exact_input.hpp"
#include "legacy_preprocess.hpp"
#include "percep_demo/horizon_dnn_runner.hpp"
#include "image_input.hpp"
#include "camera_source.hpp"
#include "resource_probe.hpp"
#include "visualization.hpp"
#include "platform_metrics.hpp"
#ifdef R29_WITH_ORT
#include "cpu_stages.hpp"
#endif
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cmath>
#include <thread>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <atomic>
#include <exception>
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
using namespace ai_node_percep_demo;
double elapsed(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
std::int64_t hostNs(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}
double cpuMs(){return r29::processCpuMs();}
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<char(c);else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<char(c);}o<<'"';return o.str();}
std::string read(const fs::path& path){std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot read "+path.string());return {std::istreambuf_iterator<char>(f),{}};}
void write(const fs::path& p,const void* data,std::size_t bytes){std::ofstream f(p,std::ios::binary);f.write(static_cast<const char*>(data),std::streamsize(bytes));if(!f)throw std::runtime_error("Write failed: "+p.string());}
void text(const fs::path& p,const std::string& s){write(p,s.data(),s.size());}
template<class T>void bytes(const fs::path& p,const std::vector<T>& v){write(p,v.data(),v.size()*sizeof(T));}
std::size_t different(const std::vector<std::uint8_t>& a,const std::vector<std::uint8_t>& b){if(a.size()!=b.size())return std::max(a.size(),b.size());std::size_t n=0;for(std::size_t i=0;i<a.size();++i)n+=a[i]!=b[i];return n;}
std::string byteDifference(const std::vector<std::uint8_t>& a,const std::vector<std::uint8_t>& b,int w,int h,bool float_nchw=false){
    std::ostringstream s;s<<"{\"baseline_bytes\":"<<a.size()<<",\"candidate_bytes\":"<<b.size()<<",\"different_bytes\":"<<different(a,b)<<",\"first\":[";
    int count=0;const auto plane=std::size_t(w)*h;
    for(std::size_t i=0;i<std::min(a.size(),b.size())&&count<20;++i)if(a[i]!=b[i]){
        if(float_nchw){
            if(count++)s<<',';const auto element=i/sizeof(float),position=element%plane;
            s<<"{\"offset\":"<<i<<",\"baseline\":"<<int(a[i])<<",\"candidate\":"<<int(b[i])
                <<",\"component\":\"float32_NCHW\",\"channel\":"<<element/plane<<",\"row\":"<<position/std::size_t(w)
                <<",\"x\":"<<position%std::size_t(w)<<",\"byte_in_float\":"<<i%sizeof(float)<<'}';
            continue;
        }
        if(count++)s<<',';const bool uv=i>=plane&&i<plane*3/2;const auto offset=uv?i-plane:i;
        s<<"{\"offset\":"<<i<<",\"baseline\":"<<int(a[i])<<",\"candidate\":"<<int(b[i])<<",\"component\":"
         <<quote(i<plane?"Y":uv?(offset%2?"V":"U"):"aligned_tail")<<",\"row\":"<<offset/std::size_t(w)<<",\"byte_x\":"<<offset%std::size_t(w)<<'}';
    }
    return s.str()+"]}";
}
std::uint64_t hash(const std::vector<std::uint8_t>& data){std::uint64_t v=14695981039346656037ULL;for(auto c:data){v^=c;v*=1099511628211ULL;}return v;}
std::map<std::string,long> resources(){return r29::processResources();}
std::map<std::string,std::string> profile(const fs::path& p){std::map<std::string,std::string> v;std::istringstream f(read(p));std::string line;while(std::getline(f,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();const auto eq=line.find('=');if(eq!=std::string::npos&&line[0]!='#')v[line.substr(0,eq)]=line.substr(eq+1);}return v;}
std::string boxes(const std::vector<DnnDetection>& v){std::ostringstream o;o<<std::setprecision(9)<<'[';bool first=true;for(const auto& b:v){if(!first)o<<',';first=false;o<<"{\"class_id\":"<<b.class_id<<",\"score\":"<<b.confidence<<",\"x1\":"<<b.x1<<",\"y1\":"<<b.y1<<",\"x2\":"<<b.x2<<",\"y2\":"<<b.y2<<",\"name\":"<<quote(b.class_name)<<'}';}return o.str()+']';}
bool sameBoxes(const std::vector<DnnDetection>& a,const std::vector<DnnDetection>& b){if(a.size()!=b.size())return false;for(std::size_t i=0;i<a.size();++i){if(a[i].class_id!=b[i].class_id||a[i].class_name!=b[i].class_name)return false;
    const float av[]={a[i].confidence,a[i].x1,a[i].y1,a[i].x2,a[i].y2},bv[]={b[i].confidence,b[i].x1,b[i].y1,b[i].x2,b[i].y2};if(std::memcmp(av,bv,sizeof av))return false;}return true;}
using Output=r25::Output;
struct Entry {int width=0,height=0,stride=0;std::string format,path;std::uint64_t id=0;};
using r27::BackgroundDrain;
int main(int argc,char** argv){fs::path out;std::uint32_t failure_code=r28::ConfigInvalid;
std::string error_stage="arguments";perception::FrameInfo failure_frame;try{
    std::map<std::string,std::string> a;
    const std::string allowed=" --backend --cpu-profile --copy-disparity --benchmark-workload --background-camera --stereo-trace --camera-retry-ms --dag-profile --close-policy --resource-snapshots --preprocess-profile --duration-seconds --reopen-every --reopen-warmup --reopen-settle-ms --preprocess --schedule --verify-reference --bpu-priority --mode --profile --out --frames --warmup --input --input-list --width --height --stride --format --stream --sdk --camera-config --disparity-mode --det-model --seg-model --save-every --visualize-every --seg-route --fused-model --allow-fused-differences --disparity-every ";
    for(int i=1;i<argc;++i){const std::string key=argv[i];if(key=="--help"){std::cout<<"dag_flow_worker --mode camera-only|capture|preprocess|verify|baseline|optimized --profile FILE --out NEW_DIR\n"
        <<" [--backend bpu|cpu --cpu-profile JSON (CPU requires ONNX profiles and frozen input)]\n"
        <<" [--input RAW --width 640 --height 544 --stride S --format nv12 | --input-list TSV]\n"
        <<" [--sdk SO --camera-config CFG --stream rgbd --format nv12 --disparity-mode 0]\n"
        <<" [--frames 120 --warmup 20 --save-every 0 --visualize-every 0 --det-model BIN_OR_ONNX --seg-model BIN_OR_ONNX]\n"
        <<"[--benchmark-workload none|det-seg-only|sdk-disparity|rgbd-det-seg|rgbd-three-parallel --background-camera 0|1 --stereo-trace 0|1 --camera-retry-ms N]\n"
        <<"[--schedule parallel|serial --preprocess r21|cached|direct|compact_cached|compact_direct --preprocess-profile 0|1 --duration-seconds S --reopen-every N --verify-reference original|r21 --bpu-priority 0..255]\nDefault: original map RGBD-left 640x544 NV12, original Engine/DAG/Task/Feature, compact_direct exact input, unchanged segmentation acceleration; serial frames. Mode2 SDK disparity is a background workload. Explicit rgbd-three-parallel probes mode1 same-frame three-branch dispatch.\n";return 0;}
        if(allowed.find(" "+key+" ")==std::string::npos||i+1>=argc)throw std::invalid_argument("Unknown/incomplete option: "+key);a[key]=argv[++i];}
    const auto get=[&](const std::string& k,const std::string& d=""){const auto it=a.find(k);return it==a.end()?d:it->second;};
    const auto number=[&](const std::string& k,int d){return std::stoi(get(k,std::to_string(d)));};
    const auto backend=get("--backend","bpu");
    if(backend!="bpu"&&backend!="cpu")throw std::invalid_argument("--backend must be bpu or cpu");
    const bool cpu_backend=backend=="cpu";
    if(!cpu_backend&&!get("--cpu-profile").empty())throw std::invalid_argument("--cpu-profile requires --backend cpu");
    const auto mode=get("--mode","verify");if(mode!="camera-only"&&mode!="capture"&&mode!="preprocess"&&mode!="verify"&&mode!="baseline"&&mode!="optimized")throw std::invalid_argument("Invalid mode");
    const int count=number("--frames",120),warmup=number("--warmup",20),save=number("--save-every",0),visual=number("--visualize-every",0);
    if(count<1||count>100000||warmup<0||warmup>10000||save<0||visual<0)throw std::invalid_argument("Invalid bounds");
    const double duration_seconds=std::stod(get("--duration-seconds","0"));
    const int reopen_every=number("--reopen-every",0),reopen_warmup=number("--reopen-warmup",5),reopen_settle_ms=number("--reopen-settle-ms",1000);
    if(!std::isfinite(duration_seconds)||duration_seconds<0||duration_seconds>86400||reopen_every<0||reopen_warmup<0||reopen_warmup>1000||reopen_settle_ms<0||reopen_settle_ms>30000)throw std::invalid_argument("Invalid duration/reopen bounds");
    r22::setProfiling(number("--preprocess-profile",0)!=0);
    out=get("--out");if(out.empty()||fs::exists(out))throw std::invalid_argument("A fresh --out directory is required");fs::create_directories(out);
    std::ofstream events(out/"resource_events.jsonl",std::ios::binary);const auto event_start=Clock::now();int camera_cycle=1;
    const bool snapshots=number("--resource-snapshots",1)!=0;
    const auto close_policy=get("--close-policy","restore");
    if(close_policy!="restore"&&close_policy!="hold")throw std::invalid_argument("Invalid --close-policy restore|hold");
    std::ofstream lifecycle(out/"camera_lifecycle.jsonl",std::ios::binary),depth_events(out/"disparity_requests.jsonl",std::ios::binary),mode_events(out/"camera_modes.jsonl",std::ios::binary);int close_error=0,close_error_cycle=0,snapshot_index=0,effective_mode=-1;std::string close_error_call;
    const auto resource_event=[&](const char* phase){
        const std::string snapshot=snapshots?"resource_"+std::to_string(++snapshot_index)+".json":"";
        if(snapshots)text(out/snapshot,r24::procSnapshot());
        const auto r=resources();events<<"{\"phase\":"<<quote(phase)<<",\"elapsed_ms\":"<<elapsed(event_start,Clock::now())<<",\"camera_cycle\":"<<camera_cycle<<",\"snapshot\":"<<quote(snapshot)<<",\"effective_disparity_mode\":"<<effective_mode<<",\"resources\":{";bool first=true;for(const auto& v:r){if(!first)events<<',';first=false;events<<quote(v.first)<<':'<<v.second;}events<<"}}\n";events.flush();if(!events)throw std::runtime_error("Resource event write failed");};
    resource_event("before_model_init");
    const auto profile_path=fs::absolute(fs::u8path(get("--profile","profiles/original.cfg"))).lexically_normal();
    auto p=profile(profile_path);
    if(p.at("det_resize_type")!="1"||p.at("seg_resize_type")!="0")throw std::invalid_argument("Original detector letterbox and segmentation direct resize are required");
    DnnConfig cfg;cfg.enabled=true;cfg.parallel_det_seg=std::stoi(p.at("parallel_det_seg"))!=0;
    const auto profile_model=[&](const char* key){
        auto path=fs::u8path(p.at(key));
        if(path.is_relative())path=profile_path.parent_path()/path;
        return path.lexically_normal().u8string();
    };
    cfg.detect.model_path=get("--det-model",profile_model("det_model"));cfg.seg.model_path=get("--seg-model",profile_model("seg_model"));
    cfg.detect.classes_num=std::stoi(p.at("det_classes"));cfg.detect.class_names=p.at("det_names");cfg.detect.score_threshold=std::stof(p.at("score_threshold"));
    cfg.detect.nms_threshold=std::stof(p.at("nms_threshold"));cfg.detect.reg=std::stoi(p.at("reg_bins"));cfg.detect.nms_top_k=std::stoi(p.at("nms_top_k"));cfg.detect.resize_type=1;
    cfg.seg.resize_type=0;cfg.seg.align_corners=std::stoi(p.at("seg_align_corners"))!=0;cfg.seg.argmax_mode=p.at("seg_argmax_mode");cfg.seg.class_names=p.at("seg_names");
    int dw=std::stoi(p.at("det_width")),dh=std::stoi(p.at("det_height")),gw=std::stoi(p.at("seg_width")),gh=std::stoi(p.at("seg_height"));
#ifdef R29_WITH_ORT
    std::unique_ptr<r29::CpuConfig> cpu_config;
    if(cpu_backend){
        if(get("--cpu-profile").empty())throw std::invalid_argument("CPU inference requires an explicit --cpu-profile JSON");
        cpu_config=std::make_unique<r29::CpuConfig>(r29::loadCpuConfig(get("--cpu-profile")));
        cfg=cpu_config->dnn;
        cfg.detect.model_path=get("--det-model",cfg.detect.model_path);
        cfg.seg.model_path=get("--seg-model",cfg.seg.model_path);
        cpu_config->dnn=cfg;
    }
#else
    if(cpu_backend)throw std::invalid_argument("CPU backend was not built; configure R29_WITH_ORT=ON with ORT_ROOT");
#endif
    const auto schedule=get("--schedule","parallel");
    cfg.parallel_det_seg=schedule=="parallel";
    const auto input_variant=r22::parseVariant(get("--preprocess","compact_direct"));
    const auto verify_reference=get("--verify-reference","original");
    if(schedule!="parallel"&&schedule!="serial")throw std::invalid_argument("Invalid schedule");
    if(verify_reference!="original"&&verify_reference!="r20"&&verify_reference!="r21")throw std::invalid_argument("Invalid verify reference");
    cfg.execution_schedule="legacy";cfg.bpu_priority=number("--bpu-priority",0);
    if(cfg.bpu_priority<0||cfg.bpu_priority>255)throw std::invalid_argument("Invalid priority");
    const auto route=get("--seg-route","exact");
    if(route!="exact" || !get("--fused-model").empty() || number("--allow-fused-differences",0))throw std::invalid_argument("DAG package requires exact original model route");
    const bool fusion=false,allow_fused=false;
    const auto workload=get("--benchmark-workload","none");
    const bool rgbd_mode1=workload=="rgbd-det-seg"||workload=="rgbd-three-parallel";
    const int disparity_every=number("--disparity-every",0);
    if(disparity_every<0 || (disparity_every>0 && !rgbd_mode1 && (get("--stream","rgbd")!="gdc"||number("--disparity-mode",0)!=1)))throw std::invalid_argument("Demand disparity requires an explicit tested stream/mode contract");
    const int background_value=number("--background-camera",0),trace_value=number("--stereo-trace",0);
    if((background_value!=0&&background_value!=1)||(trace_value!=0&&trace_value!=1))throw std::invalid_argument("Benchmark flags must be 0 or 1");
    const bool background_camera=background_value!=0,stereo_trace=trace_value!=0;
    if(workload!="none"&&workload!="det-seg-only"&&workload!="sdk-disparity"&&!rgbd_mode1)throw std::invalid_argument("Invalid benchmark workload");
    if(workload=="none"){
        r26::validateSelection(get("--stream","rgbd"),get("--format","nv12"),number("--disparity-mode",0),disparity_every);
        if(background_camera||stereo_trace)throw std::invalid_argument("Explicit benchmark workload required for background camera or stereo tracing");
    }else{
        if(get("--stream","rgbd")!="rgbd"||get("--format","nv12")!="nv12"||(disparity_every&&workload!="rgbd-three-parallel")||schedule!="parallel"||reopen_every||a.count("--dag-profile"))
            throw std::invalid_argument("Benchmark requires original RGBD/NV12 input, parallel det/seg, no demand substitution or reopen");
        if(workload=="det-seg-only"&&(background_camera||stereo_trace||number("--disparity-mode",0)!=0||mode!="optimized"))
            throw std::invalid_argument("det-seg-only requires frozen input, no SDK, and optimized mode");
        if(workload=="sdk-disparity"&&(number("--disparity-mode",0)!=2||!stereo_trace))
            throw std::invalid_argument("sdk-disparity requires explicit mode 2 and stereo callback tracing");
        if(workload=="sdk-disparity"&&mode!="optimized"&&mode!="capture"&&mode!="camera-only")
            throw std::invalid_argument("SDK workload requires optimized, capture or camera-only mode");
        if(rgbd_mode1&&(mode!="optimized"||background_camera||number("--disparity-mode",0)!=1||!stereo_trace||
            disparity_every!=(workload=="rgbd-three-parallel"?1:0)))
            throw std::invalid_argument("RGBD mode1 probe requires live optimized input, explicit mode1, tracing and the controlled demand interval");
    }
    if(cpu_backend){
        if(get("--input").empty()&&get("--input-list").empty())throw std::invalid_argument("x86 CPU requires frozen original NV12 --input or --input-list, not the X5 camera SDK");
        if(!get("--input").empty()&&!get("--input-list").empty())throw std::invalid_argument("Choose one CPU input source");
        if(workload!="none"||background_camera||stereo_trace||disparity_every||reopen_every||
           number("--copy-disparity",0)||number("--disparity-mode",0)||a.count("--sdk")||a.count("--camera-config")||
           cfg.bpu_priority||mode=="camera-only"||mode=="capture")
            throw std::invalid_argument("x86 CPU does not execute X5 camera, disparity, BPU priority or hardware benchmark options");
    }
    const bool inference=mode=="verify"||mode=="baseline"||mode=="optimized";
    std::shared_ptr<r25::Stages> native_stages;
    failure_code=1010;error_stage="model.initialize";
    if(inference){
        if(cpu_backend){
#ifdef R29_WITH_ORT
            cpu_config->dnn=cfg;
            auto cpu=std::make_shared<r29::CpuStages>(*cpu_config);
            text(out/"cpu_model_io.json",cpu->metadataJson());
            text(out/"cpu_profile.json",read(get("--cpu-profile")));
            native_stages=std::move(cpu);
#endif
        }else{
#ifdef R20_WITH_BPU
            native_stages=std::make_shared<r25::NativeStages>(cfg);
#else
            throw std::runtime_error("BPU backend unavailable on this build; use --backend cpu with an ORT-enabled build");
#endif
        }
        if(!native_stages)throw std::logic_error("Requested inference backend was not initialized");
        dw=native_stages->width(0);dh=native_stages->height(0);gw=native_stages->width(1);gh=native_stages->height(1);
    }
    const bool input_only=mode=="capture"||mode=="camera-only";
    failure_code=r28::ConfigInvalid;error_stage="dag.configure";
    const auto graph_path=input_only?fs::path(get("--profile","profiles/original.cfg")).parent_path()/"dag_capture.json":fs::path(get("--dag-profile",(fs::path(get("--profile","profiles/original.cfg")).parent_path()/(workload=="rgbd-three-parallel"?"dag_exact_depth_parallel.json":schedule=="serial"?"dag_exact_serial.json":"dag_exact.json")).string()));
    auto pipeline=std::make_unique<r25::Pipeline>(graph_path.string(),native_stages);
    text(out/"dag_profile.json",read(graph_path));
    std::ostringstream registered;registered<<'[';bool first_feature=true;for(const auto& f:pipeline->registeredFeatures()){if(!first_feature)registered<<',';first_feature=false;registered<<quote(f);}text(out/"registered_features.json",registered.str()+"]\n");
    std::ofstream graph_csv(out/"graph_frames.csv",std::ios::binary),trace(out/"dag_trace.jsonl",std::ios::binary);
    graph_csv<<"run_id,frame_id,timestamp_ns,optimized,reference,graph_ms,ai_wall_ms,pack_ms,depth_feature_ms,task_count,feature_count,begin_host_ns,end_host_ns\n";
    std::uint64_t dag_runs=0;
    resource_event("after_model_init");
    std::vector<Entry> entries;
    if(!get("--input-list").empty()){std::istringstream listing(read(get("--input-list")));std::string line;
        while(std::getline(listing,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty()||line[0]=='#')continue;std::istringstream f(line);Entry e;std::string id,w,h,s;
            if(!std::getline(f,id,'\t')||!std::getline(f,w,'\t')||!std::getline(f,h,'\t')||!std::getline(f,s,'\t')||!std::getline(f,e.format,'\t')||!std::getline(f,e.path))throw std::invalid_argument("TSV needs id,width,height,stride,format,path");
            e.id=std::stoull(id);e.width=std::stoi(w);e.height=std::stoi(h);e.stride=std::stoi(s);entries.push_back(e);}}
    else if(!get("--input").empty())entries.push_back({number("--width",640),number("--height",544),number("--stride",number("--width",640)),get("--format","nv12"),get("--input"),1});
    std::vector<example::Image> fixed;
    for(const auto& e:entries){if(e.format!="nv12"&&e.format!="i420")throw std::invalid_argument("Only original raw NV12/I420 is accepted");example::ImageInputOptions io;io.format=e.format;io.width=e.width;io.height=e.height;io.stride=e.stride;fixed.push_back(example::loadImage(e.path,io));}
    if(cpu_backend&&fixed.empty())throw std::invalid_argument("CPU input list contains no usable original frames");
    std::shared_ptr<demo_camera::Source> camera;demo_camera::Options co;
    co.sdk_path=get("--sdk","/app/data/metoak/lib/libMoGeneralSDK.so");co.config_path=get("--camera-config","/app/data/metoak/demo/config/x5_camera.cfg");
    co.stream=get("--stream","rgbd");co.format=get("--format","nv12");co.disparity_mode=number("--disparity-mode",0);co.demand_enabled=disparity_every>0;co.stereo_trace=disparity_every>0||stereo_trace;
    co.retry_interval_ms=number("--camera-retry-ms",5);
    const int copy_disparity=number("--copy-disparity",0);
    if(copy_disparity!=0&&copy_disparity!=1)throw std::invalid_argument("--copy-disparity must be 0 or 1");
    co.copy_disparity=copy_disparity==1;
    if(co.copy_disparity && (!fixed.empty()||co.stream!="rgbd"))throw std::invalid_argument("Depth visualization requires live original RGBD input");
    co.audit_rgbd_triplet=workload=="sdk-disparity"||rgbd_mode1||co.copy_disparity;
    co.allow_rgbd_mode1_probe=rgbd_mode1;
    co.restore_mode_before_close=close_policy=="restore";
    co.lifecycle=[&](const char* name,int code,double ms){
        lifecycle<<"{\"camera_cycle\":"<<camera_cycle<<",\"name\":"<<quote(name)<<",\"code\":"<<code<<",\"elapsed_ms\":"<<elapsed(event_start,Clock::now())<<",\"call_ms\":"<<ms<<"}\n";lifecycle.flush();
        const std::string call(name);
        if(!close_error && code!=0 && (call=="moCloseCamera"||call=="dlclose"||call=="moSetDisparityCalculationMode_restore"||call=="callback_unregister"||call=="callback_replace_noop")) {
            close_error=code;close_error_call=call;close_error_cycle=camera_cycle;
        }
    };
    const auto open_camera=[&]{
        failure_code=1001;error_stage="camera.open";
        camera=demo_camera::open(co);const auto d=camera->diagnostics(false);effective_mode=d.effective_mode;
        mode_events<<"{\"camera_cycle\":"<<camera_cycle<<",\"original_mode\":"<<d.original_mode<<",\"requested_mode\":"<<co.disparity_mode<<",\"effective_mode\":"<<effective_mode<<",\"close_policy\":"<<quote(close_policy)<<"}\n";mode_events.flush();
        if(!mode_events)throw std::runtime_error("Camera mode metadata write failed");
    };
    if(workload=="det-seg-only"&&fixed.empty())throw std::invalid_argument("det-seg-only requires an original frozen RGBD input list");
    if(rgbd_mode1&&!fixed.empty())throw std::invalid_argument("RGBD mode1 requires a live frame/cache identity, never frozen-ID demand requests");
    if(background_camera&&fixed.empty())throw std::invalid_argument("Background SDK load requires frozen AI input");
    if(workload=="sdk-disparity"&&!fixed.empty()&&!background_camera)throw std::invalid_argument("Frozen SDK-disparity benchmark must open its background camera");
    if(fixed.empty()||background_camera)open_camera();
    const bool camera_opened=bool(camera);
    int mode1_initial_drained_frames=0;
    if(rgbd_mode1){
        // The inspected SDK has two-entry input/output queues. Drain mode2
        // leftovers before issuing a current-frame cache request. This is
        // startup synchronization, not proof that demand mode is supported.
        // The following real request/probe must still succeed; no measured
        // acquisition or cache miss is silently skipped.
        std::ofstream startup(out/"mode1_startup.jsonl",std::ios::binary);
        int consecutive=0;
        for(int i=0;i<30&&consecutive<5;++i){
            auto frame=camera->next();r26::validateInput(frame.image->view());
            const bool empty_depth=!frame.sdk_disparity_frame_id&&!frame.sdk_disparity_timestamp_ns&&
                !frame.sdk_disparity_width&&!frame.sdk_disparity_height&&!frame.sdk_disparity_bytes_used;
            consecutive=frame.sdk_color_pair_valid&&empty_depth?consecutive+1:0;
            ++mode1_initial_drained_frames;
            startup<<"{\"discarded_index\":"<<i+1<<",\"frame_id\":"<<frame.info.id<<",\"sdk_timestamp_ns\":"<<frame.info.timestamp_ns
                <<",\"color_pair_valid\":"<<(frame.sdk_color_pair_valid?"true":"false")<<",\"depth_metadata_empty\":"<<(empty_depth?"true":"false")
                <<",\"consecutive\":"<<consecutive<<"}\n";
            if(!startup)throw std::runtime_error("Mode1 startup evidence write failed");
        }
        startup.close();if(!startup)throw std::runtime_error("Mode1 startup evidence close failed");
        if(consecutive<5)throw std::runtime_error("UNSUPPORTED_RGBD_MODE1: startup color-only delivery not established within 30 frames");
    }
    std::unique_ptr<BackgroundDrain> background_drain;
    if(background_camera)background_drain=std::make_unique<BackgroundDrain>(camera,out/"background_frames.csv");
    if(mode=="camera-only"&&!camera)throw std::invalid_argument("camera-only requires live input");
    const auto close_camera=[&](const char* phase){
        failure_code=r28::ResourceCleanupFailed;error_stage="camera.close";
        camera.reset();resource_event(phase);
        if(!lifecycle)throw std::runtime_error("Lifecycle metadata write failed");
        if(close_error) {
            text(out/"lifecycle_failure.json","{\"camera_cycle\":"+std::to_string(close_error_cycle)+",\"name\":"+quote(close_error_call)+",\"code\":"+std::to_string(close_error)+"}\n");
            throw r28::Failure({r28::ResourceCleanupFailed,close_error,close_error_call,{},"SDK lifecycle failed at camera cycle "+std::to_string(close_error_cycle)+"; see camera_lifecycle.jsonl",failure_frame});
        }
    };
    if(reopen_every>0&&(!camera||(mode!="optimized"&&mode!="camera-only")))throw std::invalid_argument("Reopen requires live optimized mode");
    resource_event(camera?"after_camera_open":"after_fixed_inputs_load");
    std::ofstream csv(out/"frames.csv",std::ios::binary),listing(out/"source.tsv",std::ios::binary);
    csv<<"sample_id,frame_id,width,height,stride,acquire_ms,pack_ms,wall_ms,cpu_ms,det_pre_ms,det_bpu_ms,det_post_ms,seg_pre_ms,seg_bpu_ms,seg_post_ms,rss_kib,cma_free_kib,output_ms,det_input_different_bytes,seg_input_different_bytes,mask_different_pixels,detections_equal,math_mode,mask_fnv,seg_math_mode,sdk_timestamp_ns,received_host_ns,sdk_get_ms,frame_copy_ms,frame_release_ms,post_filled_pixels,post_boundary_pixels,disparity_ms,disparity_code,det_gate_ms,det_submit_ms,det_wait_ms,det_release_ms,seg_gate_ms,seg_submit_ms,seg_wait_ms,seg_release_ms,schedule,preprocess,det_resize_ms,det_transfer_ms,det_padding_bytes,det_copied_bytes,det_table_blocks,det_gather_blocks,det_init,seg_resize_ms,seg_transfer_ms,seg_padding_bytes,seg_copied_bytes,seg_table_blocks,seg_gather_blocks,seg_init,det_math_y,det_math_u,det_math_v,seg_math_y,seg_math_u,seg_math_v,det_pack_ms,det_calibration_ms,det_map_ms,det_padding_ms,det_y_ms,det_uv_ms,det_tail_ms,det_account_ms,det_copy_ms,det_clean_ms,det_audit_ms,det_block_bytes,det_tail_bytes,seg_pack_ms,seg_calibration_ms,seg_map_ms,seg_padding_ms,seg_y_ms,seg_uv_ms,seg_tail_ms,seg_account_ms,seg_copy_ms,seg_clean_ms,seg_audit_ms,seg_block_bytes,seg_tail_bytes,preprocess_profile,camera_cycle,cma_total_kib,mem_available_kib,rss_hwm_kib,session_elapsed_ms,dag_run_id,dag_graph_ms,dag_depth_feature_ms,dag_task_count,dag_feature_count,sdk_timestamp_valid,sdk_attempts,sdk_last_error,sdk_retry_sleep_ms,sdk_rgbd_triplet_available,sdk_rgbd_triplet_valid,sdk_right_frame_id,sdk_right_timestamp_ns,sdk_right_width,sdk_right_height,sdk_right_bytes_used,sdk_disparity_frame_id,sdk_disparity_timestamp_ns,sdk_disparity_width,sdk_disparity_height,sdk_disparity_bytes_used,sdk_rgbd_triplet_issue,sdk_color_pair_valid,sdk_color_pair_issue,dag_visual_render_ms,dag_visual_encode_write_ms,backend,det_infer_ms,seg_infer_ms,det_cpu_infer_ms,seg_cpu_infer_ms,det_ort_run_ms,seg_ort_run_ms\n";
    std::function<r25::DepthResult(std::uint64_t)> pending_depth;
    const auto one=[&](const example::Image& image,bool optimized,bool capture,bool reference=false,perception::FrameInfo frame=perception::FrameInfo{},const demo_camera::Frame* source=nullptr){
        r26::validateInput(image.view());
        r25::RunOptions options;options.optimized=optimized;options.capture=capture;
        options.variant=reference?r22::Variant::R21:input_variant;options.preprocess_only=mode=="preprocess";options.input_only=input_only;
        options.depth_parallel=workload=="rgbd-three-parallel";
        if(!reference)options.depth=pending_depth;
        if(source&&source->disparity_pixels){
            const auto& d=*source->disparity_pixels;
            options.disparity_view=perception::fusion::DisparityView{d.data(),d.size()*sizeof(std::uint16_t),std::size_t(source->sdk_disparity_width)*sizeof(std::uint16_t),source->sdk_disparity_width,source->sdk_disparity_height,perception::fusion::DisparityType::UInt16,256.f};
        }
        const auto graph_begin_ns=hostNs();auto r=pipeline->run(image.view(),frame,options);const auto graph_end_ns=hostNs();r.dag_run_id=++dag_runs;
        std::size_t features=0;for(const auto& f:r.features)features+=f.succeeded;
        graph_csv<<std::setprecision(12)<<dag_runs<<','<<frame.id<<','<<frame.timestamp_ns<<','<<optimized<<','<<reference<<','<<r.graph_ms<<','<<r.wall_ms<<','<<r.pack_ms<<','<<(r.features[7].end_ms-r.features[7].start_ms)<<','<<r.frame.schedule.tasks.size()<<','<<features<<','<<graph_begin_ns<<','<<graph_end_ns<<'\n';
        if(capture||r22::profiling()||workload!="none"){
            trace<<"{\"run_id\":"<<dag_runs<<",\"frame_id\":"<<frame.id<<",\"timestamp_ns\":"<<frame.timestamp_ns<<",\"graph_ms\":"<<r.graph_ms<<",\"begin_host_ns\":"<<graph_begin_ns<<",\"end_host_ns\":"<<graph_end_ns<<",\"features\":[";bool first=true;
            for(const auto& f:r.features)if(f.succeeded){if(!first)trace<<',';first=false;trace<<"{\"feature\":"<<quote(f.feature)<<",\"task\":"<<quote(f.task)<<",\"thread\":"<<f.thread<<",\"start_ms\":"<<f.start_ms<<",\"end_ms\":"<<f.end_ms<<'}';}
            trace<<"],\"tasks\":[";first=true;for(const auto& t:r.frame.schedule.tasks){if(!first)trace<<',';first=false;trace<<"{\"id\":"<<quote(t.id)<<",\"state\":"<<quote(perception::runtime::taskStateName(t.state))<<",\"elapsed_ms\":"<<t.elapsed_ms<<'}';}trace<<"]}\n";
        }
        if(!graph_csv||!trace)throw std::runtime_error("DAG evidence write failed");return r;
    };
    demo_vis::Style style;style.mask_alpha=.4f;std::istringstream names(cfg.detect.class_names);std::string name;while(std::getline(names,name,','))style.detection_names.push_back(name);
    std::istringstream snames(cfg.seg.class_names);while(std::getline(snames,name,','))style.segmentation_names.push_back(name);
    const bool verify=mode=="verify"||mode=="preprocess";std::size_t det_diff_total=0,seg_diff_total=0,mask_diff_total=0;bool all_boxes_equal=true;
    double total_acquire=0,total_wall=0,total_pack=0,total_cpu=0,total_det_pre=0,total_det_bpu=0,total_det_post=0,total_seg_pre=0,total_seg_bpu=0,total_seg_post=0,total_output=0,total_rss=0;double total_det_ort=0,total_seg_ort=0;long peak_rss=0,min_cma=-1;
    Clock::time_point session;double session_cpu=0;std::int64_t session_begin_ns=0;
    int measured_count=0;
    for(int n=-warmup;n<count;++n){
        if(background_drain)background_drain->check();
        pending_depth={}; // Reopen warmup must not inherit the previous frame's request.
        if(n>0&&duration_seconds>0&&elapsed(session,Clock::now())>=duration_seconds*1000)break;
        if(n==0){resource_event("after_warmup");session=Clock::now();session_begin_ns=hostNs();session_cpu=cpuMs();}
        if(camera&&n>0&&reopen_every>0&&n%reopen_every==0){
            resource_event("before_camera_reopen_close");close_camera("after_camera_reopen_close");
            std::this_thread::sleep_for(std::chrono::milliseconds(reopen_settle_ms));resource_event("after_camera_reopen_settle");
            ++camera_cycle;open_camera();resource_event("after_camera_reopen_open");
            for(int w=0;w<reopen_warmup;++w){auto discarded=camera->next();r26::validateInput(discarded.image->view());if(mode!="camera-only")one(*discarded.image,true,false,false,discarded.info,&discarded);}
            resource_event("after_camera_reopen_warmup");
        }
        failure_code=1001;error_stage="camera.acquire";
        const auto acq=Clock::now();demo_camera::Frame frame;
        if(camera&&!background_camera)frame=camera->next();else{const auto i=std::size_t(n+warmup)%fixed.size();frame.image=std::shared_ptr<const example::Image>(&fixed[i],[](const example::Image*){});frame.info.id=entries[i].id;}
        const double acquire=elapsed(acq,Clock::now());const auto& image=*frame.image;const auto cpu_start=cpuMs();Output result,baseline;
        failure_frame=frame.info;failure_code=1011;error_stage="dag.run";
        pending_depth={};
        if(camera&&disparity_every>0&&((n>=0&&(n+1)%disparity_every==0)||(n<0&&workload=="rgbd-three-parallel")))pending_depth=[&](std::uint64_t id){
            const auto begin_ns=hostNs();auto d=camera->calculateDisparity(id);const auto end_ns=hostNs();
            r25::DepthResult result{true,d.frame_id,d.code,d.width,d.height,d.pixels.size(),d.call_ms,d.copy_ms};result.begin_host_ns=begin_ns;result.end_host_ns=end_ns;
            if(n>=0&&save>0&&(n+1)%save==0&&d.code==0){
                std::uint64_t h=14695981039346656037ULL;for(auto pixel:d.pixels){h=(h^(pixel&255U))*1099511628211ULL;h=(h^(pixel>>8))*1099511628211ULL;}result.output_fnv=h;
                const auto item=out/"items"/std::to_string(n+1);fs::create_directories(item);bytes(item/"disparity.u16",d.pixels);
                std::ostringstream meta;meta<<"{\"requested_public_frame_id\":"<<id<<",\"source_sdk_timestamp_ns\":"<<frame.info.timestamp_ns<<",\"width\":"<<d.width<<",\"height\":"<<d.height<<",\"pixels\":"<<d.pixels.size()<<",\"type\":\"uint16\",\"byte_order\":\"little-endian\",\"output_fnv\":"<<h<<",\"returned_frame_id_available\":false,\"pairing_basis\":\"SDK cache lookup by requested public ID; guarded identity mapping for full SDK; not an output-carried ID\"}\n";text(item/"disparity_metadata.json",meta.str());
            }
            return result;};
        if(verify){baseline=one(image,verify_reference!="original",true,true,frame.info,&frame);result=one(image,true,true,false,frame.info,&frame);}
        else result=one(image,mode=="optimized",false,false,frame.info,&frame);
        double det_ort_ms=0,seg_ort_ms=0;
#ifdef R29_WITH_ORT
        if(cpu_backend&&native_stages){const auto cpu=std::static_pointer_cast<r29::CpuStages>(native_stages);det_ort_ms=cpu->lastRunMs(0);seg_ort_ms=cpu->lastRunMs(1);}
#endif
        const double frame_cpu=cpuMs()-cpu_start;if(n<0)continue;++measured_count;total_acquire+=acquire;
        std::size_t dd=0,sd=0,md=0;bool eq=true;
        if(verify){dd=different(baseline.audit.at("detection").prepared,result.audit.at("detection").prepared)+different(baseline.audit.at("detection").storage,result.audit.at("detection").storage);
            sd=different(baseline.audit.at("segmentation").prepared,result.audit.at("segmentation").prepared)+different(baseline.audit.at("segmentation").storage,result.audit.at("segmentation").storage);
            md=different(baseline.mask(),result.mask());eq=sameBoxes(baseline.det,result.det);det_diff_total+=dd;seg_diff_total+=sd;mask_diff_total+=md;all_boxes_equal=all_boxes_equal&&eq;}
        double disparity_ms=0;int disparity_code=-999;
        if(result.depth.requested){const auto& disp=result.depth;disparity_ms=disp.call_ms+disp.copy_ms;disparity_code=disp.code;
            depth_events<<"{\"sample_id\":"<<n+1<<",\"requested_public_frame_id\":"<<frame.info.id<<",\"sdk_timestamp_ns\":"<<frame.info.timestamp_ns<<",\"camera_cycle\":"<<camera_cycle<<",\"code\":"<<disp.code<<",\"width\":"<<disp.width<<",\"height\":"<<disp.height<<",\"pixels\":"<<disp.pixels<<",\"call_ms\":"<<disp.call_ms<<",\"copy_ms\":"<<disp.copy_ms<<",\"begin_host_ns\":"<<disp.begin_host_ns<<",\"end_host_ns\":"<<disp.end_host_ns<<",\"output_fnv\":"<<disp.output_fnv<<"}\n";depth_events.flush();if(!depth_events)throw std::runtime_error("Disparity metadata write failed");}
        const auto output_start=Clock::now();
        failure_code=r28::OutputFailed;error_stage="output.save";
        if(mode=="capture"||(save>0&&(n+1)%save==0)||(visual>0&&(n+1)%visual==0)||(verify&&(dd||sd||md||!eq))){const auto item=out/"items"/std::to_string(n+1);fs::create_directories(item);
            bytes(item/"source.yuv",image.bytes);text(item/"frame.json","{\"frame_id\":"+std::to_string(frame.info.id)+",\"width\":"+std::to_string(image.width)+",\"height\":"+std::to_string(image.height)+",\"stride\":"+std::to_string(image.stride)+",\"format\":\"nv12\",\"format_authority\":\"original start_config dnn.left_yuv_format; no automatic vendor format inference\",\"sdk_stream_type\":"+std::to_string(frame.sdk_stream_type)+",\"sdk_bits_per_pixel\":"+std::to_string(frame.sdk_bits_per_pixel)+",\"sdk_rgb_image_type\":"+std::to_string(frame.sdk_rgb_image_type)+",\"sdk_bytes_used\":"+std::to_string(frame.sdk_bytes_used)+"}\n");
            if(frame.disparity_pixels){
                bytes(item/"disparity.u16",*frame.disparity_pixels);
                std::ostringstream meta;meta<<"{\"frame_id\":"<<frame.info.id<<",\"timestamp_ns\":"<<frame.info.timestamp_ns
                    <<",\"width\":"<<frame.sdk_disparity_width<<",\"height\":"<<frame.sdk_disparity_height
                    <<",\"stride_bytes\":"<<frame.sdk_disparity_width*sizeof(std::uint16_t)
                    <<",\"type\":\"uint16\",\"scale\":256,\"byte_order\":\"little-endian\",\"pairing_basis\":\"validated SDK RGBD triplet\"}\n";
                text(item/"disparity_metadata.json",meta.str());
            }
            listing<<frame.info.id<<'\t'<<image.width<<'\t'<<image.height<<'\t'<<image.stride<<'\t'<<(image.format==perception::PixelFormat::NV12?"nv12":"i420")<<'\t'<<fs::absolute(item/"source.yuv").string()<<'\n';
            if(mode!="capture"){bytes(item/"mask.bin",result.mask());text(item/"detections.json",boxes(result.det)+'\n');
                text(item/"mask_geometry.json","{\"width\":"+std::to_string(cfg.seg.mask_width>0?cfg.seg.mask_width:image.width)+",\"height\":"+std::to_string(cfg.seg.mask_height>0?cfg.seg.mask_height:image.height)+",\"route\":"+quote(route)+"}\n");
                const auto save_audit=[&](const auto& audits,const std::string& prefix){for(const auto& v:audits){const auto tag=prefix+v.first;bytes(item/(tag+"_prepared.nv12"),v.second.prepared);bytes(item/(tag+(cpu_backend?"_cpu_input.f32":"_bpu_input_storage.bin")),v.second.storage);if(!v.second.output_storage.empty())bytes(item/(tag+(cpu_backend?"_cpu_output.f32":"_bpu_output_storage.bin")),v.second.output_storage);if(!v.second.decoded_logits.empty())bytes(item/(tag+"_logits.f32"),v.second.decoded_logits);if(!v.second.output_metadata.empty())text(item/(tag+"_output_metadata.json"),v.second.output_metadata+'\n');}};
                save_audit(result.audit,"");
                if(verify){bytes(item/"baseline_mask.bin",baseline.mask());text(item/"baseline_detections.json",boxes(baseline.det)+'\n');save_audit(baseline.audit,"baseline_");
                    std::ostringstream details;details<<"{\"frame_id\":"<<frame.info.id<<",\"mask_different_pixels\":"<<md<<",\"detections_bit_equal\":"<<(eq?"true":"false")<<",\"input\":{";
                    bool first=true;for(const auto& pair:result.audit){if(!first)details<<',';first=false;const auto& a=baseline.audit.at(pair.first);const auto& b=pair.second;
                        details<<quote(pair.first)<<":{\"prepared\":"<<byteDifference(a.prepared,b.prepared,b.width,b.height)<<",\"storage\":"<<byteDifference(a.storage,b.storage,b.width,b.height,cpu_backend)<<'}';}
                    details<<"}}\n";text(item/"differences.json",details.str());
                }
                if(visual>0&&(n+1)%visual==0&&inference){perception::DetectionResult d;d.width=image.width;d.height=image.height;for(const auto& b:result.det)d.detections.push_back({b.class_id,b.confidence,b.x1,b.y1,b.x2,b.y2});
                    perception::SegmentationResult s;s.width=cfg.seg.mask_width>0?cfg.seg.mask_width:image.width;s.height=cfg.seg.mask_height>0?cfg.seg.mask_height:image.height;s.mask=result.mask();demo_vis::savePng(item/"visualization.png",image,&d,&s,style);}
            }
        }
        const auto mask_hash=verify?hash(result.mask()):0;
        const double output_ms=result.output_ms+elapsed(output_start,Clock::now());const auto res=resources();const auto value=[&](const char* key){const auto i=res.find(key);return i==res.end()?-1L:i->second;};
        const long rss=value("VmRSS:"),cma=value("CmaFree:");if(rss>=0){total_rss+=rss;peak_rss=std::max(peak_rss,rss);}if(cma>=0)min_cma=min_cma<0?cma:std::min(min_cma,cma);
        const auto& t=result.timing;const double post=std::max(0.,t.detect_ms-t.detect_pre_ms-result.det_infer_ms);
        total_wall+=result.wall_ms;total_pack+=result.pack_ms;total_cpu+=frame_cpu;total_det_pre+=t.detect_pre_ms;total_det_bpu+=result.det_infer_ms;total_det_post+=post;
        total_det_ort+=det_ort_ms;total_seg_ort+=seg_ort_ms;total_seg_pre+=t.seg_pre_ms;total_seg_bpu+=result.seg_infer_ms;total_seg_post+=t.seg_post_ms;total_output+=output_ms;
        csv<<std::setprecision(12)<<n+1<<','<<frame.info.id<<','<<image.width<<','<<image.height<<','<<image.stride<<','<<acquire<<','<<result.pack_ms<<','<<result.wall_ms<<','<<frame_cpu<<','<<t.detect_pre_ms<<','<<(cpu_backend?0.:t.detect_bpu_ms)<<','<<post<<','<<t.seg_pre_ms<<','<<(cpu_backend?0.:t.seg_bpu_ms)<<','<<t.seg_post_ms<<','<<rss<<','<<cma<<','<<output_ms<<','<<dd<<','<<sd<<','<<md<<','<<eq<<','<<(result.inputs[0].math_mode>=0?result.inputs[0].math_mode:inference?r20::detectorWorkspace().mathMode():-1)<<','<<mask_hash<<','<<(result.inputs[1].math_mode>=0?result.inputs[1].math_mode:inference?r20::segmentationWorkspace().mathMode():-1)<<','<<frame.info.timestamp_ns<<','<<frame.received_host_ns<<','<<frame.acquisition.sdk_get_ms<<','<<frame.acquisition.copy_ms<<','<<frame.acquisition.release_ms<<','<<(mode=="optimized"||verify?r20::segPostWorkspace().stats.filled_pixels:0)<<','<<(mode=="optimized"||verify?r20::segPostWorkspace().stats.interpolated_pixels:0)<<','<<disparity_ms<<','<<disparity_code<<','<<t.detect_api.gate_ms<<','<<t.detect_api.submit_ms<<','<<t.detect_api.wait_ms<<','<<t.detect_api.release_ms<<','<<t.seg_api.gate_ms<<','<<t.seg_api.submit_ms<<','<<t.seg_api.wait_ms<<','<<t.seg_api.release_ms<<','<<schedule<<','<<r22::variantName(input_variant);
        for(const auto& info:result.inputs)csv<<','<<info.prepare_ms<<','<<info.transfer_ms<<','<<info.padding_bytes<<','<<info.copied_bytes<<','<<info.table_blocks<<','<<info.gather_blocks<<','<<info.initialized;
        for(const auto& info:result.inputs)for(int m:info.math_modes)csv<<','<<m;
        for(const auto& info:result.inputs)csv<<','<<info.pack_ms<<','<<info.calibration_ms<<','<<info.map_ms<<','<<info.padding_ms<<','<<info.y_ms<<','<<info.uv_ms<<','<<info.tail_ms<<','<<info.account_ms<<','<<info.copy_ms<<','<<info.clean_ms<<','<<info.audit_ms<<','<<info.block_bytes<<','<<info.tail_bytes;
        csv<<','<<r22::profiling()<<','<<camera_cycle<<','<<value("CmaTotal:")<<','<<value("MemAvailable:")<<','<<value("VmHWM:")<<','<<elapsed(session,Clock::now())<<','<<result.dag_run_id<<','<<result.graph_ms<<','<<(result.features[7].end_ms-result.features[7].start_ms)<<','<<result.frame.schedule.tasks.size()<<','<<(input_only?2:8)<<','<<frame.acquisition.sdk_timestamp<<','<<frame.acquisition.attempts<<','<<frame.acquisition.last_error<<','<<frame.acquisition.retry_sleep_ms<<','<<frame.sdk_rgbd_triplet_available<<','<<frame.sdk_rgbd_triplet_valid<<','<<frame.sdk_right_frame_id<<','<<frame.sdk_right_timestamp_ns<<','<<frame.sdk_right_width<<','<<frame.sdk_right_height<<','<<frame.sdk_right_bytes_used<<','<<frame.sdk_disparity_frame_id<<','<<frame.sdk_disparity_timestamp_ns<<','<<frame.sdk_disparity_width<<','<<frame.sdk_disparity_height<<','<<frame.sdk_disparity_bytes_used<<','<<frame.sdk_rgbd_triplet_issue<<','<<frame.sdk_color_pair_valid<<','<<frame.sdk_color_pair_issue<<','<<result.render_ms<<','<<result.encode_write_ms<<','<<backend<<','<<result.det_infer_ms<<','<<result.seg_infer_ms<<','<<(cpu_backend?result.det_infer_ms:0.)<<','<<(cpu_backend?result.seg_infer_ms:0.)<<','<<det_ort_ms<<','<<seg_ort_ms<<'\n';
        if(!csv)throw std::runtime_error("CSV write failed");
        if(disparity_code!=-999&&disparity_code!=0)throw std::runtime_error("Demand disparity failed, code "+std::to_string(disparity_code));

        if(verify&&(dd||!eq||(!allow_fused&&(sd||md))))throw r28::Failure({1012,0,"exact.acceptance",{},"EXACT_ACCEPTANCE_FAILED frame="+std::to_string(frame.info.id)+" det_input="+std::to_string(dd)+" seg_input="+std::to_string(sd)+" mask_pixels="+std::to_string(md)+" boxes_equal="+std::to_string(eq)+"; byte offsets/values in items/"+std::to_string(n+1)+"/differences.json",frame.info});
    }
    const auto session_end_ns=hostNs();
    const double session_ms=elapsed(session,Clock::now()),session_cpu_ms=cpuMs()-session_cpu;
    std::uint64_t background_frames=0,background_metadata_verified=0,background_metadata_unverified=0;
    if(background_drain){background_drain->stop();background_drain->check();background_frames=background_drain->count();
        background_metadata_verified=background_drain->metadataVerifiedCount();background_metadata_unverified=background_drain->metadataUnverifiedCount();background_drain.reset();}
    demo_camera::Diagnostics final_diagnostics;
    if(camera){
        final_diagnostics=camera->diagnostics(true);
        effective_mode=final_diagnostics.effective_mode;
        const auto& d=final_diagnostics;
        std::ostringstream diagnostic;
        diagnostic<<std::setprecision(14)<<"{\"requested_mode\":"<<d.requested_mode<<",\"original_mode\":"<<d.original_mode<<",\"effective_mode\":"<<d.effective_mode<<",\"mode_available\":"<<(d.mode_available?"true":"false")
            <<",\"callback_available\":"<<(d.callback_available?"true":"false")<<",\"stereo_starts\":"<<d.stereo_starts<<",\"stereo_ends\":"<<d.stereo_ends<<",\"stereo_dropped_events\":"<<d.stereo_dropped_events<<",\"stereo_busy_ms\":"<<d.stereo_busy_ms
            <<",\"bpu_ratio_available\":"<<(d.ratio_available?"true":"false")<<",\"bpu_ratio\":"<<d.bpu_ratio<<",\"original_fps_available\":"<<(d.original_fps_available?"true":"false")<<",\"original_fps\":"<<d.original_fps
            <<",\"disparity_fps_available\":"<<(d.disparity_fps_available?"true":"false")<<",\"disparity_fps\":"<<d.disparity_fps
            <<",\"scope\":\"SDK counters include warmup; measured callback rates must be clipped to session_begin_host_ns/session_end_host_ns\",\"same_frame_three_task_dag\":"<<(workload=="rgbd-three-parallel"?"true":"false")<<"}\n";
        text(out/"camera_diagnostics.json",diagnostic.str());
        std::ofstream stereo(out/"stereo_events.csv",std::ios::binary);
        stereo<<"host_ns,status\n";for(const auto& e:camera->stereoEvents())stereo<<e.host_ns<<','<<e.status<<'\n';
        if(!stereo)throw std::runtime_error("Stereo event evidence write failed");
    }
    {std::ifstream maps("/proc/self/maps");if(maps){std::ostringstream contents;contents<<maps.rdbuf();text(out/"loaded_maps.txt",contents.str());}}
    csv.close();listing.close();if(!csv||!listing)throw std::runtime_error("CSV/listing write failed");resource_event("before_model_close");
    {r28::Status status;if(pipeline->tryClose(status))throw r28::Failure(status);}
    pipeline.reset();native_stages.reset();
    graph_csv.close();trace.close();if(!graph_csv||!trace)throw std::runtime_error("DAG evidence close failed");
    resource_event("after_model_close");close_camera("after_camera_close");
    if(measured_count<1)throw std::runtime_error("No measured frames");
    const auto after_release=resources();const auto after_cma=after_release.find("CmaFree:");
    std::ostringstream done;done<<std::setprecision(14)<<"{\"version\":30,\"input_contract\":\"original-map-rgbd-left-640x544-nv12\",\"source_width\":640,\"source_height\":544,\"source_format\":\"nv12\",\"framework\":\"original-dag-task-feature\",\"dag_runs\":"<<dag_runs<<",\"dag_profile\":"<<quote(graph_path.string())<<",\"preprocess\":"<<quote(r22::variantName(input_variant))<<",\"completed\":true,\"mode\":"<<quote(mode)<<",\"actual_bpu\":"<<(inference&&!cpu_backend?"true":"false")
        <<",\"backend\":"<<quote(backend)<<",\"actual_cpu_onnx\":"<<(inference&&cpu_backend?"true":"false")
        <<",\"cpu_profile\":"<<quote(get("--cpu-profile"))<<",\"metrics_cpu_basis\":\"sum of process user and kernel CPU; one logical CPU = 100 percent\""
        <<",\"rss_basis\":\"resident working set in KiB; Windows WorkingSetSize, Linux VmRSS\",\"infer_time_basis\":\"DAG backend Feature duration includes inference and output validation/copy; ort_run_ms measures Session::Run only\""
        <<",\"benchmark_workload\":"<<quote(workload)<<",\"camera_opened\":"<<(camera_opened?"true":"false")<<",\"background_camera\":"<<(background_camera?"true":"false")<<",\"camera_retry_ms\":"<<co.retry_interval_ms
        <<",\"background_frames_including_warmup\":"<<background_frames<<",\"background_camera_role\":\"drain RGBD tuples; never feeds AI; CPU includes SDK acquisition/copy/release\""
        <<",\"background_metadata_verified_frames\":"<<background_metadata_verified<<",\"background_metadata_unverified_frames\":"<<background_metadata_unverified<<",\"mode1_initial_drained_frames\":"<<mode1_initial_drained_frames
        <<",\"session_begin_host_ns\":"<<session_begin_ns<<",\"session_end_host_ns\":"<<session_end_ns<<",\"stereo_callback_available\":"<<(final_diagnostics.callback_available?"true":"false")
        <<",\"sdk_mode_available\":"<<(final_diagnostics.mode_available?"true":"false")
        <<",\"stereo_starts\":"<<final_diagnostics.stereo_starts<<",\"stereo_ends\":"<<final_diagnostics.stereo_ends<<",\"stereo_dropped_events\":"<<final_diagnostics.stereo_dropped_events<<",\"stereo_busy_ms\":"<<final_diagnostics.stereo_busy_ms<<",\"same_frame_three_task_dag\":"<<(workload=="rgbd-three-parallel"?"true":"false")
        <<",\"requested_disparity_mode\":"<<co.disparity_mode<<",\"effective_disparity_mode\":"<<effective_mode<<",\"disparity_every\":"<<disparity_every<<",\"close_policy\":"<<quote(close_policy)<<",\"resource_snapshots\":"<<(snapshots?"true":"false")<<",\"lifecycle_ok\":"<<(close_error?"false":"true")<<",\"samples\":"<<measured_count<<",\"requested_frames\":"<<count<<",\"duration_seconds\":"<<duration_seconds<<",\"duration_completed\":"<<((duration_seconds<=0||session_ms>=duration_seconds*1000)?"true":"false")<<",\"camera_cycles\":"<<camera_cycle<<",\"preprocess_profile\":"<<(r22::profiling()?"true":"false")<<",\"save_every\":"<<save<<",\"visualize_every\":"<<visual<<",\"warmup\":"<<warmup<<",\"double_buffer\":false,\"public_resize\":false,\"seg_route\":"<<quote(route)<<",\"vse\":"<<(cfg.seg.fused_vse?"true":"false")<<",\"fused_replacement\":"<<(fusion?"true":"false")<<",\"mask_width\":"<<cfg.seg.mask_width<<",\"mask_height\":"<<cfg.seg.mask_height<<",\"fused_differences_allowed\":"<<(allow_fused?"true":"false")<<",\"parallel_det_seg\":"<<(cfg.parallel_det_seg?"true":"false")
        <<",\"schedule\":"<<quote(schedule)<<",\"bpu_priority\":"<<cfg.bpu_priority<<",\"verify_reference\":"<<quote(verify_reference)<<",\"persistent_workers\":"<<"true"<<",\"input_zero_difference\":"<<(verify&&det_diff_total==0&&seg_diff_total==0?"true":"false")<<",\"same_frame_outputs_verified\":"<<(verify&&inference?"true":"false")
        <<",\"det_input_different_bytes\":"<<det_diff_total<<",\"seg_input_different_bytes\":"<<seg_diff_total<<",\"mask_different_pixels\":"<<mask_diff_total<<",\"detections_bit_equal\":"<<(verify&&inference?(all_boxes_equal?"true":"false"):"null")
        <<",\"det_model\":"<<quote(cfg.detect.model_path)<<",\"seg_model\":"<<quote(cfg.seg.model_path)<<",\"det_input_width\":"<<dw<<",\"det_input_height\":"<<dh<<",\"seg_input_width\":"<<gw<<",\"seg_input_height\":"<<gh
        <<",\"fused_model\":"<<quote(cfg.seg.fused_model_path)<<",\"performance_mask_hash\":false,\"neon\":"<<(r20::neonEnabled()?"true":"false")<<",\"fps\":"<<measured_count*1000./session_ms<<",\"cpu_pct\":"<<session_cpu_ms/session_ms*100<<",\"cpu_ms_per_frame\":"<<total_cpu/measured_count
        <<",\"acquire_ms\":"<<total_acquire/measured_count<<",\"wall_ms\":"<<total_wall/measured_count<<",\"pack_ms\":"<<total_pack/measured_count<<",\"det_pre_ms\":"<<total_det_pre/measured_count<<",\"det_bpu_ms\":"<<(cpu_backend?0.:total_det_bpu/measured_count)<<",\"det_post_ms\":"<<total_det_post/measured_count
        <<",\"seg_pre_ms\":"<<total_seg_pre/measured_count<<",\"seg_bpu_ms\":"<<(cpu_backend?0.:total_seg_bpu/measured_count)<<",\"seg_post_ms\":"<<total_seg_post/measured_count<<",\"output_ms\":"<<total_output/measured_count
        <<",\"det_infer_ms\":"<<total_det_bpu/measured_count<<",\"seg_infer_ms\":"<<total_seg_bpu/measured_count
        <<",\"det_cpu_infer_ms\":"<<(cpu_backend?total_det_bpu/measured_count:0.)<<",\"seg_cpu_infer_ms\":"<<(cpu_backend?total_seg_bpu/measured_count:0.)
        <<",\"det_ort_run_ms\":"<<total_det_ort/measured_count<<",\"seg_ort_run_ms\":"<<total_seg_ort/measured_count
        <<",\"rss_mean_mib\":"<<total_rss/measured_count/1024.<<",\"rss_peak_mib\":"<<peak_rss/1024.<<",\"cma_min_kib\":"<<min_cma<<",\"resources_after_close\":{\"cma_free_kib\":"<<(after_cma==after_release.end()?-1:after_cma->second)<<"}}\n";
    text(out/"complete.json",done.str());std::cout<<done.str();return 0;
}catch(const std::exception& e){
    const auto status=r28::statusFromException(e,failure_code,error_stage,{},failure_frame);
    const auto failure="{\"completed\":false,\"error\":"+quote(e.what())+",\"error_code\":"+std::to_string(status.code)+",\"status\":"+r28::statusJson(status)+"}\n";
    std::cerr<<failure;
    try{if(!out.empty()&&fs::is_directory(out))text(out/"failed.json",failure);}catch(const std::exception& write_error){std::cerr<<"Could not save failed.json: "<<write_error.what()<<'\n';}
    return r28::processExit(status.code);
}}
