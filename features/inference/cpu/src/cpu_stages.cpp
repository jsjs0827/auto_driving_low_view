#include "cpu_stages.hpp"
#include "exact_input.hpp"
#include "seg_post.hpp"
#include "runtime_status.hpp"
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace r29 { namespace {
using Clock=std::chrono::steady_clock;
using ai_node_percep_demo::DnnDetection;
[[noreturn]] void failure(std::uint32_t code,const std::string& detail,int native=0){throw r28::Failure({code,native,"cpu.onnx",{},detail,{}});}
void laneId(int i){if(i<0||i>1)throw std::invalid_argument("CPU lane must be0 or1");}
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<char(c);else if(c<32)o<<"\\u00"<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c)<<std::dec;else o<<char(c);}o<<'"';return o.str();}
std::string shapeJson(const std::vector<std::int64_t>& shape){std::ostringstream o;o<<'[';for(std::size_t i=0;i<shape.size();++i){if(i)o<<',';o<<shape[i];}o<<']';return o.str();}
bool match(const std::vector<std::int64_t>& actual,const std::vector<std::int64_t>& required,bool dynamic){if(actual.size()!=required.size())return false;for(std::size_t i=0;i<actual.size();++i)if(actual[i]!=required[i]&&!(dynamic&&actual[i]<0))return false;return true;}
void finite(const float* p,std::size_t n){for(std::size_t i=0;i<n;++i)if(!std::isfinite(p[i]))failure(1012,"ONNX output contains NaN/Inf at element "+std::to_string(i));}
std::uint8_t clampRgb(int v){return static_cast<std::uint8_t>(std::max(0,std::min(255,v)));}
// Explicit BT.601 limited integer transform; this is not a claim about a BIN's CSC.
void normalize(const std::vector<std::uint8_t>& nv12,int w,int h,const CpuInputConfig& cfg,std::vector<float>& chw){
    const auto plane=std::size_t(w)*h;if(nv12.size()!=plane*3/2)throw std::invalid_argument("CPU normalization NV12 size mismatch");chw.resize(plane*3);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){const auto p=std::size_t(y)*w+x;const auto uv=plane+std::size_t(y/2)*w+(x&~1);const int yy=int(nv12[p])-16,u=int(nv12[uv])-128,v=int(nv12[uv+1])-128;
        // Floor division reproduces arithmetic right shift for negative values portably.
        const auto q=[](int a){return a>=0?a/256:-((-a+255)/256);};
        const std::array<std::uint8_t,3> rgb{{clampRgb(q(298*yy+409*v+128)),clampRgb(q(298*yy-100*u-208*v+128)),clampRgb(q(298*yy+516*u+128))}};
        for(int c=0;c<3;++c){const int channel=cfg.color=="RGB"?c:2-c;chw[std::size_t(c)*plane+p]=(float(rgb[channel])*cfg.scale-cfg.mean[c])/cfg.stddev[c];}
    }
}
struct Box {int cls=0;float score=0,x=0,y=0,w=0,h=0;};
float iou(const Box& a,const Box& b){const float iw=std::max(0.f,std::min(a.x+a.w,b.x+b.w)-std::max(a.x,b.x));const float ih=std::max(0.f,std::min(a.y+a.h,b.y+b.h)-std::max(a.y,b.y));const float inter=iw*ih,uni=a.w*a.h+b.w*b.h-inter;return uni>0?inter/uni:0;}
// Preserve original sorting, class-wise suppression, > threshold and top-k order.
void nms(std::vector<Box>& boxes,float threshold,int topk){std::sort(boxes.begin(),boxes.end(),[](const Box& a,const Box& b){return a.score>b.score;});std::vector<Box> kept;std::vector<bool> removed(boxes.size(),false);for(std::size_t i=0;i<boxes.size();++i){if(removed[i])continue;kept.push_back(boxes[i]);if(int(kept.size())>=topk)break;for(std::size_t j=i+1;j<boxes.size();++j)if(!removed[j]&&boxes[i].cls==boxes[j].cls&&iou(boxes[i],boxes[j])>threshold)removed[j]=true;}boxes.swap(kept);}
std::vector<std::int64_t> requiredHead(const CpuLaneConfig& c,int index){const int stride=8<<(index/2),channels=index%2?c.output.reg*4:c.classes;if(c.output.layout=="NCHW")return {1,channels,c.height/stride,c.width/stride};return {1,c.height/stride,c.width/stride,channels};}
std::vector<std::int64_t> requiredSeg(const CpuLaneConfig& c){if(c.output.layout=="NCHW")return {1,c.classes,60,104};return {1,60,104,c.classes};}
struct HeadView {const float* values;int channels,plane;bool nhwc;float at(int cell,int channel) const noexcept{return nhwc?values[std::size_t(cell)*channels+channel]:values[std::size_t(channel)*plane+cell];}};
}

struct CpuStages::Impl {
    CpuConfig cfg;
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING,"perception-cpu"};
    struct Lane {
        std::unique_ptr<Ort::Session> session;
        r22::Workspace workspace;
        std::vector<std::uint8_t> prepared;
        std::vector<float> input,logits;
        std::vector<Ort::Value> outputs;
        std::vector<std::string> output_names;
        std::vector<std::vector<std::int64_t>> declared_outputs;
        std::vector<std::int64_t> declared_input;
        r20::Geometry geometry;
        double run_ms=0;
        int phase=0,source_width=0,source_height=0;
    } lanes[2];
    explicit Impl(const CpuConfig& config):cfg(config){
        Ort::SessionOptions opts;opts.SetIntraOpNumThreads(cfg.intra_threads);opts.SetInterOpNumThreads(cfg.inter_threads);opts.SetExecutionMode(cfg.inter_threads>1?ORT_PARALLEL:ORT_SEQUENTIAL);opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        // No CUDA/TensorRT/DirectML provider is appended: these sessions use CPUExecutionProvider.
        for(int i=0;i<2;++i){auto& l=lanes[i];if(!(i?cfg.dnn.seg.enabled:cfg.dnn.detect.enabled))continue;const auto& c=i?cfg.segmentation:cfg.detection;const auto& path=i?cfg.dnn.seg.model_path:cfg.dnn.detect.model_path;
            auto extension=std::filesystem::u8path(path).extension().u8string();std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char ch){return char(std::tolower(ch));});if(extension!=".onnx")failure(1010,"CPU model must be an ONNX file: "+path);
            l.session=std::make_unique<Ort::Session>(env,std::filesystem::u8path(path).c_str(),opts);Ort::AllocatorWithDefaultOptions allocator;
            if(l.session->GetInputCount()!=1)failure(1010,"CPU supports one image tensor input; model has additional inputs: "+path);
            const std::string input_name=l.session->GetInputNameAllocated(0,allocator).get();if(input_name!=c.input.name)failure(1010,"CPU model input name differs from declared profile: "+input_name);
            const auto input_type=l.session->GetInputTypeInfo(0);const auto info=input_type.GetTensorTypeAndShapeInfo();l.declared_input=info.GetShape();if(info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||!match(l.declared_input,{1,3,c.height,c.width},true))failure(1010,"CPU input must be float32 NCHW compatible with declared1x3xHxW");
            if(i==0&&c.output.format=="raw_dfl")l.output_names=c.output.names;else l.output_names={c.output.name};
            // Explicitly select required outputs by name, allowing a model's unrelated auxiliary heads.
            for(std::size_t j=0;j<l.output_names.size();++j){bool found=false;for(std::size_t k=0;k<l.session->GetOutputCount();++k){const std::string name=l.session->GetOutputNameAllocated(k,allocator).get();if(name!=l.output_names[j])continue;found=true;const auto output_type=l.session->GetOutputTypeInfo(k);const auto out=output_type.GetTensorTypeAndShapeInfo();if(out.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)failure(1010,"CPU outputs must be float32: "+name);const auto shape=out.GetShape();
                if(i==1&&!match(shape,requiredSeg(c),true))failure(1010,"Segmentation output must be declared CHW/HWC three-dimensional logits at60x104");
                if(i==0&&c.output.format=="raw_dfl"&&!match(shape,requiredHead(c,int(j)),true))failure(1010,"raw_dfl output shape/layout differs at "+name);
                if(i==0&&c.output.format=="decoded_xyxy"&&!(shape.size()==2&&shape[1]==6||shape.size()==3&&(shape[0]==1||shape[0]<0)&&shape[2]==6))failure(1010,"decoded_xyxy requires Nx6 or1xNx6 float tensor");
                l.declared_outputs.push_back(shape);break;}
                if(!found)failure(1010,"Declared CPU output not found: "+l.output_names[j]);
            }
        }
    }
};

CpuStages::CpuStages(const CpuConfig& config){try{validateCpuConfig(config);impl_=std::make_unique<Impl>(config);}catch(const r28::Failure&){throw;}catch(const Ort::Exception& e){failure(1010,e.what(),int(e.GetOrtErrorCode()));}catch(const std::exception& e){failure(1010,e.what());}}
CpuStages::~CpuStages()=default;
void CpuStages::close(){impl_.reset();}
void CpuStages::abortFrame() noexcept {if(impl_)for(auto& l:impl_->lanes){l.phase=0;l.outputs.clear();}}
int CpuStages::width(int lane) const {laneId(lane);if(!impl_)throw std::logic_error("CPU stages are closed");return lane?impl_->cfg.segmentation.width:impl_->cfg.detection.width;}
int CpuStages::height(int lane) const {laneId(lane);if(!impl_)throw std::logic_error("CPU stages are closed");return lane?impl_->cfg.segmentation.height:impl_->cfg.detection.height;}
double CpuStages::lastRunMs(int lane) const {laneId(lane);if(!impl_)throw std::logic_error("CPU stages are closed");return impl_->lanes[lane].run_ms;}
void CpuStages::preprocess(int lane,const perception::ImageView& image){
    laneId(lane);if(!impl_)throw std::logic_error("CPU stages are closed");auto& l=impl_->lanes[lane];if(!l.session)throw std::logic_error("CPU model lane is disabled");const auto& c=lane?impl_->cfg.segmentation:impl_->cfg.detection;if(l.phase)throw std::logic_error("CPU preprocess phase violation");
    if(image.format!=perception::PixelFormat::NV12&&image.format!=perception::PixelFormat::I420)failure(1002,"CPU exact preprocess accepts original NV12/I420 input");
    l.run_ms=0;const bool nv12=image.format==perception::PixelFormat::NV12;
    if(r20::optimizedPolicy()){
        if(!l.workspace.prepare(image.data,image.width,image.height,int(image.stride),image.bytes,nv12,c.width,c.height,lane?0:1,l.geometry))failure(1002,"CPU exact YUV preprocessing failed");
        l.prepared=l.workspace.output;r22::stats(lane)=l.workspace.stats;
    }else{
        if(!r20::reference::PreprocessYuv420(image.data,image.width,image.height,int(image.stride),image.bytes,nv12,c.width,c.height,lane?0:1,&l.geometry,l.prepared,127))failure(1002,"CPU original scalar YUV preprocessing failed");
    }
    normalize(l.prepared,c.width,c.height,c.input,l.input);r20::captureTensor(lane?"segmentation":"detection",l.prepared,l.input.data(),l.input.size()*sizeof(float),c.width,c.height,l.geometry.x_shift,l.geometry.y_shift,l.geometry.valid_w,l.geometry.valid_h);
    l.source_width=image.width;l.source_height=image.height;l.phase=1;
}
void CpuStages::infer(int lane,ai_node_percep_demo::DnnInferTiming& timing){
    laneId(lane);if(!impl_)throw std::logic_error("CPU stages are closed");auto& l=impl_->lanes[lane];const auto& c=lane?impl_->cfg.segmentation:impl_->cfg.detection;if(l.phase!=1)throw std::logic_error("CPU infer phase violation");
    try{
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);const std::array<std::int64_t,4> shape{{1,3,c.height,c.width}};
        auto input=Ort::Value::CreateTensor<float>(memory,l.input.data(),l.input.size(),shape.data(),shape.size());const char* input_name=c.input.name.c_str();std::vector<const char*> output_names;for(const auto& n:l.output_names)output_names.push_back(n.c_str());
        const auto start=Clock::now();l.outputs=l.session->Run(Ort::RunOptions{nullptr},&input_name,&input,1,output_names.data(),output_names.size());l.run_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        // Legacy bpu fields intentionally remain zero. The DAG's inference_ms measures CPU execution.
        if(lane)timing.seg_api={};else timing.detect_api={};
        std::ostringstream meta;meta<<"{\"backend\":\"onnxruntime_cpu\",\"dtype\":\"float32\",\"ort_run_ms\":"<<l.run_ms<<",\"layout\":"<<quote(c.output.layout)<<",\"outputs\":[";std::vector<std::uint8_t> bytes;
        for(std::size_t j=0;j<l.outputs.size();++j){auto& out=l.outputs[j];if(!out.IsTensor())failure(1012,"ONNX output is not a tensor");const auto info=out.GetTensorTypeAndShapeInfo();const auto shape=info.GetShape();if(info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)failure(1012,"ONNX output type changed from float32");
            if(lane&&!match(shape,requiredSeg(c),false))failure(1012,"Runtime segmentation logits shape differs from original60x104 contract");
            if(!lane&&c.output.format=="raw_dfl"&&!match(shape,requiredHead(c,int(j)),false))failure(1012,"Runtime raw_dfl shape/layout differs from profile");
            if(!lane&&c.output.format=="decoded_xyxy"&&!(shape.size()==2&&shape[1]==6||shape.size()==3&&shape[0]==1&&shape[2]==6))failure(1012,"Runtime decoded_xyxy tensor must be Nx6 or1xNx6");
            const auto count=info.GetElementCount();if(count>16*1024*1024)failure(1012,"ONNX output exceeds16M float safety limit");const auto* data=out.GetTensorData<float>();finite(data,count);const auto offset=bytes.size();if(r20::capturePolicy()){const auto* p=reinterpret_cast<const std::uint8_t*>(data);bytes.insert(bytes.end(),p,p+count*sizeof(float));}
            if(j)meta<<',';meta<<"{\"name\":"<<quote(l.output_names[j])<<",\"shape\":"<<shapeJson(shape)<<",\"byte_offset\":"<<offset<<",\"bytes\":"<<count*sizeof(float)<<'}';
        }
        meta<<"]}";if(lane){const auto& out=l.outputs.front();const float* p=out.GetTensorData<float>();const auto plane=std::size_t(60)*104;l.logits.resize(plane*c.classes);if(c.output.layout=="NCHW")std::copy(p,p+l.logits.size(),l.logits.begin());else for(std::size_t x=0;x<plane;++x)for(int k=0;k<c.classes;++k)l.logits[std::size_t(k)*plane+x]=p[x*c.classes+k];}
        r20::captureOutput(lane?"segmentation":"detection",bytes.data(),bytes.size(),meta.str(),lane?&l.logits:nullptr);l.phase=2;
    }catch(const r28::Failure&){throw;}catch(const Ort::Exception& e){failure(1011,e.what(),int(e.GetOrtErrorCode()));}
}
std::vector<DnnDetection> CpuStages::detection(){
    if(!impl_)throw std::logic_error("CPU stages are closed");auto& l=impl_->lanes[0];const auto& c=impl_->cfg.detection;const auto& cfg=impl_->cfg.dnn.detect;if(l.phase!=2)throw std::logic_error("CPU detection phase violation");std::vector<Box> boxes;
    if(c.output.format=="raw_dfl"){
        const float threshold=-std::log(1.f/cfg.score_threshold-1.f);
        for(int scale=0;scale<3;++scale){const int stride=8<<scale,h=c.height/stride,w=c.width/stride;const HeadView cls_head{l.outputs[scale*2].GetTensorData<float>(),c.classes,h*w,c.output.layout=="NHWC"},box_head{l.outputs[scale*2+1].GetTensorData<float>(),c.output.reg*4,h*w,c.output.layout=="NHWC"};for(int y=0;y<h;++y)for(int x=0;x<w;++x){const int cell=y*w+x;int cls=0;float max_logit=cls_head.at(cell,0);for(int k=1;k<c.classes;++k){const float v=cls_head.at(cell,k);if(v>max_logit){max_logit=v;cls=k;}}if(max_logit<threshold)continue;
            const float score=1.f/(1.f+std::exp(-max_logit));float d[4]={};for(int edge=0;edge<4;++edge){float sum=0;for(int j=0;j<c.output.reg;++j){const float v=std::exp(box_head.at(cell,edge*c.output.reg+j));d[edge]+=v*float(j);sum+=v;}d[edge]/=sum;if(!std::isfinite(d[edge]))failure(1012,"raw_dfl exponential overflows; export logits outside original decoder range");}
            if(d[2]+d[0]<=0||d[3]+d[1]<=0)continue;const float x1=(x+.5f-d[0])*stride,y1=(y+.5f-d[1])*stride,x2=(x+.5f+d[2])*stride,y2=(y+.5f+d[3])*stride;boxes.push_back({cls,score,x1,y1,x2-x1,y2-y1});
        }}
    }else{
        const auto info=l.outputs.front().GetTensorTypeAndShapeInfo();const auto count=info.GetElementCount();const float* p=l.outputs.front().GetTensorData<float>();for(std::size_t i=0;i<count;i+=6){const float cls=p[i+5],score=p[i+4];if(cls!=std::floor(cls)||cls<0||cls>=c.classes||score<0||score>1||p[i+2]<p[i]||p[i+3]<p[i+1])failure(1012,"decoded_xyxy requires model-pixel xyxy,probabilityscore,class_id rows");if(score<cfg.score_threshold||p[i+2]==p[i]||p[i+3]==p[i+1])continue;boxes.push_back({int(cls),score,p[i],p[i+1],p[i+2]-p[i],p[i+3]-p[i+1]});}
    }
    nms(boxes,cfg.nms_threshold,cfg.nms_top_k);std::vector<DnnDetection> out;out.reserve(boxes.size());for(const auto& box:boxes){const auto& g=l.geometry;float x1=(box.x-g.x_shift)/g.x_scale,y1=(box.y-g.y_shift)/g.y_scale,x2=x1+box.w/g.x_scale,y2=y1+box.h/g.y_scale;x1=std::max(0.f,std::min(x1,float(l.source_width)));y1=std::max(0.f,std::min(y1,float(l.source_height)));x2=std::max(0.f,std::min(x2,float(l.source_width)));y2=std::max(0.f,std::min(y2,float(l.source_height)));DnnDetection d;d.class_id=box.cls;d.confidence=box.score;d.x1=x1;d.y1=y1;d.x2=x1+std::max(0.f,x2-x1);d.y2=y1+std::max(0.f,y2-y1);d.class_name=c.class_names[box.cls];out.push_back(std::move(d));}
    l.outputs.clear();l.phase=0;return out;
}
std::vector<std::uint8_t> CpuStages::segmentation(){
    if(!impl_)throw std::logic_error("CPU stages are closed");auto& l=impl_->lanes[1];const auto& c=impl_->cfg.segmentation;if(l.phase!=2)throw std::logic_error("CPU segmentation phase violation");std::vector<std::uint8_t> mask;
    if(r20::optimizedPolicy())r20::segPostWorkspace().run(l.logits,c.classes,60,104,c.height,c.width,l.source_height,l.source_width,0,l.geometry,true,mask);else r20::referenceMask(l.logits,c.classes,60,104,c.height,c.width,l.source_height,l.source_width,0,l.geometry,true,mask);
    l.outputs.clear();l.phase=0;return mask;
}
std::string CpuStages::metadataJson() const {
    if(!impl_)throw std::logic_error("CPU stages are closed");const auto& cfg=impl_->cfg;std::ostringstream o;o<<std::setprecision(std::numeric_limits<float>::max_digits10)<<"{\"backend\":\"onnxruntime_cpu\",\"runtime_version\":"<<quote(Ort::GetVersionString())<<",\"provider\":\"CPUExecutionProvider\",\"single_frame\":true,\"intra_threads_per_session\":"<<cfg.intra_threads<<",\"inter_threads_per_session\":"<<cfg.inter_threads<<",\"det_seg_concurrency\":\"existing_DAG\",\"lanes\":[";
    for(int i=0;i<2;++i){if(i)o<<',';const auto& c=i?cfg.segmentation:cfg.detection;const auto& l=impl_->lanes[i];o<<"{\"name\":"<<quote(i?"segmentation":"detection")<<",\"model\":"<<quote(i?cfg.dnn.seg.model_path:cfg.dnn.detect.model_path)<<",\"input_name\":"<<quote(c.input.name)<<",\"declared_input_shape\":"<<shapeJson(l.declared_input)<<",\"actual_input_shape\":[1,3,"<<c.height<<','<<c.width<<"],\"input_dtype\":\"float32\",\"input_layout\":\"NCHW\",\"color\":"<<quote(c.input.color)<<",\"yuv_conversion\":"<<quote(c.input.yuv)<<",\"normalization\":{\"formula\":\"(uint8_channel*scale-mean)/std\",\"scale\":"<<c.input.scale<<",\"mean\":["<<c.input.mean[0]<<','<<c.input.mean[1]<<','<<c.input.mean[2]<<"],\"std\":["<<c.input.stddev[0]<<','<<c.input.stddev[1]<<','<<c.input.stddev[2]<<"]},\"output_format\":"<<quote(c.output.format)<<",\"output_layout\":"<<quote(c.output.layout)<<",\"outputs\":[";for(std::size_t j=0;j<l.output_names.size();++j){if(j)o<<',';o<<"{\"name\":"<<quote(l.output_names[j])<<",\"declared_shape\":"<<shapeJson(l.declared_outputs[j])<<'}';}o<<"]}";}
    o<<"],\"parity_scope\":\"prepared_NV12_and_same_ONNX_CPU_baseline_vs_optimized;CPU_vs_quantized_BIN_output_parity_requires_measurement\"}";return o.str();
}
}
