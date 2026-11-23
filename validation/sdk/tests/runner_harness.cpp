// Host-only lifecycle harness, compiled against real X5 API declarations.
// This is not a BPU implementation, performance test, model or deployable worker.
#include "percep_demo/horizon_dnn_runner.hpp"
#include "legacy_preprocess.hpp"
#include "exact_input.hpp"
#include "dnn/hb_dnn.h"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
using namespace ai_node_percep_demo;
namespace {
struct Model {bool seg;};
struct Task {Model* model;hbDNNTensor* output;const hbDNNTensor* input;};
std::atomic<int> active{0},released{0},allocated{0},models{0},fault{0},first_lane{-1};
std::atomic<int> expected_priority{0};
std::mutex clean_mutex;std::set<const void*> cleaned_inputs;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
hbDNNTensorProperties props(bool input,bool seg,int index){
    hbDNNTensorProperties p{};p.validShape.numDimensions=4;p.validShape.dimensionSize[0]=1;
    p.tensorLayout=input?HB_DNN_LAYOUT_NCHW:HB_DNN_LAYOUT_NHWC;p.quantiType=NONE;
    p.tensorType=input?HB_DNN_IMG_TYPE_NV12:HB_DNN_TENSOR_TYPE_F32;
    if(input){p.validShape.dimensionSize[1]=3;p.validShape.dimensionSize[2]=64;p.validShape.dimensionSize[3]=64;p.alignedByteSize=64*64*3/2+32;}
    else {int side=seg?8:(8>>(index/2));p.validShape.dimensionSize[1]=side;p.validShape.dimensionSize[2]=side;p.validShape.dimensionSize[3]=seg?3:(index%2?64:24);p.alignedByteSize=side*side*p.validShape.dimensionSize[3]*4;}
    p.alignedShape=p.validShape;return p;
}
struct Result {std::vector<DnnDetection> det;std::vector<std::uint8_t> mask;std::map<std::string,r20::TensorAudit> audit;};
Result run(HorizonDnnRunner& runner,const std::vector<std::uint8_t>& input,bool optimized){
    r20::setPolicy(optimized,true);Result result;DnnInferTiming timing;
    require(runner.inferNv12(input.data(),80,64,80,input.size(),true,&result.det,&result.mask,nullptr,&timing),"Runner failed");result.audit=r20::takeAudit();return result;
}
void equal(const Result& a,const Result& b){
    require(a.mask==b.mask,"Mask pixels differ");require(!a.det.empty() && a.det.size()==b.det.size(),"Detections differ/empty");
    for(std::size_t i=0;i<a.det.size();++i){const auto& x=a.det[i];const auto& y=b.det[i];require(x.class_id==y.class_id && x.class_name==y.class_name,"Class differs");
        const float xx[]={x.confidence,x.x1,x.y1,x.x2,x.y2},yy[]={y.confidence,y.x1,y.y1,y.x2,y.y2};require(std::memcmp(xx,yy,sizeof xx)==0,"Detection bits differ");}
    for(const auto& [tag,x]:a.audit){const auto& y=b.audit.at(tag);require(x.prepared==y.prepared && x.storage==y.storage,"Prepared/BPU input bytes differ");}
}
}
extern "C" {
int32_t hbDNNInitializeFromFiles(hbPackedDNNHandle_t* p,const char** paths,int32_t){*p=new Model{std::strstr(paths[0],"seg")!=nullptr};++models;return 0;}
int32_t hbDNNGetModelNameList(const char*** names,int32_t* count,hbPackedDNNHandle_t){static const char* n[]={"host_lifecycle_fixture"};*names=n;*count=1;return 0;}
int32_t hbDNNGetModelHandle(hbDNNHandle_t* h,hbPackedDNNHandle_t p,const char*){*h=p;return 0;}
int32_t hbDNNGetInputCount(int32_t* n,hbDNNHandle_t){*n=1;return 0;}
int32_t hbDNNGetOutputCount(int32_t* n,hbDNNHandle_t h){*n=static_cast<Model*>(h)->seg?1:6;return 0;}
int32_t hbDNNGetInputTensorProperties(hbDNNTensorProperties* p,hbDNNHandle_t h,int32_t index){*p=props(true,static_cast<Model*>(h)->seg,index);return 0;}
int32_t hbDNNGetOutputTensorProperties(hbDNNTensorProperties* p,hbDNNHandle_t h,int32_t index){*p=props(false,static_cast<Model*>(h)->seg,index);return 0;}
int32_t hbSysAllocCachedMem(hbSysMem* p,uint32_t size){p->virAddr=std::malloc(size);if(!p->virAddr)return -1;p->memSize=size;std::memset(p->virAddr,0,size);++allocated;return 0;}
int32_t hbSysFreeMem(hbSysMem* p){std::free(p->virAddr);p->virAddr=nullptr;--allocated;return 0;}
int32_t hbSysFlushMem(hbSysMem* memory,int32_t operation){
    if(operation==HB_SYS_MEM_CACHE_CLEAN){if(fault==3)return -9;std::lock_guard<std::mutex> lock(clean_mutex);cleaned_inputs.insert(memory->virAddr);}return 0;
}
int32_t hbDNNRelease(hbPackedDNNHandle_t p){delete static_cast<Model*>(p);--models;return 0;}
int32_t hbDNNInfer(hbDNNTaskHandle_t* task,hbDNNTensor** output,const hbDNNTensor* input,hbDNNHandle_t handle,hbDNNInferCtrlParam* ctrl){
    require(ctrl->more==0,"Batching changed");require(ctrl->bpuCoreId==HB_BPU_CORE_ANY,"BPU core changed");
    require(ctrl->priority==expected_priority,"Unexpected task priority");
    {std::lock_guard<std::mutex> lock(clean_mutex);require(cleaned_inputs.erase(input->sysMem[0].virAddr)==1,"Input submitted without per-frame cache clean");}
    auto* model=static_cast<Model*>(handle);int expected=-1;first_lane.compare_exchange_strong(expected,model->seg?1:0);
    *task=new Task{model,*output,input};++active;
    if(!model->seg && fault==1)return -7;return 0;
}
int32_t hbDNNWaitTaskDone(hbDNNTaskHandle_t handle,int32_t timeout){
    require(timeout==0,"Wait contract changed");auto& t=*static_cast<Task*>(handle);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(!t.model->seg && fault==2)return -8;
    const auto* input=static_cast<const unsigned char*>(t.input->sysMem[0].virAddr);
    unsigned seed=0;for(int i=0;i<64*64*3/2;++i)seed=seed*33+input[i];
    if(t.model->seg){auto* p=static_cast<float*>(t.output[0].sysMem[0].virAddr);for(int i=0;i<8*8*3;++i)p[i]=float((i*7+seed)%17)-8.f;}
    else {for(int index=0;index<6;++index){auto& out=t.output[index];auto* p=static_cast<float*>(out.sysMem[0].virAddr);const int count=out.properties.alignedByteSize/4;
        for(int i=0;i<count;++i)p[i]=index%2?0.f:-10.f;
        if(index==0)p[24*36+int(seed%24)]=2.f;}}
    return 0;
}
int32_t hbDNNReleaseTask(hbDNNTaskHandle_t task){delete static_cast<Task*>(task);--active;++released;return 0;}
}
int main(){try{
    {
        // Real runner helper (host arithmetic only), not the extracted reference copy.
        r22::OriginalSourceScope original_i420(false,1280);
        std::vector<uint8_t> raw(1280*1088*3/2,127),expected;const auto uv=std::size_t(1280)*1088;
        for(int c=0;c<2;++c){raw[uv+175*1280+14+c]=127;raw[uv+175*1280+16+c]=126;
            raw[uv+176*1280+14+c]=124;raw[uv+176*1280+16+c]=129;}
        const auto callback=r22::runnerReferencePreprocessor();r20::Geometry old,g;
        require(callback(raw.data(),1280,1088,1280,raw.size(),true,864,480,1,&old,expected,127),"Native helper regression failed");
        require(expected[481404]==127,"Native helper regression changed original U=127");
        r20::DetectorWorkspace r21;r21.setReference(callback);r22::Workspace cache,direct;cache.setReference(callback);direct.setReference(callback);
        require(r21.prepare(raw.data(),1280,1088,1280,raw.size(),true,864,480,1,g)&&r21.output==expected,"Native calibrated r21 differs");
        require(cache.prepare(raw.data(),1280,1088,1280,raw.size(),true,864,480,1,g)&&cache.output==expected,"Native cached regression differs");
        std::vector<uint8_t> target(expected.size()+37,0xa5);
        require(direct.prepare(raw.data(),1280,1088,1280,raw.size(),true,864,480,1,g,target.data(),target.size()),"Native direct regression failed");
        require(std::memcmp(target.data(),expected.data(),expected.size())==0,"Native direct regression differs");
        // Raw format/stride context changes must invalidate both calibrated workspaces.
        {
            r22::OriginalSourceScope nv12_context(true,1296);cache.setReference(callback,r22::originalSourceContext());
            require(callback(raw.data(),1280,1088,1280,raw.size(),true,864,480,1,&old,expected,127),"Native NV12 stride callback failed");
            require(cache.prepare(raw.data(),1280,1088,1280,raw.size(),true,864,480,1,g)&&cache.output==expected,"Native context transition differs");
            require(cache.stats.initialized,"Native source context did not recalibrate");
        }
    }
    {
        DnnConfig cfg;cfg.enabled=true;cfg.detect.model_path="det.host.fixture";cfg.detect.classes_num=24;cfg.seg.model_path="seg.host.fixture";cfg.seg.class_names="a,b,c";
        HorizonDnnRunner runner;require(runner.init(cfg),"Init failed");
        for(int frame=0;frame<12;++frame){std::vector<std::uint8_t> input(80*64*3/2);unsigned state=frame+1;for(auto& pixel:input){state=state*1664525u+1013904223u;pixel=state>>24;}
            runner.setExecutionSchedule("legacy",0);auto original=run(runner,input,false);r22::setVariant(r22::Variant::R21);auto baseline=run(runner,input,true);equal(original,baseline);
            for(auto variant:{r22::Variant::R21,r22::Variant::Cached,r22::Variant::Direct,r22::Variant::CompactCached,r22::Variant::CompactDirect})for(const char* schedule:{"free","det-first","seg-first"}){
                r22::setVariant(variant);runner.setExecutionSchedule(schedule,0);first_lane=-1;auto current=run(runner,input,true);equal(baseline,current);
                if(std::string(schedule)=="det-first")require(first_lane==0,"Detection not first");if(std::string(schedule)=="seg-first")require(first_lane==1,"Segmentation not first");require(active==0,"Task survived frame");}}
        r22::setVariant(r22::Variant::Direct);
        for(int failure:{1,2,3}){fault=failure;runner.setExecutionSchedule("det-first",0);r20::setPolicy(true,false);std::vector<std::uint8_t> input(80*64*3/2);std::vector<DnnDetection> det;std::vector<std::uint8_t> mask;
            require(!runner.inferNv12(input.data(),80,64,80,input.size(),true,&det,&mask,nullptr),"Injected task failure not reported");require(active==0,"Failed task leaked");}
        fault=0;runner.setExecutionSchedule("seg-first",0);std::vector<std::uint8_t> recovered(80*64*3/2);auto normal=run(runner,recovered,true);
        expected_priority=128;runner.setExecutionSchedule("seg-first",128);equal(normal,run(runner,recovered,true));expected_priority=0;
    }
    require(active==0 && allocated==0 && models==0,"Runner/model/tensor lifecycle leaked");
    std::cout<<"Host fixture: 12 frames x r21/cached/direct/compact_cached/compact_direct (original reference) x 3 schedules: zero input bytes, aligned tails, mask pixels and nonempty detection bit differences; failed submit/wait task release and recovery passed. NOT real BPU validation.\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
