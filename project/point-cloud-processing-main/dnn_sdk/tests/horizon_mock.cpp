// TEST ONLY. Uses the real vendor declarations; never installed in the delivery lib/.
#include <dnn/hb_dnn.h>
#include <dnn/hb_sys.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
namespace {
struct Model {bool seg;int classes;};
struct Task {Model* model;};
std::mutex mutex;
std::condition_variable cv;
int active=0,peak=0,created=0,released=0;
bool expect_pair=false;
std::atomic<int> allocations{0},models{0},fail_wait{0},fail_submit{0};
std::vector<uint8_t> input_bytes[2];
hbDNNTensorProperties props(int a,int b,int c,int d,int layout,int type,int bytes) {
    hbDNNTensorProperties p{};p.validShape.numDimensions=4;
    p.validShape.dimensionSize[0]=a;p.validShape.dimensionSize[1]=b;
    p.validShape.dimensionSize[2]=c;p.validShape.dimensionSize[3]=d;
    p.alignedShape=p.validShape;p.tensorLayout=layout;p.tensorType=type;p.quantiType=NONE;
    p.alignedByteSize=bytes;return p;
}
}
extern "C" {
void mock_reset(int pair){std::lock_guard<std::mutex> lock(mutex);active=peak=created=released=0;expect_pair=pair;}
int mock_peak(){std::lock_guard<std::mutex> lock(mutex);return peak;}
int mock_active(){std::lock_guard<std::mutex> lock(mutex);return active;}
int mock_created(){std::lock_guard<std::mutex> lock(mutex);return created;}
int mock_released(){std::lock_guard<std::mutex> lock(mutex);return released;}
int mock_allocations(){return allocations;}
int mock_models(){return models;}
void mock_fail_wait(){fail_wait=1;}
void mock_fail_submit(){fail_submit=1;}
size_t mock_input(int lane,uint8_t* output,size_t capacity){
    std::lock_guard<std::mutex> lock(mutex);const auto& bytes=input_bytes[lane];
    if(output&&capacity>=bytes.size())std::memcpy(output,bytes.data(),bytes.size());return bytes.size();
}
int32_t hbDNNInitializeFromFiles(hbPackedDNNHandle_t* handle,const char** files,int32_t count){
    if(count!=1)return -1;const std::string name(files[0]);const auto colon=name.find(':');
    const int classes=colon==std::string::npos?3:std::stoi(name.substr(colon+1));
    *handle=new Model{name.find("seg")==0,classes};++models;return 0;
}
int32_t hbDNNRelease(hbPackedDNNHandle_t handle){delete static_cast<Model*>(handle);--models;return 0;}
int32_t hbDNNGetModelNameList(const char*** names,int32_t* count,hbPackedDNNHandle_t){
    static const char* name[]={"fixture"};*names=name;*count=1;return 0;
}
int32_t hbDNNGetModelHandle(hbDNNHandle_t* handle,hbPackedDNNHandle_t packed,const char*){*handle=packed;return 0;}
int32_t hbDNNGetInputCount(int32_t* count,hbDNNHandle_t){*count=1;return 0;}
int32_t hbDNNGetOutputCount(int32_t* count,hbDNNHandle_t handle){*count=static_cast<Model*>(handle)->seg?1:6;return 0;}
int32_t hbDNNGetInputTensorProperties(hbDNNTensorProperties* p,hbDNNHandle_t,int32_t){
    *p=props(1,3,32,32,HB_DNN_LAYOUT_NCHW,HB_DNN_IMG_TYPE_NV12,32*32*3/2);return 0;
}
int32_t hbDNNGetOutputTensorProperties(hbDNNTensorProperties* p,hbDNNHandle_t handle,int32_t index){
    auto& m=*static_cast<Model*>(handle);
    if(m.seg)*p=props(1,m.classes,2,2,HB_DNN_LAYOUT_NCHW,HB_DNN_TENSOR_TYPE_F32,m.classes*4*4);
    else {const int side=4>>(index/2),channels=index%2?64:m.classes;
        *p=props(1,side,side,channels,HB_DNN_LAYOUT_NHWC,HB_DNN_TENSOR_TYPE_F32,side*side*channels*4);}
    return 0;
}
int32_t hbSysAllocCachedMem(hbSysMem* mem,uint32_t size){
    mem->virAddr=std::calloc(1,size);mem->memSize=size;mem->phyAddr=0;if(!mem->virAddr)return -1;++allocations;return 0;
}
int32_t hbSysFreeMem(hbSysMem* mem){if(mem->virAddr){std::free(mem->virAddr);mem->virAddr=nullptr;--allocations;}return 0;}
int32_t hbSysFlushMem(hbSysMem*,int32_t){return 0;}
int32_t hbDNNInfer(hbDNNTaskHandle_t* task,hbDNNTensor** output,const hbDNNTensor* input,
                   hbDNNHandle_t handle,hbDNNInferCtrlParam*){
    auto& m=*static_cast<Model*>(handle);
    {
        std::lock_guard<std::mutex> lock(mutex);
        input_bytes[m.seg?1:0].assign(static_cast<const uint8_t*>(input->sysMem[0].virAddr),
            static_cast<const uint8_t*>(input->sysMem[0].virAddr)+input->properties.alignedByteSize);
        ++active;++created;peak=std::max(peak,active);cv.notify_all();
    }
    *task=new Task{&m};
    if(m.seg){auto* values=static_cast<float*>((*output)[0].sysMem[0].virAddr);
        std::fill(values,values+m.classes*4,-10.f);std::fill(values+(m.classes-1)*4,values+m.classes*4,10.f);}
    else for(int i=0;i<6;++i){auto& tensor=(*output)[i];auto* values=static_cast<float*>(tensor.sysMem[0].virAddr);
        const int elements=tensor.properties.alignedByteSize/4;std::fill(values,values+elements,i%2?0.f:-10.f);
        if(i%2==0)for(int p=m.classes-1;p<elements;p+=m.classes)values[p]=10.f;}
    return fail_submit.exchange(0)?-5:0;
}
int32_t hbDNNWaitTaskDone(hbDNNTaskHandle_t,int32_t){
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(expect_pair&&!cv.wait_for(lock,std::chrono::seconds(5),[]{return peak>=2;}))return -9;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return fail_wait.exchange(0)?-3:0;
}
int32_t hbDNNReleaseTask(hbDNNTaskHandle_t task){
    delete static_cast<Task*>(task);std::lock_guard<std::mutex> lock(mutex);--active;++released;cv.notify_all();return 0;
}
}
