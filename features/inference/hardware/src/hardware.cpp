#include "hardware.hpp"
#include "label_reader.hpp"
#include "vse_layout.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <sstream>
#include <iomanip>
#include <mutex>

namespace {
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
void check(int code,const char* operation){if(code)throw std::runtime_error(std::string(operation)+" failed: "+std::to_string(code));}
using HardwareClock=std::chrono::steady_clock;
double milliseconds(HardwareClock::time_point begin){return std::chrono::duration<double,std::milli>(HardwareClock::now()-begin).count();}
}

#if defined(HW_AB_WITH_X5)
#include <dnn/hb_dnn.h>
#include <hbn_api.h>
#include <vse_cfg.h>

namespace {
std::mutex media_mutex;
unsigned media_users=0;
void openMedia(){std::lock_guard<std::mutex> lock(media_mutex);if(!media_users)check(hb_mem_module_open(),"hb_mem_module_open");++media_users;}
void closeMedia() noexcept {std::lock_guard<std::mutex> lock(media_mutex);if(media_users&&!--media_users)hb_mem_module_close();}
}

struct VseResize::Impl {
    int sw,sh,dw,dh,timeout;
    bool mem_open=false,node_open=false,flow_open=false,started=false,allocated=false,quarantined=false;
    hbn_vnode_handle_t node{};
    hbn_vflow_handle_t flow{};
    hbn_vnode_image_t input{};
    std::vector<std::uint8_t> output;
    int output_stride=0,output_vstride=0;
    Impl(int a,int b,int c,int d,int t):sw(a),sh(b),dw(c),dh(d),timeout(t),output(std::size_t(c)*d*3/2){
      try {
        require(sw>=64&&sh>=64&&dw>=64&&dh>=64&&sw<=4096&&sh<=4096&&dw<=4096&&dh<=4096,
                "VSE benchmark dimensions must be 64..4096");
        require(!(sw%2||sh%2||dw%2||dh%2),"VSE requires even NV12 dimensions");
        require(dw<=sw&&dh<=sh,"This adapter uses VSE downscale channel 0 only");
        openMedia();mem_open=true;
        check(hbn_vnode_open(HB_VSE,0,AUTO_ALLOC_ID,&node),"hbn_vnode_open VSE");node_open=true;
        vse_attr_t attr{};attr.fps.src=30;attr.fps.dst=30;
        check(hbn_vnode_set_attr(node,&attr),"hbn_vnode_set_attr");
        vse_ichn_attr_t in{};in.width=sw;in.height=sh;in.fmt=FRM_FMT_NV12;in.bit_width=8;
        check(hbn_vnode_set_ichn_attr(node,0,&in),"hbn_vnode_set_ichn_attr");
        vse_ochn_attr_t out{};out.chn_en=CAM_TRUE;out.roi.w=sw;out.roi.h=sh;
        require(dw%16==0,"VSE target width must be a multiple of 16");
        // Output strides are driver-owned; use returned frame.buffer.stride.
        out.target_w=dw;out.target_h=dh;
        out.fmt=FRM_FMT_NV12;out.bit_width=8;out.fps.src=30;out.fps.dst=30;
        check(hbn_vnode_set_ochn_attr(node,0,&out),"hbn_vnode_set_ochn_attr");
        const int64_t flags=HB_MEM_USAGE_CPU_READ_OFTEN|HB_MEM_USAGE_CPU_WRITE_OFTEN|
            HB_MEM_USAGE_CACHED|HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
        hbn_buf_alloc_attr_t buffers{};buffers.flags=flags;buffers.buffers_num=3;buffers.is_contig=1;
        check(hbn_vnode_set_ochn_buf_attr(node,0,&buffers),"hbn_vnode_set_ochn_buf_attr");
        // VSE derives the input line pitch from the 16-aligned configured width.
        // Using 64 alignment changed 864 into 896 and produced diagonal strips.
        const int expected_stride=vse_layout::inputStride(sw);
        check(hb_mem_alloc_graph_buf(sw,sh,MEM_PIX_FMT_NV12,flags,expected_stride,sh,&input.buffer),
              "hb_mem_alloc_graph_buf");allocated=true;
        require(input.buffer.plane_cnt==2&&input.buffer.stride>=sw&&
                input.buffer.virt_addr[0]&&input.buffer.virt_addr[1],"invalid VSE input buffer");
        require(input.buffer.size[0]>=std::size_t(input.buffer.stride)*sh&&
                input.buffer.size[1]>=std::size_t(input.buffer.stride)*sh/2,"short VSE input planes");
        require(input.buffer.stride==expected_stride&&input.buffer.vstride==sh,
                "VSE input allocation pitch differs from configured 16-aligned width/height; refusing corrupted input");
        std::memset(input.buffer.virt_addr[0],16,input.buffer.size[0]);
        std::memset(input.buffer.virt_addr[1],128,input.buffer.size[1]);
        input.info.bufferindex=0;
        check(hbn_vflow_create(&flow),"hbn_vflow_create");flow_open=true;
        check(hbn_vflow_add_vnode(flow,node),"hbn_vflow_add_vnode");
        check(hbn_vflow_start(flow),"hbn_vflow_start");started=true;
      } catch(...){cleanup();throw;}
    }
    void cleanup() noexcept {
        // Buffers remain owned until the flow has been stopped.
        if(started){if(hbn_vflow_stop(flow)!=0){quarantined=true;return;}started=false;quarantined=false;}
        if(flow_open){hbn_vflow_destroy(flow);flow_open=false;}
        if(node_open){hbn_vnode_close(node);node_open=false;}
        if(allocated){hb_mem_free_buf(input.buffer.fd[0]);allocated=false;}
        if(mem_open){closeMedia();mem_open=false;}
    }
    ~Impl(){cleanup();}
    const std::vector<std::uint8_t>& run(const std::uint8_t* raw,std::size_t bytes,std::size_t stride){
        require(!quarantined,"VSE quarantined after failed frame wait; recreate pipeline");
        require(raw&&stride>=std::size_t(sw)&&stride<=16U*1024U*1024U&&stride%2==0&&bytes>=stride*sh*3/2,"invalid NV12 source");
        for(int plane=0;plane<2;++plane){
            const int height=plane?sh/2:sh;
            const auto* source=raw+(plane?stride*sh:0);
            if(stride==std::size_t(sw))vse_layout::copyPlane(input.buffer.virt_addr[plane],input.buffer.stride,source,sw,height);
            else for(int y=0;y<height;++y)std::memcpy(input.buffer.virt_addr[plane]+std::size_t(y)*input.buffer.stride,source+std::size_t(y)*stride,sw);
            check(hb_mem_flush_buf_with_vaddr(reinterpret_cast<std::uintptr_t>(input.buffer.virt_addr[plane]),
                                              input.buffer.size[plane]),"VSE input cache flush");
        }
        ++input.info.frame_id;
        check(hbn_vnode_sendframe(node,0,&input),"hbn_vnode_sendframe");
        hbn_vnode_image_t frame{};
        const int frame_status=hbn_vnode_getframe(node,0,timeout,&frame);
        if(frame_status){quarantined=true;check(frame_status,"hbn_vnode_getframe");}
        try {
            const auto& buf=frame.buffer;
            require(buf.width==dw&&buf.height==dh&&buf.stride>=dw&&buf.plane_cnt==2&&
                    buf.virt_addr[0]&&buf.virt_addr[1],"unexpected VSE output dimensions/planes");
            output_stride=buf.stride;output_vstride=buf.vstride;
            for(int plane=0;plane<2;++plane){
                const int height=plane?dh/2:dh;
                require(buf.size[plane]>=std::size_t(buf.stride)*height,"short VSE output plane");
                check(hb_mem_invalidate_buf_with_vaddr(reinterpret_cast<std::uintptr_t>(buf.virt_addr[plane]),
                                                       buf.size[plane]),"VSE output invalidate");
                auto* target=output.data()+(plane?std::size_t(dw)*dh:0);
                for(int y=0;y<height;++y)std::memcpy(target+std::size_t(y)*dw,
                                                  buf.virt_addr[plane]+std::size_t(y)*buf.stride,dw);
            }
        } catch(...){hbn_vnode_releaseframe(node,0,&frame);throw;}
        check(hbn_vnode_releaseframe(node,0,&frame),"hbn_vnode_releaseframe");
        return output;
    }
};

namespace {
struct Axes {int c,h,w;};
Axes axes(const hbDNNTensorProperties& p){
    require(p.validShape.numDimensions==4&&p.alignedShape.numDimensions==4&&
            p.validShape.dimensionSize[0]==1,"post tensor must be rank 4, batch 1");
    require(p.tensorLayout==HB_DNN_LAYOUT_NCHW||p.tensorLayout==HB_DNN_LAYOUT_NHWC,
            "post model tensor must be NCHW or NHWC");
    for(int i=0;i<4;++i)require(p.validShape.dimensionSize[i]>0&&
        p.alignedShape.dimensionSize[i]>=p.validShape.dimensionSize[i]&&p.alignedShape.dimensionSize[i]<=8192,
        "invalid post model tensor shape");
    return p.tensorLayout==HB_DNN_LAYOUT_NHWC?Axes{3,1,2}:Axes{1,2,3};
}
struct Addressing {std::size_t c,y,x;};
Addressing addressing(const hbDNNTensorProperties& p){
    const auto a=axes(p);std::size_t strides[4]{};strides[3]=1;
    for(int i=2;i>=0;--i)strides[i]=strides[i+1]*p.alignedShape.dimensionSize[i+1];
    return {strides[a.c],strides[a.h],strides[a.w]};
}
int itemBytes(int type){
    switch(type){case HB_DNN_TENSOR_TYPE_U8:case HB_DNN_TENSOR_TYPE_S8:return 1;
      case HB_DNN_TENSOR_TYPE_U16:case HB_DNN_TENSOR_TYPE_S16:return 2;
      case HB_DNN_TENSOR_TYPE_U32:case HB_DNN_TENSOR_TYPE_S32:case HB_DNN_TENSOR_TYPE_F32:return 4;
      case HB_DNN_TENSOR_TYPE_U64:case HB_DNN_TENSOR_TYPE_S64:return 8;
      default:throw std::runtime_error("unsupported compiled post-model output type");}
}
void allocate(hbDNNTensor& t){
    auto a=axes(t.properties);(void)a;
    std::size_t count=1;for(int i=0;i<4;++i)count*=t.properties.alignedShape.dimensionSize[i];
    const auto bytes=count*itemBytes(t.properties.tensorType);
        require(bytes<=256U*1024U*1024U&&t.properties.alignedByteSize>0&&
                std::size_t(t.properties.alignedByteSize)>=bytes&&
                std::size_t(t.properties.alignedByteSize)<=256U*1024U*1024U,"invalid post tensor storage");
    check(hbSysAllocCachedMem(&t.sysMem[0],t.properties.alignedByteSize),"post hbSysAllocCachedMem");
    require(t.sysMem[0].virAddr&&t.sysMem[0].memSize>=bytes,"short post tensor allocation");
}
template<class T> void copyLabels(const void* data,Addressing stride,int h,int w,int classes,std::vector<uint8_t>& mask){
    const auto* source=static_cast<const T*>(data);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){
        const double v=static_cast<double>(source[std::size_t(y)*stride.y+std::size_t(x)*stride.x]);
        if constexpr(std::is_floating_point_v<T>)require(std::isfinite(v)&&v==std::floor(v),"noninteger compiled class index");
        require(v>=0&&v<classes,"compiled class index outside valid class range");
        mask[std::size_t(y)*w+x]=static_cast<uint8_t>(v);
    }
}
void copyLabels(const hbDNNTensor& t,Addressing s,int h,int w,int c,std::vector<uint8_t>& m){
    const void* p=t.sysMem[0].virAddr;
    switch(t.properties.tensorType){
      case HB_DNN_TENSOR_TYPE_U8:return copyLabels<uint8_t>(p,s,h,w,c,m);case HB_DNN_TENSOR_TYPE_S8:return copyLabels<int8_t>(p,s,h,w,c,m);
      case HB_DNN_TENSOR_TYPE_U16:return copyLabels<uint16_t>(p,s,h,w,c,m);case HB_DNN_TENSOR_TYPE_S16:return copyLabels<int16_t>(p,s,h,w,c,m);
      case HB_DNN_TENSOR_TYPE_U32:return copyLabels<uint32_t>(p,s,h,w,c,m);case HB_DNN_TENSOR_TYPE_S32:return copyLabels<int32_t>(p,s,h,w,c,m);
      case HB_DNN_TENSOR_TYPE_U64:return copyLabels<uint64_t>(p,s,h,w,c,m);case HB_DNN_TENSOR_TYPE_S64:return copyLabels<int64_t>(p,s,h,w,c,m);
      case HB_DNN_TENSOR_TYPE_F32:return copyLabels<float>(p,s,h,w,c,m);
      default:throw std::runtime_error("unsupported class index type");
    }
}
}
struct CompiledPostModel::Impl {
    hbPackedDNNHandle_t packed=nullptr;hbDNNHandle_t model=nullptr;hbDNNTaskHandle_t task=nullptr;
    hbDNNTensor input{};std::vector<hbDNNTensor> outputs;
    int classes,lh,lw,mh,mw,timeout;
    bool quarantined=false;
    Addressing input_address{},output_address{};
    Impl(const std::string& path,const std::string& name,int c,int h,int w,int a,int b,int t)
      :classes(c),lh(h),lw(w),mh(a),mw(b),timeout(t){
      try {
        const char* file=path.c_str();check(hbDNNInitializeFromFiles(&packed,&file,1),"post model load");
        const char** names=nullptr;int count=0;check(hbDNNGetModelNameList(&names,&count,packed),"post model names");
        require(count>0,"empty post model package");require(count==1||!name.empty(),"multi-model post package requires --post-name");
        check(hbDNNGetModelHandle(&model,packed,name.empty()?names[0]:name.c_str()),"post model handle");
        check(hbDNNGetInputCount(&count,model),"post input count");require(count==1,"post model requires one featuremap input");
        check(hbDNNGetInputTensorProperties(&input.properties,model,0),"post input properties");
        auto ax=axes(input.properties);
        require(input.properties.validShape.dimensionSize[ax.c]==c&&input.properties.validShape.dimensionSize[ax.h]==h&&
                input.properties.validShape.dimensionSize[ax.w]==w,"post model logits shape mismatch");
        require(input.properties.tensorType==HB_DNN_TENSOR_TYPE_F32&&input.properties.quantiType==NONE,
                "compile post model with float32 featuremap runtime input; native quantized input is not supported here");
        allocate(input);input_address=addressing(input.properties);
        check(hbDNNGetOutputCount(&count,model),"post output count");require(count==1,"post model must output only one mask");
        outputs.resize(count);check(hbDNNGetOutputTensorProperties(&outputs[0].properties,model,0),"post output properties");
        const auto& p=outputs[0].properties;ax=axes(p);
        require(p.validShape.dimensionSize[ax.c]==1&&p.validShape.dimensionSize[ax.h]==a&&p.validShape.dimensionSize[ax.w]==b,
                "post model output must be one-channel mask at source resolution");
        require(p.quantiType==NONE,"class-index output must be unquantized");allocate(outputs[0]);output_address=addressing(p);
      }catch(...){cleanup();throw;}
    }
    void cleanup() noexcept {
        if(task){if(hbDNNReleaseTask(task)!=0){quarantined=true;return;}task=nullptr;}
        for(auto& tensor:outputs)if(tensor.sysMem[0].virAddr)hbSysFreeMem(&tensor.sysMem[0]);
        outputs.clear();if(input.sysMem[0].virAddr){hbSysFreeMem(&input.sysMem[0]);input.sysMem[0]={};}
        if(packed){hbDNNRelease(packed);packed=nullptr;}
    }
    ~Impl(){cleanup();}
    void run(const perception::Tensor& logits,std::vector<std::uint8_t>& mask){
        require(!quarantined&&!task,"post task is quarantined");logits.validate();
        require(logits.channels==classes&&logits.height==lh&&logits.width==lw,"unexpected logits dimensions");
        std::memset(input.sysMem[0].virAddr,0,input.sysMem[0].memSize);
        auto* target=static_cast<float*>(input.sysMem[0].virAddr);
        for(int c=0;c<classes;++c)for(int y=0;y<lh;++y){
            auto* row=target+std::size_t(c)*input_address.c+std::size_t(y)*input_address.y;
            const auto* source=logits.data.data()+(std::size_t(c)*lh+y)*lw;
            if(input_address.x==1)std::memcpy(row,source,std::size_t(lw)*sizeof(float));
            else for(int x=0;x<lw;++x)row[std::size_t(x)*input_address.x]=source[x];
        }
        check(hbSysFlushMem(&input.sysMem[0],HB_SYS_MEM_CACHE_CLEAN),"post input flush");
        hbDNNInferCtrlParam params{};HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&params);
        auto* out=outputs.data();
        try {
            check(hbDNNInfer(&task,&out,&input,model,&params),"post model infer");
            check(hbDNNWaitTaskDone(task,timeout),"post model wait");
            check(hbDNNReleaseTask(task),"post model release task");task=nullptr;
        }catch(...){if(task&&hbDNNReleaseTask(task)==0)task=nullptr;else if(task)quarantined=true;throw;}
        check(hbSysFlushMem(&outputs[0].sysMem[0],HB_SYS_MEM_CACHE_INVALIDATE),"post output invalidate");
        mask.resize(std::size_t(mh)*mw);
        copyLabels(outputs[0],output_address,mh,mw,classes,mask);
    }
};

VseResize::VseResize(int a,int b,int c,int d,int t):impl_(new Impl(a,b,c,d,t)){}
VseResize::~VseResize(){if(impl_){impl_->cleanup();if(impl_->quarantined)impl_.release();}}
const std::vector<std::uint8_t>& VseResize::run(const std::vector<std::uint8_t>& raw){return impl_->run(raw.data(),raw.size(),impl_->sw);}
const std::vector<std::uint8_t>& VseResize::run(const std::uint8_t* raw,std::size_t bytes,std::size_t stride){return impl_->run(raw,bytes,stride);}
std::string VseResize::infoJson() const {
    std::ostringstream s;s<<"{\"source_width\":"<<impl_->sw<<",\"source_height\":"<<impl_->sh
        <<",\"expected_input_stride\":"<<vse_layout::inputStride(impl_->sw)
        <<",\"actual_input_stride\":"<<impl_->input.buffer.stride<<",\"actual_input_vstride\":"<<impl_->input.buffer.vstride
        <<",\"target_width\":"<<impl_->dw<<",\"target_height\":"<<impl_->dh
        <<",\"actual_output_stride\":"<<impl_->output_stride<<",\"actual_output_vstride\":"<<impl_->output_vstride<<'}';return s.str();
}
CompiledPostModel::CompiledPostModel(const std::string& m,const std::string& n,int c,int h,int w,int a,int b,int t)
  :impl_(new Impl(m,n,c,h,w,a,b,t)){}
CompiledPostModel::~CompiledPostModel(){
    if(impl_){impl_->cleanup();if(impl_->quarantined)impl_.release();} // Retain until the isolated worker exits.
}
void CompiledPostModel::run(const perception::Tensor& t,std::vector<std::uint8_t>& m){impl_->run(t,m);}
#else
struct VseResize::Impl{};
struct CompiledPostModel::Impl{};
VseResize::VseResize(int,int,int,int,int){throw std::runtime_error("VSE unavailable: configure HW_AB_WITH_X5=ON on X5");}
VseResize::~VseResize()=default;
const std::vector<std::uint8_t>& VseResize::run(const std::vector<std::uint8_t>&){throw std::runtime_error("VSE unavailable");}
const std::vector<std::uint8_t>& VseResize::run(const std::uint8_t*,std::size_t,std::size_t){throw std::runtime_error("VSE unavailable");}
std::string VseResize::infoJson() const{return "null";}
CompiledPostModel::CompiledPostModel(const std::string&,const std::string&,int,int,int,int,int,int){throw std::runtime_error("compiled post model unavailable in host build");}
CompiledPostModel::~CompiledPostModel()=default;
void CompiledPostModel::run(const perception::Tensor&,std::vector<std::uint8_t>&){throw std::runtime_error("compiled post model unavailable");}
#endif

#if defined(HW_AB_WITH_X5)
namespace {
double nativeValue(const hbDNNTensor& t,std::size_t pos,int channel,int channels){
    const auto& p=t.properties;const void* data=t.sysMem[0].virAddr;double value=0;
    switch(p.tensorType){
      case HB_DNN_TENSOR_TYPE_F32:value=static_cast<const float*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_S8:value=static_cast<const int8_t*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_U8:value=static_cast<const uint8_t*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_S16:value=static_cast<const int16_t*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_U16:value=static_cast<const uint16_t*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_S32:value=static_cast<const int32_t*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_U32:value=static_cast<const uint32_t*>(data)[pos];break;
      case HB_DNN_TENSOR_TYPE_S64:value=static_cast<double>(static_cast<const int64_t*>(data)[pos]);break;
      case HB_DNN_TENSOR_TYPE_U64:value=static_cast<double>(static_cast<const uint64_t*>(data)[pos]);break;
      default:throw std::runtime_error("unsupported fused output type");
    }
    if(p.quantiType==SCALE){
        const int index=p.scale.scaleLen==1?0:channel;
        require(p.scale.scaleData&&(p.scale.scaleLen==1||p.scale.scaleLen==channels),"invalid fused output scales");
        const auto ca=axes(p).c;require(p.scale.scaleLen==1||p.quantizeAxis==ca,"unsupported quantization axis");
        require(std::isfinite(p.scale.scaleData[index])&&p.scale.scaleData[index]>0,"invalid quantization scale");
        require(p.scale.zeroPointLen==0||(p.scale.zeroPointLen==p.scale.scaleLen&&p.scale.zeroPointData),"invalid zero points");
        if(p.scale.zeroPointLen)value-=p.scale.zeroPointData[index];value*=p.scale.scaleData[index];
    }else if(p.quantiType==SHIFT){
        require(p.shift.shiftData&&(p.shift.shiftLen==1||p.shift.shiftLen==channels),"invalid fused shifts");
        const int index=p.shift.shiftLen==1?0:channel;
        require(p.shift.shiftLen==1||p.quantizeAxis==axes(p).c,"unsupported shift axis");
        require(p.shift.shiftData[index]<=126,"invalid fused shift");value=std::ldexp(value,-int(p.shift.shiftData[index]));
    }else require(p.quantiType==NONE,"unknown fused quantization");
    require(std::isfinite(value),"nonfinite fused output");return value;
}
template<class T> std::vector<uint8_t> makeLabelLookup(const hbDNNTensor& tensor) {
    const std::size_t count=std::size_t(1)<<(sizeof(T)*8);
    std::vector<uint8_t> lut(count,255);T sample{};hbDNNTensor trial=tensor;
    trial.sysMem[0].virAddr=&sample;
    for(std::size_t raw=0;raw<count;++raw){
        sample=static_cast<T>(raw);
        const double value=nativeValue(trial,0,0,1),label=std::round(value);
        if(std::abs(value-label)<=1e-5&&value>=0&&value<=2)lut[raw]=static_cast<uint8_t>(label);
    }
    return lut;
}
void optimizedLabels(const hbDNNTensor& tensor,Addressing address,int h,int w,
                     const std::vector<uint8_t>& lookup,std::vector<uint8_t>& mask){
    const void* data=tensor.sysMem[0].virAddr;
    const label_reader::Layout layout{address.y,address.x};
    if(tensor.properties.quantiType==NONE){
        switch(tensor.properties.tensorType){
            case HB_DNN_TENSOR_TYPE_U8:return label_reader::integers(static_cast<const uint8_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_S8:return label_reader::integers(static_cast<const int8_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_U16:return label_reader::integers(static_cast<const uint16_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_S16:return label_reader::integers(static_cast<const int16_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_U32:return label_reader::integers(static_cast<const uint32_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_S32:return label_reader::integers(static_cast<const int32_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_U64:return label_reader::integers(static_cast<const uint64_t*>(data),layout,h,w,mask);
            case HB_DNN_TENSOR_TYPE_S64:return label_reader::integers(static_cast<const int64_t*>(data),layout,h,w,mask);
            default:return copyLabels(tensor,address,h,w,3,mask);
        }
    }
    if(!lookup.empty()){
        switch(tensor.properties.tensorType){
            case HB_DNN_TENSOR_TYPE_U8:return label_reader::lookup(static_cast<const uint8_t*>(data),layout,h,w,lookup,mask);
            case HB_DNN_TENSOR_TYPE_S8:return label_reader::lookup(static_cast<const int8_t*>(data),layout,h,w,lookup,mask);
            case HB_DNN_TENSOR_TYPE_U16:return label_reader::lookup(static_cast<const uint16_t*>(data),layout,h,w,lookup,mask);
            case HB_DNN_TENSOR_TYPE_S16:return label_reader::lookup(static_cast<const int16_t*>(data),layout,h,w,lookup,mask);
            default:throw std::runtime_error("unexpected label lookup type");
        }
    }
    mask.resize(std::size_t(h)*w);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){
        const double value=nativeValue(tensor,std::size_t(y)*address.y+std::size_t(x)*address.x,0,1);
        require(std::abs(value-std::round(value))<=1e-5&&value>=0&&value<=2,"invalid dequantized fused class index");
        mask[std::size_t(y)*w+x]=static_cast<uint8_t>(std::round(value));
    }
}
const char* tensorTypeName(int type){
    switch(type){
      case HB_DNN_TENSOR_TYPE_U8:return "uint8";case HB_DNN_TENSOR_TYPE_S8:return "int8";
      case HB_DNN_TENSOR_TYPE_U16:return "uint16";case HB_DNN_TENSOR_TYPE_S16:return "int16";
      case HB_DNN_TENSOR_TYPE_U32:return "uint32";case HB_DNN_TENSOR_TYPE_S32:return "int32";
      case HB_DNN_TENSOR_TYPE_U64:return "uint64";case HB_DNN_TENSOR_TYPE_S64:return "int64";
      case HB_DNN_TENSOR_TYPE_F32:return "float32";case HB_DNN_IMG_TYPE_NV12:return "nv12";
      default:return "other";
    }
}
void tensorJson(std::ostream& s,const hbDNNTensor& t){
    const auto& p=t.properties;s<<"{\"type\":\""<<tensorTypeName(p.tensorType)<<"\",\"type_id\":"<<p.tensorType
        <<",\"layout_id\":"<<p.tensorLayout<<",\"quanti_type_id\":"<<p.quantiType
        <<",\"aligned_bytes\":"<<p.alignedByteSize<<",\"allocated_bytes\":"<<t.sysMem[0].memSize;
    for(int aligned=0;aligned<2;++aligned){s<<(aligned?",\"aligned_shape\":[":",\"valid_shape\":[");
        const auto& shape=aligned?p.alignedShape:p.validShape;
        for(int j=0;j<shape.numDimensions;++j){if(j)s<<',';s<<shape.dimensionSize[j];}s<<']';}
    if(p.quantiType==SCALE&&p.scale.scaleData&&p.scale.scaleLen>0&&p.scale.scaleLen<=3){
        s<<",\"scales\":[";for(int j=0;j<p.scale.scaleLen;++j){if(j)s<<',';s<<p.scale.scaleData[j];}s<<']';}
    if(p.quantiType==SCALE&&p.scale.zeroPointData&&p.scale.zeroPointLen>0&&p.scale.zeroPointLen<=3){
        s<<",\"zero_points\":[";for(int j=0;j<p.scale.zeroPointLen;++j){if(j)s<<',';s<<int(p.scale.zeroPointData[j]);}s<<']';}
    if(p.quantiType==SHIFT&&p.shift.shiftData&&p.shift.shiftLen>0&&p.shift.shiftLen<=3){
        s<<",\"shifts\":[";for(int j=0;j<p.shift.shiftLen;++j){if(j)s<<',';s<<int(p.shift.shiftData[j]);}s<<']';}
    s<<'}';
}
}
struct FusedNv12Model::Impl {
    hbPackedDNNHandle_t packed=nullptr;hbDNNHandle_t model=nullptr;hbDNNTaskHandle_t task=nullptr;
    hbDNNTensor input{};std::vector<hbDNNTensor> outputs;
    Addressing mask_address{},logits_address{};int mask_index=-1,logits_index=-1;
    int width,height,mw,mh,timeout;bool diagnostic,optimized,quarantined=false;
    std::vector<uint8_t> label_lookup;FusedTimings timing{};
    std::vector<uint8_t> captured_input,captured_mask;
    std::size_t verified_frames=0;
    Impl(const std::string& file,int w,int h,int a,int b,bool debug,int t,bool fast)
      :width(w),height(h),mw(a),mh(b),timeout(t),diagnostic(debug),optimized(fast){try{
        const char* name=file.c_str();check(hbDNNInitializeFromFiles(&packed,&name,1),"fused load");
        const char** names=nullptr;int count=0;check(hbDNNGetModelNameList(&names,&count,packed),"fused names");
        require(count==1,"fused package must contain one model");check(hbDNNGetModelHandle(&model,packed,names[0]),"fused handle");
        check(hbDNNGetInputCount(&count,model),"fused input count");require(count==1,"fused model needs one input");
        check(hbDNNGetInputTensorProperties(&input.properties,model,0),"fused input properties");
        const auto& p=input.properties;const auto ax=axes(p);
        require(p.tensorType==HB_DNN_IMG_TYPE_NV12,"fused model requires packed NV12 image input (input_source: pyramid)");
        require(p.validShape.dimensionSize[ax.h]==h&&p.validShape.dimensionSize[ax.w]==w,"fused input resolution mismatch");
        require(p.alignedShape.dimensionSize[ax.h]==h&&p.alignedShape.dimensionSize[ax.w]==w,"adapter requires contiguous packed NV12 spatial storage");
        require(p.alignedByteSize>=w*h*3/2&&p.alignedByteSize<=256*1024*1024,"invalid fused input storage");
        check(hbSysAllocCachedMem(&input.sysMem[0],p.alignedByteSize),"fused input allocation");
        require(input.sysMem[0].virAddr&&input.sysMem[0].memSize>=std::size_t(w)*h*3/2,"short fused input allocation");
        check(hbDNNGetOutputCount(&count,model),"fused output count");require(count==(debug?2:1),"use mask-only BIN for performance, diagnostic BIN for debug");
        outputs.resize(count);
        for(int i=0;i<count;++i){
            check(hbDNNGetOutputTensorProperties(&outputs[i].properties,model,i),"fused output properties");
            const auto& q=outputs[i].properties;const auto aa=axes(q);allocate(outputs[i]);
            if(q.validShape.dimensionSize[aa.c]==1&&q.validShape.dimensionSize[aa.h]==b&&q.validShape.dimensionSize[aa.w]==a){
                require(mask_index<0,"duplicate masks");mask_index=i;mask_address=addressing(q);
            }else if(debug&&q.validShape.dimensionSize[aa.c]==3&&q.validShape.dimensionSize[aa.h]==60&&q.validShape.dimensionSize[aa.w]==104){
                require(logits_index<0,"duplicate logits");logits_index=i;logits_address=addressing(q);
            }else throw std::runtime_error("unexpected fused output shape/layout");
        }
        require(mask_index>=0&&(!debug||logits_index>=0),"missing fused outputs");
        if(optimized&&outputs[mask_index].properties.quantiType!=NONE){
            const auto& mask=outputs[mask_index];
            switch(mask.properties.tensorType){
                case HB_DNN_TENSOR_TYPE_U8:label_lookup=makeLabelLookup<uint8_t>(mask);break;
                case HB_DNN_TENSOR_TYPE_S8:label_lookup=makeLabelLookup<int8_t>(mask);break;
                case HB_DNN_TENSOR_TYPE_U16:label_lookup=makeLabelLookup<uint16_t>(mask);break;
                case HB_DNN_TENSOR_TYPE_S16:label_lookup=makeLabelLookup<int16_t>(mask);break;
                default:break;
            }
        }
      }catch(...){cleanup();throw;}}
    void cleanup() noexcept{
        if(quarantined&&!task)return; // A successful infer with a null handle has no confirmed cancellation.
        if(task){if(hbDNNReleaseTask(task)!=0){quarantined=true;return;}task=nullptr;}
        for(auto& o:outputs)if(o.sysMem[0].virAddr)hbSysFreeMem(&o.sysMem[0]);outputs.clear();
        if(input.sysMem[0].virAddr){hbSysFreeMem(&input.sysMem[0]);input.sysMem[0]={};}
        if(packed){hbDNNRelease(packed);packed=nullptr;}
    }
    ~Impl(){cleanup();}
    void referenceLabels(const hbDNNTensor& mt,std::vector<uint8_t>& mask){
        mask.resize(std::size_t(mw)*mh);
        if(mt.properties.quantiType==NONE)copyLabels(mt,mask_address,mh,mw,3,mask);
        else for(int y=0;y<mh;++y)for(int x=0;x<mw;++x){
            const double v=nativeValue(mt,std::size_t(y)*mask_address.y+std::size_t(x)*mask_address.x,0,1);
            require(std::abs(v-std::round(v))<=1e-5&&v>=0&&v<=2,"invalid dequantized fused class index");
            mask[std::size_t(y)*mw+x]=static_cast<uint8_t>(std::round(v));
        }
    }
    void run(const uint8_t* source,std::size_t bytes,std::vector<uint8_t>& mask,perception::Tensor* logits,bool verify,bool capture){
        require(!quarantined&&!task,"fused model quarantined");require(source&&bytes==std::size_t(width)*height*3/2,"fused NV12 size mismatch");
        require(diagnostic==(logits!=nullptr),"diagnostic output request mismatch");
        timing={};auto begin=HardwareClock::now();
        std::memcpy(input.sysMem[0].virAddr,source,bytes);
        if(input.sysMem[0].memSize>bytes)std::memset(static_cast<uint8_t*>(input.sysMem[0].virAddr)+bytes,0,input.sysMem[0].memSize-bytes);
        timing.input_copy_ms=milliseconds(begin);begin=HardwareClock::now();
        check(hbSysFlushMem(&input.sysMem[0],HB_SYS_MEM_CACHE_CLEAN),"fused input cache clean");
        if(capture){const auto* p=static_cast<const uint8_t*>(input.sysMem[0].virAddr);
            captured_input.assign(p,p+input.properties.alignedByteSize);}
        timing.input_clean_ms=milliseconds(begin);begin=HardwareClock::now();
        hbDNNInferCtrlParam ctrl{};HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);auto* out=outputs.data();
        try{
            check(hbDNNInfer(&task,&out,&input,model,&ctrl),"fused infer");
            if(!task){quarantined=true;throw std::runtime_error("fused null task; resources quarantined until worker exit");}
            check(hbDNNWaitTaskDone(task,timeout),"fused wait");check(hbDNNReleaseTask(task),"fused release");task=nullptr;
        }catch(...){if(task&&hbDNNReleaseTask(task)==0)task=nullptr;else if(task)quarantined=true;throw;}
        timing.model_wait_ms=milliseconds(begin);begin=HardwareClock::now();
        auto& mt=outputs[mask_index];check(hbSysFlushMem(&mt.sysMem[0],HB_SYS_MEM_CACHE_INVALIDATE),"fused mask invalidate");
        if(capture){const auto* p=static_cast<const uint8_t*>(mt.sysMem[0].virAddr);
            captured_mask.assign(p,p+mt.properties.alignedByteSize);}
        timing.mask_invalidate_ms=milliseconds(begin);begin=HardwareClock::now();
        mask.resize(std::size_t(mw)*mh);
        if(optimized)optimizedLabels(mt,mask_address,mh,mw,label_lookup,mask);
        else referenceLabels(mt,mask);
        timing.mask_extract_ms=milliseconds(begin);begin=HardwareClock::now();
        // Accuracy only: compare two readers of the very same native tensor.
        // Performance runs do not allocate or execute this reference pass.
        if(verify&&optimized){std::vector<uint8_t> original;referenceLabels(mt,original);
            require(original==mask,"optimized label extraction changed native output");++verified_frames;}
        begin=HardwareClock::now();
        if(logits){auto& lt=outputs[logits_index];check(hbSysFlushMem(&lt.sysMem[0],HB_SYS_MEM_CACHE_INVALIDATE),"debug logits invalidate");
            logits->channels=3;logits->height=60;logits->width=104;logits->data.resize(3*60*104);
            for(int c=0;c<3;++c)for(int y=0;y<60;++y)for(int x=0;x<104;++x)
                logits->data[(std::size_t(c)*60+y)*104+x]=static_cast<float>(nativeValue(lt,
                    std::size_t(c)*logits_address.c+std::size_t(y)*logits_address.y+std::size_t(x)*logits_address.x,c,3));
        }
        if(logits)timing.diagnostic_extract_ms=milliseconds(begin);
    }
};
FusedNv12Model::FusedNv12Model(const std::string& p,int w,int h,int a,int b,bool d,int t,bool fast):impl_(new Impl(p,w,h,a,b,d,t,fast)){}
FusedNv12Model::~FusedNv12Model(){if(impl_){impl_->cleanup();if(impl_->quarantined)impl_.release();}}
void FusedNv12Model::run(const uint8_t* p,std::size_t n,std::vector<uint8_t>& m,perception::Tensor* t,bool verify,bool capture){impl_->run(p,n,m,t,verify,capture);}
const std::vector<uint8_t>& FusedNv12Model::capturedInputStorage() const{return impl_->captured_input;}
const std::vector<uint8_t>& FusedNv12Model::capturedMaskStorage() const{return impl_->captured_mask;}
const FusedTimings& FusedNv12Model::timings() const{return impl_->timing;}
std::string FusedNv12Model::infoJson() const{
    std::ostringstream s;s<<std::setprecision(9)<<"{\"optimized_label_reader\":"<<(impl_->optimized?"true":"false")
        <<",\"same_tensor_verified_frames\":"<<impl_->verified_frames
        <<",\"label_lookup_bytes\":"<<impl_->label_lookup.size()<<",\"mask_output_index\":"<<impl_->mask_index
        <<",\"mask_stride_y\":"<<impl_->mask_address.y<<",\"mask_stride_x\":"<<impl_->mask_address.x<<",\"input\":";
    tensorJson(s,impl_->input);s<<",\"outputs\":[";
    for(std::size_t i=0;i<impl_->outputs.size();++i){if(i)s<<',';tensorJson(s,impl_->outputs[i]);}s<<"]}";return s.str();
}
#else
struct FusedNv12Model::Impl{};
FusedNv12Model::FusedNv12Model(const std::string&,int,int,int,int,bool,int,bool){throw std::runtime_error("fused BPU unavailable in host build");}
FusedNv12Model::~FusedNv12Model()=default;
void FusedNv12Model::run(const uint8_t*,std::size_t,std::vector<uint8_t>&,perception::Tensor*,bool,bool){throw std::runtime_error("fused BPU unavailable");}
const std::vector<uint8_t>& FusedNv12Model::capturedInputStorage() const{throw std::runtime_error("fused BPU unavailable");}
const std::vector<uint8_t>& FusedNv12Model::capturedMaskStorage() const{throw std::runtime_error("fused BPU unavailable");}
const FusedTimings& FusedNv12Model::timings() const{throw std::runtime_error("fused BPU unavailable");}
std::string FusedNv12Model::infoJson() const{return "null";}
#endif
