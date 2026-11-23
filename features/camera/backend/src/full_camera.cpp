#include "stereo_inputs.hpp"
#include "runtime_status.hpp"
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#if defined(__linux__) && defined(R33_WITH_MO)
#include <MoLocalSdkInterface.h>
#include <dlfcn.h>
#endif

namespace r33::stereo {
namespace {
using Clock=std::chrono::steady_clock;
double duration(Clock::time_point begin){return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();}
[[noreturn]] void failure(std::uint32_t code,int native,const char* feature,const std::string& detail,perception::FrameInfo frame={}){
    throw r28::Failure({code,native,feature,{},detail,frame});
}
}
struct FullCameraAdapter::Impl {
    std::mutex mutex;
    CameraOptions options;
    Intrinsics intrinsics;
    bool need_hires=true;
#if defined(__linux__) && defined(R33_WITH_MO)
    void* library=nullptr;
    int handle=-1,original_mode=-1;
    decltype(&moSetConfigPath) set_config=nullptr;
    decltype(&moOpenCamera) open_camera=nullptr;
    decltype(&moCloseCamera) close_camera=nullptr;
    decltype(&moGetOneRGBDFrame) get_rgbd=nullptr;
    decltype(&moReleaseRGBDFrame) release_rgbd=nullptr;
    decltype(&moGetOneGdcFrame) get_gdc=nullptr;
    decltype(&moReleaseGdcFrame) release_gdc=nullptr;
    decltype(&moSetDisparityCalculationMode) set_mode=nullptr;
    decltype(&moGetDisparityCalculationMode) get_mode=nullptr;
    decltype(&moGetBpuRatio) get_ratio=nullptr;
    decltype(&moGetFrameRateOrg) get_org_fps=nullptr;
    decltype(&moGetFrameRateDisp) get_disp_fps=nullptr;
    template<class Function> Function symbol(const char* name,bool required=true){
        dlerror();void* address=dlsym(library,name);const char* error=dlerror();
        if(error||!address){if(required)failure(1001,0,"stereo.full_camera.load",std::string("Missing SDK symbol: ")+name);return nullptr;}
        Function function=nullptr;static_assert(sizeof(function)==sizeof(address),"dlsym pointer size mismatch");std::memcpy(&function,&address,sizeof(function));return function;
    }
    void initialize(){
        library=dlopen(options.sdk_path.c_str(),RTLD_NOW|RTLD_GLOBAL);
        if(!library){const char* why=dlerror();failure(1001,0,"stereo.full_camera.load",std::string("Cannot load SDK: ")+(why?why:"unknown loader error"));}
        set_config=symbol<decltype(set_config)>("moSetConfigPath");
        open_camera=symbol<decltype(open_camera)>("moOpenCamera");close_camera=symbol<decltype(close_camera)>("moCloseCamera");
        get_rgbd=symbol<decltype(get_rgbd)>("moGetOneRGBDFrame");release_rgbd=symbol<decltype(release_rgbd)>("moReleaseRGBDFrame");
        if(need_hires){get_gdc=symbol<decltype(get_gdc)>("moGetOneGdcFrame");release_gdc=symbol<decltype(release_gdc)>("moReleaseGdcFrame");}
        set_mode=symbol<decltype(set_mode)>("moSetDisparityCalculationMode");get_mode=symbol<decltype(get_mode)>("moGetDisparityCalculationMode");
        get_ratio=symbol<decltype(get_ratio)>("moGetBpuRatio",false);get_org_fps=symbol<decltype(get_org_fps)>("moGetFrameRateOrg",false);get_disp_fps=symbol<decltype(get_disp_fps)>("moGetFrameRateDisp",false);
        if(!options.config_path.empty()){const int code=set_config(options.config_path.c_str());if(code)failure(1001,code,"stereo.full_camera.configure","moSetConfigPath failed");}
        const int open_code=open_camera(&handle);if(open_code||handle<0)failure(1001,open_code,"stereo.full_camera.open","moOpenCamera failed");
        const int mode_code=get_mode(handle,&original_mode);if(mode_code||original_mode<0)failure(1001,mode_code,"stereo.full_camera.configure","Cannot query original disparity mode");
        const int set_code=set_mode(handle,2);if(set_code)failure(1001,set_code,"stereo.full_camera.configure","Cannot enable continuous SDK disparity mode2");
        int effective=-1;const int read_code=get_mode(handle,&effective);if(read_code||effective!=2)failure(1001,read_code,"stereo.full_camera.configure","Disparity mode2 readback mismatch");
    }
    r28::Status shutdown(){
        r28::Status status;
        if(handle>=0){
            int restore_code=0;if(original_mode>0&&set_mode)restore_code=set_mode(handle,original_mode);
            const int close_code=close_camera?close_camera(handle):-999;handle=-1;
            if(close_code){status={2006,close_code,"stereo.full_camera.close",{},"moCloseCamera failed; SDK retained to keep vendor code mapped",{}};library=nullptr;return status;}
            if(restore_code)status={2006,restore_code,"stereo.full_camera.close",{},"Original disparity mode restore failed",{}};
        }
        if(library){void* owned=library;library=nullptr;const int code=dlclose(owned);if(code&&status.ok())status={2006,code,"stereo.full_camera.close",{},"dlclose failed",{}};}
        return status;
    }
    static void metadata(const MoFrame& source,int width,int height,std::size_t bytes,const char* plane,perception::FrameInfo info={}){
        if(!source.Data||source.ResolutionWidth!=width||source.ResolutionHeight!=height||source.BytesUsed!=bytes||!source.TimeStamp)
            failure(1002,0,"stereo.full_camera.frame",std::string("Invalid ")+plane+" SDK plane geometry/storage/timestamp",info);
    }
    static std::shared_ptr<const example::Image> image(const MoFrame& source,int width,int height,const char* plane){
        const auto size=std::size_t(width)*height*3/2;metadata(source,width,height,size,plane,{source.FrameId,source.TimeStamp});
        auto owned=std::make_shared<example::Image>();owned->width=width;owned->height=height;owned->stride=width;owned->format=perception::PixelFormat::NV12;
        owned->decoded_format=plane;owned->bytes.resize(size);std::memcpy(owned->bytes.data(),source.Data,size);return owned;
    }
    bool rgbd(Frame& output,int& last_code,demo_camera::AcquisitionTiming& timing){
        MoFrame left{},right{},disparity{};const auto begin=Clock::now();const int code=get_rgbd(handle,&left,&right,&disparity);
        timing.sdk_get_ms+=duration(begin);++timing.attempts;
        const auto release=[&]{const auto start=Clock::now();const int result=release_rgbd(handle,&left,&right,&disparity);timing.release_ms+=duration(start);return result;};
        if(code){last_code=timing.last_error=code;if(left.Data||right.Data||disparity.Data){const int released=release();if(released)failure(1001,released,"stereo.full_camera.release","Partial RGBD release failed");}return false;}
        try{
            const auto start=Clock::now();const perception::FrameInfo info{left.FrameId,left.TimeStamp};
            metadata(left,640,544,640U*544U*3/2,"RGBD left",info);metadata(right,640,544,640U*544U*3/2,"RGBD right",info);metadata(disparity,640,544,640U*544U*2,"Q8.8 disparity",info);
            if(left.FrameId!=right.FrameId||left.FrameId!=disparity.FrameId||left.TimeStamp!=right.TimeStamp||left.TimeStamp!=disparity.TimeStamp||disparity.BitsPerPixel!=16)
                failure(1002,0,"stereo.full_camera.frame","RGBD triplet IDs/timestamps or disparity format differ",info);
            output.info=info;output.left=image(left,640,544,"mo-rgbd-nv12");
            auto pixels=std::make_shared<std::vector<std::uint16_t>>(640U*544U);std::memcpy(pixels->data(),disparity.Data,pixels->size()*sizeof(std::uint16_t));output.disparity=std::move(pixels);
            output.right={info,640,544,right.BytesUsed,true,false};validateDisparity(output.left->view(),info,output.disparityView(),intrinsics);timing.copy_ms+=duration(start);
        }catch(...){const int released=release();if(released)failure(1001,released,"stereo.full_camera.release","RGBD release failed after rejecting the frame");throw;}
        const int released=release();if(released)failure(1001,released,"stereo.full_camera.release","RGBD release failed");return true;
    }
    struct Gdc {std::shared_ptr<const example::Image> left;perception::FrameInfo info;};
    bool gdc(Gdc& output,int& last_code,demo_camera::AcquisitionTiming& timing){
        MoFrame left{},right{};const auto begin=Clock::now();const int code=get_gdc(handle,&left,&right);
        timing.sdk_get_ms+=duration(begin);++timing.attempts;
        const auto release=[&]{const auto start=Clock::now();const int result=release_gdc(handle,&left,&right);timing.release_ms+=duration(start);return result;};
        if(code){last_code=timing.last_error=code;if(left.Data||right.Data){const int released=release();if(released)failure(1001,released,"stereo.full_camera.release","Partial GDC release failed");}return false;}
        try{
            const auto start=Clock::now();output.info={left.FrameId,left.TimeStamp};
            metadata(left,1280,1088,1280U*1088U*3/2,"GDC left",output.info);metadata(right,1280,1088,1280U*1088U*3/2,"GDC right",output.info);
            if(left.FrameId!=right.FrameId||left.TimeStamp!=right.TimeStamp)failure(1002,0,"stereo.full_camera.frame","GDC pair IDs/timestamps differ",output.info);
            output.left=image(left,1280,1088,"mo-gdc-nv12");timing.copy_ms+=duration(start);
        }catch(...){const int released=release();if(released)failure(1001,released,"stereo.full_camera.release","GDC release failed after rejecting the frame");throw;}
        const int released=release();if(released)failure(1001,released,"stereo.full_camera.release","GDC release failed");return true;
    }
    Frame next(){
        if(handle<0)failure(1001,0,"stereo.full_camera.acquire","Camera is closed");
        const auto deadline=Clock::now()+std::chrono::milliseconds(options.acquire_timeout_ms);
        std::optional<Frame> current;std::optional<Gdc> high;demo_camera::AcquisitionTiming timing;int last_code=0;
        do{
            if(!current){Frame frame;if(rgbd(frame,last_code,timing))current=std::move(frame);}
            if(current&&!need_hires){current->acquisition=timing;current->acquisition.sdk_timestamp=true;return std::move(*current);}
            if(need_hires&&!high){Gdc frame;if(gdc(frame,last_code,timing))high=std::move(frame);}
            if(current&&high){
                if(current->info.id==high->info.id){
                    validateHires(high->left->view(),high->info,current->info);current->hires_left=high->left;current->hires_info=high->info;
                    current->acquisition=timing;current->acquisition.sdk_timestamp=true;return std::move(*current);
                }
                // Discard only owned copies. All SDK planes have already been
                // released; no input/BPU/result queue is held across frames.
                if(current->info.id<high->info.id)current.reset();else high.reset();
            }
            if(Clock::now()<deadline){const auto start=Clock::now();std::this_thread::sleep_for(std::chrono::milliseconds(options.retry_interval_ms));timing.retry_sleep_ms+=duration(start);}
        }while(Clock::now()<deadline);
        failure(1001,last_code,"stereo.full_camera.acquire","Timed out waiting for an exactly paired RGBD/GDC frame",current?current->info:perception::FrameInfo{});
    }
    demo_camera::Diagnostics diagnostics(){
        demo_camera::Diagnostics out;out.requested_mode=2;out.original_mode=original_mode;
        if(handle<0)failure(1001,0,"stereo.full_camera.diagnostics","Camera is closed");
        if(get_mode&&get_mode(handle,&out.effective_mode)==0)out.mode_available=true;
        if(get_ratio&&get_ratio(handle,&out.bpu_ratio)==0)out.ratio_available=true;
        if(get_org_fps&&get_org_fps(handle,&out.original_fps)==0)out.original_fps_available=true;
        if(get_disp_fps&&get_disp_fps(handle,&out.disparity_fps)==0)out.disparity_fps_available=true;
        return out;
    }
#else
    r28::Status shutdown(){return {};}
#endif
};
FullCameraAdapter::FullCameraAdapter(const CameraOptions& options,const Intrinsics& k,bool need_hires):impl_(std::make_unique<Impl>()){
    validateIntrinsics(k);if(k.width!=640||k.height!=544||options.sdk_path.empty()||options.acquire_timeout_ms<=0||options.retry_interval_ms<=0||options.retry_interval_ms>options.acquire_timeout_ms)
        failure(2001,0,"stereo.full_camera.configure","Invalid original camera geometry/options");
    impl_->options=options;impl_->intrinsics=k;impl_->need_hires=need_hires;
#if defined(__linux__) && defined(R33_WITH_MO)
    try{impl_->initialize();}catch(...){try{impl_->shutdown();}catch(...){}throw;}
#else
    failure(1001,0,"stereo.full_camera.open","Full camera input requires Linux/X5 and real vendor headers; no simulated camera fallback");
#endif
}
FullCameraAdapter::~FullCameraAdapter(){try{close();}catch(...){}}
Frame FullCameraAdapter::next(){std::lock_guard<std::mutex> lock(impl_->mutex);
#if defined(__linux__) && defined(R33_WITH_MO)
    return impl_->next();
#else
    failure(1001,0,"stereo.full_camera.acquire","Camera input unavailable in this build");
#endif
}
demo_camera::Diagnostics FullCameraAdapter::diagnostics(){std::lock_guard<std::mutex> lock(impl_->mutex);
#if defined(__linux__) && defined(R33_WITH_MO)
    return impl_->diagnostics();
#else
    failure(1001,0,"stereo.full_camera.diagnostics","Camera input unavailable in this build");
#endif
}
void FullCameraAdapter::close(){std::lock_guard<std::mutex> lock(impl_->mutex);const auto status=impl_->shutdown();if(!status.ok())throw r28::Failure(status);}
}
