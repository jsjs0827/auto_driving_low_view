#include "perception_ros_so_demo/mo_camera.hpp"
#include <MoLocalSdkInterface.h>
#include <chrono>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <optional>
#include <thread>

namespace ros_so_demo::raw {
namespace {
using Clock=std::chrono::steady_clock;
void checked(int code,const char* call) {
    if (code) throw FatalCameraError(std::string(call)+" failed: "+std::to_string(code));
}
}
struct Camera::Impl {
    CameraOptions options;
    void* library=nullptr;
    int handle=-1, original_mode=-1;
    decltype(&moSetConfigPath) set_config=nullptr;
    decltype(&moOpenCamera) open_camera=nullptr;
    decltype(&moCloseCamera) close_camera=nullptr;
    decltype(&moGetOneRGBDFrame) get_rgbd=nullptr;
    decltype(&moReleaseRGBDFrame) release_rgbd=nullptr;
    decltype(&moGetOneGdcFrame) get_gdc=nullptr;
    decltype(&moReleaseGdcFrame) release_gdc=nullptr;
    decltype(&moGetDisparityCalculationMode) get_mode=nullptr;
    decltype(&moSetDisparityCalculationMode) set_mode=nullptr;
    template<class F> F symbol(const char* name) {
        dlerror(); auto* address=dlsym(library,name); const char* error=dlerror();
        if (error || !address) throw std::runtime_error(std::string("Missing camera SDK symbol: ")+name);
        F function=nullptr; static_assert(sizeof(function)==sizeof(address));
        std::memcpy(&function,&address,sizeof(function)); return function;
    }
    void open() {
        library=dlopen(options.sdk.c_str(),RTLD_NOW|RTLD_GLOBAL);
        if (!library) throw std::runtime_error(std::string("Cannot load camera SDK: ")+dlerror());
        set_config=symbol<decltype(set_config)>("moSetConfigPath");
        open_camera=symbol<decltype(open_camera)>("moOpenCamera");
        close_camera=symbol<decltype(close_camera)>("moCloseCamera");
        get_rgbd=symbol<decltype(get_rgbd)>("moGetOneRGBDFrame");
        release_rgbd=symbol<decltype(release_rgbd)>("moReleaseRGBDFrame");
        get_mode=symbol<decltype(get_mode)>("moGetDisparityCalculationMode");
        set_mode=symbol<decltype(set_mode)>("moSetDisparityCalculationMode");
        if (options.need_gdc) {
            get_gdc=symbol<decltype(get_gdc)>("moGetOneGdcFrame");
            release_gdc=symbol<decltype(release_gdc)>("moReleaseGdcFrame");
        }
        checked(set_config(options.config.c_str()),"moSetConfigPath");
        checked(open_camera(&handle),"moOpenCamera");
        if (handle<0) throw std::runtime_error("moOpenCamera returned an invalid handle");
        checked(get_mode(handle,&original_mode),"moGetDisparityCalculationMode");
        if (original_mode<0) throw std::runtime_error("SDK did not report its original disparity mode");
        checked(set_mode(handle,options.disparity_mode),"moSetDisparityCalculationMode");
        int mode=-1; checked(get_mode(handle,&mode),"disparity mode readback");
        if (mode!=options.disparity_mode) throw std::runtime_error("SDK disparity mode readback differs from requested mode");
    }
    void close() {
        int restore=0;
        if (handle>=0) {
            if (original_mode>=0 && set_mode) restore=set_mode(handle,original_mode);
            const int code=close_camera ? close_camera(handle) : -999;
            handle=-1;
            if (code) {
                // Keep vendor code mapped if a failed close leaves SDK threads alive.
                library=nullptr;
                checked(code,"moCloseCamera (SDK kept mapped)");
            }
        }
        if (library) { auto* owned=library; library=nullptr; checked(dlclose(owned),"dlclose"); }
        checked(restore,"restore original disparity mode");
    }
    bool rgbd(Capture& result,int& last_error) {
        MoFrame left{},right{},disp{};
        const int code=get_rgbd(handle,&left,&right,&disp);
        if (code) {
            last_error=code;
            if (left.Data||right.Data||disp.Data)
                checked(release_rgbd(handle,&left,&right,&disp),"release partial RGBD");
            return false;
        }
        try { result=raw::rgbd(left,right,disp,options.need_q8); }
        catch (...) { checked(release_rgbd(handle,&left,&right,&disp),"release rejected RGBD"); throw; }
        checked(release_rgbd(handle,&left,&right,&disp),"moReleaseRGBDFrame"); return true;
    }
    CameraParameters parameters() {
        if(handle<0)throw std::runtime_error("Camera is closed");
        const auto get=symbol<decltype(&moGetCameraParams)>("moGetCameraParams");
        CameraParameters p;
        p.source_code=get(handle,&p.rectified,&p.bxf,&p.baseline,&p.width,&p.height,&p.max_disparity);
        // Vendor contract: 0 = module parameters; 1 = X5 configuration fallback.
        if(p.source_code!=0 && p.source_code!=1)
            throw std::runtime_error("moGetCameraParams failed: "+std::to_string(p.source_code));
        const auto& k=p.rectified;
        if(p.width<1||p.height<1||p.width>4096||p.height>4096||
            !std::isfinite(k.fx)||!std::isfinite(k.fy)||!std::isfinite(k.cx)||!std::isfinite(k.cy)||
            !std::isfinite(p.bxf)||!std::isfinite(p.baseline)||k.fx<=0||k.fy<=0||p.bxf<=0||p.baseline<=0||
            k.cx<0||k.cy<0||k.cx>=p.width||k.cy>=p.height)
            throw std::runtime_error("moGetCameraParams returned invalid rectified reference intrinsics");
        for(float value:k.radial_distortion_k)if(!std::isfinite(value)||std::abs(value)>1e-6f)
            throw std::runtime_error("SDK reference intrinsics have nonzero radial distortion; rectified RGBD geometry is required");
        for(float value:k.tangential_distortion_p)if(!std::isfinite(value)||std::abs(value)>1e-6f)
            throw std::runtime_error("SDK reference intrinsics have nonzero tangential distortion; rectified RGBD geometry is required");
        return p;
    }
    bool gdc(Plane& result,int& last_error) {
        MoFrame left{},right{}; const int code=get_gdc(handle,&left,&right);
        if (code) {
            last_error=code;
            if (left.Data||right.Data) checked(release_gdc(handle,&left,&right),"release partial GDC");
            return false;
        }
        try { result=raw::gdc(left,right); }
        catch (...) { checked(release_gdc(handle,&left,&right),"release rejected GDC"); throw; }
        checked(release_gdc(handle,&left,&right),"moReleaseGdcFrame"); return true;
    }
    Capture next(const std::function<bool()>& keep_running) {
        if (handle<0) throw std::runtime_error("Camera is closed");
        const auto deadline=Clock::now()+std::chrono::milliseconds(options.timeout_ms);
        std::optional<Capture> current; std::optional<Plane> high; int last_error=0;
        while (keep_running() && Clock::now()<deadline) {
            if (!current) { Capture frame; if (rgbd(frame,last_error)) current=std::move(frame); }
            if (current && !options.need_gdc) return std::move(*current);
            if (options.need_gdc && !high) { Plane frame; if (gdc(frame,last_error)) high=std::move(frame); }
            if (current && high) {
                const int order=pairOrder(current->left.capture,high->capture);
                if (!order) { current->gdc=std::move(*high); return std::move(*current); }
                if (order<0) current.reset(); else high.reset();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(options.retry_ms));
        }
        throw std::runtime_error("Camera acquisition interrupted or timed out; last SDK code="+
            std::to_string(last_error)+"; raw RGBD/GDC require exact capture ID and timestamp");
    }
};
Camera::Camera(const CameraOptions& options):impl_(std::make_unique<Impl>()) {
    if (options.sdk.empty() || options.config.empty() || options.timeout_ms<1 ||
        options.retry_ms<1 || options.retry_ms>options.timeout_ms ||
        (options.disparity_mode!=1&&options.disparity_mode!=2) ||
        (options.need_q8&&options.disparity_mode!=2))
        throw std::invalid_argument("Invalid camera SDK path, config or timeout");
    impl_->options=options;
    try { impl_->open(); }
    catch (...) { try { impl_->close(); } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; } throw; }
}
Camera::~Camera() { try { close(); } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; } }
Capture Camera::next(const std::function<bool()>& keep_running) { return impl_->next(keep_running); }
CameraParameters Camera::parameters() { return impl_->parameters(); }
void Camera::close() { impl_->close(); }
}
