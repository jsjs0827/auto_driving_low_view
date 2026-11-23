#pragma once
#include "raw_capture.hpp"
#include <functional>
#include <memory>
#include <string>

namespace ros_so_demo::raw {
struct FatalCameraError:std::runtime_error { using std::runtime_error::runtime_error; };
struct CameraOptions {
    std::string sdk="/app/data/metoak/lib/libMoGeneralSDK.so";
    std::string config="/app/data/metoak/demo/config/x5_camera.cfg";
    bool need_q8=false, need_gdc=false;
    // Vendor mode 1 disables its internal disparity producer; mode 2 computes Q8.
    // Stereo callers can request mode 1 and consume only the rectified image pair.
    int disparity_mode=2;
    int timeout_ms=3000, retry_ms=5;
};
// Parameters of the SDK reference image after rectification, never raw lens
// calibration. Width/height describe that reference image, not an assumed RGBD
// resolution. The caller must retain the reference geometry when scaling.
struct CameraParameters {
    MoMonoCameraParams rectified{};
    double bxf=0, baseline=0;
    int width=0, height=0, max_disparity=0, source_code=-1;
};
class Camera {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    explicit Camera(const CameraOptions& options);
    ~Camera();
    Camera(const Camera&)=delete;
    Camera& operator=(const Camera&)=delete;
    // All payloads are owned copies; no vendor buffers survive this call.
    Capture next(const std::function<bool()>& keep_running);
    CameraParameters parameters();
    void close();
};
}
