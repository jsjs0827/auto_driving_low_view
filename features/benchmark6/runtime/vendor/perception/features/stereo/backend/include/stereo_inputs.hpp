#pragma once
// Internal C++ adapter, not part of the public C ABI. The external camera SDK
// produces disparity; this module validates and owns its output, never invents it.
#include "camera_source.hpp"
#include "perception/semantic_fusion.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace r33::stereo {
struct Intrinsics {
    int width=640,height=544;
    float fx=0,fy=0,cx=0,cy=0;
    double bxf=0; // Original product unit: focal_length * baseline in mm.
};
struct Q8View {
    const void* data=nullptr;
    std::size_t bytes=0,stride_bytes=0;
    int width=0,height=0;
    perception::FrameInfo frame;
};
// Fail with 2001 for invalid calibration, 1002 for invalid/mismatched frame
// buffers. Zero is a valid frame ID; timestamp zero means unverified and fails.
void validateIntrinsics(const Intrinsics&);
void validateDisparity(const perception::ImageView&,perception::FrameInfo,
                       const Q8View&,const Intrinsics&);
// Optional original GDC-left source for point-cloud colors/recharge. It must be
// supplied by the application that owns the camera handle, not guessed from a
// second SDK instance. Same-frame high-resolution NV12 is 1280x1088.
void validateHires(const perception::ImageView&,perception::FrameInfo,
                   perception::FrameInfo rgbd_frame);
std::shared_ptr<const std::vector<std::uint16_t>> copyDisparity(
    const perception::ImageView&,perception::FrameInfo,const Q8View&,const Intrinsics&);
perception::fusion::DisparityView frameworkView(const Q8View&);

struct RightMetadata {
    perception::FrameInfo frame;
    int width=0,height=0;
    std::size_t bytes=0;
    bool audited=false;
    // The preserved Source does not copy right pixels. Never imply otherwise.
    bool pixels_available=false;
};
struct Frame {
    perception::FrameInfo info;
    std::shared_ptr<const example::Image> left;
    std::shared_ptr<const std::vector<std::uint16_t>> disparity;
    std::shared_ptr<const example::Image> hires_left;
    perception::FrameInfo hires_info;
    RightMetadata right;
    demo_camera::AcquisitionTiming acquisition;
    Q8View disparityView() const;
};
// Strict conversion of the preserved Source's owned RGBD frame. CameraFrame
// metadata must prove matching left/right/disparity IDs, timestamps,640x544.
Frame fromCameraFrame(const demo_camera::Frame&,const Intrinsics&);

struct CameraOptions {
    std::string sdk_path="/app/data/metoak/lib/libMoGeneralSDK.so";
    std::string config_path="/app/data/metoak/demo/config/x5_camera.cfg";
    int acquire_timeout_ms=3000,retry_interval_ms=5;
};
class CameraAdapter {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    CameraAdapter(const CameraOptions&,const Intrinsics&);
    // Explicit injected Source supports contract tests/integration adapters;
    // production camera creation uses the real vendor API above.
    CameraAdapter(std::shared_ptr<demo_camera::Source>,const Intrinsics&);
    ~CameraAdapter();
    CameraAdapter(const CameraAdapter&)=delete;
    CameraAdapter& operator=(const CameraAdapter&)=delete;
    Frame next();
    demo_camera::Diagnostics diagnostics();
    // Idempotent. Real SDK close/restore/dlclose failure retains its native code.
    // Caller must not race close/destruction against another method.
    void close();
};
// Optional one-handle camera owner for complete RGBD + GDC delivery. Uses the
// real vendor headers/functions on Linux/X5; unsupported builds fail explicitly.
// All SDK planes are copied and released before next() returns. No /3 ID patch,
// no approximate timestamp pairing and no second camera open.
class FullCameraAdapter {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    FullCameraAdapter(const CameraOptions&,const Intrinsics&,bool need_hires=true);
    ~FullCameraAdapter();
    FullCameraAdapter(const FullCameraAdapter&)=delete;
    FullCameraAdapter& operator=(const FullCameraAdapter&)=delete;
    Frame next();
    demo_camera::Diagnostics diagnostics();
    void close();
};
}
