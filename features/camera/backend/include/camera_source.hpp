#pragma once

// Product-input adapter for the native example.  This layer deliberately lives
// outside libperception_inference: Detector/Segmenter only see ImageView.

#include "image_input.hpp"
#include "perception/types.hpp"

#include <memory>
#include <functional>
#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

namespace demo_camera {

struct Options {
    // Path to the vendor runtime loaded with dlopen on Linux/X5.
    std::string sdk_path = "/opt/metoak/lib/libMoGeneralSDK.so";
    // Optional directory/file accepted by moSetConfigPath.  Empty means that
    // the vendor runtime keeps its own default.
    std::string config_path = "/opt/metoak/demo/config/x5_camera.cfg";
    // rgbd: 640x544 left frame from moGetOneRGBDFrame.
    // gdc:  1280x1088 rectified left frame from moGetOneGdcFrame.
    std::string stream = "rgbd";
    // "YUV420P" is ambiguous across vendor documents.  The current product
    // configuration explicitly uses NV12; keep this field explicit so another
    // firmware can select I420 without content-based guessing.
    std::string format = "nv12";
    int acquire_timeout_ms = 3000;
    int retry_interval_ms = 5;
    int disparity_mode = 0; // 0 untouched; 1 on-demand (GDC or explicit RGBD probe); 2 continuous
    bool stereo_trace = false;
    bool demand_enabled = false;
    // Explicit r27 capability probe: mode 1 may deliver the original RGBD
    // color pair while caching that upstream frame for on-demand disparity.
    // Static SDK evidence supports this path; board capability must be tested.
    bool allow_rgbd_mode1_probe = false;
    // r27: inspect the RGBD triplet's IDs/timestamps/layout without copying or
    // reading disparity pixels. This checks SDK metadata, not physical sync.
    // Reject contradictory values when opted in. Missing SDK metadata remains
    // unverified (valid=false with issue codes) without discarding FPS data.
    // Kept opt-in so older camera integrations retain their existing behavior.
    bool audit_rgbd_triplet = false;
    // Output/fusion opt-in only. Copies the already returned RGBD disparity
    // before SDK release; never requests another BPU calculation or mode.
    // Requires complete matching IDs/timestamps and tight 640x544 uint16 data.
    bool copy_disparity = false;
    bool restore_mode_before_close = true; // r23 compatibility; false is explicit close-policy control
    std::function<void(const char*,int,double)> lifecycle; // boundary calls only; no per-frame tracing

};

struct AcquisitionTiming {
    double sdk_get_ms=0, copy_ms=0, release_ms=0, retry_sleep_ms=0;
    int attempts=0, last_error=0;
    bool sdk_timestamp=false;
};
struct StereoEvent { std::int64_t host_ns=0; int status=0; };
struct Diagnostics {
    int requested_mode=0, original_mode=-1, effective_mode=-1, bpu_ratio=-1;
    double original_fps=-1, disparity_fps=-1;
    bool ratio_available=false, original_fps_available=false, disparity_fps_available=false;
    bool mode_available=false;
    bool callback_available=false;
    std::uint64_t stereo_starts=0, stereo_ends=0, stereo_dropped_events=0;
    double stereo_busy_ms=0;
};

struct Frame {
    perception::FrameInfo info;
    std::shared_ptr<const example::Image> image;
    std::shared_ptr<const std::vector<std::uint16_t>> disparity_pixels;
    AcquisitionTiming acquisition;
    unsigned sdk_stream_type=0,sdk_bits_per_pixel=0,sdk_bytes_used=0;
    int sdk_rgb_image_type=0;
    bool sdk_rgbd_triplet_available=false, sdk_rgbd_triplet_valid=false;
    std::string sdk_rgbd_triplet_issue;
    bool sdk_color_pair_valid=false;
    std::string sdk_color_pair_issue;
    std::uint64_t sdk_right_frame_id=0, sdk_right_timestamp_ns=0;
    int sdk_right_width=0, sdk_right_height=0;
    unsigned sdk_right_bytes_used=0;
    std::uint64_t sdk_disparity_frame_id=0, sdk_disparity_timestamp_ns=0;
    int sdk_disparity_width=0, sdk_disparity_height=0;
    unsigned sdk_disparity_bytes_used=0;
    std::int64_t received_host_ns=0;
};

struct DisparityResult {
    std::uint64_t frame_id=0;
    int code=-1, width=0, height=0;
    double call_ms=0, copy_ms=0;
    std::int64_t call_begin_host_ns=0, call_end_host_ns=0;
    std::vector<std::uint16_t> pixels;
};

class Source {
  public:
    virtual ~Source() = default;
    Source(const Source &) = delete;
    Source &operator=(const Source &) = delete;

    // The vendor frame id is preserved exactly, including zero. Returned image
    // memory owns a copy and remains valid after the SDK frame is released.
    virtual Frame next() = 0;
    virtual const char *name() const noexcept = 0;
    virtual Diagnostics diagnostics(bool sample_sdk=false) { (void)sample_sdk; return {}; }
    virtual std::vector<StereoEvent> stereoEvents() { return {}; }
    virtual DisparityResult calculateDisparity(std::uint64_t) {
        throw std::runtime_error("UNSUPPORTED: on-demand disparity API unavailable");
    }

  protected:
    Source() = default;
};

// Opens the camera during construction and closes it during destruction.
// Throws a typed PerceptionException when the platform, shared library, ABI,
// camera open, acquisition, frame layout or release operation is invalid.
std::shared_ptr<Source> open(const Options &);

} // namespace demo_camera
