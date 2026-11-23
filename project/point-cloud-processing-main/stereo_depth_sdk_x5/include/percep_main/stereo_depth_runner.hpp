#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#if defined(__GNUC__)
#define STEREO_DEPTH_API __attribute__((visibility("default")))
#else
#define STEREO_DEPTH_API
#endif

namespace perception {

struct StereoDepthConfig {
    std::string model_path;
    bool crop = false;
    bool rectified = false;
    int source_width = 640;
    int source_height = 544;
    int crop_height = 384;
};

struct StereoImageView {
    const std::uint8_t* data = nullptr;
    std::size_t bytes = 0;
    int width = 0;
    int height = 0;
    int stride = 0;
    bool nv12 = true; // false means tightly described I420 with U/V stride=stride/2.
    std::uint64_t frame_id = 0;
    std::uint64_t timestamp_ns = 0;
};

struct StereoIntrinsics {
    int width = 0;
    int height = 0;
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
    double bxf_px_mm = 0;
};

struct StereoDepthTiming {
    double pre_ms = 0;
    double bpu_submit_ms = 0;
    double bpu_wait_ms = 0;
    double bpu_submit_wait_ms = 0;
    double post_ms = 0;
    double total_ms = 0;
    double transform_ms = 0;
    double input_pack_ms = 0;
    double output_decode_ms = 0;
    double q8_restore_ms = 0;
    double lock_wait_ms = 0;
};

struct StereoDepthResult {
    int width = 0;
    int height = 0;
    int crop_y = 0;
    std::vector<std::uint16_t> q8; // Owned full-source grid; Q8/256 is disparity pixels.
    StereoDepthTiming timing;
};

// One persistent model per runner. init() and inferPair() serialize internally.
// The caller must stop/join all users before destruction. Both image buffers
// must remain alive and unchanged until the synchronous inferPair() returns.
// Nonzero IDs/timestamps must be present on both sides and match. Zero on both
// sides means unknown identity; it does not prove capture synchronization.
class STEREO_DEPTH_API StereoDepthRunner {
public:
    StereoDepthRunner();
    ~StereoDepthRunner();
    StereoDepthRunner(const StereoDepthRunner&) = delete;
    StereoDepthRunner& operator=(const StereoDepthRunner&) = delete;
    bool init(const StereoDepthConfig& config);
    bool inferPair(const StereoImageView& left, const StereoImageView& right,
                   const StereoIntrinsics& intrinsics, StereoDepthResult* result);
    std::string lastError() const;
    std::string contractJson() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace perception
