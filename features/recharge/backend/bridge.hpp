#pragma once

#include "v0-sdk/include/percep_demo/api_recharge_mark.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace r33::recharge {

struct CropHint {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int source = 1;  // 1: current YOLO, 2: retained successful crop.
};

struct Output {
    Aiper_Recharge::MarkPoseResult pose{};
    // Owns a copy of the original overlay. pose.overlay_bgr stays null so
    // copying Output never creates an owning or dangling raw pointer.
    std::vector<std::uint8_t> overlay_bgr;
};

// The selected original V0 SDK stores configuration, priors, and DockPose filters in
// process globals. Exactly one initialized Engine is allowed per process.
// Calls on the Engine are serialized; it uses the synchronous upstream path.
class Engine final {
public:
    Engine();
    ~Engine();
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(Engine&&) = delete;

    // Requires a readable recharge_mark.json (either root or nested object).
    bool initialize(const char* config_path, std::string& error);
    void getHostConfig(Aiper_Recharge::RechargeHostConfig& out) const;
    void resetSession();
    // Set already scaled frame intrinsics for intrinsics_source=sdk_scaled.
    // With dataset intrinsics, leave this unset to retain upstream behavior.
    bool setCameraMatrix(double fx, double fy, double cx, double cy,
                         std::string& error);

    // true means the frame was processed, including a legitimate no-detection
    // result (pose.ok/pose.pose_ok false). false means invalid input or runtime
    // failure. Input remains borrowed only until this synchronous call returns.
    bool processLeftNv12(const std::uint8_t* nv12, std::size_t bytes,
                         int width, int height, int stride_y,
                         std::uint64_t frame_id, std::int64_t timestamp_ns,
                         const CropHint* crop, Output& out, std::string& error);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace r33::recharge
