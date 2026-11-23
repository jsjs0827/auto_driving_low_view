#pragma once
#include "perception_full_sdk.h"
#include <cstdint>
#include <optional>
#include <memory>
#include <string>
#include <vector>

namespace perception::v0_host {
// Transport-neutral, owned copies of the producer's original descriptors.
// No V0 C++ layout or SHM implementation crosses the public SDK boundary.
struct Nv12Image {
    uint32_t width=0, height=0, stride=0;
    uint64_t frame_id=0, timestamp_ns=0;
    std::vector<uint8_t> bytes;
};
struct Q8Disparity {
    uint32_t width=0, height=0, stride_bytes=0;
    uint64_t frame_id=0, timestamp_ns=0;
    std::vector<uint16_t> samples;
};
struct CameraParameters {
    uint32_t width=0, height=0;
    double fx=0, fy=0, cx=0, cy=0, baseline_mm=0, bxf=0;
};
struct InputFrame {
    int command=21;
    uint64_t generation=0;
    Nv12Image left;
    std::optional<Nv12Image> right;
    std::optional<Q8Disparity> disparity;
    psdk_camera_intrinsics intrinsics{};
    std::shared_ptr<const void> owner;
    std::optional<psdk_full_frame> supplied;
    std::optional<psdk_frame> supplied_hires,supplied_right,supplied_rear;
    std::optional<psdk_disparity_q8> supplied_q8;
    std::optional<psdk_raw_stereo_input> supplied_raw;
    std::optional<psdk_imu_input> supplied_imu;
    std::optional<psdk_wheel_input> supplied_wheel;
    std::optional<psdk_radar_input> supplied_radar;
    mutable psdk_frame right_descriptor{};
    // The returned frame borrows this object's storage. Descriptor arguments
    // must remain alive until the synchronous Session::process returns.
    psdk_full_frame view(psdk_frame& hires, psdk_disparity_q8& q8) const;
};

// A single latest-frame slot. The controller serializes access to this class.
// Both inputs retain their producer identity; RGBD and GDC are never paired.
class BoardInput {
    int command_=21;
    uint64_t generation_=0, overwritten_=0;
    std::optional<CameraParameters> calibration_;
    std::optional<InputFrame> pending_;
    bool makeIntrinsics(uint32_t width,uint32_t height,
                        psdk_camera_intrinsics& out,std::string& error) const;
public:
    bool setIntrinsics(const CameraParameters& parameters,std::string& error);
    // Returns true only at a real mode boundary. Same-mode heartbeats preserve
    // both the generation and pending frame, just as Session preserves pose state.
    bool setCommand(int command);
    bool onRgbd(Nv12Image left,Q8Disparity q8,std::string& error);
    bool onStereo(Nv12Image left,Nv12Image right,std::string& error);
    bool onGdc(Nv12Image left,std::string& error);
    // Rich input binders retain their buffer owner here. Optional descriptor
    // wrappers are copied while valid and rebound after queue moves; their data
    // pointers and capture metadata pass unchanged to the common Session.
    bool onFullFrame(psdk_full_frame frame,std::shared_ptr<const void> owner,std::string& error);
    std::optional<InputFrame> take();
    bool hasPending() const { return pending_.has_value(); }
    int command() const { return command_; }
    uint64_t generation() const { return generation_; }
    uint64_t overwritten() const { return overwritten_; }
    void clear();
};
} // namespace perception::v0_host
