#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace perception::motion {
struct Roi { int x=0,y=0,width=0,height=0; }; // zero dimensions select the full source image
struct Extrinsics {
    // Camera coordinates -> body coordinates; body is x forward, y left, z up.
    std::array<double,9> rotation{{0,0,1,-1,0,0,0,-1,0}};
    std::array<double,3> translation_m{{0,0,0}}; // camera origin in body coordinates
    bool confirmed=false;
};
struct VisualSettings {
    Roi roi;
    unsigned patch_radius=2,search_radius=16,grid_step=12,max_features=300;
    unsigned min_matches=12,min_inliers=10,ransac_iterations=96;
    double min_texture=8,max_patch_error=24,match_ratio=.85;
    double min_disparity_px=.5,max_depth_m=20,inlier_threshold_m=.04,max_residual_m=.03;
    double min_inlier_ratio=.5,min_dt_ms=1,max_gap_ms=250;
    double max_translation_m=2,max_rotation_rad=.7;
    Extrinsics extrinsics;
};
struct ReferenceSettings {
    std::size_t max_samples=4096;
    double max_age_ms=10000,max_sample_gap_ms=100,max_abs_speed_mps=30,max_abs_yaw_rate_rps=10;
};
struct SlipSettings {
    double min_speed_mps=.08,min_reference_distance_m=.005,max_yaw_rad=.15,max_yaw_rate_rps=.6;
    double min_visual_quality=.5,enter_ratio=.35,exit_ratio=.2,enter_duration_ms=300,exit_duration_ms=500;
    double max_gap_ms=250;
};
struct MotionSample { std::uint64_t timestamp_ns=0; double forward_mps=0,yaw_rate_rps=0; };
struct VisualResult {
    bool available=false,body_confirmed=false;
    std::string reason="not_evaluated";
    std::uint64_t t0_ns=0,t1_ns=0;
    double dt_s=0,forward_m=0,lateral_m=0,yaw_rad=0,quality=0,residual_m=0;
    unsigned features=0,matches=0,inliers=0;
    // Static-scene correspondence: P_current = R * P_previous + t.
    std::array<double,9> scene_rotation{{1,0,0,0,1,0,0,0,1}};
    std::array<double,3> scene_translation_m{{0,0,0}};
    // Actual body pose at t1 expressed in body coordinates at t0 (inverse scene motion).
    std::array<double,9> body_rotation{{1,0,0,0,1,0,0,0,1}};
    std::array<double,3> body_translation_m{{0,0,0}};
};
struct ReferenceResult {
    bool available=false;
    std::string reason="not_evaluated";
    std::uint64_t t0_ns=0,t1_ns=0;
    double dt_s=0,forward_m=0,lateral_m=0,distance_m=0,yaw_rad=0;
    unsigned samples_used=0;
};
enum class SlipState { Unavailable,Normal,Suspected,Slipping,Recovering };
inline const char* stateName(SlipState s) { switch(s){case SlipState::Normal:return "normal";case SlipState::Suspected:return "suspected";case SlipState::Slipping:return "slipping";case SlipState::Recovering:return "recovering";default:return "unavailable";} }
struct SlipResult {
    bool available=false;
    SlipState state=SlipState::Unavailable;
    std::string reason="not_evaluated";
    std::uint64_t t0_ns=0,t1_ns=0;
    double slip_ratio=0,visual_speed_mps=0,reference_speed_mps=0,enter_elapsed_ms=0,exit_elapsed_ms=0;
};
} // namespace perception::motion
