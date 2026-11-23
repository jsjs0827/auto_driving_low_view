#ifndef AI_NODE_PERCEP_DEMO_PERCEP_DEMO_CONFIG_HPP_
#define AI_NODE_PERCEP_DEMO_PERCEP_DEMO_CONFIG_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace ai_node_percep_demo {

// Legacy camera dimensions used by offline playback and buffer reservation.
// Online processing always uses the validated dimensions carried by each frame.
inline constexpr int rgb_width = 640;
inline constexpr int rgb_height = 544;
inline constexpr int gdc_width = 1280;
inline constexpr int gdc_height = 1088;

inline constexpr const char* kRechargeMarkPoseTopic =
    "/percep_demo/recharge_mark_pose";
inline constexpr const char* kRechargeMarkOverlayTopic =
    "/percep_demo/recharge_mark_overlay";

struct PatchworkConfig {
    bool enabled{false};
    bool verbose{false};
    bool publish_ground{true};
    bool publish_nonground{true};
    double sensor_height{0.23};
    double min_range{0.15};
    double max_range{3.0};
    double th_dist{0.05};
    double th_seeds{0.05};
    int log_every_n{10};
};

struct ImagePubConfig {
    bool resize_enabled{false};
    std::uint32_t width{640};
    std::uint32_t height{480};
};

struct PointCloudColorConfig {
    bool enabled{true};
    // true: use the rectified GDC image; false: use the RGBD left image.
    bool use_gdc_image{true};
    bool height_color_fallback{false};
};

struct PointCloudPubConfig {
    std::size_t max_points{50000};
    float box_min_x{-3.0F};
    float box_max_x{3.0F};
    float box_min_y{-3.0F};
    float box_max_y{3.0F};
    float box_min_z{0.1F};
    float box_max_z{3.0F};
};

// Host-side policy for using the detector to restrict recharge-code search.
struct AiRechargeConfig {
    bool enabled{false};
    std::string charger_class{"charger"};
    float score_min{0.25F};
    float pad_ratio{0.08F};
    int pad_px{16};
    int min_box_w{40};
    int min_box_h{40};
};

struct DetMatchConfig {
    bool enabled{true};
    float score_min{0.25F};
    float depth_span_m{0.80F};
    float max_depth_m{3.0F};
};

struct LabelFusionConfig {
    bool enabled{false};
    bool fuse_into_grid{false};
    std::string passable_classes{"grass"};
    bool nongrass_seg_as_obstacle{false};
    float det_score_min{0.30F};
    float max_range_m{3.0F};
    float max_height_m{1.0F};
    float det_depth_span_m{0.60F};
    float det_depth_gap_m{0.25F};
    int det_depth_min_pts{5};
    float keep_below_cam_m{0.0F};
    bool cluster_filter{true};
    float cluster_voxel_m{0.05F};
    int cluster_min_component_pts{20};
    bool seg_mask_postprocess{false};
    int seg_mask_majority_radius{1};
    int seg_mask_close_radius{1};
    int seg_mask_close_iterations{1};
    bool seg_cluster_filter{true};
    int seg_mask_erode_px{1};
    float seg_depth_gap_m{0.20F};
    int seg_ray_bin_px{2};
    int seg_min_component_pts{15};
};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_PERCEP_DEMO_CONFIG_HPP_
