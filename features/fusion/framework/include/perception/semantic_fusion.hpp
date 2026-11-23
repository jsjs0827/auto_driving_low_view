#pragma once

#include "perception/types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace perception::fusion {

// Product semantic-label domains must not overlap even when a detector grows
// past the historical 100-class boundary.
inline constexpr int kSegmentationClassBase = 1024;

enum class DisparityType { UInt16, Float32 };
struct DisparityView {
    const void *data = nullptr;
    std::size_t bytes = 0;
    std::size_t stride_bytes = 0;
    int width = 0, height = 0;
    DisparityType type = DisparityType::UInt16;
    float scale = 256.0f; // X5 Q8.8 by default; floating input normally uses 1.
};

struct StereoCalibration {
    float fx = 0, fy = 0, cx = 0, cy = 0;
    float baseline_m = 0;
    int calibration_width = 0, calibration_height = 0;
    void validate() const;
};

struct PointXYZRGB {
    float x = 0, y = 0, z = 0;
    std::uint8_t r = 0, g = 0, b = 0;
    int image_u = -1, image_v = -1; // Retain original disparity pixel when available.
};
using PointCloud = std::vector<PointXYZRGB>;

enum class CoordinateFrame { CameraOptical, BaseLink, PercepMain };
struct FramedPointCloud {
    CoordinateFrame frame = CoordinateFrame::CameraOptical;
    PointCloud points;
};

struct CloudGenerationConfig {
    float min_disparity = 0.01f;
    float min_depth_m = 0.05f, max_depth_m = 20.0f;
    int sample_step = 1;
    std::size_t max_points = 0; // Zero means unlimited.
};

PointCloud disparityToPointCloud(const DisparityView &, const ImageView &,
                                 const StereoCalibration &, const CloudGenerationConfig & = {});
PointCloud projectPointCloudToImage(const PointCloud &, const StereoCalibration &, int image_width,
                                    int image_height);

enum class SemanticKind : std::uint8_t {
    Unknown = 0,
    Passable = 1,
    DetectionObstacle = 2,
    SegmentationObstacle = 3,
    // Used by fused maps for product type 1 originating from the geometric
    // layer. It is distinct from AI-map type 1 (non-grass segmentation).
    Ground = 4
};

struct MatchRecord {
    int detection_index = -1;
    int segmentation_class = -1;
    float detection_score = 0;
    bool projection_valid = false;
    bool segmentation_stable = false;
    bool detection_depth_valid = false;
    bool detection_component_valid = false;
    bool segmentation_component_valid = true;
    // The voxel-population stage records its decision separately from the
    // connected-component stage. This makes the two registered features
    // independently testable and prevents the component stage from applying
    // the voxel population threshold a second time.
    bool detection_voxel_valid = false;
    bool segmentation_voxel_valid = true;
};

struct MatchConfig {
    float detection_score_min = 0.25f;
    // Empty accepts every detector class. Otherwise only listed product classes
    // participate in 2D/3D matching and all later detection-based filtering.
    std::vector<int> detection_include_ids;
    std::vector<int> passable_detection_ids;
    std::vector<int> passable_segmentation_ids;
    int segmentation_boundary_radius = 1;
};

std::vector<MatchRecord> matchDetectionsAndMask(const PointCloud &,
                                                const std::vector<Detection> &,
                                                const SegmentationResult &, int image_width,
                                                int image_height, const MatchConfig &);

struct DepthFilterConfig {
    float cluster_gap_m = 0.20f;
    float cluster_span_m = 1.20f;
    float margin_m = 0.03f;
    int minimum_points = 4;
};
void filterDetectionDepth(const PointCloud &, const std::vector<Detection> &,
                          const MatchConfig &, const DepthFilterConfig &,
                          std::vector<MatchRecord> &);

struct VoxelMinimumFilterConfig {
    float voxel_size_m = 0.05f;
    int minimum_points_per_voxel = 1;
    bool filter_segmentation = true;
};

struct ConnectedComponentFilterConfig {
    int minimum_component_points = 3;
    bool filter_segmentation = true;
};

struct VoxelCoordinate {
    std::int64_t x = 0, y = 0, z = 0;
    bool valid = false;
    bool operator==(const VoxelCoordinate &other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Intermediate artifact shared by the two filtering features. Coordinates are
// calculated once by the voxel stage and consumed by the component stage.
struct VoxelFilterResult {
    std::vector<MatchRecord> matches;
    std::vector<VoxelCoordinate> coordinates;
    bool filter_segmentation = true;
};

VoxelFilterResult filterVoxelMinimumPoints(const PointCloud &,
                                           const std::vector<Detection> &,
                                           const VoxelMinimumFilterConfig &,
                                           std::vector<MatchRecord>);
std::vector<MatchRecord> filterConnectedComponents(
    const PointCloud &, const std::vector<Detection> &,
    const ConnectedComponentFilterConfig &, const VoxelFilterResult &);

// Source-compatible legacy combined API. New task graphs should use the two
// functions above through fusion.voxel_filter and
// fusion.connected_component_filter.
struct VoxelFilterConfig {
    float voxel_size_m = 0.05f;
    int minimum_points_per_voxel = 1;
    int minimum_component_points = 3;
    bool filter_segmentation = true;
};
void filterVoxelComponents(const PointCloud &, const std::vector<Detection> &,
                           const VoxelFilterConfig &, std::vector<MatchRecord> &);

struct SemanticConfig {
    float min_depth_m = 0.05f, max_depth_m = 20.0f;
    float min_y_m = -10.0f, max_y_m = 10.0f;
    // Keep raw segmentation class IDs even when product policy does not treat
    // non-passable segmentation as an obstacle.
    bool nongrass_seg_as_obstacle = false;
    MatchConfig matching;
    std::vector<std::array<std::uint8_t, 3>> detection_colors;
    std::array<std::uint8_t, 3> passable_color{0, 200, 0};
    std::array<std::uint8_t, 3> segmentation_obstacle_color{240, 140, 0};
    std::array<std::uint8_t, 3> unknown_color{80, 80, 80};
};

struct SemanticPoint : PointXYZRGB {
    SemanticKind semantic = SemanticKind::Unknown;
    int class_id = -1;
    float confidence = 0;
};
using SemanticCloud = std::vector<SemanticPoint>;

SemanticCloud generateSemanticCloud(const PointCloud &, const std::vector<Detection> &,
                                    const std::vector<MatchRecord> &, const SemanticConfig &);

struct RigidTransform {
    std::array<float, 16> matrix{1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1}; // Row-major.
    CoordinateFrame source = CoordinateFrame::CameraOptical;
    CoordinateFrame target = CoordinateFrame::BaseLink;
    void validate() const;
};
struct FramedSemanticCloud {
    CoordinateFrame frame = CoordinateFrame::CameraOptical;
    SemanticCloud points;
};
FramedSemanticCloud transformSemanticCloud(const FramedSemanticCloud &, const RigidTransform &);

struct Marker {
    std::uint64_t id = 0;
    float x = 0, y = 0, z = 0, scale = 0.05f;
    std::uint8_t r = 0, g = 0, b = 0, a = 255;
    SemanticKind semantic = SemanticKind::Unknown;
    int class_id = -1;
    std::string text;
};
struct Publication {
    FrameInfo frame;
    CoordinateFrame coordinate_frame = CoordinateFrame::CameraOptical;
    // The normal include-all path aliases the immutable input cloud instead of
    // copying every point. A filtered publication owns a compact replacement.
    std::shared_ptr<const SemanticCloud> cloud;
    std::vector<Marker> markers;
};
Publication makePublication(const FramedSemanticCloud &, FrameInfo, bool include_unknown,
                            bool make_markers);
Publication makePublication(std::shared_ptr<const FramedSemanticCloud>, FrameInfo,
                            bool include_unknown, bool make_markers);

struct GridConfig {
    int rows = 0, cols = 0;
    float resolution_m = 0.05f;
    float origin_x_m = 0, origin_y_m = 0;
    void validate() const;
};
struct GridCell {
    SemanticKind semantic = SemanticKind::Unknown;
    // Product-compatible map contract: 0 unknown, 2 generic obstacle,
    // 3 grass/passable, 4 person, 5 cat, 6 dog, 7 charger, 8 toy. Type 1 is
    // layer-dependent: AI map non-grass, fused/geometric map ground. The
    // semantic field disambiguates those two meanings.
    int type = 0;
    float min_height = 0, max_height = 0, average_height = 0;
    std::uint32_t point_count = 0;
};
struct GridMap {
    // CameraOptical is the fail-closed default for an unframed/external map;
    // map projection/fusion requires an explicit robot/world frame.
    CoordinateFrame frame = CoordinateFrame::CameraOptical;
    GridConfig config;
    std::vector<GridCell> cells;
};
struct AiGridPolicy {
    bool nongrass_seg_as_obstacle = false;
    int person_detection_id = 8;
    int cat_detection_id = 9;
    int dog_detection_id = 10;
    int charger_detection_id = 18;
    // detection_x5.cfg: toy=16, tool=17. Keep this default aligned with the
    // shipped 24-class model instead of accidentally classifying tools as toys.
    int toy_detection_id = 16;
};
GridMap projectToAiMap(const FramedSemanticCloud &, const GridConfig &,
                       const AiGridPolicy & = {});
GridMap fuseMaps(const GridMap &ai_map, const GridMap &geometric_map);

struct AiFusionMaps {
    GridMap ai_map;
    GridMap fusion_map;
};

// Portable boundary for the product AiFusion implementation. An integration
// adapter converts these portable clouds to its PCL types, calls
// initData(ground, obstacle, semantic)+integrate(), then converts both maps
// back. The r12 core intentionally does not link ROS/PCL/product binaries.
class AiFusionAdapter {
  public:
    virtual ~AiFusionAdapter() = default;
    virtual AiFusionMaps integrate(const FramedPointCloud &ground_cloud,
                                   const FramedPointCloud &obstacle_cloud,
                                   const FramedSemanticCloud &semantic_cloud,
                                   const GridConfig &, const AiGridPolicy &) const = 0;
};

} // namespace perception::fusion
