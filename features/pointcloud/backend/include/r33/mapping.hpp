#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace r33 {

// The complete original start_config/extrinsics JSON is accepted. Paths are
// read once at construction; JSON content takes precedence when supplied.
struct MappingConfig {
  std::string start_config_path;
  std::string extrinsics_path;
  std::string start_config_json;
  std::string extrinsics_json;
  // Only explicit auto_cali mode writes here, after six successful frames.
  std::string calibration_output_path;
  int sample_step = 4;
  bool retain_clouds = false;
  bool render_maps = true;
  // Product graph selection. Defaults preserve the original complete chain.
  bool run_calibration = true, run_labels = true, run_transform = true;
  bool run_ground = true, run_maps = true;
};

struct MappingIntrinsics {
  float fx = 0, fy = 0, cx = 0, cy = 0, bxf_m = 0;
  // Preserve the original source scalar for V0's double /1000 then float cast.
  // Zero keeps compatibility with callers that only supply legacy bxf_m.
  double bxf_px_mm = 0;
};
struct MappingDetection {
  int class_id = -1;
  float confidence = 0, x1 = 0, y1 = 0, x2 = 0, y2 = 0;
};
struct MappingFrame {
  std::uint64_t frame_id = 0;
  int width = 0, height = 0;
  const std::uint16_t* disparity = nullptr;  // raw Q8 pixels
  std::size_t disparity_bytes = 0;
  int disparity_stride_samples = 0;
  MappingIntrinsics intrinsics;
  const std::uint8_t* rgb = nullptr;  // tightly packed same-frame RGB24
  std::size_t rgb_bytes = 0;
  bool semantic_result_valid = false;
  std::uint64_t semantic_frame_id = 0;
  std::vector<MappingDetection> detections;
  const std::uint8_t* segmentation_mask = nullptr;
  std::size_t segmentation_bytes = 0;
  int segmentation_width = 0, segmentation_height = 0;
  // Original PicData metadata; unknown/unavailable is zero, never estimated.
  float left_exposure_ms = 0, right_exposure_ms = 0;
  float left_analog_gain = 0, right_analog_gain = 0;
};

struct MappingPoint {
  float x = 0, y = 0, z = 0;
  int label = -1;
  float confidence = 0;
  std::uint8_t r = 0, g = 0, b = 0;
};
struct MappingCell {
  float min_h = 999, max_h = -999, avg_h = 0;
  int type = 0;
};
struct MappingRgbImage {
  int width = 0, height = 0;
  std::vector<std::uint8_t> pixels;
};
struct MappingCalibration {
  bool active = false, completed = false;
  int accumulated_frames = 0;
  double x = 0, y = 0, z = 0;
  double roll_degrees = 0, pitch_degrees = 0, yaw_degrees = 0;
};
struct MappingResult {
  std::uint64_t frame_id = 0;
  bool has_maps = false;
  std::uint16_t rows = 0, cols = 0, resolution_mm = 0;
  double width_m = 0, height_m = 0, resolution_m = 0;
  // Original indexing: row increases forward (+robot x), column to the right
  // (-robot y). RGB render reverses rows, exactly as Visualize::saveMap.
  std::vector<MappingCell> pcl_map, ai_map, fusion_map;
  std::vector<std::int16_t> avg_h_cm, min_h_cm, max_h_cm;
  std::vector<std::uint8_t> type;
  MappingRgbImage pcl_rgb, ai_rgb, fusion_rgb;
  bool nearest_obstacle_valid = false;
  int nearest_obstacle_index = -1;
  float nearest_obstacle_x_m = 0, nearest_obstacle_y_m = 0;
  float nearest_obstacle_distance_m = 999;
  MappingCalibration calibration;
  std::vector<MappingPoint> camera_cloud, robot_cloud;
  std::vector<MappingPoint> ground_cloud, obstacle_cloud, semantic_cloud;
  std::size_t camera_points = 0, ground_points = 0, obstacle_points = 0;
  std::size_t semantic_points = 0;
};

// Per-session state: Patchwork temporal adaptation and calibration accumulator
// persist; per-frame clouds and semantic outputs are cleared by beginFrame.
// Stages must be invoked in this order and perform actual original operations.
class MappingPipeline {
 public:
  explicit MappingPipeline(const MappingConfig& config);
  ~MappingPipeline();
  MappingPipeline(const MappingPipeline&) = delete;
  MappingPipeline& operator=(const MappingPipeline&) = delete;
  void beginFrame(const MappingFrame& frame);
  // Same-frame semantic results may arrive after geometry/cloud construction.
  // Copies detections and mask; no borrowed semantic buffers survive the call.
  // Must run after beginFrame and before labels, on the mapping stage chain.
  // Geometry/disparity/RGB fields are not consumed, except width/height/ID.
  void setSemantic(const MappingFrame& frame);
  void cloud();
  void calibration();
  void labels();
  void transform();
  void ground();
  void maps();
  std::shared_ptr<const MappingResult> result() const;
  std::string effectiveConfigJson() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace r33
