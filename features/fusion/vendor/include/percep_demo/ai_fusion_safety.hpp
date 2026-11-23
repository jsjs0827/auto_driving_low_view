#ifndef AI_NODE_PERCEP_DEMO_AI_FUSION_SAFETY_HPP_
#define AI_NODE_PERCEP_DEMO_AI_FUSION_SAFETY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "percep_demo/semantic_point_label.hpp"

namespace ai_node_percep_demo {
namespace detail {

// A bounded grid protects the embedded target from accidental multi-gigabyte
// allocations. The current fusion stage owns several arrays per cell, so the
// limit applies to cells rather than to any single vector's byte count.
inline constexpr std::size_t kMaxFusionGridCells = 1'000'000U;
inline constexpr std::size_t kMaxVisualizationPixels = 16'000'000U;

struct GridGeometry {
  std::uint16_t rows{0};
  std::uint16_t cols{0};
  std::uint16_t resolution_mm{0};
  double resolution_m{0.0};
  std::size_t cell_count{0};
};

inline std::optional<GridGeometry> TryMakeGridGeometry(
    double width_m, double height_m, double resolution_m,
    std::size_t max_cells = kMaxFusionGridCells) noexcept {
  if (!std::isfinite(width_m) || !std::isfinite(height_m) ||
      !std::isfinite(resolution_m) || width_m <= 0.0 || height_m <= 0.0 ||
      resolution_m <= 0.0 || max_cells == 0U) {
    return std::nullopt;
  }

  const double resolution_mm_value = resolution_m * 1000.0;
  const double rounded_resolution_mm = std::round(resolution_mm_value);
  constexpr double kUint16Max =
      static_cast<double>(std::numeric_limits<std::uint16_t>::max());
  constexpr double kIntegerMillimeterTolerance = 1.0e-6;
  if (!std::isfinite(resolution_mm_value) ||
      !std::isfinite(rounded_resolution_mm) || rounded_resolution_mm < 1.0 ||
      rounded_resolution_mm > kUint16Max ||
      std::fabs(resolution_mm_value - rounded_resolution_mm) >
          kIntegerMillimeterTolerance) {
    return std::nullopt;
  }

  const auto resolution_mm =
      static_cast<std::uint16_t>(rounded_resolution_mm);
  const double canonical_resolution_m =
      static_cast<double>(resolution_mm) / 1000.0;
  const double cols_value = std::floor(width_m / canonical_resolution_m);
  const double rows_value = std::floor(height_m / canonical_resolution_m);
  if (!std::isfinite(cols_value) || !std::isfinite(rows_value) ||
      cols_value < 1.0 || rows_value < 1.0 || cols_value > kUint16Max ||
      rows_value > kUint16Max) {
    return std::nullopt;
  }

  // These casts are safe only after the representability checks above.
  const auto cols = static_cast<std::uint16_t>(cols_value);
  const auto rows = static_cast<std::uint16_t>(rows_value);
  const std::size_t rows_size = static_cast<std::size_t>(rows);
  const std::size_t cols_size = static_cast<std::size_t>(cols);
  if (cols_size > max_cells / rows_size) {
    return std::nullopt;
  }
  const std::size_t cell_count = rows_size * cols_size;
  return GridGeometry{rows, cols, resolution_mm, canonical_resolution_m,
                      cell_count};
}

struct GridCellIndex {
  std::uint16_t row{0};
  std::uint16_t col{0};
  std::size_t linear{0};
};

struct GridVisualizationGeometry {
  int source_width{0};
  int source_height{0};
  int display_scale_px{0};
  int grid_step_px{0};
  int output_width{0};
  int output_height{0};
};

inline std::optional<GridVisualizationGeometry>
TryMakeGridVisualizationGeometry(
    const GridGeometry& geometry,
    std::size_t max_pixels = kMaxVisualizationPixels) noexcept {
  if (geometry.rows == 0U || geometry.cols == 0U ||
      !std::isfinite(geometry.resolution_m) || geometry.resolution_m <= 0.0 ||
      max_pixels == 0U) {
    return std::nullopt;
  }
  const std::size_t rows = static_cast<std::size_t>(geometry.rows);
  const std::size_t cols = static_cast<std::size_t>(geometry.cols);
  if (cols > std::numeric_limits<std::size_t>::max() / rows ||
      rows * cols != geometry.cell_count) {
    return std::nullopt;
  }

  const double raw_scale = std::round(geometry.resolution_m * 400.0);
  if (!std::isfinite(raw_scale) ||
      raw_scale > static_cast<double>(std::numeric_limits<int>::max() / 2)) {
    return std::nullopt;
  }
  const int display_scale =
      raw_scale < 1.0 ? 1 : static_cast<int>(raw_scale);
  const std::size_t scale = static_cast<std::size_t>(display_scale);
  if (cols > static_cast<std::size_t>(std::numeric_limits<int>::max()) / scale ||
      rows > static_cast<std::size_t>(std::numeric_limits<int>::max()) / scale) {
    return std::nullopt;
  }
  const std::size_t output_width = cols * scale;
  const std::size_t output_height = rows * scale;
  if (output_width > max_pixels / output_height) {
    return std::nullopt;
  }

  return GridVisualizationGeometry{
      static_cast<int>(cols), static_cast<int>(rows), display_scale,
      display_scale * 2, static_cast<int>(output_width),
      static_cast<int>(output_height)};
}

// Validate all floating-point operands and bounds before converting a point
// coordinate to an integral grid index. This also rejects the old truncation
// artifact where a coordinate just below zero was converted to cell zero.
inline bool TryProjectPointToGrid(double point_x, double point_y,
                                  double point_z, double map_width_m,
                                  double map_resolution_m,
                                  const GridGeometry& geometry,
                                  GridCellIndex* output) noexcept {
  if (output == nullptr || !std::isfinite(point_x) ||
      !std::isfinite(point_y) || !std::isfinite(point_z) ||
      !std::isfinite(map_width_m) || !std::isfinite(map_resolution_m) ||
      map_width_m <= 0.0 || map_resolution_m <= 0.0 || geometry.rows == 0U ||
      geometry.cols == 0U) {
    return false;
  }

  if (!std::isfinite(geometry.resolution_m) ||
      std::fabs(map_resolution_m - geometry.resolution_m) >
          std::numeric_limits<double>::epsilon() * 8.0) {
    return false;
  }

  const std::size_t rows = static_cast<std::size_t>(geometry.rows);
  const std::size_t cols = static_cast<std::size_t>(geometry.cols);
  if (cols > std::numeric_limits<std::size_t>::max() / rows ||
      rows * cols != geometry.cell_count) {
    return false;
  }

  const double row_value = point_x / map_resolution_m;
  const double col_value =
      (-point_y + map_width_m * 0.5) / map_resolution_m;
  if (!std::isfinite(row_value) || !std::isfinite(col_value) ||
      row_value < 0.0 || col_value < 0.0 ||
      row_value >= static_cast<double>(geometry.rows) ||
      col_value >= static_cast<double>(geometry.cols)) {
    return false;
  }

  const auto row = static_cast<std::uint16_t>(row_value);
  const auto col = static_cast<std::uint16_t>(col_value);
  const std::size_t linear = static_cast<std::size_t>(row) * cols + col;
  if (linear >= geometry.cell_count) {
    return false;
  }
  *output = GridCellIndex{row, col, linear};
  return true;
}

struct GridNeighborhood {
  std::array<std::size_t, 9> cells{};
  std::size_t count{0};
};

// Return the clipped 3x3 neighborhood, including the center cell. Computing
// row and column separately prevents +/-1 from wrapping across grid rows.
inline GridNeighborhood MakeGridNeighborhood(
    const GridGeometry& geometry, std::size_t center) noexcept {
  GridNeighborhood result;
  if (geometry.rows == 0U || geometry.cols == 0U ||
      center >= geometry.cell_count) {
    return result;
  }
  const std::size_t cols = static_cast<std::size_t>(geometry.cols);
  const std::size_t center_row = center / cols;
  const std::size_t center_col = center % cols;
  for (int row_delta = -1; row_delta <= 1; ++row_delta) {
    const auto neighbor_row = static_cast<std::int64_t>(center_row) + row_delta;
    if (neighbor_row < 0 ||
        neighbor_row >= static_cast<std::int64_t>(geometry.rows)) {
      continue;
    }
    for (int col_delta = -1; col_delta <= 1; ++col_delta) {
      const auto neighbor_col =
          static_cast<std::int64_t>(center_col) + col_delta;
      if (neighbor_col < 0 ||
          neighbor_col >= static_cast<std::int64_t>(geometry.cols)) {
        continue;
      }
      result.cells[result.count++] =
          static_cast<std::size_t>(neighbor_row) * cols +
          static_cast<std::size_t>(neighbor_col);
    }
  }
  return result;
}

inline std::optional<int> ResolveAiGridType(
    const std::map<int, int>& label_map, int point_label) noexcept {
  const auto found = label_map.find(point_label);
  if (found == label_map.end() || found->second <= 0) {
    return std::nullopt;  // Unknown labels must not mutate an existing cell.
  }
  return found->second;
}

inline bool ContainsClassId(const std::vector<int>& ids, int class_id) {
  return std::find(ids.begin(), ids.end(), class_id) != ids.end();
}

// Build the point-label to grid-type contract from the model's current class
// maps. Known business classes retain their historical visualization types;
// other non-passable detections become generic obstacles. Segmentation labels
// are only obstacles when the corresponding policy is enabled.
inline std::map<int, int> BuildAiGridLabelMap(
    const std::vector<std::string>& detection_names,
    const std::vector<std::string>& segmentation_names,
    const std::vector<int>& passable_detection_ids,
    const std::vector<int>& passable_segmentation_ids,
    bool nongrass_seg_as_obstacle) {
  const std::map<std::string, int> named_detection_types{
      {"person", 4}, {"cat", 5}, {"dog", 6}, {"charger", 7}};
  std::map<int, int> result;
  const std::size_t detection_count = std::min<std::size_t>(
      detection_names.size(), static_cast<std::size_t>(kAiClassIdLimit));
  for (std::size_t index = 0; index < detection_count; ++index) {
    const int class_id = static_cast<int>(index);
    int grid_type = 2;
    if (ContainsClassId(passable_detection_ids, class_id)) {
      grid_type = 3;
    } else {
      const auto named = named_detection_types.find(detection_names[index]);
      if (named != named_detection_types.end()) grid_type = named->second;
    }
    result.emplace(EncodeDetLabel(class_id), grid_type);
  }

  const std::size_t segmentation_count = std::min<std::size_t>(
      segmentation_names.size(), static_cast<std::size_t>(kAiClassIdLimit));
  for (std::size_t index = 0; index < segmentation_count; ++index) {
    const int class_id = static_cast<int>(index);
    if (ContainsClassId(passable_segmentation_ids, class_id)) {
      result.emplace(EncodeSegLabel(class_id), 3);
    } else if (nongrass_seg_as_obstacle) {
      result.emplace(EncodeSegLabel(class_id), 1);
    }
  }
  return result;
}

inline int AiGridTypePriority(int type) noexcept {
  if (type >= 4) return 4;  // detector-class obstacle
  if (type == 2) return 3;  // generic obstacle
  if (type == 3) return 2;  // grass
  if (type == 1) return 1;  // non-grass
  return 0;                 // unknown or invalid
}

// A deterministic, commutative merge makes the result independent of point
// traversal order. Lower type id is the stable tie-break for equal-priority
// detector classes.
inline int MergeAiGridType(int current, int candidate) noexcept {
  const int current_priority = AiGridTypePriority(current);
  const int candidate_priority = AiGridTypePriority(candidate);
  if (candidate_priority == 0) return current_priority == 0 ? 0 : current;
  if (current_priority == 0) return candidate;
  if (candidate_priority > current_priority) return candidate;
  if (candidate_priority < current_priority) return current;
  return current < candidate ? current : candidate;
}

}  // namespace detail
}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_AI_FUSION_SAFETY_HPP_
