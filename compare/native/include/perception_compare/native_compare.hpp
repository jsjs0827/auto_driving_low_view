#pragma once
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace perception_compare {
struct Intrinsics {
  int width = 640, height = 544;
  double fx = 0, fy = 0, cx = 0, cy = 0, bxf = 0; // bxf = focal_px * baseline_mm
};
struct Options {
  std::filesystem::path capture, sdk_config, output, model, runner, sdk_root;
  std::filesystem::path detection_model, segmentation_model;
  std::optional<Intrinsics> intrinsics;
  std::string method = "stereo", order = "original,stereo", input_mode = "resize";
  int limit = 100, warmup = 5, repeat = 3, save_every = 10;
  int input_width = 0, input_height = 0;
  std::size_t max_points = 20000;
  bool both = false, all = false, prepare_only = false;
  bool bpu_enabled = true;
  unsigned bpu_interval_ms = 100;
  std::vector<std::string> bpu_paths;
};
// Validate a genuine immutable left/right/Q8 recording, then invoke the existing
// SDK runner. No disparity synthesis or measurement estimation takes place here.
void runCompare(const Options &options);
// Rebuild reports from existing actual frames.jsonl; missing metrics stay null.
void generateReport(const std::filesystem::path &output, std::size_t max_points = 20000);
// Compare saved artifacts from two code versions, after checking input/model
// identities. Returns true only if coverage and all compared artifacts match.
bool checkVersions(const std::filesystem::path &baseline, const std::filesystem::path &candidate,
                   const std::filesystem::path &output, double xyz_tolerance = 0.0);
} // namespace perception_compare
