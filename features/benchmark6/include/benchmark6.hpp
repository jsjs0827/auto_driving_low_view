#pragma once
#include "native_config_detail.hpp"
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
namespace b6 {
using Json = v3::Json;
namespace fs = std::filesystem;
using namespace v3::detail;
struct Box { int label = 0; double score = 1, x1 = 0, y1 = 0, x2 = 0, y2 = 0; };
struct Roi { int x = 0, y = 192, width = 640, height = 352; };
struct Suite {
  fs::path file, project, capture, manifest, intrinsics;
  Json requested;
  int frames = 500, warmup = 20, quality_frames = 100, timeout_ms = 10000;
  int width = 640, height = 544, segmentation_classes = 3;
  Roi roi;
};
std::uint64_t identity(const std::string &);
std::string digest(const void *, std::size_t);
std::vector<std::uint8_t> readBytes(const fs::path &, std::size_t expected = 0);
std::vector<std::uint16_t> readQ8(const fs::path &, std::size_t pixels);
void writeBytes(const fs::path &, const void *, std::size_t);
void writeQ8(const fs::path &, const std::uint16_t *, std::size_t);
Json stats(std::vector<double>);
Json boxJson(const Box &);
std::vector<Box> boxes(const Json &);
double iou(const Box &, const Box &);
Suite loadSuite(const fs::path &);
Json makePlan(const Suite &, const fs::path &output);
void preparePlan(const Suite &, const fs::path &output);
Json report(const fs::path &output);
// Both missing labels and insufficient label coverage remain explicitly unavailable.
Json evaluate(const fs::path &prediction_dir, const fs::path &truth, int classes, Roi,
              bool require_image_digest);
Json comparePredictions(const fs::path &original_dir, const fs::path &stereo_dir, Roi);
const std::vector<std::string> &sources();
const std::vector<std::string> &methods();
std::string groupId(const std::string &source, const std::string &method);
fs::path resolved(const fs::path &base, const std::string &value);
} // namespace b6
