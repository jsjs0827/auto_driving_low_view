#include "perception_compare/native_compare.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <map>
#include <set>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;
namespace {
void help() {
  std::cout
      << "Pure C++ immutable-capture comparison and saved-result inspection\n\n"
         "r36_compare compare --capture DIR --sdk-config FULL.json --output NEW\n"
         "  [--model stereo.bin] [--runner r36_compare_runner] [--sdk-root DIR]\n"
         "  [--method detseg_only|original|stereo] [--both | --all] [--order comma-separated methods]\n"
         "  [--bpu-enabled 0|1] [--bpu-interval-ms 100] [--bpu-path /sys/.../ratio]\n"
         "  [--limit 100] [--warmup 5] [--repeat 3] [--save-every 10] [--max-points 20000]\n"
         "  [--input-mode crop|resize|strict] [--input-width N --input-height N]\n"
         "  crop requires declared model dimensions; bottom ROI is shared by all methods.\n"
         "  [--det-model FILE] [--seg-model FILE] [--prepare-only]\n"
         "  [--width 640 --height 544 --fx N --fy N --cx N --cy N --bxf N]\n"
         "r36_compare report --output EXISTING [--max-points 20000]\n"
         "r36_compare check --baseline DIR --candidate DIR [--output NEW] [--xyz-tolerance 0]\n\n"
         "Original disparity must be genuine recorded Q8.8. bxf is focal_px*baseline_mm.\n"
         "No Python interpreter, synthetic disparity, estimated CPU, or invented FPS is used.\n";
}
int integer(const std::string &s) {
  std::size_t used = 0;
  const auto n = std::stol(s, &used);
  if (used != s.size() || n < 0 || n > 1000000)
    throw std::runtime_error("Expected nonnegative integer");
  return int(n);
}
double real(const std::string &s) {
  std::size_t used = 0;
  const auto n = std::stod(s, &used);
  if (used != s.size() || !std::isfinite(n))
    throw std::runtime_error("Expected finite number");
  return n;
}
fs::path siblingRunner() {
  char buffer[PATH_MAX];
  const auto n = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (n < 0)
    return {};
  buffer[n] = 0;
  return fs::path(buffer).parent_path() / "r36_compare_runner";
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc < 2 || std::string(argv[1]) == "--help") {
      help();
      return 0;
    }
    const std::string command = argv[1];
    const std::set<std::string> flag_names = {"--both", "--all", "--prepare-only"};
    const std::set<std::string> value_names = {
        "--capture",      "--sdk-config", "--output",       "--model",      "--runner",
        "--sdk-root",     "--method",     "--order",        "--limit",      "--warmup",
        "--repeat",       "--save-every", "--max-points",   "--input-mode", "--input-width",
        "--input-height", "--det-model",  "--seg-model",    "--width",      "--height",
        "--fx",           "--fy",         "--cx",           "--cy",         "--bxf",
        "--baseline",     "--candidate",  "--xyz-tolerance", "--bpu-enabled", "--bpu-interval-ms", "--bpu-path"};
    std::map<std::string, std::string> values;
    std::set<std::string> flags;
    std::vector<std::string> bpu_paths;
    for (int i = 2; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--bpu-path" && i + 1 < argc) { bpu_paths.push_back(argv[++i]); continue; }
      if (key == "--help") {
        help();
        return 0;
      }
      if (values.count(key) || flags.count(key))
        throw std::runtime_error("Duplicate argument " + key);
      if (flag_names.count(key))
        flags.insert(key);
      else if (value_names.count(key) && i + 1 < argc)
        values[key] = argv[++i];
      else
        throw std::runtime_error("Unknown/incomplete argument " + key);
    }
    auto get = [&](const char *key, const std::string &fallback = {}) {
      const auto i = values.find(key);
      return i == values.end() ? fallback : i->second;
    };
    auto require = [&](const char *key) {
      auto value = get(key);
      if (value.empty())
        throw std::runtime_error(std::string("Missing ") + key);
      return value;
    };
    if (command == "report") {
      for (const auto &[key, value] : values) {
        (void)value;
        if (key != "--output" && key != "--max-points")
          throw std::runtime_error("Unused report argument " + key);
      }
      if (!flags.empty() || !bpu_paths.empty())
        throw std::runtime_error("Unused report flags");
      perception_compare::generateReport(require("--output"),
                                         integer(get("--max-points", "20000")));
      return 0;
    }
    if (command == "check") {
      for (const auto &[key, value] : values) {
        (void)value;
        if (key != "--baseline" && key != "--candidate" && key != "--output" &&
            key != "--xyz-tolerance")
          throw std::runtime_error("Unused check argument " + key);
      }
      if (!flags.empty() || !bpu_paths.empty())
        throw std::runtime_error("Unused check flags");
      const auto candidate = fs::u8path(require("--candidate"));
      return perception_compare::checkVersions(
                 require("--baseline"), candidate,
                 get("--output", (candidate / "version_check").u8string()),
                 real(get("--xyz-tolerance", "0")))
                 ? 0
                 : 3;
    }
    if (command != "compare")
      throw std::runtime_error("Command must be compare/report/check");
    for (const auto *key : {"--baseline", "--candidate", "--xyz-tolerance"})
      if (values.count(key))
        throw std::runtime_error(std::string("Unused compare argument ") + key);
    perception_compare::Options o;
    o.capture = require("--capture");
    o.sdk_config = require("--sdk-config");
    o.output = require("--output");
    o.model = get("--model");
    o.runner = get("--runner", siblingRunner().u8string());
    o.sdk_root = get("--sdk-root");
    o.detection_model = get("--det-model");
    o.segmentation_model = get("--seg-model");
    o.method = get("--method", "stereo");
    o.order = get("--order", flags.count("--all") ? "detseg_only,original,stereo" : "original,stereo");
    o.both = flags.count("--both");
    o.all = flags.count("--all");
    if (o.all && o.both) throw std::runtime_error("--all and --both are mutually exclusive");
    const auto bpu_enabled = get("--bpu-enabled", "1");
    if (bpu_enabled != "0" && bpu_enabled != "1") throw std::runtime_error("--bpu-enabled requires 0 or 1");
    o.bpu_enabled = bpu_enabled == "1";
    o.bpu_interval_ms = integer(get("--bpu-interval-ms", "100"));
    o.bpu_paths = std::move(bpu_paths);
    o.prepare_only = flags.count("--prepare-only");
    o.limit = integer(get("--limit", "100"));
    o.warmup = integer(get("--warmup", "5"));
    o.repeat = integer(get("--repeat", "3"));
    o.save_every = integer(get("--save-every", "10"));
    o.max_points = integer(get("--max-points", "20000"));
    o.input_mode = get("--input-mode", "resize");
    o.input_width = integer(get("--input-width", "0"));
    o.input_height = integer(get("--input-height", "0"));
    std::size_t calibrated = 0;
    for (const auto *key : {"--width", "--height", "--fx", "--fy", "--cx", "--cy", "--bxf"})
      calibrated += values.count(key);
    if (calibrated) {
      if (calibrated != 7)
        throw std::runtime_error("Explicit intrinsics require all width/height/fx/fy/cx/cy/bxf");
      perception_compare::Intrinsics k;
      k.width = integer(get("--width"));
      k.height = integer(get("--height"));
      k.fx = real(get("--fx"));
      k.fy = real(get("--fy"));
      k.cx = real(get("--cx"));
      k.cy = real(get("--cy"));
      k.bxf = real(get("--bxf"));
      o.intrinsics = k;
    }
    perception_compare::runCompare(o);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Comparison error: " << e.what() << '\n';
    return 2;
  }
}
