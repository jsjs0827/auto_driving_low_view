#include "compare_util.hpp"
#include "comparison_roi.hpp"
#include "camera_capture_util.hpp"
#include "perception_full_sdk.h"
#include "perception_temporal_sdk.h"
#include "perception/inference_task.hpp"
#include "replay_passes.hpp"
#include "json.hpp"
#include "bpu_metrics.hpp"
#ifdef R36_COMPARE_WITH_HORIZON
#include "r36_stereo/horizon_stereo.hpp"
#include "r36_stereo/nv12.hpp"
#endif
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <thread>
#include <sys/resource.h>
#include <unistd.h>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using r36compare::quote;
using r36compare::number;
namespace {
namespace raw=ros_so_demo::raw;
volatile std::sig_atomic_t interrupted=0;
void stopCamera(int) { interrupted=1; }
struct CameraInterrupted:std::runtime_error { CameraInterrupted():std::runtime_error("Camera run interrupted"){} };
double ms(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }
double processCpuMs() {
  struct timespec value{};
  if (::clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value)) throw std::runtime_error("clock_gettime process CPU failed");
  return value.tv_sec * 1000.0 + value.tv_nsec / 1e6;
}
struct Resource {
  double cpu_ms = 0;
  long rss_kib = 0, peak_rss_kib = 0;
};
Resource resource() {
  struct rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage)) throw std::runtime_error("getrusage failed");
  Resource result;
  result.cpu_ms = usage.ru_utime.tv_sec * 1000.0 + usage.ru_utime.tv_usec / 1000.0 +
                  usage.ru_stime.tv_sec * 1000.0 + usage.ru_stime.tv_usec / 1000.0;
  result.peak_rss_kib = usage.ru_maxrss;
  std::ifstream status("/proc/self/status"); std::string line;
  while (std::getline(status, line)) if (line.rfind("VmRSS:", 0) == 0) {
    std::istringstream value(line.substr(6)); value >> result.rss_kib; break;
  }
  if (!status || result.rss_kib <= 0) throw std::runtime_error("Cannot read Linux /proc/self/status VmRSS");
  return result;
}
void checked(std::uint32_t code, const psdk_status& status, const char* operation) {
  if (!code) return;
  std::ostringstream out; out << operation << " failed code=" << code << " native=" << status.native_code
    << " feature=" << std::string(status.feature, strnlen(status.feature, sizeof(status.feature)))
    << " task=" << std::string(status.task, strnlen(status.task, sizeof(status.task)))
    << " detail=" << std::string(status.detail, strnlen(status.detail, sizeof(status.detail)));
  throw std::runtime_error(out.str());
}
std::vector<std::uint8_t> readExact(const fs::path& path, std::size_t expected) {
  if (!fs::is_regular_file(path) || fs::file_size(path) != expected)
    throw std::invalid_argument("Expected exactly " + std::to_string(expected) + " bytes: " + path.u8string());
  std::vector<std::uint8_t> data(expected);
  std::ifstream in(path, std::ios::binary); in.read(reinterpret_cast<char*>(data.data()), std::streamsize(data.size()));
  if (!in) throw std::runtime_error("Cannot read " + path.u8string());
  return data;
}
struct Options {
  fs::path manifest, config, output;
  std::string method, model, det_model, seg_model;
  std::string stereo_input_mode = "strict";
  int stereo_input_width = 0, stereo_input_height = 0;
  int width = 640, height = 544, warmup = 5, repeat = 3, save_every = 10;
  cv::Rect comparison_roi;
  double fx = 0, fy = 0, cx = 0, cy = 0, bxf = 0;
  r29::BpuSamplerOptions bpu;
  bool camera_input=false;
  int frames=100;
  raw::CameraOptions camera;
};
void help() {
  std::cout << "r36_compare_runner --manifest pairs.tsv --sdk-config full.json --method detseg_only|original|stereo\n"
    "  --output NEW_DIRECTORY --width 640 --height 544 --fx N --fy N --cx N --cy N --bxf FX_TIMES_BASELINE_MM\n"
    "  [--model stereo.bin] [--det-model det.bin] [--seg-model seg.bin] [--warmup 5] [--repeat 3] [--save-every 10]\n"
    "  [--stereo-input-mode strict|resize|crop] [--stereo-input-width N --stereo-input-height N]\n"
    "  [--comparison-roi x,y,width,height] (strict even integers, inside full source; default full source)\n"
    "Crop takes bottom full-width NV12 without resizing; comparison ROI must match the actual inference ROI.\n"
    "Original Q8 is invalidated outside comparison ROI inside compute timing; detseg retains the full left image.\n"
    "Strict is the default; resize adapts stereo NV12 Y/UV inside computation. Optional dimensions assert model metadata.\n"
    "Manifest: six TSV columns: stem left_nv12 left_png right_png original_q8_raw right_nv12 (legacy five-column PNG accepted).\n"
    "  [--bpu-enabled 0|1] [--bpu-interval-ms 100] [--bpu-path /sys/.../ratio (repeatable)]\n"
    "Camera alternative: omit --manifest and all geometry arguments; supply --camera-sdk LIB --camera-config FILE\n"
    "  [--frames 100] [--camera-mode 1|2] [--camera-timeout-ms 3000] [--camera-retry-ms 5] --repeat 1\n"
    "Camera intrinsics come from SDK rectified reference; original requires mode2, detseg_only/stereo require mode1.\n"
    "Camera is synchronous next->process without ROS/latest-frame queue; live groups use unpaired captures.\n"
    "warmup cycles the first N dataset frames; repeat measures complete dataset N times.\n"
    "save-every 0 disables files; N saves measured ordinal 0,N,2N,... in every method.\n"
    "Resource metrics are Linux process CPU time and VmRSS; peak RSS is process-lifetime high-water mark.\n"
#ifdef R36_COMPARE_WITH_HORIZON
    "This binary supports real Horizon stereo and recorded original Q8.\n";
#else
    "This binary supports recorded original Q8 only; --method stereo fails explicitly.\n";
#endif
}
Options options(int argc, char** argv) {
  std::map<std::string, std::string> values;
  std::vector<std::string> bpu_paths;
  const std::set<std::string> allowed = {"--manifest", "--sdk-config", "--method", "--output", "--model", "--det-model", "--seg-model",
    "--width", "--height", "--fx", "--fy", "--cx", "--cy", "--bxf", "--warmup", "--repeat", "--save-every",
    "--stereo-input-mode", "--stereo-input-width", "--stereo-input-height", "--comparison-roi", "--bpu-enabled", "--bpu-interval-ms", "--bpu-path",
    "--camera-sdk", "--camera-config", "--frames", "--camera-mode", "--camera-timeout-ms", "--camera-retry-ms"};
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--bpu-path" && i + 1 < argc) { bpu_paths.push_back(argv[++i]); continue; }
    if (!allowed.count(key) || i + 1 == argc || values.count(key)) throw std::invalid_argument("Unknown, missing or duplicated argument: " + key);
    values[key] = argv[++i];
  }
  auto text = [&](const char* key) { const auto it = values.find(key); return it == values.end() ? std::string() : it->second; };
  for (const auto* key : {"--sdk-config", "--method", "--output"})
    if (text(key).empty()) throw std::invalid_argument(std::string("Required argument: ") + key);
  auto integer = [&](const char* key, int fallback) {
    const auto value = text(key); if (value.empty()) return fallback;
    if (!std::all_of(value.begin(), value.end(), [](unsigned char c) { return c >= '0' && c <= '9'; }))
      throw std::invalid_argument(std::string("Expected nonnegative integer: ") + key);
    std::size_t end = 0; const auto parsed = std::stoul(value, &end);
    if (end != value.size() || parsed > 1000000) throw std::invalid_argument(std::string("Integer outside supported range: ") + key);
    return int(parsed);
  };
  auto real = [&](const char* key) { std::size_t end = 0; const auto value = text(key); const double parsed = std::stod(value, &end);
    if (end != value.size() || !std::isfinite(parsed)) throw std::invalid_argument(std::string("Expected finite number: ") + key);
    return parsed;
  };
  Options o; o.manifest = fs::u8path(text("--manifest")); o.config = fs::u8path(text("--sdk-config")); o.output = fs::u8path(text("--output"));
  o.method = text("--method"); o.model = text("--model"); o.det_model = text("--det-model"); o.seg_model = text("--seg-model");
  for(const auto* key:{"--camera-sdk","--camera-config","--frames","--camera-mode","--camera-timeout-ms","--camera-retry-ms"})
    if(values.count(key))o.camera_input=true;
  if(o.camera_input) {
    if(values.count("--manifest"))throw std::invalid_argument("Camera and --manifest are mutually exclusive");
    if(text("--camera-sdk").empty()||text("--camera-config").empty())
      throw std::invalid_argument("Camera requires both --camera-sdk and --camera-config");
    for(const auto* key:{"--width","--height","--fx","--fy","--cx","--cy","--bxf"})
      if(values.count(key))throw std::invalid_argument(std::string("Camera geometry is obtained from the SDK; do not pass ")+key);
    o.camera.sdk=text("--camera-sdk");o.camera.config=text("--camera-config");
    o.frames=integer("--frames",100);o.repeat=1;
    o.camera.need_q8=o.method=="original";o.camera.need_gdc=false;
    const int mode=r36compare::runner::requiredCameraMode(o.method);
    o.camera.disparity_mode=integer("--camera-mode",mode);
    if(o.camera.disparity_mode!=mode)throw std::invalid_argument("Camera mode must be 2 for original and 1 for detseg_only/stereo; no fallback");
    o.camera.timeout_ms=integer("--camera-timeout-ms",3000);o.camera.retry_ms=integer("--camera-retry-ms",5);
    if(!o.frames||!o.camera.timeout_ms||!o.camera.retry_ms||o.camera.retry_ms>o.camera.timeout_ms)
      throw std::invalid_argument("Camera frames/timeout/retry must be positive, retry <= timeout");
    if(!fs::is_regular_file(o.camera.sdk)||!fs::is_regular_file(o.camera.config))
      throw std::invalid_argument("Camera SDK and vendor configuration must be existing regular files");
    o.camera.sdk=fs::absolute(fs::u8path(o.camera.sdk)).u8string();
    o.camera.config=fs::absolute(fs::u8path(o.camera.config)).u8string();
  } else {
    for(const auto* key:{"--manifest","--fx","--fy","--cx","--cy","--bxf"})
      if(text(key).empty())throw std::invalid_argument(std::string("Required argument: ")+key);
  }
  if (values.count("--stereo-input-mode")) o.stereo_input_mode = text("--stereo-input-mode");
  if (values.count("--stereo-input-width") != values.count("--stereo-input-height"))
    throw std::invalid_argument("--stereo-input-width and --stereo-input-height must both be set or both omitted");
  o.stereo_input_width = integer("--stereo-input-width", 0); o.stereo_input_height = integer("--stereo-input-height", 0);
  if (values.count("--stereo-input-width") && (!o.stereo_input_width || !o.stereo_input_height))
    throw std::invalid_argument("Stereo model metadata assertions must be positive");
  r36compare::validateStereoInputRequest(o.stereo_input_mode, {o.stereo_input_width, o.stereo_input_height});
  o.width = integer("--width", o.width); o.height = integer("--height", o.height);
  o.warmup = integer("--warmup", o.warmup); o.repeat = integer("--repeat", o.repeat); o.save_every = integer("--save-every", o.save_every);
  if(o.camera_input&&o.repeat!=1)throw std::invalid_argument("Camera runs require --repeat 1; live captures cannot be replayed");
  const auto bpu_enabled = values.count("--bpu-enabled") ? text("--bpu-enabled") : "1";
  if (bpu_enabled != "0" && bpu_enabled != "1") throw std::invalid_argument("--bpu-enabled requires 0 or 1");
  o.bpu.enabled = bpu_enabled == "1"; o.bpu.sample_interval_ms = integer("--bpu-interval-ms", 100); o.bpu.paths = std::move(bpu_paths);
  if(!o.camera_input) {o.fx = real("--fx"); o.fy = real("--fy"); o.cx = real("--cx"); o.cy = real("--cy"); o.bxf = real("--bxf");}
  if ((o.method != "original" && o.method != "stereo" && o.method != "detseg_only") || !o.repeat || o.width < 2 || o.height < 2 ||
      o.width > 4096 || o.height > 4096 || ((o.width | o.height) & 1) || (!o.camera_input&&(o.fx <= 0 || o.fy <= 0 || o.bxf <= 0)))
    throw std::invalid_argument("Invalid method, dimensions, repeat or intrinsics (bxf must be fx*baseline_mm)");
  o.comparison_roi=values.count("--comparison-roi")?
    r36compare::runner::parseComparisonRoi(text("--comparison-roi"),{o.width,o.height}):cv::Rect(0,0,o.width,o.height);
  if (o.method == "stereo" && o.model.empty()) throw std::invalid_argument("--model is required for stereo; no fallback is permitted");
#ifndef R36_COMPARE_WITH_HORIZON
  if (o.method == "stereo") throw std::invalid_argument("This binary was built without Horizon stereo support; rebuild with R36_COMPARE_WITH_HORIZON=ON");
#endif
  if ((!o.camera_input&&!fs::is_regular_file(o.manifest)) || !fs::is_regular_file(o.config)) throw std::invalid_argument("Manifest and SDK config must be existing regular files");
  if (fs::exists(o.output) || fs::is_symlink(o.output)) throw std::invalid_argument("Output must be a NEW directory: " + o.output.u8string());
  return o;
}
struct Pair { std::string stem; fs::path nv12, left, right, original_q8, right_nv12; };
std::vector<Pair> manifest(const Options& options) {
  std::ifstream in(options.manifest, std::ios::binary); std::string line; std::vector<Pair> pairs; std::set<std::string> stems;
  auto path = [&](const std::string& value) { auto p = fs::u8path(value); return p.is_absolute() ? p : options.manifest.parent_path() / p; };
  const auto pixels = std::size_t(options.width) * options.height;
  while (std::getline(in, line)) {
    auto fields = r36compare::tsv(line);
    if (!stems.insert(fields[0]).second) throw std::invalid_argument("Duplicate manifest stem: " + fields[0]);
    Pair pair{fields[0], path(fields[1]), path(fields[2]), path(fields[3]), path(fields[4])};
    if (fields.size() == 6) pair.right_nv12 = path(fields[5]);
    if (!fs::is_regular_file(pair.nv12) || fs::file_size(pair.nv12) != pixels * 3 / 2) throw std::invalid_argument("Invalid tight NV12 input: " + pair.nv12.u8string());
    if (options.method == "original") {
      if (!fs::is_regular_file(pair.original_q8) || fs::file_size(pair.original_q8) != pixels * 2) throw std::invalid_argument("Invalid tight original uint16 Q8 input: " + pair.original_q8.u8string());
    } else if (options.method == "stereo") {
      if (!pair.right_nv12.empty()) {
        if (!fs::is_regular_file(pair.right_nv12) || fs::file_size(pair.right_nv12) != pixels * 3 / 2)
          throw std::invalid_argument("Stereo requires tight genuine right NV12");
      } else if (!fs::is_regular_file(pair.right)) throw std::invalid_argument("Stereo requires genuine right image path");
    }
    pairs.push_back(std::move(pair));
  }
  if (in.bad() || pairs.empty()) throw std::invalid_argument("Manifest is unreadable or empty");
  return pairs;
}
struct Provider {
  perception::inference::StagedInvocation invocation;
  cv::Mat left, right;
  std::vector<std::uint16_t> q8;
  r36compare::StereoInputAdaptation adaptation;
  cv::Rect comparison_roi;
  cv::Size output_size;
  int width = 0, height = 0;
  double stereo_ms = 0;
  bool called = false;
#ifdef R36_COMPARE_WITH_HORIZON
  std::unique_ptr<r36stereo::HorizonStereoModel> model;
#endif
};
std::uint32_t disparity(void* userdata, const psdk_frame* frame, psdk_disparity_q8* out, psdk_status* status) noexcept {
  auto& provider = *static_cast<Provider*>(userdata); const auto start = Clock::now();
  try {
    if (!frame || !out) throw std::invalid_argument("Null stereo hook argument");
#ifdef R36_COMPARE_WITH_HORIZON
    if (!provider.model) throw std::logic_error("Stereo model is not initialized");
    // Explicit NV12 crop/resize and model packing stay inside this SDK Task.
    // Source NV12, intrinsics and bxf remain at the recording resolution.
    using perception::inference::Stage;
    const perception::FrameInfo identity{frame->frame_id,frame->timestamp_ns};
    provider.invocation.begin("stereo",identity,{identity,identity},2);
    cv::Mat left,right,prediction;
    provider.invocation.execute(Stage::Preprocess,[&]{
      const auto images=r36compare::prepareStereoNv12(provider.left,provider.right,provider.adaptation);
      left=images.left;right=images.right;
    });
    provider.invocation.execute(Stage::Backend,[&]{prediction=provider.model->infer_nv12(left,right);});
    provider.invocation.execute(Stage::Postprocess,[&]{
      if(prediction.cols!=provider.adaptation.model_input_size.width)
        throw std::runtime_error("Actual model disparity width differs from model input width");
      if(provider.output_size.width&&provider.output_size!=prediction.size())
        throw std::runtime_error("Stereo model output dimensions changed between frames");
      if(!provider.output_size.width){
        provider.output_size=prediction.size();
        std::cerr<<"Stereo input adaptation: "<<r36compare::stereoInputAdaptationJson(provider.adaptation,provider.output_size)<<'\n';
      }
      provider.q8=r36compare::restoreStereoQ8(prediction,provider.adaptation);
      if(provider.adaptation.mode!="crop")
        r36compare::runner::maskOutsideComparisonRoi(provider.q8,{provider.width,provider.height},provider.comparison_roi);
    });
#else
    throw std::logic_error("Horizon stereo support is disabled");
#endif
    out->struct_size = sizeof(*out); out->abi_version = PSDK_FULL_ABI_VERSION; out->width = provider.width; out->height = provider.height;
    out->stride_bytes = provider.width * 2; out->reserved = 0; out->data = provider.q8.data(); out->bytes = provider.q8.size() * 2;
    out->frame_id = frame->frame_id; out->timestamp_ns = frame->timestamp_ns;
    provider.stereo_ms = ms(start); provider.called = true; return 0;
  } catch (const std::exception& error) {
    if (status) { *status = {}; status->struct_size = sizeof(*status); status->abi_version = PSDK_ABI_VERSION; status->code = 1002;
      std::snprintf(status->feature, sizeof(status->feature), "%s", "compare.stereo.provider");
      std::snprintf(status->detail, sizeof(status->detail), "%s", error.what()); }
    return 1002;
  } catch (...) { return 1002; }
}
struct Engine {
  psdk_full_engine* value = nullptr;
  ~Engine() { psdk_status status{}; psdk_full_destroy(&value, &status); }
};
struct Result {
  psdk_full_result* value = nullptr;
  ~Result() { psdk_full_result_release(value); }
};
std::string cloudName(const psdk_cloud_view& cloud) {
  const auto size = strnlen(cloud.name, sizeof(cloud.name));
  if (!size || size == sizeof(cloud.name)) throw std::runtime_error("Invalid SDK cloud name");
  const std::string name(cloud.name, size);
  if (name != "camera" && name != "robot" && name != "ground" && name != "obstacle" && name != "semantic") throw std::runtime_error("Unknown SDK cloud name");
  if (cloud.point_count && !cloud.points) throw std::runtime_error("Null SDK cloud data");
  return name;
}
void ply(const fs::path& path, const psdk_cloud_view& cloud, const std::string& name) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("Cannot create PLY: " + path.u8string());
  out << "ply\nformat ascii 1.0\ncomment unit=m\ncomment source=actual_perception_sdk_mapping\ncomment coordinate_frame="
      << (name == "camera" ? "left_rectified_optical" : "configured_robot_extrinsics")
      << "\ncomment color=actual_SDK_RGB\nelement vertex " << cloud.point_count
      << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue"
      << "\nproperty float confidence\nproperty int label\nend_header\n" << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (std::uint64_t i = 0; i < cloud.point_count; ++i) { const auto& p = cloud.points[i];
    out << p.x << ' ' << p.y << ' ' << p.z << ' ' << unsigned(p.r) << ' ' << unsigned(p.g) << ' ' << unsigned(p.b) << ' ' << p.confidence << ' ' << p.label << '\n'; }
  out.close(); if (!out) throw std::runtime_error("Cannot finish PLY: " + path.u8string());
}
void writeText(const fs::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary); out << text << '\n'; out.close(); if (!out) throw std::runtime_error("Cannot write " + path.u8string());
}
int run(const Options& requested_options) {
  Options o=requested_options;
  const auto pairs = o.camera_input?std::vector<Pair>{}:manifest(o);
  const auto requested=r33::json::load(fs::absolute(o.config).u8string());
  const auto& inference_config=r33::json::required(requested,"inference");
  const bool expect_inference=r33::json::optional(inference_config,"enabled")?
    r33::json::boolean(*r33::json::optional(inference_config,"enabled")):true;
  const bool expect_mapping=r33::json::boolean(r33::json::required(r33::json::required(requested,"mapping"),"enabled"));
  bool expect_det=false,expect_seg=false,expect_maps=expect_mapping;
  bool expect_disparity=expect_mapping,expect_history=true;
  if(expect_inference) {
    auto dag=fs::u8path(r33::json::string(r33::json::required(inference_config,"dag")));
    if(dag.is_relative())dag=fs::absolute(o.config).parent_path()/dag;
    const auto engine=r33::json::load(dag.u8string());
    for(const auto& task:r33::json::required(engine,"tasks").object) {
      if(auto enabled=r33::json::optional(task.second,"enabled"))if(!r33::json::boolean(*enabled))continue;
      for(const auto& feature:r33::json::required(task.second,"features").array) {
        const auto name=r33::json::string(feature);
        if(name=="ai.exact.det.backend")expect_det=true;
        if(name=="ai.exact.seg.backend")expect_seg=true;
      }
    }
  }
  if(auto graph=r33::json::optional(requested,"graph")) {
    expect_maps=false;
    expect_disparity=false;expect_history=false;
    for(const auto& node:r33::json::required(*graph,"nodes").array) {
      if(auto enabled=r33::json::optional(node,"enabled"))if(!r33::json::boolean(*enabled))continue;
      const auto feature=r33::json::string(r33::json::required(node,"feature"));
      if(feature=="maps"||feature=="fusion"||feature=="product.fusion.ai_fusion")expect_maps=true;
      if(feature=="disparity"||feature=="product.stereo.disparity")expect_disparity=true;
      if(feature=="inference")if(auto algorithm=r33::json::optional(node,"algorithm"))
        if(r33::json::string(*algorithm)=="stereo")expect_disparity=true;
      if(feature=="history"||feature=="product.temporal.history")expect_history=true;
    }
  }
  if(o.method=="stereo"&&!expect_disparity)throw std::runtime_error("Stereo disparity comparison requires an enabled disparity stage");
  if(o.method=="detseg_only" && (expect_mapping || expect_disparity || !expect_det || !expect_seg))
    throw std::runtime_error("detseg_only requires det+seg enabled and disparity/mapping disabled");
  const auto capabilities = psdk_full_capabilities();
  if (expect_mapping&&!(capabilities & PSDK_CAP_MAPPING)) throw std::runtime_error("The real SDK was built without original mapping; comparison requires PSDK_CAP_MAPPING");
  const auto init_start = Clock::now();
  std::unique_ptr<raw::Camera> camera;
  std::optional<raw::CameraParameters> camera_parameters;
  if(o.camera_input) {
    interrupted=0;std::signal(SIGINT,stopCamera);std::signal(SIGTERM,stopCamera);
    camera=std::make_unique<raw::Camera>(o.camera);
    camera_parameters=camera->parameters();
    const auto k=r36compare::runner::cameraIntrinsics(*camera_parameters);
    o.width=k.width;o.height=k.height;o.fx=k.fx;o.fy=k.fy;o.cx=k.cx;o.cy=k.cy;o.bxf=k.bxf;
  }
  Provider provider; provider.width = o.width; provider.height = o.height;provider.comparison_roi=o.comparison_roi;
#ifdef R36_COMPARE_WITH_HORIZON
  if (o.method == "stereo") { provider.model = std::make_unique<r36stereo::HorizonStereoModel>(o.model, true);
    provider.adaptation = r36compare::stereoInputAdaptation(o.stereo_input_mode, {o.width, o.height}, provider.model->input_size(),
      {o.stereo_input_width, o.stereo_input_height});
    r36compare::runner::requireMatchingCropComparisonRoi(o.stereo_input_mode,o.comparison_roi,provider.adaptation.source_roi);
    std::cerr << "Stereo input adaptation (before inference): " << r36compare::stereoInputAdaptationJson(provider.adaptation) << '\n'; }
#endif
  psdk_full_hooks hooks{}; hooks.struct_size = sizeof(hooks); hooks.abi_version = PSDK_FULL_ABI_VERSION; hooks.userdata = &provider; hooks.disparity_provider = disparity;
  const auto config_path = fs::absolute(o.config).u8string();
  psdk_full_config config{}; config.struct_size = sizeof(config); config.abi_version = PSDK_FULL_ABI_VERSION; config.json_file = config_path.c_str();
  config.detection_model = o.det_model.empty() ? nullptr : o.det_model.c_str(); config.segmentation_model = o.seg_model.empty() ? nullptr : o.seg_model.c_str();
  config.hooks = o.method == "stereo" ? &hooks : nullptr;
  Engine engine; psdk_status status{}; checked(psdk_full_create(&config, &engine.value, &status), status, "psdk_full_create");
  checked(psdk_full_command(engine.value, 1, &status), status, "psdk_full_command(map)");
  const double init_ms = ms(init_start);
  // No process failure is represented as a successful benchmark. Only completed
  // runs get complete.json. The orchestrator preserves partial evidence.
  if (!fs::create_directory(o.output)) throw std::runtime_error("Cannot exclusively create output: " + o.output.u8string());
  std::ofstream rows(o.output / "frames.jsonl", std::ios::binary); if (!rows) throw std::runtime_error("Cannot create frames.jsonl");
  r29::BpuSampler bpu(o.bpu);
  const long online_cpus = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
  unsigned history_resets=0;
  std::optional<raw::Identity> previous_capture;
  std::uint64_t capture_sequence=0,duplicate_capture_count=0;
  double measured_original_roi_mask_ms=0;
  const auto comparison_roi_json=r36compare::runner::comparisonRoiJson(o.comparison_roi,{o.width,o.height});
  const auto outside_roi_policy=o.method=="detseg_only"?"not_applicable_no_disparity":
    (o.comparison_roi==cv::Rect(0,0,o.width,o.height)?"full_frame":"invalid_q8_zero");
  auto runMetadata = [&] {
  std::ostringstream run;
  run << "{\"schema\":\"r36_compare_runner_v1\",\"method\":" << quote(o.method) << ",\"manifest\":" << (o.camera_input?"null":quote(fs::absolute(o.manifest).u8string()))
      << ",\"sdk_config\":" << quote(config_path) << ",\"stereo_model\":" << quote(o.model)
       << ",\"comparison_roi\":" << comparison_roi_json << ",\"outside_roi_policy\":" << quote(outside_roi_policy)
       << ",\"full_input_and_intrinsics_unchanged\":true,\"original_roi_mask_scope\":"
       << (o.method=="original"?quote("inside_compute_and_cpu_intervals"):"null")
      << ",\"stereo_input_adaptation\":" << (o.method == "stereo" ? r36compare::stereoInputAdaptationJson(provider.adaptation, provider.output_size) : "null")
      << ",\"sdk_capabilities\":" << capabilities << ",\"online_cpus\":" << online_cpus << ",\"initialization_ms\":" << number(init_ms)
      << ",\"pair_count\":" << (o.camera_input?"null":std::to_string(pairs.size())) << ",\"warmup\":" << o.warmup << ",\"repeat\":" << o.repeat << ",\"save_every\":" << o.save_every
      << ",\"original_disparity_generation_timed\":false,\"original_disparity_generation_ms\":null,\"original_disparity_source\":" <<
          (o.camera_input?(o.method=="original"?quote("camera_sdk_mode2_uint16_Q8"):"null"):quote("recorded_uint16_Q8"))
      << ",\"stereo_input_source\":" << (o.method=="stereo" ? quote(o.camera_input?"camera_sdk_owned_left_right_NV12":(pairs.front().right_nv12.empty()?"legacy_right_PNG_to_NV12":"genuine_recorded_left_right_NV12")) : "null")
      << ",\"stereo_backend_scope\":\"NV12_tensor_pack_infer_wait_and_output_decode_not_pure_BPU_latency\""
      << ",\"bpu\":" << bpu.summaryJson()
      << ",\"bpu_window_scope\":" << quote(o.camera_input?"whole_measured_live_camera_pipeline_including_acquire_wait_SDK_threads_compute_save_logging_device_shared_not_per_frame":"whole_measured_replay_including_load_save_logging_device_shared_not_per_frame")
      << ",\"cpu_scope\":" << quote(o.camera_input?"compute_interval_process_including_concurrent_camera_SDK_threads":"compute")
      << ",\"rss_scope\":\"process\",\"peak_rss_scope\":\"process_lifetime_including_initialization_and_warmup\""
      << ",\"total_wall_scope\":" << quote(o.camera_input?"acquire_wait_owned_copy_release_compute_and_result_save_excludes_measurement_JSON_logging":"load_compute_and_explicit_result_save_excludes_measurement_json_logging")
      << ",\"intrinsics\":{\"width\":" << o.width << ",\"height\":" << o.height << ",\"fx\":" << number(o.fx) << ",\"fy\":" << number(o.fy)
      << ",\"cx\":" << number(o.cx) << ",\"cy\":" << number(o.cy) << ",\"bxf\":" << number(o.bxf) << ",\"bxf_unit\":\"pixel_mm\"}"
#ifdef R36_COMPARE_WITH_HORIZON
      << ",\"stereo_model_contract\":" << (provider.model ? provider.model->contract_json() : "null")
#else
      << ",\"stereo_model_contract\":null"
#endif
      ;
  if(o.camera_input) {
    const auto& p=*camera_parameters;
    run << ",\"input_mode\":\"camera\",\"source_mode\":\"camera\",\"comparison_cohort\":\"unpaired_live_runs\""
        << ",\"requested_frames\":" << o.frames << ",\"duplicate_capture_count\":" << duplicate_capture_count
        << ",\"camera\":{\"sdk\":" << quote(o.camera.sdk) << ",\"config\":" << quote(o.camera.config)
        << ",\"mode\":" << o.camera.disparity_mode << ",\"timeout_ms\":" << o.camera.timeout_ms << ",\"retry_ms\":" << o.camera.retry_ms
        << ",\"need_q8\":" << (o.camera.need_q8?"true":"false") << ",\"owned_right_copy\":true}"
        << ",\"intrinsics_source\":\"camera_sdk_moGetCameraParams_rectified_reference\""
        << ",\"sdk_reference\":{\"width\":" << p.width << ",\"height\":" << p.height << ",\"return_code\":" << p.source_code
        << ",\"fx\":" << number(p.rectified.fx) << ",\"fy\":" << number(p.rectified.fy) << ",\"cx\":" << number(p.rectified.cx)
        << ",\"cy\":" << number(p.rectified.cy) << ",\"bxf\":" << number(p.bxf) << '}'
        << ",\"intrinsics_scaling\":\"fx,cx,bxf*=640/reference_width;fy,cy*=544/reference_height;no_half_pixel_offset\""
        << ",\"capture_identity\":\"SDK_frame_id_and_timestamp_ns_preserved;temporal_order_by_timestamp;numeric_ID_wrap_allowed\""
        << ",\"pipeline\":\"synchronous_camera_next_then_process_without_ROS_or_latest_frame_queue\""
        << ",\"whole_window_cpu_scope\":\"process_including_camera_SDK_threads;mode2_original_additionally_includes_legacy_disparity_producer\""
        << ",\"acquire_scope\":\"SDK_wait_owned_left_right_and_optional_Q8_copies_release_Q8_decode_and_duplicate_retry\""
        << ",\"timeout_scope\":\"unique_capture_outer_deadline_checked_between_vendor_calls;vendor_blocking_call_is_not_preempted\"";
  }
  run << '}';
  return run.str();
  };
  writeText(o.output / "run.json", runMetadata());
  std::uint64_t frame_id = 0, measured = 0, last_stamp = 0;
  const auto pixels = std::size_t(o.width) * o.height;
  auto process = [&](std::size_t index, int repeat, bool warmup) {
    const double total_cpu_before = processCpuMs(); const auto total_start = Clock::now(); const auto load_start = Clock::now();
    Pair pair;raw::Capture capture;std::vector<std::uint8_t> nv12;
    double acquire_cpu_ms=0;std::uint64_t acquire_duplicates=0;
    provider.called = false; provider.stereo_ms = 0;
    if(o.camera_input) {
      const auto deadline=load_start+std::chrono::milliseconds(o.camera.timeout_ms);
      for(;;) {
        if(interrupted)throw CameraInterrupted();
        if(Clock::now()>=deadline)throw std::runtime_error("No unique camera capture within configured timeout; duplicate count="+std::to_string(acquire_duplicates));
        try {capture=camera->next([]{return !interrupted;});}
        catch(const raw::FatalCameraError&){throw;}
        catch(const std::exception&){if(interrupted)throw CameraInterrupted();throw;}
        if(Clock::now()>=deadline)throw std::runtime_error("Unique camera capture acquisition exceeded configured timeout");
        if(r36compare::runner::validateCameraCapture(capture,o.camera.need_q8,previous_capture))break;
        ++duplicate_capture_count;++acquire_duplicates;
        std::this_thread::sleep_for(std::chrono::milliseconds(o.camera.retry_ms));
      }
      previous_capture=capture.left.capture;++capture_sequence;
      pair.stem="capture-"+std::to_string(capture_sequence)+"-"+std::to_string(capture.left.capture.id)+"-"+std::to_string(capture.left.capture.ns);
      nv12=std::move(capture.left.bytes);
      if(o.method=="original")provider.q8=r36compare::decodeQ8(capture.disparity.bytes);
      else if(o.method=="stereo") {
#ifdef R36_COMPARE_WITH_HORIZON
        provider.left=cv::Mat(o.height*3/2,o.width,CV_8UC1,nv12.data());
        provider.right=cv::Mat(o.height*3/2,o.width,CV_8UC1,capture.right.bytes.data());
#endif
      }
    } else {
    pair=pairs[index];nv12=readExact(pair.nv12,pixels*3/2);
    if (o.method == "original") provider.q8 = r36compare::decodeQ8(readExact(pair.original_q8, pixels * 2));
    else if (o.method == "stereo") {
#ifdef R36_COMPARE_WITH_HORIZON
      // The owned left buffer remains alive until psdk_full_process returns.
      provider.left = cv::Mat(o.height*3/2,o.width,CV_8UC1,nv12.data());
      if (!pair.right_nv12.empty()) {
        const auto bytes=readExact(pair.right_nv12,pixels*3/2);
        provider.right=r36stereo::copy_yuv420_to_nv12(bytes.data(),o.width,o.height,o.width,bytes.size(),true);
      } else {
        const auto right=cv::imread(pair.right.u8string(),cv::IMREAD_COLOR);
        if(right.type()!=CV_8UC3 || right.size()!=cv::Size(o.width,o.height))
          throw std::invalid_argument("Legacy right PNG dimensions differ from recording");
        provider.right=r36stereo::bgr_to_nv12(right);
      }
#endif
    }
    }
    const double load_ms = ms(load_start);
    if(o.camera_input)acquire_cpu_ms=processCpuMs()-total_cpu_before;
    const double cpu_before = processCpuMs(); const auto compute_start = Clock::now();
    double original_roi_mask_ms=0;
    if(o.method=="original") {
      const auto mask_start=Clock::now();
      r36compare::runner::maskOutsideComparisonRoi(provider.q8,{o.width,o.height},o.comparison_roi);
      original_roi_mask_ms=ms(mask_start);
      if(!warmup)measured_original_roi_mask_ms+=original_roi_mask_ms;
    }
    if(o.camera_input) {frame_id=capture.left.capture.id;last_stamp=capture.left.capture.ns;}
    else {
      const auto stamp = std::uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
      last_stamp = std::max(last_stamp + 1, stamp); ++frame_id;
    }
    psdk_full_frame frame{}; frame.struct_size = sizeof(frame); frame.abi_version = PSDK_FULL_ABI_VERSION;
    auto& left = frame.left; left.struct_size = sizeof(left); left.abi_version = PSDK_ABI_VERSION; left.format = PSDK_FORMAT_NV12;
    left.width = o.width; left.height = o.height; left.stride = o.width; left.data = nv12.data(); left.bytes = nv12.size(); left.frame_id = frame_id; left.timestamp_ns = last_stamp;
    frame.intrinsics = {std::uint32_t(o.width), std::uint32_t(o.height), o.fx, o.fy, o.cx, o.cy, o.bxf};
    psdk_frame right_frame{};
    if (o.method == "stereo") {
      right_frame.struct_size=sizeof(right_frame);right_frame.abi_version=PSDK_ABI_VERSION;
      right_frame.format=PSDK_FORMAT_NV12;right_frame.width=o.width;right_frame.height=o.height;
      right_frame.stride=static_cast<std::uint32_t>(provider.right.step);
      right_frame.data=provider.right.data;right_frame.bytes=provider.right.step*provider.right.rows;
      right_frame.frame_id=frame_id;right_frame.timestamp_ns=last_stamp;
      frame.right=&right_frame;
    }
    psdk_disparity_q8 q8{}; q8.struct_size = sizeof(q8); q8.abi_version = PSDK_FULL_ABI_VERSION; q8.width = o.width; q8.height = o.height;
    q8.stride_bytes = o.width * 2; q8.data = provider.q8.data(); q8.bytes = provider.q8.size() * 2; q8.frame_id = frame_id; q8.timestamp_ns = last_stamp;
    frame.disparity = o.method == "original" ? &q8 : nullptr;
    Result result; const auto sdk_start = Clock::now(); checked(psdk_full_process(engine.value, &frame, &result.value, &status), status, "psdk_full_process"); const double sdk_ms = ms(sdk_start);
    psdk_full_result_view view{}; view.struct_size = sizeof(view); view.abi_version = PSDK_FULL_ABI_VERSION;
    checked(psdk_full_result_get_view(result.value, &view, &status), status, "psdk_full_result_get_view");
    const double compute_ms = ms(compute_start); const double cpu_ms = processCpuMs() - cpu_before; const auto after = resource();
    if (bool(view.has_inference)!=expect_inference||(expect_maps&&!view.map_count)||
        (expect_mapping&&(!view.cloud_count||!view.clouds)))throw std::runtime_error("SDK result is missing configured inference/maps/clouds; retain_clouds is required for pointcloud comparison");
    if(o.method=="detseg_only" && (view.map_count || view.cloud_count || provider.called))
      throw std::runtime_error("detseg_only unexpectedly produced mapping/stereo work");
    const auto& inference = view.inference; const auto& timings = inference.timings;
    if (view.frame_id != frame_id || view.timestamp_ns != last_stamp || (expect_inference&&(inference.frame_id != frame_id || inference.timestamp_ns != last_stamp ||
        inference.width != std::uint32_t(o.width) || inference.height != std::uint32_t(o.height) ||
        (expect_seg&&(!inference.segmentation_mask || inference.segmentation_bytes != pixels)) || (inference.detection_count && !inference.detections))))
      throw std::runtime_error("SDK did not return complete detection/segmentation for the current frame identity and dimensions");
    for (const auto value : {timings.detection_pre_ms, timings.detection_infer_ms, timings.detection_post_ms,
                             timings.segmentation_pre_ms, timings.segmentation_infer_ms, timings.segmentation_post_ms})
      if (!std::isfinite(value) || value < 0) throw std::runtime_error("SDK returned invalid detection/segmentation timing");
    if ((expect_det&&timings.detection_infer_ms <= 0) || (expect_seg&&timings.segmentation_infer_ms <= 0))
      throw std::runtime_error("Each enabled model lane must execute on every measured/warmup frame");
    if (o.method == "stereo" && !provider.called) throw std::runtime_error("SDK did not execute the stereo provider; check mapping DAG configuration");
    std::string directory; std::map<std::string, std::string> clouds; double save_ms = 0;
    if (!warmup && o.save_every && measured % std::uint64_t(o.save_every) == 0) {
      std::ostringstream name; name << "repeat-" << std::setw(3) << std::setfill('0') << repeat << "-frame-" << std::setw(6) << index << '-' << pair.stem;
      directory = name.str(); const auto save_start = Clock::now(); const auto destination = o.output / fs::u8path(directory);
      checked(psdk_full_save_result(result.value, destination.u8string().c_str(), &status), status, "psdk_full_save_result");
      for (std::uint32_t i = 0; i < view.cloud_count; ++i) { const auto cloud_name = cloudName(view.clouds[i]);
        const auto file = cloud_name + ".ply"; ply(destination / file, view.clouds[i], cloud_name); clouds[cloud_name] = directory + '/' + file; }
      save_ms = ms(save_start);
    }
    const double total_ms = ms(total_start); const double cpu_total_ms = processCpuMs()-total_cpu_before; const auto& timing = view.inference.timings;
    const auto& stereo_timings = provider.invocation.timings();
    rows << "{\"method\":" << quote(o.method) << ",\"stem\":" << quote(pair.stem) << ",\"frame_index\":" << index << ",\"repeat\":" << repeat
      << ",\"warmup\":" << (warmup ? "true" : "false") << ",\"measured_index\":" << (warmup ? "null" : std::to_string(measured))
      << ",\"frame_id\":" << frame_id << ",\"timestamp_ns\":" << last_stamp << ",\"load_ms\":" << number(load_ms)
       << ",\"comparison_roi\":" << comparison_roi_json << ",\"outside_roi_policy\":" << quote(outside_roi_policy)
       << ",\"original_roi_mask_ms\":" << (o.method=="original"?number(original_roi_mask_ms):"null")
      << ",\"stereo_ms\":" << (o.method == "stereo" ? number(provider.stereo_ms) : "null")
      << ",\"stereo_pre_ms\":" << (provider.called ? number(stereo_timings[0].elapsed_ms) : "null")
      << ",\"stereo_backend_ms\":" << (provider.called ? number(stereo_timings[1].elapsed_ms) : "null")
      << ",\"stereo_post_ms\":" << (provider.called ? number(stereo_timings[2].elapsed_ms) : "null")
      << ",\"original_disparity_generation_ms\":null,\"bpu_latest\":" << bpu.latestJson()
      << ",\"sdk_call_ms\":" << number(sdk_ms) << ",\"graph_ms\":" << number(view.graph_ms) << ",\"process_ms\":" << number(view.process_ms)
      << ",\"compute_wall_ms\":" << number(compute_ms) << ",\"cpu_ms\":" << number(cpu_ms) << ",\"cpu_wall_ms\":" << number(compute_ms)
      << ",\"cpu_scope\":" << quote(o.camera_input?"compute_interval_process_including_concurrent_camera_SDK_threads":"compute") << ",\"cpu_percent\":" << number(cpu_ms / compute_ms * 100)
      << ",\"cpu_percent_machine\":" << number(cpu_ms / compute_ms * 100 / online_cpus) << ",\"online_cpus\":" << online_cpus
      << ",\"rss_kib\":" << after.rss_kib << ",\"peak_rss_kib\":" << after.peak_rss_kib
      << ",\"peak_rss_scope\":\"process_lifetime\",\"save_ms\":" << number(save_ms) << ",\"total_wall_ms\":" << number(total_ms)
      << ",\"cpu_total_ms\":" << number(cpu_total_ms)
      << ",\"compute_fps\":" << number(1000 / compute_ms) << ",\"total_fps\":" << number(1000 / total_ms)
      << ",\"has_inference\":"<<(view.has_inference?"true":"false")
      << ",\"detection_enabled\":"<<(expect_det?"true":"false")<<",\"segmentation_enabled\":"<<(expect_seg?"true":"false")
      << ",\"map_count\":" << view.map_count << ",\"detection_count\":" << view.inference.detection_count
      << ",\"inference_ms\":{\"det_pre\":" << (expect_det?number(timing.detection_pre_ms):"null") << ",\"det_infer\":" << (expect_det?number(timing.detection_infer_ms):"null")
      << ",\"det_post\":" << (expect_det?number(timing.detection_post_ms):"null") << ",\"seg_pre\":" << (expect_seg?number(timing.segmentation_pre_ms):"null")
      << ",\"seg_infer\":" << (expect_seg?number(timing.segmentation_infer_ms):"null") << ",\"seg_post\":" << (expect_seg?number(timing.segmentation_post_ms):"null") << '}'
      << ",\"task_trace\":" << (view.task_trace_json && *view.task_trace_json ? view.task_trace_json : "null")
      << ",\"output_dir\":" << (directory.empty() ? "null" : quote(directory)) << ",\"clouds\":{";
    bool first = true; for (const auto& cloud : clouds) { if (!first) rows << ','; first = false; rows << quote(cloud.first) << ':' << quote(cloud.second); }
    rows << "},\"cloud_point_counts\":{"; first = true;
    for (std::uint32_t i = 0; i < view.cloud_count; ++i) { if (!first) rows << ','; first = false; rows << quote(cloudName(view.clouds[i])) << ':' << view.clouds[i].point_count; }
    rows << '}';
    if(o.camera_input) rows << ",\"input_mode\":\"camera\",\"source_mode\":\"camera\",\"comparison_cohort\":\"unpaired_live_runs\""
        << ",\"capture_sequence\":" << capture_sequence << ",\"acquire_ms\":" << number(load_ms) << ",\"acquire_cpu_ms\":" << number(acquire_cpu_ms)
        << ",\"acquire_duplicate_count\":" << acquire_duplicates << ",\"duplicate_capture_count\":" << duplicate_capture_count;
    rows << "}\n"; rows.flush(); if (!rows) throw std::runtime_error("Cannot write frame metrics");
    if (!warmup) { ++measured; std::cout << o.method << ' ' << pair.stem << " repeat=" << repeat << " compute_ms=" << compute_ms << " rss_kib=" << after.rss_kib << '\n'; }
  };
  bool measurement_started=false,stopped=false;unsigned warmup_completed=0;
  double measured_cpu_before=0;Clock::time_point measured_start{};
  std::exception_ptr failure;
  const auto resetHistory=[&] {
    checked(psdk_full_reset_history(engine.value,&status),status,"psdk_full_reset_history");++history_resets;
  };
  try {
    for(int i=0;i<o.warmup;++i) {
      process(o.camera_input?std::size_t(i):std::size_t(i)%pairs.size(),-1,true);++warmup_completed;
    }
    if(o.camera_input&&interrupted)throw CameraInterrupted();
    bpu.begin((o.output/"bpu_samples.jsonl").u8string());
    measured_cpu_before=processCpuMs();measured_start=Clock::now();measurement_started=true;
    if(o.camera_input) {
      if(expect_history)resetHistory();
      for(int i=0;i<o.frames;++i)process(std::size_t(i),0,false);
      if(interrupted)throw CameraInterrupted();
    } else r36compare::runMeasuredPasses(pairs.size(),o.repeat,expect_history,resetHistory,process);
  } catch(const CameraInterrupted&) {stopped=true;}
    catch(...) {failure=std::current_exception();}
  rows.close(); if (!rows) throw std::runtime_error("Cannot close frame metrics");
  std::cout.flush(); if (!std::cout) throw std::runtime_error("Cannot flush progress log");
  const double measured_wall_ms=measurement_started?ms(measured_start):0;
  const double measured_cpu_ms=measurement_started?processCpuMs()-measured_cpu_before:0;
  bpu.end();
  if(camera) {
    try{camera->close();}catch(...){if(!failure)failure=std::current_exception();}
  }
  if(o.camera_input&&interrupted)stopped=true;
  writeText(o.output/"bpu_summary.json",bpu.summaryJson());
  // The raw model output size is known after the first successful real infer.
  // Final metadata writing is outside the measured repeat loop, like complete.json.
  writeText(o.output / "run.json", runMetadata());
  if(failure) {
    if(o.camera_input) {
      std::string detail="unknown exception";
      try{std::rethrow_exception(failure);}catch(const std::exception& error){detail=error.what();}catch(...){}
      writeText(o.output/"failed.json","{\"complete\":false,\"input_mode\":\"camera\",\"source_mode\":\"camera\",\"comparison_cohort\":\"unpaired_live_runs\",\"method\":"+
          quote(o.method)+",\"error\":"+quote(detail)+",\"measured_frames\":"+std::to_string(measured)+",\"warmup_frames_completed\":"+
          std::to_string(warmup_completed)+",\"duplicate_capture_count\":"+std::to_string(duplicate_capture_count)+"}");
    }
    std::rethrow_exception(failure);
  }
  const std::string live_scope="whole_synchronous_camera_pipeline_including_wait_copy_release_history_compute_save_logging_result_release_and_camera_SDK_threads_excluding_initialization_warmup_and_close";
  const auto fps=measured&&measured_wall_ms>0?number(double(measured)*1000/measured_wall_ms):std::string("null");
  const auto completion=[&] {
  std::string complete="{\"complete\":"+std::string(stopped?"false":"true")+",\"method\":" + quote(o.method) + ",\"measured_frames\":" + std::to_string(measured) +
    ",\"comparison_roi\":"+comparison_roi_json+",\"outside_roi_policy\":"+quote(outside_roi_policy)+
    ",\"original_roi_mask_ms_total\":"+(o.method=="original"?number(measured_original_roi_mask_ms):"null")+
    ",\"original_disparity_generation_ms\":null"+
    ",\"warmup_frames\":" + std::to_string(o.warmup) + ",\"online_cpus\":" + std::to_string(online_cpus) + ",\"measured_wall_ms\":" + number(measured_wall_ms) +
    ",\"history_enabled\":" + (expect_history ? "true" : "false") + ",\"history_resets\":" + std::to_string(history_resets) +
    ",\"history_repeat_policy\":\"empty_before_each_measured_repeat\"" +
    ",\"bpu\":" + bpu.summaryJson() +
    ",\"measured_cpu_ms\":" + number(measured_cpu_ms) +
    ",\"observed_replay_fps\":" + fps +
    ",\"observed_replay_scope\":"+quote(o.camera_input?live_scope:"all_measured_replay_including_history_reset_load_compute_save_measurement_logging_and_result_release_excluding_initialization_and_warmup");
  if(o.camera_input)complete+=",\"input_mode\":\"camera\",\"source_mode\":\"camera\",\"comparison_cohort\":\"unpaired_live_runs\",\"interrupted\":"+
      std::string(stopped?"true":"false")+",\"requested_frames\":"+std::to_string(o.frames)+",\"warmup_frames_completed\":"+std::to_string(warmup_completed)+
      ",\"duplicate_capture_count\":"+std::to_string(duplicate_capture_count)+",\"camera_mode\":"+std::to_string(o.camera.disparity_mode)+
      ",\"intrinsics_source\":\"camera_sdk_moGetCameraParams_rectified_reference\",\"measured_pipeline_fps\":"+fps+
      ",\"whole_window_cpu_scope\":"+quote(live_scope)+",\"camera_sdk_closed\":true";
  complete+='}';
  return complete;
  };
  if(o.camera_input&&interrupted)stopped=true;
  auto complete=completion();
  // Include interruption during camera.close/metadata serialization before
  // choosing the output marker. A filesystem commit itself is not preemptible.
  if(o.camera_input&&interrupted&&!stopped){stopped=true;complete=completion();}
  writeText(o.output/(stopped?"partial.json":"complete.json"),complete);
  return stopped?130:0;
}
}  // namespace
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") { help(); return 0; }
  try { return run(options(argc, argv)); }
  catch (const std::exception& error) { std::cerr << "Comparison error: " << error.what() << '\n'; return 1; }
  catch (...) { std::cerr << "Comparison error: unknown exception\n"; return 1; }
}
