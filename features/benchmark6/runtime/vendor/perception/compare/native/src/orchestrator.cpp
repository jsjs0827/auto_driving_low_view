#include "json_util.hpp"
#include "disparity_compare.hpp"
#include "product_graph.hpp"
#include "perception_compare/native_compare.hpp"
#include <cerrno>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

namespace perception_compare {
using namespace detail;
namespace {
struct RecordedFrame {
  Json identity;
  fs::path left, right, q8;
  std::string stem;
};
struct Recording {
  fs::path root;
  Json metadata;
  Intrinsics calibration;
  std::vector<RecordedFrame> frames;
};

double requiredNumber(const Json &parent, const std::string &key) {
  const auto n = numeric(field(parent, key));
  if (!n)
    throw std::runtime_error("Invalid finite nonnegative calibration " + key);
  return *n;
}
Intrinsics calibration(const Json &metadata, const std::optional<Intrinsics> &override) {
  Intrinsics k;
  if (override)
    k = *override;
  else {
    const auto &j = field(metadata, "intrinsics");
    const auto width = uint64(field(j, "width"), "intrinsics.width", false),
               height = uint64(field(j, "height"), "intrinsics.height", false);
    if (width > 4096 || height > 4096)
      throw std::runtime_error("Intrinsic dimensions exceed 4096");
    k.width = int(width);
    k.height = int(height);
    k.fx = requiredNumber(j, "fx");
    k.fy = requiredNumber(j, "fy");
    k.cx = requiredNumber(j, "cx");
    k.cy = requiredNumber(j, "cy");
    k.bxf = requiredNumber(j, "bxf");
  }
  if (k.width < 2 || k.height < 2 || k.width > 4096 || k.height > 4096 || (k.width & 1) ||
      (k.height & 1) || !std::isfinite(k.fx) || !std::isfinite(k.fy) || !std::isfinite(k.cx) ||
      !std::isfinite(k.cy) || !std::isfinite(k.bxf) || k.fx <= 0 || k.fy <= 0 || k.bxf <= 0 ||
      k.cx < 0 || k.cx >= k.width || k.cy < 0 || k.cy >= k.height)
    throw std::runtime_error("Invalid rectified intrinsics: even dimensions, positive "
                             "fx/fy/bxf(px*mm), principal point inside image required");
  return k;
}
Json cameraJson(const Intrinsics &k) {
  return obj({{"width", integer(k.width)},
              {"height", integer(k.height)},
              {"fx", number(k.fx)},
              {"fy", number(k.fy)},
              {"cx", number(k.cx)},
              {"cy", number(k.cy)},
              {"bxf", number(k.bxf)}});
}
fs::path captureFile(const fs::path &root, const Json &j, const char *key) {
  const auto name = textValue(field(j, key));
  if (name.empty())
    throw std::runtime_error(std::string("Missing genuine recorded ") + key +
                             "; original Q8 is never fabricated");
  const auto p = regular(root / fs::u8path(name));
  if (!contained(root, p))
    throw std::runtime_error("Recording path escapes capture directory: " + p.u8string());
  return p;
}
Recording validateRecording(const Options &o) {
  Recording r;
  r.root = fs::canonical(o.capture);
  if (!fs::is_directory(r.root))
    throw std::runtime_error("Capture is not a directory");
  r.metadata = readJson(r.root / "metadata.json");
  r.calibration = calibration(r.metadata, o.intrinsics);
  std::ifstream manifest(r.root / "manifest.jsonl");
  if (!manifest)
    throw std::runtime_error("Missing capture manifest.jsonl");
  std::set<std::uint64_t> ids, stamps;
  std::uint64_t last = 0;
  std::string line;
  std::size_t ordinal = 0;
  while (std::getline(manifest, line) && r.frames.size() < std::size_t(o.limit)) {
    ++ordinal;
    if (line.find_first_not_of(" \t\r") == std::string::npos)
      continue;
    auto j = parse(line, (r.root / "manifest.jsonl").u8string() + ":" + std::to_string(ordinal));
    const auto id = uint64(field(j, "frame_id"), "frame_id"),
               stamp = uint64(field(j, "timestamp_ns"), "timestamp_ns", false);
    if (id > INT64_MAX || stamp > INT64_MAX || !ids.insert(id).second ||
        !stamps.insert(stamp).second || stamp <= last)
      throw std::runtime_error("Capture IDs must be unique; timestamps must be unique, positive "
                               "and strictly increasing");
    last = stamp;
    if (uint64(field(j, "width"), "frame.width") != std::uint64_t(r.calibration.width) ||
        uint64(field(j, "height"), "frame.height") != std::uint64_t(r.calibration.height))
      throw std::runtime_error("Recorded dimensions differ from rectified intrinsics");
    if (textValue(field(j, "left_format")) != "nv12" ||
        textValue(field(j, "right_format")) != "nv12" ||
        textValue(field(j, "disparity_format")) != "q8_le")
      throw std::runtime_error(
          "Recording requires two tight NV12 views and genuine little-endian Q8.8 disparity");
    RecordedFrame f;
    f.identity = j;
    f.left = captureFile(r.root, j, "left");
    f.right = captureFile(r.root, j, "right");
    f.q8 = captureFile(r.root, j, "disparity");
    if (f.left == f.right || f.left == f.q8 || f.right == f.q8)
      throw std::runtime_error("Distinct original left/right/Q8 files are required");
    const auto pixels = std::uint64_t(r.calibration.width) * r.calibration.height;
    if (fs::file_size(f.left) != pixels * 3 / 2 || fs::file_size(f.right) != pixels * 3 / 2 ||
        fs::file_size(f.q8) != pixels * 2)
      throw std::runtime_error("Recorded raw byte size mismatch (tight NV12/Q8 required)");
    f.identity.object["left_sha256"] = str(sha256(f.left));
    f.identity.object["right_sha256"] = str(sha256(f.right));
    f.identity.object["disparity_sha256"] = str(sha256(f.q8));
    std::ostringstream stem;
    stem << "capture-" << std::setw(6) << std::setfill('0') << (r.frames.size() + 1) << '-' << id
         << '-' << stamp;
    f.stem = stem.str();
    r.frames.push_back(std::move(f));
  }
  if (r.frames.empty())
    throw std::runtime_error("Recording has no complete same-capture left/right/Q8 frames");
  return r;
}
void requireObject(Json &j) {
  if (j.kind == Json::Kind::Null)
    j = object();
  if (j.kind != Json::Kind::Object)
    throw std::runtime_error("SDK configuration section must be object");
}
Json prepareSdk(const fs::path &full_path, Json &original, Json &inventory) {
  auto full = readJson(full_path);
  if (full.kind != Json::Kind::Object)
    throw std::runtime_error("SDK full configuration must be an object");
  inventory = object();
  inventory.object["base_config"] =
      obj({{"path", str(full_path.u8string())}, {"sha256", str(sha256(full_path))}});
  for (const auto *section : {"inference", "mapping", "recharge"}) {
    auto &j = full.object[section];
    requireObject(j);
    for (const auto *key :
         {"original_profile", "dag", "detection_dag", "segmentation_dag", "cpu_profile", "detection_model", "segmentation_model",
          "original_config", "extrinsics", "calibration_output", "config"}) {
      const auto text = textValue(field(j, key));
      if (text.empty())
        continue;
      const auto p = fs::absolute(full_path.parent_path() / fs::u8path(text)).lexically_normal();
      j.object[key] = str(p.u8string());
      if (fs::is_regular_file(p))
        inventory.object[std::string(section) + "." + key] =
            obj({{"path", str(p.u8string())}, {"sha256", str(sha256(p))}});
    }
  }
  const auto mapping = textValue(field(field(full, "mapping"), "original_config"));
  const bool mapping_enabled = field(full.object["mapping"], "enabled").boolean;
  if (mapping.empty() && mapping_enabled)
    throw std::runtime_error("SDK mapping.original_config is required");
  original = mapping.empty() ? object() : readJson(regular(fs::u8path(mapping)));
  // Preserve the selected graph, enabled inference lanes and mapping outputs.
  // Only retain exported cloud views when mapping is already enabled.
  if (mapping_enabled)
    full.object["mapping"].object["retain_clouds"] = boolean(true);
  auto &output = full.object["output"];
  requireObject(output);
  output.object["every_n"] = integer(0);
  output.object["save_overlay"] = boolean(false);
  output.object["save_inputs"] = boolean(false);
  // Replay the requested temporal configuration. The old one-frame baseline
  // remains a fallback only when a legacy SDK configuration has no history.
  auto &history = full.object["history"];
  if (history.kind == Json::Kind::Null)
    history = obj({{"capacity", integer(1)},
                   {"frame_interval", integer(1)},
                   {"max_memory_mb", integer(64)},
                   {"difference", boolean(false)},
                   {"match", boolean(false)}});
  else
    requireObject(history);
  for (const auto *key : {"debug", "point_cloud_color", "label_fusion", "dnn"})
    requireObject(original.object[key]);
  original.object["debug"].object["visualize"] = boolean(true);
  original.object["point_cloud_color"].object["enabled"] = boolean(true);
  original.object["point_cloud_color"].object["use_gdc_image"] = boolean(false);
  if (original.object.find("auto_cali") == original.object.end())
    original.object["auto_cali"] = boolean(false);
  return full;
}
void decodePng(const fs::path &source, const fs::path &target, const Intrinsics &k) {
  std::ifstream f(source, std::ios::binary);
  std::vector<unsigned char> bytes(std::size_t(k.width) * k.height * 3 / 2);
  if (!f.read(reinterpret_cast<char *>(bytes.data()), std::streamsize(bytes.size())))
    throw std::runtime_error("Raw input changed during preparation");
  cv::Mat yuv(k.height * 3 / 2, k.width, CV_8UC1, bytes.data()), bgr;
  cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
  if (!cv::imwrite(target.u8string(), bgr))
    throw std::runtime_error("Cannot write prepared PNG");
}
volatile std::sig_atomic_t child_pid = 0;
void forwardSignal(int signal) {
  if (child_pid > 0)
    ::kill(-child_pid, signal);
}
int execute(const std::vector<std::string> &args, const fs::path &log, const fs::path &sdk_root) {
  const int fd = ::open(log.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
  if (fd < 0)
    throw std::runtime_error("Cannot exclusively create runner log");
  const auto pid = ::fork();
  if (pid < 0) {
    ::close(fd);
    throw std::runtime_error("fork failed");
  }
  if (!pid) {
    ::setpgid(0, 0);
    ::dup2(fd, STDOUT_FILENO);
    ::dup2(fd, STDERR_FILENO);
    ::close(fd);
    if (!sdk_root.empty()) {
      const char *previous = std::getenv("LD_LIBRARY_PATH");
      const auto path =
          (sdk_root / "lib").u8string() + (previous ? ":" + std::string(previous) : "");
      ::setenv("LD_LIBRARY_PATH", path.c_str(), 1);
    }
    std::vector<char *> argv;
    for (const auto &s : args)
      argv.push_back(const_cast<char *>(s.c_str()));
    argv.push_back(nullptr);
    ::execv(argv[0], argv.data());
    std::cerr << "exec failed: " << argv[0] << '\n';
    ::_exit(127);
  }
  ::close(fd);
  ::setpgid(pid, pid);
  child_pid = pid;
  const auto previous_int = std::signal(SIGINT, forwardSignal),
             previous_term = std::signal(SIGTERM, forwardSignal);
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) {
      child_pid = 0;
      std::signal(SIGINT, previous_int);
      std::signal(SIGTERM, previous_term);
      throw std::runtime_error("waitpid failed");
    }
  }
  child_pid = 0;
  std::signal(SIGINT, previous_int);
  std::signal(SIGTERM, previous_term);
  return WIFEXITED(status) ? WEXITSTATUS(status)
                           : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
}
void option(std::vector<std::string> &args, const std::string &key, const std::string &value) {
  args.push_back("--" + key);
  args.push_back(value);
}
std::string decimal(double n) {
  std::ostringstream s;
  s << std::setprecision(17) << n;
  return s.str();
}
} // namespace

void runCompare(const Options &o) {
  if (o.limit < 1 || o.limit > 1000000 || o.warmup < 0 || o.repeat < 1 || o.save_every < 0 ||
      o.max_points < 1 || o.max_points > 200000)
    throw std::runtime_error("Invalid limit/warmup/repeat/save-every/max-points");
  if (o.method != "original" && o.method != "stereo" && o.method != "detseg_only")
    throw std::runtime_error("method must be detseg_only, original or stereo");
  if (o.bpu_interval_ms < 10 || o.bpu_interval_ms > 60000)
    throw std::runtime_error("bpu-interval-ms must be 10..60000");
  if (o.all && o.both) throw std::runtime_error("--all and --both are mutually exclusive");
  if (o.input_mode != "strict" && o.input_mode != "resize" && o.input_mode != "crop")
    throw std::runtime_error("input-mode must be crop, strict or resize");
  if ((o.input_width == 0) != (o.input_height == 0) || o.input_width < 0 || o.input_height < 0 ||
      o.input_width > 4096 || o.input_height > 4096 || (o.input_width & 1) || (o.input_height & 1))
    throw std::runtime_error(
        "Model input dimensions must both be zero/unspecified or positive even values");
  std::vector<std::string> methods = {o.method};
  if (o.both || o.all) {
    methods.clear(); std::istringstream order(o.order); std::string method;
    while (std::getline(order, method, ',')) methods.push_back(method);
    const std::set<std::string> expected = o.all ? std::set<std::string>{"detseg_only", "original", "stereo"}
                                               : std::set<std::string>{"original", "stereo"};
    if (methods.size() != expected.size() || std::set<std::string>(methods.begin(), methods.end()) != expected)
      throw std::runtime_error("--order must contain each selected method exactly once");
  }
  const bool use_stereo = std::find(methods.begin(), methods.end(), "stereo") != methods.end();
  const auto recording = validateRecording(o);
  // This geometry is derived without opening a Stereo model. Original-Q8 and
  // detseg-only crop runs need only the declared dimensions and calibration.
  const auto comparison_roi=comparisonRoi(o.input_mode,o.input_width,o.input_height,
                                           recording.calibration);
  const auto full_path = regular(o.sdk_config);
  Json original, profiles;
  auto full = prepareSdk(full_path, original, profiles);
  std::optional<r33::full::ProductGraphPlan> graph;
  if(field(full,"graph").kind==Json::Kind::Object)
    graph=r33::full::parseProductGraph(field(full,"graph"));
  if(o.all && (!field(field(full,"mapping"),"enabled").boolean || (graph && !graph->enabled("cloud"))))
    throw std::runtime_error("Three-way fusion benchmark requires enabled mapping/cloud in the base graph");
  if (o.all || o.method == "detseg_only") {
    const auto &inference = field(full,"inference");
    if (!field(inference,"enabled").boolean) throw std::runtime_error("Three-way/detseg_only requires inference enabled");
    bool det=false,seg=false;
    const auto dag=readJson(regular(fs::u8path(textValue(field(inference,"dag")))));
    for(const auto &[id,task]:field(dag,"tasks").object) {
      (void)id;
      if(field(task,"enabled").kind==Json::Kind::Boolean && !field(task,"enabled").boolean) continue;
      for(const auto &feature:field(task,"features").array) {
        if(textValue(feature)=="ai.exact.det.backend") det=true;
        if(textValue(feature)=="ai.exact.seg.backend") seg=true;
      }
    }
    if(graph && !graph->enabled("dnn")) {
      det=graph->enabled("det");seg=graph->enabled("seg");
    }
    if(!det||!seg) throw std::runtime_error("Three-way/detseg_only requires both detection and segmentation lanes");
  }
  const auto requested_source = textValue(field(field(full, "input"), "disparity_source"), "auto");
  bool enabled_disparity = field(field(full, "mapping"), "enabled").boolean;
  if (graph) enabled_disparity = graph->enabled("disparity");
  if (use_stereo && !enabled_disparity)
    throw std::runtime_error("Stereo/--both requires an enabled disparity algorithm; infer-only "
                             "replay uses --method original");
  const auto model = use_stereo ? regular(o.model) : fs::path{};
  fs::path runner = o.runner;
  if (!o.prepare_only) {
    runner = regular(runner);
    if (::access(runner.c_str(), X_OK))
      throw std::runtime_error("Runner is not executable");
  }
  auto overrides = object();
  for (const auto &item : std::vector<std::pair<std::string, fs::path>>{
           {"detection_model", o.detection_model}, {"segmentation_model", o.segmentation_model}})
    if (!item.second.empty()) {
      const auto p = regular(item.second);
      overrides.object[item.first] = obj({{"path", str(p.u8string())}, {"sha256", str(sha256(p))}});
    }
  const auto out = fs::absolute(o.output).lexically_normal();
  if (fs::exists(out))
    throw std::runtime_error("Output already exists: " + out.u8string());
  fs::create_directories(out.parent_path());
  if (!fs::create_directory(out))
    throw std::runtime_error("Cannot exclusively create output directory");
  Json run =
      obj({{"schema_version", integer(3)},
           {"recording", str(recording.root.u8string())},
           {"recording_manifest_sha256", str(sha256(recording.root / "manifest.jsonl"))},
           {"recording_metadata", recording.metadata},
           {"intrinsics", cameraJson(recording.calibration)},
           {"comparison_roi", comparisonRoiJson(comparison_roi,recording.calibration)},
           {"quality_analysis_scope", str("after_all_runner_measurement_windows")},
           {"frame_count", integer(recording.frames.size())},
           {"method_order", array()},
           {"requested_disparity_source", str(requested_source)},
           {"disparity_source_policy", str("Explicit comparison --method selects the source; --both generates one matching SDK configuration per method")},
           {"effective_full_configs", object()},
           {"original_disparity_generation_ms", Json{}},
           {"original_disparity_note",
            str("Original Q8 was recorded earlier; internal producer compute is unavailable")},
           {"prepared_inputs_excluded_from_compute_timing", boolean(true)},
           {"sdk_profiles", profiles},
           {"model_overrides", overrides},
           {"stereo_model",
            use_stereo ? obj({{"path", str(model.u8string())}, {"sha256", str(sha256(model))}})
                       : Json{}},
           {"evidence", str(o.prepare_only ? "prepared_only"
                                           : (field(recording.metadata, "validation_only").boolean
                                                  ? "synthetic_validation"
                                                  : "runtime_measurements"))}});
  struct utsname platform{};
  if (!::uname(&platform)) {
    run.object["platform"] = str(std::string(platform.sysname) + " " + platform.release);
    run.object["machine"] = str(platform.machine);
  }
  if (fs::is_regular_file(recording.root / "complete.json"))
    run.object["capture_summary"] = readJson(recording.root / "complete.json");
  for (const auto &method : methods)
    run.object["method_order"].array.push_back(str(method));
  run.object["benchmark"] = obj({{"warmup", integer(o.warmup)},
                                 {"repeat", integer(o.repeat)},
                                 {"save-every", integer(o.save_every)}});
  run.object["stereo_input_requested"] = use_stereo
                                             ? obj({{"input_mode", str(o.input_mode)},
                                                    {"input_width", integer(o.input_width)},
                                                    {"input_height", integer(o.input_height)}})
                                             : Json{};
  const auto prepared = out / "prepared";
  fs::create_directory(prepared);
  std::ofstream manifest(prepared / "pairs.tsv", std::ios::binary);
  auto inventory = array();
  try {
    for (const auto &frame : recording.frames) {
      const auto pair = prepared / frame.stem;
      fs::create_directory(pair);
      decodePng(frame.left, pair / "left.png", recording.calibration);
      decodePng(frame.right, pair / "right.png", recording.calibration);
      // Check immutable sources again after decoding; original NV12 is never re-encoded.
      for (const auto &item :
           std::vector<std::pair<const char *, fs::path>>{{"left_sha256", frame.left},
                                                          {"right_sha256", frame.right},
                                                          {"disparity_sha256", frame.q8}})
        if (sha256(item.second) != textValue(field(frame.identity, item.first)))
          throw std::runtime_error("Capture bytes changed during preparation");
      manifest << frame.stem << '\t' << frame.left.u8string() << '\t'
               << (pair / "left.png").u8string() << '\t' << (pair / "right.png").u8string() << '\t'
               << frame.q8.u8string() << '\t' << frame.right.u8string() << '\n';
      auto identity = frame.identity;
      identity.object["stem"] = str(frame.stem);
      identity.object["prepared_left_sha256"] = str(sha256(pair / "left.png"));
      identity.object["prepared_right_sha256"] = str(sha256(pair / "right.png"));
      inventory.array.push_back(std::move(identity));
    }
    manifest.close();
    if (!manifest)
      throw std::runtime_error("Cannot finish prepared manifest");
    writeJson(out / "input_inventory.json", inventory);
    writeJson(out / "original_start.json", original);
    full.object["mapping"].object["original_config"] =
        str((out / "original_start.json").u8string());
    // Comparison method is an explicit source selection. A --both run must not
    // pass the same frozen selector to original-Q8 and model inference runners.
    std::map<std::string, fs::path> method_configs;
    const auto base_full = full;
    for (const auto &method : methods) {
      auto selected = base_full;
      requireObject(selected.object["input"]);
      selected.object["input"].object["disparity_source"] =
          str(method == "stereo" ? "raft_stereo" : "original_q8");
      if (method == "detseg_only") {
        selected.object["mode"] = str("map");
        selected.object["mapping"].object["enabled"] = boolean(false);
        selected.object["recharge"].object["enabled"] = boolean(false);
        selected.object["history"].object["capacity"] = integer(1);
        selected.object["history"].object["difference"] = boolean(false);
        selected.object["history"].object["match"] = boolean(false);
        if (selected.object["graph"].kind == Json::Kind::Object) {
          for (auto &node : selected.object["graph"].object["nodes"].array) {
            const auto stage = graph->stage_for_task.at(textValue(field(node,"id")));
            const bool keep = stage=="dnn" || stage=="det" || stage=="seg" || stage=="publish";
            if (!keep) node.object["enabled"] = boolean(false);
          }
        }
      }
      const auto path = out / ("full_" + method + ".json");
      writeJson(path, selected);
      method_configs.emplace(method, path);
      run.object["effective_full_configs"].object[method] =
          obj({{"path", str(path.filename().u8string())}, {"sha256", str(sha256(path))},
               {"disparity_source", field(field(selected, "input"), "disparity_source")}});
      if (method == methods.front()) full = std::move(selected);
    }
    // Keep the canonical file for existing report/version-check consumers.
    writeJson(out / "full.json", full);
    run.object["effective_full_config_sha256"] = str(sha256(out / "full.json"));
    run.object["effective_mapping_config_sha256"] = str(sha256(out / "original_start.json"));
    run.object["prepared_manifest_sha256"] = str(sha256(prepared / "pairs.tsv"));
    run.object["actual_models"] = actualModels(out, run, full);
    writeJson(out / "run.json", run);
    if (o.prepare_only) {
      std::cout << "Prepared only; no algorithm or performance measurement: " << out << '\n';
      return;
    }
    auto executions = array();
    for (const auto &method : methods) {
      for (const auto &frame : recording.frames) {
        if (sha256(frame.left) != textValue(field(frame.identity, "left_sha256")) ||
            sha256(frame.right) != textValue(field(frame.identity, "right_sha256")) ||
            sha256(frame.q8) != textValue(field(frame.identity, "disparity_sha256")))
          throw std::runtime_error("Recorded input changed between comparison methods");
      }
      actualModels(out, run, full);
      std::vector<std::string> args = {runner.u8string()};
      option(args, "manifest", (prepared / "pairs.tsv").u8string());
      option(args, "sdk-config", method_configs.at(method).u8string());
      option(args, "method", method);
      option(args, "output", (out / method).u8string());
      const auto &k = recording.calibration;
      option(args, "width", std::to_string(k.width));
      option(args, "height", std::to_string(k.height));
      if(o.input_mode=="crop")
        option(args,"comparison-roi",std::to_string(comparison_roi.x)+","+
          std::to_string(comparison_roi.y)+","+std::to_string(comparison_roi.width)+","+
          std::to_string(comparison_roi.height));
      for (const auto &[key, value] : std::vector<std::pair<std::string, double>>{
               {"fx", k.fx}, {"fy", k.fy}, {"cx", k.cx}, {"cy", k.cy}, {"bxf", k.bxf}})
        option(args, key, decimal(value));
      option(args, "warmup", std::to_string(o.warmup));
      option(args, "repeat", std::to_string(o.repeat));
      option(args, "save-every", std::to_string(o.save_every));
      option(args, "bpu-enabled", o.bpu_enabled ? "1" : "0");
      option(args, "bpu-interval-ms", std::to_string(o.bpu_interval_ms));
      for (const auto &path : o.bpu_paths) option(args, "bpu-path", path);
      for (const auto &[key, value] : overrides.object)
        option(args, key == "detection_model" ? "det-model" : "seg-model",
               textValue(field(value, "path")));
      if (method == "stereo") {
        option(args, "model", model.u8string());
        option(args, "stereo-input-mode", o.input_mode);
        if (o.input_width) {
          option(args, "stereo-input-width", std::to_string(o.input_width));
          option(args, "stereo-input-height", std::to_string(o.input_height));
        }
      }
      std::cout << "Running " << method << " on " << recording.frames.size()
                << " identical recorded captures\n";
      const auto start = std::chrono::steady_clock::now();
      const int code = execute(args, out / (method + ".log"), o.sdk_root);
      actualModels(out, run, full);
      auto argv = array();
      for (const auto &a : args)
        argv.array.push_back(str(a));
      executions.array.push_back(
          obj({{"method", str(method)},
               {"arguments", argv},
               {"returncode", integer(code)},
               {"process_wall_s",
                number(std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                           .count())}}));
      writeJson(out / "executions.json", executions);
      if (code)
        throw std::runtime_error(method + " runner failed (" + std::to_string(code) + "); see " +
                                 (out / (method + ".log")).u8string());
    }
    generateReport(out, o.max_points);
    writeJson(out / "complete.json", obj({{"status", str("complete")},
                                          {"frame_count", integer(recording.frames.size())},
                                          {"methods", field(run, "method_order")}}));
    std::cout << "Comparison output: " << out << '\n';
  } catch (const std::exception &e) {
    writeJson(out / "failed.json", obj({{"error", str(e.what())}, {"status", str("failed")}}));
    throw;
  }
}
} // namespace perception_compare
