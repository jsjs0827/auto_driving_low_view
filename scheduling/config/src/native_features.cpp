#include "native_config_detail.hpp"
#include "algorithm_catalog.hpp"
#include "motion_settings_json.hpp"
#include <algorithm>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
namespace v3::detail {
namespace {
const std::vector<std::string> names = {"ros",       "offline",      "stereo",     "inference",
                                        "detection", "segmentation", "pointcloud", "fusion",
                                        "metrics",   "temporal",     "recharge",
                                        "visual_motion","motion_reference","slip_detection"};
void numericOverrides(const Json &settings,
                      const std::map<std::string, std::pair<double, double>> &ranges,
                      const std::string &name, bool positive = false) {
  for (const auto &item : ranges)
    if (explicitValue(settings, item.first))
      number(get(settings, item.first), name + "." + item.first, item.second.first,
             item.second.second, positive);
}
void countOverrides(const Json &settings, const std::map<std::string, std::pair<int, int>> &ranges,
                    const std::string &name) {
  for (const auto &item : ranges)
    if (explicitValue(settings, item.first))
      count(get(settings, item.first), name + "." + item.first, item.second.first,
            item.second.second);
}
void flagOverrides(const Json &settings, std::initializer_list<const char *> fields,
                   const std::string &name) {
  for (auto key : fields)
    if (explicitValue(settings, key))
      flag(get(settings, key), name + "." + key);
}
Json nonnull(const Json &value) {
  Json out = object();
  for (const auto &item : value.object)
    if (item.first != "_help" && item.second.kind != Json::Kind::Null)
      out.object[item.first] = item.second;
  return out;
}
std::string profileText(const Path &source, const Json &overrides) {
  std::ifstream in(source, std::ios::binary);
  if (!in)
    throw std::invalid_argument("Cannot read inference profile: " + source.u8string());
  std::ostringstream out;
  std::string row;
  std::set<std::string> seen, remaining;
  for (const auto &item : overrides.object)
    remaining.insert(item.first);
  const std::regex expression(R"(^(\s*)([A-Za-z_][A-Za-z0-9_]*)(\s*=\s*)([^#]*?)(\s*(?:#.*)?)$)");
  while (std::getline(in, row)) {
    if (!row.empty() && row.back() == '\r')
      row.pop_back();
    std::smatch m;
    if (std::regex_match(row, m, expression)) {
      const std::string key = m[2];
      if (!seen.insert(key).second)
        throw std::invalid_argument("Duplicate inference profile key: " + key);
      if (has(overrides, key)) {
        row = m[1].str() + key + m[3].str() + jsonText(get(overrides, key), 0) + m[5].str();
        remaining.erase(key);
      } else if (key == "det_model" || key == "seg_model") {
        std::string raw = m[4];
        auto first = raw.find_first_not_of(" \t"), last = raw.find_last_not_of(" \t");
        if (first != std::string::npos) {
          raw = raw.substr(first, last - first + 1);
          row = m[1].str() + key + m[3].str() + resolvePath(str(raw), source.parent_path(), key) +
                m[5].str();
        }
      }
    }
    out << row << '\n';
  }
  if (!remaining.empty())
    throw std::invalid_argument("SDK profile lacks override key: " + *remaining.begin());
  return out.str();
}
} // namespace
Json loadFeatures(const Json &requested, const Context &context, Json &source_files) {
  const Json refs = get(requested, "features", object());
  auto feature_names = names;
  for (const auto& entry : perception::algorithms::catalog())
    feature_names.push_back(entry.first);
  std::set<std::string> allowed(feature_names.begin(), feature_names.end());
  keys(refs, allowed, "features");
  Json loaded = object();
  source_files = object();
  for (const auto &name : feature_names) {
    if (perception::algorithms::find(name) && !explicitValue(refs,name)) {
      const auto candidate=context.base/"features"/name/"config.json";
      if(!std::filesystem::is_regular_file(candidate)) {
        loaded.object[name]=object();continue;
      }
    }
    const bool motion=name=="visual_motion"||name=="motion_reference"||name=="slip_detection";
    if(motion&&!explicitValue(refs,name)) {
      const auto candidate=context.base/"features"/name/"config.json";
      if(!std::filesystem::is_regular_file(candidate)){loaded.object[name]=object();continue;}
    }
    const auto location = resolvePath(get(refs, name, str("features/" + name + "/config.json")),
                                      context.base, "features." + name, "file");
    loaded.object[name] = readJson(location);
    source_files.object[name] = str(location);
  }
  return loaded;
}
void validateFeatureSettings(const Json &f, const Context &c) {
  const auto &ros = get(f, "ros");
  keys(ros,
       {"left_topic", "right_topic", "q8_topic", "gdc_topic", "left_camera_info_topic",
        "right_camera_info_topic", "reliability", "queue_capacity", "intrinsics", "rectified",
        "calibration_confirmed", "frame_id"},
       "ros");
  choice(get(ros, "reliability", str("best_effort")), {"best_effort", "reliable"},
         "ros.reliability");
  count(get(ros, "queue_capacity", integer(4)), "ros.queue_capacity", 1, 16);
  const auto intrinsic = get(ros, "intrinsics", str("camera_info"));
  if (intrinsic.kind != Json::Kind::Object &&
      !(intrinsic.kind == Json::Kind::String && intrinsic.text == "camera_info"))
    throw std::invalid_argument("ros.intrinsics must be camera_info or an intrinsics object");
  for (auto key : {"left_topic", "right_topic", "q8_topic", "gdc_topic", "left_camera_info_topic",
                   "right_camera_info_topic"})
    if (has(ros, key))
      topic(get(ros, key), "ros." + std::string(key));
  for (auto key : {"rectified", "calibration_confirmed"})
    if (has(ros, key))
      flag(get(ros, key), "ros." + std::string(key));
  if (has(ros, "frame_id"))
    text(get(ros, "frame_id"), "ros.frame_id");
  const auto &offline = get(f, "offline");
  if (has(offline, "intrinsics"))
    throw std::invalid_argument(
        "offline.intrinsics is no longer supported; move the seven "
        "calibration values to a JSON file and set offline.intrinsics_file");
  keys(offline,
       {"left_files", "right_files", "q8_files", "format", "fps", "loop", "max_frames", "wait_for_subscribers",
        "intrinsics_file", "rectified", "calibration_confirmed", "frame_id"},
       "offline");
  choice(get(offline, "format", str("nv12")), {"nv12", "i420"}, "offline.format");
  number(get(offline, "fps", num(2)), "offline.fps", .001, 120);
  count(get(offline, "max_frames", integer(0)), "offline.max_frames", 0);
  for (auto key : {"loop", "wait_for_subscribers", "rectified", "calibration_confirmed"})
    if (has(offline, key))
      flag(get(offline, key), "offline." + std::string(key));
  for (auto key : {"left_files", "right_files", "q8_files"}) {
    auto list = get(offline, key, array());
    if (list.kind != Json::Kind::Array)
      throw std::invalid_argument("offline." + std::string(key) + " must be an array");
    for (const auto &item : list.array) {
      if (c.mode == "v0_host")
        text(item, "offline." + std::string(key));
      else
        resolvePath(item, c.base, "offline." + std::string(key), c.mode == "offline" ? "file" : "");
    }
  }
  if (has(offline, "frame_id"))
    text(get(offline, "frame_id"), "offline.frame_id");
  const auto &stereo = get(f, "stereo");
  keys(stereo, {"model", "image_transform", "input_width", "input_height"}, "stereo");
  choice(get(stereo, "image_transform", str("crop")), {"crop", "resize", "strict"},
         "stereo.image_transform");
  if (explicitValue(stereo, "input_width") != explicitValue(stereo, "input_height"))
    throw std::invalid_argument("Stereo dimensions must both be null or both explicit");
  for (auto key : {"input_width", "input_height"})
    if (explicitValue(stereo, key) &&
        count(get(stereo, key), "stereo." + std::string(key), 2, 4096) % 2)
      throw std::invalid_argument("Stereo dimensions must be even");
  if ((c.cloud || c.disparity) && c.method == "raft_stereo")
    resolvePath(get(stereo, "model"), c.base, "stereo.model", "file");
  const auto &inference = get(f, "inference");
  keys(inference, {"backend", "original_profile", "cpu_profile", "dag"}, "inference");
  if (explicitValue(inference, "backend"))
    choice(get(inference, "backend"), {"bpu", "cpu"}, "inference.backend");
  if (explicitValue(inference, "dag"))
    throw std::invalid_argument("The inference DAG is generated from graph and "
                                "detection/segmentation.enabled; remove inference.dag override");
  for (auto key : {"original_profile", "cpu_profile"})
    if (explicitValue(inference, key))
      resolvePath(get(inference, key), c.base, "inference." + std::string(key), "file");
  const auto &det = get(f, "detection");
  keys(det, {"enabled", "model", "score_threshold", "nms_threshold", "nms_top_k"}, "detection");
  if (has(det, "enabled"))
    flag(get(det, "enabled"), "detection.enabled");
  if (c.detection && explicitValue(det, "model"))
    resolvePath(get(det, "model"), c.base, "detection.model", "file");
  numericOverrides(det, {{"score_threshold", {0, 1}}, {"nms_threshold", {0, 1}}}, "detection");
  if (explicitValue(det, "score_threshold")) {
    auto score = r33::json::number(get(det, "score_threshold"));
    if (score <= 0 || score >= 1)
      throw std::invalid_argument("detection.score_threshold must be >0 and <1");
  }
  countOverrides(det, {{"nms_top_k", {1, 10000}}}, "detection");
  const auto &seg = get(f, "segmentation");
  keys(seg, {"enabled", "model", "align_corners", "argmax_mode"}, "segmentation");
  if (has(seg, "enabled"))
    flag(get(seg, "enabled"), "segmentation.enabled");
  if (c.segmentation && explicitValue(seg, "model"))
    resolvePath(get(seg, "model"), c.base, "segmentation.model", "file");
  if (explicitValue(seg, "align_corners") &&
      !flag(get(seg, "align_corners"), "segmentation.align_corners"))
    throw std::invalid_argument(
        "The SDK contract requires segmentation.align_corners=true (or null)");
  if (explicitValue(seg, "argmax_mode"))
    choice(get(seg, "argmax_mode"), {"bilinear"}, "segmentation.argmax_mode");
  const auto &cloud = get(f, "pointcloud");
  keys(cloud, {"sample_step", "original_config", "extrinsics", "ground"}, "pointcloud");
  if (explicitValue(cloud, "sample_step"))
    count(get(cloud, "sample_step"), "pointcloud.sample_step", 1, 32);
  for (auto key : {"original_config", "extrinsics"})
    if (explicitValue(cloud, key))
      resolvePath(get(cloud, key), c.base, "pointcloud." + std::string(key), "file");
  auto ground = get(cloud, "ground", object());
  keys(ground, {"enabled", "sensor_height", "min_range", "max_range", "th_dist", "th_seeds"},
       "pointcloud.ground");
  flagOverrides(ground, {"enabled"}, "pointcloud.ground");
  for (auto key : {"sensor_height", "min_range", "max_range", "th_dist", "th_seeds"})
    if (explicitValue(ground, key))
      number(get(ground, key), "pointcloud.ground." + std::string(key), 0,
             std::numeric_limits<double>::max(), true);
  const auto &fusion = get(f, "fusion");
  keys(fusion, {"labels", "map"}, "fusion");
  auto labels = get(fusion, "labels", object());
  keys(labels,
       {"fuse_into_grid",
        "nongrass_seg_as_obstacle",
        "cluster_filter",
        "seg_cluster_filter",
        "seg_mask_postprocess",
        "passable_classes",
        "det_score_min",
        "max_range_m",
        "max_height_m",
        "det_depth_span_m",
        "det_depth_gap_m",
        "cluster_voxel_m",
        "seg_depth_gap_m",
        "keep_below_cam_m",
        "det_depth_min_pts",
        "cluster_min_component_pts",
        "seg_ray_bin_px",
        "seg_min_component_pts",
        "seg_mask_majority_radius",
        "seg_mask_close_radius",
        "seg_mask_close_iterations",
        "seg_mask_erode_px"},
       "fusion.labels");
  flagOverrides(labels,
                {"fuse_into_grid", "nongrass_seg_as_obstacle", "cluster_filter",
                 "seg_cluster_filter", "seg_mask_postprocess"},
                "fusion.labels");
  if (explicitValue(labels, "passable_classes"))
    text(get(labels, "passable_classes"), "fusion.labels.passable_classes");
  numericOverrides(labels,
                   {{"det_score_min", {0, 1}},
                    {"max_range_m", {.1, 100}},
                    {"max_height_m", {0, 20}},
                    {"det_depth_span_m", {0, 100}},
                    {"det_depth_gap_m", {0, 100}},
                    {"cluster_voxel_m", {.001, 10}},
                    {"seg_depth_gap_m", {0, 100}},
                    {"keep_below_cam_m", {0, 20}}},
                   "fusion.labels");
  countOverrides(labels,
                 {{"det_depth_min_pts", {1, 1000000}},
                  {"cluster_min_component_pts", {1, 1000000}},
                  {"seg_ray_bin_px", {1, 4096}},
                  {"seg_min_component_pts", {1, 1000000}},
                  {"seg_mask_majority_radius", {0, 4}},
                  {"seg_mask_close_radius", {0, 4}},
                  {"seg_mask_close_iterations", {1, 3}},
                  {"seg_mask_erode_px", {0, 8}}},
                 "fusion.labels");
  auto map = get(fusion, "map", object());
  keys(map, {"resolution_m", "width_m", "height_m"}, "fusion.map");
  for (auto key : {"resolution_m", "width_m", "height_m"})
    if (explicitValue(map, key))
      number(get(map, key), "fusion.map." + std::string(key), 0, std::numeric_limits<double>::max(),
             true);
  const auto &metrics = get(f, "metrics");
  keys(metrics, {"enabled", "write_jsonl", "every_n"}, "metrics");
  flag(get(metrics, "enabled", boolean(true)), "metrics.enabled");
  flag(get(metrics, "write_jsonl", boolean(true)), "metrics.write_jsonl");
  count(get(metrics, "every_n", integer(1)), "metrics.every_n");
  const auto &temporal = get(f, "temporal");
  keys(temporal,
       {"capacity", "frame_interval", "max_memory_mb", "difference_threshold",
        "match_iou_threshold", "match_max_detections"},
       "temporal");
  countOverrides(temporal,
                 {{"capacity", {1, 32}},
                  {"frame_interval", {1, 1000000}},
                  {"max_memory_mb", {1, 4096}},
                  {"difference_threshold", {0, 255}},
                  {"match_max_detections", {1, 1024}}},
                 "temporal");
  if (explicitValue(temporal, "match_iou_threshold"))
    number(get(temporal, "match_iou_threshold"), "temporal.match_iou_threshold", 0, 1, true);
  (void)r33::full::visualSettingsFromJson(nonnull(get(f,"visual_motion")));
  (void)r33::full::referenceSettingsFromJson(nonnull(get(f,"motion_reference")));
  (void)r33::full::slipSettingsFromJson(nonnull(get(f,"slip_detection")));
  const auto &reference=get(f,"motion_reference");
  const auto odom=text(get(reference,"odom_topic",str("")),"motion_reference.odom_topic",true);
  if(!odom.empty())(void)topic(str(odom),"motion_reference.odom_topic");
  if(c.motion_reference) {
    const bool csv=explicitValue(reference,"input_csv");
    if(csv&&!odom.empty())throw std::invalid_argument("Motion reference CSV and odom_topic are mutually exclusive");
    if(csv&&c.mode=="ros")throw std::invalid_argument("ROS motion input does not consume CSV; select odom_topic or use native camera/recording input");
    if(!odom.empty()&&c.mode!="ros")throw std::invalid_argument("motion_reference.odom_topic requires ROS input");
  }
  if(c.motion_reference&&explicitValue(reference,"input_csv")) {
    resolvePath(get(reference,"input_csv"),c.base,"motion_reference.input_csv","file");
    if(!flag(get(reference,"same_clock_confirmed",boolean(false)),"motion_reference.same_clock_confirmed")||
       text(get(reference,"clock_domain",str("")),"motion_reference.clock_domain",true).empty())
      throw std::invalid_argument("Motion CSV requires same_clock_confirmed=true and a nonempty shared clock_domain");
  }
  if(c.motion_reference&&!odom.empty()&&
      (!flag(get(reference,"same_clock_confirmed",boolean(false)),"motion_reference.same_clock_confirmed")||
       text(get(reference,"clock_domain",str("")),"motion_reference.clock_domain",true).empty()))
    throw std::invalid_argument("Motion Odometry requires confirmed shared clock_domain");
  const auto &recharge = get(f, "recharge");
  keys(recharge, {"config", "allow_source_fallback"}, "recharge");
  if (c.recharge && explicitValue(recharge, "config"))
    resolvePath(get(recharge, "config"), c.base, "recharge.config", "file");
  flagOverrides(recharge, {"allow_source_fallback"}, "recharge");
}
void applyFeatures(const Json &f, const Context &c, const Path &run, Json &full, Json &original,
                   std::map<Path, std::string> &generated) {
  auto &ai = full.object["inference"];
  auto &map = full.object["mapping"];
  // Difference/match execution is selected by graph, never by a second switch
  // in the feature settings. Only storage/sampling/analysis parameters merge.
  merge(full.object["history"], nonnull(get(f, "temporal")));
  for(const auto* feature:{"visual_motion","motion_reference","slip_detection"})
    full.object[feature]=nonnull(get(f,feature));
  auto &reference=full.object["motion_reference"];
  if(c.motion_reference&&explicitValue(reference,"input_csv"))
    reference.object["input_csv"]=str(resolvePath(get(reference,"input_csv"),c.base,"motion_reference.input_csv","file"));
  else reference.object.erase("input_csv");
  const auto &recharge = get(f, "recharge");
  if (c.recharge && explicitValue(recharge, "config"))
    full.object["recharge"].object["config"] =
        str(resolvePath(get(recharge, "config"), c.base, "recharge.config", "file"));
  if (explicitValue(recharge, "allow_source_fallback"))
    full.object["recharge"].object["allow_source_fallback"] =
        get(recharge, "allow_source_fallback");
  const auto &inference = get(f, "inference");
  if (explicitValue(inference, "backend"))
    ai.object["backend"] = get(inference, "backend");
  for (auto key : {"original_profile", "cpu_profile"})
    if (explicitValue(inference, key))
      ai.object[key] =
          str(resolvePath(get(inference, key), c.base, "inference." + std::string(key), "file"));
  for (const auto &lane : std::vector<std::pair<std::string, bool>>{
           {"detection", c.detection}, {"segmentation", c.segmentation}}) {
    const auto &config = get(f, lane.first);
    if (lane.second && explicitValue(config, "model"))
      ai.object[lane.first + "_model"] =
          str(resolvePath(get(config, "model"), c.base, lane.first + ".model", "file"));
    else
      ai.object.erase(lane.first + "_model");
  }
  const auto &pc = get(f, "pointcloud");
  for (auto key : {"original_config", "extrinsics"})
    if (explicitValue(pc, key))
      map.object[key] =
          str(resolvePath(get(pc, key), c.base, "pointcloud." + std::string(key), "file"));
  if (explicitValue(pc, "sample_step"))
    map.object["sample_step"] = get(pc, "sample_step");
  if (has(pc, "ground"))
    merge(original.object["patchwork"], nonnull(get(pc, "ground")));
  const auto &fusion = get(f, "fusion");
  if (has(fusion, "labels"))
    merge(original.object["label_fusion"], nonnull(get(fusion, "labels")));
  auto grid = get(fusion, "map", object());
  for (const auto &item : std::map<std::string, std::string>{
           {"resolution_m", "MapResolution"}, {"width_m", "MapWidth"}, {"height_m", "MapHeight"}})
    if (explicitValue(grid, item.first))
      original.object["map"].object[item.second] = get(grid, item.first);
  auto ground = get(original, "patchwork", object());
  if (explicitValue(ground, "min_range") && explicitValue(ground, "max_range") &&
      r33::json::number(get(ground, "min_range")) >= r33::json::number(get(ground, "max_range")))
    throw std::invalid_argument("Resolved ground.min_range must be below max_range");
  Json thresholds = object();
  for (auto key : {"score_threshold", "nms_threshold", "nms_top_k"})
    if (c.detection && explicitValue(get(f, "detection"), key))
      thresholds.object[key] = get(get(f, "detection"), key);
  const auto backend =
      choice(get(ai, "backend", str("bpu")), {"cpu", "bpu"}, "SDK inference.backend");
  if (c.inference && backend == "cpu") {
    if (!explicitValue(ai, "cpu_profile"))
      throw std::invalid_argument("CPU inference requires a real cpu_profile");
    Path source =
        std::filesystem::u8path(resolvePath(get(ai, "cpu_profile"), c.base, "cpu_profile", "file"));
    Json cpu = readJson(source);
    for (const auto &lane : std::vector<std::pair<std::string, bool>>{
             {"detection", c.detection}, {"segmentation", c.segmentation}}) {
      auto &config = cpu.object[lane.first];
      if (config.kind != Json::Kind::Object)
        throw std::invalid_argument("CPU profile lacks " + lane.first);
      const auto model = get(ai, lane.first + "_model", get(config, "model"));
      config.object["model"] = str(resolvePath(
          model, source.parent_path(), "CPU " + lane.first + " model", lane.second ? "file" : ""));
    }
    merge(cpu.object["detection"], thresholds);
    const Path target = run / "inference_cpu.json";
    generated[target] = jsonText(cpu) + "\n";
    ai.object["cpu_profile"] = str(target.u8string());
  } else if (c.inference && backend == "bpu") {
    if (explicitValue(ai, "cpu_profile"))
      throw std::invalid_argument("cpu_profile requires inference.backend=cpu");
    Path source = std::filesystem::u8path(
        resolvePath(get(ai, "original_profile"), c.base, "original_profile", "file"));
    const Path target = run / "inference_original.cfg";
    generated[target] = profileText(source, thresholds);
    ai.object["original_profile"] = str(target.u8string());
  }
}
Json generatedInferenceDag(const Context &c, int workers) {
  Json root = object(), engine = object(), tasks = object(), sequence = array(),
       artifacts = array();
  root.object["config_version"] = integer(3);
  engine.object["worker_threads"] = integer(workers);
  Json pack = array();
  pack.array.push_back(str("input.exact.pack"));
  tasks.object["input"] = object();
  tasks.object["input"].object["features"] = pack;
  sequence.array.push_back(str("input"));
  Json lanes = array();
  for (const auto &lane : std::vector<std::pair<std::string, bool>>{
           {"detection", c.detection}, {"segmentation", c.segmentation}})
    if (lane.second) {
      Json features = array();
      const std::string code = lane.first == "detection" ? "det" : "seg";
      for (auto stage : {"preprocess", "backend", "postprocess"})
        features.array.push_back(str("ai.exact." + code + "." + stage));
      tasks.object[lane.first] = object();
      tasks.object[lane.first].object["features"] = features;
      lanes.array.push_back(str(lane.first));
      artifacts.array.push_back(str("ai." + lane.first));
      if (code == "det")
        artifacts.array.push_back(str("exact.detections"));
    }
  if (lanes.array.size() > 1) {
    Json parallel = object();
    parallel.object["parallel"] = lanes;
    sequence.array.push_back(parallel);
  } else if (!lanes.array.empty())
    sequence.array.push_back(lanes.array.front());
  // Disparity is a full graph node in v3; never schedule stereo.demand a second time.
  engine.object["result_artifacts"] = artifacts;
  root.object["engine"] = engine;
  root.object["tasks"] = tasks;
  root.object["dag"] = object();
  root.object["dag"].object["sequence"] = sequence;
  return root;
}
Json cameraSettings(const Json &requested, const Context &c, bool check, const std::string &sdk,
                    const std::string &cfg, int forced_timeout) {
  const auto file = resolvePath(get(requested, "camera_config", str("camera.json")), c.base,
                                "camera_config", "file");
  Json settings = readJson(file);
  keys(settings,
       {"sdk_library", "vendor_config", "disparity_mode", "acquire_timeout_ms", "retry_interval_ms",
        "frame_id", "shm"},
       "camera");
  if (!sdk.empty())
    settings.object["sdk_library"] = str(sdk);
  if (!cfg.empty())
    settings.object["vendor_config"] = str(cfg);
  if (c.mode == "capture") {
    settings.object["disparity_mode"] = integer(2);
    if (forced_timeout)
      settings.object["acquire_timeout_ms"] = integer(forced_timeout);
    settings.object["retry_interval_ms"] =
        integer(std::min(5, count(get(settings, "acquire_timeout_ms", integer(3000)),
                                  "capture.timeout_ms", 1, 60000)));
  }
  const int timeout = count(get(settings, "acquire_timeout_ms", integer(3000)),
                            "camera.acquire_timeout_ms", 1, 60000),
            retry = count(get(settings, "retry_interval_ms", integer(5)),
                          "camera.retry_interval_ms", 1, 1000);
  if (retry > timeout)
    throw std::invalid_argument("camera.retry_interval_ms exceeds acquire_timeout_ms");
  auto mode = get(settings, "disparity_mode", str("auto"));
  int value;
  if (mode.kind == Json::Kind::String && mode.text == "auto")
    value = (c.cloud || c.disparity) && c.method == "original_q8" ? 2 : 1;
  else
    value = count(mode, "camera.disparity_mode", 1, 2);
  if ((c.mode == "camera" || c.mode == "camera_shm" || c.mode == "capture") && (c.cloud || c.disparity) &&
      c.method == "original_q8" && value != 2)
    throw std::invalid_argument("Original Q8 camera requires disparity_mode=2 or auto");
  if(c.mode == "camera_shm") {
    // Mode2 is explicit compatibility for SDKs whose RGBD API needs internal disparity.
    // The DAG still consumes left/right and runs its configured stereo provider.
    if(!has(settings,"shm"))throw std::invalid_argument("camera_shm requires explicit camera.shm configuration");
    auto shared=get(settings,"shm");
    keys(shared,{"name","slots","yuv_format","raw_profile_confirmed","max_capture_attempts",
      "cross_stream_sync","max_cross_stream_skew_ns","capture_frame_period_ns"},"camera.shm");
    const auto name=text(get(shared,"name",str("/perception_v2_camera_bundle")),"camera.shm.name");
    if(name.size()<2||name.size()>200||name.front()!='/'||name.find('/',1)!=std::string::npos||name.find('\0')!=std::string::npos)
      throw std::invalid_argument("camera.shm.name must be a single POSIX /name");
    shared.object["name"]=str(name);
    shared.object["slots"]=integer(count(get(shared,"slots",integer(3)),"camera.shm.slots",2,16));
    shared.object["yuv_format"]=str(choice(get(shared,"yuv_format"),{"nv12","i420"},"camera.shm.yuv_format"));
    const bool profile=flag(get(shared,"raw_profile_confirmed",boolean(false)),"camera.shm.raw_profile_confirmed");
    if(!profile)throw std::invalid_argument("camera_shm requires explicit confirmation of native RAW16LE DN10 BGGR1088x1280 profile");
    shared.object["raw_profile_confirmed"]=boolean(profile);
    shared.object["max_capture_attempts"]=integer(count(get(shared,"max_capture_attempts",integer(4)),"camera.shm.max_capture_attempts",1,64));
    const auto sync=choice(get(shared,"cross_stream_sync",str("exact_timestamps")),
      {"exact_timestamps","sdk_frame_id_bounded"},"camera.shm.cross_stream_sync");
    const int skew=count(get(shared,"max_cross_stream_skew_ns",integer(0)),"camera.shm.max_cross_stream_skew_ns",0,20000000);
    const int period=count(get(shared,"capture_frame_period_ns",integer(0)),"camera.shm.capture_frame_period_ns",0,1000000000);
    if((sync=="exact_timestamps" && skew!=0) ||
       (sync=="sdk_frame_id_bounded" && (skew==0 || period<=skew)))
      throw std::invalid_argument("camera.shm bounded association needs 0<max_cross_stream_skew_ns<capture_frame_period_ns; exact requires max=0");
    shared.object["cross_stream_sync"]=str(sync);
    shared.object["max_cross_stream_skew_ns"]=integer(skew);
    shared.object["capture_frame_period_ns"]=integer(period);
    settings.object["shm"]=shared;
  }
  settings.object["disparity_mode"] = integer(value);
  settings.object["acquire_timeout_ms"] = integer(timeout);
  settings.object["retry_interval_ms"] = integer(retry);
  settings.object["frame_id"] =
      str(text(get(settings, "frame_id", str("camera_optical_frame")), "camera.frame_id"));
  for (auto key : {"sdk_library", "vendor_config"})
    if (check)
      settings.object[key] =
          str(resolvePath(get(settings, key), c.base, "camera." + std::string(key), "file"));
  return settings;
}
} // namespace v3::detail
