#include "intrinsics_file.hpp"
#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <regex>
#include <sstream>
namespace v3 {
namespace {
using namespace detail;
struct GraphInfo {
  Json json;
  std::map<std::string, bool> enabled;
  int workers = 2;
};
GraphInfo validateGraph(const Json &graph) {
  keys(graph, {"worker_threads", "nodes"}, "graph");
  count(get(graph, "worker_threads", integer(2)), "graph.worker_threads", 1, 32);
  const auto plan = r33::full::parseProductGraph(graph);
  GraphInfo result;
  result.json = graph;
  result.workers = plan.config.worker_threads;
  result.json.object["worker_threads"] = integer(result.workers);
  for (const auto &stage : r33::full::productStages())
    result.enabled[std::string(stage.key) == "dnn" ? "inference" : stage.key] =
        plan.enabled(stage.key);
  for (auto &node : result.json.object["nodes"].array) {
    keys(node, {"id", "feature", "algorithm", "inputs", "depends_on", "enabled"}, "graph node");
    if (!has(node, "enabled"))
      node.object["enabled"] = boolean(true);
    if (!has(node, "depends_on"))
      node.object["depends_on"] = array();
  }
  return result;
}
void checkRoot(const Json &requested) {
  keys(requested,
       {"schema_version", "input", "graph", "sdk", "features", "camera_config",
        "visualization_config", "rviz_config", "output", "runtime", "recording"},
       "pipeline");
  count(get(requested, "schema_version"), "schema_version", 3, 4);
  keys(get(requested, "input"), {"mode", "disparity_source", "timeline_csv"}, "input");
  if (has(requested, "recording")) {
    if (has(get(requested, "recording"), "intrinsics"))
      throw std::invalid_argument("recording.intrinsics is no longer supported; set "
                                  "recording.intrinsics_file to the calibration JSON file or "
                                  "recording metadata.json");
    keys(get(requested, "recording"),
         {"directory", "limit", "intrinsics_file", "warmup_frames", "repeat", "save_every_n",
          "viewer_max_points", "runner", "order", "suite", "bpu"},
         "recording");
  }
}
Json visualSettings(const Json &requested, const Context &c, Json &files) {
  const auto path = resolvePath(get(requested, "visualization_config", str("visualization.json")),
                                c.base, "visualization_config", "file");
  auto visual = readJson(path);
  files.object["visualization"] = str(path);
  const std::set<std::string> allowed = {"save_root",       "save_every_n",   "save_ply",
                                         "publish_every_n", "publish_clouds", "publish_maps",
                                         "publish_mask",    "publish_input",  "publish_disparity",
                                         "render_maps",     "robot_frame_id"};
  keys(visual, allowed, "visualization");
  if (has(requested, "output")) {
    const auto &output = get(requested, "output");
    keys(output, allowed, "output");
    merge(visual, output);
  }
  for (auto key : {"save_ply", "publish_clouds", "publish_maps", "publish_mask", "publish_input",
                   "publish_disparity", "render_maps"})
    if (has(visual, key))
      flag(get(visual, key), "visualization." + std::string(key));
  count(get(visual, "save_every_n", integer(0)), "visualization.save_every_n", 0);
  count(get(visual, "publish_every_n", integer(1)), "visualization.publish_every_n");
  text(get(visual, "robot_frame_id", str("base_link")), "visualization.robot_frame_id");
  if (has(visual, "save_root"))
    visual.object["save_root"] =
        str(resolvePath(get(visual, "save_root"), c.base, "visualization.save_root", {}, true));
  return visual;
}
void normalizeSDK(Json &full, const Path &base) {
  for (auto section : {"inference", "mapping", "recharge"})
    if (get(full, section).kind != Json::Kind::Object)
      throw std::invalid_argument("SDK full configuration requires " + std::string(section) +
                                  " object");
  if (!has(full, "output"))
    full.object["output"] = object();
  keys(get(full, "output"), {"directory", "every_n", "save_overlay", "save_inputs"}, "SDK output");
  for (auto section : {"inference", "mapping", "recharge"})
    for (auto key :
         {"original_profile", "dag", "detection_dag", "segmentation_dag", "cpu_profile", "detection_model", "segmentation_model",
          "original_config", "extrinsics", "calibration_output", "config"})
      if (explicitValue(get(full, section), key))
        full.object[section].object[key] = str(resolvePath(
            get(get(full, section), key), base, "SDK " + std::string(section) + "." + key));
}
void calibration(const Json &source, const Context &c, bool recharge_frame_intrinsics, Json &params,
                 Json &files, Json &evidence) {
  params.object["use_camera_info"] = boolean(false);
  evidence = object();
  if (c.mode == "camera") {
    if (c.cloud || c.disparity) {
      params.object["stereo_rectified"] = boolean(true);
      params.object["calibration_confirmed"] = boolean(true);
    }
    evidence.object["source"] = str("camera_sdk");
    return;
  }
  if (!c.cloud && !c.disparity && !recharge_frame_intrinsics) {
    evidence.object["source"] = str("unused");
    return;
  }
  const bool rectified = flag(get(source, "rectified", boolean(false)), "input.rectified"),
             confirmed = flag(get(source, "calibration_confirmed", boolean(false)),
                              "input.calibration_confirmed");
  params.object["stereo_rectified"] = boolean(rectified);
  params.object["calibration_confirmed"] = boolean(confirmed);
  auto intrinsic = get(source, "intrinsics", str("camera_info"));
  if (c.mode == "offline") {
    if (!explicitValue(source, "intrinsics_file"))
      throw std::invalid_argument("Offline geometry requires offline.intrinsics_file; specify "
                                  "the calibration JSON file for these source images");
    const auto path =
        resolvePath(get(source, "intrinsics_file"), c.base, "offline.intrinsics_file", "file");
    intrinsic = loadIntrinsicsFile(std::filesystem::u8path(path), true);
    params.object["intrinsics_file"] = str(path);
    files.object["intrinsics"] = str(path);
    evidence.object["source"] = str("intrinsics_file");
    evidence.object["file"] = str(path);
  }
  if ((c.cloud || c.disparity) && c.method == "raft_stereo" && (!rectified || !confirmed))
    throw std::invalid_argument("Stereo requires confirmed rectified input");
  if (intrinsic.kind == Json::Kind::String && intrinsic.text == "camera_info") {
    if (c.mode != "ros")
      throw std::invalid_argument("camera_info intrinsics require ROS input");
    params.object["use_camera_info"] = boolean(true);
    evidence.object["source"] = str("ros_camera_info");
    return;
  }
  keys(intrinsic, {"width", "height", "fx", "fy", "cx", "cy", "bxf"}, "input.intrinsics");
  if (!rectified || !confirmed)
    throw std::invalid_argument("Explicit intrinsics require rectified/calibration_confirmed=true");
  if (count(get(intrinsic, "width"), "intrinsics.width", 640, 640) != 640 ||
      count(get(intrinsic, "height"), "intrinsics.height", 544, 544) != 544)
    throw std::invalid_argument("Intrinsics must describe 640x544 source");
  for (auto key : {"fx", "fy", "bxf"}) {
    number(get(intrinsic, key), "intrinsics." + std::string(key), 0,
           std::numeric_limits<double>::max(), true);
    params.object[key] = get(intrinsic, key);
  }
  for (auto key : {"cx", "cy"}) {
    const double limit = std::string(key) == "cx" ? 640 : 544;
    const double value = number(get(intrinsic, key), "intrinsics." + std::string(key), 0, limit);
    if (value >= limit)
      throw std::invalid_argument("Principal point outside source");
    params.object[key] = get(intrinsic, key);
  }
  if (!has(evidence, "source"))
    evidence.object["source"] = str("ros_explicit_intrinsics");
  evidence.object["intrinsics"] = intrinsic;
}
Json resolvedRecording(const Json &root, const Context &c, Json &files, Json &evidence) {
  const bool motion_input=c.visual_motion||c.motion_reference||c.slip_detection;
  Json out = get(root, "recording", object());
  keys(out,
       {"directory", "limit", "intrinsics_file", "warmup_frames", "repeat", "save_every_n",
        "viewer_max_points", "runner", "order", "suite", "bpu"},
       "recording");
  Json default_directory=str("${CAPTURE_ROOT}");
  if(explicitValue(get(root,"input"),"timeline_csv"))
    default_directory=str(std::filesystem::u8path(resolvePath(get(get(root,"input"),"timeline_csv"),c.base,"input.timeline_csv","file")).parent_path().u8string());
  out.object["directory"] = str(resolvePath(get(out, "directory", default_directory), c.base,
                                            "recording.directory", "directory"));
  for (const auto &item : std::map<std::string, int>{{"limit", 100},
                                                     {"warmup_frames", motion_input?0:5},
                                                     {"repeat", motion_input?1:3},
                                                     {"save_every_n", motion_input?0:10},
                                                     {"viewer_max_points", 20000}}) {
    const int low = (item.first == "warmup_frames" || item.first == "save_every_n" || (motion_input&&item.first=="limit")) ? 0 : 1,
              high = item.first == "viewer_max_points" ? 200000 : 1000000;
    out.object[item.first] = integer(
        count(get(out, item.first, integer(item.second)), "recording." + item.first, low, high));
  }
  if(motion_input&&(get(out,"warmup_frames").text!="0"||get(out,"repeat").text!="1"))
    throw std::invalid_argument("Motion recording requires warmup_frames=0 and repeat=1; keep one original capture epoch");
  evidence = object();
  if (c.cloud || c.disparity) {
    if (!explicitValue(out, "intrinsics_file"))
      throw std::invalid_argument(
          "Recording geometry requires recording.intrinsics_file; "
          "explicitly select the calibration JSON or recording metadata.json");
    const auto path =
        resolvePath(get(out, "intrinsics_file"), c.base, "recording.intrinsics_file", "file");
    out.object["intrinsics_file"] = str(path);
    out.object["intrinsics"] = loadIntrinsicsFile(std::filesystem::u8path(path), false);
    files.object["intrinsics"] = str(path);
    evidence.object["source"] = str("intrinsics_file");
    evidence.object["file"] = str(path);
    evidence.object["intrinsics"] = get(out, "intrinsics");
  } else {
    // Inference-only comparison still accepts the capture metadata internally,
    // but an unused user-supplied path is neither opened nor required.
    out.object["intrinsics"] = str("recording");
    evidence.object["source"] = str("unused");
  }
  if (explicitValue(out, "runner"))
    out.object["runner"] = str(resolvePath(get(out, "runner"), c.base, "recording.runner", "file"));
  const bool all = choice(get(out,"suite",str("selected")), {"selected","three_way"}, "recording.suite") == "three_way";
  out.object["suite"] = str(all ? "three_way" : "selected");
  if(all && (!(c.cloud || c.disparity) || c.method != "raft_stereo" || !c.inference))
    throw std::invalid_argument("three_way suite requires det+seg, enabled stereo/cloud graph and raft_stereo source");
  const auto requested_order = get(out, "order", str(all ? "detseg_only,original,stereo" : "original,stereo"));
  out.object["order"] = str(all ? choice(requested_order,
      {"detseg_only,original,stereo","detseg_only,stereo,original", "original,detseg_only,stereo",
        "original,stereo,detseg_only","stereo,detseg_only,original","stereo,original,detseg_only"}, "recording.order")
      : choice(requested_order,{"original,stereo", "stereo,original"}, "recording.order"));
  auto bpu = get(out,"bpu",object());
  keys(bpu,{"enabled","sample_interval_ms","paths"},"recording.bpu");
  bpu.object["enabled"] = boolean(flag(get(bpu,"enabled",boolean(true)),"recording.bpu.enabled"));
  bpu.object["sample_interval_ms"] = integer(count(get(bpu,"sample_interval_ms",integer(100)),"recording.bpu.sample_interval_ms",10,60000));
  bpu.object["paths"] = get(bpu,"paths",array());
  if(get(bpu,"paths").kind != Json::Kind::Array) throw std::invalid_argument("recording.bpu.paths must be array");
  for(const auto &p:get(bpu,"paths").array) if(text(p,"recording.bpu.paths item").empty()) throw std::invalid_argument("BPU path cannot be empty");
  out.object["bpu"] = std::move(bpu);
  return out;
}
std::string yamlParameters(const Json &params) {
  // ROS parameters have declared scalar types. JSON has one numeric kind, but
  // YAML's plain "2" becomes an integer and cannot initialize a ROS double.
  static const std::set<std::string> doubles = {"offline_fps", "fx", "fy", "cx", "cy", "bxf"};
  std::ostringstream out;
  out << "/**:\n  ros__parameters:\n";
  for (const auto &item : params.object) {
    auto scalar = jsonText(item.second, 0);
    if (doubles.count(item.first)) {
      r33::json::number(item.second);
      if (scalar.find_first_of(".eE") == std::string::npos)
        scalar += ".0";
    }
    out << "    " << item.first << ": " << scalar << '\n';
  }
  return out.str();
}
std::string env(const char *key) {
  const char *value = std::getenv(key);
  return value ? value : "";
}
Path applicationPrefix(const Context &context) {
  if (!env("INSTALL_ROOT").empty())
    return std::filesystem::u8path(
        resolvePath(str(env("INSTALL_ROOT")), context.base, "INSTALL_ROOT"));
  return context.project / "install/perception_ros_so_demo";
}
} // namespace
PreparedPipeline prepare(const Path &configuration, const Path &output, bool create_files) {
  const Path file = std::filesystem::weakly_canonical(std::filesystem::absolute(configuration));
  PreparedPipeline out;
  out.requested = readJson(file);
  checkRoot(out.requested);
  const auto base = file.parent_path(), project = projectRoot(file);
  out.project_root = project.u8string();
  Context c{base, project};
  c.mode = choice(get(get(out.requested, "input"), "mode", str("camera")),
                  {"camera", "ros", "offline", "recording"}, "input.mode");
  c.method = choice(get(get(out.requested, "input"), "disparity_source", str("raft_stereo")),
                    {"original_q8", "raft_stereo"}, "input.disparity_source");
  out.mode = c.mode;
  out.method = c.method;
  auto graph = validateGraph(get(out.requested, "graph"));
  out.graph = graph.json;
  c.cloud = graph.enabled["cloud"];
  c.disparity = graph.enabled["disparity"];
  c.recharge = graph.enabled["recharge"];
  if ((c.mode == "recording" || c.mode == "offline") && c.recharge)
    throw std::invalid_argument(c.mode + " recharge requires a genuine same-capture "
                                         "high-resolution input and is unsupported");
  c.inference = graph.enabled["inference"] || graph.enabled["det"] || graph.enabled["seg"];
  c.visual_motion=graph.enabled["visual_motion"];c.motion_reference=graph.enabled["motion_reference"];
  c.slip_detection=graph.enabled["slip_detection"];
  Json source_files;
  out.features = loadFeatures(out.requested, c, source_files);
  c.detection = (graph.enabled["inference"] || graph.enabled["det"]) && flag(get(get(out.features, "detection"), "enabled", boolean(true)),
                                    "detection.enabled");
  c.segmentation =
      (graph.enabled["inference"] || graph.enabled["seg"]) && flag(get(get(out.features, "segmentation"), "enabled", boolean(true)),
                          "segmentation.enabled");
  // The legacy combined entry must contain a model lane. Split instances can
  // both be pruned by feature switches; their remaining graph is validated below.
  if (graph.enabled["inference"] && !c.detection && !c.segmentation)
    throw std::invalid_argument(
        "inference is enabled but both detection/segmentation lanes are disabled");
  // An algorithm instance needs both its graph switch and feature switch. Freeze
  // that effective selection in the graph so runtime and dependency checks agree.
  for (auto &node : graph.json.object["nodes"].array) {
    const auto algorithm = has(node, "algorithm") ? get(node, "algorithm").text :
        r33::full::productStageKey(get(node, "feature").text);
    if ((algorithm == "det" && !c.detection) || (algorithm == "seg" && !c.segmentation))
      node.object["enabled"] = boolean(false);
  }
  graph = validateGraph(graph.json);
  out.graph = graph.json;
  c.inference = graph.enabled["inference"] || graph.enabled["det"] || graph.enabled["seg"];
  validateFeatureSettings(out.features, c);
  const Json sdk = get(out.requested, "sdk", object());
  keys(sdk, {"root", "base_config"}, "sdk");
  out.sdk_root = resolvePath(get(sdk, "root", str("${SDK_ROOT}")), base, "sdk.root", "directory");
  const Path sdk_full = std::filesystem::u8path(resolvePath(
      (explicitValue(sdk, "base_config")
           ? get(sdk, "base_config")
           : str((std::filesystem::u8path(out.sdk_root) / "config/full.json").u8string())),
      base, "sdk.base_config", "file"));
  out.full = readJson(sdk_full);
  normalizeSDK(out.full, sdk_full.parent_path());
  const Path run = std::filesystem::absolute(output).lexically_normal();
  if (output.empty())
    throw std::invalid_argument("--run-dir must be a new directory");
  if (std::filesystem::exists(run))
    throw std::invalid_argument("Run directory already exists: " + run.u8string());
  out.run_directory = run.u8string();
  const auto visual = visualSettings(out.requested, c, source_files);
  const auto camera = cameraSettings(out.requested, c, c.mode == "camera");
  source_files.object["camera"] = str(resolvePath(
      get(out.requested, "camera_config", str("camera.json")), base, "camera_config", "file"));
  auto &ai = out.full.object["inference"];
  auto &mapping = out.full.object["mapping"];
  for (auto key : {"original_config", "extrinsics"})
    if (explicitValue(get(out.features, "pointcloud"), key))
      mapping.object[key] = str(resolvePath(get(get(out.features, "pointcloud"), key), base,
                                            "pointcloud." + std::string(key), "file"));
  if (explicitValue(mapping, "original_config"))
    out.original = readJson(resolvePath(get(mapping, "original_config"), sdk_full.parent_path(),
                                        "mapping.original_config", "file"));
  else if (c.cloud)
    throw std::invalid_argument("Cloud work requires mapping.original_config");
  else
    out.original = object();
  for (auto section : {"patchwork", "map", "label_fusion", "dnn", "debug", "point_cloud_color"}) {
    if (!has(out.original, section))
      out.original.object[section] = object();
    if (get(out.original, section).kind != Json::Kind::Object)
      throw std::invalid_argument("Mapping section must be an object: " + std::string(section));
  }
  out.full.object["graph"] = out.graph;
  ai.object["enabled"] = boolean(c.inference);
  mapping.object["enabled"] = boolean(c.cloud);
  mapping.object["retain_clouds"] = boolean(c.cloud);
  mapping.object["render_maps"] = get(visual, "render_maps", boolean(true));
  if (!has(mapping, "sample_step"))
    mapping.object["sample_step"] = integer(4);
  out.full.object["recharge"].object["enabled"] = boolean(graph.enabled["recharge"]);
  const bool geometry = c.cloud || c.disparity;
  out.full.object["input"] = object();
  out.full.object["input"].object["disparity_source"] =
      str(geometry ? c.method : "original_q8");
  const bool active_perception = geometry || c.inference || graph.enabled["rgb"];
  out.full.object["mode"] =
      str(c.recharge ? (active_perception ? "map_and_recharge" : "recharge") : "map");
  auto history = get(out.full, "history", object());
  if (!has(history, "capacity"))
    history.object["capacity"] = integer(1);
  if (!has(history, "frame_interval"))
    history.object["frame_interval"] = integer(1);
  if (!has(history, "max_memory_mb"))
    history.object["max_memory_mb"] = integer(64);
  history.object["difference"] = boolean(graph.enabled["temporal_difference"]);
  history.object["match"] = boolean(graph.enabled["temporal_match"]);
  out.full.object["history"] = history;
  out.original.object["debug"].kind = Json::Kind::Object;
  merge(out.original.object["debug"],
        r33::json::JsonParser(
            "{\"visualize\":true,\"save_log\":false,\"save_map\":false,\"save_data\":false}",
            "defaults")
            .parse());
  out.original.object["point_cloud_color"].kind = Json::Kind::Object;
  out.original.object["point_cloud_color"].object["enabled"] = boolean(true);
  out.original.object["point_cloud_color"].object["use_gdc_image"] = boolean(false);
  out.original.object["label_fusion"].kind = Json::Kind::Object;
  out.original.object["label_fusion"].object["enabled"] = boolean(graph.enabled["labels"]);
  out.original.object["dnn"].kind = Json::Kind::Object;
  out.original.object["dnn"].object["enabled"] = boolean(c.inference);
  out.original.object["dnn"].object["run_every_n"] = integer(1);
  for (const auto &lane : std::vector<std::pair<std::string, bool>>{{"detect", c.detection},
                                                                    {"seg", c.segmentation}}) {
    out.original.object["dnn"].object[lane.first].kind = Json::Kind::Object;
    out.original.object["dnn"].object[lane.first].object["enabled"] = boolean(lane.second);
  }
  out.original.object["auto_cali"] = boolean(graph.enabled["calibration"]);
  std::map<Path, std::string> generated;
  applyFeatures(out.features, c, run, out.full, out.original, generated);
  if ((graph.enabled["temporal_difference"] || graph.enabled["temporal_match"] || c.visual_motion) &&
      count(get(get(out.full, "history"), "capacity"), "temporal.capacity", 1, 32) < 2)
    throw std::invalid_argument("Temporal motion/difference/match require temporal.capacity >= 2");
  out.original.object["label_fusion"].object["enabled"] = boolean(graph.enabled["labels"]);
  if (!graph.enabled["ground"])
    out.original.object["patchwork"].object["enabled"] = boolean(false);
  else if (explicitValue(get(get(out.features, "pointcloud"), "ground", object()), "enabled") &&
           !flag(get(get(get(out.features, "pointcloud"), "ground"), "enabled"), "ground.enabled"))
    throw std::invalid_argument("graph ground is enabled but pointcloud.ground.enabled=false; "
                                "disable graph ground explicitly");
  if (c.cloud)
    resolvePath(get(mapping, "extrinsics"), sdk_full.parent_path(), "mapping.extrinsics", "file");
  const Path inference_dag = run / "inference_dag.json";
  generated[inference_dag] = jsonText(generatedInferenceDag(c, graph.workers)) + "\n";
  ai.object["dag"] = str(inference_dag.u8string());
  Context detection_only = c, segmentation_only = c;
  detection_only.segmentation = false;
  segmentation_only.detection = false;
  const Path detection_dag = run / "inference_detection_dag.json",
             segmentation_dag = run / "inference_segmentation_dag.json";
  // Each lane has one sequential preprocess/backend/postprocess task. Product
  // graph workers provide inter-model parallelism; extra lane workers add no work.
  generated[detection_dag] = jsonText(generatedInferenceDag(detection_only, 1)) + "\n";
  generated[segmentation_dag] = jsonText(generatedInferenceDag(segmentation_only, 1)) + "\n";
  ai.object["detection_dag"] = str(detection_dag.u8string());
  ai.object["segmentation_dag"] = str(segmentation_dag.u8string());
  mapping.object["original_config"] = str((run / "original_start.json").u8string());
  Json params = object();
  params.object["input_mode"] = str(c.mode);
  params.object["mode"] =
      str(c.recharge ? (active_perception ? "both" : "recharge") : (c.cloud ? "map" : "infer"));
  params.object["mapping_enabled"] = boolean(c.cloud);
  params.object["disparity_enabled"] = boolean(c.disparity);
  params.object["disparity_source"] = str(geometry ? c.method : "original_q8");
  params.object["sdk_config"] = str((run / "effective_full.json").u8string());
  if(explicitValue(out.requested,"rviz_config"))
    params.object["rviz_config"]=str(resolvePath(get(out.requested,"rviz_config"),base,"rviz_config","file"));
  params.object["publish_enabled"] = boolean(graph.enabled["publish"]);
  params.object["record_enabled"] = boolean(graph.enabled["record"]);
  const Json source = c.mode == "ros" ? get(out.features, "ros") : get(out.features, "offline");
  params.object["frame_id"] =
      get(c.mode == "camera" ? camera : source, "frame_id", str("camera_optical_frame"));
  out.stereo = object();
  if (geometry && c.method == "raft_stereo") {
    const auto &stereo = get(out.features, "stereo");
    out.stereo.object["model"] =
        str(resolvePath(get(stereo, "model"), base, "stereo.model", "file"));
    out.stereo.object["input_mode"] = get(stereo, "image_transform", str("crop"));
    for (auto key : {"input_width", "input_height"})
      out.stereo.object[key] = explicitValue(stereo, key) ? get(stereo, key) : integer(0);
    params.object["stereo_model"] = get(out.stereo, "model");
    params.object["stereo_input_mode"] = get(out.stereo, "input_mode");
    params.object["stereo_input_width"] = get(out.stereo, "input_width");
    params.object["stereo_input_height"] = get(out.stereo, "input_height");
  }
  Json calibration_evidence;
  const bool recharge_frame_intrinsics =
      c.mode == "ros" && !geometry && c.recharge &&
      rechargeUsesFrameIntrinsics(out.full, sdk_full.parent_path());
  if (c.mode != "recording")
    calibration(source, c, recharge_frame_intrinsics, params, source_files, calibration_evidence);
  if (c.mode == "camera") {
    params.object["camera_sdk"] = get(camera, "sdk_library");
    params.object["camera_config"] = get(camera, "vendor_config");
    params.object["camera_disparity_mode"] = get(camera, "disparity_mode");
    params.object["acquire_timeout_ms"] = get(camera, "acquire_timeout_ms");
    params.object["retry_interval_ms"] = get(camera, "retry_interval_ms");
  }
  if (c.mode == "ros") {
    const auto &ros = get(out.features, "ros");
    params.object["input_reliability"] = get(ros, "reliability", str("best_effort"));
    params.object["pair_capacity"] = get(ros, "queue_capacity", integer(4));
    params.object["left_topic"] =
        str(topic(get(ros, "left_topic", str("/camera/rgbd_left_nv12")), "ros.left_topic"));
    if (c.recharge)
      params.object["gdc_topic"] =
          str(topic(get(ros, "gdc_topic", str("/camera/gdc_left_nv12")), "ros.gdc_topic"));
    if (geometry) {
      const auto key = c.method == "raft_stereo" ? "right_topic" : "q8_topic";
      params.object[key] =
          get(ros, key,
              str(c.method == "raft_stereo" ? "/camera/rgbd_right_nv12" : "/camera/disparity_q8"));
      if (c.method == "raft_stereo" && get(params, key).text == get(params, "left_topic").text)
        throw std::invalid_argument("Left and right topics must differ");
    }
    if (flag(get(params, "use_camera_info", boolean(false)), "use_camera_info")) {
      params.object["left_camera_info_topic"] =
          get(ros, "left_camera_info_topic", str("/stereo/left/camera_info"));
      params.object["right_camera_info_topic"] =
          get(ros, "right_camera_info_topic", str("/stereo/right/camera_info"));
    }
  }
  if (c.mode == "offline") {
    if (geometry && c.method != "raft_stereo")
      throw std::invalid_argument(
          "Offline mapping needs paired stereo; genuine original Q8 uses recording mode");
    if (graph.enabled["recharge"])
      throw std::invalid_argument("Offline recharge requires a genuine same-capture "
                                  "high-resolution input and is unsupported");
    for (auto key : {"left_files", "right_files"}) {
      const auto list = get(source, key, array());
      Json paths = array();
      for (const auto &item : list.array)
        paths.array.push_back(str(resolvePath(item, base, "offline." + std::string(key), "file")));
      if (std::string(key) == "left_files" || geometry)
        params.object[std::string(key) == "left_files" ? "offline_files" : "offline_right_files"] =
            paths;
    }
    const auto &left = get(params, "offline_files").array;
    if (left.empty())
      throw std::invalid_argument("Offline left_files must be nonempty");
    if (geometry) {
      const auto &right = get(params, "offline_right_files").array;
      if (left.size() != right.size())
        throw std::invalid_argument("Offline pair lists have different lengths");
      for (std::size_t i = 0; i < left.size(); ++i)
        if (left[i].text == right[i].text)
          throw std::invalid_argument("A stereo pair cannot reuse one file for both eyes");
    }
    params.object["offline_format"] = get(source, "format", str("nv12"));
    params.object["offline_resize"] = str("stretch");
    params.object["offline_fps"] = get(source, "fps", num(2));
    params.object["offline_loop"] = get(source, "loop", boolean(false));
    params.object["offline_max_frames"] = get(source, "max_frames", integer(0));
    params.object["offline_wait_for_subscribers"] =
        get(source, "wait_for_subscribers", boolean(false));
  }
  const auto save_root = get(visual, "save_root", str("")).text;
  const int save_every =
      count(get(visual, "save_every_n", integer(0)), "visualization.save_every_n", 0);
  const auto actual_save =
      save_root.empty() ? "" : (std::filesystem::u8path(save_root) / run.filename()).u8string();
  params.object["save_root"] = str(actual_save);
  params.object["save_every_n"] =
      integer(graph.enabled["record"] && !actual_save.empty() ? save_every : 0);
  params.object["save_clouds_ply"] =
      boolean(graph.enabled["record"] && !actual_save.empty() && save_every > 0 &&
              flag(get(visual, "save_ply", boolean(false)), "visualization.save_ply"));
  params.object["publish_every_n"] = get(visual, "publish_every_n", integer(1));
  for (const auto &item : std::map<std::string, bool>{{"publish_clouds", true},
                                                      {"publish_disparity", true},
                                                      {"publish_maps", true},
                                                      {"publish_mask", true},
                                                      {"publish_input", false}})
    params.object[item.first] =
        boolean(graph.enabled["publish"] &&
                flag(get(visual, item.first, boolean(item.second)), "visualization." + item.first));
  params.object["robot_frame_id"] = get(visual, "robot_frame_id", str("base_link"));
  const auto &metrics = get(out.features, "metrics");
  params.object["metrics_enabled"] = get(metrics, "enabled", boolean(true));
  params.object["metrics_every_n"] = get(metrics, "every_n", integer(1));
  params.object["metrics_path"] =
      str(flag(get(metrics, "enabled", boolean(true)), "metrics.enabled") &&
                  flag(get(metrics, "write_jsonl", boolean(true)), "metrics.write_jsonl")
              ? (run / "metrics.jsonl").u8string()
              : "");
  // The ROS application owns per-frame saving/PLY; the SDK record node uses a
  // separate explicit directory, avoiding duplicate writes to frame folders.
  out.full.object["output"].object["every_n"] = integer(0);
  out.full.object["output"].object["directory"] = str((run / "sdk_record").u8string());
  out.full.object["output"].object["save_overlay"] = boolean(false);
  out.full.object["output"].object["save_inputs"] = boolean(false);
  out.parameters = params;
  const auto runtime = get(out.requested, "runtime", object());
  keys(runtime, {"executable", "compare_executable", "max_frames"}, "runtime");
  const bool motion_pipeline=c.visual_motion||c.motion_reference||c.slip_detection;
  out.slip_input=object();
  if(motion_pipeline) {
    const auto &reference=get(out.full,"motion_reference");
    out.slip_input.object["motion_reference_csv"]=get(reference,"input_csv",str(""));
    out.slip_input.object["clock_domain"]=get(reference,"clock_domain",str(""));
    out.slip_input.object["same_clock_confirmed"]=get(reference,"same_clock_confirmed",boolean(false));
    out.slip_input.object["max_frames"]=integer(count(get(runtime,"max_frames",integer(100)),"runtime.max_frames",0));
    if(c.mode=="recording") {
      if(!explicitValue(get(out.requested,"input"),"timeline_csv"))throw std::invalid_argument("Motion recording requires input.timeline_csv with original timestamps");
      out.slip_input.object["timeline_csv"]=str(resolvePath(get(get(out.requested,"input"),"timeline_csv"),base,"input.timeline_csv","file"));
    }else out.slip_input.object["timeline_csv"]=str("");
    params.object["motion_reference_csv"]=get(out.slip_input,"motion_reference_csv");
    params.object["slip_timeline_csv"]=get(out.slip_input,"timeline_csv");
    params.object["slip_max_frames"]=get(out.slip_input,"max_frames");
    params.object["wheel_odom_topic"]=c.motion_reference?get(reference,"odom_topic",str("")):str("");
    params.object["wheel_odom_wait_timeout_ms"]=get(reference,"wait_timeout_ms",integer(50));
    params.object["motion_clock_domain"]=get(out.slip_input,"clock_domain");
    params.object["motion_same_clock_confirmed"]=get(out.slip_input,"same_clock_confirmed");
    out.parameters=params;
  }
  out.environment["LD_LIBRARY_PATH"] = (std::filesystem::u8path(out.sdk_root) / "lib").u8string();
  if (c.mode == "camera")
    out.environment["LD_LIBRARY_PATH"] +=
#if defined(_WIN32)
        ";";
#else
        ":";
#endif
  if (c.mode == "camera")
    out.environment["LD_LIBRARY_PATH"] +=
        std::filesystem::u8path(get(camera, "sdk_library").text).parent_path().u8string();
  if (!env("LD_LIBRARY_PATH").empty())
    out.environment["LD_LIBRARY_PATH"] +=
#if defined(_WIN32)
        ";" + env("LD_LIBRARY_PATH");
#else
        ":" + env("LD_LIBRARY_PATH");
#endif
  if (c.mode == "recording") {
    out.recording = resolvedRecording(out.requested, c, source_files, calibration_evidence);
    const auto binary = resolvePath(
        (explicitValue(runtime, "compare_executable")
             ? get(runtime, "compare_executable")
             : str((applicationPrefix(c) / "lib/perception_ros_so_demo/r36_compare").u8string())),
        base, "runtime.compare_executable");
    out.argv = {binary,         "compare",
                "--capture",    get(out.recording, "directory").text,
                "--sdk-config", (run / "effective_full.json").u8string(),
                "--sdk-root",   out.sdk_root,
                "--output",     (run / "comparison").u8string(),
                "--method",     geometry && c.method == "raft_stereo" ? "stereo" : "original"};
    if(get(out.recording,"suite").text == "three_way") out.argv.push_back("--all");
    const auto &bpu = get(out.recording,"bpu");
    out.argv.insert(out.argv.end(),{"--bpu-enabled",get(bpu,"enabled").boolean?"1":"0",
        "--bpu-interval-ms",jsonText(get(bpu,"sample_interval_ms"),0)});
    for(const auto &p:get(bpu,"paths").array) out.argv.insert(out.argv.end(),{"--bpu-path",p.text});
    for (const auto &item : std::map<std::string, std::string>{{"limit", "limit"},
                                                               {"warmup_frames", "warmup"},
                                                               {"repeat", "repeat"},
                                                               {"save_every_n", "save-every"},
                                                               {"viewer_max_points", "max-points"},
                                                               {"order", "order"}}) {
      out.argv.push_back("--" + item.second);
      out.argv.push_back(get(out.recording, item.first).kind == Json::Kind::String
                             ? get(out.recording, item.first).text
                             : jsonText(get(out.recording, item.first), 0));
    }
    if (explicitValue(out.recording, "runner")) {
      out.argv.push_back("--runner");
      out.argv.push_back(get(out.recording, "runner").text);
    }
    if (!out.stereo.object.empty()) {
      out.argv.insert(out.argv.end(), {"--model", get(out.stereo, "model").text, "--input-mode",
                                       get(out.stereo, "input_mode").text, "--input-width",
                                       get(out.stereo, "input_width").text, "--input-height",
                                       get(out.stereo, "input_height").text});
    } else if (geometry) {
      // Original-only replay must keep the same comparison ROI without
      // resolving or loading a Stereo model it does not execute.
      const auto &shape = get(out.features, "stereo");
      out.argv.insert(out.argv.end(), {"--input-mode",
          get(shape, "image_transform", str("crop")).text, "--input-width",
          explicitValue(shape, "input_width") ? jsonText(get(shape, "input_width"), 0) : "0", "--input-height",
          explicitValue(shape, "input_height") ? jsonText(get(shape, "input_height"), 0) : "0"});
    }
    if (get(out.recording, "intrinsics").kind == Json::Kind::Object) {
      const auto &calibration = get(out.recording, "intrinsics");
      keys(calibration, {"width", "height", "fx", "fy", "cx", "cy", "bxf"}, "recording.intrinsics");
      for (auto key : {"width", "height", "fx", "fy", "cx", "cy", "bxf"}) {
        out.argv.push_back("--" + std::string(key));
        out.argv.push_back(jsonText(get(calibration, key), 0));
      }
    } else
      choice(get(out.recording, "intrinsics"), {"recording"}, "recording.intrinsics");
  } else {
    const auto binary = resolvePath(
        (explicitValue(runtime, "executable")
             ? get(runtime, "executable")
             : str((applicationPrefix(c) / "lib/perception_ros_so_demo/demo").u8string())),
        base, "runtime.executable");
    out.argv = {binary, "--ros-args", "--params-file", (run / "parameters.yaml").u8string()};
  }
  if(motion_pipeline && c.mode!="ros") {
    const auto binary=resolvePath(explicitValue(runtime,"executable")?get(runtime,"executable"):
      str((applicationPrefix(c)/"lib/perception_ros_so_demo/r36_slip_runner").u8string()),base,"runtime.executable");
    out.argv={binary,"--config",file.u8string(),"--output",(run/"slip").u8string()};
  }
  Json resolved = object();
  resolved.object["schema_version"] = integer(3);
  resolved.object["input_mode"] = str(c.mode);
  resolved.object["requested_disparity_source"] = str(c.method);
  resolved.object["effective_disparity_source"] = get(params, "disparity_source");
  resolved.object["parameters"] = params;
  resolved.object["graph"] = out.graph;
  resolved.object["sdk_root"] = str(out.sdk_root);
  resolved.object["run_directory"] = str(run.u8string());
  resolved.object["evidence"] = str("prepared_only");
  resolved.object["arguments"] = array();
  for (const auto &arg : out.argv)
    resolved.object["arguments"].array.push_back(str(arg));
  resolved.object["ld_library_path"] = str(out.environment.at("LD_LIBRARY_PATH"));
  resolved.object["recording"] = out.recording;
  resolved.object["slip_input"] = out.slip_input;
  resolved.object["calibration"] = calibration_evidence;
  Json feature_evidence = object();
  feature_evidence.object["settings"] = out.features;
  feature_evidence.object["source_files"] = source_files;
  feature_evidence.object["path_rule"] =
      str("All user paths are relative to the main pipeline.json; SDK paths are relative to their "
          "original full/profile file");
  feature_evidence.object["null_rule"] = str("null inherits installed values only for nullable "
                                             "fields; graph enabled=false disables functionality");
  feature_evidence.object["stereo_skipped"] = boolean(!geometry);
  generated[run / "requested_pipeline.json"] = jsonText(out.requested) + "\n";
  generated[run / "resolved_features.json"] = jsonText(feature_evidence) + "\n";
  generated[run / "effective_full.json"] = jsonText(out.full) + "\n";
  generated[run / "original_start.json"] = jsonText(out.original) + "\n";
  generated[run / "parameters.yaml"] = yamlParameters(params);
  generated[run / "resolved.json"] = jsonText(resolved) + "\n";
  if (create_files) {
    std::filesystem::create_directories(run.parent_path());
    if (!std::filesystem::create_directory(run))
      throw std::invalid_argument("Cannot create new run directory: " + run.u8string());
    for (const auto &item : generated) {
      std::ofstream stream(item.first, std::ios::binary);
      if (!stream)
        throw std::runtime_error("Cannot write generated file: " + item.first.u8string());
      stream << item.second;
      if (!stream)
        throw std::runtime_error("Cannot finish generated file: " + item.first.u8string());
    }
  }
  return out;
}
std::vector<std::string> cameraArguments(const Path &configuration, const std::string &mode) {
  if (mode != "infer" && mode != "map" && mode != "both" && mode != "recharge")
    throw std::invalid_argument("camera mode must be infer/map/both/recharge");
  const Path file = std::filesystem::absolute(configuration);
  const auto root = readJson(file);
  checkRoot(root);
  Context c{file.parent_path(), projectRoot(file), "camera",
            mode == "map" || mode == "both" ? "original_q8" : "raft_stereo",
            mode == "map" || mode == "both"};
  auto camera = cameraSettings(root, c, true, env("CAMERA_SDK"), env("CAMERA_CONFIG"));
  Json source_files;
  const auto f = loadFeatures(root, c, source_files);
  validateFeatureSettings(f, c);
  const auto &ros = get(f, "ros");
  Json params = object();
  params.object["mode"] = str(mode);
  params.object["camera_sdk"] = get(camera, "sdk_library");
  params.object["camera_config"] = get(camera, "vendor_config");
  params.object["camera_disparity_mode"] = get(camera, "disparity_mode");
  params.object["acquire_timeout_ms"] = get(camera, "acquire_timeout_ms");
  params.object["retry_interval_ms"] = get(camera, "retry_interval_ms");
  params.object["frame_id"] = get(camera, "frame_id");
  params.object["input_reliability"] = get(ros, "reliability", str("best_effort"));
  params.object["publish_camera_info"] = boolean(true);
  for (auto key : {"left_topic", "right_topic", "q8_topic", "left_camera_info_topic",
                   "right_camera_info_topic"})
    if (has(ros, key))
      params.object[key] = get(ros, key);
  std::vector<std::string> argv = {
      (applicationPrefix(c) / "lib/perception_ros_so_demo/raw_camera").u8string(), "--ros-args"};
  for (const auto &item : params.object)
    argv.insert(argv.end(), {"-p", item.first + ":=" + jsonText(item.second, 0)});
  return argv;
}
std::vector<std::string> captureArguments(const Path &configuration, const Path &output, int frames,
                                          const std::string &sdk, const std::string &config,
                                          int timeout) {
  const Path file = std::filesystem::absolute(configuration);
  const auto root = readJson(file);
  checkRoot(root);
  Context c{file.parent_path(), projectRoot(file), "capture", "original_q8", true};
  auto settings = cameraSettings(root, c, true, sdk.empty() ? env("CAMERA_SDK") : sdk,
                                 config.empty() ? env("CAMERA_CONFIG") : config, timeout);
  if (timeout)
    settings.object["acquire_timeout_ms"] =
        integer(count(integer(timeout), "capture.timeout_ms", 1, 60000));
  count(integer(frames), "capture.frames", 1);
  const std::string binary =
      env("CAPTURE_EXE").empty()
          ? (applicationPrefix(c) / "bin/r36_stereo_capture").u8string()
          : resolvePath(str(env("CAPTURE_EXE")), std::filesystem::current_path(), "CAPTURE_EXE",
                        "file");
  return {binary,
          "--sdk",
          get(settings, "sdk_library").text,
          "--camera-config",
          get(settings, "vendor_config").text,
          "--output",
          std::filesystem::absolute(output).u8string(),
          "--frames",
          std::to_string(frames),
          "--timeout-ms",
          get(settings, "acquire_timeout_ms").text};
}
std::map<std::string, std::string> cameraEnvironment(const std::vector<std::string> &arguments) {
  std::string library;
  for (std::size_t i = 0; i < arguments.size(); ++i) {
    if (arguments[i] == "--sdk" && i + 1 < arguments.size())
      library = arguments[i + 1];
    const std::string prefix = "camera_sdk:=";
    if (arguments[i].rfind(prefix, 0) == 0)
      library = detail::text(
          r33::json::JsonParser(arguments[i].substr(prefix.size()), "camera_sdk argument").parse(),
          "camera_sdk");
  }
  if (library.empty())
    throw std::invalid_argument("Camera executable arguments contain no SDK library");
  std::string search = std::filesystem::u8path(library).parent_path().u8string();
  if (!env("LD_LIBRARY_PATH").empty())
    search +=
#if defined(_WIN32)
        ";" + env("LD_LIBRARY_PATH");
#else
        ":" + env("LD_LIBRARY_PATH");
#endif
  return {{"LD_LIBRARY_PATH", search}};
}
} // namespace v3
