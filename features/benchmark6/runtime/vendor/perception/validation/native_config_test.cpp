#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace {
using namespace v3;
using namespace v3::detail;
void save(const Path &file, const Json &value) {
  std::ofstream out(file, std::ios::binary);
  out << jsonText(value) << '\n';
  if (!out)
    throw std::runtime_error("fixture write failed");
}
void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
bool samePath(const std::string &utf8, const Path &expected) {
  const auto actual=std::filesystem::u8path(utf8);
  if(!actual.is_absolute())return false;
  // Missing-model rejection also compares paths whose leaf no longer exists.
  // Normalize separators first, without requiring access to that missing leaf.
  const auto a=actual.lexically_normal();
  const auto b=std::filesystem::absolute(expected).lexically_normal();
  if(a==b)return true;
  // Windows short/long directory spellings can identify the same file. The
  // intended parent must also match, so an alternate model directory fails.
  std::error_code ec;
  if(!std::filesystem::equivalent(a,b,ec)||ec)return false;
  return std::filesystem::equivalent(a.parent_path(),b.parent_path());
}
Path checkedFixtureRoot(const Path& root,const Path& parent) {
  const std::string prefix="native-config-contract-",name=root.filename().u8string();
  require(root.is_absolute()&&name.size()>prefix.size()&&name.rfind(prefix,0)==0&&
      std::all_of(name.begin()+prefix.size(),name.end(),[](unsigned char c){return c>='0'&&c<='9';}),
      "unsafe temporary cleanup name");
  require(!std::filesystem::is_symlink(std::filesystem::symlink_status(root))&&std::filesystem::is_directory(root),
      "unsafe temporary cleanup root type");
  const auto resolved=std::filesystem::canonical(root),base=std::filesystem::canonical(parent);
  require(std::filesystem::equivalent(root.parent_path(),base)&&std::filesystem::equivalent(resolved.parent_path(),base)&&
      resolved.filename()==root.filename(),"unsafe temporary cleanup path");
  return resolved;
}
void environment(const std::string &key, const std::string &value) {
#ifdef _WIN32
  _putenv_s(key.c_str(), value.c_str());
#else
  setenv(key.c_str(), value.c_str(), 1);
#endif
}
class WorkingDirectory {
public:
  explicit WorkingDirectory(const Path &path) : previous_(std::filesystem::current_path()) {
    std::filesystem::current_path(path);
  }
  ~WorkingDirectory() {
    std::error_code ignored;
    std::filesystem::current_path(previous_, ignored);
  }

private:
  Path previous_;
};
template <class F> void rejected(F action, const std::string &expected) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(expected) == std::string::npos)
      throw std::runtime_error("Expected '" + expected + "', got: " + error.what());
    return;
  }
  throw std::runtime_error("Expected failure: " + expected);
}
template<class F>void rejectedPath(F action,const std::string& prefix,const Path& expected) {
  try{action();}catch(const std::exception& error) {
    const std::string message=error.what();const auto at=message.find(prefix);
    require(at!=std::string::npos,"Expected missing model error prefix: "+prefix+", got: "+message);
    require(samePath(message.substr(at+prefix.size()),expected),"Missing model error reported the wrong resolved path: "+message);
    return;
  }
  throw std::runtime_error("Expected failure: "+prefix+expected.u8string());
}
} // namespace
int main(int argc, char **argv) {
  Path temp;
  try {
    if (argc != 2)
      throw std::invalid_argument("native_config_test PROJECT_ROOT");
    const Path project = std::filesystem::absolute(argv[1]);
    const auto tick = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const auto temp_parent=std::filesystem::canonical(std::filesystem::temp_directory_path());
    temp = temp_parent / ("native-config-contract-" + std::to_string(tick));
    require(std::filesystem::create_directory(temp),"fixture directory must be new");
    temp=checkedFixtureRoot(temp,temp_parent);
    std::filesystem::create_directory(temp / "scheduling");
    std::filesystem::copy(project / "features", temp / "features",
                          std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(project / "camera.json", temp / "camera.json");
    std::filesystem::copy_file(project / "visualization.json", temp / "visualization.json");
    std::filesystem::create_directories(temp / "sdk/config");
    std::filesystem::copy(project / "scheduling/config/sdk_templates", temp / "sdk/config",
                          std::filesystem::copy_options::recursive);
    std::filesystem::copy(project / "scheduling/config/profiles", temp / "sdk/profiles",
                          std::filesystem::copy_options::recursive);
    environment("PERCEPTION_PROJECT_ROOT", temp.u8string());
    environment("SDK_ROOT", (temp / "sdk").u8string());
    const std::map<std::string, std::string> models{
        {"detection", "models/x5/mmyolo0828.bin"},
        {"segmentation", "models/x5/pidnet_0825_3classes.bin"},
        {"stereo", "models/x5/r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin"}};
    std::filesystem::create_directories(temp / "models/x5");
    for (const auto &item : models) {
      const Path model = temp / item.second;
      std::ofstream(model) << "configuration contract fixture only";
      require(get(readJson(temp / "features" / item.first / "config.json"), "model").text ==
                  item.second,
              item.first + " default model is not the managed relative path");
    }
    // Stale exports from previous versions cannot override explicit relative files.
    for (auto key : {"DET_MODEL", "SEG_MODEL", "STEREO_MODEL", "MODEL_PATH", "modelpath"})
      environment(key, (temp / "nonexistent-old-environment-model").u8string());
    auto v4_root = readJson(project / "pipeline.json");
    v4_root.object["input"].object["mode"] = str("ros");
    save(temp / "v4-pipeline.json", v4_root);
    const auto v4_prepared = v3::prepare(temp / "v4-pipeline.json", temp / "v4-default");
    std::filesystem::create_directories(temp / "config");
    std::filesystem::copy(project / "config/examples", temp / "config/examples",
                          std::filesystem::copy_options::recursive);
    for (const auto *example : {"det_only_ros", "stereo_cloud_ros", "full_ros"}) {
      const auto sample = v3::prepare(temp / "config/examples" / (std::string(example) + ".json"),
                                      temp / (std::string("example-") + example));
      require(sample.mode == "ros", "Runnable example selected wrong input mode");
      if (std::string(example) == "det_only_ros")
        require(!has(sample.parameters, "right_topic") &&
                    !get(sample.parameters, "mapping_enabled").boolean,
                "Detection example requires geometry input");
      if (std::string(example) == "stereo_cloud_ros")
        require(!get(get(sample.full, "inference"), "enabled").boolean &&
                    get(sample.parameters, "mapping_enabled").boolean,
                "Geometry-only example accidentally enables semantic models");
    }
    const auto v4_plan = r33::full::parseProductGraph(v4_prepared.graph);
    require(v4_plan.enabled("det") && v4_plan.enabled("seg") &&
                v4_plan.enabled("disparity") && !v4_plan.enabled("dnn") &&
                !v4_plan.enabled("rgb"), "v4 did not split inference/remove explicit RGB");
    require(get(get(v4_prepared.full, "input"), "disparity_source").text == "raft_stereo",
            "Resolved full SDK lost the selected disparity source");
    auto v4_original = v4_root;
    v4_original.object["input"].object["disparity_source"] = str("original_q8");
    save(temp / "v4-original.json", v4_original);
    const Path stereo_model_file = temp / models.at("stereo"),
               stereo_model_hidden = temp / "stereo-model.hidden";
    std::filesystem::rename(stereo_model_file, stereo_model_hidden);
    const auto original_q8 = v3::prepare(temp / "v4-original.json", temp / "v4-original");
    require(get(get(original_q8.full, "input"), "disparity_source").text == "original_q8" &&
                has(original_q8.parameters, "q8_topic") &&
                !has(original_q8.parameters, "right_topic") &&
                !has(original_q8.parameters, "stereo_model"),
            "Original Q8 selector loaded Stereo or requested a right image");
    std::filesystem::rename(stereo_model_hidden, stereo_model_file);
    const auto det_dag = readJson(temp / "v4-default/inference_detection_dag.json"),
               seg_dag = readJson(temp / "v4-default/inference_segmentation_dag.json");
    require(has(get(det_dag, "tasks"), "detection") &&
                !has(get(det_dag, "tasks"), "segmentation") &&
                has(get(seg_dag, "tasks"), "segmentation") &&
                !has(get(seg_dag, "tasks"), "detection"),
            "v4 lane instances do not have independent model pipelines");
    require(has(get(v4_prepared.full, "inference"), "detection_dag") &&
                has(get(v4_prepared.full, "inference"), "segmentation_dag"),
            "v4 runtime model pipeline paths missing");
    const auto v4_seg_settings = readJson(temp / "features/segmentation/config.json");
    auto v4_seg_off = v4_seg_settings;
    v4_seg_off.object["enabled"] = boolean(false);
    save(temp / "features/segmentation/config.json", v4_seg_off);
    const Path seg_model_file = temp / models.at("segmentation"),
               seg_model_hidden = temp / "segmentation-model.hidden";
    std::filesystem::rename(seg_model_file, seg_model_hidden);
    const auto without_seg = v3::prepare(temp / "v4-pipeline.json", temp / "v4-no-seg");
    const auto without_seg_plan = r33::full::parseProductGraph(without_seg.graph);
    require(without_seg_plan.enabled("det") && !without_seg_plan.enabled("seg") &&
                !has(get(without_seg.full, "inference"), "segmentation_model"),
            "Feature JSON switch did not disable v4 segmentation scheduling/model requirement");
    std::filesystem::rename(seg_model_hidden, seg_model_file);
    save(temp / "features/segmentation/config.json", v4_seg_settings);
    const auto v4_det_settings = readJson(temp / "features/detection/config.json");
    auto v4_det_off = v4_det_settings;
    v4_det_off.object["enabled"] = boolean(false);
    save(temp / "features/detection/config.json", v4_det_off);
    save(temp / "features/segmentation/config.json", v4_seg_off);
    auto geometric_only = v4_root;
    for (auto &node : geometric_only.object["graph"].object["nodes"].array)
      if (get(node, "feature").text == "labels") node.object["enabled"] = boolean(false);
    save(temp / "v4-geometric.json", geometric_only);
    const auto geometric = v3::prepare(temp / "v4-geometric.json", temp / "v4-both-lanes-off");
    const auto geometric_plan = r33::full::parseProductGraph(geometric.graph);
    require(!geometric_plan.enabled("det") && !geometric_plan.enabled("seg") &&
                geometric_plan.enabled("cloud") && geometric_plan.enabled("maps") &&
                !get(get(geometric.full, "inference"), "enabled").boolean,
            "Both split feature switches did not prune AI while retaining geometry-only work");
    rejected([&] { v3::prepare(temp / "v4-pipeline.json", temp / "v4-labels-without-ai"); },
             "labels requires an enabled det, seg or legacy inference producer");
    save(temp / "features/detection/config.json", v4_det_settings);
    save(temp / "features/segmentation/config.json", v4_seg_settings);
    auto invalid_binding = v4_root;
    invalid_binding.object["graph"].object["nodes"].array.front().object["inputs"].object["image"] =
        str("input.right");
    save(temp / "bad-v4-binding.json", invalid_binding);
    rejected([&] { v3::prepare(temp / "bad-v4-binding.json", temp / "bad-v4-output"); },
             "Unsupported inference input binding");
    // The existing broad configuration contracts continue to exercise schema 3
    // with a single legacy inference entry, alongside the v4 contracts above.
    auto root = v4_root;
    root.object["schema_version"] = integer(3);
    auto &legacy_nodes = root.object["graph"].object["nodes"].array;
    legacy_nodes.erase(std::remove_if(legacy_nodes.begin(), legacy_nodes.end(), [](const Json &n) {
      return has(n, "algorithm") && get(n, "algorithm").text == "seg";
    }), legacy_nodes.end());
    for (auto &node : legacy_nodes) {
      if (has(node, "algorithm")) {
        if (get(node, "algorithm").text == "stereo") node.object["feature"] = str("disparity");
        node.object.erase("algorithm"); node.object.erase("inputs");
      }
      auto &parents = node.object["depends_on"].array;
      for (auto &parent : parents) if (parent.text == "seg") parent = str("det");
      std::set<std::string> seen;
      parents.erase(std::remove_if(parents.begin(), parents.end(), [&](const Json &p) {
        return !seen.insert(p.text).second;
      }), parents.end());
    }
    root.object["input"].object["mode"] = str("ros");
    save(temp / "features/detection/config.json", v4_det_off);
    save(temp / "features/segmentation/config.json", v4_seg_off);
    save(temp / "legacy-no-lanes.json", root);
    rejected([&] { v3::prepare(temp / "legacy-no-lanes.json", temp / "legacy-no-lanes"); },
             "inference is enabled but both detection/segmentation lanes are disabled");
    save(temp / "features/detection/config.json", v4_det_settings);
    save(temp / "features/segmentation/config.json", v4_seg_settings);
    save(temp / "pipeline.json", root);
    const auto prepared = v3::prepare(temp / "pipeline.json", temp / "new");
    require(std::filesystem::is_regular_file(temp / "new/parameters.yaml"),
            "missing generated YAML");
    require(get(prepared.full, "mapping").object.at("enabled").boolean, "cloud enable lost");
    require(get(prepared.parameters, "right_topic").text == "/camera/rgbd_right_nv12",
            "right subscription missing");
    require(get(prepared.full, "output").object.at("every_n").text == "0",
            "SDK duplicate saving enabled");
    for (const auto &lane : {"detection", "segmentation"})
      require(samePath(get(get(prepared.full, "inference"), std::string(lane) + "_model").text,
                       temp / models.at(lane)),
              std::string(lane) + " model was not resolved from the main JSON");
    require(samePath(get(prepared.stereo, "model").text, temp / models.at("stereo")),
            "stereo model was not resolved from the main JSON");
    for (auto key : {"DET_MODEL", "SEG_MODEL", "STEREO_MODEL", "MODEL_PATH", "modelpath"})
      environment(key, "");
    const Path elsewhere = temp / "unrelated working directory";
    std::filesystem::create_directory(elsewhere);
    {
      WorkingDirectory cwd(elsewhere);
      const auto from_elsewhere = v3::prepare(temp / "pipeline.json", temp / "other-cwd");
      require(samePath(get(get(from_elsewhere.full, "inference"), "detection_model").text,
                       temp / models.at("detection")) &&
                  samePath(get(from_elsewhere.stereo, "model").text, temp / models.at("stereo")),
              "model paths depend on the process working directory");
    }
    // Copy a complete config/SDK/model tree to a path containing spaces. No model
    // environment export changes are needed after deployment to a new directory.
    const Path relocated = temp / "relocated project bundle";
    std::filesystem::create_directories(relocated / "scheduling");
    for (const auto &directory : {"features", "sdk", "models"})
      std::filesystem::copy(temp / directory, relocated / directory,
                            std::filesystem::copy_options::recursive);
    for (const auto &file : {"camera.json", "visualization.json"})
      std::filesystem::copy_file(temp / file, relocated / file);
    auto portable = root;
    portable.object["sdk"].object["root"] = str("sdk");
    save(relocated / "pipeline.json", portable);
    environment("PERCEPTION_PROJECT_ROOT", relocated.u8string());
    {
      WorkingDirectory cwd(elsewhere);
      const auto moved = v3::prepare(relocated / "pipeline.json", relocated / "run");
      for (const auto &lane : {"detection", "segmentation"})
        require(samePath(get(get(moved.full, "inference"), std::string(lane) + "_model").text,
                         relocated / models.at(lane)),
                std::string(lane) + " retained the pre-deployment model directory");
      require(samePath(get(moved.stereo, "model").text, relocated / models.at("stereo")),
              "stereo retained the pre-deployment model directory");
    }
    environment("PERCEPTION_PROJECT_ROOT", temp.u8string());
    for (const auto &item : models) {
      const Path model = temp / item.second;
      Path hidden=model;hidden += ".hidden";
      std::filesystem::rename(model, hidden);
      const Path output = temp / ("missing-" + item.first);
      rejectedPath([&] { v3::prepare(temp / "pipeline.json", output); },
                   item.first + ".model file does not exist: ",model);
      require(!std::filesystem::exists(output), "missing model configuration created output");
      std::filesystem::rename(hidden, model);
    }
    // CPU models use the same project-relative rule, even when their SDK profile
    // lives in a different directory. Prepared profiles contain resolved paths.
    const auto original_inference = readJson(temp / "features/inference/config.json");
    const auto original_detection = readJson(temp / "features/detection/config.json");
    const auto original_segmentation = readJson(temp / "features/segmentation/config.json");
    const std::map<std::string, std::string> cpu_models{
        {"detection", "models/cpu/case2_20260821_V2.1_epoch_160_dy.onnx"},
        {"segmentation", "models/cpu/train_20260924_3class_aug_flagstone.onnx"}};
    std::filesystem::create_directories(temp / "models/cpu");
    std::filesystem::create_directories(temp / "profiles");
    auto cpu_profile = object();
    for (const auto &item : cpu_models) {
      std::ofstream(temp / item.second) << "configuration contract fixture only";
      auto feature = readJson(temp / "features" / item.first / "config.json");
      feature.object["model"] = str(item.second);
      save(temp / "features" / item.first / "config.json", feature);
      cpu_profile.object[item.first] = object();
      cpu_profile.object[item.first].object["model"] = str("unused-sdk-profile-default.onnx");
    }
    save(temp / "profiles/cpu.json", cpu_profile);
    auto cpu_inference = original_inference;
    cpu_inference.object["backend"] = str("cpu");
    cpu_inference.object["cpu_profile"] = str("profiles/cpu.json");
    save(temp / "features/inference/config.json", cpu_inference);
    v3::prepare(temp / "pipeline.json", temp / "cpu-models");
    const auto prepared_cpu = readJson(temp / "cpu-models/inference_cpu.json");
    for (const auto &item : cpu_models)
      require(samePath(get(get(prepared_cpu, item.first), "model").text, temp / item.second),
              "CPU " + item.first + " model used the profile directory as its base");
    save(temp / "features/inference/config.json", original_inference);
    save(temp / "features/detection/config.json", original_detection);
    save(temp / "features/segmentation/config.json", original_segmentation);
    const auto dag = readJson(temp / "new/inference_dag.json");
    require(!has(get(dag, "tasks"), "disparity"), "stereo scheduled twice");
    rejected([&] { v3::prepare(temp / "pipeline.json", temp / "new"); }, "already exists");
    auto detection = readJson(temp / "features/detection/config.json");
    detection.object["score_threshold"] = num(.42);
    save(temp / "features/detection/config.json", detection);
    auto fusion = readJson(temp / "features/fusion/config.json");
    fusion.object["map"].object["resolution_m"] = num(.03);
    fusion.object["labels"].object["det_score_min"] = num(.51);
    save(temp / "features/fusion/config.json", fusion);
    const auto tuned = v3::prepare(temp / "pipeline.json", temp / "tuned");
    require(r33::json::number(get(get(tuned.original, "map"), "MapResolution")) == .03,
            "grid parameter lost");
    require(r33::json::number(get(get(tuned.original, "label_fusion"), "det_score_min")) == .51,
            "fusion parameter lost");
    std::ifstream cfg(temp / "tuned/inference_original.cfg");
    std::string cfgtext((std::istreambuf_iterator<char>(cfg)), {});
    require(cfgtext.find("0.419999") != std::string::npos ||
                cfgtext.find("0.42") != std::string::npos,
            "detection threshold did not reach profile");
    // A disabled lane and unused stereo require no model file. Inference-only
    // subscribes to left images only.
    auto infer = root;
    for (auto &node : infer.object["graph"].object["nodes"].array) {
      auto feature = get(node, "feature").text;
      node.object["enabled"] = boolean(feature == "inference" || feature == "publish");
    }
    auto seg = readJson(temp / "features/segmentation/config.json");
    seg.object["enabled"] = boolean(false);
    save(temp / "features/segmentation/config.json", seg);
    std::filesystem::remove(temp / models.at("segmentation"));
    std::filesystem::remove(temp / models.at("stereo"));
    save(temp / "pipeline.json", infer);
    auto only = v3::prepare(temp / "pipeline.json", temp / "infer");
    require(get(only.parameters, "mode").text == "infer", "wrong inference mode");
    require(!has(only.parameters, "right_topic") && !has(only.parameters, "q8_topic"),
            "unused subscriptions leaked");
    require(!get(only.parameters, "use_camera_info").boolean, "unused camera info dependency");
    require(!has(get(only.full, "inference"), "segmentation_model"),
            "disabled model override retained");
    auto inferdag = readJson(temp / "infer/inference_dag.json");
    require(has(get(inferdag, "tasks"), "detection") &&
                !has(get(inferdag, "tasks"), "segmentation"),
            "disabled lane still scheduled");
    auto unknown = infer;
    unknown.object["typo"] = integer(1);
    save(temp / "bad.json", unknown);
    rejected([&] { v3::prepare(temp / "bad.json", temp / "not_created"); }, "unknown field");
    require(!std::filesystem::exists(temp / "not_created"), "invalid configuration created output");
    auto invalid = infer;
    invalid.object["graph"].object["nodes"].array.front().object["depends_on"] = array();
    invalid.object["graph"].object["nodes"].array.front().object["depends_on"].array.push_back(
        str("record"));
    save(temp / "bad.json", invalid);
    rejected([&] { v3::prepare(temp / "bad.json", temp / "cycle"); }, "Cycle");
    invalid = infer;
    invalid.object["graph"].object["nodes"].array.front().object["enabled"] = str("false");
    save(temp / "bad.json", invalid);
    rejected([&] { v3::prepare(temp / "bad.json", temp / "bool"); }, "boolean");
    invalid = infer;
    invalid.object["sdk"].object["root"] = str("${MISSING_NATIVE_CONFIG_TEST_VARIABLE}");
    save(temp / "bad.json", invalid);
    rejected([&] { v3::prepare(temp / "bad.json", temp / "env"); }, "Missing environment variable");
    // Input file paths stay relative to the main JSON, not the module JSON.
    auto offline = readJson(temp / "features/offline/config.json");
    offline.object["left_files"] = array();
    offline.object["left_files"].array.push_back(str("left.png"));
    std::ofstream(temp / "left.png") << "not decoded in prepare-only";
    offline.object["right_files"] = array();
    offline.object["fps"] = integer(2);
    save(temp / "features/offline/config.json", offline);
    infer.object["input"].object["mode"] = str("offline");
    save(temp / "pipeline.json", infer);
    auto still = v3::prepare(temp / "pipeline.json", temp / "offline");
    require(std::filesystem::equivalent(
                std::filesystem::u8path(get(still.parameters, "offline_files").array.front().text),
                temp / "left.png"),
            "relative path base changed");
    require(!has(still.parameters, "offline_right_files"), "inference-only requires a right image");
    offline.object["intrinsics_file"] = str("calibration/nonexistent-unused.json");
    save(temp / "features/offline/config.json", offline);
    const auto unused_calibration =
        v3::prepare(temp / "pipeline.json", temp / "offline-unused-calibration");
    require(!has(unused_calibration.parameters, "intrinsics_file") &&
                !has(unused_calibration.parameters, "fx"),
            "offline inference-only loaded unused intrinsics");
    {
      std::ifstream yaml(temp / "offline/parameters.yaml", std::ios::binary);
      const std::string text((std::istreambuf_iterator<char>(yaml)), {});
      require(text.find("    offline_fps: 2.0\n") != std::string::npos,
              "integer-valued FPS lost its declared ROS double type");
      require(text.find("    offline_max_frames: 0\n") != std::string::npos,
              "integer frame limit was incorrectly converted to double");
    }
    // Offline geometry has no default calibration. Its explicit file is rooted
    // at pipeline.json, shared with the runtime reader, and copied to provenance.
    auto offline_cloud = root;
    offline_cloud.object["input"].object["mode"] = str("offline");
    save(temp / "offline-cloud.json", offline_cloud);
    std::ofstream(temp / models.at("stereo")) << "configuration contract fixture only";
    std::ofstream(temp / "right.png") << "not decoded in prepare-only";
    offline.object["right_files"].array.push_back(str("right.png"));
    offline.object["rectified"] = boolean(true);
    offline.object["calibration_confirmed"] = boolean(true);
    offline.object["intrinsics_file"] = Json{};
    save(temp / "features/offline/config.json", offline);
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-missing-file"); },
             "Offline geometry requires offline.intrinsics_file");
    offline.object["intrinsics_file"] = str("calibration/intrinsics.json");
    save(temp / "features/offline/config.json", offline);
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-absent-file"); },
             "offline.intrinsics_file file does not exist");
    Json file_intrinsics = object();
    for (const auto &item : std::map<std::string, int>{{"width", 640},
                                                       {"height", 544},
                                                       {"fx", 401},
                                                       {"fy", 402},
                                                       {"cx", 319},
                                                       {"cy", 271},
                                                       {"bxf", 20050}})
      file_intrinsics.object[item.first] = integer(item.second);
    file_intrinsics.object["_help"] = str("configuration fixture, not real camera calibration");
    std::filesystem::create_directory(temp / "calibration");
    save(temp / "calibration/intrinsics.json", file_intrinsics);
    {
      WorkingDirectory cwd(elsewhere);
      const auto calibrated_offline =
          v3::prepare(temp / "offline-cloud.json", temp / "offline-calibrated");
      require(samePath(get(calibrated_offline.parameters, "intrinsics_file").text,
                       temp / "calibration/intrinsics.json"),
              "intrinsics file is not relative to the main JSON");
      for (auto key : {"fx", "fy", "cx", "cy", "bxf"})
        require(get(calibrated_offline.parameters, key).text == get(file_intrinsics, key).text,
                std::string("offline calibration lost ") + key);
      const auto evidence = readJson(temp / "offline-calibrated/resolved.json");
      require(get(get(evidence, "calibration"), "source").text == "intrinsics_file",
              "offline calibration provenance lost");
      const auto feature_evidence = readJson(temp / "offline-calibrated/resolved_features.json");
      require(samePath(get(get(feature_evidence, "source_files"), "intrinsics").text,
                       temp / "calibration/intrinsics.json"),
              "intrinsics source file was not recorded");
    }
    auto invalid_intrinsics = file_intrinsics;
    invalid_intrinsics.object["height"] = integer(352);
    save(temp / "calibration/intrinsics.json", invalid_intrinsics);
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-wrong-size"); },
             "must describe 640x544 source images");
    for (const auto &item :
         std::map<std::string, int>{{"fx", 0}, {"fy", -1}, {"bxf", 0}, {"cx", 640}, {"cy", 544}}) {
      invalid_intrinsics = file_intrinsics;
      invalid_intrinsics.object[item.first] = integer(item.second);
      save(temp / "calibration/intrinsics.json", invalid_intrinsics);
      rejected(
          [&] { v3::prepare(temp / "offline-cloud.json", temp / ("offline-bad-" + item.first)); },
          item.first == "cx" || item.first == "cy" ? "principal point" : "must be positive");
    }
    std::ofstream(temp / "calibration/intrinsics.json", std::ios::binary)
        << "{\"width\":640,\"height\":544,\"fx\":1e309,\"fy\":402,"
           "\"cx\":319,\"cy\":271,\"bxf\":20050}\n";
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-nonfinite"); },
             "Invalid intrinsics file");
    invalid_intrinsics = file_intrinsics;
    invalid_intrinsics.object.erase("bxf");
    save(temp / "calibration/intrinsics.json", invalid_intrinsics);
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-incomplete"); },
             "Missing JSON field: bxf");
    auto metadata = object();
    metadata.object["intrinsics"] = file_intrinsics;
    metadata.object["capture_id"] = str("camera-sdk-recording-fixture");
    save(temp / "calibration/intrinsics.json", metadata);
    const auto from_metadata = v3::prepare(temp / "offline-cloud.json", temp / "offline-metadata");
    require(get(from_metadata.parameters, "fx").text == "401", "metadata intrinsics not loaded");
    offline.object["calibration_confirmed"] = boolean(false);
    save(temp / "features/offline/config.json", offline);
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-unconfirmed"); },
             "confirmed rectified input");
    offline.object["calibration_confirmed"] = boolean(true);
    offline.object["intrinsics"] = file_intrinsics;
    save(temp / "features/offline/config.json", offline);
    rejected([&] { v3::prepare(temp / "offline-cloud.json", temp / "offline-inline"); },
             "offline.intrinsics is no longer supported");
    offline.object.erase("intrinsics");
    offline.object["intrinsics_file"] = str("${UNUSED_INTRINSICS_FILE_ENVIRONMENT}");
    save(temp / "features/offline/config.json", offline);
    // Whole-valued calibration parameters also have declared ROS double types.
    auto ros = readJson(temp / "features/ros/config.json");
    ros.object["intrinsics"] = object();
    for (const auto &item : std::map<std::string, int>{{"width", 640},
                                                       {"height", 544},
                                                       {"fx", 300},
                                                       {"fy", 300},
                                                       {"cx", 320},
                                                       {"cy", 272},
                                                       {"bxf", 18000}})
      ros.object["intrinsics"].object[item.first] = integer(item.second);
    ros.object["rectified"] = boolean(true);
    ros.object["calibration_confirmed"] = boolean(true);
    save(temp / "features/ros/config.json", ros);
    auto calibrated = root;
    calibrated.object["input"].object["disparity_source"] = str("original_q8");
    save(temp / "calibrated.json", calibrated);
    v3::prepare(temp / "calibrated.json", temp / "typed_calibration");
    {
      std::ifstream yaml(temp / "typed_calibration/parameters.yaml", std::ios::binary);
      const std::string text((std::istreambuf_iterator<char>(yaml)), {});
      for (const auto &item : std::map<std::string, int>{
               {"fx", 300}, {"fy", 300}, {"cx", 320}, {"cy", 272}, {"bxf", 18000}})
        require(text.find("    " + item.first + ": " + std::to_string(item.second) + ".0\n") !=
                    std::string::npos,
                "whole-valued " + item.first + " lost its declared ROS double type");
    }
    // Recharge uses the delivered frame camera matrix only when its own host
    // configuration requests it. The retained dataset path has no new ROS
    // CameraInfo dependency, while sdk_scaled must use genuine calibration.
    const auto original_ros = ros;
    const auto original_recharge = readJson(temp / "sdk/config/recharge_mark.json");
    auto recharge = original_recharge;
    auto recharge_only = root;
    for (auto &node : recharge_only.object["graph"].object["nodes"].array) {
      const auto feature = get(node, "feature").text;
      node.object["enabled"] = boolean(feature == "recharge" || feature == "publish");
    }
    save(temp / "recharge-only.json", recharge_only);
    ros.object["intrinsics"] = object();
    ros.object["rectified"] = boolean(false);
    ros.object["calibration_confirmed"] = boolean(false);
    save(temp / "features/ros/config.json", ros);
    const auto dataset_recharge =
        v3::prepare(temp / "recharge-only.json", temp / "recharge-dataset");
    require(!get(dataset_recharge.parameters, "use_camera_info").boolean &&
                !has(dataset_recharge.parameters, "fx"),
            "dataset recharge acquired an unused calibration dependency");
    recharge.object["intrinsics_source"] = str("sdk_scaled");
    save(temp / "sdk/config/recharge_mark.json", recharge);
    rejected(
        [&] { v3::prepare(temp / "recharge-only.json", temp / "recharge-missing-intrinsics"); },
        "Explicit intrinsics require rectified/calibration_confirmed=true");
    ros.object["intrinsics"] = str("camera_info");
    save(temp / "features/ros/config.json", ros);
    const auto frame_recharge =
        v3::prepare(temp / "recharge-only.json", temp / "recharge-camera-info");
    require(get(frame_recharge.parameters, "use_camera_info").boolean &&
                has(frame_recharge.parameters, "left_camera_info_topic") &&
                has(frame_recharge.parameters, "right_camera_info_topic") &&
                !has(frame_recharge.parameters, "stereo_model"),
            "sdk_scaled recharge did not configure genuine CameraInfo without Stereo");
    save(temp / "features/ros/config.json", original_ros);
    const auto explicit_recharge =
        v3::prepare(temp / "recharge-only.json", temp / "recharge-explicit-intrinsics");
    require(!get(explicit_recharge.parameters, "use_camera_info").boolean &&
                get(explicit_recharge.parameters, "fx").text == "300",
            "sdk_scaled recharge explicit camera matrix lost");
    // The recharge backend also accepts a start-config pointer to a separate
    // wrapped mark configuration; inspect the same target as the SDK.
    auto wrapped_recharge = object();
    wrapped_recharge.object["recharge_mark"] = recharge;
    save(temp / "sdk/config/recharge_actual.json", wrapped_recharge);
    auto recharge_pointer = object();
    recharge_pointer.object["recharge_mark"] = object();
    recharge_pointer.object["recharge_mark"].object["config_file"] = str("recharge_actual.json");
    save(temp / "sdk/config/recharge_mark.json", recharge_pointer);
    const auto pointed_recharge =
        v3::prepare(temp / "recharge-only.json", temp / "recharge-config-pointer");
    require(get(pointed_recharge.parameters, "fx").text == "300",
            "recharge start-config pointer lost its intrinsics source");
    std::filesystem::rename(temp / "sdk/config/recharge_mark.json",
                            temp / "sdk/config/recharge_mark.hidden");
    auto without_recharge = infer;
    without_recharge.object["input"].object["mode"] = str("ros");
    save(temp / "without-recharge.json", without_recharge);
    const auto inactive_recharge =
        v3::prepare(temp / "without-recharge.json", temp / "inactive-recharge");
    require(!get(inactive_recharge.parameters, "use_camera_info").boolean,
            "inactive recharge introduced CameraInfo");
    std::filesystem::rename(temp / "sdk/config/recharge_mark.hidden",
                            temp / "sdk/config/recharge_mark.json");
    save(temp / "sdk/config/recharge_mark.json", original_recharge);
    // Independent disparity is real geometry work even with cloud and AI off.
    auto disparity_only = calibrated;
    for (auto &node : disparity_only.object["graph"].object["nodes"].array) {
      const auto feature = get(node, "feature").text;
      node.object["enabled"] = boolean(feature == "disparity" || feature == "publish");
    }
    save(temp / "disparity-only.json", disparity_only);
    const auto disparity = v3::prepare(temp / "disparity-only.json", temp / "disparity-only");
    require(!get(disparity.full, "mapping").object.at("enabled").boolean &&
                !get(disparity.full, "inference").object.at("enabled").boolean &&
                get(disparity.parameters, "disparity_enabled").boolean &&
                !get(disparity.parameters, "mapping_enabled").boolean &&
                has(disparity.parameters, "q8_topic") &&
                get(disparity.parameters, "publish_disparity").boolean,
            "independent Q8 disparity was skipped or forced cloud/AI");
    disparity_only.object["input"].object["disparity_source"] = str("raft_stereo");
    save(temp / "disparity-stereo.json", disparity_only);
    const auto standalone_stereo =
        v3::prepare(temp / "disparity-stereo.json", temp / "disparity-stereo");
    require(has(standalone_stereo.parameters, "right_topic") &&
                has(standalone_stereo.parameters, "stereo_model") &&
                !has(standalone_stereo.parameters, "q8_topic"),
            "independent Stereo did not configure left/right geometry input");
    auto temporal = readJson(temp / "features/temporal/config.json");
    const auto default_temporal = temporal;
    temporal.object["capacity"] = integer(3);
    temporal.object["frame_interval"] = integer(2);
    temporal.object["max_memory_mb"] = integer(12);
    temporal.object["difference_threshold"] = integer(17);
    temporal.object["match_iou_threshold"] = num(.7);
    temporal.object["match_max_detections"] = integer(40);
    save(temp / "features/temporal/config.json", temporal);
    auto temporal_infer = infer;
    temporal_infer.object["input"].object["mode"] = str("ros");
    for (auto &node : temporal_infer.object["graph"].object["nodes"].array) {
      const auto feature = get(node, "feature").text;
      node.object["enabled"] = boolean(feature == "inference" || feature == "publish" ||
                                        feature == "history" || feature == "temporal_difference" ||
                                        feature == "temporal_match");
    }
    save(temp / "temporal.json", temporal_infer);
    const auto timed = v3::prepare(temp / "temporal.json", temp / "temporal-custom");
    const auto &effective_history = get(timed.full, "history");
    for (auto key : {"capacity", "frame_interval", "max_memory_mb", "difference_threshold",
                     "match_iou_threshold", "match_max_detections"})
      require(get(effective_history, key).text == get(temporal, key).text,
              std::string("temporal setting did not reach SDK: ") + key);
    require(get(effective_history, "difference").boolean && get(effective_history, "match").boolean,
            "temporal feature selection lost");
    temporal.object["capacity"] = integer(1);
    save(temp / "features/temporal/config.json", temporal);
    rejected([&] { v3::prepare(temp / "temporal.json", temp / "temporal-no-pair"); },
             "temporal.capacity >= 2");
    save(temp / "features/temporal/config.json", default_temporal);
    auto inference_recharge = recharge_only;
    for (auto &node : inference_recharge.object["graph"].object["nodes"].array)
      if (get(node, "feature").text == "inference")
        node.object["enabled"] = boolean(true);
    ros = original_ros;
    ros.object["gdc_topic"] = str("/custom/rectified_hires_left");
    save(temp / "features/ros/config.json", ros);
    save(temp / "inference-recharge.json", inference_recharge);
    const auto combined =
        v3::prepare(temp / "inference-recharge.json", temp / "inference-recharge");
    require(get(combined.parameters, "mode").text == "both" &&
                !get(combined.parameters, "mapping_enabled").boolean &&
                !has(combined.parameters, "q8_topic") &&
                get(combined.parameters, "gdc_topic").text == "/custom/rectified_hires_left",
            "AI/recharge without mapping acquired Q8 or lost GDC routing");
    inference_recharge.object["input"].object["mode"] = str("recording");
    save(temp / "recording-recharge.json", inference_recharge);
    rejected([&] { v3::prepare(temp / "recording-recharge.json", temp / "recording-recharge"); },
             "recording recharge");
    save(temp / "features/ros/config.json", original_ros);
    const Path prefix = temp / "custom-install";
    auto inactive_camera = readJson(temp / "camera.json");
    inactive_camera.object["disparity_mode"] = integer(1);
    save(temp / "camera.json", inactive_camera);
    // ROS receives its Q8 externally; an unused camera mode must not veto it.
    v3::prepare(temp / "calibrated.json", temp / "ros-unused-camera-mode");
    inactive_camera.object["disparity_mode"] = str("auto");
    save(temp / "camera.json", inactive_camera);
    environment("INSTALL_ROOT", prefix.u8string());
    auto custom = v3::prepare(temp / "pipeline.json", temp / "custom");
    require(samePath(custom.argv.front(), prefix / "lib/perception_ros_so_demo/demo"),
            "custom INSTALL_ROOT did not select application");
    auto camera = readJson(temp / "camera.json");
    const Path vendor = temp / "vendor with spaces";
    std::filesystem::create_directory(vendor);
    std::ofstream(vendor / "camera.so") << "not a real library; config only";
    std::ofstream(vendor / "camera.cfg") << "fixture";
    camera.object["sdk_library"] = str((vendor / "camera.so").u8string());
    camera.object["vendor_config"] = str((vendor / "camera.cfg").u8string());
    camera.object["disparity_mode"] = str("auto");
    save(temp / "camera.json", camera);
    auto camera_cloud = calibrated;
    camera_cloud.object["input"].object["mode"] = str("camera");
    save(temp / "camera-cloud.json", camera_cloud);
    const auto automatic_camera =
        v3::prepare(temp / "camera-cloud.json", temp / "camera-automatic-calibration");
    for (auto key : {"fx", "fy", "cx", "cy", "bxf", "intrinsics_file"})
      require(!has(automatic_camera.parameters, key),
              std::string("camera SDK calibration overridden by ") + key);
    const auto camera_evidence = readJson(temp / "camera-automatic-calibration/resolved.json");
    require(get(get(camera_evidence, "calibration"), "source").text == "camera_sdk",
            "camera SDK calibration provenance lost");
    auto camera_inference = infer;
    camera_inference.object["input"].object["mode"] = str("camera");
    save(temp / "camera-inference.json", camera_inference);
    v3::prepare(temp / "camera-inference.json", temp / "camera-inference");
    const auto camera_inference_evidence = readJson(temp / "camera-inference/resolved.json");
    require(get(get(camera_inference_evidence, "calibration"), "source").text == "camera_sdk",
            "inference-only camera lost its actual SDK calibration provenance");
    const auto publisher = v3::cameraArguments(temp / "pipeline.json", "infer");
    require(samePath(publisher.front(), prefix / "lib/perception_ros_so_demo/raw_camera"),
            "custom INSTALL_ROOT did not select camera publisher");
    const auto camera_search=v3::cameraEnvironment(publisher).at("LD_LIBRARY_PATH");
#ifdef _WIN32
    const auto first_search_end=camera_search.find(';');
#else
    const auto first_search_end=camera_search.find(':');
#endif
    require(samePath(camera_search.substr(0,first_search_end),vendor),
            "vendor shared-library directory missing from camera environment");
    const auto capture =
        v3::captureArguments(temp / "pipeline.json", temp / "capture", 3, {}, {}, 1);
    require(samePath(capture.front(), prefix / "bin/r36_stereo_capture"),
            "custom INSTALL_ROOT did not select recorder");
    require(capture.back() == "1", "capture explicit timeout was ignored");
    auto metrics = readJson(temp / "features/metrics/config.json");
    metrics.object["enabled"] = boolean(false);
    save(temp / "features/metrics/config.json", metrics);
    infer.object["input"].object["mode"] = str("recording");
    infer.object["recording"] = object();
    std::filesystem::create_directory(temp / "capture");
    infer.object["recording"].object["directory"] = str("capture");
    infer.object["recording"].object["intrinsics_file"] =
        str("${UNUSED_INTRINSICS_FILE_ENVIRONMENT}");
    save(temp / "pipeline.json", infer);
    auto replay = v3::prepare(temp / "pipeline.json", temp / "replay");
    require(samePath(replay.argv.front(), prefix / "lib/perception_ros_so_demo/r36_compare"),
            "custom INSTALL_ROOT did not select comparison controller");
    const auto method = std::find(replay.argv.begin(), replay.argv.end(), "--method");
    require(method != replay.argv.end() && *(method + 1) == "original",
            "inference-only replay unexpectedly requested a stereo model");
    require(!get(replay.parameters, "metrics_enabled").boolean &&
                get(replay.parameters, "metrics_path").text.empty(),
            "metrics.enabled=false did not disable metrics parameters");
    require(get(replay.recording, "intrinsics").text == "recording",
            "inference-only recording opened unused intrinsics file");
    auto recording_cloud = calibrated;
    recording_cloud.object["input"].object["mode"] = str("recording");
    recording_cloud.object["recording"] = object();
    recording_cloud.object["recording"].object["directory"] = str("capture");
    save(temp / "recording-cloud.json", recording_cloud);
    rejected([&] { v3::prepare(temp / "recording-cloud.json", temp / "recording-missing-file"); },
             "Recording geometry requires recording.intrinsics_file");
    recording_cloud.object["recording"].object["intrinsics_file"] =
        str("calibration/custom-recording.json");
    auto recording_intrinsics = file_intrinsics;
    recording_intrinsics.object["width"] = integer(800);
    recording_intrinsics.object["height"] = integer(600);
    recording_intrinsics.object["bxf"] = integer(25000);
    save(temp / "calibration/custom-recording.json", recording_intrinsics);
    save(temp / "recording-cloud.json", recording_cloud);
    const auto explicit_recording =
        v3::prepare(temp / "recording-cloud.json", temp / "recording-custom-file");
    for (const auto &item : std::map<std::string, std::string>{
             {"width", "800"}, {"height", "600"}, {"fx", "401"}, {"bxf", "25000"}}) {
      const auto argument = std::find(explicit_recording.argv.begin(),
                                      explicit_recording.argv.end(), "--" + item.first);
      require(argument != explicit_recording.argv.end() &&
                  argument + 1 != explicit_recording.argv.end() && *(argument + 1) == item.second,
              "recording intrinsics file did not reach CLI argument " + item.first);
    }
    require(samePath(get(explicit_recording.recording, "intrinsics_file").text,
                     temp / "calibration/custom-recording.json"),
            "recording intrinsics path is not relative to main JSON");
    save(temp / "capture/metadata.json", metadata);
    recording_cloud.object["recording"].object["intrinsics_file"] = str("capture/metadata.json");
    save(temp / "recording-cloud.json", recording_cloud);
    const auto recording_metadata =
        v3::prepare(temp / "recording-cloud.json", temp / "recording-metadata-file");
    require(get(get(recording_metadata.recording, "intrinsics"), "bxf").text == "20050",
            "recording metadata calibration lost");
    auto three_way=v4_root;
    three_way.object["input"].object["mode"]=str("recording");
    three_way.object["recording"]=object();
    three_way.object["recording"].object["directory"]=str("capture");
    three_way.object["recording"].object["intrinsics_file"]=str("capture/metadata.json");
    three_way.object["recording"].object["suite"]=str("three_way");
    three_way.object["recording"].object["bpu"]=object();
    three_way.object["recording"].object["bpu"].object["enabled"]=boolean(false);
    three_way.object["recording"].object["bpu"].object["sample_interval_ms"]=integer(50);
    save(temp/"three-way.json",three_way);
    const auto three=v3::prepare(temp/"three-way.json",temp/"three-way");
    require(std::find(three.argv.begin(),three.argv.end(),"--all")!=three.argv.end(),"three_way did not select all groups");
    const auto crop_arg=std::find(three.argv.begin(),three.argv.end(),"--input-mode");
    require(crop_arg!=three.argv.end()&&*(crop_arg+1)=="crop","default bottom crop did not reach comparison controller");
    auto stereo_settings=readJson(temp/"features/stereo/config.json");
    stereo_settings.object["image_transform"]=str("unsupported_crop");
    save(temp/"features/stereo/config.json",stereo_settings);
    rejected([&]{v3::prepare(temp/"three-way.json",temp/"unsupported-crop");},"stereo.image_transform");
    stereo_settings.object["image_transform"]=str("crop");
    save(temp/"features/stereo/config.json",stereo_settings);
    auto original_replay=three_way;
    original_replay.object["input"].object["disparity_source"]=str("original_q8");
    original_replay.object["recording"].object["suite"]=str("selected");
    save(temp/"original-crop.json",original_replay);
    std::filesystem::rename(stereo_model_file,stereo_model_hidden);
    const auto original_crop=v3::prepare(temp/"original-crop.json",temp/"original-crop");
    std::filesystem::rename(stereo_model_hidden,stereo_model_file);
    const auto original_mode=std::find(original_crop.argv.begin(),original_crop.argv.end(),"--input-mode");
    require(original_mode!=original_crop.argv.end()&&*(original_mode+1)=="crop",
            "original-only JSON replay lost comparison crop policy");
    require(std::find(original_crop.argv.begin(),original_crop.argv.end(),"--model")==original_crop.argv.end(),
            "original-only crop resolved or forwarded unused Stereo model");
    const auto bpu_arg=std::find(three.argv.begin(),three.argv.end(),"--bpu-enabled");
    require(bpu_arg!=three.argv.end()&&*(bpu_arg+1)=="0","BPU disable did not reach runner");
    three_way.object["recording"].object["order"]=str("original,stereo");
    save(temp/"three-bad-order.json",three_way);
    rejected([&]{v3::prepare(temp/"three-bad-order.json",temp/"three-bad-order");},"recording.order");
    recording_cloud.object["recording"].object["intrinsics"] = file_intrinsics;
    save(temp / "recording-cloud.json", recording_cloud);
    rejected([&] { v3::prepare(temp / "recording-cloud.json", temp / "recording-inline"); },
             "recording.intrinsics is no longer supported");
    environment("INSTALL_ROOT", "");
    cfg.close();
    const auto cleanup_root=checkedFixtureRoot(temp,temp_parent);
    std::filesystem::remove_all(cleanup_root);
    std::cout << "Native configuration contracts passed (models/camera not executed).\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Native configuration test failed: " << error.what()
              << "\nFixture retained: " << temp.u8string() << '\n';
    return 1;
  }
}
