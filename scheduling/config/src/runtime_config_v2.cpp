#include "perception/runtime_engine.hpp"
#include "runtime_config_json.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace perception::runtime {
namespace {
using Values = std::map<std::string, std::string>;

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

Values readValues(const std::string &path) {
    std::ifstream stream(std::filesystem::u8path(path));
    if (!stream)
        throw std::runtime_error("cannot open perception config: " + path);
    Values values;
    std::string line;
    int line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line_number == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0)
            line.erase(0, 3);
        line = trim(std::move(line));
        if (line.empty() || line.front() == '#')
            continue;
        const auto equal = line.find('=');
        if (equal == std::string::npos)
            throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                     ": expected key=value");
        const auto key = trim(line.substr(0, equal));
        const auto value = trim(line.substr(equal + 1));
        if (key.empty() || !values.emplace(key, value).second)
            throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                     ": empty or duplicate key");
    }
    return values;
}

std::string take(Values &values, const std::string &key,
                 const std::string &fallback) {
    const auto found = values.find(key);
    if (found == values.end())
        return fallback;
    auto value = found->second;
    values.erase(found);
    return value;
}

std::string takeRequired(Values &values, const std::string &key) {
    auto value = take(values, key, {});
    if (value.empty())
        throw std::runtime_error("perception preset requires " + key);
    return value;
}

bool booleanValue(const std::string &key, const std::string &value) {
    if (value == "true" || value == "1")
        return true;
    if (value == "false" || value == "0")
        return false;
    throw std::runtime_error("invalid boolean value for " + key + ": " + value);
}

std::size_t workerCount(const std::string &value) {
    std::size_t used = 0;
    unsigned long long parsed = 0;
    try {
        parsed = std::stoull(value, &used);
    } catch (const std::exception &) {
        throw std::runtime_error("invalid worker_threads value: " + value);
    }
    if (used != value.size() || parsed > std::numeric_limits<std::size_t>::max())
        throw std::runtime_error("invalid worker_threads value: " + value);
    return static_cast<std::size_t>(parsed);
}

void copyParameter(Values &values, Parameters &target, const std::string &friendly,
                   const std::string &feature, const std::string &fallback) {
    target.emplace(feature, take(values, friendly, fallback));
}

void copyOptionalParameter(Values &values, Parameters &target,
                           const std::string &friendly,
                           const std::string &feature) {
    const auto found = values.find(friendly);
    if (found == values.end())
        return;
    target.emplace(feature, found->second);
    values.erase(found);
}

TaskGraphConfig expandSemanticPerceptionPreset(Values values) {
    if (takeRequired(values, "config_version") != "2")
        throw std::runtime_error("only config_version=2 is supported by the simple preset");
    if (takeRequired(values, "pipeline.preset") != "semantic_perception")
        throw std::runtime_error("unknown pipeline.preset; expected semantic_perception");

    TaskGraphConfig config;
    config.worker_threads = workerCount(take(values, "worker_threads", "3"));
    config.result_artifacts = {
        "ai.detection", "ai.segmentation", "cloud.projected", "cloud.semantic",
        "cloud.semantic_percep_main", "output.publication", "map.ai", "map.fusion"};

    TaskSpec detection;
    detection.id = "detection";
    detection.features = {"ai.preprocess", "ai.backend", "ai.det.postprocess"};
    detection.parameters = {{"profile", takeRequired(values, "model.det.profile")},
                            {"invocation_key", "ai.det.frame"},
                            {"result_key", "ai.detection"}};
    copyOptionalParameter(values, detection.parameters, "model.det.instance",
                          "model_instance");
    config.tasks.push_back(std::move(detection));

    TaskSpec segmentation;
    segmentation.id = "segmentation";
    segmentation.features = {"ai.preprocess", "ai.backend", "ai.seg.postprocess"};
    segmentation.parameters = {{"profile", takeRequired(values, "model.seg.profile")},
                               {"invocation_key", "ai.seg.frame"},
                               {"result_key", "ai.segmentation"}};
    copyOptionalParameter(values, segmentation.parameters, "model.seg.instance",
                          "model_instance");
    config.tasks.push_back(std::move(segmentation));

    TaskSpec stereo;
    stereo.id = "stereo_geometry";
    stereo.features = {"pointcloud.disparity_to_cloud",
                       "pointcloud.project_to_image"};
    copyParameter(values, stereo.parameters, "camera.fx", "fx", {});
    copyParameter(values, stereo.parameters, "camera.fy", "fy", {});
    copyParameter(values, stereo.parameters, "camera.cx", "cx", {});
    copyParameter(values, stereo.parameters, "camera.cy", "cy", {});
    copyParameter(values, stereo.parameters, "camera.baseline_m", "baseline_m", {});
    copyParameter(values, stereo.parameters, "camera.width", "calibration_width", {});
    copyParameter(values, stereo.parameters, "camera.height", "calibration_height", {});
    for (const char *required : {"fx", "fy", "cx", "cy", "baseline_m",
                                 "calibration_width", "calibration_height"})
        if (stereo.parameters.at(required).empty())
            throw std::runtime_error("perception preset requires camera calibration");
    copyParameter(values, stereo.parameters, "camera.min_disparity", "min_disparity",
                  "0.01");
    copyParameter(values, stereo.parameters, "camera.min_depth_m", "min_depth_m",
                  "0.05");
    copyParameter(values, stereo.parameters, "camera.max_depth_m", "max_depth_m",
                  "10.0");
    copyParameter(values, stereo.parameters, "camera.sample_step", "sample_step", "1");
    copyParameter(values, stereo.parameters, "camera.max_points", "max_points", "20000");
    config.tasks.push_back(std::move(stereo));

    TaskSpec mapping;
    mapping.id = "semantic_mapping";
    mapping.features = {
        "fusion.match_detection_segmentation", "fusion.depth_filter",
        "fusion.voxel_filter", "fusion.connected_component_filter",
        "fusion.semantic_cloud", "pointcloud.transform", "pointcloud.transform"};
    mapping.depends_on = {"detection", "segmentation", "stereo_geometry"};
    copyParameter(values, mapping.parameters, "mapping.detection_score_min",
                  "detection_score_min", "0.30");
    copyParameter(values, mapping.parameters, "mapping.detection_include_ids",
                  "detection_include_ids", "8,9,10,18");
    copyParameter(values, mapping.parameters, "mapping.passable_detection_ids",
                  "passable_detection_ids", "");
    copyParameter(values, mapping.parameters, "mapping.passable_segmentation_ids",
                  "passable_segmentation_ids", "1");
    copyParameter(values, mapping.parameters, "mapping.segmentation_boundary_radius",
                  "segmentation_boundary_radius", "1");
    copyParameter(values, mapping.parameters, "mapping.depth_gap_m", "depth_gap_m",
                  "0.25");
    copyParameter(values, mapping.parameters, "mapping.depth_span_m", "depth_span_m",
                  "0.60");
    copyParameter(values, mapping.parameters, "mapping.depth_margin_m", "depth_margin_m",
                  "0.03");
    copyParameter(values, mapping.parameters, "mapping.depth_minimum_points",
                  "depth_minimum_points", "5");
    copyParameter(values, mapping.parameters, "mapping.voxel_size_m", "voxel_size_m",
                  "0.05");
    copyParameter(values, mapping.parameters, "mapping.voxel_minimum_points",
                  "voxel_minimum_points", "1");
    copyParameter(values, mapping.parameters, "mapping.component_minimum_points",
                  "component_minimum_points", "8");
    copyParameter(values, mapping.parameters, "mapping.filter_segmentation",
                  "filter_segmentation", "true");
    copyParameter(values, mapping.parameters, "mapping.min_depth_m", "min_depth_m",
                  "0.05");
    copyParameter(values, mapping.parameters, "mapping.max_depth_m", "max_depth_m",
                  "10.0");
    copyParameter(values, mapping.parameters, "mapping.min_y_m", "min_y_m", "-2.0");
    copyParameter(values, mapping.parameters, "mapping.max_y_m", "max_y_m", "2.0");
    copyParameter(values, mapping.parameters, "mapping.nongrass_seg_as_obstacle",
                  "nongrass_seg_as_obstacle", "false");
    mapping.feature_parameters.resize(mapping.features.size());
    const auto require_non_identity =
        take(values, "transform.require_non_identity", "true");
    mapping.feature_parameters[5] = {
        {"input_key", "cloud.semantic_camera"}, {"output_key", "cloud.semantic"},
        {"source_frame", "camera_optical"}, {"target_frame", "base_link"},
        {"transform", takeRequired(values, "transform.camera_to_base")},
        {"require_non_identity", require_non_identity}};
    mapping.feature_parameters[6] = {
        {"input_key", "cloud.semantic_camera"},
        {"output_key", "cloud.semantic_percep_main"},
        {"source_frame", "camera_optical"}, {"target_frame", "percep_main"},
        {"transform", takeRequired(values, "transform.camera_to_percep_main")},
        {"require_non_identity", require_non_identity}};
    config.tasks.push_back(std::move(mapping));

    TaskSpec publish;
    publish.id = "publish";
    publish.features = {"output.semantic_cloud_markers"};
    publish.depends_on = {"semantic_mapping"};
    publish.enabled = booleanValue("publish.enabled",
                                   take(values, "publish.enabled", "true"));
    copyParameter(values, publish.parameters, "publish.include_unknown",
                  "include_unknown", "true");
    copyParameter(values, publish.parameters, "publish.markers", "markers", "true");
    config.tasks.push_back(std::move(publish));

    TaskSpec ai_fusion;
    ai_fusion.id = "ai_fusion";
    ai_fusion.features = {"fusion.ai_fusion"};
    ai_fusion.depends_on = {"semantic_mapping"};
    ai_fusion.enabled = booleanValue("ai_fusion.enabled",
                                     take(values, "ai_fusion.enabled", "true"));
    copyParameter(values, ai_fusion.parameters, "ai_fusion.grid_rows", "grid_rows",
                  "100");
    copyParameter(values, ai_fusion.parameters, "ai_fusion.grid_cols", "grid_cols",
                  "80");
    copyParameter(values, ai_fusion.parameters, "ai_fusion.grid_resolution_m",
                  "grid_resolution_m", "0.02");
    copyParameter(values, ai_fusion.parameters, "ai_fusion.grid_origin_x_m",
                  "grid_origin_x_m", "0.0");
    copyParameter(values, ai_fusion.parameters, "ai_fusion.grid_origin_y_m",
                  "grid_origin_y_m", "-0.8");
    copyParameter(values, ai_fusion.parameters,
                  "ai_fusion.nongrass_seg_as_obstacle",
                  "nongrass_seg_as_obstacle", "false");
    copyParameter(values, ai_fusion.parameters, "ai_fusion.require_product_adapter",
                  "require_product_adapter", "true");
    copyParameter(values, ai_fusion.parameters, "ai_fusion.require_geometric_map",
                  "require_geometric_map", "false");
    copyOptionalParameter(values, ai_fusion.parameters,
                          "ai_fusion.person_detection_id", "person_detection_id");
    copyOptionalParameter(values, ai_fusion.parameters,
                          "ai_fusion.cat_detection_id", "cat_detection_id");
    copyOptionalParameter(values, ai_fusion.parameters,
                          "ai_fusion.dog_detection_id", "dog_detection_id");
    copyOptionalParameter(values, ai_fusion.parameters,
                          "ai_fusion.charger_detection_id", "charger_detection_id");
    copyOptionalParameter(values, ai_fusion.parameters,
                          "ai_fusion.toy_detection_id", "toy_detection_id");
    config.tasks.push_back(std::move(ai_fusion));

    if (!values.empty())
        throw std::runtime_error("unknown simple perception field: " +
                                 values.begin()->first);
    return config;
}
} // namespace

TaskGraphConfig loadPerceptionEngineConfig(const std::string &path) {
    auto extension = std::filesystem::u8path(path).extension().u8string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](char value) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
    });
    if (extension == ".json")
        return loadNestedJsonTaskGraphConfig(path);
    auto values = readValues(path);
    if (values.count("config_version") == 0 && values.count("pipeline.preset") == 0)
        return loadTaskGraphConfig(path);
    return expandSemanticPerceptionPreset(std::move(values));
}
} // namespace perception::runtime
