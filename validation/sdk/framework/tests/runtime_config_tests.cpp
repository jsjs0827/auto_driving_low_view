#include "perception/engine.hpp"
#include "perception/runtime_engine.hpp"
#include "perception/staged_inference.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}

void expectConfigFailure(const std::filesystem::path &path,
                         const std::string &expected_text,
                         const char *failure_message) {
    try {
        (void)perception::runtime::loadPerceptionEngineConfig(path.u8string());
    } catch (const std::exception &error) {
        check(std::string(error.what()).find(expected_text) != std::string::npos,
              "configuration rejection omitted the offending field");
        return;
    }
    throw std::runtime_error(failure_message);
}

class FakeSegBackend final : public perception::Backend {
  public:
    std::array<int, 2> inputSize() const override { return {32, 32}; }
    perception::ModelInfo modelInfo() const override {
        return {32, 32, {{3, 8, 8}}, {}, false};
    }
    std::vector<perception::Tensor>
    run(const std::vector<std::uint8_t> &input) override {
        check(input.size() == 32U * 32U * 3U,
              "default DAG segmentation input size changed");
        perception::Tensor logits{3, 8, 8, std::vector<float>(3U * 8U * 8U, -4.0f)};
        std::fill(logits.data.begin() + 8U * 8U,
                  logits.data.begin() + 2U * 8U * 8U, 5.0f);
        return {std::move(logits)};
    }
};

class FakeDetBackend final : public perception::Backend {
  public:
    std::array<int, 2> inputSize() const override { return {32, 32}; }
    perception::ModelInfo modelInfo() const override {
        return {32,
                32,
                {{2, 4, 4}, {16, 4, 4}, {2, 2, 2},
                 {16, 2, 2}, {2, 1, 1}, {16, 1, 1}},
                {},
                false};
    }
    std::vector<perception::Tensor>
    run(const std::vector<std::uint8_t> &input) override {
        check(input.size() == 32U * 32U * 3U,
              "default DAG detection input size changed");
        std::vector<perception::Tensor> outputs;
        for (const int stride : {8, 16, 32}) {
            const int side = 32 / stride;
            outputs.push_back({2, side, side,
                               std::vector<float>(static_cast<std::size_t>(2 * side * side),
                                                  -10.0f)});
            outputs.push_back({16, side, side,
                               std::vector<float>(static_cast<std::size_t>(16 * side * side),
                                                  -4.0f)});
        }
        return outputs;
    }
};

perception::ModelConfig modelConfig() {
    perception::ModelConfig config;
    config.source.path = "injected";
    config.preprocess.width = config.preprocess.height = 32;
    config.preprocess.letterbox = false;
    return config;
}

perception::DetectionConfig detectionConfig() {
    perception::DetectionConfig config;
    config.classes = 2;
    config.reg_bins = 4;
    return config;
}

perception::SegmentationConfig segmentationConfig() {
    perception::SegmentationConfig config;
    config.classes = 3;
    config.seg_output = 0;
    return config;
}

perception::runtime::TaskSpec &taskById(perception::runtime::TaskGraphConfig &config,
                                        const std::string &id) {
    const auto found = std::find_if(config.tasks.begin(), config.tasks.end(),
                                    [&](const auto &task) { return task.id == id; });
    if (found == config.tasks.end())
        throw std::runtime_error("missing default task: " + id);
    return *found;
}

const perception::runtime::TaskSpec &
taskById(const perception::runtime::TaskGraphConfig &config, const std::string &id) {
    const auto found = std::find_if(config.tasks.begin(), config.tasks.end(),
                                    [&](const auto &task) { return task.id == id; });
    if (found == config.tasks.end())
        throw std::runtime_error("missing default task: " + id);
    return *found;
}

void checkDefaultGraphContract(const perception::runtime::TaskGraphConfig &config,
                               const perception::runtime::FeatureRegistry &registry) {
    struct ExpectedTask {
        const char *id;
        std::vector<std::string> features;
        std::vector<std::string> dependencies;
        bool enabled;
    };
    const std::vector<ExpectedTask> expected_tasks{
        {"detection", {"ai.preprocess", "ai.backend", "ai.det.postprocess"}, {},
         true},
        {"segmentation", {"ai.preprocess", "ai.backend", "ai.seg.postprocess"},
         {}, true},
        {"stereo_geometry",
         {"pointcloud.disparity_to_cloud", "pointcloud.project_to_image"}, {}, true},
        {"semantic_mapping",
         {"fusion.match_detection_segmentation", "fusion.depth_filter",
          "fusion.voxel_filter", "fusion.connected_component_filter",
          "fusion.semantic_cloud", "pointcloud.transform", "pointcloud.transform"},
         {"detection", "segmentation", "stereo_geometry"}, true},
        {"publish", {"output.semantic_cloud_markers"}, {"semantic_mapping"}, true},
        {"ai_fusion", {"fusion.ai_fusion"}, {"semantic_mapping"}, true},
    };
    check(config.worker_threads == 3, "default r12-v3 worker count changed");
    check(config.tasks.size() == expected_tasks.size(),
          "default r12-v3 task count changed");
    for (std::size_t index = 0; index < expected_tasks.size(); ++index) {
        const auto &actual = config.tasks[index];
        const auto &expected = expected_tasks[index];
        check(actual.id == expected.id, "default r12-v3 task order/id changed");
        check(actual.features == expected.features,
              "default r12-v3 task feature composition changed");
        check(actual.depends_on == expected.dependencies,
              "default r12-v3 task dependencies changed");
        check(actual.enabled == expected.enabled,
              "default r12-v3 task enabled/disabled state changed");
    }

    const std::vector<std::string> expected_features{
        "ai.backend", "ai.det.postprocess", "ai.preprocess", "ai.seg.postprocess",
        "fusion.ai_fusion", "fusion.ai_map", "fusion.connected_component_filter",
        "fusion.depth_filter", "fusion.fusion_map",
        "fusion.match_detection_segmentation", "fusion.semantic_cloud",
        "fusion.voxel_connected_filter", "fusion.voxel_filter",
        "output.semantic_cloud_markers", "pointcloud.disparity_to_cloud",
        "pointcloud.project_to_image", "pointcloud.transform"};
    check(registry.names() == expected_features,
          "built-in feature registry differs from the exact 17-feature contract");
}

class RecordingAiFusionAdapter final : public perception::fusion::AiFusionAdapter {
  public:
    mutable std::atomic<int> calls{0};
    mutable std::atomic<bool> received_product_contract{false};

    perception::fusion::AiFusionMaps integrate(
        const perception::fusion::FramedPointCloud &ground,
        const perception::fusion::FramedPointCloud &obstacles,
        const perception::fusion::FramedSemanticCloud &semantic,
        const perception::fusion::GridConfig &config,
        const perception::fusion::AiGridPolicy &policy) const override {
        ++calls;
        check(ground.frame == perception::fusion::CoordinateFrame::BaseLink &&
                  obstacles.frame == perception::fusion::CoordinateFrame::BaseLink &&
                  semantic.frame == perception::fusion::CoordinateFrame::BaseLink,
              "AiFusion adapter did not receive three base_link clouds");
        check(!ground.points.empty() && !obstacles.points.empty() &&
                  !semantic.points.empty(),
              "AiFusion adapter received an empty product cloud");
        check(config.rows == 100 && config.cols == 80 &&
                  std::fabs(config.resolution_m - 0.02f) < 1e-6f &&
                  std::fabs(config.origin_x_m) < 1e-6f &&
                  std::fabs(config.origin_y_m + 0.8f) < 1e-6f,
              "AiFusion adapter grid contract differs from the product map");
        check(!policy.nongrass_seg_as_obstacle && policy.person_detection_id == 8 &&
                  policy.cat_detection_id == 9 && policy.dog_detection_id == 10 &&
                  policy.charger_detection_id == 18 && policy.toy_detection_id == 16,
              "AiFusion adapter semantic policy changed unexpectedly");
        received_product_contract = true;
        perception::fusion::AiFusionMaps maps;
        maps.ai_map.frame = semantic.frame;
        maps.ai_map.config = config;
        maps.ai_map.cells.resize(static_cast<std::size_t>(config.rows) * config.cols);
        maps.ai_map.cells.front().type = 4;
        maps.fusion_map = maps.ai_map;
        return maps;
    }
};

void defaultDagEndToEnd(const perception::runtime::TaskGraphConfig &template_config) {
    auto config = template_config;
    taskById(config, "detection").parameters["profile"] = "det";
    taskById(config, "segmentation").parameters["profile"] = "seg";
    auto &mapping = taskById(config, "semantic_mapping");
    check(mapping.feature_parameters.size() == mapping.features.size(),
          "default semantic_mapping feature overrides are incomplete");
    mapping.feature_parameters[5]["transform"] =
        "1,0,0,0.10,0,1,0,0,0,0,1,0,0,0,0,1";
    mapping.feature_parameters[6]["transform"] =
        "1,0,0,0,0,1,0,0.20,0,0,1,0,0,0,0,1";
    taskById(config, "ai_fusion").parameters["require_product_adapter"] = "true";

    auto detection = std::make_shared<perception::StagedInference>(
        modelConfig(), detectionConfig(), std::make_unique<FakeDetBackend>());
    auto segmentation = std::make_shared<perception::StagedInference>(
        modelConfig(), segmentationConfig(), std::make_unique<FakeSegBackend>());
    perception::runtime::FeatureRegistry registry;
    perception::runtime::registerBuiltinFeatures(
        registry, [&](const std::string &profile) {
            if (profile == "det")
                return detection;
            if (profile == "seg")
                return segmentation;
            throw std::runtime_error("unexpected default DAG profile: " + profile);
        });
    perception::runtime::Engine engine(std::move(config), std::move(registry));

    const std::vector<std::uint8_t> pixels(32U * 32U * 3U, 96);
    const std::vector<std::uint16_t> disparity(32U * 32U, 256);
    auto ground = std::make_shared<perception::fusion::FramedPointCloud>();
    ground->frame = perception::fusion::CoordinateFrame::BaseLink;
    ground->points.resize(1);
    ground->points.front().x = 0.25f;
    auto obstacles = std::make_shared<perception::fusion::FramedPointCloud>();
    obstacles->frame = perception::fusion::CoordinateFrame::BaseLink;
    obstacles->points.resize(1);
    obstacles->points.front().x = 0.50f;
    auto adapter = std::make_shared<RecordingAiFusionAdapter>();

    perception::runtime::FrameInput input;
    input.image = {pixels.data(), pixels.size(), 32U * 3U, 32, 32,
                   perception::PixelFormat::RGB};
    input.disparity = perception::fusion::DisparityView{
        disparity.data(), disparity.size() * sizeof(disparity[0]), 32U * 2U, 32, 32,
        perception::fusion::DisparityType::UInt16, 256.0f};
    input.frame = {101, 202};
    input.ground_cloud = ground;
    input.obstacle_cloud = obstacles;
    input.ai_fusion_adapter = adapter;

    const auto result = engine.run(input);
    check(result.ok(), "default r12-v3 DAG failed end-to-end");
    check(result.schedule.tasks.size() == template_config.tasks.size() &&
              result.schedule.tasks.size() == 6,
          "default r12-v3 DAG did not execute all 6 business tasks");
    check(std::all_of(result.schedule.tasks.begin(), result.schedule.tasks.end(),
                      [](const auto &task) {
                          return task.state == perception::runtime::TaskState::Succeeded;
                      }),
          "a default r12-v3 DAG task did not succeed");
    check(result.detection && result.detection->width == 32 &&
              result.detection->height == 32,
          "default DAG detection result is missing");
    check(result.segmentation && result.segmentation->width == 32 &&
              result.segmentation->height == 32 &&
              result.segmentation->mask.size() == 32U * 32U,
          "default DAG segmentation result is missing");
    check(result.point_cloud && !result.point_cloud->empty(),
          "default DAG projected cloud is missing");
    check(result.semantic_cloud && !result.semantic_cloud->points.empty() &&
              result.semantic_cloud->frame ==
                  perception::fusion::CoordinateFrame::BaseLink,
          "default DAG base_link semantic cloud is missing");
    check(result.percep_main_cloud && !result.percep_main_cloud->points.empty() &&
              result.percep_main_cloud->frame ==
                  perception::fusion::CoordinateFrame::PercepMain,
          "default DAG percep_main semantic cloud is missing");
    check(result.publication && result.publication->frame.id == 101 &&
              result.publication->coordinate_frame ==
                  perception::fusion::CoordinateFrame::BaseLink,
          "default DAG publication is missing");
    check(result.ai_map && result.fusion_map && result.ai_map->config.rows == 100 &&
              result.ai_map->config.cols == 80 &&
              result.ai_map->cells.size() == 100U * 80U &&
              result.fusion_map->cells.size() == 100U * 80U &&
              result.ai_map->frame == perception::fusion::CoordinateFrame::BaseLink &&
              result.fusion_map->frame == perception::fusion::CoordinateFrame::BaseLink,
          "default DAG AiFusion outputs are missing");
    check(adapter->calls == 1 && adapter->received_product_contract,
          "default DAG did not invoke the product AiFusion adapter exactly once");

    check(result.artifacts && result.artifacts->contains("ai.detection") &&
              result.artifacts->contains("ai.segmentation") &&
              result.artifacts->contains("cloud.semantic") &&
              result.artifacts->contains("cloud.semantic_percep_main") &&
              result.artifacts->contains("output.publication") &&
              result.artifacts->contains("map.ai") &&
              result.artifacts->contains("map.fusion"),
          "default DAG result artifact snapshot is incomplete");
    for (const auto &key : result.artifacts->keys())
        check(key.rfind("input.", 0) != 0,
              "result artifact snapshot retained a borrowed input");
    check(!result.artifacts->contains("ai.det.frame") &&
              !result.artifacts->contains("ai.seg.frame"),
          "result artifact snapshot retained an InferenceFrame invocation");
    check(!result.artifacts->contains("cloud.raw") &&
              !result.artifacts->contains("fusion.matches") &&
              !result.artifacts->contains("fusion.depth_matches") &&
              !result.artifacts->contains("fusion.voxel_matches") &&
              !result.artifacts->contains("fusion.filtered_matches"),
          "default result snapshot retained transient DAG artifacts");
    check(result.detection == result.artifacts->get<perception::DetectionResult>(
                                    "ai.detection") &&
              result.semantic_cloud ==
                  result.artifacts->get<perception::fusion::FramedSemanticCloud>(
                      "cloud.semantic") &&
              result.ai_map ==
                  result.artifacts->get<perception::fusion::GridMap>("map.ai"),
          "typed FrameResult fields copied large artifacts instead of aliasing them");
    engine.close();
}

void requiredAiFusionAdapterFailure() {
    perception::runtime::FeatureRegistry registry;
    perception::runtime::registerBuiltinFeatures(registry);
    perception::runtime::TaskGraphConfig config;
    config.worker_threads = 1;
    config.tasks.push_back(
        {"ai_fusion_required",
         {"fusion.ai_fusion"},
         {},
         {{"require_product_adapter", "true"},
          {"grid_rows", "2"},
          {"grid_cols", "2"},
          {"grid_resolution_m", "0.5"}},
         true});
    perception::runtime::TaskGraph graph(std::move(config), registry);
    perception::runtime::ArtifactStore artifacts;
    perception::fusion::FramedSemanticCloud cloud;
    cloud.frame = perception::fusion::CoordinateFrame::BaseLink;
    cloud.points.resize(1);
    artifacts.put("cloud.semantic", std::move(cloud));
    const auto report = graph.run(artifacts);
    check(!report.ok() && report.tasks.size() == 1 &&
              report.tasks.front().state == perception::runtime::TaskState::Failed &&
              report.tasks.front().error.find("adapter is unavailable") !=
                  std::string::npos,
          "required AiFusion adapter absence did not fail explicitly");
    check(!artifacts.contains("map.ai") && !artifacts.contains("map.fusion"),
          "failed AiFusion task committed partial map outputs");
    graph.close();
    registry.close();
}

void relativeAliasedProfileResolution() {
    namespace fs = std::filesystem;
    const auto unique = std::chrono::high_resolution_clock::now()
                            .time_since_epoch()
                            .count();
    const auto directory = fs::temp_directory_path() /
                           fs::u8path(u8"感知配置_r12_runtime_" +
                                      std::to_string(unique));
    fs::create_directories(directory);
    const auto graph_path = directory / "relative_profile.cfg";
    const auto relative_profile = fs::u8path(u8"模型/不存在.cfg");
    const auto expected = (directory / relative_profile).lexically_normal().u8string();
    {
        std::ofstream graph(graph_path);
        if (!graph)
            throw std::runtime_error("cannot create relative-profile graph test");
        graph << "worker_threads=1\n"
                 "tasks=model\n"
                 "task.model.features=prep:ai.preprocess\n"
                 "task.model.depends_on=\n"
                 "task.model.param.profile=wrong.cfg\n"
                 "task.model.feature.prep.param.profile="
              << relative_profile.u8string() << "\n"
                 "task.model.param.invocation_key=internal.frame\n";
    }
    perception::runtime::Engine engine(graph_path.u8string());
    const std::vector<std::uint8_t> pixel{1, 2, 3};
    perception::runtime::FrameInput input;
    input.image = {pixel.data(), pixel.size(), 3, 1, 1,
                   perception::PixelFormat::RGB};

    const auto original_cwd = fs::current_path();
    perception::runtime::FrameResult result;
    try {
        fs::current_path(fs::temp_directory_path());
        result = engine.run(input);
        fs::current_path(original_cwd);
    } catch (...) {
        fs::current_path(original_cwd);
        throw;
    }
    check(!result.ok() && result.schedule.tasks.size() == 1 &&
              result.schedule.tasks.front().state ==
                  perception::runtime::TaskState::Failed,
          "missing relative profile unexpectedly succeeded");
    check(result.schedule.tasks.front().error.find(expected) != std::string::npos,
          "aliased feature profile was not resolved against the graph directory");
    engine.close();
    fs::remove_all(directory);
}

void configurationCompatibilityAndRejection(
    const std::filesystem::path &default_v3_path,
    const perception::runtime::FeatureRegistry &registry) {
    namespace fs = std::filesystem;

    const auto legacy_path = default_v3_path.parent_path() / "runtime_r12_default.cfg";
    const auto legacy =
        perception::runtime::loadPerceptionEngineConfig(legacy_path.u8string());
    legacy.validate(registry);
    check(legacy.worker_threads == 4 && legacy.tasks.size() == 13 &&
              legacy.tasks.front().id == "detect" &&
              legacy.tasks.back().id == "ai_fusion",
          "r12 expanded task configuration is no longer backward compatible");

    const auto default_v2_path =
        default_v3_path.parent_path() / "runtime_r12_v2_default.cfg";
    const auto v2 =
        perception::runtime::loadPerceptionEngineConfig(default_v2_path.u8string());
    v2.validate(registry);
    check(v2.worker_threads == 3 && v2.tasks.size() == 6 &&
              v2.tasks.front().id == "detection" &&
              v2.tasks.back().id == "ai_fusion",
          "r12-v2 preset configuration is no longer backward compatible");

    std::ifstream input(default_v2_path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot reopen the r12-v2 compatibility configuration");
    const std::string valid((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    const auto unique = std::chrono::high_resolution_clock::now()
                            .time_since_epoch()
                            .count();
    const auto directory = fs::temp_directory_path() /
                           ("perception-r12-v3-compat-config-tests-" +
                            std::to_string(unique));
    fs::create_directories(directory);
    const auto unknown_path = directory / "unknown-field.cfg";
    const auto missing_path = directory / "missing-field.cfg";
    {
        std::ofstream output(unknown_path, std::ios::binary);
        output << valid << "\nmodel.det.profiel=typo.cfg\n";
    }
    expectConfigFailure(unknown_path, "model.det.profiel",
                        "r12-v2 simple config accepted an unknown field");

    auto missing = valid;
    const auto first = missing.find("model.seg.profile=");
    check(first != std::string::npos,
          "r12-v2 default configuration lost model.seg.profile");
    const auto next = missing.find('\n', first);
    missing.erase(first, next == std::string::npos ? std::string::npos
                                                   : next - first + 1);
    {
        std::ofstream output(missing_path, std::ios::binary);
        output << missing;
    }
    expectConfigFailure(missing_path, "model.seg.profile",
                        "r12-v2 simple config accepted a missing required field");

    std::error_code ignored;
    fs::remove_all(directory, ignored);
}

class EmitOutputsFeature final : public perception::runtime::Feature {
  public:
    void execute(perception::runtime::FeatureContext &context) override {
        perception::fusion::Publication publication;
        publication.frame = *context.get<perception::FrameInfo>("input.frame");
        perception::fusion::GridMap map;
        map.config = {1, 1, 1.0f, 0.0f, 0.0f};
        map.cells.resize(1);
        context.set("output.publication", std::move(publication));
        context.set("map.ai", map);
        context.set("map.fusion", std::move(map));
        context.set<int>("extension.custom_value", 73);
    }
};

class CheckGeometricMapAliasFeature final : public perception::runtime::Feature {
  public:
    CheckGeometricMapAliasFeature(const perception::fusion::GridMap *expected,
                                  std::atomic<bool> &executed)
        : expected_(expected), executed_(executed) {}

    void execute(perception::runtime::FeatureContext &context) override {
        const auto actual =
            context.get<perception::fusion::GridMap>("input.geometric_map");
        check(actual.get() == expected_,
              "FrameInput geometric map was copied before Feature execution");
        executed_ = true;
    }

  private:
    const perception::fusion::GridMap *expected_;
    std::atomic<bool> &executed_;
};

void geometricMapInputIsZeroCopy() {
    auto geometric = std::make_shared<perception::fusion::GridMap>();
    geometric->frame = perception::fusion::CoordinateFrame::BaseLink;
    geometric->config = {1, 1, 1.0f, 0.0f, 0.0f};
    geometric->cells.resize(1);
    std::atomic<bool> executed{false};
    perception::runtime::FeatureRegistry registry;
    registry.add("test.check_geometric_alias",
                 [expected = geometric.get(), &executed](
                     const perception::runtime::Parameters &) {
                     return std::make_unique<CheckGeometricMapAliasFeature>(expected,
                                                                           executed);
                 });
    perception::runtime::TaskGraphConfig config{
        1, {{"check_alias", {"test.check_geometric_alias"}, {}, {}, true}}};
    perception::runtime::Engine engine(std::move(config), std::move(registry));
    const std::vector<std::uint8_t> pixel{1, 2, 3};
    perception::runtime::FrameInput input;
    input.image = {pixel.data(), pixel.size(), 3, 1, 1,
                   perception::PixelFormat::RGB};
    input.geometric_map = geometric;
    check(engine.run(input).ok() && executed,
          "geometric-map zero-copy runtime graph failed");
    engine.close();
}

void distinctModelInstancesRejectSharedSession() {
    auto shared_model = std::make_shared<perception::StagedInference>(
        modelConfig(), detectionConfig(), std::make_unique<FakeDetBackend>());
    perception::runtime::FeatureRegistry registry;
    perception::runtime::registerBuiltinFeatures(
        registry, [shared_model](const std::string &) { return shared_model; });
    perception::runtime::TaskGraphConfig config{
        1,
        {{"first", {"ai.preprocess"}, {},
          {{"profile", "same.cfg"}, {"model_instance", "first"},
           {"invocation_key", "first.frame"}}, true},
         {"second", {"ai.preprocess"}, {"first"},
          {{"profile", "same.cfg"}, {"model_instance", "second"},
           {"invocation_key", "second.frame"}}, true}}};
    perception::runtime::Engine engine(std::move(config), std::move(registry));
    const std::vector<std::uint8_t> pixels(32U * 32U * 3U, 64);
    perception::runtime::FrameInput input;
    input.image = {pixels.data(), pixels.size(), 32U * 3U, 32, 32,
                   perception::PixelFormat::RGB};
    const auto result = engine.run(input);
    const auto second = std::find_if(result.schedule.tasks.begin(),
                                     result.schedule.tasks.end(), [](const auto &task) {
                                         return task.id == "second";
                                     });
    check(!result.ok() && second != result.schedule.tasks.end() &&
              second->state == perception::runtime::TaskState::Failed &&
              second->error.find("reused one session") != std::string::npos,
          "loader reuse across distinct model_instance resources was not rejected");
    engine.close();
}

void deferredCallbacksAndRegistryClose() {
    perception::runtime::FeatureRegistry registry;
    registry.add("test.emit", [](const perception::runtime::Parameters &) {
        return std::make_unique<EmitOutputsFeature>();
    });
    std::atomic<int> closes{0};
    registry.addCloser([&] { ++closes; });
    perception::runtime::TaskGraphConfig config{
        1, {{"emit", {"test.emit"}, {}, {}, true}}};
    perception::runtime::Engine engine(std::move(config), std::move(registry));
    const std::vector<std::uint8_t> pixel{1, 2, 3};
    perception::runtime::FrameInput input;
    input.image = {pixel.data(), pixel.size(), 3, 1, 1, perception::PixelFormat::RGB};
    input.frame = {41, 42};
    std::atomic<int> publications{0}, map_callbacks{0};
    input.ai_fusion_sink = [&](const perception::fusion::GridMap &ai,
                               const perception::fusion::GridMap &fusion) {
        if (ai.cells.size() == 1 && fusion.cells.size() == 1)
            ++map_callbacks;
    };
    input.publication_sink = [&](const perception::fusion::Publication &publication) {
        check(publication.frame.id == 41, "deferred publication has wrong frame");
        ++publications;
        // Re-enter the same engine. This deadlocked when the transport callback
        // ran from a DAG worker while Engine::run still held its mutex.
        auto nested = input;
        nested.publication_sink = {};
        nested.ai_fusion_sink = {};
        check(engine.run(nested).ok(), "reentrant runtime run failed");
    };
    const auto result = engine.run(input);
    check(result.ok(), "runtime output graph failed");
    check(result.artifacts &&
              *result.artifacts->get<int>("extension.custom_value") == 73,
          "custom registered feature output was discarded by Engine::run");
    check(publications == 1 && map_callbacks == 1,
          "deferred output callbacks were not invoked exactly once");
    engine.close();
    engine.close();
    check(closes == 1, "feature registry close hook was not exactly once");

    perception::runtime::FeatureRegistry invalid_registry;
    std::atomic<int> construction_closes{0};
    invalid_registry.addCloser([&] { ++construction_closes; });
    perception::runtime::TaskGraphConfig invalid_config{
        1, {{"bad", {"not.registered"}, {}, {}, true}}};
    try {
        perception::runtime::Engine invalid(std::move(invalid_config),
                                            std::move(invalid_registry));
        throw std::runtime_error("invalid graph unexpectedly constructed");
    } catch (const std::invalid_argument &) {
    }
    check(construction_closes == 1,
          "graph construction failure did not close registered resources");
}
} // namespace

int main(int argc, char **argv) {
    try {
        if (argc != 2)
            throw std::invalid_argument(
                "usage: runtime_config_tests GRAPH.(json|cfg)");
        const auto config =
            perception::runtime::loadPerceptionEngineConfig(argv[1]);
        perception::runtime::FeatureRegistry registry;
        perception::runtime::registerBuiltinFeatures(registry);
        config.validate(registry);
        checkDefaultGraphContract(config, registry);
        const auto &mapping = taskById(config, "semantic_mapping");
        if (mapping.feature_parameters.size() != mapping.features.size() ||
            mapping.feature_parameters[6].at("target_frame") != "percep_main" ||
            mapping.feature_parameters[6].at("require_non_identity") != "true")
            throw std::runtime_error("default percep_main transform branch is missing");
        if (taskById(config, "ai_fusion").parameters.at("require_product_adapter") !=
            "true")
            throw std::runtime_error(
                "default product graph permits a silent portable AiFusion fallback");
        {
            auto misspelled = config;
            auto &parameters = taskById(misspelled, "ai_fusion").parameters;
            parameters.erase("require_product_adapter");
            parameters["require_prodcut_adapter"] = "true";
            try {
                misspelled.validate(registry);
                throw std::runtime_error(
                    "misspelled safety-critical built-in parameter was accepted");
            } catch (const std::invalid_argument &error) {
                check(std::string(error.what()).find("require_prodcut_adapter") !=
                          std::string::npos,
                      "unknown built-in parameter error omitted its name");
            }
        }
        {
            perception::runtime::TaskGraphConfig shared_model{
                2,
                {{"left", {"ai.preprocess"}, {},
                  {{"profile", "same.cfg"}, {"invocation_key", "left.frame"}}, true},
                 {"right", {"ai.preprocess"}, {},
                  {{"profile", "same.cfg"}, {"invocation_key", "right.frame"}}, true}}};
            try {
                shared_model.validate(registry);
                throw std::runtime_error(
                    "parallel AI tasks sharing one model workspace were accepted");
            } catch (const std::invalid_argument &error) {
                check(std::string(error.what()).find("exclusive resource") !=
                          std::string::npos,
                      "shared model rejection did not identify its resource conflict");
            }
            shared_model.tasks[1].parameters["model_instance"] = "right-session";
            shared_model.validate(registry);
            shared_model.tasks[1].parameters.erase("model_instance");
            shared_model.tasks[1].depends_on = {"left"};
            shared_model.validate(registry);
        }
        bool rejected_identity = false;
        try {
            perception::PerceptionEngine engine{perception::runtime::ConfigPath(argv[1])};
        } catch (const std::exception &error) {
            rejected_identity = std::string(error.what()).find("still identity") !=
                                std::string::npos;
        }
        if (!rejected_identity)
            throw std::runtime_error(
                "default template accepted uncalibrated coordinate transforms");

        // Exercise every registered non-model feature through the real task graph.
        // Model-stage behavior is covered by staged_inference_tests with an injected backend.
        perception::runtime::TaskGraphConfig pipeline_config;
        pipeline_config.worker_threads = 3;
        auto task = [&](std::string id, std::string feature,
                        std::vector<std::string> dependencies,
                        perception::runtime::Parameters parameters = {}) {
            pipeline_config.tasks.push_back({std::move(id), {std::move(feature)},
                                             std::move(dependencies),
                                             std::move(parameters), true});
        };
        const perception::runtime::Parameters calibration{
            {"fx", "10"}, {"fy", "10"}, {"cx", "0"}, {"cy", "0"},
            {"baseline_m", "0.1"}, {"calibration_width", "2"},
            {"calibration_height", "2"}};
        task("cloud", "pointcloud.disparity_to_cloud", {}, calibration);
        task("project", "pointcloud.project_to_image", {"cloud"}, calibration);
        task("match", "fusion.match_detection_segmentation", {"project"},
             {{"passable_segmentation_ids", "1"}, {"segmentation_boundary_radius", "0"}});
        task("depth", "fusion.depth_filter", {"match"},
             {{"depth_minimum_points", "1"}});
        task("voxel", "fusion.voxel_filter", {"depth"},
             {{"voxel_minimum_points", "1"},
              {"output_key", "fusion.voxel_matches"}});
        task("legacy_voxel_components", "fusion.voxel_connected_filter", {"depth"},
             {{"voxel_minimum_points", "1"},
              {"component_minimum_points", "1"},
              {"output_key", "fusion.legacy_filtered_matches"}});
        task("components", "fusion.connected_component_filter", {"voxel"},
             {{"component_minimum_points", "1"},
              {"input_key", "fusion.voxel_matches"},
              {"output_key", "fusion.filtered_matches"}});
        task("semantic", "fusion.semantic_cloud", {"components"},
             {{"passable_segmentation_ids", "1"}});
        task("transform", "pointcloud.transform", {"semantic"}, {});
        task("publish", "output.semantic_cloud_markers", {"transform"}, {});
        task("ai_map", "fusion.ai_map", {"transform"},
             {{"grid_rows", "8"}, {"grid_cols", "8"},
              {"grid_resolution_m", "0.5"}, {"grid_origin_x_m", "-1"},
              {"grid_origin_y_m", "-1"}});
        task("fusion_map", "fusion.fusion_map", {"ai_map"}, {});
        task("ai_fusion_fallback", "fusion.ai_fusion", {"transform"},
             {{"grid_rows", "8"}, {"grid_cols", "8"},
              {"grid_resolution_m", "0.5"}, {"grid_origin_x_m", "-1"},
              {"grid_origin_y_m", "-1"},
              {"ai_output_key", "map.ai_fallback"},
              {"fusion_output_key", "map.fusion_fallback"}});
        perception::runtime::FeatureRegistry pipeline_registry;
        perception::runtime::registerBuiltinFeatures(pipeline_registry);
        perception::runtime::TaskGraph pipeline_graph(std::move(pipeline_config),
                                                       pipeline_registry);
        perception::runtime::ArtifactStore artifacts;
        const std::vector<std::uint8_t> pixels(2U * 2U * 3U, 100);
        const std::vector<std::uint16_t> disparity(4, 256);
        artifacts.put("input.image",
                      perception::ImageView{pixels.data(), pixels.size(), 6, 2, 2,
                                            perception::PixelFormat::RGB});
        artifacts.put("input.disparity",
                      perception::fusion::DisparityView{
                          disparity.data(), disparity.size() * sizeof(disparity[0]), 4, 2, 2,
                          perception::fusion::DisparityType::UInt16, 256.0f});
        perception::DetectionResult detection;
        detection.width = detection.height = 2;
        detection.detections.push_back({5, 0.9f, 0, 0, 2, 2});
        artifacts.put("ai.detection", detection);
        perception::SegmentationResult segmentation;
        segmentation.width = segmentation.height = 2;
        segmentation.mask = {1, 1, 1, 1};
        artifacts.put("ai.segmentation", segmentation);
        artifacts.put("input.frame", perception::FrameInfo{7, 8});
        const auto pipeline_report = pipeline_graph.run(artifacts);
        if (!pipeline_report.ok() ||
            !artifacts.contains("cloud.raw") || !artifacts.contains("cloud.projected") ||
            !artifacts.contains("fusion.matches") ||
            !artifacts.contains("fusion.depth_matches") ||
            !artifacts.contains("fusion.voxel_matches") ||
            !artifacts.contains("fusion.legacy_filtered_matches") ||
            !artifacts.contains("fusion.filtered_matches") ||
            !artifacts.contains("cloud.semantic") ||
            !artifacts.contains("output.publication") || !artifacts.contains("map.ai") ||
            !artifacts.contains("map.fusion") ||
            !artifacts.contains("map.ai_fallback") ||
            !artifacts.contains("map.fusion_fallback"))
            throw std::runtime_error("registered non-model feature pipeline failed");
        const auto publication =
            artifacts.get<perception::fusion::Publication>("output.publication");
        const auto published_source =
            artifacts.get<perception::fusion::FramedSemanticCloud>("cloud.semantic");
        if (publication->frame.id != 7 || !publication->cloud ||
            publication->cloud->size() != 4 ||
            publication->cloud.get() != &published_source->points)
            throw std::runtime_error("registered publication payload is wrong");
        pipeline_graph.close();
        pipeline_registry.close();
        configurationCompatibilityAndRejection(std::filesystem::u8path(argv[1]),
                                               registry);
        registry.close();
        defaultDagEndToEnd(config);
        requiredAiFusionAdapterFailure();
        relativeAliasedProfileResolution();
        deferredCallbacksAndRegistryClose();
        geometricMapInputIsZeroCopy();
        distinctModelInstancesRejectSharedSession();
        std::cout
            << "PASS r12-v3 JSON plus r12-v2/legacy configuration and built-in feature registration\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
