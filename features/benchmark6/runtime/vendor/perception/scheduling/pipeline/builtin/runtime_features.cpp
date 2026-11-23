#include "perception/runtime_engine.hpp"
#include "perception/config.hpp"
#include "perception/error.hpp"
#include "perception/staged_inference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

namespace perception::runtime {
namespace {
using Invocation = std::shared_ptr<InferenceFrame>;

std::vector<std::string> split(const std::string &value) {
    std::vector<std::string> out;
    std::istringstream stream(value);
    std::string part;
    while (std::getline(stream, part, ',')) {
        const auto first = part.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
            continue;
        part = part.substr(first, part.find_last_not_of(" \t\r\n") - first + 1);
        out.push_back(std::move(part));
    }
    return out;
}

std::vector<int> integerList(const std::string &value) {
    std::vector<int> out;
    for (const auto &part : split(value)) {
        std::size_t used = 0;
        const int parsed = std::stoi(part, &used);
        if (used != part.size())
            throw std::invalid_argument("invalid integer list");
        out.push_back(parsed);
    }
    return out;
}

std::vector<float> floatList(const std::string &value) {
    std::vector<float> out;
    for (const auto &part : split(value)) {
        std::size_t used = 0;
        const float parsed = std::stof(part, &used);
        if (used != part.size() || !std::isfinite(parsed))
            throw std::invalid_argument("invalid float list");
        out.push_back(parsed);
    }
    return out;
}

std::string parameter(const Parameters &parameters, const std::string &key,
                      const std::string &fallback = {}) {
    const auto found = parameters.find(key);
    return found == parameters.end() ? fallback : found->second;
}

bool booleanParameter(const Parameters &parameters, const std::string &key, bool fallback) {
    const auto found = parameters.find(key);
    if (found == parameters.end())
        return fallback;
    if (found->second == "true" || found->second == "1")
        return true;
    if (found->second == "false" || found->second == "0")
        return false;
    throw std::invalid_argument("boolean must be true/false: " + key);
}

FeatureParameterSchema featureSchema(
    std::initializer_list<const char *> strings = {},
    std::initializer_list<const char *> integers = {},
    std::initializer_list<const char *> floats = {},
    std::initializer_list<const char *> booleans = {},
    std::initializer_list<const char *> integer_lists = {},
    std::initializer_list<const char *> float_lists = {}) {
    FeatureParameterSchema result;
    const auto add = [&](std::initializer_list<const char *> names,
                         FeatureParameterType type) {
        for (const char *name : names)
            if (!result.emplace(name, type).second)
                throw std::logic_error(std::string("duplicate feature schema key: ") + name);
    };
    add(strings, FeatureParameterType::String);
    add(integers, FeatureParameterType::Integer);
    add(floats, FeatureParameterType::Float);
    add(booleans, FeatureParameterType::Boolean);
    add(integer_lists, FeatureParameterType::IntegerList);
    add(float_lists, FeatureParameterType::FloatList);
    return result;
}

std::vector<std::string> aiExclusiveResources(const Parameters &parameters) {
    const auto profile = parameter(parameters, "profile");
    if (profile.empty())
        throw std::invalid_argument("AI feature requires profile parameter");
    const auto normalized = std::filesystem::u8path(profile).lexically_normal().u8string();
    const auto model = parameter(parameters, "model");
    const auto normalized_model =
        model.empty() ? std::string{} : std::filesystem::u8path(model).lexically_normal().u8string();
    return {"ai-model:" + normalized + "#" + normalized_model + "#" +
            parameter(parameters, "model_instance")};
}

fusion::StereoCalibration calibration(const FeatureContext &context) {
    fusion::StereoCalibration result;
    result.fx = context.parameterFloat("fx", 0);
    result.fy = context.parameterFloat("fy", 0);
    result.cx = context.parameterFloat("cx", 0);
    result.cy = context.parameterFloat("cy", 0);
    result.baseline_m = context.parameterFloat("baseline_m", 0);
    result.calibration_width = context.parameterInt("calibration_width", 0);
    result.calibration_height = context.parameterInt("calibration_height", 0);
    result.validate();
    return result;
}

fusion::MatchConfig matchConfig(const FeatureContext &context) {
    fusion::MatchConfig result;
    result.detection_score_min = context.parameterFloat("detection_score_min", 0.25f);
    result.detection_include_ids =
        integerList(context.parameter("detection_include_ids", ""));
    result.passable_detection_ids =
        integerList(context.parameter("passable_detection_ids", ""));
    result.passable_segmentation_ids =
        integerList(context.parameter("passable_segmentation_ids", ""));
    result.segmentation_boundary_radius = context.parameterInt("segmentation_boundary_radius", 1);
    return result;
}

fusion::GridConfig gridConfig(const FeatureContext &context) {
    fusion::GridConfig config;
    config.rows = context.parameterInt("grid_rows", 200);
    config.cols = context.parameterInt("grid_cols", 200);
    config.resolution_m = context.parameterFloat("grid_resolution_m", 0.05f);
    config.origin_x_m = context.parameterFloat("grid_origin_x_m", 0.0f);
    config.origin_y_m = context.parameterFloat("grid_origin_y_m", -5.0f);
    config.validate();
    return config;
}

fusion::AiGridPolicy aiGridPolicy(const FeatureContext &context) {
    fusion::AiGridPolicy policy;
    policy.nongrass_seg_as_obstacle = context.parameterBool(
        "nongrass_seg_as_obstacle", policy.nongrass_seg_as_obstacle);
    policy.person_detection_id =
        context.parameterInt("person_detection_id", policy.person_detection_id);
    policy.cat_detection_id = context.parameterInt("cat_detection_id", policy.cat_detection_id);
    policy.dog_detection_id = context.parameterInt("dog_detection_id", policy.dog_detection_id);
    policy.charger_detection_id =
        context.parameterInt("charger_detection_id", policy.charger_detection_id);
    policy.toy_detection_id = context.parameterInt("toy_detection_id", policy.toy_detection_id);
    return policy;
}

void requireBaseLinkFrame(fusion::CoordinateFrame frame, const char *artifact) {
    if (frame != fusion::CoordinateFrame::BaseLink)
        throw std::invalid_argument(std::string(artifact) +
                                    " must use base_link coordinates in metres");
}

void validateMapOutput(const fusion::GridMap &map, const fusion::GridConfig &expected,
                       fusion::CoordinateFrame expected_frame, const char *name) {
    map.config.validate();
    const auto cells = static_cast<std::size_t>(expected.rows) *
                       static_cast<std::size_t>(expected.cols);
    if (map.frame != expected_frame ||
        map.config.rows != expected.rows || map.config.cols != expected.cols ||
        std::fabs(map.config.resolution_m - expected.resolution_m) > 1e-6f ||
        std::fabs(map.config.origin_x_m - expected.origin_x_m) > 1e-6f ||
        std::fabs(map.config.origin_y_m - expected.origin_y_m) > 1e-6f ||
        map.cells.size() != cells)
        throw std::runtime_error(std::string("AiFusion adapter returned incompatible ") + name);
    for (const auto &cell : map.cells) {
        if (cell.type < 0 || cell.type > 8 || !std::isfinite(cell.min_height) ||
            !std::isfinite(cell.max_height) || !std::isfinite(cell.average_height) ||
            (cell.point_count &&
             (cell.min_height > cell.max_height ||
              cell.average_height + 1e-5f < cell.min_height ||
              cell.average_height > cell.max_height + 1e-5f)))
            throw std::runtime_error(std::string("AiFusion adapter returned invalid ") +
                                     name + " cell data");
    }
}

fusion::CoordinateFrame frameFromString(const std::string &value) {
    if (value == "camera_optical")
        return fusion::CoordinateFrame::CameraOptical;
    if (value == "base_link")
        return fusion::CoordinateFrame::BaseLink;
    if (value == "percep_main")
        return fusion::CoordinateFrame::PercepMain;
    throw std::invalid_argument("unknown coordinate frame: " + value);
}

class ModelCache {
  public:
    explicit ModelCache(StagedInferenceLoader loader) : loader_(std::move(loader)) {}

    std::shared_ptr<StagedInference> get(const std::string &path,
                                         const std::string &instance,
                                         const std::string &model_override) {
        if (path.empty())
            throw std::invalid_argument("AI feature requires profile parameter");
        const std::string cache_key = path + '\x1f' + model_override + '\x1f' + instance;
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_)
            throw std::runtime_error("AI model cache is closed");
        const auto found = models_.find(cache_key);
        if (found != models_.end())
            return found->second;
        std::shared_ptr<StagedInference> model;
        if (loader_) {
            if (!model_override.empty())
                throw std::invalid_argument(
                    "model override is unavailable with an injected staged inference loader");
            model = loader_(path);
            if (!model)
                throw std::runtime_error("staged inference loader returned null: " + path);
            for (const auto &existing : models_)
                if (existing.second == model)
                    throw std::runtime_error(
                        "staged inference loader reused one session for distinct "
                        "profile/model_instance resources");
        } else {
            auto profile = loadProfile(path);
            if (!model_override.empty())
                profile.model.source.path = model_override;
            if (std::holds_alternative<DetectionConfig>(profile.task))
                model = std::make_shared<StagedInference>(
                    profile.model, std::get<DetectionConfig>(profile.task));
            else
                model = std::make_shared<StagedInference>(
                    profile.model, std::get<SegmentationConfig>(profile.task));
        }
        models_.emplace(cache_key, model);
        return model;
    }

    void close() {
        std::unordered_map<std::string, std::shared_ptr<StagedInference>> models;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_)
                return;
            closed_ = true;
            models.swap(models_);
        }
        std::exception_ptr first;
        for (auto &model : models)
            try {
                model.second->close();
            } catch (...) {
                if (!first)
                    first = std::current_exception();
            }
        if (first)
            std::rethrow_exception(first);
    }

  private:
    std::mutex mutex_;
    StagedInferenceLoader loader_;
    std::unordered_map<std::string, std::shared_ptr<StagedInference>> models_;
    bool closed_ = false;
};

class AiPreprocessFeature final : public Feature {
  public:
    explicit AiPreprocessFeature(std::shared_ptr<ModelCache> cache) : cache_(std::move(cache)) {}
    void execute(FeatureContext &context) override {
        auto model = cache_->get(context.parameter("profile"),
                                 context.parameter("model_instance"),
                                 context.parameter("model"));
        const auto image = context.get<ImageView>(context.parameter("image_key", "input.image"));
        auto invocation = model->begin(*image);
        model->preprocess(*invocation);
        context.set<Invocation>(context.parameter("invocation_key", "ai.frame"),
                                std::move(invocation));
    }
  private:
    std::shared_ptr<ModelCache> cache_;
};

class AiBackendFeature final : public Feature {
  public:
    explicit AiBackendFeature(std::shared_ptr<ModelCache> cache) : cache_(std::move(cache)) {}
    void execute(FeatureContext &context) override {
        auto model = cache_->get(context.parameter("profile"),
                                 context.parameter("model_instance"),
                                 context.parameter("model"));
        const auto invocation =
            context.get<Invocation>(context.parameter("invocation_key", "ai.frame"));
        model->infer(**invocation);
    }
  private:
    std::shared_ptr<ModelCache> cache_;
};

class AiPostprocessFeature final : public Feature {
  public:
    AiPostprocessFeature(std::shared_ptr<ModelCache> cache, Task task)
        : cache_(std::move(cache)), task_(task) {}
    void execute(FeatureContext &context) override {
        auto model = cache_->get(context.parameter("profile"),
                                 context.parameter("model_instance"),
                                 context.parameter("model"));
        if (model->task() != task_)
            throw std::invalid_argument("AI postprocess feature/profile task mismatch");
        const auto invocation =
            context.get<Invocation>(context.parameter("invocation_key", "ai.frame"));
        if (task_ == Task::Detection)
            context.set<DetectionResult>(context.parameter("result_key", "ai.detection"),
                                         model->postprocessDetection(**invocation));
        else
            context.set<SegmentationResult>(context.parameter("result_key", "ai.segmentation"),
                                            model->postprocessSegmentation(**invocation));
    }
  private:
    std::shared_ptr<ModelCache> cache_;
    Task task_;
};

class DisparityCloudFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto disparity =
            context.get<fusion::DisparityView>(context.parameter("disparity_key", "input.disparity"));
        const auto image = context.get<ImageView>(context.parameter("image_key", "input.image"));
        fusion::CloudGenerationConfig config;
        config.min_disparity = context.parameterFloat("min_disparity", config.min_disparity);
        config.min_depth_m = context.parameterFloat("min_depth_m", config.min_depth_m);
        config.max_depth_m = context.parameterFloat("max_depth_m", config.max_depth_m);
        config.sample_step = context.parameterInt("sample_step", config.sample_step);
        const int max_points = context.parameterInt("max_points", 0);
        if (max_points < 0)
            throw std::invalid_argument("max_points must not be negative");
        config.max_points = static_cast<std::size_t>(max_points);
        context.set<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.raw"),
            fusion::disparityToPointCloud(*disparity, *image, calibration(context), config));
    }
};

class ProjectionFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud =
            context.get<fusion::PointCloud>(context.parameter("input_cloud_key", "cloud.raw"));
        const auto image = context.get<ImageView>(context.parameter("image_key", "input.image"));
        context.set<fusion::PointCloud>(
            context.parameter("output_cloud_key", "cloud.projected"),
            fusion::projectPointCloudToImage(*cloud, calibration(context), image->width,
                                             image->height));
    }
};

class MatchFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.projected"));
        const auto detections = context.get<DetectionResult>(
            context.parameter("detection_key", "ai.detection"));
        const auto segmentation = context.get<SegmentationResult>(
            context.parameter("segmentation_key", "ai.segmentation"));
        const auto image = context.get<ImageView>(context.parameter("image_key", "input.image"));
        context.set<std::vector<fusion::MatchRecord>>(
            context.parameter("output_key", "fusion.matches"),
            fusion::matchDetectionsAndMask(*cloud, detections->detections, *segmentation,
                                           image->width, image->height, matchConfig(context)));
    }
};

class DepthFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.projected"));
        const auto detections = context.get<DetectionResult>(
            context.parameter("detection_key", "ai.detection"));
        auto matches = *context.get<std::vector<fusion::MatchRecord>>(
            context.parameter("input_key", "fusion.matches"));
        fusion::DepthFilterConfig config;
        config.cluster_gap_m = context.parameterFloat("depth_gap_m", config.cluster_gap_m);
        config.cluster_span_m = context.parameterFloat("depth_span_m", config.cluster_span_m);
        config.margin_m = context.parameterFloat("depth_margin_m", config.margin_m);
        config.minimum_points = context.parameterInt("depth_minimum_points", config.minimum_points);
        fusion::filterDetectionDepth(*cloud, detections->detections, matchConfig(context), config,
                                     matches);
        context.set<std::vector<fusion::MatchRecord>>(
            context.parameter("output_key", "fusion.depth_matches"), std::move(matches));
    }
};

class VoxelFilterFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.projected"));
        const auto detections = context.get<DetectionResult>(
            context.parameter("detection_key", "ai.detection"));
        auto matches = *context.get<std::vector<fusion::MatchRecord>>(
            context.parameter("input_key", "fusion.depth_matches"));
        fusion::VoxelMinimumFilterConfig config;
        config.voxel_size_m = context.parameterFloat("voxel_size_m", config.voxel_size_m);
        config.minimum_points_per_voxel =
            context.parameterInt("voxel_minimum_points", config.minimum_points_per_voxel);
        config.filter_segmentation =
            context.parameterBool("filter_segmentation", config.filter_segmentation);
        context.set<fusion::VoxelFilterResult>(
            context.parameter("output_key", "fusion.voxel_matches"),
            fusion::filterVoxelMinimumPoints(*cloud, detections->detections, config,
                                             std::move(matches)));
    }
};

class ConnectedComponentFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.projected"));
        const auto detections = context.get<DetectionResult>(
            context.parameter("detection_key", "ai.detection"));
        const auto input = context.get<fusion::VoxelFilterResult>(
            context.parameter("input_key", "fusion.voxel_matches"));
        fusion::ConnectedComponentFilterConfig config;
        config.minimum_component_points =
            context.parameterInt("component_minimum_points", config.minimum_component_points);
        config.filter_segmentation =
            context.parameterBool("filter_segmentation", config.filter_segmentation);
        context.set<std::vector<fusion::MatchRecord>>(
            context.parameter("output_key", "fusion.filtered_matches"),
            fusion::filterConnectedComponents(*cloud, detections->detections, config, *input));
    }
};

class LegacyVoxelConnectedFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.projected"));
        const auto detections = context.get<DetectionResult>(
            context.parameter("detection_key", "ai.detection"));
        auto matches = *context.get<std::vector<fusion::MatchRecord>>(
            context.parameter("input_key", "fusion.depth_matches"));
        fusion::VoxelFilterConfig config;
        config.voxel_size_m = context.parameterFloat("voxel_size_m", config.voxel_size_m);
        config.minimum_points_per_voxel =
            context.parameterInt("voxel_minimum_points", config.minimum_points_per_voxel);
        config.minimum_component_points =
            context.parameterInt("component_minimum_points", config.minimum_component_points);
        config.filter_segmentation =
            context.parameterBool("filter_segmentation", config.filter_segmentation);
        fusion::filterVoxelComponents(*cloud, detections->detections, config, matches);
        context.set<std::vector<fusion::MatchRecord>>(
            context.parameter("output_key", "fusion.filtered_matches"), std::move(matches));
    }
};

class SemanticFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::PointCloud>(
            context.parameter("cloud_key", "cloud.projected"));
        const auto detections = context.get<DetectionResult>(
            context.parameter("detection_key", "ai.detection"));
        const auto matches = context.get<std::vector<fusion::MatchRecord>>(
            context.parameter("matches_key", "fusion.filtered_matches"));
        fusion::SemanticConfig config;
        config.min_depth_m = context.parameterFloat("min_depth_m", config.min_depth_m);
        config.max_depth_m = context.parameterFloat("max_depth_m", config.max_depth_m);
        config.min_y_m = context.parameterFloat("min_y_m", config.min_y_m);
        config.max_y_m = context.parameterFloat("max_y_m", config.max_y_m);
        config.nongrass_seg_as_obstacle = context.parameterBool(
            "nongrass_seg_as_obstacle", config.nongrass_seg_as_obstacle);
        config.matching = matchConfig(context);
        fusion::FramedSemanticCloud framed;
        framed.frame = fusion::CoordinateFrame::CameraOptical;
        framed.points = fusion::generateSemanticCloud(*cloud, detections->detections, *matches,
                                                      config);
        context.set<fusion::FramedSemanticCloud>(
            context.parameter("output_key", "cloud.semantic_camera"), std::move(framed));
    }
};

class TransformFeature final : public Feature {
  public:
    explicit TransformFeature(const Parameters &parameters)
        : input_key_(parameter(parameters, "input_key", "cloud.semantic_camera")),
          output_key_(parameter(parameters, "output_key", "cloud.semantic")) {
        const auto values = floatList(parameter(
            parameters, "transform", "1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1"));
        if (values.size() != 16)
            throw std::invalid_argument("transform requires 16 row-major values");
        std::copy(values.begin(), values.end(), transform_.matrix.begin());
        transform_.source = frameFromString(
            parameter(parameters, "source_frame", "camera_optical"));
        transform_.target = frameFromString(parameter(parameters, "target_frame", "base_link"));
        transform_.validate();
        if (booleanParameter(parameters, "require_non_identity", false)) {
            static constexpr std::array<float, 16> identity{
                1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            bool is_identity = true;
            for (std::size_t i = 0; i < identity.size(); ++i)
                is_identity = is_identity &&
                              std::fabs(transform_.matrix[i] - identity[i]) < 1e-6f;
            if (is_identity)
                throw std::invalid_argument(
                    "transform is still identity; replace it with calibrated extrinsics");
        }
    }

    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::FramedSemanticCloud>(input_key_);
        context.set<fusion::FramedSemanticCloud>(
            output_key_, fusion::transformSemanticCloud(*cloud, transform_));
    }

  private:
    std::string input_key_;
    std::string output_key_;
    fusion::RigidTransform transform_;
};

class PublicationFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::FramedSemanticCloud>(
            context.parameter("cloud_key", "cloud.semantic"));
        const auto frame = context.get<FrameInfo>(context.parameter("frame_key", "input.frame"));
        auto publication = fusion::makePublication(
            cloud, *frame, context.parameterBool("include_unknown", true),
            context.parameterBool("markers", true));
        context.set<fusion::Publication>(context.parameter("output_key", "output.publication"),
                                         std::move(publication));
    }
};

class AiMapFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::FramedSemanticCloud>(
            context.parameter("cloud_key", "cloud.semantic"));
        requireBaseLinkFrame(cloud->frame, "AI semantic cloud");
        context.set<fusion::GridMap>(context.parameter("output_key", "map.ai"),
                                     fusion::projectToAiMap(*cloud, gridConfig(context),
                                                            aiGridPolicy(context)));
    }
};

class AiFusionFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto cloud = context.get<fusion::FramedSemanticCloud>(
            context.parameter("cloud_key", "cloud.semantic"));
        requireBaseLinkFrame(cloud->frame, "AI semantic cloud");
        const auto config = gridConfig(context);
        const auto policy = aiGridPolicy(context);
        fusion::AiFusionMaps maps;
        const auto adapter_key =
            context.parameter("adapter_key", "input.ai_fusion_adapter");
        if (context.contains(adapter_key)) {
            const auto adapter = context.get<fusion::AiFusionAdapter>(adapter_key);
            const auto ground = context.get<fusion::FramedPointCloud>(
                context.parameter("ground_cloud_key", "input.ground_cloud"));
            const auto obstacles = context.get<fusion::FramedPointCloud>(
                context.parameter("obstacle_cloud_key", "input.obstacle_cloud"));
            requireBaseLinkFrame(ground->frame, "AiFusion ground cloud");
            requireBaseLinkFrame(obstacles->frame, "AiFusion obstacle cloud");
            maps = adapter->integrate(*ground, *obstacles, *cloud, config, policy);
        } else {
            if (context.parameterBool("require_product_adapter", false))
                throw std::runtime_error("required product AiFusion adapter is unavailable");
            maps.ai_map = fusion::projectToAiMap(*cloud, config, policy);
            fusion::GridMap geometry;
            const auto geometry_key =
                context.parameter("geometric_key", "input.geometric_map");
            if (context.contains(geometry_key)) {
                geometry = *context.get<fusion::GridMap>(geometry_key);
            } else if (context.parameterBool("require_geometric_map", false)) {
                throw std::runtime_error("required geometric map artifact not found: " +
                                         geometry_key);
            } else {
                geometry.frame = cloud->frame;
                geometry.config = config;
                geometry.cells.resize(maps.ai_map.cells.size());
            }
            maps.fusion_map = fusion::fuseMaps(maps.ai_map, geometry);
        }
        validateMapOutput(maps.ai_map, config, cloud->frame, "ai_map");
        validateMapOutput(maps.fusion_map, config, cloud->frame, "fusion_map");
        context.set<fusion::GridMap>(context.parameter("ai_output_key", "map.ai"),
                                     std::move(maps.ai_map));
        context.set<fusion::GridMap>(context.parameter("fusion_output_key", "map.fusion"),
                                     std::move(maps.fusion_map));
    }
};

class OptionalFusionMapFeature final : public Feature {
    void execute(FeatureContext &context) override {
        const auto ai = context.get<fusion::GridMap>(context.parameter("ai_key", "map.ai"));
        fusion::GridMap geometry;
        geometry.frame = ai->frame;
        geometry.config = ai->config;
        geometry.cells.resize(ai->cells.size());
        const auto key = context.parameter("geometric_key", "input.geometric_map");
        if (context.contains(key)) {
            geometry = *context.get<fusion::GridMap>(key);
        } else if (context.parameterBool("require_geometric_map", false)) {
            throw std::runtime_error("required geometric map artifact not found: " + key);
        }
        context.set<fusion::GridMap>(context.parameter("output_key", "map.fusion"),
                                     fusion::fuseMaps(*ai, geometry));
    }
};
} // namespace

namespace {
void registerBuiltinFeaturesImpl(FeatureRegistry &registry, StagedInferenceLoader loader = {}) {
    auto models = std::make_shared<ModelCache>(std::move(loader));
    registry.addCloser([models] { models->close(); });
    registry.add("ai.preprocess", [models](const Parameters &) {
        return std::make_unique<AiPreprocessFeature>(models);
    }, featureSchema({"profile", "model", "model_instance", "image_key", "invocation_key"}),
                 aiExclusiveResources);
    registry.add("ai.backend", [models](const Parameters &) {
        return std::make_unique<AiBackendFeature>(models);
    }, featureSchema({"profile", "model", "model_instance", "invocation_key"}),
                 aiExclusiveResources);
    registry.add("ai.det.postprocess", [models](const Parameters &) {
        return std::make_unique<AiPostprocessFeature>(models, Task::Detection);
    }, featureSchema({"profile", "model", "model_instance", "invocation_key", "result_key"}),
                 aiExclusiveResources);
    registry.add("ai.seg.postprocess", [models](const Parameters &) {
        return std::make_unique<AiPostprocessFeature>(models, Task::Segmentation);
    }, featureSchema({"profile", "model", "model_instance", "invocation_key", "result_key"}),
                 aiExclusiveResources);
    registry.add("pointcloud.disparity_to_cloud", [](const Parameters &) {
        return std::make_unique<DisparityCloudFeature>();
    }, featureSchema({"disparity_key", "image_key", "cloud_key"},
                     {"calibration_width", "calibration_height", "sample_step", "max_points"},
                     {"fx", "fy", "cx", "cy", "baseline_m", "min_disparity",
                      "min_depth_m", "max_depth_m"}));
    registry.add("pointcloud.project_to_image", [](const Parameters &) {
        return std::make_unique<ProjectionFeature>();
    }, featureSchema({"input_cloud_key", "image_key", "output_cloud_key"},
                     {"calibration_width", "calibration_height"},
                     {"fx", "fy", "cx", "cy", "baseline_m"}));
    registry.add("fusion.match_detection_segmentation", [](const Parameters &) {
        return std::make_unique<MatchFeature>();
    }, featureSchema({"cloud_key", "detection_key", "segmentation_key", "image_key",
                      "output_key"},
                     {"segmentation_boundary_radius"}, {"detection_score_min"}, {},
                     {"detection_include_ids", "passable_detection_ids",
                      "passable_segmentation_ids"}));
    registry.add("fusion.depth_filter", [](const Parameters &) {
        return std::make_unique<DepthFeature>();
    }, featureSchema({"cloud_key", "detection_key", "input_key", "output_key"},
                     {"segmentation_boundary_radius", "depth_minimum_points"},
                     {"detection_score_min", "depth_gap_m", "depth_span_m",
                      "depth_margin_m"}, {},
                     {"detection_include_ids", "passable_detection_ids",
                      "passable_segmentation_ids"}));
    registry.add("fusion.voxel_filter", [](const Parameters &) {
        return std::make_unique<VoxelFilterFeature>();
    }, featureSchema({"cloud_key", "detection_key", "input_key", "output_key"},
                     {"voxel_minimum_points"}, {"voxel_size_m"},
                     {"filter_segmentation"}));
    registry.add("fusion.connected_component_filter", [](const Parameters &) {
        return std::make_unique<ConnectedComponentFeature>();
    }, featureSchema({"cloud_key", "detection_key", "input_key", "output_key"},
                     {"component_minimum_points"}, {}, {"filter_segmentation"}));
    registry.add("fusion.voxel_connected_filter", [](const Parameters &) {
        return std::make_unique<LegacyVoxelConnectedFeature>();
    }, featureSchema({"cloud_key", "detection_key", "input_key", "output_key"},
                     {"voxel_minimum_points", "component_minimum_points"},
                     {"voxel_size_m"}, {"filter_segmentation"}));
    registry.add("fusion.semantic_cloud", [](const Parameters &) {
        return std::make_unique<SemanticFeature>();
    }, featureSchema({"cloud_key", "detection_key", "matches_key", "output_key"},
                     {"segmentation_boundary_radius"},
                     {"detection_score_min", "min_depth_m", "max_depth_m", "min_y_m",
                      "max_y_m"},
                     {"nongrass_seg_as_obstacle"},
                     {"detection_include_ids", "passable_detection_ids",
                      "passable_segmentation_ids"}));
    registry.add("pointcloud.transform", [](const Parameters &parameters) {
        return std::make_unique<TransformFeature>(parameters);
    }, featureSchema({"input_key", "output_key", "source_frame", "target_frame"}, {}, {},
                     {"require_non_identity"}, {}, {"transform"}));
    registry.add("output.semantic_cloud_markers", [](const Parameters &) {
        return std::make_unique<PublicationFeature>();
    }, featureSchema({"cloud_key", "frame_key", "output_key"}, {}, {},
                     {"include_unknown", "markers"}));
    registry.add("fusion.ai_map", [](const Parameters &) {
        return std::make_unique<AiMapFeature>();
    }, featureSchema({"cloud_key", "output_key"},
                     {"grid_rows", "grid_cols", "person_detection_id", "cat_detection_id",
                      "dog_detection_id", "charger_detection_id", "toy_detection_id"},
                     {"grid_resolution_m", "grid_origin_x_m", "grid_origin_y_m"},
                     {"nongrass_seg_as_obstacle"}));
    registry.add("fusion.fusion_map", [](const Parameters &) {
        return std::make_unique<OptionalFusionMapFeature>();
    }, featureSchema({"ai_key", "geometric_key", "output_key"}, {}, {},
                     {"require_geometric_map"}));
    registry.add("fusion.ai_fusion", [](const Parameters &) {
        return std::make_unique<AiFusionFeature>();
    }, featureSchema({"cloud_key", "adapter_key", "ground_cloud_key",
                      "obstacle_cloud_key", "geometric_key", "ai_output_key",
                      "fusion_output_key"},
                     {"grid_rows", "grid_cols", "person_detection_id", "cat_detection_id",
                      "dog_detection_id", "charger_detection_id", "toy_detection_id"},
                     {"grid_resolution_m", "grid_origin_x_m", "grid_origin_y_m"},
                     {"nongrass_seg_as_obstacle", "require_product_adapter",
                      "require_geometric_map"}));
}
} // namespace

void registerBuiltinFeatures(FeatureRegistry &registry, StagedInferenceLoader loader) {
    registerBuiltinFeaturesImpl(registry, std::move(loader));
}

struct Engine::Impl {
    FeatureRegistry registry;
    std::unique_ptr<TaskGraph> graph;
    std::vector<std::string> result_artifacts;
    std::mutex run_mutex;
    bool closed = false;
};

namespace {
void resolveProfilePaths(TaskGraphConfig &config, const std::filesystem::path &base) {
    const auto resolve = [&](Parameters &parameters) {
        for (const char *key : {"profile", "model"}) {
            const auto found = parameters.find(key);
            if (found == parameters.end() || found->second.empty())
                continue;
            auto path = std::filesystem::u8path(found->second);
            if (path.is_relative())
                found->second = (base / path).lexically_normal().u8string();
        }
    };
    for (auto &task : config.tasks) {
        resolve(task.parameters);
        for (auto &feature_parameters : task.feature_parameters)
            resolve(feature_parameters);
    }
}
} // namespace

Engine::Engine(const std::string &task_graph_config) : impl_(std::make_unique<Impl>()) {
    registerBuiltinFeaturesImpl(impl_->registry);
    try {
        const auto config_path =
            std::filesystem::absolute(std::filesystem::u8path(task_graph_config))
                .lexically_normal();
        auto config = loadPerceptionEngineConfig(config_path.u8string());
        resolveProfilePaths(config, config_path.parent_path());
        impl_->result_artifacts = config.result_artifacts;
        impl_->graph = std::make_unique<TaskGraph>(std::move(config), impl_->registry);
    } catch (...) {
        try {
            impl_->registry.close();
        } catch (...) {
            // Preserve the configuration/construction error.
        }
        throw;
    }
}

Engine::Engine(TaskGraphConfig config, FeatureRegistry registry)
    : impl_(std::make_unique<Impl>()) {
    impl_->registry = std::move(registry);
    try {
        impl_->result_artifacts = config.result_artifacts;
        impl_->graph = std::make_unique<TaskGraph>(std::move(config), impl_->registry);
    } catch (...) {
        try {
            impl_->registry.close();
        } catch (...) {
            // Preserve the graph construction error.
        }
        throw;
    }
}
Engine::~Engine() {
    try {
        close();
    } catch (...) {
        // Destructors must not terminate a process during exception unwinding.
    }
}

FrameResult Engine::run(const FrameInput &input) {
    FrameResult output;
    const auto publication_sink = input.publication_sink;
    const auto ai_fusion_sink = input.ai_fusion_sink;
    {
        std::lock_guard<std::mutex> lock(impl_->run_mutex);
        if (impl_->closed)
            throw std::runtime_error("runtime perception engine is closed");
        validateCameraImage(input.image);
        auto artifacts = std::make_shared<ArtifactStore>();
        artifacts->put<ImageView>("input.image", input.image);
        artifacts->put<FrameInfo>("input.frame", input.frame);
        if (input.disparity)
            artifacts->put<fusion::DisparityView>("input.disparity", *input.disparity);
        if (input.geometric_map)
            artifacts->putShared<fusion::GridMap>("input.geometric_map",
                                                   input.geometric_map);
        if (input.ground_cloud)
            artifacts->putShared<fusion::FramedPointCloud>("input.ground_cloud",
                                                            input.ground_cloud);
        if (input.obstacle_cloud)
            artifacts->putShared<fusion::FramedPointCloud>("input.obstacle_cloud",
                                                            input.obstacle_cloud);
        if (input.ai_fusion_adapter)
            artifacts->putShared<fusion::AiFusionAdapter>("input.ai_fusion_adapter",
                                                           input.ai_fusion_adapter);
        output.frame = input.frame;
        output.schedule = impl_->graph->run(*artifacts);
        const auto &requested = impl_->result_artifacts;
        const auto exported = artifacts->snapshot([&](const std::string &key,
                                                       std::type_index type) {
            if (key.rfind("input.", 0) == 0 ||
                type == std::type_index(typeid(Invocation)))
                return false;
            return requested.empty() ||
                   std::find(requested.begin(), requested.end(), key) != requested.end();
        });
        auto extract = [&](const std::string &key, auto &destination) {
            using Element = typename std::decay_t<decltype(destination)>::element_type;
            using Value = std::remove_const_t<Element>;
            if (exported->contains(key))
                destination = exported->get<Value>(key);
        };
        extract("ai.detection", output.detection);
        extract("ai.segmentation", output.segmentation);
        extract("cloud.projected", output.point_cloud);
        extract("cloud.semantic", output.semantic_cloud);
        extract("cloud.semantic_percep_main", output.percep_main_cloud);
        extract("output.publication", output.publication);
        extract("map.ai", output.ai_map);
        extract("map.fusion", output.fusion_map);
        output.artifacts = std::move(exported);
    }
    // External code is deliberately called after releasing both the DAG and
    // engine serialization locks. This makes publication re-entry safe.
    if (publication_sink && output.publication)
        publication_sink(*output.publication);
    if (ai_fusion_sink && output.ai_map && output.fusion_map)
        ai_fusion_sink(*output.ai_map, *output.fusion_map);
    return output;
}

void Engine::close() {
    if (!impl_)
        return;
    std::lock_guard<std::mutex> lock(impl_->run_mutex);
    if (impl_->closed)
        return;
    impl_->closed = true;
    std::exception_ptr first;
    try {
        if (impl_->graph)
            impl_->graph->close();
    } catch (...) {
        first = std::current_exception();
    }
    try {
        impl_->registry.close();
    } catch (...) {
        if (!first)
            first = std::current_exception();
    }
    if (first)
        std::rethrow_exception(first);
}

std::vector<std::string> Engine::registeredFeatures() const {
    return impl_->registry.names();
}

} // namespace perception::runtime
