#pragma once

#include "perception/semantic_fusion.hpp"
#include "perception/task_graph.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace perception {
class StagedInference;
}

namespace perception::runtime {

struct ConfigPath {
    explicit ConfigPath(std::string path_value) : value(std::move(path_value)) {}
    std::string value;
};

// Loads the strict config_version=3 nested JSON DAG, the newcomer-oriented
// config_version=2 perception preset, or the fully expanded r12 cfg graph.
// Every format resolves to the same TaskGraphConfig and registered Features.
TaskGraphConfig loadPerceptionEngineConfig(const std::string &path);

struct FrameInput {
    ImageView image;
    std::optional<fusion::DisparityView> disparity;
    FrameInfo frame;
    // Must carry the same explicit coordinate frame and grid geometry as the
    // AI map; an unframed/default CameraOptical map is rejected by fusion.
    // Shared ownership avoids copying a full grid on every frame.
    std::shared_ptr<const fusion::GridMap> geometric_map;
    // Optional zero-copy inputs for the product AiFusion adapter. They are not
    // needed by the portable map fallback.
    std::shared_ptr<const fusion::FramedPointCloud> ground_cloud;
    std::shared_ptr<const fusion::FramedPointCloud> obstacle_cloud;
    std::shared_ptr<const fusion::AiFusionAdapter> ai_fusion_adapter;
    // Optional transport adapter. ROS2/MQTT/IPC code stays outside the portable
    // core. It is invoked only after the DAG has committed and engine locks have
    // been released, so an adapter may safely submit another frame.
    std::function<void(const fusion::Publication &)> publication_sink;
    // Optional output notification after either the product adapter or portable
    // fallback has generated both maps. This sink does not generate the maps.
    std::function<void(const fusion::GridMap &, const fusion::GridMap &)> ai_fusion_sink;
};

struct FrameResult {
    FrameInfo frame;
    GraphReport schedule;
    // These are aliases of immutable artifacts, not full-size copies.
    std::shared_ptr<const DetectionResult> detection;
    std::shared_ptr<const SegmentationResult> segmentation;
    std::shared_ptr<const fusion::PointCloud> point_cloud;
    std::shared_ptr<const fusion::FramedSemanticCloud> semantic_cloud;
    std::shared_ptr<const fusion::FramedSemanticCloud> percep_main_cloud;
    std::shared_ptr<const fusion::Publication> publication;
    std::shared_ptr<const fusion::GridMap> ai_map;
    std::shared_ptr<const fusion::GridMap> fusion_map;
    // Snapshot of committed outputs, including custom feature outputs and
    // configured result keys. Reserved input.* artifacts and internal
    // InferenceFrame invocations are excluded because they borrow caller or
    // workspace memory. The typed fields above alias this snapshot's payloads.
    std::shared_ptr<const ArtifactStore> artifacts;
    bool ok() const noexcept { return schedule.ok(); }
};

// Config-driven registered-feature engine used by r12-v3. One run is serialized so
// model workspaces remain reusable; independent tasks inside that run use the
// persistent DAG pool. Strict nested JSON and legacy cfg inputs are both accepted.
class Engine {
  public:
    explicit Engine(const std::string &task_graph_config);
    Engine(TaskGraphConfig, FeatureRegistry); // Extension/testing injection.
    ~Engine();
    Engine(const Engine &) = delete;
    Engine &operator=(const Engine &) = delete;

    FrameResult run(const FrameInput &);
    void close();
    std::vector<std::string> registeredFeatures() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Registers all r12-v3 built-ins. New compiled features are added in the same way;
// the task graph can then select them without changing scheduler code.
using StagedInferenceLoader =
    std::function<std::shared_ptr<StagedInference>(const std::string &profile_path)>;
void registerBuiltinFeatures(FeatureRegistry &, StagedInferenceLoader = {});

} // namespace perception::runtime
