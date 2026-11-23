#pragma once
#include "perception/logging.hpp"
#include "perception/pipeline.hpp"
#include "perception/runtime_engine.hpp"
#include <optional>

namespace perception {
enum class RunStatus { Disabled, Ok, NotFound, Error };
const char *runStatusName(RunStatus) noexcept;
template <class T> struct TaskResult {
    RunStatus status = RunStatus::Disabled;
    std::string error;
    double elapsed_ms = 0;
    std::optional<T> value;
    ErrorInfo diagnostic;
};
struct EngineConfig {
    std::optional<Profile> detection, segmentation;
    std::shared_ptr<Logger> logger;
};
struct FrameResult {
    FrameInfo frame;
    TaskResult<DetectionResult> detection;
    TaskResult<SegmentationResult> segmentation;
    double elapsed_ms = 0;
    bool hasError() const noexcept;
};
// Loads each enabled model once. Runs enabled tasks sequentially on one frame;
// runtime failures are isolated per task. Initialization errors throw.
class PerceptionEngine {
  public:
    explicit PerceptionEngine(EngineConfig);
    // Dependency injection also permits bounded host tests with fake model backends.
    PerceptionEngine(EngineConfig, std::unique_ptr<Backend> detection_backend,
                     std::unique_ptr<Backend> segmentation_backend);
    // r12-v3 registered-feature engine. Strict JSON separates business Task
    // definitions from a nested sequence/parallel DAG; legacy cfg remains supported.
    explicit PerceptionEngine(runtime::ConfigPath task_graph_config);
    ~PerceptionEngine();
    PerceptionEngine(const PerceptionEngine &) = delete;
    PerceptionEngine &operator=(const PerceptionEngine &) = delete;
    FrameResult run(const ImageView &, FrameInfo = {});
    runtime::FrameResult run(const runtime::FrameInput &);
    std::vector<std::string> registeredFeatures() const;
    void close();        // Idempotent; run after close throws.
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace perception
