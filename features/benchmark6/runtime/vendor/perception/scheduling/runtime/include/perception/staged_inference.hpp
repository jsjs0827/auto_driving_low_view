#pragma once

#include "perception/backend.hpp"
#include "perception/model_plan.hpp"
#include <memory>

namespace perception {

// One frame invocation shared by the registered preprocess/backend/postprocess
// features. It is move-free and owned through shared_ptr by the task artifact store.
class InferenceFrame {
  public:
    ~InferenceFrame();
    InferenceFrame(const InferenceFrame &) = delete;
    InferenceFrame &operator=(const InferenceFrame &) = delete;

  private:
    struct Impl;
    explicit InferenceFrame(std::unique_ptr<Impl>);
    std::unique_ptr<Impl> impl_;
    friend class StagedInference;
};

// Explicit diagnostic copies. Normal inference does not allocate or hash these.
struct InferenceInputSnapshot {
    std::vector<std::uint8_t> bytes;
    Geometry geometry{};
    bool nv12 = false;
};

class StagedInference {
  public:
    StagedInference(ModelConfig, DetectionConfig);
    StagedInference(ModelConfig, SegmentationConfig);
    StagedInference(ModelConfig, DetectionConfig, std::unique_ptr<Backend>);
    StagedInference(ModelConfig, SegmentationConfig, std::unique_ptr<Backend>);
    ~StagedInference();
    StagedInference(const StagedInference &) = delete;
    StagedInference &operator=(const StagedInference &) = delete;

    std::shared_ptr<InferenceFrame> begin(const ImageView &);
    void preprocess(InferenceFrame &);
    void infer(InferenceFrame &);
    DetectionResult postprocessDetection(InferenceFrame &);
    SegmentationResult postprocessSegmentation(InferenceFrame &);
    // Call after preprocess / infer respectively, before the next stage.
    // Copies remain owned and valid after the frame completes. Diagnostic work
    // between stages is included in result.timing; diagnostic callers should
    // measure each stage call independently when reporting inference latency.
    InferenceInputSnapshot diagnosticInputSnapshot(const InferenceFrame &) const;
    Tensor diagnosticSegmentationSnapshot(const InferenceFrame &) const;
    // The configured task is immutable and can be queried concurrently with a run.
    Task task() const noexcept;
    ModelPlan plan() const;
    BackendState state() const;
    void close();

  private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

} // namespace perception
