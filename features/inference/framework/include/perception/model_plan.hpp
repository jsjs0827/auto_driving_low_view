#pragma once
#include "perception/config.hpp"
namespace perception {
class ModelPlan {
  public:
    ModelPlan(ModelConfig, TaskConfig, ModelInfo);
    bool ready() const noexcept { return ready_; }
    const ModelConfig &model() const noexcept { return model_; }
    const ModelInfo &info() const noexcept { return info_; }
    Task task() const noexcept;
    const DetectionConfig &detection() const;
    const SegmentationConfig &segmentation() const;
    const std::array<int, 6> &heads() const;
    // Returns a new validated plan; failure leaves the current plan untouched.
    ModelPlan resolve(const std::vector<Tensor> &) const;
    void validateOutputs(const std::vector<Tensor> &) const;

  private:
    void bind();
    ModelConfig model_;
    TaskConfig task_;
    ModelInfo info_;
    std::array<int, 6> heads_{};
    bool ready_ = false;
};
} // namespace perception
