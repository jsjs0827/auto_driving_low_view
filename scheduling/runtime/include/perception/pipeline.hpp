#pragma once
#include "perception/backend.hpp"
#include "perception/model_plan.hpp"
#include "perception/preprocess.hpp"
namespace perception {
class Detector {
  public:
    Detector(ModelConfig, DetectionConfig);
    Detector(ModelConfig, DetectionConfig, std::unique_ptr<Backend>);
    ~Detector();
    Detector(const Detector &) = delete;
    Detector &operator=(const Detector &) = delete;
    DetectionResult run(const ImageView &);
    ModelPlan plan() const; // Snapshot, safe to retain across future calls.
    BackendState state() const;
    void close();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
class Segmenter {
  public:
    Segmenter(ModelConfig, SegmentationConfig);
    Segmenter(ModelConfig, SegmentationConfig, std::unique_ptr<Backend>);
    ~Segmenter();
    Segmenter(const Segmenter &) = delete;
    Segmenter &operator=(const Segmenter &) = delete;
    SegmentationResult run(const ImageView &);
    ModelPlan plan() const;
    BackendState state() const;
    void close();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace perception
