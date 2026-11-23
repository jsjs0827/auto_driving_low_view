#pragma once
#include "perception/types.hpp"
#include <memory>
#include <variant>

namespace perception {
class Logger;
struct BackendConfig {
    Device device = Device::CPU;
    int cpu_threads = 1, device_id = 0, bpu_timeout_ms = 1000;
    void validate() const;
};
struct ModelSource {
    std::string path, model_name;
};
struct PreprocessConfig {
    int width = 0, height = 0; // Both zero: resolve from static model input.
    bool letterbox = true, input_bgr = false;
    std::uint8_t pad = 129;
    std::array<float, 3> mean{0, 0, 0}, stddev{255, 255, 255};
    void validate() const;
};
struct ModelConfig {
    ModelSource source;
    BackendConfig backend;
    PreprocessConfig preprocess;
    Layout output_layout = Layout::NHWC; // ONNX output contract, not guessed from dimensions.
    std::shared_ptr<Logger> logger;      // Optional; no default process-global logger.
    void validate() const;
};
struct DetectionConfig {
    int classes = 0, reg_bins = 16;
    std::vector<std::string> class_names;
    std::array<int, 6> heads{-1, -1, -1, -1, -1, -1};
    float score_threshold = .25f, iou_threshold = .70f;
    int max_detections = 300, candidate_budget = 4096;
    void validate() const;
};
struct SegmentationConfig {
    int classes = 0, seg_output = -1;
    std::vector<std::string> class_names;
    bool align_corners = true, bilinear_logits = true;
    void validate() const;
};
using TaskConfig = std::variant<DetectionConfig, SegmentationConfig>;
struct Profile {
    ModelConfig model;
    TaskConfig task = DetectionConfig{};
    void validate() const;
};
Profile loadProfile(const std::string &path); // Rejects fields belonging to the other task.
} // namespace perception
