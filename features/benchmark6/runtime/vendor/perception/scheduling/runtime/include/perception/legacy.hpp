#pragma once
#include "perception/pipeline.hpp"
namespace perception {
struct Config {
    Task task = Task::Detection;
    Device device = Device::CPU;
    std::string model, model_name; // BPU: required when package has multiple models
    int width = 0, height = 0;     // 0: use static metadata; dynamic ONNX requires explicit size
    int classes = 0, reg_bins = 16;
    std::vector<std::string> class_names; // empty: numeric IDs only; never guessed names
    bool letterbox = true;
    std::uint8_t pad = 129;
    bool input_bgr = false; // ONNX channel order; BPU channel order is compiled in model
    std::array<float, 3> mean{0, 0, 0}, stddev{255, 255, 255};
    Layout output_layout = Layout::NHWC;              // ONNX has no semantic layout metadata
    std::array<int, 6> heads{-1, -1, -1, -1, -1, -1}; // cls8,box8,cls16,box16,cls32,box32
    int seg_output = -1;
    bool align_corners = true, bilinear_logits = true;
    float score_threshold = .25f, iou_threshold = .70f;
    int max_detections = 300, candidate_budget = 4096;
    int cpu_threads = 1, device_id = 0, bpu_timeout_ms = 1000;
    void validate() const;
};
struct Result {
    std::vector<Detection> detections;
    std::vector<std::uint8_t> mask; // source resolution, raw IDs 0..classes-1
    int width = 0, height = 0;
    std::size_t rejected_anchors = 0;
    double preprocess_ms = 0, inference_ms = 0, postprocess_ms = 0;
};
Profile toProfile(const Config &);
Config fromProfile(const Profile &);
Config loadConfig(const std::string &);
std::unique_ptr<Backend> makeBackend(const Config &);
std::vector<float> normalize(const std::vector<std::uint8_t> &, const Config &);
std::vector<Detection> decodeDetection(const std::vector<Tensor> &, const Config &,
                                       const Geometry &, std::size_t &);
std::vector<std::uint8_t> decodeSegmentation(const std::vector<Tensor> &, const Config &,
                                             const Geometry &);
class Engine {
  public:
    explicit Engine(Config);
    Engine(Config, std::unique_ptr<Backend>);
    ~Engine();
    Result run(const ImageView &);
    const Config &config() const noexcept { return config_; }

  private:
    struct Impl;
    Config config_;
    std::unique_ptr<Impl> impl_;
};
} // namespace perception
