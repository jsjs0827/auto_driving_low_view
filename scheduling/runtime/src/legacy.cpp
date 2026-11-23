#include "perception/inference.hpp"
#include <stdexcept>
namespace perception {
Profile toProfile(const Config &c) {
    Profile p;
    auto &m = p.model;
    m.source = {c.model, c.model_name};
    m.output_layout = c.output_layout;
    m.backend = {c.device, c.cpu_threads, c.device_id, c.bpu_timeout_ms};
    m.preprocess.width = c.width;
    m.preprocess.height = c.height;
    m.preprocess.letterbox = c.letterbox;
    m.preprocess.pad = c.pad;
    m.preprocess.input_bgr = c.input_bgr;
    m.preprocess.mean = c.mean;
    m.preprocess.stddev = c.stddev;
    if (c.task == Task::Detection) {
        DetectionConfig d;
        d.classes = c.classes;
        d.reg_bins = c.reg_bins;
        d.class_names = c.class_names;
        d.heads = c.heads;
        d.score_threshold = c.score_threshold;
        d.iou_threshold = c.iou_threshold;
        d.max_detections = c.max_detections;
        d.candidate_budget = c.candidate_budget;
        p.task = std::move(d);
    } else if (c.task == Task::Segmentation) {
        SegmentationConfig s;
        s.classes = c.classes;
        s.class_names = c.class_names;
        s.seg_output = c.seg_output;
        s.align_corners = c.align_corners;
        s.bilinear_logits = c.bilinear_logits;
        p.task = std::move(s);
    } else
        throw std::invalid_argument("unknown legacy task");
    return p;
}
Config fromProfile(const Profile &p) {
    Config c;
    const auto &m = p.model;
    const auto &pre = m.preprocess;
    c.model = m.source.path;
    c.model_name = m.source.model_name;
    c.device = m.backend.device;
    c.cpu_threads = m.backend.cpu_threads;
    c.device_id = m.backend.device_id;
    c.bpu_timeout_ms = m.backend.bpu_timeout_ms;
    c.output_layout = m.output_layout;
    c.width = pre.width;
    c.height = pre.height;
    c.letterbox = pre.letterbox;
    c.pad = pre.pad;
    c.input_bgr = pre.input_bgr;
    c.mean = pre.mean;
    c.stddev = pre.stddev;
    if (const auto *d = std::get_if<DetectionConfig>(&p.task)) {
        c.classes = d->classes;
        c.class_names = d->class_names;
        c.reg_bins = d->reg_bins;
        c.heads = d->heads;
        c.score_threshold = d->score_threshold;
        c.iou_threshold = d->iou_threshold;
        c.max_detections = d->max_detections;
        c.candidate_budget = d->candidate_budget;
    } else {
        const auto &s = std::get<SegmentationConfig>(p.task);
        c.task = Task::Segmentation;
        c.classes = s.classes;
        c.class_names = s.class_names;
        c.seg_output = s.seg_output;
        c.align_corners = s.align_corners;
        c.bilinear_logits = s.bilinear_logits;
    }
    return c;
}
void Config::validate() const {
    toProfile(*this).validate();
}
Config loadConfig(const std::string &path) {
    return fromProfile(loadProfile(path));
}
std::unique_ptr<Backend> makeBackend(const Config &c) {
    c.validate();
    return makeBackend(toProfile(c).model);
}
std::vector<float> normalize(const std::vector<std::uint8_t> &rgb, const Config &c) {
    return normalize(rgb, toProfile(c).model.preprocess);
}
namespace {
ModelPlan planFor(const std::vector<Tensor> &tensors, const Config &c, const Geometry &g,
                  Task task) {
    auto cfg = c;
    cfg.task = task;
    auto p = toProfile(cfg);
    ModelInfo info;
    info.width = g.input_w;
    info.height = g.input_h;
    for (const auto &t : tensors) {
        t.validate();
        info.outputs.push_back(t.shape());
    }
    return ModelPlan(p.model, p.task, std::move(info));
}
} // namespace
std::vector<Detection> decodeDetection(const std::vector<Tensor> &t, const Config &c,
                                       const Geometry &g, std::size_t &rejected) {
    return YoloDflDecoder::decode(t, planFor(t, c, g, Task::Detection), g, rejected);
}
std::vector<std::uint8_t> decodeSegmentation(const std::vector<Tensor> &t, const Config &c,
                                             const Geometry &g) {
    return SemanticDecoder::decode(t, planFor(t, c, g, Task::Segmentation), g);
}
struct Engine::Impl {
    std::unique_ptr<Detector> detector;
    std::unique_ptr<Segmenter> segmenter;
};
Engine::Engine(Config c) : Engine(c, makeBackend(c)) {}
Engine::Engine(Config c, std::unique_ptr<Backend> b)
    : config_(std::move(c)), impl_(std::make_unique<Impl>()) {
    auto p = toProfile(config_);
    p.validate();
    if (const auto *d = std::get_if<DetectionConfig>(&p.task)) {
        impl_->detector = std::make_unique<Detector>(p.model, *d, std::move(b));
        const auto info = impl_->detector->plan().info();
        config_.width = info.width;
        config_.height = info.height;
    } else {
        impl_->segmenter = std::make_unique<Segmenter>(
            p.model, std::get<SegmentationConfig>(p.task), std::move(b));
        const auto info = impl_->segmenter->plan().info();
        config_.width = info.width;
        config_.height = info.height;
    }
}
Engine::~Engine() = default;
Result Engine::run(const ImageView &image) {
    Result out;
    Timing timing;
    if (impl_->detector) {
        auto r = impl_->detector->run(image);
        out.detections = std::move(r.detections);
        out.rejected_anchors = r.rejected_anchors;
        timing = r.timing;
    } else {
        auto r = impl_->segmenter->run(image);
        out.mask = std::move(r.mask);
        timing = r.timing;
    }
    out.width = image.width;
    out.height = image.height;
    out.preprocess_ms = timing.preprocess_ms;
    out.inference_ms = timing.inference_ms;
    out.postprocess_ms = timing.postprocess_ms;
    return out;
}
} // namespace perception
