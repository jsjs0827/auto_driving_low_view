#include "perception/model_plan.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>

namespace perception {
namespace {
void require(bool valid, const char *message) {
    if (!valid)
        throw std::invalid_argument(message);
}
} // namespace
void TensorShape::validate(bool dynamic) const {
    auto valid = [dynamic](int n, int limit) {
        return (dynamic && n == -1) || (n > 0 && n <= limit);
    };
    require(valid(channels, 1024) && valid(height, 4096) && valid(width, 4096),
            "invalid tensor shape metadata");
    if (resolved())
        require(std::size_t(channels) * height * width <= 64U * 1024U * 1024U,
                "tensor exceeds element limit");
}
ModelPlan::ModelPlan(ModelConfig model, TaskConfig task, ModelInfo info)
    : model_(std::move(model)), task_(std::move(task)), info_(std::move(info)) {
    model_.validate();
    std::visit([](const auto &c) { c.validate(); }, task_);
    require(info_.width > 0 && info_.height > 0 && info_.width <= 4096 && info_.height <= 4096,
            "invalid model input dimensions");
    const auto &p = model_.preprocess;
    require(p.width == 0 || (p.width == info_.width && p.height == info_.height),
            "profile/model dimensions differ");
    model_.preprocess.width = info_.width;
    model_.preprocess.height = info_.height;
    if (info_.native_nv12)
        require(info_.width % 2 == 0 && info_.height % 2 == 0,
                "NV12 model requires even dimensions");
    if (this->task() == Task::Detection)
        require(info_.width % 32 == 0 && info_.height % 32 == 0,
                "detection size must be divisible by 32");
    require(info_.outputs.size() <= 64, "too many model outputs");
    require(info_.output_names.empty() || info_.output_names.size() == info_.outputs.size(),
            "output name count mismatch");
    if (!info_.outputs.empty()) {
        if (this->task() == Task::Detection)
            require(info_.outputs.size() == 6, "detector requires six output heads");
        else {
            const auto &cfg = segmentation();
            require(std::size_t(cfg.seg_output) < info_.outputs.size(), "seg_output out of range");
            const int channels = info_.outputs[cfg.seg_output].channels;
            require(channels == -1 || channels == cfg.classes, "segmentation class count mismatch");
        }
        for (const auto &shape : info_.outputs)
            shape.validate(true);
        if (std::all_of(info_.outputs.begin(), info_.outputs.end(),
                        [](const auto &s) { return s.resolved(); }))
            bind();
    }
}
Task ModelPlan::task() const noexcept {
    return std::holds_alternative<DetectionConfig>(task_) ? Task::Detection : Task::Segmentation;
}
const DetectionConfig &ModelPlan::detection() const {
    return std::get<DetectionConfig>(task_);
}
const SegmentationConfig &ModelPlan::segmentation() const {
    return std::get<SegmentationConfig>(task_);
}
const std::array<int, 6> &ModelPlan::heads() const {
    require(ready_ && task() == Task::Detection, "detection plan is not bound");
    return heads_;
}
void ModelPlan::bind() {
    if (task() == Task::Detection) {
        const auto &c = detection();
        heads_ = c.heads;
        require(info_.outputs.size() == 6, "detector requires six raw DFL heads");
        for (int i = 0; i < 6; ++i) {
            const int stride = 8 << (i / 2);
            const TensorShape expected{i % 2 ? 4 * c.reg_bins : c.classes, info_.height / stride,
                                       info_.width / stride};
            if (c.heads[0] == -1) {
                int found = -1;
                for (int j = 0; j < 6; ++j)
                    if (info_.outputs[j] == expected) {
                        require(found == -1, "ambiguous output heads; set explicit indices");
                        found = j;
                    }
                require(found >= 0, "missing detection head");
                heads_[i] = found;
            } else
                require(heads_[i] < 6 && info_.outputs[heads_[i]] == expected,
                        "explicit head shape mismatch");
        }
        require(std::set<int>(heads_.begin(), heads_.end()).size() == 6,
                "overlapping output binding");
    } else {
        const auto &c = segmentation();
        require(std::size_t(c.seg_output) < info_.outputs.size(), "seg_output out of range");
        require(info_.outputs[c.seg_output].channels == c.classes,
                "segmentation class count mismatch");
    }
    ready_ = true;
}
ModelPlan ModelPlan::resolve(const std::vector<Tensor> &tensors) const {
    if (ready_) {
        validateOutputs(tensors);
        return *this;
    }
    require(!tensors.empty() && tensors.size() <= 64, "invalid output count");
    require(info_.outputs.empty() || tensors.size() == info_.outputs.size(),
            "output count differs from model metadata");
    ModelPlan next = *this;
    next.info_.outputs.clear();
    for (std::size_t i = 0; i < tensors.size(); ++i) {
        const auto &t = tensors[i];
        t.validate();
        const auto actual = t.shape();
        if (!info_.outputs.empty()) {
            const auto declared = info_.outputs[i];
            require((declared.channels == -1 || declared.channels == actual.channels) &&
                        (declared.height == -1 || declared.height == actual.height) &&
                        (declared.width == -1 || declared.width == actual.width),
                    "runtime output violates model metadata");
        }
        next.info_.outputs.push_back(actual);
    }
    next.bind();
    return next;
}
void ModelPlan::validateOutputs(const std::vector<Tensor> &tensors) const {
    require(ready_, "model output plan is not resolved");
    require(tensors.size() == info_.outputs.size(), "output count changed after plan binding");
    for (std::size_t i = 0; i < tensors.size(); ++i) {
        tensors[i].validate();
        require(tensors[i].shape() == info_.outputs[i],
                "output shape changed; recreate model session");
    }
}
} // namespace perception
