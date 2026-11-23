#include "perception/pipeline.hpp"
#include "detail/decoder_workspace.hpp"
#include "detail/diagnostic_boundary.hpp"
#include "detail/native_detection.hpp"
#include "detail/preprocess_workspace.hpp"
#include "detail/reusable_backend.hpp"
#include "perception/decoders.hpp"
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace perception {
PreprocessPlan::PreprocessPlan(PreprocessConfig config, bool native)
    : config_(std::move(config)), native_nv12_(native) {
    config_.validate();
    if (config_.width == 0)
        throw std::invalid_argument("preprocess plan needs resolved input size");
}
PreparedInput PreprocessPlan::execute(const ImageView &image) const {
    if (native_nv12_ && (image.format == PixelFormat::NV12 || image.format == PixelFormat::I420)) {
        auto p = prepareNv12(image, config_.width, config_.height, config_.letterbox, config_.pad);
        return {p.geometry, std::move(p.nv12), true};
    }
    auto p = prepare(image, config_.width, config_.height, config_.letterbox, config_.pad);
    return {p.geometry, std::move(p.rgb), false};
}
namespace {
using Clock = std::chrono::steady_clock;
std::unique_ptr<Backend> validatedBackend(const ModelConfig &model, const TaskConfig &task) {
    return detail::initialization(model.logger, ErrorCode::AiInitFailed, "ai.init", [&] {
        model.validate();
        std::visit([](const auto &config) { config.validate(); }, task);
        detail::log(model.logger, LogLevel::Info, "model.load.begin", "ai.init", model.source.path);
        auto backend = makeBackend(model);
        detail::log(model.logger, LogLevel::Info, "model.load.end", "ai.init", model.source.path);
        return backend;
    });
}
template <class Impl, class Config>
std::unique_ptr<Impl> initialize(ModelConfig model, Config cfg, std::unique_ptr<Backend> backend,
                                 const char *name) {
    return detail::initialization(model.logger, ErrorCode::AiInitFailed, name, [&] {
        auto result = std::make_unique<Impl>(model, std::move(cfg), std::move(backend));
        detail::log(model.logger, LogLevel::Info, "ai.ready", name);
        return result;
    });
}
double ms(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}
detail::OutputSelection requiredOutputs(const ModelPlan &plan) {
    detail::OutputSelection selection;
    if (!plan.ready()) {
        selection.all = true;
        return selection;
    }
    if (plan.task() == Task::Detection) {
        selection.indices = plan.heads();
        selection.count = selection.indices.size();
    } else {
        selection.indices[0] = plan.segmentation().seg_output;
        selection.count = 1;
    }
    return selection;
}
class ModelSession {
    std::unique_ptr<Backend> backend_;
    std::shared_ptr<Logger> logger_;
    ModelPlan plan_;
    detail::ReusableBackend *reusable_backend_ = nullptr;
    detail::NativeDetectionBackend *native_detection_backend_ = nullptr;
    detail::PreprocessWorkspace preprocess_workspace_;
    detail::DecoderWorkspace decoder_workspace_;
    mutable std::mutex mutex_;
    bool closed_ = false;
    static ModelInfo info(Backend *b) {
        if (!b)
            throw std::invalid_argument("null backend");
        return b->modelInfo();
    }

  public:
    ModelSession(ModelConfig m, TaskConfig task, std::unique_ptr<Backend> backend)
        : backend_(std::move(backend)), logger_(m.logger),
          plan_(std::move(m), std::move(task), info(backend_.get())),
          reusable_backend_(dynamic_cast<detail::ReusableBackend *>(backend_.get())),
          native_detection_backend_(dynamic_cast<detail::NativeDetectionBackend *>(backend_.get())) {}
    ~ModelSession() { detail::log(logger_, LogLevel::Info, "ai.destroy", "ai.lifecycle"); }
    // Keep the full frame serialized: future reusable workspaces remain protected.
    template <class Decode> auto execute(const ImageView &image, Decode decode) try {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_)
            throw PerceptionException(ErrorCode::PerceptionStateInvalid, "model session is closed",
                                      "ai.run");
        if (backend_->state() == BackendState::Quarantined)
            throw PerceptionException(ErrorCode::PerceptionStateInvalid,
                                      "model session quarantined; restart process", "ai.run");
        validateCameraImage(image);
        const auto begin = Clock::now();
        auto input = detail::boundary(ErrorCode::InferenceFailed, "ai.preprocess", [&] {
            return preprocess_workspace_.execute(image, plan_.model().preprocess,
                                                 backend_->acceptsNv12(),
                                                 reusable_backend_ != nullptr);
        });
        const auto pre = Clock::now();
        std::vector<Tensor> owned_tensors;
        const std::vector<Tensor> *tensor_view = nullptr;
        const std::vector<detail::NativeTensorView> *native_view = nullptr;
        const auto selection = requiredOutputs(plan_);
        if (native_detection_backend_ && plan_.ready() && plan_.task() == Task::Detection &&
            native_detection_backend_->supportsNativeDetection(selection)) {
            native_view = &detail::boundary(ErrorCode::InferenceFailed, "ai.inference", [&]()
                                            -> const std::vector<detail::NativeTensorView> & {
                return native_detection_backend_->runNativeDetection(
                    input.data, input.bytes, input.nv12, selection);
            });
        } else if (reusable_backend_) {
            tensor_view = &detail::boundary(ErrorCode::InferenceFailed, "ai.inference", [&]()
                                                -> const std::vector<Tensor> & {
                return reusable_backend_->runReusable(input.data, input.bytes, input.nv12,
                                                       selection);
            });
            // In-tree reusable backends bind and validate immutable output descriptors at
            // initialization, and finite-check each selected conversion exactly once.
            if (!plan_.ready())
                plan_ = detail::boundary(ErrorCode::InferenceResultInvalid, "ai.outputs",
                                         [&] { return plan_.resolve(*tensor_view); });
        } else {
            owned_tensors = detail::boundary(ErrorCode::InferenceFailed, "ai.inference", [&] {
                const auto &bytes = preprocess_workspace_.ownedBytes();
                return input.nv12 ? backend_->runNv12(bytes) : backend_->run(bytes);
            });
            tensor_view = &owned_tensors;
            detail::boundary(ErrorCode::InferenceResultInvalid, "ai.outputs", [&] {
                if (!plan_.ready())
                    plan_ = plan_.resolve(*tensor_view);
                else
                    plan_.validateOutputs(*tensor_view);
                // Legacy/injected backends do not implement the validated-output contract.
                // Scan once here; validated decoders below deliberately do not scan again.
                for (const auto &tensor : *tensor_view)
                    for (float value : tensor.data)
                        if (!std::isfinite(value))
                            throw std::runtime_error("nonfinite model output");
            });
        }
        const auto inferred = Clock::now();
        auto result = detail::boundary(ErrorCode::InferenceResultInvalid, "ai.decode",
                                       [&] {
                                            return decode(tensor_view, native_view, plan_, input.geometry,
                                                         decoder_workspace_);
                                       });
        const auto end = Clock::now();
        result.width = image.width;
        result.height = image.height;
        result.timing = {ms(begin, pre), ms(pre, inferred), ms(inferred, end)};
        return result;
    } catch (const std::exception &e) {
        auto error = errorFromException(ErrorCode::InferenceFailed, e, "ai.run");
        if (logger_)
            logger_->error(error);
        throw PerceptionException(std::move(error));
    }
    ModelPlan plan() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return plan_;
    }
    BackendState state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_ ? BackendState::Closed : backend_->state();
    }
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!closed_) {
            detail::boundary(ErrorCode::PerceptionStateInvalid, "ai.close",
                             [&] { backend_->close(); });
            closed_ = true;
            detail::log(logger_, LogLevel::Info, "ai.closed", "ai.lifecycle");
        }
    }
};
} // namespace
struct Detector::Impl {
    ModelSession session;
    Impl(ModelConfig m, DetectionConfig d, std::unique_ptr<Backend> b)
        : session(std::move(m), std::move(d), std::move(b)) {};
};
struct Segmenter::Impl {
    ModelSession session;
    Impl(ModelConfig m, SegmentationConfig s, std::unique_ptr<Backend> b)
        : session(std::move(m), std::move(s), std::move(b)) {};
};
Detector::Detector(ModelConfig m, DetectionConfig d) : Detector(m, d, validatedBackend(m, d)) {}
Detector::Detector(ModelConfig m, DetectionConfig d, std::unique_ptr<Backend> b)
    : impl_(initialize<Impl>(m, std::move(d), std::move(b), "detector")) {}
Detector::~Detector() = default;
DetectionResult Detector::run(const ImageView &image) {
    return impl_->session.execute(image, [](const auto *tensors, const auto *native,
                                            const auto &plan, const auto &g, auto &workspace) {
        DetectionResult out;
        if (native)
            detail::decodeDetectionNativeValidated(*native, plan, g, workspace.detection,
                                                   out.detections, out.rejected_anchors);
        else
            detail::decodeDetectionValidated(*tensors, plan, g, workspace.detection,
                                             out.detections, out.rejected_anchors);
        return out;
    });
}
ModelPlan Detector::plan() const {
    return impl_->session.plan();
}
BackendState Detector::state() const {
    return impl_->session.state();
}
void Detector::close() {
    impl_->session.close();
}
Segmenter::Segmenter(ModelConfig m, SegmentationConfig s)
    : Segmenter(m, s, validatedBackend(m, s)) {}
Segmenter::Segmenter(ModelConfig m, SegmentationConfig s, std::unique_ptr<Backend> b)
    : impl_(initialize<Impl>(m, std::move(s), std::move(b), "segmenter")) {}
Segmenter::~Segmenter() = default;
SegmentationResult Segmenter::run(const ImageView &image) {
    return impl_->session.execute(image, [](const auto *tensors, const auto *,
                                            const auto &plan, const auto &g, auto &workspace) {
        SegmentationResult out;
        detail::decodeSegmentationValidated(*tensors, plan, g, workspace.segmentation, out.mask);
        return out;
    });
}
ModelPlan Segmenter::plan() const {
    return impl_->session.plan();
}
BackendState Segmenter::state() const {
    return impl_->session.state();
}
void Segmenter::close() {
    impl_->session.close();
}
} // namespace perception
