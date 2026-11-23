#include "perception/staged_inference.hpp"
#include "detail/decoder_workspace.hpp"
#include "detail/diagnostic_boundary.hpp"
#include "detail/native_detection.hpp"
#include "detail/preprocess_workspace.hpp"
#include "detail/reusable_backend.hpp"
#include "perception/decoders.hpp"
#include "perception/error.hpp"
#include "perception/logging.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <stdexcept>

namespace perception {
namespace {
using Clock = std::chrono::steady_clock;

double ms(Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::unique_ptr<Backend> createBackend(const ModelConfig &model, const TaskConfig &task) {
    return detail::initialization(model.logger, ErrorCode::AiInitFailed, "staged_ai.init", [&] {
        model.validate();
        std::visit([](const auto &config) { config.validate(); }, task);
        return makeBackend(model);
    });
}

ModelInfo modelInfo(Backend *backend) {
    if (!backend)
        throw std::invalid_argument("null staged inference backend");
    return backend->modelInfo();
}

detail::OutputSelection selectedOutputs(const ModelPlan &plan) {
    detail::OutputSelection selection;
    if (!plan.ready()) {
        selection.all = true;
    } else if (plan.task() == Task::Detection) {
        selection.indices = plan.heads();
        selection.count = selection.indices.size();
    } else {
        selection.indices[0] = plan.segmentation().seg_output;
        selection.count = 1;
    }
    return selection;
}
} // namespace

struct InferenceFrame::Impl {
    enum class Phase { Created, Preprocessed, Inferred, Completed, Failed };
    ImageView image;
    std::uint64_t token = 0;
    Phase phase = Phase::Created;
    detail::PreparedInputView input;
    std::vector<Tensor> owned_tensors;
    const std::vector<Tensor> *tensors = nullptr;
    const std::vector<detail::NativeTensorView> *native_views = nullptr;
    Clock::time_point begin, preprocessed, inferred;
    std::weak_ptr<void> owner;
    std::function<void(std::uint64_t)> cancel;
};

InferenceFrame::InferenceFrame(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
InferenceFrame::~InferenceFrame() {
    if (impl_ && impl_->phase != Impl::Phase::Completed && impl_->cancel)
        impl_->cancel(impl_->token);
}

struct StagedInference::Impl : std::enable_shared_from_this<Impl> {
    std::unique_ptr<Backend> backend;
    std::shared_ptr<Logger> logger;
    const Task configured_task;
    ModelPlan model_plan;
    detail::ReusableBackend *reusable = nullptr;
    detail::NativeDetectionBackend *native_detection = nullptr;
    detail::PreprocessWorkspace preprocess_workspace;
    detail::DecoderWorkspace decoder_workspace;
    mutable std::mutex mutex;
    bool closed = false;
    std::uint64_t next_token = 1;
    std::uint64_t active_token = 0;

    Impl(ModelConfig model, TaskConfig task_config, std::unique_ptr<Backend> selected)
        : backend(std::move(selected)), logger(model.logger),
          configured_task(std::holds_alternative<DetectionConfig>(task_config)
                              ? Task::Detection
                              : Task::Segmentation),
          model_plan(std::move(model), std::move(task_config), modelInfo(backend.get())),
          reusable(dynamic_cast<detail::ReusableBackend *>(backend.get())),
          native_detection(dynamic_cast<detail::NativeDetectionBackend *>(backend.get())) {}

    bool owns(const InferenceFrame &frame) const {
        return frame.impl_ && frame.impl_->owner.lock().get() == this;
    }

    void cancel(std::uint64_t token) {
        std::lock_guard<std::mutex> lock(mutex);
        if (active_token == token)
            active_token = 0;
    }

    void ensureFrame(const InferenceFrame &frame, InferenceFrame::Impl::Phase expected) {
        if (!frame.impl_)
            throw std::invalid_argument("empty inference frame");
        if (!owns(frame))
            throw std::runtime_error("inference frame belongs to another model session");
        if (frame.impl_->phase != expected)
            throw std::runtime_error("inference stage order violation");
        if (closed)
            throw std::runtime_error("staged inference is closed");
        if (active_token != frame.impl_->token)
            throw std::runtime_error("inference frame is not active");
    }

    void fail(InferenceFrame &frame) noexcept {
        if (frame.impl_) {
            frame.impl_->phase = InferenceFrame::Impl::Phase::Failed;
            if (active_token == frame.impl_->token)
                active_token = 0;
            std::vector<Tensor>().swap(frame.impl_->owned_tensors);
            frame.impl_->tensors = nullptr;
            frame.impl_->native_views = nullptr;
            frame.impl_->input = {};
            frame.impl_->image = {};
            frame.impl_->cancel = {};
            frame.impl_->owner.reset();
        }
    }

    void complete(InferenceFrame &frame) noexcept {
        frame.impl_->phase = InferenceFrame::Impl::Phase::Completed;
        active_token = 0;
        // The task graph may retain its internal invocation artifact until all
        // downstream tasks finish. Release frame-scale outputs and every
        // borrowed pointer immediately after decoding instead of tying their
        // lifetime to that artifact.
        std::vector<Tensor>().swap(frame.impl_->owned_tensors);
        frame.impl_->tensors = nullptr;
        frame.impl_->native_views = nullptr;
        frame.impl_->input = {};
        frame.impl_->image = {};
        frame.impl_->cancel = {};
        frame.impl_->owner.reset();
    }
};

StagedInference::StagedInference(ModelConfig model, DetectionConfig config)
    : StagedInference(model, config, createBackend(model, config)) {}
StagedInference::StagedInference(ModelConfig model, SegmentationConfig config)
    : StagedInference(model, config, createBackend(model, config)) {}
StagedInference::StagedInference(ModelConfig model, DetectionConfig config,
                                 std::unique_ptr<Backend> backend)
    : impl_(detail::initialization(model.logger, ErrorCode::AiInitFailed, "staged_ai.init", [&] {
          auto result = std::make_shared<Impl>(std::move(model), std::move(config),
                                               std::move(backend));
          detail::log(result->logger, LogLevel::Info, "staged_ai.ready", "staged_ai");
          return result;
      })) {}
StagedInference::StagedInference(ModelConfig model, SegmentationConfig config,
                                 std::unique_ptr<Backend> backend)
    : impl_(detail::initialization(model.logger, ErrorCode::AiInitFailed, "staged_ai.init", [&] {
          auto result = std::make_shared<Impl>(std::move(model), std::move(config),
                                               std::move(backend));
          detail::log(result->logger, LogLevel::Info, "staged_ai.ready", "staged_ai");
          return result;
      })) {}
StagedInference::~StagedInference() = default;

std::shared_ptr<InferenceFrame> StagedInference::begin(const ImageView &image) {
    validateCameraImage(image);
    auto frame_impl = std::make_unique<InferenceFrame::Impl>();
    frame_impl->image = image;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->closed)
            throw std::runtime_error("staged inference is closed");
        frame_impl->token = impl_->next_token++;
        frame_impl->owner = impl_;
    }
    std::weak_ptr<Impl> weak = impl_;
    frame_impl->cancel = [weak](std::uint64_t token) {
        if (const auto owner = weak.lock())
            owner->cancel(token);
    };
    return std::shared_ptr<InferenceFrame>(new InferenceFrame(std::move(frame_impl)));
}

void StagedInference::preprocess(InferenceFrame &frame) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->closed)
        throw std::runtime_error("staged inference is closed");
    if (!frame.impl_)
        throw std::invalid_argument("empty inference frame");
    if (!impl_->owns(frame))
        throw std::runtime_error("inference frame belongs to another model session");
    if (frame.impl_->phase != InferenceFrame::Impl::Phase::Created)
        throw std::runtime_error("inference preprocess stage order violation");
    if (impl_->active_token != 0)
        throw std::runtime_error("another frame is active in this model session");
    impl_->active_token = frame.impl_->token;
    try {
        frame.impl_->begin = Clock::now();
        frame.impl_->input = detail::boundary(ErrorCode::InferenceFailed, "ai.preprocess", [&] {
            return impl_->preprocess_workspace.execute(frame.impl_->image,
                                                       impl_->model_plan.model().preprocess,
                                                       impl_->backend->acceptsNv12(),
                                                       impl_->reusable != nullptr);
        });
        frame.impl_->preprocessed = Clock::now();
        frame.impl_->phase = InferenceFrame::Impl::Phase::Preprocessed;
    } catch (...) {
        impl_->fail(frame);
        throw;
    }
}

void StagedInference::infer(InferenceFrame &frame) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureFrame(frame, InferenceFrame::Impl::Phase::Preprocessed);
    try {
        if (impl_->backend->state() == BackendState::Quarantined)
            throw std::runtime_error("model session quarantined; restart process");
        const auto selection = selectedOutputs(impl_->model_plan);
        if (impl_->native_detection && impl_->model_plan.ready() &&
            impl_->model_plan.task() == Task::Detection &&
            impl_->native_detection->supportsNativeDetection(selection)) {
            frame.impl_->native_views =
                &detail::boundary(ErrorCode::InferenceFailed, "ai.inference", [&]()
                                      -> const std::vector<detail::NativeTensorView> & {
                    return impl_->native_detection->runNativeDetection(
                        frame.impl_->input.data, frame.impl_->input.bytes,
                        frame.impl_->input.nv12, selection);
                });
        } else if (impl_->reusable) {
            frame.impl_->tensors =
                &detail::boundary(ErrorCode::InferenceFailed, "ai.inference", [&]()
                                      -> const std::vector<Tensor> & {
                    return impl_->reusable->runReusable(
                        frame.impl_->input.data, frame.impl_->input.bytes,
                        frame.impl_->input.nv12, selection);
                });
            if (!impl_->model_plan.ready())
                impl_->model_plan = detail::boundary(
                    ErrorCode::InferenceResultInvalid, "ai.outputs",
                    [&] { return impl_->model_plan.resolve(*frame.impl_->tensors); });
        } else {
            const auto &bytes = impl_->preprocess_workspace.ownedBytes();
            frame.impl_->owned_tensors = detail::boundary(
                ErrorCode::InferenceFailed, "ai.inference", [&] {
                    return frame.impl_->input.nv12 ? impl_->backend->runNv12(bytes)
                                                   : impl_->backend->run(bytes);
                });
            frame.impl_->tensors = &frame.impl_->owned_tensors;
            detail::boundary(ErrorCode::InferenceResultInvalid, "ai.outputs", [&] {
                if (!impl_->model_plan.ready())
                    impl_->model_plan = impl_->model_plan.resolve(*frame.impl_->tensors);
                else
                    impl_->model_plan.validateOutputs(*frame.impl_->tensors);
                for (const auto &tensor : *frame.impl_->tensors)
                    for (float value : tensor.data)
                        if (!std::isfinite(value))
                            throw std::runtime_error("nonfinite model output");
            });
        }
        frame.impl_->inferred = Clock::now();
        frame.impl_->phase = InferenceFrame::Impl::Phase::Inferred;
    } catch (...) {
        impl_->fail(frame);
        throw;
    }
}

DetectionResult StagedInference::postprocessDetection(InferenceFrame &frame) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureFrame(frame, InferenceFrame::Impl::Phase::Inferred);
    try {
        if (impl_->model_plan.task() != Task::Detection)
            throw std::runtime_error("detection postprocess requested for segmentation model");
        DetectionResult result;
        if (frame.impl_->native_views)
            detail::decodeDetectionNativeValidated(*frame.impl_->native_views,
                                                   impl_->model_plan,
                                                   frame.impl_->input.geometry,
                                                   impl_->decoder_workspace.detection,
                                                   result.detections, result.rejected_anchors);
        else
            detail::decodeDetectionValidated(*frame.impl_->tensors, impl_->model_plan,
                                             frame.impl_->input.geometry,
                                             impl_->decoder_workspace.detection,
                                             result.detections, result.rejected_anchors);
        const auto end = Clock::now();
        result.width = frame.impl_->image.width;
        result.height = frame.impl_->image.height;
        result.timing = {ms(frame.impl_->begin, frame.impl_->preprocessed),
                         ms(frame.impl_->preprocessed, frame.impl_->inferred),
                         ms(frame.impl_->inferred, end)};
        impl_->complete(frame);
        return result;
    } catch (...) {
        impl_->fail(frame);
        throw;
    }
}

SegmentationResult StagedInference::postprocessSegmentation(InferenceFrame &frame) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureFrame(frame, InferenceFrame::Impl::Phase::Inferred);
    try {
        if (impl_->model_plan.task() != Task::Segmentation)
            throw std::runtime_error("segmentation postprocess requested for detection model");
        SegmentationResult result;
        detail::decodeSegmentationValidated(*frame.impl_->tensors, impl_->model_plan,
                                            frame.impl_->input.geometry,
                                            impl_->decoder_workspace.segmentation,
                                            result.mask);
        const auto end = Clock::now();
        result.width = frame.impl_->image.width;
        result.height = frame.impl_->image.height;
        result.timing = {ms(frame.impl_->begin, frame.impl_->preprocessed),
                         ms(frame.impl_->preprocessed, frame.impl_->inferred),
                         ms(frame.impl_->inferred, end)};
        impl_->complete(frame);
        return result;
    } catch (...) {
        impl_->fail(frame);
        throw;
    }
}

InferenceInputSnapshot
StagedInference::diagnosticInputSnapshot(const InferenceFrame &frame) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureFrame(frame, InferenceFrame::Impl::Phase::Preprocessed);
    const auto &input = frame.impl_->input;
    InferenceInputSnapshot result;
    result.geometry = input.geometry;
    result.nv12 = input.nv12;
    result.bytes.assign(input.data, input.data + input.bytes);
    return result;
}

Tensor StagedInference::diagnosticSegmentationSnapshot(const InferenceFrame &frame) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureFrame(frame, InferenceFrame::Impl::Phase::Inferred);
    if (impl_->model_plan.task() != Task::Segmentation || !frame.impl_->tensors)
        throw std::runtime_error("segmentation snapshot requires segmentation outputs");
    return frame.impl_->tensors->at(impl_->model_plan.segmentation().seg_output);
}

Task StagedInference::task() const noexcept { return impl_->configured_task; }
ModelPlan StagedInference::plan() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->model_plan;
}
BackendState StagedInference::state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->closed ? BackendState::Closed : impl_->backend->state();
}
void StagedInference::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->closed)
        return;
    if (impl_->active_token)
        throw std::runtime_error("cannot close staged inference with an active frame");
    impl_->backend->close();
    impl_->closed = true;
}

} // namespace perception
