#include "perception/engine.hpp"
#include "detail/diagnostic_boundary.hpp"
#include <chrono>
#include <mutex>
#include <stdexcept>

namespace perception {
namespace {
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
template <class T, class F>
void execute(TaskResult<T> &out, const std::shared_ptr<Logger> &logger, const char *component,
             FrameInfo frame, ErrorCode fallback, F fn) {
    const auto start = Clock::now();
    try {
        out.value = fn();
        out.status = RunStatus::Ok;
    } catch (const std::exception &e) {
        out.status = RunStatus::Error;
        out.error = e.what();
        out.diagnostic = errorFromException(fallback, e, component);
        out.diagnostic.component = std::string(component) + "/" + out.diagnostic.component;
    }
    out.elapsed_ms = milliseconds(start);
    if (!out.diagnostic.ok()) {
        out.diagnostic.frame = frame;
        out.diagnostic.elapsed_ms = out.elapsed_ms;
        if (logger)
            logger->error(out.diagnostic);
    }
}
} // namespace
const char *runStatusName(RunStatus status) noexcept {
    switch (status) {
    case RunStatus::Disabled:
        return "disabled";
    case RunStatus::Ok:
        return "ok";
    case RunStatus::NotFound:
        return "not_found";
    case RunStatus::Error:
        return "error";
    }
    return "error";
}
bool FrameResult::hasError() const noexcept {
    return detection.status == RunStatus::Error || segmentation.status == RunStatus::Error;
}
struct PerceptionEngine::Impl {
    std::mutex mutex;
    bool closed = false;
    std::uint64_t completed = 0;
    std::shared_ptr<Logger> logger;
    std::unique_ptr<Detector> detector;
    std::unique_ptr<Segmenter> segmenter;
    std::unique_ptr<runtime::Engine> runtime_engine;
};
PerceptionEngine::PerceptionEngine(EngineConfig cfg)
    : PerceptionEngine(std::move(cfg), nullptr, nullptr) {}
PerceptionEngine::PerceptionEngine(EngineConfig cfg, std::unique_ptr<Backend> det,
                                   std::unique_ptr<Backend> seg)
    : impl_(new Impl) {
    impl_->logger = cfg.logger;
    detail::log(cfg.logger, LogLevel::Info, "engine.init.begin", "engine");
    try {
        if (!cfg.detection && !cfg.segmentation)
            throw std::invalid_argument("enable at least one perception task");
        if ((det && !cfg.detection) || (seg && !cfg.segmentation))
            throw std::invalid_argument("backend provided for disabled task");
        if (cfg.detection) {
            if (!cfg.detection->model.logger)
                cfg.detection->model.logger = cfg.logger;
            detail::boundary(ErrorCode::AiInitFailed, "detection.config",
                             [&] { cfg.detection->validate(); });
            if (!std::holds_alternative<DetectionConfig>(cfg.detection->task))
                throw PerceptionException(ErrorCode::AiInitFailed,
                                          "detection profile has wrong task", "detection.config");
            auto task = std::get<DetectionConfig>(cfg.detection->task);
            impl_->detector =
                det ? std::make_unique<Detector>(cfg.detection->model, task, std::move(det))
                    : std::make_unique<Detector>(cfg.detection->model, task);
        }
        if (cfg.segmentation) {
            if (!cfg.segmentation->model.logger)
                cfg.segmentation->model.logger = cfg.logger;
            detail::boundary(ErrorCode::AiInitFailed, "segmentation.config",
                             [&] { cfg.segmentation->validate(); });
            if (!std::holds_alternative<SegmentationConfig>(cfg.segmentation->task))
                throw PerceptionException(ErrorCode::AiInitFailed,
                                          "segmentation profile has wrong task",
                                          "segmentation.config");
            auto task = std::get<SegmentationConfig>(cfg.segmentation->task);
            impl_->segmenter =
                seg ? std::make_unique<Segmenter>(cfg.segmentation->model, task, std::move(seg))
                    : std::make_unique<Segmenter>(cfg.segmentation->model, task);
        }
        detail::log(cfg.logger, LogLevel::Info, "engine.ready", "engine");
    } catch (const std::exception &e) {
        auto error =
            withErrorContext(errorFromException(ErrorCode::PerceptionInitFailed, e, "engine.init"),
                             ErrorCode::PerceptionInitFailed);
        if (cfg.logger)
            cfg.logger->error(error);
        throw PerceptionException(std::move(error));
    }
}
PerceptionEngine::PerceptionEngine(runtime::ConfigPath task_graph_config) : impl_(new Impl) {
    impl_->runtime_engine = std::make_unique<runtime::Engine>(task_graph_config.value);
}
PerceptionEngine::~PerceptionEngine() {
    detail::log(impl_->logger, LogLevel::Info, "engine.destroy", "engine");
}
FrameResult PerceptionEngine::run(const ImageView &image, FrameInfo frame) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->closed) {
        throw PerceptionException(reportError(impl_->logger, ErrorCode::PerceptionStateInvalid,
                                              "perception engine is closed", "engine.run", frame));
    }
    if (impl_->runtime_engine)
        throw std::logic_error("image-only run is unavailable for the r12 task-graph engine");
    const auto start = Clock::now();
    if (impl_->logger)
        impl_->logger->write(LogLevel::Debug, "frame.begin", "engine", {}, frame);
    FrameResult result;
    result.frame = frame;
    if (impl_->detector)
        execute(result.detection, impl_->logger, "detection", frame, ErrorCode::InferenceFailed,
                [&] { return impl_->detector->run(image); });
    if (impl_->segmenter)
        execute(result.segmentation, impl_->logger, "segmentation", frame,
                ErrorCode::InferenceFailed, [&] { return impl_->segmenter->run(image); });
    result.elapsed_ms = milliseconds(start);
    ++impl_->completed;
    if (impl_->logger) {
        impl_->logger->write(LogLevel::Debug, "frame.end", "engine",
                             result.hasError() ? "error" : "completed", frame, result.elapsed_ms);
        const auto every = impl_->logger->progressEveryFrames();
        if (every && (impl_->completed == 1 || impl_->completed % every == 0))
            impl_->logger->write(LogLevel::Info, "engine.progress", "engine",
                                 result.hasError() ? "frame has errors" : "frame completed", frame,
                                 result.elapsed_ms);
    }
    return result;
}
runtime::FrameResult PerceptionEngine::run(const runtime::FrameInput &input) {
    runtime::Engine *runtime_engine = nullptr;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->closed)
            throw std::logic_error("perception engine is closed");
        if (!impl_->runtime_engine)
            throw std::logic_error("task-graph run requires the r12 configuration constructor");
        runtime_engine = impl_->runtime_engine.get();
    }
    // The runtime invokes transport callbacks after its own lock is released.
    // Do not retain this wrapper lock across that callback either.
    return runtime_engine->run(input);
}
std::vector<std::string> PerceptionEngine::registeredFeatures() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->runtime_engine ? impl_->runtime_engine->registeredFeatures()
                                 : std::vector<std::string>{};
}
void PerceptionEngine::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->closed)
        return;
    // Set state first so a failed close can never permit another run.
    impl_->closed = true;
    std::exception_ptr error;
    try {
        if (impl_->detector)
            impl_->detector->close();
    } catch (...) {
        error = std::current_exception();
    }
    try {
        if (impl_->segmenter)
            impl_->segmenter->close();
    } catch (...) {
        if (!error)
            error = std::current_exception();
    }
    try {
        if (impl_->runtime_engine)
            impl_->runtime_engine->close();
    } catch (...) {
        if (!error)
            error = std::current_exception();
    }
    if (error)
        try {
            std::rethrow_exception(error);
        } catch (const std::exception &e) {
            auto fault = errorFromException(ErrorCode::PerceptionStateInvalid, e, "engine.close");
            if (impl_->logger)
                impl_->logger->error(fault);
            throw PerceptionException(std::move(fault));
        }
    detail::log(impl_->logger, LogLevel::Info, "engine.closed", "engine");
}
} // namespace perception
