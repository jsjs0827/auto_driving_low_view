#include "perception/fault_signal.hpp"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <utility>

namespace perception {
namespace {
const char *businessMessage(std::uint32_t code) {
    switch (code) {
    case 400: return "RGB数据未收到";
    case 402: return "RGB数据异常";
    case 404: return "GDC图未收到";
    case 405: return "GDC图数据异常";
    case 412: return "模型文件异常";
    case 413: return "BPU资源申请异常";
    case 414: return "AI推理数据异常";
    case 416: return "感知日志异常";
    default: return "未映射的内部诊断";
    }
}
std::string quote(const std::string &value) {
    std::ostringstream out;
    out << '"';
    for (const unsigned char c : value) {
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20)
                out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << unsigned(c)
                    << std::dec;
            else
                out << char(c);
        }
    }
    out << '"';
    return out.str();
}
} // namespace

FaultSignal mapFault(const ErrorInfo &error, std::string task, ImageSource source) {
    FaultSignal event;
    event.task = std::move(task);
    event.frame = error.frame;
    event.internal = error;
    if (!error.ok()) {
        switch (error.code) {
        case ErrorCode::CameraAcquisitionFailed:
            event.event_code = source == ImageSource::Gdc ? 404 : 400;
            break;
        case ErrorCode::CameraDataInvalid:
            event.event_code = source == ImageSource::Gdc ? 405 : 402;
            break;
        case ErrorCode::ModelLoadFailed:
            event.event_code = 412;
            break;
        default:
            // Components identify the actual failing operation, never error-text matching.
            if (error.code == ErrorCode::AiInitFailed &&
                error.component == "bpu.memory.allocate")
                event.event_code = 413;
            else if (error.code == ErrorCode::PerceptionStateInvalid &&
                     error.component == "input.watchdog")
                event.event_code = 414;
            else if (error.component == "logging.disabled" ||
                     error.component == "logging.save" ||
                     error.component == "visualization.save") {
                event.event_code = 416;
                event.message_type = 2;
            }
        }
    }
    event.message = businessMessage(event.event_code.value_or(0));
    return event;
}

std::string toJson(const FaultSignal &event) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "{\"event_code\":";
    if (event.event_code)
        out << *event.event_code;
    else
        out << "null";
    out << ",\"message_type\":" << event.message_type
        << ",\"active\":" << (event.active ? "true" : "false")
        << ",\"task\":" << quote(event.task)
        << ",\"frame\":{\"id\":" << event.frame.id
        << ",\"timestamp_ns\":" << event.frame.timestamp_ns << "}"
        << ",\"message\":" << quote(event.message)
        << ",\"internal\":{\"code\":" << static_cast<std::uint32_t>(event.internal.code)
        << ",\"name\":" << quote(errorCodeName(event.internal.code))
        << ",\"component\":" << quote(event.internal.component)
        << ",\"detail\":" << quote(event.internal.detail)
        << ",\"frame\":{\"id\":" << event.internal.frame.id
        << ",\"timestamp_ns\":" << event.internal.frame.timestamp_ns << "}"
        << ",\"elapsed_ms\":";
    if (std::isfinite(event.internal.elapsed_ms))
        out << std::setprecision(17) << event.internal.elapsed_ms;
    else
        out << "null";
    out << ",\"context_codes\":[";
    for (std::size_t i = 0; i < event.internal.context_codes.size(); ++i) {
        if (i) out << ',';
        out << static_cast<std::uint32_t>(event.internal.context_codes[i]);
    }
    out << "]}}";
    return out.str();
}

FaultReporter::FaultReporter(Callback callback) : callback_(std::move(callback)) {}
void FaultReporter::setCallback(Callback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(callback);
}
bool FaultReporter::report(const ErrorInfo &error, std::string task, ImageSource source) {
    if (error.ok()) return false;
    return report(mapFault(error, std::move(task), source));
}
bool FaultReporter::report(FaultSignal event) {
    if (!event.active) {
        return event.event_code && recover(*event.event_code, event.task, event.frame,
                                            event.internal.detail);
    }
    if (!event.event_code && event.internal.ok()) return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (event.event_code) {
            const Key key{*event.event_code, event.task};
            if (active_.count(key)) return false;
            active_.emplace(key, event);
        }
        pending_.push_back(std::move(event));
    }
    dispatch();
    return true;
}
bool FaultReporter::recover(std::uint32_t code, const std::string &task, FrameInfo frame,
                            std::string detail) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = active_.find({code, task});
        if (it == active_.end()) return false;
        auto event = it->second;
        event.active = false;
        event.frame = frame;
        event.internal = {};
        event.internal.frame = frame;
        event.internal.component = it->second.internal.component;
        event.internal.detail = std::move(detail);
        pending_.push_back(std::move(event));
        active_.erase(it);
    }
    dispatch();
    return true;
}
bool FaultReporter::isActive(std::uint32_t code, const std::string &task) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_.count({code, task}) != 0;
}
void FaultReporter::dispatch() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dispatching_) return;
        dispatching_ = true;
    }
    for (;;) {
        FaultSignal event;
        Callback callback;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_.empty()) {
                dispatching_ = false;
                return;
            }
            event = std::move(pending_.front());
            pending_.pop_front();
            callback = callback_;
        }
        if (callback) {
            try { callback(event); } catch (...) { /* Host callbacks cannot fail inference. */ }
        }
    }
}

InputWatchdog::InputWatchdog(FaultReporter &reporter, std::string task,
                             std::chrono::milliseconds timeout)
    : reporter_(reporter), task_(std::move(task)), timeout_(timeout) {
    if (timeout_.count() <= 0)
        throw std::invalid_argument("input watchdog timeout must be positive");
}
void InputWatchdog::startWaiting(TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!waiting_) {
        started_ = now;
        waiting_ = true;
    }
}
void InputWatchdog::received(FrameInfo frame, TimePoint now) {
    (void)now; // Receipt ends waiting; an inference's duration must not be monitored.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        waiting_ = false;
        last_frame_ = frame;
        if (faulted_) {
            pending_.push_back({false, frame, 0, "valid input frame received"});
            faulted_ = false;
        }
    }
    dispatch();
}
void InputWatchdog::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        waiting_ = false;
        if (faulted_) {
            pending_.push_back({false, last_frame_, 0, "input monitoring stopped"});
            faulted_ = false;
        }
    }
    dispatch();
}
bool InputWatchdog::poll(TimePoint now) {
    bool triggered = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (waiting_ && !faulted_ && now >= started_ && now - started_ >= timeout_) {
            const double elapsed = std::chrono::duration<double, std::milli>(now - started_).count();
            pending_.push_back({true, last_frame_, elapsed,
                                "no valid image received while waiting for input"});
            faulted_ = true;
            triggered = true;
        }
    }
    dispatch();
    return triggered;
}
void InputWatchdog::dispatch() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dispatching_) return;
        dispatching_ = true;
    }
    for (;;) {
        Transition transition;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_.empty()) {
                dispatching_ = false;
                return;
            }
            transition = std::move(pending_.front());
            pending_.pop_front();
        }
        if (transition.active) {
            ErrorInfo error;
            error.code = ErrorCode::PerceptionStateInvalid;
            error.component = "input.watchdog";
            error.detail = std::move(transition.detail);
            error.frame = transition.frame;
            error.elapsed_ms = transition.elapsed_ms;
            reporter_.report(error, task_);
        } else {
            reporter_.recover(414, task_, transition.frame, std::move(transition.detail));
        }
    }
}
} // namespace perception
