#pragma once
#include "perception/error.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>

namespace perception {
enum class ImageSource { Rgb, Gdc };

// Owning event: safe to copy into a host application's message queue. Existing
// ErrorCode wire values remain internal diagnostics; event_code is the business code.
struct FaultSignal {
    std::optional<std::uint32_t> event_code;
    std::uint32_t message_type = 4; // 4 = fault; 2 = warning.
    bool active = true;            // false = recovery/explicit monitoring stop.
    std::string task;
    FrameInfo frame;
    ErrorInfo internal;
    std::string message;
};

// Does not infer input loss from inference duration, model outputs, or image pixels.
// Unmapped failures have event_code=nullopt and retain their full diagnostic.
FaultSignal mapFault(const ErrorInfo &, std::string task = {},
                     ImageSource source = ImageSource::Rgb);
std::string toJson(const FaultSignal &);

class FaultReporter {
  public:
    using Callback = std::function<void(const FaultSignal &)>;
    explicit FaultReporter(Callback callback = {});
    void setCallback(Callback);
    // True means a transition was queued. Active business faults are deduplicated
    // by code + task. Unmapped errors are delivered on every report.
    bool report(FaultSignal);
    bool report(const ErrorInfo &, std::string task = {},
                ImageSource source = ImageSource::Rgb);
    bool recover(std::uint32_t code, const std::string &task = {}, FrameInfo frame = {},
                 std::string detail = {});
    bool isActive(std::uint32_t code, const std::string &task = {}) const;

  private:
    using Key = std::pair<std::uint32_t, std::string>;
    mutable std::mutex mutex_;
    std::map<Key, FaultSignal> active_;
    std::deque<FaultSignal> pending_;
    Callback callback_;
    bool dispatching_ = false;
    // Callbacks are serialized, run without mutex_, and may reenter the reporter.
    // Callback exceptions are contained so monitoring cannot abort inference.
    void dispatch();
};

// Explicit input-wait monitor. Start before waiting for a NEW input, received()
// only after a valid frame arrives, then leave it idle throughout inference.
// Call poll() from a host timer while waiting; no worker thread or sleep is hidden.
// supplied times must use the same monotonic Clock (also enables deterministic tests).
class InputWatchdog {
  public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    explicit InputWatchdog(FaultReporter &, std::string task = {},
                           std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
    void startWaiting(TimePoint now = Clock::now());
    void received(FrameInfo frame = {}, TimePoint now = Clock::now());
    void stop();
    bool poll(TimePoint now = Clock::now());

  private:
    struct Transition {
        bool active;
        FrameInfo frame;
        double elapsed_ms;
        std::string detail;
    };
    FaultReporter &reporter_;
    const std::string task_;
    const std::chrono::milliseconds timeout_;
    std::mutex mutex_;
    TimePoint started_{};
    FrameInfo last_frame_{};
    bool waiting_ = false, faulted_ = false, dispatching_ = false;
    std::deque<Transition> pending_;
    void dispatch();
};
} // namespace perception
