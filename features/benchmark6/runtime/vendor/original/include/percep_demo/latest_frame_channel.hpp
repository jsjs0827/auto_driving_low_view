#ifndef AI_NODE_PERCEP_DEMO_LATEST_FRAME_CHANNEL_HPP_
#define AI_NODE_PERCEP_DEMO_LATEST_FRAME_CHANNEL_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>

namespace ai_node_percep_demo {

enum class ChannelTakeStatus {
    kValue,
    kEmpty,
    kTimeout,
    kStopped,
};

enum class ChannelStopMode {
    // Stop immediately and account for an unconsumed frame as discarded.
    kDiscardPending,
    // Keep the last frame available. Once it is taken, reads return kStopped.
    kDrainPending,
};

struct LatestFrameChannelStats {
    std::uint64_t accepted{0};
    std::uint64_t taken{0};
    std::uint64_t overwritten{0};
    std::uint64_t discarded_by_clear{0};
    std::uint64_t discarded_on_stop{0};
    std::uint64_t rejected_after_stop{0};
    bool has_pending{false};
    bool stopped{false};

    std::uint64_t dropped() const noexcept {
        return overwritten + discarded_by_clear + discarded_on_stop;
    }
};

template <typename T>
struct ChannelTakeResult {
    ChannelTakeStatus status{ChannelTakeStatus::kEmpty};
    std::optional<T> value;

    explicit operator bool() const noexcept {
        return status == ChannelTakeStatus::kValue;
    }
};

// Thread-safe latest-value channel with a logical capacity of one.
//
// A successful publish replaces an unread value instead of queueing behind it.
// T is owned by the channel; ownership of any memory referenced by pointers
// inside T remains the caller's responsibility.
//
// Destruction is not synchronization. The owner must call stop(), join every
// thread that accesses the channel, and only then destroy the channel.
template <typename T>
class LatestFrameChannel final {
public:
    static_assert(std::is_move_constructible<T>::value,
                  "LatestFrameChannel<T> requires move-constructible T");
    static_assert(std::is_nothrow_move_constructible<T>::value,
                  "LatestFrameChannel<T> requires noexcept move construction");

    using Result = ChannelTakeResult<T>;
    using Stats = LatestFrameChannelStats;

    LatestFrameChannel() = default;
    ~LatestFrameChannel() = default;

    LatestFrameChannel(const LatestFrameChannel&) = delete;
    LatestFrameChannel& operator=(const LatestFrameChannel&) = delete;
    LatestFrameChannel(LatestFrameChannel&&) = delete;
    LatestFrameChannel& operator=(LatestFrameChannel&&) = delete;

    // Copies an lvalue into the channel. This overload is instantiated only
    // when it is used, so move-only T remains supported.
    bool publish(const T& value) {
        // Copy before locking. If T's copy throws, the unread latest value is
        // preserved and producers do not hold the channel mutex while doing
        // potentially expensive work.
        T prepared(value);
        return publishImpl(std::move(prepared));
    }

    // Moves only after the stopped check. On rejection, value is untouched.
    bool publish(T&& value) {
        return publishImpl(std::move(value));
    }

    template <typename... Args>
    bool emplace(Args&&... args) {
        // Build first to give replacement a strong exception guarantee: a
        // throwing constructor cannot erase the previous unread frame.
        T prepared(std::forward<Args>(args)...);
        return publishImpl(std::move(prepared));
    }

    Result tryTakeLatest() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_) {
            return takeLocked();
        }

        return {stopped_ ? ChannelTakeStatus::kStopped
                         : ChannelTakeStatus::kEmpty,
                std::nullopt};
    }

    Result waitTakeLatest() {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {
            return stopped_ || latest_.has_value();
        });

        // A pending value wins over stopped in drain mode.
        if (latest_) {
            return takeLocked();
        }
        return {ChannelTakeStatus::kStopped, std::nullopt};
    }

    template <typename Rep, typename Period>
    Result waitTakeLatestFor(
        const std::chrono::duration<Rep, Period>& timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool ready = condition_.wait_for(lock, timeout, [this]() {
            return stopped_ || latest_.has_value();
        });

        if (!ready) {
            return {ChannelTakeStatus::kTimeout, std::nullopt};
        }
        if (latest_) {
            return takeLocked();
        }
        return {ChannelTakeStatus::kStopped, std::nullopt};
    }

    // Drops a pending value without closing the channel. This is useful at a
    // work-mode boundary, where processing one frame from the previous mode is
    // less correct than waiting for a fresh frame.
    bool discardPending() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_) {
            return false;
        }
        latest_.reset();
        ++discarded_by_clear_;
        return true;
    }

    // Idempotent. The first call fixes the stop mode; later calls are no-ops.
    // After stop(), all publish/emplace calls fail and are counted as rejected.
    void stop(
        ChannelStopMode mode = ChannelStopMode::kDiscardPending) noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) {
                return;
            }

            stopped_ = true;
            if (mode == ChannelStopMode::kDiscardPending && latest_) {
                latest_.reset();
                ++discarded_on_stop_;
            }
        }

        condition_.notify_all();
    }

    Stats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return Stats{
            accepted_,
            taken_,
            overwritten_,
            discarded_by_clear_,
            discarded_on_stop_,
            rejected_after_stop_,
            latest_.has_value(),
            stopped_,
        };
    }

private:
    template <typename U>
    bool publishImpl(U&& value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) {
                ++rejected_after_stop_;
                return false;
            }

            const bool replacing = latest_.has_value();
            latest_.emplace(std::forward<U>(value));
            ++accepted_;
            if (replacing) {
                ++overwritten_;
            }
        }

        condition_.notify_one();
        return true;
    }

    // Precondition: mutex_ is held.
    Result takeLocked() {
        std::optional<T> value(std::in_place, std::move(*latest_));
        latest_.reset();
        ++taken_;
        return {ChannelTakeStatus::kValue, std::move(value)};
    }

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<T> latest_;
    bool stopped_{false};

    std::uint64_t accepted_{0};
    std::uint64_t taken_{0};
    std::uint64_t overwritten_{0};
    std::uint64_t discarded_by_clear_{0};
    std::uint64_t discarded_on_stop_{0};
    std::uint64_t rejected_after_stop_{0};
};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_LATEST_FRAME_CHANNEL_HPP_
