#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <rapidjson/document.h>

namespace perception {

struct DarkSceneRawFrameView {
    const std::uint8_t* data = nullptr;
    std::size_t bytes = 0;
    int width = 0, height = 0;
    std::size_t stride_bytes = 0;
    std::uint64_t frame_id = 0, capture_timestamp_ns = 0;
};

enum class DarkSceneLabel {
    Unknown = -1,
    Day = 0,
    Night = 1,
    Normal = 2,
    LowLight = 3,
    VeryLowLight = 4,
};

inline const char* darkSceneLabelName(DarkSceneLabel label) noexcept {
    switch (label) {
    case DarkSceneLabel::Day: return "DAY";
    case DarkSceneLabel::Night: return "NIGHT";
    case DarkSceneLabel::Normal: return "NOT_LOW_LIGHT";
    case DarkSceneLabel::LowLight: return "LOW_LIGHT";
    case DarkSceneLabel::VeryLowLight: return "VERY_LOW_LIGHT";
    default: return "UNKNOWN";
    }
}

struct DarkSceneWindowVote {
    DarkSceneLabel label = DarkSceneLabel::Unknown;
    std::size_t samples = 0, dark_votes = 0, not_dark_votes = 0;
    std::size_t sample_count = 0, consensus_count = 0;
    std::uint64_t sample_interval_ms = 0, latest_sample_steady_ns = 0;
};

struct DarkSceneResult {
    // input_accepted and updated describe this call. Other decision fields
    // describe the latest policy state, even when the call did not update it.
    bool input_accepted = false, updated = false;
    bool ready = false, held = false, stale = false;
    bool is_dark = false;  // Valid only when ready && !stale.
    bool has_light_recommendation = false, light_on = false;
    bool light_changed = false;  // First recommendation or a new ON/OFF value on this call.
    bool has_source_exposure = false;
    DarkSceneLabel label = DarkSceneLabel::Unknown;
    std::uint64_t frame_id = 0, capture_timestamp_ns = 0;
    std::uint64_t emitted_steady_ns = 0, confirmed_steady_ns = 0;
    double source_exposure_ms = 0, source_analog_gain = 0;
    DarkSceneWindowVote light_on_window, light_off_window;
    std::string reason;
    // Supporting data for the continuous, periodic and RAW policies.
    std::size_t votes = 0, window_size = 0;
    std::uint64_t cycle_id = 0, valid_until_steady_ns = 0;
    bool classification_valid = false;
    DarkSceneLabel raw_label = DarkSceneLabel::Unknown;
    std::array<double, 3> raw_probabilities{};
    double feature_ms = 0, classification_ms = 0, classification_total_ms = 0;
};

// The host supplies frames and calls the policy synchronously. Calls on one
// instance are serialized. Do not call any method while destroying the instance.
class __attribute__((visibility("default"))) DarkSceneRuntime {
public:
    DarkSceneRuntime();
    ~DarkSceneRuntime();
    DarkSceneRuntime(const DarkSceneRuntime&) = delete;
    DarkSceneRuntime& operator=(const DarkSceneRuntime&) = delete;
    bool start(const rapidjson::Value& root, bool offline, std::string* error);
    void stop() noexcept;
    bool enabled() const noexcept;

    // Supply original frame identity, exposure in milliseconds, and gain as a
    // linear multiplier. The policy samples eligible calls at its own cadence.
    DarkSceneResult evaluateExposure(std::uint64_t frame_id, std::uint64_t capture_timestamp_ns,
                                    double exposure_ms, double analog_gain);
    // RAW matches the exact exposure frame. The caller keeps the view readable
    // during this call. Mode5 applies its existing exposure-based computation gate.
    DarkSceneResult evaluateRaw(const DarkSceneRawFrameView& frame);
    // Advance the monotonic clock and return the latest state. Call this when
    // checking expiration during input loss, because there is no background timer.
    DarkSceneResult readResult();

    // Source-compatible entry points for callers that discard the returned state.
    void feedExposure(std::uint64_t frame_id, std::uint64_t capture_timestamp_ns,
                      double exposure_ms, double analog_gain) noexcept;
    bool feedRaw(const DarkSceneRawFrameView& frame, std::string* error = nullptr) noexcept;
private:
    struct __attribute__((visibility("hidden"))) Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace perception
