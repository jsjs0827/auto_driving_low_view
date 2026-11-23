#pragma once

#include "perception/error.hpp"
#include "perception/types.hpp"

#include <chrono>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace perception {

struct RgbFrame {
    FrameInfo frame;
    int width = 0, height = 0;
    std::vector<std::uint8_t> pixels; // Owned, tightly packed RGB24.
    double conversion_ms = 0;
};

// One instance belongs to one synchronous frame execution. The caller retains
// the source buffer until that execution has joined every task. Converted RGB
// is owned here, immutable after publication and shared by all frame consumers.
// There is deliberately no cross-frame or process-wide image cache.
class FrameResources {
public:
    using Converter = std::function<std::vector<std::uint8_t>(const ImageView&)>;

    FrameResources(ImageView left, FrameInfo frame, Converter converter)
        : left_(left), frame_(frame), converter_(std::move(converter)) {
        validateCameraImage(left_);
        if (!converter_)
            throw std::invalid_argument("frame RGB converter is required");
    }
    FrameResources(const FrameResources&) = delete;
    FrameResources& operator=(const FrameResources&) = delete;

    const ImageView& left() const noexcept { return left_; }
    FrameInfo identity() const noexcept { return frame_; }

    const RgbFrame& leftRgb() const {
        std::call_once(rgb_once_, [this] {
            const auto start = std::chrono::steady_clock::now();
            // Build off to the side. Exceptions and malformed converter output
            // leave the cache unpublished; call_once permits a later retry.
            RgbFrame candidate;
            candidate.frame = frame_;
            candidate.width = left_.width;
            candidate.height = left_.height;
            candidate.pixels = converter_(left_);
            const auto expected = std::size_t(left_.width) * left_.height * 3U;
            if (candidate.pixels.size() != expected)
                throw std::runtime_error("frame RGB converter returned an invalid tight RGB24 buffer");
            candidate.conversion_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            rgb_ = std::move(candidate);
        });
        return rgb_;
    }

private:
    ImageView left_;
    FrameInfo frame_;
    Converter converter_;
    mutable std::once_flag rgb_once_;
    mutable RgbFrame rgb_;
};

} // namespace perception
