#ifndef AI_NODE_PERCEP_DEMO_RGB_RESIZE_HPP_
#define AI_NODE_PERCEP_DEMO_RGB_RESIZE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ai_node_percep_demo {

// Half-pixel RGB24 bilinear resize with replicate-border semantics. Continuous
// coordinates are clamped before floor/weight calculation; clamping only the
// integer index would leave negative weights and accidentally extrapolate at
// the top/left border.
inline bool resizeRgb24Bilinear(const std::uint8_t* source, int source_width,
                                int source_height,
                                std::vector<std::uint8_t>* destination,
                                int destination_width,
                                int destination_height) {
    if (source == nullptr || destination == nullptr || source_width <= 0 ||
        source_height <= 0 || destination_width <= 0 ||
        destination_height <= 0) {
        return false;
    }
    if (source_width == destination_width &&
        source_height == destination_height) {
        destination->assign(
            source,
            source + static_cast<std::size_t>(source_width) * source_height *
                         3U);
        return true;
    }

    destination->assign(
        static_cast<std::size_t>(destination_width) * destination_height * 3U,
        0U);
    for (int y = 0; y < destination_height; ++y) {
        const float source_y = std::clamp(
            (static_cast<float>(y) + 0.5F) * source_height /
                    destination_height -
                0.5F,
            0.0F, static_cast<float>(source_height - 1));
        const int y0 = static_cast<int>(std::floor(source_y));
        const int y1 = std::min(source_height - 1, y0 + 1);
        const float weight_y = source_y - static_cast<float>(y0);

        for (int x = 0; x < destination_width; ++x) {
            const float source_x = std::clamp(
                (static_cast<float>(x) + 0.5F) * source_width /
                        destination_width -
                    0.5F,
                0.0F, static_cast<float>(source_width - 1));
            const int x0 = static_cast<int>(std::floor(source_x));
            const int x1 = std::min(source_width - 1, x0 + 1);
            const float weight_x = source_x - static_cast<float>(x0);

            for (int channel = 0; channel < 3; ++channel) {
                const float v00 = source
                    [(static_cast<std::size_t>(y0) * source_width + x0) * 3U +
                     channel];
                const float v01 = source
                    [(static_cast<std::size_t>(y0) * source_width + x1) * 3U +
                     channel];
                const float v10 = source
                    [(static_cast<std::size_t>(y1) * source_width + x0) * 3U +
                     channel];
                const float v11 = source
                    [(static_cast<std::size_t>(y1) * source_width + x1) * 3U +
                     channel];
                const float top = v00 + (v01 - v00) * weight_x;
                const float bottom = v10 + (v11 - v10) * weight_x;
                const float value = std::clamp(
                    top + (bottom - top) * weight_y, 0.0F, 255.0F);
                (*destination)
                    [(static_cast<std::size_t>(y) * destination_width + x) *
                         3U +
                     channel] = static_cast<std::uint8_t>(value + 0.5F);
            }
        }
    }
    return true;
}

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_RGB_RESIZE_HPP_
