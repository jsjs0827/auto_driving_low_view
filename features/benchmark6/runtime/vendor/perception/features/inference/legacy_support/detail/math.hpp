#pragma once
#include "perception/types.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace perception {
namespace {
inline void require(bool ok, const char *message) {
    if (!ok)
        throw std::invalid_argument(message);
}
inline std::size_t pixels(int w, int h) {
    require(w > 0 && h > 0 && w <= 4096 && h <= 4096, "image size must be 1..4096");
    return static_cast<std::size_t>(w) * h;
}
inline std::uint8_t byte(float x) {
    return static_cast<std::uint8_t>(std::clamp(std::lround(x), 0L, 255L));
}
inline float coordinate(int i, int src, int dst, bool corners) {
    return corners ? (dst == 1 ? 0.f : float(i) * (src - 1) / (dst - 1))
                   : (float(i) + .5f) * src / dst - .5f;
}
inline void geometryValid(const Geometry &g) {
    pixels(g.source_w, g.source_h);
    pixels(g.input_w, g.input_h);
    require(g.resized_w > 0 && g.resized_h > 0 && g.left >= 0 && g.top >= 0 &&
                g.resized_w <= g.input_w - g.left && g.resized_h <= g.input_h - g.top,
            "invalid geometry");
}
inline float sample(const Tensor &t, int c, float x, float y) {
    x = std::clamp(x, 0.f, float(t.width - 1));
    y = std::clamp(y, 0.f, float(t.height - 1));
    const int x0 = int(x), y0 = int(y), x1 = std::min(x0 + 1, t.width - 1),
              y1 = std::min(y0 + 1, t.height - 1);
    const double wx = x - x0, wy = y - y0;
    return float((1 - wy) * ((1 - wx) * t.at(c, y0, x0) + wx * t.at(c, y0, x1)) +
                 wy * ((1 - wx) * t.at(c, y1, x0) + wx * t.at(c, y1, x1)));
}
inline float iou(const Detection &a, const Detection &b) {
    const float inter = std::max(0.f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1)) *
                        std::max(0.f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    const float area = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
    return area > 0 ? inter / area : 0;
}
} // namespace
} // namespace perception
