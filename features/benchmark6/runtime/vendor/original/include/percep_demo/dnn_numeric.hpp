#ifndef AI_NODE_PERCEP_DEMO_DNN_NUMERIC_HPP_
#define AI_NODE_PERCEP_DEMO_DNN_NUMERIC_HPP_

#include <cmath>
#include <cstdint>
#include <limits>

namespace ai_node_percep_demo {

// SCALE dequantization must widen before subtraction. int64_t preserves the
// exact difference of any two int32_t values; doing the subtraction in float
// would collapse adjacent large integers to the same value. Multiplication is
// performed in double so range can be checked before narrowing to float.
inline float dequantizeScale(std::int32_t raw, std::int32_t zero_point,
                             float scale) noexcept {
    if (!std::isfinite(scale) || !(scale > 0.0F)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const std::int64_t centered = static_cast<std::int64_t>(raw) -
                                  static_cast<std::int64_t>(zero_point);
    const double value = static_cast<double>(centered) *
                         static_cast<double>(scale);
    const double float_max =
        static_cast<double>(std::numeric_limits<float>::max());
    if (!std::isfinite(value) || value > float_max || value < -float_max) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(value);
}

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_DNN_NUMERIC_HPP_
