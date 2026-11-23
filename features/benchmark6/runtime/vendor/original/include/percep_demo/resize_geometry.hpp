#ifndef AI_NODE_PERCEP_DEMO_RESIZE_GEOMETRY_HPP_
#define AI_NODE_PERCEP_DEMO_RESIZE_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>

namespace ai_node_percep_demo {

// Map a (possibly fractional) coordinate in the destination grid back to the
// source grid using the same convention as tensor resize operators.  Keeping
// this in a host-only header lets us test the geometry without a Horizon SDK.
inline float resizeDestinationToSource(float destination_coordinate,
                                       int destination_size,
                                       int source_size,
                                       bool align_corners) noexcept {
    if (!std::isfinite(destination_coordinate) || destination_size <= 1 ||
        source_size <= 1) {
        return 0.0F;
    }
    float source_coordinate = 0.0F;
    if (align_corners) {
        source_coordinate =
            destination_coordinate * static_cast<float>(source_size - 1) /
            static_cast<float>(destination_size - 1);
    } else {
        source_coordinate =
            (destination_coordinate + 0.5F) *
                static_cast<float>(source_size) /
                static_cast<float>(destination_size) -
            0.5F;
    }
    return std::max(0.0F,
                    std::min(static_cast<float>(source_size - 1),
                             source_coordinate));
}

inline float resizeDestinationToSource(int destination_index,
                                       int destination_size,
                                       int source_size,
                                       bool align_corners) noexcept {
    return resizeDestinationToSource(static_cast<float>(destination_index),
                                     destination_size, source_size,
                                     align_corners);
}

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_RESIZE_GEOMETRY_HPP_
