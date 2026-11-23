#ifndef AI_NODE_PERCEP_DEMO_SAVE_MAP_TASK_HPP_
#define AI_NODE_PERCEP_DEMO_SAVE_MAP_TASK_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "percep_demo/common.hpp"
#include "percep_demo/pre_pointcloud_types.hpp"

namespace ai_node_percep_demo {

// Immutable-by-convention snapshot transferred from the processing tick to the
// asynchronous save thread. All buffers own their storage; no camera callback
// memory is retained here.
struct SaveMapTask {
    OccMapData map_data;
    std::vector<std::uint8_t> rgb;
    GdcFramePtr gdc;
    std::vector<float> disp;
    std::vector<std::uint32_t> u;
    std::vector<std::uint32_t> v;
    std::string time_str;
    int rows{0};
    int cols{0};
};

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_SAVE_MAP_TASK_HPP_
