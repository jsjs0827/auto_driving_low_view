#ifndef AI_NODE_PERCEP_DEMO_DNN_PALETTE_HPP_
#define AI_NODE_PERCEP_DEMO_DNN_PALETTE_HPP_

#include <cstdint>
#include <string>

namespace ai_node_percep_demo {

// Keep DNN visualization independent from the ROS/PCL-heavy common.hpp so the
// production inference runner can also be built as a small standalone X5
// executable.
inline void SegClassVisRgb(int class_id, const std::string& class_name,
                           std::uint8_t& red, std::uint8_t& green,
                           std::uint8_t& blue) {
    static constexpr std::uint8_t kPalette[16][3] = {
        {128, 64, 128},  {0, 180, 0},     {244, 35, 232},
        {70, 70, 70},    {102, 102, 156}, {190, 153, 153},
        {153, 153, 153}, {250, 170, 30},  {220, 220, 0},
        {0, 128, 255},   {255, 128, 0},   {0, 255, 255},
        {139, 69, 19},   {192, 192, 192}, {64, 64, 64},
        {180, 180, 40},
    };

    if (class_name == "grass") {
        red = 0;
        green = 180;
        blue = 0;
        return;
    }
    if (class_name == "no_grass") {
        red = 128;
        green = 64;
        blue = 128;
        return;
    }
    if (class_name == "flagstone") {
        red = 250;
        green = 170;
        blue = 30;
        return;
    }
    if (class_name == "fence") {
        red = 244;
        green = 35;
        blue = 232;
        return;
    }
    if (class_name == "puddle" || class_name == "waterpipe") {
        red = 0;
        green = 128;
        blue = 255;
        return;
    }

    const int palette_index = class_id >= 0 ? class_id % 16 : 0;
    red = kPalette[palette_index][0];
    green = kPalette[palette_index][1];
    blue = kPalette[palette_index][2];
}

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_DNN_PALETTE_HPP_
