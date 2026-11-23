#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace ros_so_demo::raw {
struct Identity { std::uint64_t id=0, ns=0; };
struct Plane {
    Identity capture;
    std::uint32_t width=0, height=0, step=0;
    std::string encoding;
    std::vector<std::uint8_t> bytes;
};
struct Capture { Plane left, right, disparity, gdc; };

} // namespace ros_so_demo::raw
