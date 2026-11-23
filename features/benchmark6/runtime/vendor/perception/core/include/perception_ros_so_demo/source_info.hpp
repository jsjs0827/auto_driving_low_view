#pragma once
#include <string>

namespace ros_so_demo {
struct SourceInfo {
    std::string id_source="ros_pair_sequence", timestamp_source="ros_header", file, right_file, resize="none";
    unsigned width=640,height=544;
};
} // namespace ros_so_demo
