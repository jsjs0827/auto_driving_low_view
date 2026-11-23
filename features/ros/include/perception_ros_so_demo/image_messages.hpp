#pragma once
#include "perception_sdk.h"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ros_so_demo {
uint64_t timestamp(const std_msgs::msg::Header& header);
psdk_frame imageView(const sensor_msgs::msg::Image& image,uint64_t capture_id,uint64_t stamp);
} // namespace ros_so_demo
