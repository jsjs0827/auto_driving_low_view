#pragma once
#include "perception_full_sdk.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <filesystem>
#include <string>

namespace ros_so_demo {
std::string cloudName(const psdk_cloud_view& cloud);
std::string cloudFrame(const psdk_cloud_view& cloud,const std::string& optical_frame,
                              const std::string& robot_frame);
void validateCloud(const psdk_cloud_view& cloud);
void cloudU32LE(uint8_t* destination,uint32_t value);
void cloudFloatLE(uint8_t* destination,float value);
sensor_msgs::msg::PointCloud2 makePointCloud2(const psdk_cloud_view& cloud,
        const std_msgs::msg::Header& capture,const std::string& robot_frame);
std::string cloudJsonQuote(const std::string& text);
void writePly(const psdk_cloud_view& cloud,const std::filesystem::path& path,
                     const std::string& frame);
void savePlyClouds(const psdk_full_result_view& result,const std::filesystem::path& directory,
        const std_msgs::msg::Header& capture,const std::string& robot_frame);
} // namespace ros_so_demo
