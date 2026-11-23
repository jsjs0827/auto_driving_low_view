#pragma once

#include "rviz_settings.hpp"
#include "rviz_wire.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <map>
#include <string>

namespace perception::rviz {
// Source timestamps are preserved verbatim; no receive-time substitution or TF
// is performed. ROS Header has no numeric frame number: /perception_rviz/meta
// carries frame_number, session and sequence separately.
std_msgs::msg::Header sourceHeader(const Frame& frame, const Packet& packet);
sensor_msgs::msg::Image imageMessage(const Image& image, const std_msgs::msg::Header& header);
sensor_msgs::msg::PointCloud2 cloudMessage(const PointCloud& cloud, const std_msgs::msg::Header& header);
visualization_msgs::msg::MarkerArray slipMarkers(const std::string& json,
    const ViewSettings& view, const rclcpp::Time& display_time);
visualization_msgs::msg::MarkerArray resultMarkers(const std::string& json,
    const ViewSettings& view, const rclcpp::Time& display_time);

// Only locally configured, known stream keys create publishers. Remote packet
// topic strings are never used directly as ROS topic names.
class RosOutput {
 public:
  RosOutput(rclcpp::Node& node, RvizSettings settings);
  void publish(const Frame& frame, std::uint64_t dropped_frames = 0);
  void connectionState(bool connected, const std::string& detail = {});
  // Use host monotonic elapsed time for stale detection, and host ROS time for
  // Marker lifetime. X5 capture clocks may be monotonic rather than epoch time.
  void expire();
 private:
  using ImagePublisher = rclcpp::Publisher<sensor_msgs::msg::Image>;
  using CloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using TextPublisher = rclcpp::Publisher<std_msgs::msg::String>;
  using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;
  void clearStream(const std::string& key);
  void status(const std::string& state, const std::string& detail);
  rclcpp::Node& node_;
  RvizSettings settings_;
  std::map<std::string, ImagePublisher::SharedPtr> images_;
  std::map<std::string, CloudPublisher::SharedPtr> clouds_;
  std::map<std::string, TextPublisher::SharedPtr> texts_;
  std::map<std::string, MarkerPublisher::SharedPtr> markers_;
  std::map<std::string, std_msgs::msg::Header> headers_;
  std::map<std::string, std::chrono::steady_clock::time_point> seen_;
  TextPublisher::SharedPtr status_, metadata_;
  std::string last_state_, last_detail_;
  std::uint64_t session_ = 0;
  bool connected_ = false, received_frame_ = false;
  std::chrono::steady_clock::time_point last_frame_;
};
} // namespace perception::rviz
