#include "rviz_ros_output.hpp"
#include "json.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace rviz = perception::rviz;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template<class Function> void rejects(Function function, const char* message) {
  bool rejected = false; try { function(); } catch (const std::exception&) { rejected = true; }
  check(rejected, message);
}
template<class Message> Message roundtrip(const Message& message) {
  rclcpp::Serialization<Message> serializer; rclcpp::SerializedMessage bytes;
  serializer.serialize_message(&message, &bytes);
  Message decoded; serializer.deserialize_message(&bytes, &decoded); return decoded;
}
std::uint32_t little32(const std::vector<std::uint8_t>& data, std::size_t at) {
  return std::uint32_t(data.at(at)) | (std::uint32_t(data.at(at+1)) << 8U) |
      (std::uint32_t(data.at(at+2)) << 16U) | (std::uint32_t(data.at(at+3)) << 24U);
}
const visualization_msgs::msg::Marker* added(const visualization_msgs::msg::MarkerArray& array, int id) {
  for (const auto& marker : array.markers)
    if (marker.id == id && marker.action == visualization_msgs::msg::Marker::ADD) return &marker;
  return nullptr;
}
std::string slip(const char* state, bool visual, bool reference, bool calibrated, const char* ratio) {
  return std::string("{\"enabled\":true,\"state\":\"") + state + "\",\"visual_valid\":" + (visual ? "true" : "false") +
      ",\"reference_valid\":" + (reference ? "true" : "false") + ",\"body_calibration_confirmed\":" + (calibrated ? "true" : "false") +
      ",\"t0_ns\":9007199254740993,\"t1_ns\":9007199354740993,\"dt_s\":0.1,\"visual_forward_m\":0.03,\"visual_lateral_m\":-0.01,"
      "\"reference_forward_m\":0.08,\"reference_lateral_m\":0,\"slip_ratio\":" + ratio + '}';
}
void messageTests() {
  rviz::Frame frame; frame.frame_number = 9007199254740993ULL;
  frame.capture_timestamp_ns = 1288123456789ULL;
  rviz::Packet packet; packet.frame_id = "camera_optical_frame";
  const auto header = rviz::sourceHeader(frame, packet);
  check(header.stamp.sec == 1288 && header.stamp.nanosec == 123456789, "capture timestamp changed");
  check(header.frame_id == "camera_optical_frame", "source frame_id changed");
  frame.capture_timestamp_ns = 0;
  check(rviz::sourceHeader(frame, packet).stamp.sec == 0, "zero source timestamp must not become receive time");
  frame.capture_timestamp_ns = std::numeric_limits<std::uint64_t>::max();
  rejects([&] { rviz::sourceHeader(frame, packet); }, "unrepresentable timestamp accepted");
  packet.frame_id.clear(); rejects([&] { rviz::sourceHeader(frame, packet); }, "empty frame accepted");

  rviz::Image image; image.width = 2; image.height = 2; image.step = 6;
  image.encoding = rviz::ImageEncoding::Mono16;
  image.data = {0,1, 0,2, 0x55,0x66, 0xff,0x7f, 0,4, 0x77,0x88};
  const auto ros_image = roundtrip(rviz::imageMessage(image, header));
  check(ros_image.encoding == "16UC1" && !ros_image.is_bigendian && ros_image.step == 6, "raw Q8 encoding/stride lost");
  check(ros_image.data == image.data && ros_image.header == header, "Q8 bytes or capture header lost in serialization");
  image.data[0] = 42; check(ros_image.data[0] == 0, "ROS Image borrowed a source buffer");
  image.data.pop_back(); rejects([&] { rviz::imageMessage(image, header); }, "truncated image accepted");
  image.encoding = rviz::ImageEncoding::Rgb8; image.width = 1; image.height = 1; image.step = 3;
  image.data = {17,91,220};
  check(roundtrip(rviz::imageMessage(image, header)).encoding == "rgb8", "RGB image encoding lost");
  image.encoding = rviz::ImageEncoding::Mono8; image.step = 1; image.data = {42};
  check(rviz::imageMessage(image, header).encoding == "mono8", "mono8 unsupported");

  rviz::PointCloud cloud;
  cloud.points = {{1,-2,3,0x001234abU,0.75F,-7}, {std::numeric_limits<float>::quiet_NaN(),0,0,0,0,0}};
  const auto ros_cloud = roundtrip(rviz::cloudMessage(cloud, header));
  check(ros_cloud.header == header && ros_cloud.width == 2 && ros_cloud.height == 1, "cloud identity lost");
  check(!ros_cloud.is_dense && !ros_cloud.is_bigendian && ros_cloud.point_step == 24 && ros_cloud.row_step == 48,
      "cloud layout/density incorrect");
  check(ros_cloud.fields.size() == 6 && ros_cloud.fields[3].name == "rgb" &&
      ros_cloud.fields[3].datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      ros_cloud.fields[4].name == "confidence" && ros_cloud.fields[5].name == "label" &&
      ros_cloud.fields[5].datatype == sensor_msgs::msg::PointField::INT32, "cloud fields lost");
  check(little32(ros_cloud.data,0) == 0x3f800000U && little32(ros_cloud.data,4) == 0xc0000000U &&
      little32(ros_cloud.data,12) == 0x001234abU && little32(ros_cloud.data,16) == 0x3f400000U &&
      little32(ros_cloud.data,20) == std::uint32_t(-7), "cloud payload bytes are incorrect");

  rviz::ViewSettings view; view.fixed_frame = view.body_frame = "base_link"; view.motion_arrow = true;
  const rclcpp::Time display_time(1700000000,123, RCL_SYSTEM_TIME);
  const auto valid = roundtrip(rviz::slipMarkers(slip("slipping",true,true,true,"0.625"),view,display_time));
  const auto* status = added(valid,0); const auto* visual = added(valid,1); const auto* wheel = added(valid,2);
  check(status && visual && wheel, "valid measured interval should render status and two arrows");
  check(status->text.find("ratio=0.625") != std::string::npos &&
      status->text.find("9007199254740993") != std::string::npos, "ratio or exact 64-bit interval timestamp lost");
  check(status->header.stamp.sec == 1700000000 && status->lifetime.sec == 1 && status->lifetime.nanosec == 500000000,
      "marker lifetime must use Ubuntu ROS display time");
  check(visual->header.frame_id == "base_link" && visual->points.size() == 2 &&
      std::abs(visual->points[1].x-0.03) < 1e-9 && std::abs(visual->points[1].y+0.01) < 1e-9,
      "visual local displacement vector changed");
  check(wheel->color != visual->color, "visual and wheel arrows need different colors");
  const auto missing = rviz::slipMarkers(slip("normal",true,false,true,"0"),view,display_time);
  check(added(missing,0) && added(missing,0)->text.find("slip: unavailable") != std::string::npos &&
      added(missing,0)->text.find("ratio=") == std::string::npos && !added(missing,2),
      "missing reference falsely rendered normal, ratio or reference arrow");
  const auto uncalibrated = rviz::slipMarkers(slip("normal",true,true,false,"0"),view,display_time);
  check(!added(uncalibrated,1) && added(uncalibrated,0)->text.find("slip: unavailable") != std::string::npos,
      "unconfirmed camera/body transform produced a body visual arrow");
  view.fixed_frame = "camera_optical_frame";
  const auto mismatched = rviz::slipMarkers(slip("slipping",true,true,true,"0.625"),view,display_time);
  check(!added(mismatched,1) && !added(mismatched,2) && added(mismatched,0)->header.frame_id == view.fixed_frame,
      "body motion was rendered in camera frame without a real TF");
  view.fixed_frame = view.body_frame;
  const auto bad_json = rviz::slipMarkers("{\"state\":\"normal\",\"state\":\"normal\"}",view,display_time);
  check(added(bad_json,0) && added(bad_json,0)->text.find("invalid JSON") != std::string::npos && !added(bad_json,1),
      "malformed/duplicate JSON should clear arrows and display invalid state");
}
void waitFor(rclcpp::executors::SingleThreadedExecutor& executor, const std::function<bool()>& predicate, const char* error) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  do { executor.spin_some(); if (predicate()) return; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  while (std::chrono::steady_clock::now() < end);
  throw std::runtime_error(error);
}
void publisherTests() {
  auto settings = rviz::defaultSettings(); settings.rviz.marker_lifetime_ms = 20;
  for (auto& stream : settings.publish.streams) stream.second = false;
  for (const std::string key : {"input_rgb", "clouds/camera", "slip"}) {
    settings.publish.streams[key] = true; settings.rviz.displays[key].enabled = true;
  }
  settings.rviz.displays["input_rgb"].topic = "/perception_rviz/test/input";
  settings.rviz.displays["clouds/camera"].topic = "/perception_rviz/test/cloud";
  settings.rviz.displays["slip"].topic = "/perception_rviz/test/slip";
  auto node = std::make_shared<rclcpp::Node>("rviz_ros_output_test");
  rviz::RosOutput output(*node,settings);
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::PointCloud2> cloud;
  std::optional<visualization_msgs::msg::MarkerArray> markers;
  std::string metadata, state, original;
  auto iq = rclcpp::QoS(1).best_effort();
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>("/perception_rviz/test/input",iq,
      [&](sensor_msgs::msg::Image::ConstSharedPtr v) { image = *v; });
  auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/perception_rviz/test/cloud",iq,
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr v) { cloud = *v; });
  auto marker_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>("/perception_rviz/test/slip",iq,
      [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr v) { markers = *v; });
  auto original_sub = node->create_subscription<std_msgs::msg::String>("/perception_rviz/test/slip/json",iq,
      [&](std_msgs::msg::String::ConstSharedPtr v) { original = v->data; });
  auto mq = rclcpp::QoS(1).reliable().transient_local();
  auto meta_sub = node->create_subscription<std_msgs::msg::String>("/perception_rviz/meta",mq,
      [&](std_msgs::msg::String::ConstSharedPtr v) { metadata = v->data; });
  auto status_sub = node->create_subscription<std_msgs::msg::String>("/perception_rviz/status",mq,
      [&](std_msgs::msg::String::ConstSharedPtr v) { state = v->data; });
  waitFor(executor,[&] { return image_sub->get_publisher_count() && cloud_sub->get_publisher_count() &&
      marker_sub->get_publisher_count() && meta_sub->get_publisher_count(); },"ROS publisher discovery failed");
  rviz::Frame frame; frame.session_id=17; frame.sequence=4; frame.frame_number=9007199254740993ULL;
  frame.capture_timestamp_ns=3123456789ULL; frame.send_timestamp_ns=6000000001ULL;
  rviz::Image source; source.width=source.height=1; source.step=3; source.data={1,2,3};
  rviz::PointCloud points; points.points.push_back({1,2,3,0x00ff0000U,0.5F,2});
  const auto source_slip=slip("unavailable",true,false,true,"null");
  frame.packets = {{"input_rgb","camera_optical_frame",source}, {"clouds/camera","camera_optical_frame",points},
      {"slip","camera_optical_frame",rviz::Text{source_slip}}, {"/untrusted/topic","camera_optical_frame",source},
      {"clouds/camera","camera_optical_frame",source}};
  output.publish(frame,3);
  waitFor(executor,[&] { return image && cloud && markers && !metadata.empty() && original==source_slip; },"ROS frame output missing");
  check(image->header.stamp.sec==3 && image->header.stamp.nanosec==123456789 && cloud->header==image->header,
      "receiver replaced source capture header");
  auto json = r33::json::JsonParser(metadata,"meta").parse();
  check(r33::json::required(json,"frame_number").text=="9007199254740993" &&
      r33::json::required(json,"accepted_packets").text=="3" && r33::json::required(json,"rejected_packets").text=="2" &&
      r33::json::required(json,"dropped_frames").text=="3", "allowlist or numeric frame metadata lost");
  check(node->count_publishers("/untrusted/topic")==0,"remote topic created an arbitrary ROS publisher");
  std::this_thread::sleep_for(std::chrono::milliseconds(25)); output.expire();
  waitFor(executor,[&] { return cloud && cloud->width==0 && markers && !markers->markers.empty() &&
      markers->markers[0].action==visualization_msgs::msg::Marker::DELETEALL && state.find("\"state\":\"stale\"")!=std::string::npos; },
      "stale data did not clear cloud/markers and publish stale state");
  output.connectionState(false,"fixture disconnect");
  waitFor(executor,[&] { return state.find("\"state\":\"disconnected\"")!=std::string::npos; }, "disconnect state missing");
  frame.session_id=18; frame.sequence=1; output.publish(frame);
  waitFor(executor,[&] { return cloud && cloud->width==1 && metadata.find("\"session_id\":18")!=std::string::npos; },
      "new publisher session did not resume output");
}
} // namespace
int main(int argc,char** argv) {
  try {
    rclcpp::init(argc,argv); messageTests(); publisherTests(); rclcpp::shutdown();
    std::cout << "RViz ROS output contract passed: stamps, serialization, allowlist, slip validity, stale cleanup and reconnect\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RViz ROS output test failed: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown(); return 1;
  }
}
