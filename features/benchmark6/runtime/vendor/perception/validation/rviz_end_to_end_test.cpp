#include "rviz_ros_output.hpp"
#include "rviz_transport.hpp"
#include "json.hpp"
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <thread>

namespace rviz = perception::rviz;
namespace {
void check(bool value, const char* error) { if (!value) throw std::runtime_error(error); }
void waitFor(rclcpp::executors::SingleThreadedExecutor& executor, const std::function<bool()>& predicate, const char* error) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  do { executor.spin_some(); if (predicate()) return; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  while (std::chrono::steady_clock::now() < end);
  throw std::runtime_error(error);
}
bool connect(rviz::Client& client, rviz::AsyncPublisher& publisher, rviz::RosOutput& output) {
  bool session_changed=false;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < end) {
    rviz::Frame frame; rviz::ReceiveInfo info;
    const auto result = client.receive(frame,info,200);
    session_changed=session_changed || info.session_changed;
    if (result == rviz::ReceiveResult::Heartbeat) output.connectionState(true,"fixture heartbeat");
    if (publisher.hasClient() && info.connected) return session_changed;
  }
  throw std::runtime_error("TCP client did not connect to native publisher");
}
rviz::Frame receiveFrame(rviz::Client& client, rviz::ReceiveInfo& info) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < end) {
    rviz::Frame frame;
    const auto result = client.receive(frame,info,1000);
    if (result == rviz::ReceiveResult::Frame) return frame;
    if (result == rviz::ReceiveResult::Error) throw std::runtime_error("TCP receive failed: "+info.error);
  }
  throw std::runtime_error("TCP publisher did not deliver a frame");
}
std::string slipJson() {
  return "{\"enabled\":true,\"state\":\"slipping\",\"visual_valid\":true,\"reference_valid\":true,"
      "\"body_calibration_confirmed\":true,\"t0_ns\":8000000000,\"t1_ns\":8100000000,\"dt_s\":0.1,"
      "\"visual_forward_m\":0.03,\"visual_lateral_m\":0,\"reference_forward_m\":0.08,\"reference_lateral_m\":0,\"slip_ratio\":0.625}";
}
int run() {
  auto settings = rviz::defaultSettings();
  for (auto& stream : settings.publish.streams) stream.second = false;
  for (const std::string key : {"input_rgb","disparity_q8","clouds/camera","slip"}) {
    settings.publish.streams[key]=true; settings.rviz.displays[key].enabled=true;
  }
  for (auto& entry : settings.rviz.displays) entry.second.topic="/perception_rviz/e2e/"+entry.first;
  settings.rviz.fixed_frame=settings.rviz.body_frame="base_link";
  settings.rviz.motion_arrow=true; settings.rviz.marker_lifetime_ms=100;
  auto node=std::make_shared<rclcpp::Node>("rviz_end_to_end_test");
  rviz::RosOutput output(*node,settings);
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  std::optional<sensor_msgs::msg::Image> image,q8;
  std::optional<sensor_msgs::msg::PointCloud2> cloud;
  std::optional<visualization_msgs::msg::MarkerArray> markers;
  std::string metadata,status,raw_slip;
  auto sensor_qos=rclcpp::QoS(1).best_effort();
  auto image_sub=node->create_subscription<sensor_msgs::msg::Image>("/perception_rviz/e2e/input_rgb",sensor_qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr v){ image=*v; });
  auto q8_sub=node->create_subscription<sensor_msgs::msg::Image>("/perception_rviz/e2e/disparity_q8",sensor_qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr v){ q8=*v; });
  auto cloud_sub=node->create_subscription<sensor_msgs::msg::PointCloud2>("/perception_rviz/e2e/clouds/camera",sensor_qos,
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr v){ cloud=*v; });
  auto marker_sub=node->create_subscription<visualization_msgs::msg::MarkerArray>("/perception_rviz/e2e/slip",sensor_qos,
      [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr v){ markers=*v; });
  auto raw_sub=node->create_subscription<std_msgs::msg::String>("/perception_rviz/e2e/slip/json",sensor_qos,
      [&](std_msgs::msg::String::ConstSharedPtr v){ raw_slip=v->data; });
  const auto diagnostics_qos=rclcpp::QoS(1).reliable().transient_local();
  auto meta_sub=node->create_subscription<std_msgs::msg::String>("/perception_rviz/meta",diagnostics_qos,
      [&](std_msgs::msg::String::ConstSharedPtr v){ metadata=v->data; });
  auto status_sub=node->create_subscription<std_msgs::msg::String>("/perception_rviz/status",diagnostics_qos,
      [&](std_msgs::msg::String::ConstSharedPtr v){ status=v->data; });
  waitFor(executor,[&]{return image_sub->get_publisher_count() && q8_sub->get_publisher_count() &&
      cloud_sub->get_publisher_count() && marker_sub->get_publisher_count() && raw_sub->get_publisher_count();},"ROS subscriptions not discovered");

  rviz::PublisherOptions publisher_options; publisher_options.port=0; publisher_options.heartbeat_interval_ms=25;
  rviz::AsyncPublisher publisher(publisher_options); std::string error;
  check(publisher.start(&error),"native TCP publisher failed to start");
  rviz::ClientOptions client_options; client_options.port=publisher.port(); client_options.reconnect_delay_ms=10;
  rviz::Client client(client_options); connect(client,publisher,output);
  rviz::Frame source; source.frame_number=9007199254740993ULL; source.capture_timestamp_ns=8123456789ULL;
  rviz::Image rgb; rgb.width=2; rgb.height=1; rgb.step=6; rgb.data={1,2,3,241,128,9};
  rviz::Image disparity; disparity.width=2; disparity.height=1; disparity.step=4;
  disparity.encoding=rviz::ImageEncoding::Mono16; disparity.data={0,1,128,2};
  rviz::PointCloud points; points.points={{1,-2,3,0x001234abU,0.75F,-7}};
  source.packets={{"input_rgb","camera_optical_frame",rgb},{"disparity_q8","camera_optical_frame",disparity},
      {"clouds/camera","camera_optical_frame",points},{"slip","camera_optical_frame",rviz::Text{slipJson()}}};
  check(publisher.publish(source,&error),"native TCP publisher rejected fixture");
  rviz::ReceiveInfo info; const auto decoded=receiveFrame(client,info);
  check(decoded.frame_number==source.frame_number && decoded.capture_timestamp_ns==source.capture_timestamp_ns &&
      decoded.packets.size()==4 && decoded.session_id!=0 && decoded.sequence>0 && info.connected,"TCP decoded frame identity changed");
  output.publish(decoded,info.dropped_frames);
  waitFor(executor,[&]{return image && q8 && cloud && markers && raw_slip==slipJson() && !metadata.empty();},"decoded TCP frame did not reach ROS subscribers");
  check(image->data==rgb.data && image->encoding=="rgb8" && image->header.frame_id=="camera_optical_frame" &&
      image->header.stamp.sec==8 && image->header.stamp.nanosec==123456789,"TCP-to-ROS image pixels/header changed");
  check(q8->data==disparity.data && q8->encoding=="16UC1" && q8->step==4 && !q8->is_bigendian,"TCP-to-ROS Q8 data changed");
  check(cloud->header==image->header && cloud->width==1 && cloud->point_step==24 &&
      cloud->data.at(12)==0xab && cloud->data.at(13)==0x34 && cloud->data.at(14)==0x12 &&
      cloud->fields.at(4).name=="confidence" && cloud->fields.at(5).name=="label","TCP-to-ROS cloud data changed");
  unsigned arrows=0; bool measured_status=false;
  for(const auto& marker:markers->markers) if(marker.action==visualization_msgs::msg::Marker::ADD) {
    if(marker.type==visualization_msgs::msg::Marker::ARROW) ++arrows;
    if(marker.text.find("slip: slipping")!=std::string::npos && marker.text.find("ratio=0.625")!=std::string::npos) measured_status=true;
  }
  check(arrows==2 && measured_status,"TCP-to-ROS slip status or measured arrows missing");
  const auto meta=r33::json::JsonParser(metadata,"e2e.meta").parse();
  check(r33::json::required(meta,"frame_number").text=="9007199254740993" &&
      r33::json::required(meta,"capture_timestamp_ns").text=="8123456789","TCP-to-ROS numeric identity lost");

  const auto old_session=decoded.session_id; const auto server_port=publisher.port(); publisher.stop();
  bool disconnected=false;
  for(int attempt=0;attempt<8 && !disconnected;++attempt) {
    rviz::Frame frame; rviz::ReceiveInfo event;
    const auto result=client.receive(frame,event,200);
    if(result==rviz::ReceiveResult::Error || (result==rviz::ReceiveResult::Timeout && !event.connected)) {
      output.connectionState(false,event.error); disconnected=true;
    }
  }
  check(disconnected,"TCP disconnect was not reported");
  waitFor(executor,[&]{return cloud && cloud->width==0 && status.find("\"state\":\"disconnected\"")!=std::string::npos;},
      "TCP disconnect did not clear stale ROS cloud");
  publisher_options.port=server_port;
  rviz::AsyncPublisher restarted(publisher_options); check(restarted.start(&error),"TCP publisher restart failed");
  const bool new_session_heartbeat=connect(client,restarted,output);
  ++source.frame_number; source.capture_timestamp_ns+=100000000;
  check(restarted.publish(source,&error),"restarted TCP publisher rejected frame");
  const auto resumed=receiveFrame(client,info);
  check(resumed.session_id!=old_session && (info.session_changed || new_session_heartbeat),"publisher restart did not establish a new session");
  output.publish(resumed,info.dropped_frames);
  waitFor(executor,[&]{return cloud && cloud->width==1 && image && image->header.stamp.nanosec==223456789;},
      "TCP reconnect did not resume fresh ROS output");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  rviz::Frame unused; rviz::ReceiveInfo heartbeat;
  check(client.receive(unused,heartbeat,300)==rviz::ReceiveResult::Heartbeat && heartbeat.connected,
      "native TCP heartbeat did not preserve connection");
  output.expire();
  waitFor(executor,[&]{return cloud && cloud->width==0 && status.find("\"state\":\"stale\"")!=std::string::npos;},
      "heartbeat-only connection must not keep stale source data visible");
  client.stop(); restarted.stop();
  std::cout<<"Native TCP -> decoded frame -> ROS Image/Q8/PointCloud/slip Marker subscribers passed; disconnect, new-session reconnect and stale heartbeat cleanup passed\n";
  return 0;
}
} // namespace
int main(int argc,char** argv) {
  try { rclcpp::init(argc,argv); const int code=run(); rclcpp::shutdown(); return code; }
  catch(const std::exception& error) { std::cerr<<"RViz end-to-end test failed: "<<error.what()<<'\n'; if(rclcpp::ok())rclcpp::shutdown(); return 1; }
}
