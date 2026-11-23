#include "rviz_ros_output.hpp"
#include "rviz_transport.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>

namespace rviz = perception::rviz;
namespace {
struct Options {
  std::filesystem::path config = "rviz.json";
  std::string host = "127.0.0.1";
  std::optional<std::uint16_t> port;
  bool check = false, help = false;
};
Options options(const std::vector<std::string>& args) {
  Options out; std::set<std::string> seen;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const auto& key = args[i];
    if (!seen.insert(key).second) throw std::invalid_argument("Duplicate option: " + key);
    if (key == "--help" || key == "-h") { out.help = true; continue; }
    if (key == "--check") { out.check = true; continue; }
    if ((key != "--config" && key != "--host" && key != "--port") || i + 1 == args.size())
      throw std::invalid_argument("Expected --config rviz.json [--host 127.0.0.1] [--port 18765] [--check]");
    const auto value = args[++i]; if (value.empty()) throw std::invalid_argument("Empty option: " + key);
    if (key == "--config") out.config = std::filesystem::u8path(value);
    else if (key == "--host") out.host = value;
    else {
      if (value.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("Port must be an integer in 1..65535");
      const auto port = std::stoul(value);
      if (!port || port > 65535) throw std::invalid_argument("Port outside 1..65535");
      out.port = std::uint16_t(port);
    }
  }
  return out;
}
struct Received {
  std::optional<rviz::Frame> latest;
  bool connected = false;
  bool reset_required = false;
  std::string detail;
  std::uint64_t revision = 0, dropped_frames = 0;
};
int run(const Options& args) {
  auto settings = rviz::loadSettings(args.config);
  const auto port = args.port.value_or(settings.transport.local_port);
  if (args.check) {
    std::cout << "RViz receiver configuration valid; tcp=" << args.host << ':' << port
              << " fixed_frame=" << settings.rviz.fixed_frame << '\n';
    return 0;
  }
  if (!settings.enabled) { std::cout << "RViz is disabled by JSON configuration\n"; return 0; }
  rviz::ClientOptions client_options;
  client_options.host = args.host; client_options.port = port;
  client_options.connect_timeout_ms = settings.transport.connect_timeout_ms;
  client_options.frame_timeout_ms = std::max(2000, settings.transport.connect_timeout_ms);
  client_options.reconnect_delay_ms = settings.transport.reconnect_ms;
  client_options.limits.max_body_bytes = std::uint32_t(settings.transport.max_frame_bytes - rviz::kWireHeaderSize);
  client_options.limits.max_image_bytes = std::min(client_options.limits.max_body_bytes, 64U * 1024U * 1024U);
  client_options.limits.max_points = std::uint32_t(settings.publish.max_points);
  rviz::Client client(client_options);
  auto node = std::make_shared<rclcpp::Node>("perception_rviz_receiver");
  rviz::RosOutput output(*node, settings);
  RCLCPP_INFO(node->get_logger(), "Receiving X5 TCP bundle at %s:%u; local ROS frame=%s",
      args.host.c_str(), unsigned(port), settings.rviz.fixed_frame.c_str());
  std::mutex mutex; Received pending; std::atomic<bool> stop{false};
  std::thread worker([&] {
    try {
      while (!stop.load()) {
        rviz::Frame frame; rviz::ReceiveInfo info;
        const auto result = client.receive(frame, info, client_options.frame_timeout_ms);
        if (result == rviz::ReceiveResult::Stopped) break;
        std::lock_guard<std::mutex> lock(mutex);
        if (result == rviz::ReceiveResult::Frame) {
          // One owning latest slot: the GUI cannot build an unbounded frame queue.
          if (pending.latest) ++pending.dropped_frames;
          pending.latest = std::move(frame); pending.dropped_frames += info.dropped_frames;
          pending.reset_required = pending.reset_required || info.session_changed;
          pending.connected = true; pending.detail = "transport connected"; ++pending.revision;
        } else if (result == rviz::ReceiveResult::Heartbeat) {
          if (!pending.connected || info.session_changed) {
            pending.reset_required = pending.reset_required || info.session_changed;
            pending.connected = true; pending.detail = "heartbeat; waiting for source frame"; ++pending.revision;
          }
        } else if (result == rviz::ReceiveResult::Error ||
            (result == rviz::ReceiveResult::Timeout && !info.connected)) {
          pending.latest.reset(); pending.dropped_frames = 0; pending.connected = false;
          pending.reset_required = true;
          pending.detail = info.error.empty() ? "transport disconnected" : info.error; ++pending.revision;
        }
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(mutex);
      pending.latest.reset(); pending.connected = false; pending.detail = error.what(); ++pending.revision;
      pending.reset_required = true;
    }
  });
  std::uint64_t revision = 0;
  try {
    while (rclcpp::ok()) {
      Received ready;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (pending.revision != revision) {
          ready.connected = pending.connected; ready.detail = pending.detail;
          ready.reset_required = pending.reset_required; pending.reset_required = false;
          ready.revision = pending.revision; revision = pending.revision;
          ready.latest = std::move(pending.latest); pending.latest.reset();
          ready.dropped_frames = pending.dropped_frames; pending.dropped_frames = 0;
        }
      }
      if (ready.revision) {
        // A rapid disconnect/reconnect can coalesce between GUI ticks. Retain
        // the reset edge even when the newest event is already connected.
        if (ready.reset_required && ready.connected) output.connectionState(false,"publisher session changed or transport reconnected");
        output.connectionState(ready.connected, ready.detail);
        if (ready.latest) output.publish(*ready.latest, ready.dropped_frames);
      }
      output.expire(); rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } catch (...) {
    stop = true; client.stop(); worker.join(); throw;
  }
  stop = true; client.stop(); worker.join(); return 0;
}
} // namespace
int main(int argc, char** argv) {
  try {
    const auto args = options(rclcpp::remove_ros_arguments(argc, argv));
    if (args.help) {
      std::cout << "r36_rviz_receiver --config rviz.json [--host 127.0.0.1] [--port 18765] [--check]\n"
                << "Ubuntu ROS 2 receiver for X5 TCP frames through adb forward. No camera/BPU SDK required.\n";
      return 0;
    }
    rclcpp::init(argc, argv);
    const auto result = run(args); rclcpp::shutdown(); return result;
  } catch (const std::exception& error) {
    std::cerr << "RViz receiver: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown(); return 1;
  }
}
