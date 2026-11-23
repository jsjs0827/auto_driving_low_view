#pragma once
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>

namespace perception::rviz {
struct TransportSettings {
  std::string bind_address = "127.0.0.1";
  std::uint16_t local_port = 18765;
  std::uint16_t remote_port = 8765;
  std::string serial;
  std::string adb_executable = "adb";
  int connect_timeout_ms = 3000;
  int reconnect_ms = 1000;
  int send_timeout_ms = 500;
  int max_frame_bytes = 16 * 1024 * 1024;
};
struct PublishSettings {
  double max_fps = 10.0;
  int every_n = 2;
  int max_points = 60000;
  int point_stride = 2;
  double disparity_min_px = 0.0;
  double disparity_max_px = 144.0;
  double overlay_alpha = 0.35;
  std::map<std::string, bool> streams;
};
struct DisplaySettings {
  bool enabled = false;
  std::string topic;
  std::string frame_id;
  std::string style = "Points";
  std::string color_transformer = "RGB8";
  double point_size = 2.0;
  std::string reliability = "best_effort";
  std::string durability = "volatile";
  int depth = 1;
};
struct ViewSettings {
  std::string fixed_frame = "camera_optical_frame";
  std::string body_frame = "base_link";
  int frame_rate = 15;
  std::string rviz_executable = "rviz2";
  int marker_lifetime_ms = 1500;
  bool motion_arrow = false;
  std::map<std::string, DisplaySettings> displays;
};
struct RvizSettings {
  bool enabled = true;
  TransportSettings transport;
  PublishSettings publish;
  ViewSettings rviz;
  std::filesystem::path source_path;
};
// Stream keys are the transport allowlist, never arbitrary remote ROS topics.
RvizSettings defaultSettings();
RvizSettings loadSettings(const std::filesystem::path& path);
std::string settingsJson(const RvizSettings& settings);
std::string rvizConfig(const RvizSettings& settings);
const DisplaySettings& displaySettings(const RvizSettings& settings, const std::string& stream);
bool streamEnabled(const RvizSettings& settings, const std::string& stream);
} // namespace perception::rviz
