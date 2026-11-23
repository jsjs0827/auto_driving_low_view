#include "rviz_settings.hpp"
#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace perception::rviz {
namespace {
using Json = r33::json::JsonValue;
namespace json = r33::json;
constexpr const char* streams[] = {
  "input_rgb", "overlay", "mask_rgb", "disparity_rgb", "disparity_q8",
  "clouds/camera", "clouds/robot", "clouds/ground", "clouds/obstacle", "clouds/semantic",
  "maps/pcl", "maps/ai", "maps/fusion", "result", "slip"
};
bool isCloud(const std::string& stream) { return stream.rfind("clouds/", 0) == 0; }
bool isMarker(const std::string& stream) { return stream == "slip" || stream == "result"; }
void knownStreams(const Json& value) {
  if (value.kind != Json::Kind::Object) throw std::invalid_argument("Stream settings must be an object");
  for (const auto& entry : value.object) {
    if (std::find(std::begin(streams), std::end(streams), entry.first) == std::end(streams))
      throw std::invalid_argument("Unknown RViz stream: " + entry.first);
  }
}
std::string cleanString(const Json& value, const char* name, bool empty = false) {
  if (value.kind != Json::Kind::String || (!empty && value.text.empty()) || value.text.size() > 4096 ||
      std::any_of(value.text.begin(), value.text.end(), [](unsigned char c) { return c < 32 || c == 127; }))
    throw std::invalid_argument(std::string("Invalid RViz string: ") + name);
  return value.text;
}
std::string choice(const Json& value, const char* name, std::initializer_list<const char*> allowed) {
  auto result = cleanString(value, name);
  if (std::find(allowed.begin(), allowed.end(), result) == allowed.end())
    throw std::invalid_argument(std::string("Unsupported RViz ") + name + ": " + result);
  return result;
}
std::string topic(const Json& value) {
  auto result = cleanString(value, "topic");
  if (result.size() > 255 || !std::regex_match(result, std::regex(R"((/[A-Za-z_][A-Za-z0-9_]*)+)")))
    throw std::invalid_argument("RViz topic must be an absolute ROS topic: " + result);
  return result;
}
std::string frame(const Json& value) {
  auto result = cleanString(value, "frame_id");
  if (result.size() > 255 || !std::regex_match(result, std::regex(R"([A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*)")))
    throw std::invalid_argument("Invalid RViz frame_id: " + result);
  return result;
}
double bounded(const Json& value, double low, double high, const char* name) {
  auto result = json::number(value);
  if (result < low || result > high) throw std::invalid_argument(std::string("RViz value outside range: ") + name);
  return result;
}
std::string executable(const Json& value, const std::filesystem::path& base, const char* name) {
  auto result = cleanString(value, name);
  if (result.front() == '-') throw std::invalid_argument(std::string("Invalid executable: ") + name);
  // A bare executable uses PATH. Every path with a separator is relative to the JSON directory.
  auto path = std::filesystem::u8path(result);
  if (path.is_absolute()) return path.lexically_normal().u8string();
  if (result.find('/') != std::string::npos || result.find('\\') != std::string::npos)
    return (base / path).lexically_normal().u8string();
  return result;
}
void ipv4(const std::string& address) {
  std::istringstream input(address);
  for (int i = 0; i < 4; ++i) {
    std::string octet;
    if (!std::getline(input, octet, '.') || octet.empty() || octet.size() > 3 ||
        !std::all_of(octet.begin(), octet.end(), [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
        (octet.size() > 1 && octet[0] == '0') || std::stoi(octet) > 255)
      throw std::invalid_argument("transport.bind_address must be an IPv4 address");
  }
  if (input.peek() != std::char_traits<char>::eof() || address.back() == '.')
    throw std::invalid_argument("transport.bind_address must be an IPv4 address");
}
std::string boolean(bool value) { return value ? "true" : "false"; }
void qos(std::ostream& out, const DisplaySettings& display, const std::string& indentation) {
  out << indentation << "Topic:\n"
      << indentation << "  Value: " << json::quote(display.topic) << '\n'
      << indentation << "  Reliability Policy: " << (display.reliability == "best_effort" ? "Best Effort" : "Reliable") << '\n'
      << indentation << "  Durability Policy: " << (display.durability == "volatile" ? "Volatile" : "Transient Local") << '\n'
      << indentation << "  History Policy: Keep Last\n"
      << indentation << "  Depth: " << display.depth << '\n';
}
} // namespace

RvizSettings defaultSettings() {
  RvizSettings settings;
  for (const auto* stream : streams) {
    const std::string key(stream);
    const bool enabled = key == "input_rgb" || key == "overlay" || key == "mask_rgb" ||
                         key == "disparity_rgb" || key == "clouds/camera" || key == "slip";
    settings.publish.streams.emplace(key, enabled);
    DisplaySettings display;
    display.enabled = enabled;
    display.topic = "/perception_rviz/" + key;
    display.frame_id = isCloud(key) && key != "clouds/camera" ? settings.rviz.body_frame : settings.rviz.fixed_frame;
    settings.rviz.displays.emplace(key, std::move(display));
  }
  return settings;
}

RvizSettings loadSettings(const std::filesystem::path& path) {
  auto settings = defaultSettings();
  settings.source_path = std::filesystem::absolute(path).lexically_normal();
  const auto base = settings.source_path.parent_path();
  const auto value = json::load(settings.source_path.u8string());
  json::fields(value, {"schema_version", "enabled", "transport", "publish", "rviz"});
  if (json::integer(json::required(value, "schema_version"), 1, 1) != 1)
    throw std::invalid_argument("Unsupported RViz schema_version");
  if (const auto* item = json::optional(value, "enabled")) settings.enabled = json::boolean(*item);
  if (const auto* transport = json::optional(value, "transport")) {
    json::fields(*transport, {"bind_address", "local_port", "remote_port", "serial", "adb_executable",
                            "connect_timeout_ms", "reconnect_ms", "send_timeout_ms", "max_frame_bytes"});
    if (const auto* item = json::optional(*transport, "bind_address")) settings.transport.bind_address = cleanString(*item, "bind_address");
    if (const auto* item = json::optional(*transport, "local_port")) settings.transport.local_port = static_cast<std::uint16_t>(json::integer(*item, 1024, 65535));
    if (const auto* item = json::optional(*transport, "remote_port")) settings.transport.remote_port = static_cast<std::uint16_t>(json::integer(*item, 1024, 65535));
    if (const auto* item = json::optional(*transport, "serial")) settings.transport.serial = cleanString(*item, "serial", true);
    if (const auto* item = json::optional(*transport, "adb_executable")) settings.transport.adb_executable = executable(*item, base, "adb_executable");
    if (const auto* item = json::optional(*transport, "connect_timeout_ms")) settings.transport.connect_timeout_ms = json::integer(*item, 100, 60000);
    if (const auto* item = json::optional(*transport, "reconnect_ms")) settings.transport.reconnect_ms = json::integer(*item, 100, 60000);
    if (const auto* item = json::optional(*transport, "send_timeout_ms")) settings.transport.send_timeout_ms = json::integer(*item, 10, 60000);
    if (const auto* item = json::optional(*transport, "max_frame_bytes")) settings.transport.max_frame_bytes = json::integer(*item, 1024, 64 * 1024 * 1024);
  }
  ipv4(settings.transport.bind_address);
  if (settings.transport.serial.size() > 255 || (!settings.transport.serial.empty() &&
      (settings.transport.serial.front() == '-' || !std::regex_match(settings.transport.serial, std::regex(R"([A-Za-z0-9_.:/-]+)")))))
    throw std::invalid_argument("Invalid ADB device serial");
  if (const auto* publish = json::optional(value, "publish")) {
    json::fields(*publish, {"max_fps", "every_n", "max_points", "point_stride", "disparity_min_px", "disparity_max_px", "overlay_alpha", "streams"});
    if (const auto* item = json::optional(*publish, "max_fps")) settings.publish.max_fps = bounded(*item, 0.1, 120.0, "max_fps");
    if (const auto* item = json::optional(*publish, "every_n")) settings.publish.every_n = json::integer(*item, 1, 1000);
    if (const auto* item = json::optional(*publish, "max_points")) settings.publish.max_points = json::integer(*item, 1, 1000000);
    if (const auto* item = json::optional(*publish, "point_stride")) settings.publish.point_stride = json::integer(*item, 1, 10000);
    if (const auto* item = json::optional(*publish, "disparity_min_px")) settings.publish.disparity_min_px = bounded(*item, 0.0, 65535.0 / 256.0, "disparity_min_px");
    if (const auto* item = json::optional(*publish, "disparity_max_px")) settings.publish.disparity_max_px = bounded(*item, 0.0, 65535.0 / 256.0, "disparity_max_px");
    if (const auto* item = json::optional(*publish, "overlay_alpha")) settings.publish.overlay_alpha = bounded(*item, 0.0, 1.0, "overlay_alpha");
    if (const auto* selected = json::optional(*publish, "streams")) {
      knownStreams(*selected);
      for (const auto& item : selected->object) settings.publish.streams.at(item.first) = json::boolean(item.second);
    }
  }
  if (settings.publish.disparity_max_px <= settings.publish.disparity_min_px)
    throw std::invalid_argument("publish.disparity_max_px must exceed disparity_min_px");
  if (const auto* rviz = json::optional(value, "rviz")) {
    json::fields(*rviz, {"fixed_frame", "body_frame", "frame_rate", "rviz_executable", "marker_lifetime_ms", "motion_arrow", "displays"});
    if (const auto* item = json::optional(*rviz, "fixed_frame")) settings.rviz.fixed_frame = frame(*item);
    if (const auto* item = json::optional(*rviz, "body_frame")) settings.rviz.body_frame = frame(*item);
    if (const auto* item = json::optional(*rviz, "frame_rate")) settings.rviz.frame_rate = json::integer(*item, 1, 120);
    if (const auto* item = json::optional(*rviz, "rviz_executable")) settings.rviz.rviz_executable = executable(*item, base, "rviz_executable");
    if (const auto* item = json::optional(*rviz, "marker_lifetime_ms")) settings.rviz.marker_lifetime_ms = json::integer(*item, 100, 60000);
    if (const auto* item = json::optional(*rviz, "motion_arrow")) settings.rviz.motion_arrow = json::boolean(*item);
    // RViz's fixed frame is a view choice. It must not relabel source camera data.
    for (auto& item : settings.rviz.displays)
      item.second.frame_id = isCloud(item.first) && item.first != "clouds/camera" ? settings.rviz.body_frame :
          isMarker(item.first) ? settings.rviz.fixed_frame : "camera_optical_frame";
    if (const auto* displays = json::optional(*rviz, "displays")) {
      knownStreams(*displays);
      for (const auto& entry : displays->object) {
        auto& display = settings.rviz.displays.at(entry.first);
        const auto& object = entry.second;
        json::fields(object, {"enabled", "topic", "frame_id", "style", "color_transformer", "point_size", "qos"});
        if (const auto* item = json::optional(object, "enabled")) display.enabled = json::boolean(*item);
        if (const auto* item = json::optional(object, "topic")) display.topic = topic(*item);
        if (const auto* item = json::optional(object, "frame_id")) display.frame_id = frame(*item);
        if (const auto* item = json::optional(object, "style")) display.style = choice(*item, "style", {"Points", "Squares", "Flat Squares", "Spheres", "Boxes"});
        if (const auto* item = json::optional(object, "color_transformer")) display.color_transformer = choice(*item, "color_transformer", {"RGB8", "AxisColor", "FlatColor", "Intensity"});
        if (const auto* item = json::optional(object, "point_size")) display.point_size = bounded(*item, 0.1, 100.0, "point_size");
        if (const auto* qos_object = json::optional(object, "qos")) {
          json::fields(*qos_object, {"reliability", "durability", "depth"});
          if (const auto* item = json::optional(*qos_object, "reliability")) display.reliability = choice(*item, "reliability", {"best_effort", "reliable"});
          if (const auto* item = json::optional(*qos_object, "durability")) display.durability = choice(*item, "durability", {"volatile", "transient_local"});
          if (const auto* item = json::optional(*qos_object, "depth")) display.depth = json::integer(*item, 1, 1000);
        }
      }
    }
  }
  // The receiver also owns status/meta and a raw JSON companion for each marker stream.
  std::set<std::string> topics{"/perception_rviz/status", "/perception_rviz/meta"};
  for (const auto& item : settings.rviz.displays) {
    if (!topics.insert(item.second.topic).second) throw std::invalid_argument("Duplicate or reserved RViz topic: " + item.second.topic);
    if (isMarker(item.first) && !topics.insert(item.second.topic + "/json").second)
      throw std::invalid_argument("Duplicate RViz JSON companion topic: " + item.second.topic + "/json");
  }
  return settings;
}

const DisplaySettings& displaySettings(const RvizSettings& settings, const std::string& stream) {
  const auto item = settings.rviz.displays.find(stream);
  if (item == settings.rviz.displays.end()) throw std::invalid_argument("Unknown RViz stream: " + stream);
  return item->second;
}
bool streamEnabled(const RvizSettings& settings, const std::string& stream) {
  const auto item = settings.publish.streams.find(stream);
  return settings.enabled && item != settings.publish.streams.end() && item->second && displaySettings(settings, stream).enabled;
}

std::string settingsJson(const RvizSettings& settings) {
  std::ostringstream out;
  out << std::setprecision(12);
  out << "{\n  \"schema_version\": 1,\n  \"enabled\": " << boolean(settings.enabled)
      << ",\n  \"transport\": {\n    \"bind_address\": " << json::quote(settings.transport.bind_address)
      << ",\n    \"local_port\": " << settings.transport.local_port
      << ",\n    \"remote_port\": " << settings.transport.remote_port
      << ",\n    \"serial\": " << json::quote(settings.transport.serial)
      << ",\n    \"adb_executable\": " << json::quote(settings.transport.adb_executable)
      << ",\n    \"connect_timeout_ms\": " << settings.transport.connect_timeout_ms
      << ",\n    \"reconnect_ms\": " << settings.transport.reconnect_ms
      << ",\n    \"send_timeout_ms\": " << settings.transport.send_timeout_ms
      << ",\n    \"max_frame_bytes\": " << settings.transport.max_frame_bytes
      << "\n  },\n  \"publish\": {\n    \"max_fps\": " << settings.publish.max_fps
      << ",\n    \"every_n\": " << settings.publish.every_n
      << ",\n    \"max_points\": " << settings.publish.max_points
      << ",\n    \"point_stride\": " << settings.publish.point_stride
      << ",\n    \"disparity_min_px\": " << settings.publish.disparity_min_px
      << ",\n    \"disparity_max_px\": " << settings.publish.disparity_max_px
      << ",\n    \"overlay_alpha\": " << settings.publish.overlay_alpha << ",\n    \"streams\": {\n";
  std::size_t index = 0;
  for (const auto& item : settings.publish.streams)
    out << (index++ ? ",\n" : "") << "      " << json::quote(item.first) << ": " << boolean(item.second);
  out << "\n    }\n  },\n  \"rviz\": {\n    \"fixed_frame\": " << json::quote(settings.rviz.fixed_frame)
      << ",\n    \"body_frame\": " << json::quote(settings.rviz.body_frame)
      << ",\n    \"frame_rate\": " << settings.rviz.frame_rate
      << ",\n    \"rviz_executable\": " << json::quote(settings.rviz.rviz_executable)
      << ",\n    \"marker_lifetime_ms\": " << settings.rviz.marker_lifetime_ms
      << ",\n    \"motion_arrow\": " << boolean(settings.rviz.motion_arrow)
      << ",\n    \"displays\": {\n";
  index = 0;
  for (const auto& item : settings.rviz.displays) {
    const auto& display = item.second;
    out << (index++ ? ",\n" : "") << "      " << json::quote(item.first) << ": {"
        << "\"enabled\": " << boolean(display.enabled) << ", \"topic\": " << json::quote(display.topic)
        << ", \"frame_id\": " << json::quote(display.frame_id) << ", \"style\": " << json::quote(display.style)
        << ", \"color_transformer\": " << json::quote(display.color_transformer)
        << ", \"point_size\": " << display.point_size
        << ", \"qos\": {\"reliability\": " << json::quote(display.reliability)
        << ", \"durability\": " << json::quote(display.durability) << ", \"depth\": " << display.depth << "}}";
  }
  out << "\n    }\n  }\n}\n";
  return out.str();
}

std::string rvizConfig(const RvizSettings& settings) {
  std::ostringstream out;
  out << std::setprecision(12)
      << "Panels:\n  - Class: rviz_common/Displays\n    Name: Displays\n"
      << "Visualization Manager:\n  Class: \"\"\n  Global Options:\n"
      << "    Background Color: 25; 29; 36\n    Fixed Frame: " << json::quote(settings.rviz.fixed_frame)
      << "\n    Frame Rate: " << settings.rviz.frame_rate << "\n  Displays:\n";
  for (const auto& item : settings.rviz.displays) {
    const auto& stream = item.first;
    const auto& display = item.second;
    out << "    - Class: rviz_default_plugins/" << (isCloud(stream) ? "PointCloud2" : isMarker(stream) ? "MarkerArray" : "Image") << '\n'
        << "      Name: " << json::quote(stream) << '\n'
        << "      Enabled: " << boolean(streamEnabled(settings, stream)) << '\n';
    qos(out, display, "      ");
    if (isCloud(stream)) {
      out << "      Style: " << json::quote(display.style) << '\n'
          << "      Size (Pixels): " << display.point_size << '\n'
          << "      Size (m): " << display.point_size / 1000.0 << '\n'
          << "      Decay Time: 0\n      Position Transformer: XYZ\n      Color Transformer: " << json::quote(display.color_transformer) << '\n';
      if (display.color_transformer == "Intensity") out << "      Channel Name: confidence\n";
    } else if (!isMarker(stream)) {
      out << "      Transport Hint: raw\n";
      if (stream == "disparity_q8")
        out << "      Normalize Range: false\n      Min Value: " << settings.publish.disparity_min_px * 256.0
            << "\n      Max Value: " << settings.publish.disparity_max_px * 256.0 << "\n      Median window: 5\n";
    }
  }
  out << "  Tools:\n    - Class: rviz_default_plugins/Interact\n    - Class: rviz_default_plugins/MoveCamera\n"
      << "    - Class: rviz_default_plugins/Select\n    - Class: rviz_default_plugins/FocusCamera\n"
      << "  Views:\n    Current:\n      Class: rviz_default_plugins/Orbit\n      Distance: 5\n"
      << "      Focal Point:\n        X: 0\n        Y: 0\n        Z: 2\n      Name: Current View\n"
      << "Window Geometry:\n  Width: 1280\n  Height: 850\n";
  return out.str();
}
} // namespace perception::rviz
