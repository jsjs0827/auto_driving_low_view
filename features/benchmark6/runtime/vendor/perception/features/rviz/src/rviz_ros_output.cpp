#include "rviz_ros_output.hpp"
#include "json.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace perception::rviz {
namespace {
using Json = r33::json::JsonValue;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
enum class StreamKind { Image, Cloud, Text, Unknown };
StreamKind streamKind(const std::string& key) {
  if (key == "input_rgb" || key == "overlay" || key == "mask_rgb" ||
      key == "disparity_rgb" || key == "disparity_q8" || key == "maps/pcl" ||
      key == "maps/ai" || key == "maps/fusion") return StreamKind::Image;
  if (key == "clouds/camera" || key == "clouds/robot" || key == "clouds/ground" ||
      key == "clouds/obstacle" || key == "clouds/semantic") return StreamKind::Cloud;
  if (key == "slip" || key == "result") return StreamKind::Text;
  return StreamKind::Unknown;
}
const Json* field(const Json& json, const char* name) {
  return r33::json::optional(json, name);
}
bool flag(const Json& json, const char* name) {
  const auto* v = field(json, name);
  return v && v->kind == Json::Kind::Boolean && v->boolean;
}
std::string text(const Json& json, const char* name, const std::string& fallback = {}) {
  const auto* v = field(json, name);
  return v && v->kind == Json::Kind::String ? v->text : fallback;
}
std::optional<double> number(const Json& json, const char* name) {
  const auto* v = field(json, name);
  if (!v || v->kind != Json::Kind::Number) return {};
  try { const double value = std::stod(v->text); if (std::isfinite(value)) return value; }
  catch (const std::exception&) {}
  return {};
}
std::optional<std::uint64_t> unsignedInteger(const Json& json, const char* name) {
  const auto* v = field(json, name);
  if (!v || v->kind != Json::Kind::Number || v->text.empty() ||
      v->text.find_first_not_of("0123456789") != std::string::npos) return {};
  try { return std::stoull(v->text); } catch (const std::exception&) { return {}; }
}
std::string scalar(double value) {
  std::ostringstream out; out.imbue(std::locale::classic()); out << std::setprecision(4) << value; return out.str();
}
Json parseObject(const std::string& data) {
  if (data.size() > 64U * 1024U) throw std::invalid_argument("RViz JSON exceeds 64 KiB");
  auto json = r33::json::JsonParser(data, "rviz.text").parse();
  if (json.kind != Json::Kind::Object) throw std::invalid_argument("RViz JSON object required");
  return json;
}
Marker marker(const ViewSettings& view, const rclcpp::Time& time, int id, int type) {
  Marker out; out.header.frame_id = view.fixed_frame; out.header.stamp = time;
  out.ns = "perception_rviz"; out.id = id; out.type = type; out.action = Marker::ADD;
  out.pose.orientation.w = 1; out.color.a = 1;
  out.lifetime = rclcpp::Duration::from_nanoseconds(std::int64_t(view.marker_lifetime_ms) * 1000000LL);
  return out;
}
Marker removeMarker(const ViewSettings& view, const rclcpp::Time& time, int id) {
  auto out = marker(view, time, id, Marker::ARROW); out.action = Marker::DELETE; return out;
}
Marker statusText(const ViewSettings& view, const rclcpp::Time& time, const std::string& data) {
  auto out = marker(view, time, 0, Marker::TEXT_VIEW_FACING);
  out.pose.position.z = 0.5; out.scale.z = 0.08;
  out.color.r = 0.85F; out.color.g = 0.85F; out.color.b = 0.85F; out.text = data; return out;
}
void addArrow(MarkerArray& out, const ViewSettings& view, const rclcpp::Time& time,
    int id, double forward, double lateral, bool visual) {
  // Zero displacement is valid but has no visible arrow geometry.
  if (std::hypot(forward, lateral) < 1e-9) return;
  auto arrow = marker(view, time, id, Marker::ARROW);
  arrow.header.frame_id = view.body_frame;
  arrow.scale.x = 0.012; arrow.scale.y = 0.025; arrow.scale.z = 0.04;
  arrow.color.r = visual ? 0.12F : 1.0F;
  arrow.color.g = visual ? 0.65F : 0.65F;
  arrow.color.b = visual ? 1.0F : 0.08F;
  geometry_msgs::msg::Point start, end;
  end.x = forward; end.y = lateral;
  arrow.points = {start, end}; out.markers.push_back(std::move(arrow));
}
rclcpp::QoS qos(const DisplaySettings& display) {
  rclcpp::QoS out(rclcpp::KeepLast(std::size_t(display.depth)));
  if (display.reliability == "reliable") out.reliable(); else out.best_effort();
  if (display.durability == "transient_local") out.transient_local(); else out.durability_volatile();
  return out;
}
void put32(std::vector<std::uint8_t>& data, std::size_t at, std::uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) data[at + i] = std::uint8_t(value >> (8U * i));
}
void putFloat(std::vector<std::uint8_t>& data, std::size_t at, float value) {
  static_assert(sizeof(float) == sizeof(std::uint32_t), "ROS PointCloud2 requires float32");
  std::uint32_t bits; std::memcpy(&bits, &value, sizeof(bits)); put32(data, at, bits);
}
} // namespace

std_msgs::msg::Header sourceHeader(const Frame& frame, const Packet& packet) {
  if (packet.frame_id.empty()) throw std::invalid_argument("Source frame_id is empty");
  const auto seconds = frame.capture_timestamp_ns / 1000000000ULL;
  if (seconds > std::uint64_t(std::numeric_limits<std::int32_t>::max()))
    throw std::invalid_argument("Capture timestamp exceeds ROS Header range");
  std_msgs::msg::Header out;
  out.frame_id = packet.frame_id;
  out.stamp.sec = std::int32_t(seconds);
  out.stamp.nanosec = std::uint32_t(frame.capture_timestamp_ns % 1000000000ULL);
  return out;
}

sensor_msgs::msg::Image imageMessage(const Image& image, const std_msgs::msg::Header& header) {
  const std::uint32_t bytes = image.encoding == ImageEncoding::Rgb8 ? 3U :
      image.encoding == ImageEncoding::Mono8 ? 1U : image.encoding == ImageEncoding::Mono16 ? 2U : 0U;
  if (!bytes || !image.width || !image.height || std::uint64_t(image.step) < std::uint64_t(image.width) * bytes ||
      std::uint64_t(image.step) * image.height != image.data.size())
    throw std::invalid_argument("Invalid RViz image layout");
  sensor_msgs::msg::Image out; out.header = header;
  out.width = image.width; out.height = image.height; out.step = image.step;
  out.encoding = image.encoding == ImageEncoding::Rgb8 ? "rgb8" : image.encoding == ImageEncoding::Mono8 ? "mono8" : "16UC1";
  out.is_bigendian = false; out.data = image.data; return out;
}

sensor_msgs::msg::PointCloud2 cloudMessage(const PointCloud& cloud, const std_msgs::msg::Header& header) {
  if (cloud.points.size() > std::numeric_limits<std::uint32_t>::max() / 24U)
    throw std::invalid_argument("RViz PointCloud2 is too large");
  sensor_msgs::msg::PointCloud2 out; out.header = header;
  out.height = 1; out.width = std::uint32_t(cloud.points.size()); out.point_step = 24;
  out.row_step = out.width * out.point_step; out.is_bigendian = false; out.is_dense = true;
  const char* names[] = {"x", "y", "z", "rgb", "confidence", "label"};
  for (std::uint32_t i = 0; i < 6; ++i) {
    sensor_msgs::msg::PointField f; f.name = names[i]; f.offset = i * 4; f.count = 1;
    f.datatype = i == 5 ? sensor_msgs::msg::PointField::INT32 : sensor_msgs::msg::PointField::FLOAT32;
    out.fields.push_back(std::move(f));
  }
  out.data.resize(out.row_step);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& p = cloud.points[i]; const auto offset = i * 24;
    putFloat(out.data, offset, p.x); putFloat(out.data, offset + 4, p.y); putFloat(out.data, offset + 8, p.z);
    put32(out.data, offset + 12, p.rgb); putFloat(out.data, offset + 16, p.confidence);
    put32(out.data, offset + 20, std::uint32_t(p.label));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) out.is_dense = false;
  }
  return out;
}

MarkerArray slipMarkers(const std::string& data, const ViewSettings& view, const rclcpp::Time& time) {
  // Explicit deletes prevent an old valid displacement surviving a new invalid frame.
  MarkerArray out; out.markers = {removeMarker(view, time, 1), removeMarker(view, time, 2)};
  try {
    const auto json = parseObject(data);
    auto state = text(json, "state", text(json, "state_name", "unavailable"));
    auto t0 = unsignedInteger(json, "t0_ns"), t1 = unsignedInteger(json, "t1_ns");
    if (!t0) t0 = unsignedInteger(json, "start_timestamp_ns");
    if (!t1) t1 = unsignedInteger(json, "end_timestamp_ns");
    const auto dt = number(json, "dt_s");
    const bool interval = t0 && t1 && *t1 > *t0 && dt && *dt > 0;
    const bool visual = flag(json, "visual_valid"), reference = flag(json, "reference_valid");
    const bool calibrated = flag(json, "body_calibration_confirmed");
    const bool evaluated_state = state == "normal" || state == "suspected" || state == "slipping" || state == "recovering";
    const bool evaluated = flag(json, "enabled") && evaluated_state && visual && reference && calibrated && interval;
    const std::string shown = evaluated_state && !evaluated ? "unavailable" : state;
    std::string label = "slip: " + shown;
    if (shown != state) label += " (reported " + state + ")";
    const auto ratio = number(json, "slip_ratio");
    if (evaluated && ratio) label += "  ratio=" + scalar(*ratio);
    label += "\nvisual=" + std::string(visual ? "valid" : "unavailable") +
        "  wheel=" + (reference ? "valid" : "unavailable") + "  body=" + (calibrated ? "confirmed" : "unconfirmed");
    if (t0 && t1) label += "\ninterval ns: " + std::to_string(*t0) + " -> " + std::to_string(*t1);
    const auto vf = number(json, "visual_forward_m"), vl = number(json, "visual_lateral_m");
    const auto rf = number(json, "reference_forward_m"), rl = number(json, "reference_lateral_m");
    if (visual && calibrated && interval && vf && vl)
      label += "\nvisual delta body m: (" + scalar(*vf) + ", " + scalar(*vl) + ")";
    if (reference && interval && rf && rl)
      label += "\nwheel delta body m: (" + scalar(*rf) + ", " + scalar(*rl) + ")";
    const auto reason = text(json, "reason"); if (!reason.empty()) label += "\n" + reason.substr(0, 256);
    if (view.motion_arrow && view.fixed_frame != view.body_frame)
      label += "\nlocal arrows require Fixed Frame=" + view.body_frame;
    auto status = statusText(view, time, label);
    if (evaluated && (state == "slipping" || state == "suspected")) {
      status.color.r = 1; status.color.g = state == "slipping" ? 0.15F : 0.65F; status.color.b = 0.1F;
    } else if (evaluated && state == "normal") {
      status.color.r = 0.2F; status.color.g = 0.9F; status.color.b = 0.3F;
    }
    out.markers.push_back(std::move(status));
    if (view.motion_arrow && view.fixed_frame == view.body_frame && flag(json, "enabled") && interval) {
      if (visual && calibrated && vf && vl) addArrow(out, view, time, 1, *vf, *vl, true);
      if (reference && rf && rl) addArrow(out, view, time, 2, *rf, *rl, false);
    }
  } catch (const std::exception& error) {
    out.markers.push_back(statusText(view, time, std::string("slip: invalid JSON\n") + error.what()));
  }
  return out;
}

MarkerArray resultMarkers(const std::string& data, const ViewSettings& view, const rclcpp::Time& time) {
  MarkerArray out;
  try {
    const auto json = parseObject(data);
    std::string label = "perception result";
    if (auto id = unsignedInteger(json, "frame_id")) label += "  frame=" + std::to_string(*id);
    if (auto graph = number(json, "graph_ms")) label += "\ngraph=" + scalar(*graph) + " ms";
    if (auto process = number(json, "process_ms")) label += "  process=" + scalar(*process) + " ms";
    const auto* detections = field(json, "detections");
    if (detections && detections->kind == Json::Kind::Array) label += "\ndetections=" + std::to_string(detections->array.size());
    auto status = statusText(view, time, label); status.pose.position.z = 1; out.markers.push_back(std::move(status));
  } catch (const std::exception& error) {
    out.markers.push_back(statusText(view, time, std::string("result: invalid JSON\n") + error.what()));
  }
  return out;
}

RosOutput::RosOutput(rclcpp::Node& node, RvizSettings settings) : node_(node), settings_(std::move(settings)) {
  // Diagnostics are small, reliable and latched for a newly opened subscriber.
  const auto diagnostics_qos = rclcpp::QoS(1).reliable().transient_local();
  status_ = node_.create_publisher<std_msgs::msg::String>("/perception_rviz/status", diagnostics_qos);
  metadata_ = node_.create_publisher<std_msgs::msg::String>("/perception_rviz/meta", diagnostics_qos);
  for (const auto& item : settings_.rviz.displays) {
    const auto& key = item.first; const auto& display = item.second;
    if (!settings_.enabled || !streamEnabled(settings_, key)) continue;
    switch (streamKind(key)) {
      case StreamKind::Image: images_[key] = node_.create_publisher<sensor_msgs::msg::Image>(display.topic, qos(display)); break;
      case StreamKind::Cloud: clouds_[key] = node_.create_publisher<sensor_msgs::msg::PointCloud2>(display.topic, qos(display)); break;
      case StreamKind::Text:
        markers_[key] = node_.create_publisher<MarkerArray>(display.topic, qos(display));
        texts_[key] = node_.create_publisher<std_msgs::msg::String>(display.topic + "/json", qos(display)); break;
      case StreamKind::Unknown: break;
    }
  }
  status("disconnected", "waiting for X5 publisher through adb forward");
}

void RosOutput::publish(const Frame& frame, std::uint64_t dropped_frames) {
  if (session_ && session_ != frame.session_id) {
    for (const auto& item : seen_) clearStream(item.first);
    seen_.clear();
  }
  session_ = frame.session_id; connected_ = true; received_frame_ = true;
  last_frame_ = std::chrono::steady_clock::now();
  status("connected", "receiving X5 frames");
  const auto display_time = node_.now();
  std::string streams = "[", rejections = "["; std::size_t accepted = 0, rejected = 0;
  const auto reject = [&](const std::string& stream, const std::string& reason) {
    if (rejected++) rejections += ',';
    rejections += "{\"stream\":" + r33::json::quote(stream) + ",\"reason\":" + r33::json::quote(reason) + '}';
  };
  for (const auto& packet : frame.packets) {
    const auto configured = settings_.rviz.displays.find(packet.topic);
    if (!settings_.enabled || configured == settings_.rviz.displays.end() ||
        !streamEnabled(settings_, packet.topic) || streamKind(packet.topic) == StreamKind::Unknown) {
      reject(packet.topic, "stream is disabled or outside the local allowlist"); continue;
    }
    try {
      const auto header = sourceHeader(frame, packet);
      if (streamKind(packet.topic) != StreamKind::Text && !configured->second.frame_id.empty() &&
          configured->second.frame_id != packet.frame_id)
        throw std::invalid_argument("source frame_id=" + packet.frame_id + " differs from configured expected frame_id=" +
            configured->second.frame_id + "; no implicit coordinate transform is permitted");
      if (auto image = std::get_if<Image>(&packet.payload)) {
        const auto found = images_.find(packet.topic);
        if (found == images_.end() || (packet.topic == "disparity_q8" && image->encoding != ImageEncoding::Mono16))
          throw std::invalid_argument("Unexpected payload type for image stream");
        found->second->publish(imageMessage(*image, header));
      } else if (auto cloud = std::get_if<PointCloud>(&packet.payload)) {
        const auto found = clouds_.find(packet.topic);
        if (found == clouds_.end()) throw std::invalid_argument("Unexpected cloud stream");
        found->second->publish(cloudMessage(*cloud, header));
      } else if (auto data = std::get_if<Text>(&packet.payload)) {
        const auto found = texts_.find(packet.topic);
        if (found == texts_.end()) throw std::invalid_argument("Unexpected text stream");
        std_msgs::msg::String original; original.data = data->data; found->second->publish(original);
        auto marker_array = packet.topic == "slip" ? slipMarkers(data->data, settings_.rviz, display_time) :
            resultMarkers(data->data, settings_.rviz, display_time);
        markers_.at(packet.topic)->publish(marker_array);
      }
      headers_[packet.topic] = header; seen_[packet.topic] = last_frame_;
      if (accepted++) streams += ',';
      streams += "{\"stream\":" + r33::json::quote(packet.topic) + ",\"topic\":" +
          r33::json::quote(configured->second.topic) + ",\"frame_id\":" + r33::json::quote(packet.frame_id) + "}";
    } catch (const std::exception& error) {
      reject(packet.topic, error.what());
      RCLCPP_WARN(node_.get_logger(), "Rejected stream %s: %s", packet.topic.c_str(), error.what());
    }
  }
  streams += ']'; rejections += ']';
  std_msgs::msg::String metadata;
  metadata.data = "{\"session_id\":" + std::to_string(frame.session_id) + ",\"sequence\":" + std::to_string(frame.sequence) +
      ",\"frame_number\":" + std::to_string(frame.frame_number) + ",\"capture_timestamp_ns\":" + std::to_string(frame.capture_timestamp_ns) +
      ",\"send_timestamp_ns\":" + std::to_string(frame.send_timestamp_ns) + ",\"receive_ros_timestamp_ns\":" +
      std::to_string(display_time.nanoseconds()) + ",\"dropped_frames\":" + std::to_string(dropped_frames) +
      ",\"accepted_packets\":" + std::to_string(accepted) + ",\"rejected_packets\":" + std::to_string(rejected) +
      ",\"rejections\":" + rejections + ",\"streams\":" + streams + '}';
  metadata_->publish(metadata);
}

void RosOutput::status(const std::string& state, const std::string& detail) {
  if (state == last_state_ && detail == last_detail_) return;
  std_msgs::msg::String message;
  message.data = "{\"state\":" + r33::json::quote(state) + ",\"detail\":" + r33::json::quote(detail) +
      ",\"session_id\":" + std::to_string(session_) + ",\"receive_ros_timestamp_ns\":" + std::to_string(node_.now().nanoseconds()) + '}';
  status_->publish(message); last_state_ = state; last_detail_ = detail;
  // A separate namespace makes transport state visible in the configured RViz
  // text display. A disconnected label is diagnostic, not a stale slip result.
  auto visible = markers_.find("slip");
  if (visible == markers_.end()) visible = markers_.find("result");
  if (visible != markers_.end()) {
    MarkerArray array;
    auto notice = statusText(settings_.rviz, node_.now(), "X5 bridge: " + state + "\n" + detail);
    notice.ns = "perception_rviz_bridge"; notice.pose.position.z = 0.25;
    notice.lifetime = rclcpp::Duration::from_nanoseconds(0);
    if (state == "connected" && received_frame_) notice.action = Marker::DELETE;
    else {
      // Include removal and the diagnostic in one sample: a depth-one DDS queue
      // must not discard the removal while retaining only the status notice.
      auto remove = marker(settings_.rviz, node_.now(), 0, Marker::TEXT_VIEW_FACING);
      remove.action = Marker::DELETEALL; array.markers.push_back(std::move(remove));
      notice.color.r = 1; notice.color.g = 0.5F; notice.color.b = 0.1F;
    }
    array.markers.push_back(std::move(notice)); visible->second->publish(array);
  }
  RCLCPP_INFO(node_.get_logger(), "X5 RViz bridge %s: %s", state.c_str(), detail.c_str());
}

void RosOutput::clearStream(const std::string& key) {
  // A clear sample keeps the last source header. Receiving/clearing data never
  // rewrites a capture timestamp into a host receive timestamp.
  const auto header = headers_[key];
  if (auto found = clouds_.find(key); found != clouds_.end()) found->second->publish(cloudMessage({}, header));
  if (auto found = images_.find(key); found != images_.end()) {
    // Deliberately synthetic black tile replaces stale imagery; diagnostics mark
    // the stale/disconnected state. It is not a source frame.
    Image blank; blank.width = blank.height = 1;
    blank.encoding = key == "disparity_q8" ? ImageEncoding::Mono16 : ImageEncoding::Rgb8;
    blank.step = key == "disparity_q8" ? 2 : 3; blank.data.resize(blank.step);
    found->second->publish(imageMessage(blank, header));
  }
  if (auto found = markers_.find(key); found != markers_.end()) {
    MarkerArray cleared; auto remove = marker(settings_.rviz, node_.now(), 0, Marker::TEXT_VIEW_FACING);
    remove.action = Marker::DELETEALL; cleared.markers.push_back(std::move(remove)); found->second->publish(cleared);
  }
}

void RosOutput::connectionState(bool connected, const std::string& detail) {
  connected_ = connected;
  if (!connected) {
    for (const auto& item : seen_) clearStream(item.first);
    seen_.clear(); received_frame_ = false; status("disconnected", detail);
  } else if (!received_frame_) status("connected", detail.empty() ? "transport connected; waiting for frames" : detail);
}

void RosOutput::expire() {
  const auto now = std::chrono::steady_clock::now();
  const auto lifetime = std::chrono::milliseconds(settings_.rviz.marker_lifetime_ms);
  for (auto it = seen_.begin(); it != seen_.end();) {
    if (now - it->second >= lifetime) { clearStream(it->first); it = seen_.erase(it); } else ++it;
  }
  if (connected_ && received_frame_ && now - last_frame_ >= lifetime)
    status("stale", "transport may be connected but no fresh source frame; cleared stale displays");
}
} // namespace perception::rviz
