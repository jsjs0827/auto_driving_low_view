#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

// This protocol is independent of ROS. Header integers are network (big-endian)
// order; typed body integers and IEEE-754 float32 are explicitly little-endian.
// There are no packed C structs on the wire and no borrowed image/cloud buffers.
namespace perception::rviz {

struct Limits {
  std::uint32_t max_body_bytes = 32U * 1024U * 1024U;
  std::uint32_t max_packets = 32;
  std::uint32_t max_string_bytes = 1024;
  std::uint32_t max_image_width = 4096;
  std::uint32_t max_image_height = 4096;
  std::uint32_t max_image_bytes = 16U * 1024U * 1024U;
  std::uint32_t max_points = 500000;
  std::uint32_t max_text_bytes = 64U * 1024U;
};

enum class ImageEncoding : std::uint8_t { Rgb8 = 1, Mono8 = 2, Mono16 = 3 };
struct Image {
  std::uint32_t width = 0, height = 0, step = 0;
  ImageEncoding encoding = ImageEncoding::Rgb8;
  std::vector<std::uint8_t> data; // Mono16 is 16UC1, little-endian.
};
struct Point {
  float x = 0, y = 0, z = 0;
  std::uint32_t rgb = 0; // 0x00RRGGBB
  float confidence = 0;
  std::int32_t label = 0;
};
struct PointCloud { std::vector<Point> points; };
struct Text { std::string data; }; // UTF-8 JSON for result/slip.
struct Packet {
  // Logical stream key (e.g. "clouds/camera"), never an authoritative ROS topic.
  // Receiver must map allowed keys to its locally configured ROS topics.
  std::string topic;
  std::string frame_id;
  std::variant<Image, PointCloud, Text> payload;
};
struct Frame {
  std::uint64_t session_id = 0;
  std::uint64_t sequence = 0;
  std::uint64_t frame_number = 0;
  std::uint64_t capture_timestamp_ns = 0;
  std::uint64_t send_timestamp_ns = 0;
  std::vector<Packet> packets;
};

// v1 outer header, offsets in bytes (all big-endian):
// 0:u32 "RVZ1", 4:u16 version=1, 6:u16 kind, 8:u32 body_bytes,
// 12:u32 packet_count, 16:u64 session, 24:u64 sequence, 32:u64 frame_number,
// 40:u64 capture_ns, 48:u64 send_ns, 56:u64 reserved=0.
// Each little-endian body record is: u32 record_bytes (excluding this u32),
// u8 type(1=image,2=cloud,3=text), u8 image_encoding(or0), u16 reserved=0,
// u32 stream_bytes, u32 frame_id_bytes, stream chars, frame_id chars, followed by:
// image: u32 width,height,step,data_bytes + data;
// cloud: u32 point_count + 24bytes/point (f32 xyz,u32 rgb,f32 confidence,i32 label);
// text: u32 data_bytes + UTF-8 chars. Strings have no terminating NUL.
constexpr std::size_t kWireHeaderSize = 64;
enum class MessageKind : std::uint16_t { Frame = 1, Heartbeat = 2 };
struct WireHeader {
  MessageKind kind = MessageKind::Frame;
  std::uint32_t body_bytes = 0, packet_count = 0;
  std::uint64_t session_id = 0, sequence = 0, frame_number = 0;
  std::uint64_t capture_timestamp_ns = 0, send_timestamp_ns = 0;
};

bool validate_frame(const Frame& frame, const Limits& limits = {}, std::string* error = nullptr);
bool encode_frame(const Frame& frame, std::vector<std::uint8_t>& bytes,
                  std::string* error = nullptr, const Limits& limits = {});
bool decode_header(const std::uint8_t* bytes, std::size_t size, WireHeader& header,
                   std::string* error = nullptr, const Limits& limits = {});
bool decode_frame(const WireHeader& header, const std::uint8_t* body, std::size_t size,
                  Frame& frame, std::string* error = nullptr, const Limits& limits = {});
bool decode_frame(const std::vector<std::uint8_t>& bytes, Frame& frame,
                  std::string* error = nullptr, const Limits& limits = {});
std::vector<std::uint8_t> encode_heartbeat(std::uint64_t session_id, std::uint64_t sequence,
                                         std::uint64_t send_timestamp_ns);
std::uint64_t system_time_ns();
const char* image_encoding_name(ImageEncoding encoding);

} // namespace perception::rviz
