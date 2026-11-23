#include "rviz_wire.hpp"

#include <chrono>
#include <cstring>
#include <limits>

namespace perception::rviz {
namespace {
constexpr std::uint32_t kMagic = 0x52565a31U; // RVZ1
constexpr std::uint16_t kVersion = 1;
constexpr std::uint32_t kHardBodyLimit = 64U * 1024U * 1024U;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "RViz wire requires IEEE-754 float32");
bool fail(std::string* error, const char* reason) {
  if (error) *error = reason;
  return false;
}
bool valid_limits(const Limits& l) {
  return l.max_body_bytes > 0 && l.max_body_bytes <= kHardBodyLimit &&
         l.max_packets > 0 && l.max_packets <= 64 && l.max_string_bytes <= 4096 &&
         l.max_image_width <= 16384 && l.max_image_height <= 16384 &&
         l.max_image_bytes <= kHardBodyLimit && l.max_points <= 1000000 &&
         l.max_text_bytes <= 1024U * 1024U;
}
void put(std::vector<std::uint8_t>& out, std::uint64_t v, unsigned count, bool be = false) {
  for (unsigned i = 0; i < count; ++i)
    out.push_back(static_cast<std::uint8_t>(v >> (8U * (be ? count - 1 - i : i))));
}
std::uint64_t get(const std::uint8_t* p, unsigned count, bool be = false) {
  std::uint64_t v = 0;
  for (unsigned i = 0; i < count; ++i)
    v |= std::uint64_t(p[i]) << (8U * (be ? count - 1 - i : i));
  return v;
}
void put_float(std::vector<std::uint8_t>& out, float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  put(out, bits, 4);
}
void put_string(std::vector<std::uint8_t>& out, const std::string& s) {
  out.insert(out.end(), s.begin(), s.end());
}
std::uint64_t packet_size(const Packet& p) {
  std::uint64_t size = 12ULL + p.topic.size() + p.frame_id.size();
  if (const auto* image = std::get_if<Image>(&p.payload)) size += 16ULL + image->data.size();
  else if (const auto* cloud = std::get_if<PointCloud>(&p.payload)) size += 4ULL + 24ULL * cloud->points.size();
  else size += 4ULL + std::get<Text>(p.payload).data.size();
  return size;
}
void put_header(std::vector<std::uint8_t>& out, const WireHeader& h) {
  put(out, kMagic, 4, true); put(out, kVersion, 2, true);
  put(out, static_cast<std::uint16_t>(h.kind), 2, true);
  put(out, h.body_bytes, 4, true); put(out, h.packet_count, 4, true);
  put(out, h.session_id, 8, true); put(out, h.sequence, 8, true);
  put(out, h.frame_number, 8, true); put(out, h.capture_timestamp_ns, 8, true);
  put(out, h.send_timestamp_ns, 8, true); put(out, 0, 8, true);
}
class Reader {
 public:
  Reader(const std::uint8_t* p, std::size_t size) : p_(p), size_(size) {}
  bool integer(std::uint64_t& value, unsigned count) {
    if (count > remaining()) return false;
    value = get(p_ + pos_, count); pos_ += count; return true;
  }
  bool u32(std::uint32_t& value) {
    std::uint64_t v; if (!integer(v, 4)) return false;
    value = static_cast<std::uint32_t>(v); return true;
  }
  bool real(float& value) {
    std::uint32_t bits; if (!u32(bits)) return false;
    std::memcpy(&value, &bits, sizeof(value)); return true;
  }
  bool string(std::string& s, std::uint32_t count) {
    if (count > remaining()) return false;
    s.assign(reinterpret_cast<const char*>(p_ + pos_), count); pos_ += count; return true;
  }
  bool bytes(std::vector<std::uint8_t>& out, std::uint32_t count) {
    if (count > remaining()) return false;
    out.assign(p_ + pos_, p_ + pos_ + count); pos_ += count; return true;
  }
  const std::uint8_t* current() const { return p_ + pos_; }
  bool skip(std::size_t count) {
    if (count > remaining()) return false;
    pos_ += count; return true;
  }
  std::size_t remaining() const { return size_ - pos_; }
 private:
  const std::uint8_t* p_;
  std::size_t size_, pos_ = 0;
};
bool valid_image(const Image& image, const Limits& limits) {
  std::uint32_t bpp = 0;
  switch (image.encoding) {
    case ImageEncoding::Rgb8: bpp = 3; break;
    case ImageEncoding::Mono8: bpp = 1; break;
    case ImageEncoding::Mono16: bpp = 2; break;
    default: return false;
  }
  return image.width > 0 && image.width <= limits.max_image_width &&
         image.height > 0 && image.height <= limits.max_image_height &&
         std::uint64_t(image.step) >= std::uint64_t(image.width) * bpp &&
         std::uint64_t(image.step) * image.height == image.data.size() &&
         image.data.size() <= limits.max_image_bytes;
}
} // namespace

std::uint64_t system_time_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}
const char* image_encoding_name(ImageEncoding encoding) {
  switch (encoding) {
    case ImageEncoding::Rgb8: return "rgb8";
    case ImageEncoding::Mono8: return "mono8";
    case ImageEncoding::Mono16: return "16UC1";
    default: return "";
  }
}
bool validate_frame(const Frame& frame, const Limits& limits, std::string* error) {
  if (!valid_limits(limits)) return fail(error, "invalid wire limits");
  if (frame.packets.empty() || frame.packets.size() > limits.max_packets)
    return fail(error, "invalid packet count");
  std::uint64_t total = 0;
  for (const auto& p : frame.packets) {
    if (p.topic.empty() || p.topic.size() > limits.max_string_bytes ||
        p.frame_id.size() > limits.max_string_bytes || p.topic.find('\0') != std::string::npos ||
        p.frame_id.find('\0') != std::string::npos)
      return fail(error, "invalid stream/frame identifier");
    if (const auto* image = std::get_if<Image>(&p.payload)) {
      if (!valid_image(*image, limits)) return fail(error, "invalid image dimensions/stride/data");
    } else if (const auto* cloud = std::get_if<PointCloud>(&p.payload)) {
      if (cloud->points.size() > limits.max_points) return fail(error, "point count exceeds limit");
    } else if (std::get<Text>(p.payload).data.size() > limits.max_text_bytes) {
      return fail(error, "text exceeds limit");
    }
    total += 4 + packet_size(p);
    if (total > limits.max_body_bytes) return fail(error, "frame body exceeds limit");
  }
  if (error) error->clear();
  return true;
}
bool encode_frame(const Frame& frame, std::vector<std::uint8_t>& bytes,
                  std::string* error, const Limits& limits) {
  if (!validate_frame(frame, limits, error)) return false;
  WireHeader h;
  h.session_id = frame.session_id; h.sequence = frame.sequence; h.frame_number = frame.frame_number;
  h.capture_timestamp_ns = frame.capture_timestamp_ns; h.send_timestamp_ns = frame.send_timestamp_ns;
  h.packet_count = static_cast<std::uint32_t>(frame.packets.size());
  for (const auto& p : frame.packets) h.body_bytes += static_cast<std::uint32_t>(4 + packet_size(p));
  bytes.clear(); bytes.reserve(kWireHeaderSize + h.body_bytes);
  put_header(bytes, h);
  for (const auto& p : frame.packets) {
    put(bytes, packet_size(p), 4);
    const auto* image = std::get_if<Image>(&p.payload);
    const auto* cloud = std::get_if<PointCloud>(&p.payload);
    put(bytes, image ? 1 : cloud ? 2 : 3, 1);
    put(bytes, image ? static_cast<std::uint8_t>(image->encoding) : 0, 1); put(bytes, 0, 2);
    put(bytes, p.topic.size(), 4); put(bytes, p.frame_id.size(), 4);
    put_string(bytes, p.topic); put_string(bytes, p.frame_id);
    if (image) {
      put(bytes, image->width, 4); put(bytes, image->height, 4); put(bytes, image->step, 4);
      put(bytes, image->data.size(), 4); bytes.insert(bytes.end(), image->data.begin(), image->data.end());
    } else if (cloud) {
      put(bytes, cloud->points.size(), 4);
      for (const auto& point : cloud->points) {
        put_float(bytes, point.x); put_float(bytes, point.y); put_float(bytes, point.z);
        put(bytes, point.rgb, 4); put_float(bytes, point.confidence);
        put(bytes, static_cast<std::uint32_t>(point.label), 4);
      }
    } else {
      const auto& text = std::get<Text>(p.payload).data;
      put(bytes, text.size(), 4); put_string(bytes, text);
    }
  }
  return true;
}
bool decode_header(const std::uint8_t* bytes, std::size_t size, WireHeader& header,
                   std::string* error, const Limits& limits) {
  if (!valid_limits(limits)) return fail(error, "invalid wire limits");
  if (!bytes || size != kWireHeaderSize) return fail(error, "invalid header size");
  if (get(bytes, 4, true) != kMagic || get(bytes + 4, 2, true) != kVersion)
    return fail(error, "unsupported magic/version");
  const auto kind = get(bytes + 6, 2, true);
  if ((kind != 1 && kind != 2) || get(bytes + 56, 8, true) != 0)
    return fail(error, "invalid message kind/reserved header");
  WireHeader h;
  h.kind = static_cast<MessageKind>(kind);
  h.body_bytes = static_cast<std::uint32_t>(get(bytes + 8, 4, true));
  h.packet_count = static_cast<std::uint32_t>(get(bytes + 12, 4, true));
  if (h.body_bytes > limits.max_body_bytes || h.packet_count > limits.max_packets ||
      (h.kind == MessageKind::Heartbeat && (h.body_bytes || h.packet_count)) ||
      (h.kind == MessageKind::Frame && (!h.packet_count || h.body_bytes < h.packet_count * 20U)))
    return fail(error, "invalid body/packet limits");
  h.session_id = get(bytes + 16, 8, true); h.sequence = get(bytes + 24, 8, true);
  h.frame_number = get(bytes + 32, 8, true); h.capture_timestamp_ns = get(bytes + 40, 8, true);
  h.send_timestamp_ns = get(bytes + 48, 8, true);
  header = h;
  if (error) error->clear();
  return true;
}
bool decode_frame(const WireHeader& h, const std::uint8_t* body, std::size_t size,
                  Frame& frame, std::string* error, const Limits& limits) {
  if (!valid_limits(limits) || h.kind != MessageKind::Frame || h.body_bytes != size ||
      size > limits.max_body_bytes || !body || !h.packet_count || h.packet_count > limits.max_packets)
    return fail(error, "invalid frame body/header");
  Frame decoded;
  decoded.session_id = h.session_id; decoded.sequence = h.sequence; decoded.frame_number = h.frame_number;
  decoded.capture_timestamp_ns = h.capture_timestamp_ns; decoded.send_timestamp_ns = h.send_timestamp_ns;
  decoded.packets.reserve(h.packet_count);
  Reader body_reader(body, size);
  for (std::uint32_t i = 0; i < h.packet_count; ++i) {
    std::uint32_t record_size;
    if (!body_reader.u32(record_size) || record_size > body_reader.remaining())
      return fail(error, "truncated packet");
    Reader r(body_reader.current(), record_size);
    std::uint64_t type, encoding, reserved;
    std::uint32_t topic_size, frame_size;
    if (!r.integer(type, 1) || !r.integer(encoding, 1) || !r.integer(reserved, 2) || reserved ||
        !r.u32(topic_size) || !r.u32(frame_size) || !topic_size ||
        topic_size > limits.max_string_bytes || frame_size > limits.max_string_bytes)
      return fail(error, "invalid packet metadata");
    Packet packet;
    if (!r.string(packet.topic, topic_size) || !r.string(packet.frame_id, frame_size))
      return fail(error, "truncated identifiers");
    if (type == 1) {
      Image image; std::uint32_t data_size;
      image.encoding = static_cast<ImageEncoding>(encoding);
      if (!r.u32(image.width) || !r.u32(image.height) || !r.u32(image.step) || !r.u32(data_size) ||
          !image.width || image.width > limits.max_image_width || !image.height ||
          image.height > limits.max_image_height || data_size > limits.max_image_bytes ||
          data_size != r.remaining() || std::uint64_t(image.step) * image.height != data_size)
        return fail(error, "invalid image size");
      const auto bpp = encoding == 1 ? 3U : encoding == 2 ? 1U : encoding == 3 ? 2U : 0U;
      if (!bpp || std::uint64_t(image.step) < std::uint64_t(image.width) * bpp || !r.bytes(image.data, data_size))
        return fail(error, "invalid image stride/encoding");
      packet.payload = std::move(image);
    } else if (type == 2 && encoding == 0) {
      PointCloud cloud; std::uint32_t count;
      if (!r.u32(count) || count > limits.max_points || std::uint64_t(count) * 24 != r.remaining())
        return fail(error, "invalid cloud count");
      cloud.points.resize(count);
      for (auto& point : cloud.points) {
        std::uint32_t label;
        if (!r.real(point.x) || !r.real(point.y) || !r.real(point.z) || !r.u32(point.rgb) ||
            !r.real(point.confidence) || !r.u32(label)) return fail(error, "truncated point");
        // Copy the two's-complement bit pattern without implementation-defined unsigned conversion.
        std::memcpy(&point.label, &label, sizeof(label));
      }
      packet.payload = std::move(cloud);
    } else if (type == 3 && encoding == 0) {
      Text text; std::uint32_t count;
      if (!r.u32(count) || count > limits.max_text_bytes || count != r.remaining() || !r.string(text.data, count))
        return fail(error, "invalid text length");
      packet.payload = std::move(text);
    } else return fail(error, "unsupported packet type/encoding");
    if (r.remaining() || !body_reader.skip(record_size)) return fail(error, "trailing packet data");
    decoded.packets.push_back(std::move(packet));
  }
  if (body_reader.remaining()) return fail(error, "trailing frame data");
  if (!validate_frame(decoded, limits, error)) return false;
  frame = std::move(decoded);
  return true;
}
bool decode_frame(const std::vector<std::uint8_t>& bytes, Frame& frame,
                  std::string* error, const Limits& limits) {
  if (bytes.size() < kWireHeaderSize) return fail(error, "truncated header");
  WireHeader h;
  if (!decode_header(bytes.data(), kWireHeaderSize, h, error, limits)) return false;
  return decode_frame(h, bytes.data() + kWireHeaderSize, bytes.size() - kWireHeaderSize, frame, error, limits);
}
std::vector<std::uint8_t> encode_heartbeat(std::uint64_t session_id, std::uint64_t sequence,
                                         std::uint64_t send_timestamp_ns) {
  WireHeader h; h.kind = MessageKind::Heartbeat;
  h.session_id = session_id; h.sequence = sequence; h.send_timestamp_ns = send_timestamp_ns;
  std::vector<std::uint8_t> bytes; bytes.reserve(kWireHeaderSize); put_header(bytes, h); return bytes;
}
} // namespace perception::rviz
