#include "rviz_transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <cerrno>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace perception::rviz;
using namespace std::chrono_literals;
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #condition); } while (false)

namespace {
Frame sample(std::uint64_t number = 77) {
  Frame frame; frame.session_id = 0x0102030405060708ULL; frame.sequence = 2;
  frame.frame_number = number; frame.capture_timestamp_ns = 123456789000ULL;
  frame.send_timestamp_ns = 123456790000ULL;
  frame.packets.push_back(Packet{"input_rgb", "camera_optical_frame", Image{2, 1, 8, ImageEncoding::Rgb8, {255, 0, 0, 0, 255, 0, 99, 99}}});
  frame.packets.push_back(Packet{"disparity_q8", "camera_optical_frame", Image{2, 1, 4, ImageEncoding::Mono16, {0, 1, 128, 2}}});
  frame.packets.push_back(Packet{"clouds/camera", "camera_optical_frame", PointCloud{{Point{1.0f, -2.0f, 3.5f, 0x00aabbcc, 0.75f, -9}}}});
  frame.packets.push_back(Packet{"slip", "base_link", Text{"{\"slip\":true}"}});
  return frame;
}
void check_sample(const Frame& frame) {
  CHECK(frame.frame_number == 77); CHECK(frame.capture_timestamp_ns == 123456789000ULL);
  CHECK(frame.packets.size() == 4); CHECK(frame.packets[0].frame_id == "camera_optical_frame");
  const auto& rgb = std::get<Image>(frame.packets[0].payload);
  CHECK(rgb.width == 2 && rgb.step == 8 && rgb.data.size() == 8 && rgb.data[7] == 99);
  const auto& q8 = std::get<Image>(frame.packets[1].payload);
  CHECK(q8.encoding == ImageEncoding::Mono16 && q8.data[1] == 1 && q8.data[3] == 2);
  const auto& point = std::get<PointCloud>(frame.packets[2].payload).points.at(0);
  CHECK(point.x == 1.0f && point.y == -2.0f && point.z == 3.5f);
  CHECK(point.rgb == 0x00aabbcc && point.confidence == 0.75f && point.label == -9);
  CHECK(std::get<Text>(frame.packets[3].payload).data == "{\"slip\":true}");
}
// Hand-authored golden packet, independent of the encoder: one "slip" text
// stream in frame "b", frame=77, sequence=2, session=1 and text="{}".
std::vector<std::uint8_t> golden_text() {
  return {
    0x52,0x56,0x5a,0x31, 0,1, 0,1, 0,0,0,27, 0,0,0,1,
    0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,2, 0,0,0,0,0,0,0,77,
    0,0,0,0,0,0,0,9, 0,0,0,0,0,0,0,10, 0,0,0,0,0,0,0,0,
    23,0,0,0, 3,0,0,0, 4,0,0,0, 1,0,0,0,
    's','l','i','p','b', 2,0,0,0,'{','}'
  };
}
void wire_tests() {
  Frame frame; std::string error;
  const auto golden = golden_text();
  CHECK(decode_frame(golden, frame, &error));
  CHECK(frame.session_id == 1 && frame.sequence == 2 && frame.frame_number == 77);
  CHECK(frame.capture_timestamp_ns == 9 && frame.send_timestamp_ns == 10);
  CHECK(frame.packets[0].topic == "slip" && frame.packets[0].frame_id == "b");
  CHECK(std::get<Text>(frame.packets[0].payload).data == "{}");
  std::vector<std::uint8_t> encoded;
  CHECK(encode_frame(frame, encoded, &error)); CHECK(encoded == golden);

  for (std::size_t size = 0; size < golden.size(); ++size) {
    const std::vector<std::uint8_t> truncated(golden.begin(), golden.begin() + size);
    Frame unchanged; unchanged.frame_number = 555;
    CHECK(!decode_frame(truncated, unchanged, &error)); CHECK(unchanged.frame_number == 555);
  }
  auto corrupt = golden; corrupt[8] = 0xff;
  CHECK(!decode_frame(corrupt, frame, &error)); // Reject 4 GiB advertisement before body allocation.
  corrupt = golden; corrupt[15] = 33; CHECK(!decode_frame(corrupt, frame, &error));
  corrupt = golden; corrupt[56] = 1; CHECK(!decode_frame(corrupt, frame, &error));
  corrupt = golden; corrupt[68] = 99; CHECK(!decode_frame(corrupt, frame, &error));
  corrupt = golden; corrupt[72] = 0xff; CHECK(!decode_frame(corrupt, frame, &error));
  corrupt = golden; corrupt.push_back(0); CHECK(!decode_frame(corrupt, frame, &error));
  corrupt = golden; corrupt[golden.size() - 6] = 0xff; CHECK(!decode_frame(corrupt, frame, &error));
  Limits tiny; tiny.max_body_bytes = 26; CHECK(!decode_frame(golden, frame, &error, tiny));

  auto input = sample(); CHECK(encode_frame(input, encoded, &error));
  CHECK(encoded[16] == 1 && encoded[23] == 8); // Header uses network order.
  CHECK(decode_frame(encoded, frame, &error)); check_sample(frame);
  Limits bounds; bounds.max_points = 0; CHECK(!decode_frame(encoded, frame, &error, bounds));
  bounds = {}; bounds.max_image_width = 1; CHECK(!decode_frame(encoded, frame, &error, bounds));
  bounds = {}; bounds.max_text_bytes = 2; CHECK(!decode_frame(encoded, frame, &error, bounds));
  auto invalid = input; std::get<Image>(invalid.packets[0].payload).step = 0xffffffff;
  CHECK(!validate_frame(invalid));
  invalid = input; std::get<Image>(invalid.packets[1].payload).data.pop_back(); CHECK(!validate_frame(invalid));
  invalid = input; invalid.packets[0].topic = std::string("bad\0stream", 10); CHECK(!validate_frame(invalid));
  invalid = input; std::get<Image>(invalid.packets[0].payload).encoding = static_cast<ImageEncoding>(255);
  CHECK(!validate_frame(invalid));
  bounds = {}; bounds.max_body_bytes = 65U * 1024U * 1024U; CHECK(!validate_frame(input, bounds));
  auto heartbeat = encode_heartbeat(9, 123, 456);
  WireHeader h; CHECK(heartbeat.size() == 64 && decode_header(heartbeat.data(), heartbeat.size(), h));
  CHECK(h.kind == MessageKind::Heartbeat && h.session_id == 9 && h.sequence == 123 && h.send_timestamp_ns == 456);
  CHECK(!decode_frame(heartbeat, frame));
}

#if defined(__linux__) || defined(__APPLE__)
class Socket {
 public:
  explicit Socket(int value = -1) : fd(value) {}
  ~Socket() { if (fd >= 0) ::close(fd); }
  Socket(const Socket&) = delete; Socket& operator=(const Socket&) = delete;
  int fd;
};
bool until(const std::function<bool()>& predicate, int timeout_ms = 1500) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}
void send_bytes(int socket, const std::uint8_t* bytes, std::size_t size) {
  while (size) {
    const auto count = ::send(socket, bytes, size, MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) continue;
    CHECK(count > 0); bytes += count; size -= static_cast<std::size_t>(count);
  }
}
class Peer {
 public:
  explicit Peer(std::function<void(int)> handler) : listener(::socket(AF_INET, SOCK_STREAM, 0)) {
    CHECK(listener.fd >= 0);
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    CHECK(::bind(listener.fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    CHECK(::listen(listener.fd, 1) == 0);
    socklen_t size = sizeof(addr); CHECK(getsockname(listener.fd, reinterpret_cast<sockaddr*>(&addr), &size) == 0);
    port = ntohs(addr.sin_port);
    thread = std::thread([this, handler] {
      Socket client(::accept(listener.fd, nullptr, nullptr));
      if (client.fd >= 0) {
        timeval timeout{2, 0}; setsockopt(client.fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        try { handler(client.fd); } catch (...) { failure = std::current_exception(); }
      }
    });
  }
  ~Peer() { ::shutdown(listener.fd, SHUT_RDWR); if (thread.joinable()) thread.join(); }
  void finish() { if (thread.joinable()) thread.join(); if (failure) std::rethrow_exception(failure); }
  Socket listener;
  std::uint16_t port = 0;
  std::thread thread;
  std::exception_ptr failure;
};
ClientOptions for_port(std::uint16_t port) {
  ClientOptions options; options.port = port; options.reconnect_delay_ms = 10; return options;
}
void socketpair_test() {
  int fd[2]; CHECK(::socketpair(AF_UNIX, SOCK_STREAM, 0, fd) == 0);
  Socket sender(fd[0]), receiver(fd[1]);
  const auto expected = golden_text();
  std::thread writer([&] {
    for (auto byte : expected) { send_bytes(sender.fd, &byte, 1); std::this_thread::sleep_for(100us); }
    ::shutdown(sender.fd, SHUT_WR);
  });
  std::vector<std::uint8_t> received;
  std::uint8_t fragment[7];
  for (;;) {
    const auto size = ::recv(receiver.fd, fragment, sizeof(fragment), 0);
    CHECK(size >= 0); if (!size) break; received.insert(received.end(), fragment, fragment + size);
  }
  writer.join(); CHECK(received == expected);
  Frame frame; CHECK(decode_frame(received, frame)); CHECK(frame.frame_number == 77);
}
void fragmented_peer_test() {
  const auto bytes = golden_text();
  Peer server([&](int fd) {
    for (auto byte : bytes) { send_bytes(fd, &byte, 1); std::this_thread::sleep_for(1ms); }
    auto other = bytes; other[31] = 5; // Sequence 2 -> 5 reports two omitted frames.
    send_bytes(fd, other.data(), other.size());
    const auto heartbeat = encode_heartbeat(2, 0, 99);
    send_bytes(fd, heartbeat.data(), heartbeat.size());
  });
  Client client(for_port(server.port)); Frame frame; ReceiveInfo info;
  CHECK(client.receive(frame, info, 2000) == ReceiveResult::Frame);
  CHECK(info.connected && info.session_changed && !info.dropped_frames);
  CHECK(frame.frame_number == 77 && frame.capture_timestamp_ns == 9);
  CHECK(client.receive(frame, info, 1000) == ReceiveResult::Frame);
  CHECK(!info.session_changed && info.dropped_frames == 2);
  CHECK(client.receive(frame, info, 1000) == ReceiveResult::Heartbeat);
  CHECK(info.session_changed && info.header.session_id == 2);
  CHECK(client.receive(frame, info, 1000) == ReceiveResult::Error); CHECK(!info.connected);
  server.finish();
}
void invalid_and_partial_test() {
  {
    auto bytes = golden_text(); bytes[8] = 0xff;
    Peer server([&](int fd) { send_bytes(fd, bytes.data(), 64); });
    Client client(for_port(server.port)); Frame frame; ReceiveInfo info;
    CHECK(client.receive(frame, info, 1000) == ReceiveResult::Error);
    CHECK(!info.connected && !info.error.empty()); server.finish();
  }
  {
    std::atomic<bool> release{false};
    Peer server([&](int fd) {
      const auto bytes = golden_text(); send_bytes(fd, bytes.data(), 70);
      while (!release.load()) std::this_thread::sleep_for(1ms);
    });
    Client client(for_port(server.port)); Frame frame; ReceiveInfo info;
    const auto result = client.receive(frame, info, 120);
    release.store(true); server.finish();
    CHECK(result == ReceiveResult::Timeout && !info.connected && !info.error.empty());
  }
  {
    std::atomic<bool> release{false};
    Peer server([&](int) { while (!release.load()) std::this_thread::sleep_for(1ms); });
    Client client(for_port(server.port)); Frame frame; ReceiveInfo info;
    CHECK(client.receive(frame, info, 80) == ReceiveResult::Timeout); CHECK(info.connected && info.error.empty());
    ReceiveResult result = ReceiveResult::Error;
    std::thread reader([&] { result = client.receive(frame, info, 5000); });
    std::this_thread::sleep_for(30ms); const auto begin = std::chrono::steady_clock::now();
    client.stop(); reader.join(); release.store(true); server.finish();
    CHECK(result == ReceiveResult::Stopped);
    CHECK(std::chrono::steady_clock::now() - begin < 300ms);
  }
}
ReceiveResult next_frame(Client& client, Frame& frame, ReceiveInfo& info, int timeout_ms = 2000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto result = client.receive(frame, info, 200);
    if (result == ReceiveResult::Frame || result == ReceiveResult::Stopped) return result;
  }
  return ReceiveResult::Timeout;
}
void publisher_and_reconnect_test() {
  PublisherOptions options; options.port = 0; options.heartbeat_interval_ms = 30;
  AsyncPublisher publisher(options); std::string error; CHECK(publisher.start(&error)); CHECK(publisher.port());
  CHECK(!publisher.hasClient()); CHECK(!publisher.publish(sample())); CHECK(publisher.stats().published == 0);
  auto client_options = for_port(publisher.port());
  std::uint64_t first_session;
  {
    Client client(client_options); Frame output; ReceiveInfo info;
    CHECK(client.receive(output, info, 1000) == ReceiveResult::Heartbeat); CHECK(info.connected);
    CHECK(until([&] { return publisher.hasClient(); }));
    first_session = publisher.stats().session_id;
    CHECK(publisher.publish(sample(), &error));
    CHECK(next_frame(client, output, info) == ReceiveResult::Frame); check_sample(output);
    CHECK(output.session_id == first_session && output.sequence == 1);
    CHECK(output.send_timestamp_ns > output.capture_timestamp_ns);
    auto invalid = sample(); std::get<Image>(invalid.packets[0].payload).data.clear();
    CHECK(!publisher.publish(std::move(invalid), &error)); CHECK(publisher.stats().rejected == 1);
    client.stop();
  }
  CHECK(until([&] { return !publisher.hasClient() && publisher.stats().disconnects >= 1; }));
  {
    Client client(client_options); Frame output; ReceiveInfo info;
    CHECK(client.receive(output, info, 1000) == ReceiveResult::Heartbeat);
    CHECK(publisher.publish(sample())); CHECK(next_frame(client, output, info) == ReceiveResult::Frame);
    CHECK(output.sequence == 2 && output.session_id == first_session); check_sample(output);
    publisher.stop();
    CHECK(client.receive(output, info, 1000) == ReceiveResult::Error);
    CHECK(!info.connected);
    options.port = client_options.port;
    AsyncPublisher restarted(options); CHECK(restarted.start());
    ReceiveResult result = ReceiveResult::Error;
    CHECK(until([&] { result = client.receive(output, info, 200); return result == ReceiveResult::Heartbeat; }));
    CHECK(info.session_changed && restarted.stats().session_id != first_session);
    CHECK(restarted.publish(sample())); CHECK(next_frame(client, output, info) == ReceiveResult::Frame);
    CHECK(output.sequence == 1); restarted.stop();
  }
  publisher.stop(); // Idempotent.
  PublisherOptions disabled; disabled.enabled = false;
  AsyncPublisher off(disabled); CHECK(off.start()); CHECK(off.port() == 0 && !off.hasClient());
  CHECK(!off.publish(sample()) && off.stats().published == 0);
}
void slow_client_test() {
  PublisherOptions options; options.port = 0; options.send_timeout_ms = 200; options.heartbeat_interval_ms = 50;
  AsyncPublisher publisher(options); CHECK(publisher.start());
  Socket client(::socket(AF_INET, SOCK_STREAM, 0)); CHECK(client.fd >= 0);
  const int buffer = 4096; setsockopt(client.fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer));
  sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(publisher.port()); CHECK(::connect(client.fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  CHECK(until([&] { return publisher.hasClient(); }));
  Frame large; Image image; image.width = 2048; image.height = 2048; image.step = 2048;
  image.encoding = ImageEncoding::Mono8; image.data.resize(4 * 1024 * 1024, 255);
  large.packets.push_back(Packet{"mask_rgb", "camera", std::move(image)});
  CHECK(publisher.publish(std::move(large))); std::this_thread::sleep_for(40ms);
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000; ++i) {
    Frame frame; frame.frame_number = i; frame.packets.push_back(Packet{"slip", "base_link", Text{"{}"}});
    publisher.publish(std::move(frame));
  }
  CHECK(std::chrono::steady_clock::now() - begin < 500ms);
  CHECK(publisher.stats().replaced > 0); CHECK(until([&] { return !publisher.hasClient(); }));
  const auto stop_begin = std::chrono::steady_clock::now(); publisher.stop();
  CHECK(std::chrono::steady_clock::now() - stop_begin < 300ms);
}
#endif
} // namespace
int main() {
  try {
    wire_tests();
#if defined(__linux__) || defined(__APPLE__)
    socketpair_test(); fragmented_peer_test(); invalid_and_partial_test();
    publisher_and_reconnect_test(); slow_client_test();
#else
    PublisherOptions disabled; disabled.enabled = false;
    AsyncPublisher publisher(disabled); CHECK(publisher.start() && !publisher.hasClient());
    Client client; Frame frame; ReceiveInfo info; CHECK(client.receive(frame, info) == ReceiveResult::Error);
    client.stop(); CHECK(client.receive(frame, info) == ReceiveResult::Stopped);
#endif
    std::cout << "rviz wire/transport tests passed\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
