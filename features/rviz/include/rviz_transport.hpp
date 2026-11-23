#pragma once

#include "rviz_wire.hpp"
#include <memory>
#include <string>

namespace perception::rviz {
struct PublisherOptions {
  bool enabled = true;
  std::string bind_address = "127.0.0.1";
  std::uint16_t port = 8765; // 0 requests an ephemeral port (useful in tests).
  int send_timeout_ms = 1000;
  int heartbeat_interval_ms = 1000;
  Limits limits;
};
struct PublisherStats {
  std::uint64_t session_id = 0;
  std::uint64_t published = 0, sent = 0, replaced = 0, rejected = 0;
  std::uint64_t connections = 0, disconnects = 0;
};
// One client, one replaceable latest slot, one in-flight frame. Socket writes,
// serialization and waits run on the worker; publish never waits for socket I/O.
class AsyncPublisher {
 public:
  explicit AsyncPublisher(PublisherOptions options = {});
  ~AsyncPublisher();
  AsyncPublisher(const AsyncPublisher&) = delete;
  AsyncPublisher& operator=(const AsyncPublisher&) = delete;
  bool start(std::string* error = nullptr);
  void stop();
  bool hasClient() const;
  // Takes ownership. Returns false when disabled/stopped/disconnected/invalid.
  // Accepted frames receive this server's session and monotonically increasing sequence.
  bool publish(Frame frame, std::string* error = nullptr);
  PublisherStats stats() const;
  std::uint16_t port() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct ClientOptions {
  std::string host = "127.0.0.1";
  std::uint16_t port = 18765;
  int connect_timeout_ms = 1000;
  int frame_timeout_ms = 2000;
  int reconnect_delay_ms = 200;
  Limits limits;
};
enum class ReceiveResult { Frame, Heartbeat, Timeout, Stopped, Error };
struct ReceiveInfo {
  WireHeader header;
  bool connected = false; // True even on a no-data timeout; false after partial-frame timeout.
  bool session_changed = false;
  std::uint64_t dropped_frames = 0; // Sequence gaps since the previous received frame.
  std::string error;
};
// receive is called by one consumer thread. stop may be called by another thread
// and interrupts active reads/connects. Partial timed-out frames force reconnect;
// malformed headers never allocate their advertised payload.
class Client {
 public:
  explicit Client(ClientOptions options = {});
  ~Client();
  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  ReceiveResult receive(Frame& frame, ReceiveInfo& info, int timeout_ms = 2000);
  void stop();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace perception::rviz
