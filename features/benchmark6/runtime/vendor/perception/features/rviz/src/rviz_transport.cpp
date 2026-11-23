#include "rviz_transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <random>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace perception::rviz {
namespace {
bool set_error(std::string* error, const std::string& message) {
  if (error) *error = message;
  return false;
}
bool valid_wire_limits(const Limits& limits, std::string* error) {
  const auto heartbeat = encode_heartbeat(1, 0, 0);
  WireHeader header;
  return decode_header(heartbeat.data(), heartbeat.size(), header, error, limits);
}
}

#if defined(__linux__) || defined(__APPLE__)
namespace {
using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;
Deadline after_ms(int value) { return Clock::now() + std::chrono::milliseconds(std::max(0, value)); }
int remaining_ms(Deadline deadline) {
  const auto count = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
  return count <= 0 ? 0 : static_cast<int>(std::min<std::int64_t>(count + 1, 2147483647));
}
void close_socket(int fd) { if (fd >= 0) ::close(fd); }
bool nonblocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) >= 0;
}
bool address(const std::string& host, std::uint16_t port, sockaddr_in& out) {
  out = {}; out.sin_family = AF_INET; out.sin_port = htons(port);
  return inet_pton(AF_INET, host == "localhost" ? "127.0.0.1" : host.c_str(), &out.sin_addr) == 1;
}
std::string socket_error(const char* operation) { return std::string(operation) + ": " + std::strerror(errno); }
// All sockets are nonblocking. Polls are sliced so cancellation does not depend
// on a platform's behavior when shutdown interrupts a connecting socket.
int wait_socket(int fd, short events, Deadline deadline, const std::atomic<bool>& active) {
  while (active.load()) {
    const int remaining = remaining_ms(deadline);
    if (!remaining) return 0;
    pollfd p{fd, events, 0};
    const int result = ::poll(&p, 1, std::min(remaining, 50));
    if (result < 0 && errno == EINTR) continue;
    if (result < 0) return -1;
    if (!result) continue;
    if (p.revents & events) return 1;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
  }
  return -2;
}
bool send_all(int fd, const std::vector<std::uint8_t>& data, int timeout_ms,
              const std::atomic<bool>& active) {
  const Deadline deadline = after_ms(timeout_ms);
  std::size_t sent = 0;
  while (sent < data.size() && active.load()) {
    if (!remaining_ms(deadline)) return false;
#if defined(MSG_NOSIGNAL)
    const auto count = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
#else
    const auto count = ::send(fd, data.data() + sent, data.size() - sent, 0);
#endif
    if (count > 0) { sent += static_cast<std::size_t>(count); continue; }
    if (count < 0 && errno == EINTR) continue;
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (wait_socket(fd, POLLOUT, deadline, active) == 1) continue;
    }
    return false;
  }
  return sent == data.size();
}
// Returns 1 complete, 0 timeout, -1 disconnect, -2 cancellation.
int read_all(int fd, std::uint8_t* data, std::size_t size, Deadline deadline,
             const std::atomic<bool>& active, std::size_t& transferred) {
  transferred = 0;
  while (transferred < size && active.load()) {
    if (!remaining_ms(deadline)) return 0;
    const auto count = ::recv(fd, data + transferred, size - transferred, 0);
    if (count > 0) { transferred += static_cast<std::size_t>(count); continue; }
    if (!count) return -1;
    if (errno == EINTR) continue;
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      const int result = wait_socket(fd, POLLIN, deadline, active);
      if (result == 1) continue;
      return result;
    }
    return -1;
  }
  return transferred == size ? 1 : -2;
}
std::uint64_t new_session() {
  std::uint64_t value = system_time_ns() ^ (std::uint64_t(::getpid()) << 32);
  try { std::random_device random; value ^= (std::uint64_t(random()) << 32) ^ random(); } catch (...) {}
  return value ? value : 1;
}
}

struct AsyncPublisher::Impl {
  explicit Impl(PublisherOptions value) : options(std::move(value)) {}
  PublisherOptions options;
  std::atomic<bool> running{false}, connected{false};
  std::atomic<std::uint16_t> bound_port{0};
  std::atomic<std::uint64_t> session{0}, sequence{0};
  std::atomic<std::uint64_t> published{0}, sent{0}, replaced{0}, rejected{0}, connections{0}, disconnects{0};
  std::mutex lifecycle_mutex, slot_mutex, socket_mutex;
  std::unique_ptr<Frame> latest;
  int listen_fd = -1, client_fd = -1;
  std::thread worker;

  void disconnect() {
    connected.store(false);
    { std::lock_guard<std::mutex> lock(socket_mutex);
      if (client_fd >= 0) { close_socket(client_fd); client_fd = -1; ++disconnects; }
    }
    std::lock_guard<std::mutex> lock(slot_mutex); latest.reset();
  }
  void run() {
    Deadline heartbeat_due = Clock::now();
    std::vector<std::uint8_t> bytes;
    while (running.load()) {
      int client;
      { std::lock_guard<std::mutex> lock(socket_mutex); client = client_fd; }
      pollfd descriptors[2]{{listen_fd, POLLIN, 0}, {client, POLLIN, 0}};
      const int result = ::poll(descriptors, client >= 0 ? 2 : 1, 20);
      if (result < 0 && errno != EINTR) break;
      if (!running.load()) break;
      if (result > 0 && (descriptors[0].revents & POLLIN)) {
        const int incoming = ::accept(listen_fd, nullptr, nullptr);
        if (incoming >= 0) {
          if (!nonblocking(incoming)) close_socket(incoming);
          else {
            disconnect();
            const int one = 1, buffer = 256 * 1024;
            setsockopt(incoming, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            setsockopt(incoming, SOL_SOCKET, SO_SNDBUF, &buffer, sizeof(buffer));
#if defined(SO_NOSIGPIPE)
            setsockopt(incoming, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
            { std::lock_guard<std::mutex> lock(socket_mutex); client_fd = incoming; }
            connected.store(true); ++connections; client = incoming; heartbeat_due = Clock::now();
          }
        }
      } else if (client >= 0 && result > 0 && descriptors[1].revents) {
        // This is a one-way protocol; any incoming data or EOF ends the session.
        disconnect(); client = -1;
      }
      if (client < 0 || !connected.load()) continue;
      std::unique_ptr<Frame> frame;
      { std::lock_guard<std::mutex> lock(slot_mutex); frame = std::move(latest); }
      bool ready = false;
      if (frame) {
        frame->send_timestamp_ns = system_time_ns();
        ready = encode_frame(*frame, bytes, nullptr, options.limits);
        if (!ready) ++rejected;
      } else if (Clock::now() >= heartbeat_due) {
        bytes = encode_heartbeat(session.load(), sequence.load(), system_time_ns()); ready = true;
      }
      if (ready) {
        if (!send_all(client, bytes, options.send_timeout_ms, running)) disconnect();
        else if (frame) ++sent;
        heartbeat_due = Clock::now() + std::chrono::milliseconds(options.heartbeat_interval_ms);
      }
    }
    running.store(false); disconnect();
  }
};
AsyncPublisher::AsyncPublisher(PublisherOptions options) : impl_(new Impl(std::move(options))) {}
AsyncPublisher::~AsyncPublisher() { stop(); }
bool AsyncPublisher::start(std::string* error) {
  auto& p = *impl_;
  std::lock_guard<std::mutex> lifecycle(p.lifecycle_mutex);
  if (!p.options.enabled) { if (error) error->clear(); return true; }
  if (p.running.load()) { if (error) error->clear(); return true; }
  if (p.worker.joinable()) p.worker.join();
  close_socket(p.listen_fd); p.listen_fd = -1;
  if (p.options.send_timeout_ms <= 0 || p.options.send_timeout_ms > 60000 ||
      p.options.heartbeat_interval_ms < 20 || p.options.heartbeat_interval_ms > 60000 ||
      !valid_wire_limits(p.options.limits, error)) return set_error(error, "invalid publisher options/limits");
  sockaddr_in bind_address;
  if (!address(p.options.bind_address, p.options.port, bind_address)) return set_error(error, "bind address must be numeric IPv4");
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return set_error(error, socket_error("socket"));
  const int one = 1; setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (!nonblocking(fd) || ::bind(fd, reinterpret_cast<sockaddr*>(&bind_address), sizeof(bind_address)) < 0 || ::listen(fd, 1) < 0) {
    const auto reason = socket_error("bind/listen"); close_socket(fd); return set_error(error, reason);
  }
  socklen_t length = sizeof(bind_address);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bind_address), &length) < 0) {
    const auto reason = socket_error("getsockname"); close_socket(fd); return set_error(error, reason);
  }
  p.listen_fd = fd; p.bound_port.store(ntohs(bind_address.sin_port));
  p.session.store(new_session()); p.sequence.store(0);
  p.published = 0; p.sent = 0; p.replaced = 0; p.rejected = 0; p.connections = 0; p.disconnects = 0;
  p.running.store(true);
  try { p.worker = std::thread([&p] { p.run(); }); }
  catch (const std::exception& e) {
    p.running.store(false); close_socket(p.listen_fd); p.listen_fd = -1;
    return set_error(error, e.what());
  }
  if (error) error->clear();
  return true;
}
void AsyncPublisher::stop() {
  auto& p = *impl_; std::lock_guard<std::mutex> lifecycle(p.lifecycle_mutex);
  p.running.store(false);
  { std::lock_guard<std::mutex> lock(p.socket_mutex);
    if (p.client_fd >= 0) ::shutdown(p.client_fd, SHUT_RDWR);
  }
  if (p.worker.joinable()) p.worker.join();
  close_socket(p.listen_fd); p.listen_fd = -1; p.bound_port.store(0);
  std::lock_guard<std::mutex> lock(p.slot_mutex); p.latest.reset();
}
bool AsyncPublisher::hasClient() const { return impl_->options.enabled && impl_->running.load() && impl_->connected.load(); }
bool AsyncPublisher::publish(Frame frame, std::string* error) {
  auto& p = *impl_;
  if (!hasClient()) return set_error(error, "publisher disabled/stopped/no client");
  if (!validate_frame(frame, p.options.limits, error)) { ++p.rejected; return false; }
  std::unique_ptr<Frame> owned(new Frame(std::move(frame)));
  std::lock_guard<std::mutex> lock(p.slot_mutex);
  if (!hasClient()) return set_error(error, "publisher disconnected");
  owned->session_id = p.session.load(); owned->sequence = ++p.sequence;
  if (p.latest) ++p.replaced;
  p.latest = std::move(owned); ++p.published;
  if (error) error->clear();
  return true;
}
PublisherStats AsyncPublisher::stats() const {
  const auto& p = *impl_; PublisherStats stats;
  stats.session_id = p.session.load(); stats.published = p.published.load(); stats.sent = p.sent.load();
  stats.replaced = p.replaced.load(); stats.rejected = p.rejected.load();
  stats.connections = p.connections.load(); stats.disconnects = p.disconnects.load(); return stats;
}
std::uint16_t AsyncPublisher::port() const { return impl_->bound_port.load(); }

struct Client::Impl {
  explicit Impl(ClientOptions value) : options(std::move(value)) {}
  ClientOptions options;
  std::atomic<bool> active{true};
  std::mutex socket_mutex, read_mutex;
  int fd = -1;
  Deadline next_connect = Clock::now();
  std::uint64_t session = 0, last_sequence = 0;
  void disconnect() {
    std::lock_guard<std::mutex> lock(socket_mutex);
    close_socket(fd); fd = -1; next_connect = after_ms(options.reconnect_delay_ms);
  }
  int connect(Deadline deadline, std::string& error) {
    while (active.load() && Clock::now() < next_connect) {
      if (!remaining_ms(deadline)) return 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(std::min(20, remaining_ms(deadline))));
    }
    if (!active.load()) return -2;
    sockaddr_in endpoint;
    if (!address(options.host, options.port, endpoint)) { error = "host must be numeric IPv4"; return -1; }
    const int socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket < 0) { error = socket_error("socket"); return -1; }
    if (!nonblocking(socket)) { error = socket_error("nonblocking"); close_socket(socket); return -1; }
    { std::lock_guard<std::mutex> lock(socket_mutex);
      if (!active.load()) { close_socket(socket); return -2; }
      fd = socket;
    }
    const int one = 1; setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    const int result = ::connect(socket, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint));
    if (result < 0 && errno != EINPROGRESS) { error = socket_error("connect"); disconnect(); return -1; }
    if (result < 0) {
      const int waited = wait_socket(socket, POLLOUT, std::min(deadline, after_ms(options.connect_timeout_ms)), active);
      if (waited != 1) { error = "connect timeout/disconnected"; disconnect(); return waited; }
      int failure = 0; socklen_t size = sizeof(failure);
      if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &failure, &size) < 0 || failure) {
        error = failure ? std::string("connect: ") + std::strerror(failure) : socket_error("getsockopt");
        disconnect(); return -1;
      }
    }
    return 1;
  }
};
Client::Client(ClientOptions options) : impl_(new Impl(std::move(options))) {}
Client::~Client() { stop(); std::lock_guard<std::mutex> lock(impl_->read_mutex); impl_->disconnect(); }
void Client::stop() {
  auto& p = *impl_; p.active.store(false); std::lock_guard<std::mutex> lock(p.socket_mutex);
  if (p.fd >= 0) ::shutdown(p.fd, SHUT_RDWR);
}
ReceiveResult Client::receive(Frame& frame, ReceiveInfo& info, int timeout_ms) {
  auto& p = *impl_; std::lock_guard<std::mutex> read_lock(p.read_mutex); info = {};
  struct ConnectionResult {
    ReceiveInfo& info; const int& fd; const std::atomic<bool>& active;
    ~ConnectionResult() { info.connected = active.load() && fd >= 0; }
  } connection_result{info, p.fd, p.active};
  if (!p.active.load()) return ReceiveResult::Stopped;
  if (timeout_ms < 0 || p.options.connect_timeout_ms <= 0 || p.options.frame_timeout_ms <= 0 ||
      p.options.reconnect_delay_ms < 0 || !p.options.port || !valid_wire_limits(p.options.limits, &info.error)) {
    info.error = "invalid client options/limits"; return ReceiveResult::Error;
  }
  const Deadline deadline = after_ms(timeout_ms);
  if (p.fd < 0) {
    const int connected = p.connect(deadline, info.error);
    if (connected != 1) return !p.active.load() ? ReceiveResult::Stopped : connected == 0 ? ReceiveResult::Timeout : ReceiveResult::Error;
  }
  const Deadline frame_deadline = std::min(deadline, after_ms(p.options.frame_timeout_ms));
  std::uint8_t header_bytes[kWireHeaderSize]; std::size_t transferred = 0;
  int result = read_all(p.fd, header_bytes, sizeof(header_bytes), frame_deadline, p.active, transferred);
  if (result != 1) {
    if (transferred || result < 0) p.disconnect();
    info.error = transferred ? "partial header timeout/disconnected" : result < 0 ? "disconnected" : "";
    return !p.active.load() ? ReceiveResult::Stopped : result == 0 ? ReceiveResult::Timeout : ReceiveResult::Error;
  }
  if (!decode_header(header_bytes, sizeof(header_bytes), info.header, &info.error, p.options.limits)) {
    p.disconnect(); return ReceiveResult::Error;
  }
  // Header has been fully validated before allocating any remote-sized buffer.
  if (info.header.kind == MessageKind::Frame) {
    std::vector<std::uint8_t> body(info.header.body_bytes);
    result = read_all(p.fd, body.data(), body.size(), frame_deadline, p.active, transferred);
    if (result != 1) {
      p.disconnect(); info.error = "partial body timeout/disconnected";
      return !p.active.load() ? ReceiveResult::Stopped : result == 0 ? ReceiveResult::Timeout : ReceiveResult::Error;
    }
    if (!decode_frame(info.header, body.data(), body.size(), frame, &info.error, p.options.limits)) {
      p.disconnect(); return ReceiveResult::Error;
    }
  }
  info.session_changed = p.session != info.header.session_id;
  if (info.session_changed) { p.session = info.header.session_id; p.last_sequence = 0; }
  if (info.header.kind == MessageKind::Frame) {
    if (p.last_sequence && info.header.sequence <= p.last_sequence) {
      info.error = "non-monotonic sequence"; p.disconnect(); return ReceiveResult::Error;
    }
    if (p.last_sequence) info.dropped_frames = info.header.sequence - p.last_sequence - 1;
    p.last_sequence = info.header.sequence;
    return ReceiveResult::Frame;
  }
  return ReceiveResult::Heartbeat;
}

#else
// Native Windows builds still validate/encode/decode the identical portable wire
// format. TCP transport runs on the Linux X5 and Ubuntu endpoints.
struct AsyncPublisher::Impl { explicit Impl(PublisherOptions v) : options(std::move(v)) {} PublisherOptions options; };
AsyncPublisher::AsyncPublisher(PublisherOptions options) : impl_(new Impl(std::move(options))) {}
AsyncPublisher::~AsyncPublisher() = default;
bool AsyncPublisher::start(std::string* error) {
  return !impl_->options.enabled || set_error(error, "RViz TCP transport requires Linux/POSIX");
}
void AsyncPublisher::stop() {}
bool AsyncPublisher::hasClient() const { return false; }
bool AsyncPublisher::publish(Frame, std::string* error) { return set_error(error, "RViz TCP transport requires Linux/POSIX"); }
PublisherStats AsyncPublisher::stats() const { return {}; }
std::uint16_t AsyncPublisher::port() const { return 0; }
struct Client::Impl { explicit Impl(ClientOptions v) : options(std::move(v)) {} ClientOptions options; std::atomic<bool> stopped{false}; };
Client::Client(ClientOptions options) : impl_(new Impl(std::move(options))) {}
Client::~Client() = default;
void Client::stop() { impl_->stopped = true; }
ReceiveResult Client::receive(Frame&, ReceiveInfo& info, int) {
  info = {}; info.error = "RViz TCP transport requires Linux/POSIX";
  return impl_->stopped ? ReceiveResult::Stopped : ReceiveResult::Error;
}
#endif
} // namespace perception::rviz
