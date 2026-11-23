#include "ab/runtime.hpp"
#include <array>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
namespace ab {
namespace {
using Clock = std::chrono::steady_clock;
struct Socket {
    int fd = -1;
    Socket() = default;
    explicit Socket(int n) : fd(n) {
    }
    ~Socket() {
        if (fd >= 0)
            ::close(fd);
    }
    Socket(const Socket &) = delete;
    Socket &operator=(const Socket &) = delete;
};
constexpr std::uint64_t magic = 0x3942414652414D45ULL;
using Header = std::array<std::uint64_t, 16>;
std::array<std::uint8_t, 128> encode(const Header &h) {
    std::array<std::uint8_t, 128> out{};
    for (std::size_t i = 0; i < 16; ++i)
        for (int j = 0; j < 8; ++j)
            out[i * 8 + j] = std::uint8_t(h[i] >> (j * 8));
    return out;
}
Header decode(const std::array<std::uint8_t, 128> &b) {
    Header h{};
    for (std::size_t i = 0; i < 16; ++i)
        for (int j = 0; j < 8; ++j)
            h[i] |= std::uint64_t(b[i * 8 + j]) << (j * 8);
    return h;
}
std::uint64_t bits(double d) {
    std::uint64_t n;
    std::memcpy(&n, &d, 8);
    return n;
}
double value(std::uint64_t n) {
    double d;
    std::memcpy(&d, &n, 8);
    return d;
}
void transfer(int fd, void *data, std::size_t n, bool send, int timeout) {
    auto *p = static_cast<std::uint8_t *>(data);
    auto deadline = Clock::now() + std::chrono::milliseconds(timeout);
    while (n && running) {
        pollfd f{fd, short(send ? POLLOUT : POLLIN), 0};
        int ready = ::poll(&f, 1, 100);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("TCP poll failed");
        }
        if (!ready) {
            if (Clock::now() > deadline)
                throw std::runtime_error("ROS bridge TCP payload timeout");
            continue;
        }
        auto count = send ? ::send(fd, p, n, MSG_NOSIGNAL) : ::recv(fd, p, n, 0);
        if (count <= 0) {
            if (count < 0 && (errno == EINTR || errno == EAGAIN))
                continue;
            throw std::runtime_error("ROS bridge connection closed/incomplete frame");
        }
        p += count;
        n -= std::size_t(count);
    }
    if (n)
        throw std::runtime_error("ROS bridge interrupted");
}
class Tcp final : public Source {
    Config c_;
    bool need_;
    Socket listen_, client_;

  public:
    Tcp(const Config &c, bool need) : c_(c), need_(need) {
        listen_.fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_.fd < 0)
            throw std::runtime_error("Cannot create ROS TCP input socket");
        int one = 1;
        ::setsockopt(listen_.fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(c.bridge_port);
        if (::bind(listen_.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
            ::listen(listen_.fd, 1))
            throw std::runtime_error("Cannot bind ROS bridge port " +
                                     std::to_string(c.bridge_port));
        auto deadline = Clock::now() + std::chrono::milliseconds(c.timeout_ms);
        while (running) {
            pollfd f{listen_.fd, POLLIN, 0};
            if (::poll(&f, 1, 100) > 0) {
                client_.fd = ::accept(listen_.fd, nullptr, nullptr);
                if (client_.fd >= 0)
                    return;
            }
            if (Clock::now() > deadline)
                throw std::runtime_error("Waiting for Ubuntu ROS bridge timed out");
        }
        throw std::runtime_error("ROS bridge accept interrupted");
    }
    Frame next() override {
        std::array<std::uint8_t, 128> b{};
        transfer(client_.fd, b.data(), b.size(), false, c_.timeout_ms);
        auto h = decode(b);
        auto image_bytes = std::uint64_t(c_.width) * c_.height * 3 / 2,
             qbytes = std::uint64_t(c_.width) * c_.height * 2;
        if (h[0] != magic || h[1] != 1 || !h[3] || h[4] != std::uint64_t(c_.width) ||
            h[5] != std::uint64_t(c_.height) || h[6] != image_bytes || h[7] != image_bytes ||
            (h[8] != 0 && h[8] != qbytes) || (need_ && h[8] != qbytes) || h[14] > 256 || h[15] != 0)
            throw std::invalid_argument("Invalid/incompatible ROS bridge frame header");
        Frame o;
        o.id = h[2];
        o.ns = h[3];
        o.k = {c_.width,     c_.height,    value(h[9]), value(h[10]),
               value(h[11]), value(h[12]), value(h[13])};
        validate(o.k);
        o.left = cv::Mat(c_.height * 3 / 2, c_.width, CV_8UC1);
        o.right = cv::Mat(c_.height * 3 / 2, c_.width, CV_8UC1);
        transfer(client_.fd, o.left.ptr(), image_bytes, false, c_.timeout_ms);
        transfer(client_.fd, o.right.ptr(), image_bytes, false, c_.timeout_ms);
        if (h[8]) {
            std::vector<std::uint8_t> q(qbytes);
            transfer(client_.fd, q.data(), q.size(), false, c_.timeout_ms);
            if (need_) {
                o.q8.resize(q.size() / 2);
                for (std::size_t i = 0; i < o.q8.size(); ++i)
                    o.q8[i] = std::uint16_t(q[i * 2]) | (std::uint16_t(q[i * 2 + 1]) << 8);
            }
        }
        o.key.resize(h[14]);
        if (h[14])
            transfer(client_.fd, o.key.data(), o.key.size(), false, c_.timeout_ms);
        return o;
    }
    std::string metadata() const override {
        return "{\"kind\":\"ros_via_cpp_tcp_bridge\",\"original_stamps_and_rectified_intrinsics\":"
               "true,\"publisher_and_bridge_resources_inside_worker\":false}";
    }
};
} // namespace
std::unique_ptr<Source> tcpSource(const Config &c, bool need) {
    return std::make_unique<Tcp>(c, need);
}
void bridgeForward(const Config &c, bool need) {
    Socket connection(::socket(AF_INET, SOCK_STREAM, 0));
    if (connection.fd < 0)
        throw std::runtime_error("Cannot create forward socket");
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(c.bridge_port);
    if (::inet_pton(AF_INET, c.bridge_host.c_str(), &a.sin_addr) != 1)
        throw std::invalid_argument("bridge_host must be IPv4 (127.0.0.1 for ADB forward)");
    auto deadline = Clock::now() + std::chrono::seconds(60);
    while (running && ::connect(connection.fd, reinterpret_cast<sockaddr *>(&a), sizeof(a))) {
        if (Clock::now() > deadline)
            throw std::runtime_error("Cannot connect to X5 bridge endpoint");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto input = nativeRosSource(c, need);
    for (int i = 0; i < c.warmup + c.frames && running; ++i) {
        auto f = input->next();
        if (f.key.size() > 256)
            throw std::invalid_argument("Frame key exceeds bridge limit");
        Header h{magic,
                 1,
                 f.id,
                 f.ns,
                 std::uint64_t(f.k.w),
                 std::uint64_t(f.k.h),
                 f.left.total(),
                 f.right.total(),
                 f.q8.size() * 2,
                 bits(f.k.fx),
                 bits(f.k.fy),
                 bits(f.k.cx),
                 bits(f.k.cy),
                 bits(f.k.bxf),
                 f.key.size(),
                 0};
        auto encoded = encode(h);
        transfer(connection.fd, encoded.data(), encoded.size(), true, c.timeout_ms);
        transfer(connection.fd, f.left.ptr(), f.left.total(), true, c.timeout_ms);
        transfer(connection.fd, f.right.ptr(), f.right.total(), true, c.timeout_ms);
        if (!f.q8.empty()) {
            std::vector<std::uint8_t> q(f.q8.size() * 2);
            for (std::size_t n = 0; n < f.q8.size(); ++n) {
                q[n * 2] = f.q8[n] & 255;
                q[n * 2 + 1] = f.q8[n] >> 8;
            }
            transfer(connection.fd, q.data(), q.size(), true, c.timeout_ms);
        }
        if (!f.key.empty())
            transfer(connection.fd, f.key.data(), f.key.size(), true, c.timeout_ms);
    }
}
} // namespace ab
