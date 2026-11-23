#include "ab/runtime.hpp"
#include "r36_stereo/nv12.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>
#include <optional>
namespace ab {
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Clock = std::chrono::steady_clock;
static std::uint64_t stamp(const builtin_interfaces::msg::Time &s) {
    if (s.sec < 0 || s.nanosec >= 1000000000)
        throw std::invalid_argument("Invalid ROS timestamp");
    return std::uint64_t(s.sec) * 1000000000 + s.nanosec;
}
static cv::Mat nv12(const Image &m, int w, int h) {
    if (m.width != unsigned(w) || m.height != unsigned(h) || m.step < unsigned(w) ||
        m.data.size() < std::size_t(m.step) * h)
        throw std::invalid_argument("ROS image geometry/stride mismatch");
    if (m.encoding == "nv12" || m.encoding == "i420")
        return r36stereo::copy_yuv420_to_nv12(m.data.data(), w, h, m.step, m.data.size(),
                                              m.encoding == "nv12");
    if (m.encoding != "rgb8" && m.encoding != "bgr8")
        throw std::invalid_argument("ROS left/right supports nv12, i420, rgb8 or bgr8");
    if (m.step < unsigned(w * 3))
        throw std::invalid_argument("RGB stride too small");
    cv::Mat bgr(h, w, CV_8UC3, const_cast<std::uint8_t *>(m.data.data()), m.step);
    if (m.encoding == "rgb8") {
        cv::Mat copy;
        cv::cvtColor(bgr, copy, cv::COLOR_RGB2BGR);
        return r36stereo::bgr_to_nv12(copy);
    }
    return r36stereo::bgr_to_nv12(bgr);
}
class Ros final : public Source {
    struct Pair {
        Image::ConstSharedPtr l, r, q;
        Info::ConstSharedPtr li, ri;
    };
    Config c_;
    bool need_;
    rclcpp::Node::SharedPtr node_;
    std::map<std::uint64_t, Pair> cache_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    rclcpp::Publisher<Image>::SharedPtr acknowledgment_;
    std::uint64_t last_ = 0, sequence_ = 0, dropped_ = 0;
    std::string error_;
    template <class T, class Fn> void sub(const std::string &topic, Fn fn) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(c_.cache_size));
        if (c_.ros_reliable)
            qos.reliable();
        else
            qos.best_effort();
        subs_.push_back(
            node_->create_subscription<T>(topic, qos, [this, fn](typename T::ConstSharedPtr m) {
                try {
                    auto ns = stamp(m->header.stamp);
                    if (!ns || ns <= last_)
                        return;
                    fn(cache_[ns], m);
                    while (cache_.size() > std::size_t(c_.cache_size)) {
                        cache_.erase(cache_.begin());
                        ++dropped_;
                    }
                } catch (const std::exception &e) {
                    error_ = e.what();
                }
            }));
    }

  public:
    Ros(const Config &c, bool need) : c_(c), need_(need) {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("ab_benchmark_input");
        if (c_.ros_acknowledged) acknowledgment_ = node_->create_publisher<Image>(c_.ros_ack_topic, rclcpp::QoS(4).reliable());
        sub<Image>(c.left_topic, [](Pair &p, auto m) { p.l = m; });
        sub<Image>(c.right_topic, [](Pair &p, auto m) { p.r = m; });
        if (need)
            sub<Image>(c.q8_topic, [](Pair &p, auto m) { p.q = m; });
        sub<Info>(c.left_info_topic, [](Pair &p, auto m) { p.li = m; });
        sub<Info>(c.right_info_topic, [](Pair &p, auto m) { p.ri = m; });
    }
    Frame next() override {
        auto deadline = Clock::now() + std::chrono::milliseconds(c_.timeout_ms);
        while (running && rclcpp::ok()) {
            rclcpp::spin_some(node_);
            if (!error_.empty())
                throw std::runtime_error(error_);
            for (auto it = cache_.begin(); it != cache_.end(); ++it) {
                auto &p = it->second;
                if (!p.l || !p.r || !p.li || !p.ri || (need_ && !p.q))
                    continue;
                if (p.li->width != unsigned(c_.width) || p.li->height != unsigned(c_.height) ||
                    p.ri->width != unsigned(c_.width) || p.ri->height != unsigned(c_.height))
                    throw std::invalid_argument("ROS CameraInfo dimensions differ from image");
                Frame o;
                o.ns = it->first;
                o.id = ++sequence_;
                auto marker = p.l->header.frame_id.find("#sdk_id=");
                if (marker != std::string::npos)
                    o.id = std::stoull(p.l->header.frame_id.substr(marker + 8));
                o.key = "ros-" + std::to_string(o.ns);
                o.left = nv12(*p.l, c_.width, c_.height);
                o.right = nv12(*p.r, c_.width, c_.height);
                o.k = {c_.width,   c_.height,  p.li->p[0],          p.li->p[5],
                       p.li->p[2], p.li->p[6], -p.ri->p[3] * 1000.0};
                validate(o.k);
                if (std::abs(p.li->p[0] - p.ri->p[0]) > 1e-5 ||
                    std::abs(p.li->p[5] - p.ri->p[5]) > 1e-5)
                    throw std::invalid_argument("Rectified stereo CameraInfo focal lengths differ");
                if (need_) {
                    auto &q = *p.q;
                    if (q.encoding != "16UC1" || q.width != unsigned(c_.width) ||
                        q.height != unsigned(c_.height) || q.step < unsigned(c_.width * 2) ||
                        q.data.size() < std::size_t(q.step) * c_.height)
                        throw std::invalid_argument(
                            "ROS Q8 must be original 16UC1 Q8.8, not depth or mono8");
                    o.q8.resize(std::size_t(c_.width) * c_.height);
                    for (int y = 0; y < c_.height; ++y)
                        for (int x = 0; x < c_.width; ++x) {
                            auto i = std::size_t(y) * q.step + x * 2;
                            auto a = q.data[i], b = q.data[i + 1];
                            o.q8[std::size_t(y) * c_.width + x] =
                                q.is_bigendian ? (a << 8) | b : a | (b << 8);
                        }
                }
                if (acknowledgment_) { Image ack; ack.header = p.l->header; acknowledgment_->publish(ack); }
                last_ = o.ns;
                cache_.erase(cache_.begin(), std::next(it));
                return o;
            }
            if (Clock::now() > deadline)
                throw std::runtime_error("ROS exact-stamp input timeout: need left/right + both "
                                         "CameraInfo + Q8 for Q8 methods; check QoS and topics");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        throw std::runtime_error("ROS interrupted");
    }
    std::string metadata() const override {
        return "{\"kind\":\"ros\",\"synchronization\":\"exact_original_capture_stamp_for_all_"
               "required_messages\",\"cache_evictions\":" +
               std::to_string(dropped_) + ",\"q8_producer_inside_worker\":false}";
    }
};
std::unique_ptr<Source> nativeRosSource(const Config &c, bool need) {
    return std::make_unique<Ros>(c, need);
}
void rosPublish(const Config &c, const std::string &source, bool need) {
    if (source != "camera" && source != "offline")
        throw std::invalid_argument("Publisher source must be camera/offline");
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("ab_benchmark_source");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(c.cache_size));
    if (c.ros_reliable)
        qos.reliable();
    else
        qos.best_effort();
    std::uint64_t acknowledged = 0;
    auto ack_subscription = c.ros_acknowledged ? node->create_subscription<Image>(c.ros_ack_topic, rclcpp::QoS(4).reliable(), [&](Image::ConstSharedPtr message) { acknowledged = stamp(message->header.stamp); }) : rclcpp::Subscription<Image>::SharedPtr{};
    auto l = node->create_publisher<Image>(c.left_topic, qos),
         r = node->create_publisher<Image>(c.right_topic, qos),
         q = node->create_publisher<Image>(c.q8_topic, qos);
    auto li = node->create_publisher<Info>(c.left_info_topic, qos),
         ri = node->create_publisher<Info>(c.right_info_topic, qos);
    auto deadline = Clock::now() + std::chrono::seconds(60);
    while (running && (l->get_subscription_count() == 0 || r->get_subscription_count() == 0 ||
                       li->get_subscription_count() == 0 || ri->get_subscription_count() == 0 ||
                       (need && q->get_subscription_count() == 0))) {
        rclcpp::spin_some(node);
        if (Clock::now() > deadline)
            throw std::runtime_error("No complete ROS subscriber after 60 seconds");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    auto input = source == "camera" ? cameraSource(c, need) : offlineSource(c, need);
    for (int i = 0; i < c.frames + c.warmup && running; ++i) {
        auto begin = Clock::now();
        auto f = input->next();
        std_msgs::msg::Header h;
        h.stamp.sec = std::int32_t(f.ns / 1000000000);
        h.stamp.nanosec = f.ns % 1000000000;
        h.frame_id = "camera_optical_frame#sdk_id=" + std::to_string(f.id);
        auto image = [&](const cv::Mat &mat) {
            Image m;
            m.header = h;
            m.width = c.width;
            m.height = c.height;
            m.encoding = "nv12";
            m.step = c.width;
            m.data.assign(mat.ptr(), mat.ptr() + mat.total());
            return m;
        };
        auto info = [&] {
            Info m;
            m.header = h;
            m.width = c.width;
            m.height = c.height;
            m.distortion_model = "plumb_bob";
            m.d = {0, 0, 0, 0, 0};
            m.k = {f.k.fx, 0, f.k.cx, 0, f.k.fy, f.k.cy, 0, 0, 1};
            m.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            m.p = {f.k.fx, 0, f.k.cx, 0, 0, f.k.fy, f.k.cy, 0, 0, 0, 1, 0};
            return m;
        };
        auto left_info = info(), right_info = info();
        right_info.p[3] = -f.k.bxf / 1000.0;
        li->publish(left_info);
        ri->publish(right_info);
        l->publish(image(f.left));
        r->publish(image(f.right));
        if (need) {
            Image m;
            m.header = h;
            m.width = c.width;
            m.height = c.height;
            m.encoding = "16UC1";
            m.step = c.width * 2;
            m.data.resize(f.q8.size() * 2);
            for (std::size_t n = 0; n < f.q8.size(); ++n) {
                m.data[n * 2] = f.q8[n] & 255;
                m.data[n * 2 + 1] = f.q8[n] >> 8;
            }
            q->publish(m);
        }
        rclcpp::spin_some(node);
        if (c.ros_acknowledged) {
            auto ack_deadline = Clock::now() + std::chrono::milliseconds(c.timeout_ms);
            while (running && acknowledged != f.ns) {
                rclcpp::spin_some(node);
                if (Clock::now() > ack_deadline) throw std::runtime_error("ROS replay acknowledgement timed out");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        auto end = begin + std::chrono::duration_cast<Clock::duration>(
                               std::chrono::duration<double>(1.0 / c.ros_publish_hz));
        while (running && Clock::now() < end)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Keep the publisher process alive for the worker's final same-window /proc sample.
    while (running && rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
} // namespace ab
