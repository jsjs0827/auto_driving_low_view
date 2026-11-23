#include "percep_demo/api_recharge_mark.h"
#include "recharge_mark_config.h"
#include "recharge_mark_logger.h"

#include <cstdarg>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <memory>

#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "common_interface/msg/dock_pose.hpp"

namespace Aiper_Recharge {
namespace {

constexpr float kDockPoseInvalidSentinel = 999.f;

}  // namespace

struct RechargeMarkHost::RosOut {
    rclcpp::Node* node = nullptr;
    rclcpp::Publisher<common_interface::msg::DockPose>::SharedPtr pose;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay;
};

RechargeMarkHost* RechargeMarkHost::Create() { return new RechargeMarkHost(); }

void RechargeMarkHost::Destroy() { delete this; }

RechargeMarkHost::~RechargeMarkHost() {
    Stop();
    ros_.reset();
    if (api_) {
        if (api_->destroy) api_->destroy(false);
        delete api_;
        api_ = nullptr;
    }
    recharge_log::Shutdown();
}

void RechargeMarkHost::logLine(const char* line) {
    if (!line || !line[0]) return;
    RM_LOG_INFO("%s", line);
    if (log_sink_) log_sink_(line);
    std::cout << line << std::endl;
}

void RechargeMarkHost::setLogSink(RechargeMarkHost::LogSink fn) {
    log_sink_ = fn;
}

void RechargeMarkHost::logf(const char* fmt, ...) {
    char buf[768];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    logLine(buf);
}

int RechargeMarkHost::Init(const char* sdk_so_dir, const char* start_or_cfg,
                           void* rclcpp_node) {
    recharge_log::Init(nullptr);
    logf("[recharge_mark] ===== init start (pid=%d) =====", ::getpid());
    if (!api_) api_ = createRechargeMarkApi();
    if (!api_ || !api_->init) return -1;
    std::string resolve_err;
    cfg_path_ = recharge_cfg::ResolveMarkJsonPath(start_or_cfg, &resolve_err);
    if (!resolve_err.empty()) {
        logf("[recharge_mark] resolve cfg: %s", resolve_err.c_str());
    }
    logf("[recharge_mark] cfg=%s", cfg_path_.c_str());
    const int rc =
        api_->init(sdk_so_dir ? sdk_so_dir : "", cfg_path_.c_str());
    if (rc != 0) {
        RM_LOG_ERROR("[recharge_mark] Init failed path=%s", cfg_path_.c_str());
    }
    if (api_->getHostConfig) api_->getHostConfig(&host_);
    host_.use_hires = true;
    if (api_->setResultCallback) {
        api_->setResultCallback(&RechargeMarkHost::OnResultThunk, this);
    }
    ros_.reset();
    if (rclcpp_node && host_.publish_enabled) {
        ros_ = std::make_unique<RosOut>();
        ros_->node = static_cast<rclcpp::Node*>(rclcpp_node);
        ros_->pose = ros_->node->create_publisher<common_interface::msg::DockPose>(
            kDockPoseTopic, rclcpp::SensorDataQoS());
        if (host_.publish_overlay) {
            const auto overlay_qos = rclcpp::QoS(rclcpp::KeepLast(5))
                                         .best_effort()
                                         .durability_volatile();
            ros_->overlay =
                ros_->node->create_publisher<sensor_msgs::msg::Image>(
                    kOverlayTopic, overlay_qos);
        }
        logf("[recharge_mark] ros pub pose=%s overlay=%s", kDockPoseTopic,
             host_.publish_overlay ? kOverlayTopic : "off");
    } else if (rclcpp_node) {
        logLine("[recharge_mark] publish_enabled=false, skip ROS publish");
    } else {
        logLine("[recharge_mark] no rclcpp node, skip ROS publish");
    }
    // 状态一次说清：等待感知仓 ToPercepCmd cmd=2/3 调 Start() 开始任务。
    logf("[recharge_mark] ready (wait ToPercepCmd cmd=2/3) "
         "pose=%s cfg=%s min_interval_ms=%u intrinsics=%s",
         kDockPoseTopic, cfg_path_.c_str(), host_.min_interval_ms,
         host_.intrinsics_source);
    return rc;
}

void RechargeMarkHost::setCameraK(int cal_width, int cal_height, double fx,
                                  double fy, double cx, double cy) {
    cam_k_ = {};
    cam_k_.cal_width = cal_width;
    cam_k_.cal_height = cal_height;
    cam_k_.fx = fx;
    cam_k_.fy = fy;
    cam_k_.cx = cx;
    cam_k_.cy = cy;
    have_cam_k_ = fx > 1.0 && fy > 1.0;
    logf("[recharge_mark] cameraK cal=%dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
         "valid=%d mode=%s",
         cal_width, cal_height, fx, fy, cx, cy, have_cam_k_ ? 1 : 0,
         needsSdkScaledK() ? "sdk_scaled" : "dataset(ignored)");
}

void RechargeMarkHost::publishPose(float x, float y, float yaw,
                                   int64_t stamp_ns) {
    if (!ros_ || !ros_->pose) return;
    common_interface::msg::DockPose msg;
    msg.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    msg.header.frame_id = "camera";
    msg.x = x;
    msg.y = y;
    msg.yaw = yaw;
    ros_->pose->publish(msg);
}

void RechargeMarkHost::publishOverlay(const uint8_t* bgr, int w, int h,
                                      int step, int64_t stamp_ns) {
    if (!ros_ || !ros_->overlay || !bgr || w <= 0 || h <= 0) return;
    const int ostep = step > 0 ? step : w * 3;
    sensor_msgs::msg::Image msg;
    msg.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    msg.header.frame_id = "camera";
    msg.height = static_cast<uint32_t>(h);
    msg.width = static_cast<uint32_t>(w);
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(w * 3);
    msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
    for (int y = 0; y < h; ++y) {
        const uint8_t* s = bgr + static_cast<size_t>(y) * ostep;
        uint8_t* d = msg.data.data() + static_cast<size_t>(y) * msg.step;
        for (int x = 0; x < w; ++x) {
            d[x * 3 + 0] = s[x * 3 + 2];
            d[x * 3 + 1] = s[x * 3 + 1];
            d[x * 3 + 2] = s[x * 3 + 0];
        }
    }
    ros_->overlay->publish(msg);
}

bool RechargeMarkHost::Start() {
    if (!api_ || !api_->startWorker) return false;
    abort_.store(false, std::memory_order_release);
    // 新任务开始先清下游：发一帧 999 无效 DockPose，让 dock 组件立刻
    // 作废上一轮任务残留的最后位姿（否则第一拍会沿用旧 pose）。
    publishInvalidPose("task start, clear stale dock pose");
    if (api_->resetSession) api_->resetSession();
    api_->startWorker();
    logf("[recharge_mark] task start: worker=on min_interval_ms=%u",
         host_.min_interval_ms);
    return true;
}

void RechargeMarkHost::Stop() {
    if (api_ && api_->stopWorker) api_->stopWorker();
    abort_.store(true, std::memory_order_release);
    done_cv_.notify_all();
    // 任务结束同样清下游：停止后不再有结果发布，不发这帧的话
    // dock 组件会一直持有本轮最后位姿直到下一轮 Start。
    publishInvalidPose("task stop, clear dock pose");
    RM_LOG_INFO("[recharge_mark] task stop: worker=off");
}

void RechargeMarkHost::publishInvalidPose(const char* why) {
    if (!ros_ || !ros_->pose) return;
    timespec ts{};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    const int64_t now_ns =
        static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    publishPose(kDockPoseInvalidSentinel, kDockPoseInvalidSentinel,
                kDockPoseInvalidSentinel, now_ns);
    logf("[recharge_mark] publish DockPose=INVALID(999) %s", why ? why : "");
}

void RechargeMarkHost::OnResultThunk(void* userdata, MarkPoseResult* result) {
    auto* self = static_cast<RechargeMarkHost*>(userdata);
    if (!self) return;
    if (result) self->onResult(result);
    self->done_seq_.fetch_add(1, std::memory_order_release);
    self->done_cv_.notify_all();
}

bool RechargeMarkHost::waitSettled(uint64_t done_before, int timeout_ms) {
    std::unique_lock<std::mutex> lk(done_mtx_);
    const bool ok = done_cv_.wait_for(
        lk, std::chrono::milliseconds(timeout_ms), [&] {
            return abort_.load(std::memory_order_acquire) ||
                   done_seq_.load(std::memory_order_acquire) > done_before;
        });
    if (!ok) {
        logf("[recharge_mark] settle timeout %dms", timeout_ms);
    }
    return ok;
}

int RechargeMarkHost::Tick(const void* gdc_frame, std::size_t gdc_bytes,
                           int settle_timeout_ms) {
    if (!api_ || !api_->submitFromShm) {
        return 1;
    }
    const uint64_t done_before = done_seq_.load(std::memory_order_acquire);
    const RechargeCameraK* pk =
        (needsSdkScaledK() && have_cam_k_) ? &cam_k_ : nullptr;
    const int sub_rc =
        api_->submitFromShm(nullptr, 0, gdc_frame, gdc_bytes, pk);
    if (settle_timeout_ms > 0 && sub_rc == 0) {
        waitSettled(done_before, settle_timeout_ms);
    }
    return sub_rc;
}

void RechargeMarkHost::onResult(MarkPoseResult* r) {
    if (!r || abort_.load(std::memory_order_acquire)) {
        return;
    }
    const int w = r->image_w;
    const int h = r->image_h;
    const int64_t stamp_ns = r->post.time_stamp;
    const double mark_ms = r->process_ms;
    const uint64_t fid = r->frame_id;

    if (host_.publish_overlay && r->ok) {
        if (r->overlay_bgr && r->overlay_w > 0 && r->overlay_h > 0) {
            const int ostep =
                r->overlay_step > 0 ? r->overlay_step : r->overlay_w * 3;
            publishOverlay(r->overlay_bgr, r->overlay_w, r->overlay_h, ostep,
                           stamp_ns);
        } else if ((++overlay_warn_tick_ % 30ull) == 1ull) {
            logLine("[recharge_mark] overlay enabled but empty");
        }
    }

    float px = kDockPoseInvalidSentinel;
    float py = kDockPoseInvalidSentinel;
    float pyaw = kDockPoseInvalidSentinel;
    if (r->pose_ok) {
        px = r->post.x;
        py = r->post.y;
        pyaw = r->post.yaw;
    }
    publishPose(px, py, pyaw, stamp_ns);

    if (r->pose_ok) {
        logf("[recharge_mark] fid=%llu gdc %dx%d mark_ms=%d cand=%d scaf=%d "
             "ov=%dx%d ok=1 pose_ok=1 pts=%d DockPose x=%.4f y=%.4f yaw=%.4f%s "
             "xf=%s stamp_ns=%lld dist_mm=%.0f",
             static_cast<unsigned long long>(fid), w, h,
             static_cast<int>(mark_ms), r->debug_candidates, r->debug_scaffolds,
             r->overlay_w, r->overlay_h, r->detected_point_count,
             static_cast<double>(px), static_cast<double>(py),
             static_cast<double>(pyaw),
             r->yaw_jump_hold ? " yaw_hold=1" : "",
             r->x_filter_reason[0] ? r->x_filter_reason : "-",
             static_cast<long long>(stamp_ns), r->distance_mm);
    } else if (r->dist_jump_hold) {
        logf("[recharge_mark] fid=%llu gdc %dx%d mark_ms=%d cand=%d scaf=%d "
             "ov=%dx%d ok=1 pose_ok=0 filter=dist_jump raw_y=%.4f dist_mm=%.0f",
             static_cast<unsigned long long>(fid), w, h,
             static_cast<int>(mark_ms), r->debug_candidates, r->debug_scaffolds,
             r->overlay_w, r->overlay_h, static_cast<double>(r->post.y),
             r->distance_mm);
    } else {
        char logbuf[640];
        int n = std::snprintf(
            logbuf, sizeof(logbuf),
            "[recharge_mark] fid=%llu gdc %dx%d mark_ms=%d cand=%d scaf=%d "
            "ov=%dx%d ok=%d pose_ok=0 pts=%d Pose=empty",
            static_cast<unsigned long long>(fid), w, h,
            static_cast<int>(mark_ms), r->debug_candidates, r->debug_scaffolds,
            r->overlay_w, r->overlay_h, r->ok ? 1 : 0, r->detected_point_count);
        if (n > 0 && n < static_cast<int>(sizeof(logbuf))) {
            if (r->error[0])
                n += std::snprintf(logbuf + n, sizeof(logbuf) - n, " err=%s",
                                   r->error);
            if (r->pnp_error[0])
                std::snprintf(logbuf + n, sizeof(logbuf) - n, " pnp=%s",
                              r->pnp_error);
        }
        logLine(logbuf);
    }
}

RechargeMarkHost* createRechargeMarkHost() { return RechargeMarkHost::Create(); }

void destroyRechargeMarkHost(RechargeMarkHost* p) {
    if (p) p->Destroy();
}

}  // namespace Aiper_Recharge
