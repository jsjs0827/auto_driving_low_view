/**
 * @file percep_demo.cpp
 * @brief Minimal pub/sub/timer function example.
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved.
 */
#include "percep_demo/percep_demo.h"
#include "percep_demo/ai_fusion.hpp"
#include "percep_demo/camera_shm_adapter.hpp"
#include "percep_demo/charger_crop.hpp"
#include "percep_demo/patchworkpp.hpp"
#include "percep_demo/pcl_common.hpp"
#include "percep_demo/visualize.hpp"
#include "topic_macros.h"

#include <common_interface/msg/dock_pose.hpp>
#include <common_interface/msg/grid_map_info.hpp>
#include <common_interface/srv/to_app_cmd.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <new>
#include <sstream>
#include <system_error>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

using namespace std;
using namespace rapidjson;
using namespace cv;

namespace fs = std::filesystem;

static std::mutex resize_mtx; // 全局静态锁

namespace {

std::string dirnameOf(const std::string& path) {
    const auto pos = path.find_last_of('/');
    if (pos == std::string::npos) return ".";
    if (pos == 0) return "/";
    return path.substr(0, pos);
}

std::string resolveConfigPath(const std::string& base_file,
                              const std::string& maybe_rel) {
    if (maybe_rel.empty()) return maybe_rel;
    if (maybe_rel[0] == '/') return maybe_rel;
    return dirnameOf(base_file) + "/" + maybe_rel;
}

}  // namespace


REGISTER_FUNCTION(factory) {
  cout << "[PercepDemo] register collect function, function_name:" << FUNCTION_NAME << endl;
  factory.registerCreater<ai_node_percep_demo::PercepDemo>(FUNCTION_NAME);
}

namespace ai_node_percep_demo {

void RechargeApiDeleter::operator()(
    Aiper_Recharge::RechargeMarkApi* api) const noexcept {
    if (api == nullptr) {
        return;
    }
    if (api->destroy != nullptr) {
        api->destroy(false);
    }
    delete api;
}

PercepDemo::PercepDemo(const rapidjson::Value& json_conf) : daye::Function(json_conf) {
    global_json.CopyFrom(json_conf, global_json.GetAllocator());
    sub_callback_group_ =
        daye::CreateCallbackGroup(daye::GroupType::MutuallyExclusive);
    if(!this->initParams()) { cerr<<"参数初始化失败！"<<endl;return; }
    if(!this->initInterface()) { cerr<<"接口初始化失败！"<<endl;return; }
    if (offline)
    {
        cout<<"进入离线测试模式"<<endl;
        this->offline_loop();    
    }
    //-----------------ros service-------------------//
    if (!self_test)
    {
        percepStatus_server_ = daye::CreateServer<common_interface::srv::GetPercepStatus>(kGetPercepStatus, &PercepDemo::percepStatus_callback,this);  
        percepCmd_server_ = daye::CreateServer<common_interface::srv::ToPercepCmd>(kToPercepCmd, &PercepDemo::percepCmd_callback,this);
        appCmd_client_ = daye::CreateClient<common_interface::srv::ToAppCmd>(kToApplicationCmd);  
        
        //感知初始化完毕，进入待机
        percep_status_.store(0, std::memory_order_release);
    }
    else{
        //1.RGB 2.GDC 3.同时启动 4.全部停止
        std_msgs::msg::UInt8 camera_request;
        camera_request.data = 3;
        camera_request_pub.publish(camera_request);
        work_mode_.store(WorkMode::kMapAndRecharge,
                         std::memory_order_release);
    }
    if (!offline && !startOnlineRuntime()) {
        percep_status_.store(-1, std::memory_order_release);
        cerr << "[PercepDemo] 在线输入启动失败" << endl;
    }
}

PercepDemo::~PercepDemo() {
    cout << "[PercepDemo] ~PercepDemo" << endl;
    shutting_down_.store(true, std::memory_order_release);
    work_mode_.store(WorkMode::kIdle, std::memory_order_release);
    rgb_frame_channel_.stop();
    gdc_frame_channel_.stop();
    mark_done_cv_.notify_all();
    if(offline){
        g_running = false;
    }
    else{
        timer_.stop();
        // 停 subscriber 后才等待 timer，生产者不会再向已关闭 Channel 发布。
        rgb_subscriber_ = daye::ShmSubscriber{};
        gdc_subscriber_ = daye::ShmSubscriber{};
    }

    {
        // 等当前 onTimer（含 BPU 推理）结束，再释放模型和回充接口。
        std::lock_guard<std::mutex> wait_timer(timer_run_mtx_);
#if defined(PERCEP_DNN_ENABLE)
        dnn_runner_.reset();
#endif
    }
    stopRechargeMarkWorker();
    recharge_api_.reset();
    save_map_exit_ = true;
    cv_save_map.notify_all();
    if (saveLogThreadHandle.joinable()) {
        saveLogThreadHandle.join();
    }
    if (!offline) {
        RCLCPP_WARN(rclcpp::get_logger("exit"), "perception exit!");
    }
}

void PercepDemo::onCameraFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info) {
    if (shutting_down_.load(std::memory_order_acquire)) {
        return;
    }
    if (info.size != size) {
        const auto invalid = invalid_rgb_frames_.fetch_add(
                                 1, std::memory_order_relaxed) +
                             1;
        if ((invalid % 30U) == 1U) {
            LOG_ERROR("[PercepDemo] reject RGBD envelope size "
                      "metadata=%zu actual=%zu invalid=%llu",
                      static_cast<std::size_t>(info.size), size,
                      static_cast<unsigned long long>(invalid));
        }
        return;
    }

    OwnedRgbdFrame owned;
    const auto copied = copyRgbdFrameFromShm(
        data, size, static_cast<std::uint64_t>(info.sequence),
        static_cast<std::int64_t>(info.timestamp_ns), &owned);
    if (!copied) {
        const auto invalid = invalid_rgb_frames_.fetch_add(
                                 1, std::memory_order_relaxed) +
                             1;
        if ((invalid % 30U) == 1U) {
            const auto layout = cameraShmLayout();
            LOG_ERROR("[PercepDemo] reject RGBD frame status=%s validation=%s "
                      "expected=%zu actual=%zu invalid=%llu",
                      frameCopyStatusName(copied.status),
                      frameValidationErrorName(copied.validation_error),
                      layout.rgbd_message_bytes, size,
                      static_cast<unsigned long long>(invalid));
        }
        return;
    }

#if defined(PERCEP_DNN_ENABLE)
    owned.left.format =
        resolveLeftYuvFormat(owned.left, DnnImageSource::kRgbd);
#else
    owned.left.format = Yuv420Format::kNv12;
#endif

    try {
        RgbdFramePtr frame =
            std::make_shared<const OwnedRgbdFrame>(std::move(owned));
        if (rgb_frame_channel_.publish(std::move(frame))) {
            rgb_camera_ready_.store(true, std::memory_order_release);
        }
    } catch (const std::bad_alloc&) {
        invalid_rgb_frames_.fetch_add(1, std::memory_order_relaxed);
        LOG_ERROR("[PercepDemo] reject RGBD frame: shared_ptr allocation failed");
    }
}

void PercepDemo::onGdcFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info) {
    if (shutting_down_.load(std::memory_order_acquire)) {
        return;
    }
    if (info.size != size) {
        const auto invalid = invalid_gdc_frames_.fetch_add(
                                 1, std::memory_order_relaxed) +
                             1;
        if ((invalid % 30U) == 1U) {
            LOG_ERROR("[PercepDemo] reject GDC envelope size "
                      "metadata=%zu actual=%zu invalid=%llu",
                      static_cast<std::size_t>(info.size), size,
                      static_cast<unsigned long long>(invalid));
        }
        return;
    }

    OwnedGdcFrame owned;
    const auto copied = copyGdcFrameFromShm(
        data, size, static_cast<std::uint64_t>(info.sequence),
        static_cast<std::int64_t>(info.timestamp_ns), &owned);
    if (!copied) {
        const auto invalid = invalid_gdc_frames_.fetch_add(
                                 1, std::memory_order_relaxed) +
                             1;
        if ((invalid % 30U) == 1U) {
            const auto layout = cameraShmLayout();
            LOG_ERROR("[PercepDemo] reject GDC frame status=%s validation=%s "
                      "expected=%zu actual=%zu invalid=%llu",
                      frameCopyStatusName(copied.status),
                      frameValidationErrorName(copied.validation_error),
                      layout.gdc_message_bytes, size,
                      static_cast<unsigned long long>(invalid));
        }
        return;
    }

#if defined(PERCEP_DNN_ENABLE)
    owned.left.format =
        resolveLeftYuvFormat(owned.left, DnnImageSource::kGdc);
#else
    owned.left.format = Yuv420Format::kNv12;
#endif

    try {
        GdcFramePtr frame =
            std::make_shared<const OwnedGdcFrame>(std::move(owned));
        if (gdc_frame_channel_.publish(std::move(frame))) {
            gdc_camera_ready_.store(true, std::memory_order_release);
        }
    } catch (const std::bad_alloc&) {
        invalid_gdc_frames_.fetch_add(1, std::memory_order_relaxed);
        LOG_ERROR("[PercepDemo] reject GDC frame: shared_ptr allocation failed");
    }
}

void PercepDemo::onCamParams(const common_interface::msg::CameraParam &params)
{
    cout<<"收到相机参数：fx:"<<params.fx<<" fy:"<<params.fy<<" bxf:"<<params.bxf<<" base_line:"<<params.base_line<<" width:"<<params.width<<" height:"<<params.height<<endl;
    const auto width = static_cast<std::uint64_t>(params.width);
    const auto height = static_cast<std::uint64_t>(params.height);
    if (width == 0 || height == 0 ||
        width > static_cast<std::uint64_t>(kMaxCameraFrameDimension) ||
        height > static_cast<std::uint64_t>(kMaxCameraFrameDimension)) {
        LOG_ERROR("[PercepDemo] reject CameraParam dimensions=%llux%llu",
                  static_cast<unsigned long long>(width),
                  static_cast<unsigned long long>(height));
        return;
    }
    CameraIntrinsics intrinsics;
    intrinsics.calibration_width = static_cast<int>(width);
    intrinsics.calibration_height = static_cast<int>(height);
    intrinsics.fx = params.fx;
    intrinsics.fy = params.fy;
    intrinsics.cx = params.cx;
    intrinsics.cy = params.cy;
    intrinsics.bxf_m = params.bxf / 1000.0F;
    intrinsics.baseline_m = params.base_line;
    if (!intrinsics.depthValid()) {
        // Keep the previous valid snapshot. Replacing it with a malformed
        // asynchronous update would make every following map frame fail.
        LOG_ERROR("[PercepDemo] reject invalid CameraParam intrinsics");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(camera_params_mtx_);
        camera_intrinsics_ = intrinsics;
    }

    if (save_log){
        this->saveCamParams(params);
     
    }

}

WorkMode PercepDemo::currentWorkMode() const noexcept {
    return work_mode_.load(std::memory_order_acquire);
}

std::optional<CameraIntrinsics> PercepDemo::cameraIntrinsicsSnapshot() const {
    std::lock_guard<std::mutex> lock(camera_params_mtx_);
    return camera_intrinsics_;
}

bool PercepDemo::percepStatus_callback(common_interface::srv::GetPercepStatus::Request& req,common_interface::srv::GetPercepStatus::Response& res)
{
  //查询感知状态
    if (req.cmd == 1)
    {
        cout<<"[PercepDemo] 收到感知查询请求"<<endl;
        // -1 - 异常，0 - 待机中，1 - 感知地图发布中，2 - 基站二维码发布中 3 - 同时发布二维码和地图
        res.result = percep_status_.load(std::memory_order_acquire);
        cout<<"感知状态："<<res.result<<" (-1 - 异常，0 - 待机中，1 - 感知地图发布中，2 - 基站二维码发布中，3-同时发布二维码和地图)"<<endl;
    }
    else
    {
       cerr<<"[PercepDemo err] 接收查询指令异常！"<<endl;
    }
    return true;
}
bool PercepDemo::percepCmd_callback(common_interface::srv::ToPercepCmd::Request& req,common_interface::srv::ToPercepCmd::Response& res)
{
    // The framework may dispatch service calls concurrently. Treat a mode
    // change as one transaction instead of interleaving two worker/channel
    // state machines.
    std::lock_guard<std::mutex> command_lock(mode_transition_mtx_);
    if (req.cmd == 1)
    {
        cout<<"[PercepDemo] 收到启动感知地图发布请求"<<endl;
        res.result = 0;
        work_mode_.store(WorkMode::kIdle, std::memory_order_release);
        // Wait for a tick that captured the previous mode to finish. While
        // this lock is held no submit, DNN or map publication can overlap the
        // worker/channel transition below.
        std::unique_lock<std::mutex> timer_transition(timer_run_mtx_);
        // A frame left by the previous work mode must not become the first
        // frame of the new session.  discardPending() keeps the channel open;
        // stop() is reserved for object teardown and cannot be undone.
        rgb_frame_channel_.discardPending();
        gdc_frame_channel_.discardPending();
        stopRechargeMarkWorker();
        work_mode_.store(WorkMode::kMap, std::memory_order_release);
        std_msgs::msg::UInt8 camera_request;
        camera_request.data = 1;
        camera_request_pub.publish(camera_request);
        timer_transition.unlock();
        this->callAppService(2001,1,10000);
    }
    else if (req.cmd == 2)
    {
        cout << "[PercepDemo] 收到基站二维码发布模块启动指令" << endl;
        LOG_INFO("[PercepDemo] 收到基站二维码发布模块启动指令");
        res.result = 0;
        work_mode_.store(WorkMode::kIdle, std::memory_order_release);
        std::unique_lock<std::mutex> timer_transition(timer_run_mtx_);
        rgb_frame_channel_.discardPending();
        gdc_frame_channel_.discardPending();
        stopRechargeMarkWorker();
        if (!recharge_host_.enabled || !recharge_host_.publish_enabled) {
            cerr << "[PercepDemo] recharge_mark: disabled in config" << endl;
            LOG_WARN("[PercepDemo] recharge_mark: disabled in config");
            stopRechargeMarkWorker();
            work_mode_.store(WorkMode::kIdle, std::memory_order_release);
            timer_transition.unlock();
            this->callAppService(2002, 2, 10000);
            return true;
        }
        if (!recharge_api_) {
            cerr << "[PercepDemo] recharge_mark: api not ready" << endl;
            LOG_ERROR("[PercepDemo] recharge_mark: api not ready");
            stopRechargeMarkWorker();
            work_mode_.store(WorkMode::kIdle, std::memory_order_release);
            timer_transition.unlock();
            this->callAppService(2002, 2, 10000);
            return true;
        }
        if (recharge_api_->resetSession) recharge_api_->resetSession();
        if (!startRechargeMarkWorker()) {
            work_mode_.store(WorkMode::kIdle, std::memory_order_release);
            timer_transition.unlock();
            this->callAppService(2002, 2, 10000);
            return true;
        }
        work_mode_.store(WorkMode::kRecharge, std::memory_order_release);
        std_msgs::msg::UInt8 camera_request;
        camera_request.data = 2;  // 只发二维码；DNN 在 onTimer 里对 station 另开，不改 cmd 序号
        camera_request_pub.publish(camera_request);
        cout << "[PercepDemo] recharge_mark runtime=on min_interval_ms="
            << recharge_host_.min_interval_ms << endl;
        LOG_INFO("[PercepDemo] recharge_mark runtime=on min_interval_ms=%u",
                recharge_host_.min_interval_ms);
        timer_transition.unlock();
        this->callAppService(2002, 1, 10000);
    }
    else if (req.cmd == 3)
    {
        LOG_INFO("[PercepDemo] 收到同时发布启动指令");
        res.result = 0;
        work_mode_.store(WorkMode::kIdle, std::memory_order_release);
        std::unique_lock<std::mutex> timer_transition(timer_run_mtx_);
        rgb_frame_channel_.discardPending();
        gdc_frame_channel_.discardPending();
        stopRechargeMarkWorker();
        if (!recharge_host_.enabled || !recharge_host_.publish_enabled) {
            LOG_WARN("[PercepDemo] recharge_mark: disabled in config");
            stopRechargeMarkWorker();
            work_mode_.store(WorkMode::kMap, std::memory_order_release);
            std_msgs::msg::UInt8 camera_request;
            camera_request.data = 1;
            camera_request_pub.publish(camera_request);
            timer_transition.unlock();
            this->callAppService(2002, 2, 10000);
            return true;
        }
        if (!recharge_api_) {
            LOG_ERROR("[PercepDemo] recharge_mark: api not ready");
            stopRechargeMarkWorker();
            work_mode_.store(WorkMode::kMap, std::memory_order_release);
            std_msgs::msg::UInt8 camera_request;
            camera_request.data = 1;
            camera_request_pub.publish(camera_request);
            timer_transition.unlock();
            this->callAppService(2002, 2, 10000);
            return true;
        }
        if (recharge_api_->resetSession) recharge_api_->resetSession();
        if (!startRechargeMarkWorker()) {
            work_mode_.store(WorkMode::kMap, std::memory_order_release);
            std_msgs::msg::UInt8 camera_request;
            camera_request.data = 1;
            camera_request_pub.publish(camera_request);
            timer_transition.unlock();
            this->callAppService(2002, 2, 10000);
            return true;
        }
        work_mode_.store(WorkMode::kMapAndRecharge,
                         std::memory_order_release);
        std_msgs::msg::UInt8 camera_request;
        camera_request.data = 3;
        camera_request_pub.publish(camera_request);
        LOG_INFO("[PercepDemo] recharge_mark runtime=on min_interval_ms=%u",
                recharge_host_.min_interval_ms);
        timer_transition.unlock();
        this->callAppService(2002, 1, 10000);
    }
    else if (req.cmd == 21)
    {
        cout<<"[PercepDemo] 收到感知暂停工作请求"<<endl;
        //停止工作
        res.result = 0;
        work_mode_.store(WorkMode::kIdle, std::memory_order_release);
        std::unique_lock<std::mutex> timer_transition(timer_run_mtx_);
        std_msgs::msg::UInt8 camera_request;
        camera_request.data = 4;
        camera_request_pub.publish(camera_request);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        stopRechargeMarkWorker();
        // Frames can still arrive while the camera processes its stop command.
        // Drop that tail here so a later restart cannot consume an old frame.
        rgb_frame_channel_.discardPending();
        gdc_frame_channel_.discardPending();

        timer_transition.unlock();
        this->callAppService(2021,1,10000);  //上报感知停止工作完成           
        // this->callAppService(2021,2,10000);  //上报感知停止工作失败 
        
    }
    else{
      cerr<<"[PercepDemo err] 接收感知执行指令异常！"<<endl;
    }
    return true;
}

void PercepDemo::onTimer() {
    // 防重入：如果上一轮还没执行完，直接返回，丢弃本次timer触发
    if (shutting_down_.load(std::memory_order_acquire)) {
        return;
    }
    if (!timer_run_mtx_.try_lock()) {
        return;
    }
    std::lock_guard<std::mutex> lock(timer_run_mtx_, std::adopt_lock);
    if (shutting_down_.load(std::memory_order_acquire)) {
        return;
    }

    const bool log_on = (++log_count % 200) == 1;
    const WorkMode mode = currentWorkMode();
    if (mode == WorkMode::kIdle) {
        percep_status_.store(0, std::memory_order_release);
        return;
    }

    const bool mark_runtime =
        mark_runtime_on_.load(std::memory_order_acquire);
    const bool mark_only_task = mode == WorkMode::kRecharge && mark_runtime;
    // For recharge-only operation, do not consume a frame or run YOLO while
    // the SO's single-slot worker cannot accept the corresponding submission.
    if (mark_only_task &&
        (!recharge_api_ || !recharge_api_->canSubmit ||
         !recharge_api_->canSubmit())) {
        if ((++skip_mark_yolo_log_tick_ % 30U) == 1U) {
            LOG_INFO("[PercepDemo] recharge_mark skip frame "
                     "(busy or min_interval)");
        }
        return;
    }

    FrameDemandOptions demand_options;
    demand_options.recharge_prefers_gdc = recharge_host_.use_hires;
    demand_options.save_gdc_for_map = save_data;
    const FrameDemand demand = demandFor(mode, demand_options);

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(1000);
    const auto take_before_deadline = [&deadline](auto& channel,
                                                  bool wait) {
        if (!wait) {
            return channel.tryTakeLatest();
        }
        const auto now = std::chrono::steady_clock::now();
        const auto remaining = now < deadline
                                   ? deadline - now
                                   : std::chrono::steady_clock::duration::zero();
        return channel.waitTakeLatestFor(remaining);
    };

    RgbdFramePtr rgbd_frame;
    GdcFramePtr gdc_frame;
    if (demand.try_rgbd) {
        auto result =
            take_before_deadline(rgb_frame_channel_, demand.wait_rgbd);
        if (result.status == ChannelTakeStatus::kStopped) {
            return;
        }
        if (result.value) {
            rgbd_frame = std::move(*result.value);
        }
    }
    if (demand.try_gdc) {
        auto result =
            take_before_deadline(gdc_frame_channel_, demand.wait_gdc);
        if (result.status == ChannelTakeStatus::kStopped) {
            return;
        }
        if (result.value) {
            gdc_frame = std::move(*result.value);
        }
    }
    if (shutting_down_.load(std::memory_order_acquire)) {
        return;
    }

    if (log_on && demand.wait_rgbd && !rgbd_frame) {
        LOG_WARN("[PercepDemo] required RGBD frame timed out ready=%d",
                 rgb_camera_ready_.load(std::memory_order_acquire) ? 1 : 0);
    }
    if (log_on && demand.wait_gdc && !gdc_frame) {
        LOG_WARN("[PercepDemo] required GDC frame timed out ready=%d",
                 gdc_camera_ready_.load(std::memory_order_acquire) ? 1 : 0);
    }

    switch (mode) {
        case WorkMode::kMap:
            percep_status_.store(1, std::memory_order_release);
            break;
        case WorkMode::kRecharge:
            percep_status_.store(2, std::memory_order_release);
            break;
        case WorkMode::kMapAndRecharge:
            percep_status_.store(3, std::memory_order_release);
            break;
        case WorkMode::kIdle:
            break;
    }

    const auto calibration = cameraIntrinsicsSnapshot();
    std::optional<PrePointCloudInput> pointcloud_input;
    if (demand.run_map && rgbd_frame && calibration) {
        const auto scaled = calibration->scaledTo(
            rgbd_frame->disparity.width, rgbd_frame->disparity.height);
        if (scaled && scaled->depthValid()) {
            PrePointCloudInput input{rgbd_frame, *scaled};
            if (input.valid()) {
                pointcloud_input = std::move(input);
            }
        }
    }
    if (demand.run_map && !pointcloud_input && log_on) {
        LOG_WARN("[PercepDemo] map skipped: RGBD or valid CameraParam missing");
    }

    gdc_for_save_ =
        (demand.run_map && save_data && gdc_frame) ? gdc_frame : nullptr;
    const auto start_alg = std::chrono::high_resolution_clock::now();

    // Semantic output is frame-scoped. If DNN is disabled, skipped or fails,
    // the current map must receive an empty cloud instead of the previous
    // frame's semantic result.
    if (pointcloud_input) {
        last_semantic_cloud_.reset(new PointCloudT);
    }

#if defined(PERCEP_DNN_ENABLE)
    // Results are local to this timer invocation. Nothing from a previous
    // frame can be observed by recharge or point-cloud fusion.
    std::optional<DnnFrameResult> map_dnn_result;
    bool map_dnn_attempted = false;
#endif

    const RechargeFrameSource recharge_source =
        demand.run_recharge && mark_runtime
            ? selectRechargeFrameSource(recharge_host_.use_hires,
                                        recharge_allow_source_fallback_,
                                        static_cast<bool>(rgbd_frame),
                                        static_cast<bool>(gdc_frame))
            : RechargeFrameSource::kNone;
    const Nv12Image* recharge_image = nullptr;
    if (recharge_source == RechargeFrameSource::kGdc) {
        recharge_image = &gdc_frame->left;
    } else if (recharge_source == RechargeFrameSource::kRgbd) {
        recharge_image = &rgbd_frame->left;
    }

    const bool recharge_can_submit =
        mark_runtime && recharge_api_ && recharge_api_->canSubmit &&
        recharge_api_->canSubmit();

    if (demand.run_recharge && mark_runtime && recharge_image != nullptr &&
        recharge_can_submit) {
        const bool yolo_for_mark =
            recharge_host_.yolo_assist_crop && ai_recharge_.enabled;
#if defined(PERCEP_DNN_ENABLE)
        std::optional<DnnFrameResult> recharge_dnn_result;
        const DnnFrameResult* recharge_result = nullptr;
        if (yolo_for_mark && dnn_cfg_.enabled && dnn_runner_) {
            const bool shares_map_image =
                pointcloud_input &&
                recharge_image == &pointcloud_input->frame->left;
            if (shares_map_image) {
                // Advance both independent cadence lanes. If either lane is
                // due, submit this exact owning image only once; only a due
                // consumer receives the immutable result.
                map_dnn_attempted = true;
                const bool map_due =
                    consumeDnnCadence(DnnInferenceLane::kMap);
                const bool recharge_due =
                    consumeDnnCadence(DnnInferenceLane::kRecharge);
                if (map_due) {
                    map_dnn_result = runDnnInference(
                        *recharge_image, DnnInferenceLane::kMap,
                        demand.run_segmentation);
                    if (recharge_due && map_dnn_result) {
                        recharge_result = &*map_dnn_result;
                    }
                } else if (recharge_due) {
                    recharge_dnn_result = runDnnInference(
                        *recharge_image, DnnInferenceLane::kRecharge,
                        /*run_seg=*/false);
                    if (recharge_dnn_result) {
                        recharge_result = &*recharge_dnn_result;
                    }
                }
            } else {
                // GDC and RGBD are different owning images, so correctness
                // requires a separate recharge detection.
                if (consumeDnnCadence(DnnInferenceLane::kRecharge)) {
                    recharge_dnn_result = runDnnInference(
                        *recharge_image, DnnInferenceLane::kRecharge,
                        /*run_seg=*/false);
                    if (recharge_dnn_result) {
                        recharge_result = &*recharge_dnn_result;
                    }
                }
            }
        }
        applyYoloAssistCropHint(*recharge_image, recharge_result);
#else
        (void)yolo_for_mark;
        // A DNN-off build must never leave a frame-specific crop active.
        if (recharge_api_ && recharge_api_->setExternalCropHint) {
            recharge_api_->setExternalCropHint(0, 0, 0, 0, 1);
        }
#endif

        const std::uint64_t done_before =
            mark_done_seq_.load(std::memory_order_acquire);
        const int submit_result =
            submitRechargeMark(*recharge_image, calibration);
        // Recharge-only keeps YOLO/result/submission in a strict 1:1 cycle.
        if (mark_only_task && submit_result == 0) {
            waitRechargeMarkSettled(done_before, 2000);
        }
        if (log_on) {
            const char* source_name =
                recharge_source == RechargeFrameSource::kGdc ? "gdc" : "rgbd";
            LOG_INFO("[PercepDemo] recharge tick source=%s frame=%llu sub=%d",
                     source_name,
                     static_cast<unsigned long long>(
                         recharge_image->key.camera_frame_id),
                     submit_result);
        }
    } else if (demand.run_recharge && mark_runtime && recharge_image != nullptr &&
               !recharge_can_submit) {
        if ((++skip_combined_mark_log_tick_ % 30U) == 1U) {
            LOG_INFO("[PercepDemo] recharge branch skipped before YOLO "
                     "(busy or min_interval)");
        }
    } else if (demand.run_recharge && mark_runtime && log_on) {
        LOG_WARN("[PercepDemo] recharge skipped: no validated image");
    }

#if defined(PERCEP_DNN_ENABLE)
    if (pointcloud_input && dnn_cfg_.enabled && dnn_runner_ &&
        !map_dnn_attempted) {
        map_dnn_attempted = true;
        const auto dnn_start = std::chrono::high_resolution_clock::now();
        if (consumeDnnCadence(DnnInferenceLane::kMap)) {
            static const bool pair_bench_requested = [] {
                const char* value = std::getenv("PERCEP_PAIR_BENCH");
                return value && value[0] == '1' && value[1] == '\0';
            }();
            const bool measure_pair = pair_bench_requested &&
                                      demand.run_segmentation &&
                                      dnn_cfg_.detect.enabled && dnn_cfg_.seg.enabled;
            const auto pair_start = measure_pair
                ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
            map_dnn_result = runDnnInference(
                pointcloud_input->frame->left, DnnInferenceLane::kMap,
                demand.run_segmentation);
            if (measure_pair) {
                const double wall_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - pair_start).count();
                const auto frame_id = static_cast<unsigned long long>(
                    pointcloud_input->frame->left.key.camera_frame_id);
                const bool full_success = map_dnn_result &&
                    map_dnn_result->detection_requested &&
                    map_dnn_result->detection_succeeded &&
                    map_dnn_result->segmentation_requested &&
                    map_dnn_result->segmentation_succeeded;
                std::fprintf(stderr, "%s frame=%llu wall_ms=%.3f\n",
                             full_success ? "ORIGINAL_PAIR_MS" : "ORIGINAL_PAIR_FAIL",
                             frame_id, wall_ms);
            }
        }
        if (log_on) {
            const auto dnn_elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - dnn_start);
            cout << "1.runDnnInference Time taken: "
                 << dnn_elapsed.count() << "ms" << endl;
        }
    }
#endif

    if (pointcloud_input)
    {
        frame_save_time_str_ = std::to_string(daye::getSystemTimeMs());


// //////////////////数据格式转换///////////////
        auto start_2 = std::chrono::high_resolution_clock::now();
        frame_rgb_buf_.clear();
        const std::vector<uint8_t> *proj_rgb = &frame_rgb_buf_;
        const Nv12Image& rgbd_left = pointcloud_input->frame->left;
        if (!decodeYuv420ToRgb(rgbd_left, frame_rgb_buf_)) {
            cerr<<"[PercepDemo err] YUV420 decode failed!"<<endl;
            LOG_ERROR("[PercepDemo err] YUV420 decode failed!");
            percep_status_.store(-1, std::memory_order_release);
            return;}
        proj_rgb = &frame_rgb_buf_; 

       if (visualize )
        {
            sensor_msgs::msg::Image rgbd_img_msg;
            fillRgbRosImage(rgbd_left, frame_rgb_buf_, rgbd_img_msg);
            rgbd_img_msg.header.frame_id = "percep_demo";
            rgb_img_pub_.publish(rgbd_img_msg);   
        }


// //////////////////生成点云///////////////
        auto start_3 = std::chrono::high_resolution_clock::now();
        PointCloudT::Ptr pcl_cloud(new PointCloudT);
        if(this->dense2sparse_pointcloud(*pointcloud_input, sample_step,
                                        pcl_cloud, pcl_pub_cfg_))
        {
            if (auto_cali) {this->autocali(pcl_cloud); cout<<endl;
                percep_status_.store(0, std::memory_order_release);return;}
            ori_cloud = pcl_cloud;
            const auto& intrinsics = pointcloud_input->intrinsics;
            const float fx = intrinsics.fx;
            const float fy = intrinsics.fy;
            const float cx = intrinsics.cx;
            const float cy = intrinsics.cy;
            const int image_width = rgbd_left.width;
            const int image_height = rgbd_left.height;
            auto end_3 = std::chrono::high_resolution_clock::now();
            auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_3- start_3);
            cout<<"3.PointCloud generate Time taken: "<<duration3.count()<<"ms"<<endl;
// //////////////////////////////////////////

cout<<"原始点云个数："<<pcl_cloud->size()<<endl;
// //////////////////AI点云匹配///////////////
            auto start_4 = std::chrono::high_resolution_clock::now();
            #if defined(PERCEP_DNN_ENABLE)
                if (map_dnn_result) {
                    this->matchAndPublishDetCloud(
                        pcl_cloud, rgbd_left, *map_dnn_result,
                        fx, fy, cx, cy);
                } else if (dnn_cfg_.enabled) {
                    if ((++no_map_dnn_log_tick_ % 20U) == 1U) {
                        LOG_INFO("[PercepDemo] map DNN skipped or failed; "
                                 "semantic cloud remains empty");
                    }
                }
            #endif
            auto end_4 = std::chrono::high_resolution_clock::now();
            auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(end_4- start_4);
            cout<<"4.matchAndPublishDetCloud Time taken: "<<duration4.count()<<"ms"<<endl;
// //////////////////////////////////////////

// ////////////////点云后处理/////////////////
            auto start_5 = std::chrono::high_resolution_clock::now();
                this->postprocess(pcl_cloud, *proj_rgb, image_width,
                                  image_height, fx, fy, cx, cy);
            auto end_5 = std::chrono::high_resolution_clock::now();
            auto duration5 = std::chrono::duration_cast<std::chrono::milliseconds>(end_5- start_5);
            cout<<"5.postprocess Time taken: "<<duration5.count()<<"ms"<<endl;            
        }
        else{
            cerr<<"点云生成失败！"<<endl;
            LOG_ERROR("点云生成失败！");
            percep_status_.store(-1, std::memory_order_release);
            return;
        }
// //////////////////////////////////////////

// ////////////////地图融合/////////////////
        auto start_6 = std::chrono::high_resolution_clock::now();
        this->convertGridMap(frame_count_);
        auto end_6 = std::chrono::high_resolution_clock::now();
        auto duration6 = std::chrono::duration_cast<std::chrono::milliseconds>(end_6- start_6);
        cout<<"6.convertGridMap Time taken: "<<duration6.count()<<"ms"<<endl;
// /////////////////////////////////////

        ++frame_count_;
        auto end_alg = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_alg- start_alg);
        cout<<"All Alg Time taken: "<<duration.count()<<"ms"<<endl;
        LOG_INFO("[percepDemo]All Alg Time taken: %d ms",duration.count());
        cout<<endl;
    }
}
bool PercepDemo::dense2sparse_pointcloud(
    const PrePointCloudInput& input, int sample_step,
    PointCloudT::Ptr& out_sparse_cloud, const PointCloudPubConfig& box) {
    if (!input.valid() || !out_sparse_cloud || sample_step < 1) {
        std::cerr << "[PclCommon] invalid point-cloud input or sample step\n";
        return false;
    }

    const DisparityImage& disparity = input.frame->disparity;
    const CameraIntrinsics& intrinsics = input.intrinsics;
    const int width = disparity.width;
    const int height = disparity.height;
    const std::size_t stride =
        static_cast<std::size_t>(disparity.stride_samples);
    const float min_disparity = 1.0F;
    const float max_disparity = 144.0F;

    const std::size_t sampled_rows =
        (static_cast<std::size_t>(height) + sample_step - 1U) /
        static_cast<std::size_t>(sample_step);
    const std::size_t sampled_columns =
        (static_cast<std::size_t>(width) + sample_step - 1U) /
        static_cast<std::size_t>(sample_step);
    std::size_t reserve_count = 0;
    if (!checkedMultiply(sampled_rows, sampled_columns, &reserve_count)) {
        LOG_ERROR("[PclCommon] point reserve size overflow");
        return false;
    }
    if (box.max_points > 0) {
        reserve_count = std::min(reserve_count, box.max_points);
    }

    try {
        out_sparse_cloud->clear();
        out_sparse_cloud->reserve(reserve_count);
        disp_vec.clear();
        disp_vec.reserve(reserve_count);
        u_vec.clear();
        u_vec.reserve(reserve_count);
        v_vec.clear();
        v_vec.reserve(reserve_count);

        std::vector<float> disparity_for_display;
        if (visualize) {
            std::size_t pixel_count = 0;
            if (!checkedMultiply(static_cast<std::size_t>(width),
                                 static_cast<std::size_t>(height),
                                 &pixel_count)) {
                return false;
            }
            disparity_for_display.resize(pixel_count);
            for (int row = 0; row < height; ++row) {
                for (int column = 0; column < width; ++column) {
                    const auto source_index =
                        static_cast<std::size_t>(row) * stride + column;
                    const auto display_index =
                        static_cast<std::size_t>(row) * width + column;
                    const float value = static_cast<float>(
                                            disparity.samples[source_index]) /
                                        256.0F;
                    disparity_for_display[display_index] =
                        std::clamp(value, min_disparity, max_disparity);
                }
            }
        }

        bool reached_limit = false;
        for (int row = 0; row < height && !reached_limit;
             row += sample_step) {
            for (int column = 0; column < width; column += sample_step) {
                const auto index =
                    static_cast<std::size_t>(row) * stride + column;
                const float disparity_pixels =
                    static_cast<float>(disparity.samples[index]) / 256.0F;
                if (!std::isfinite(disparity_pixels) ||
                    disparity_pixels <= 0.0F) {
                    continue;
                }

                const float z = intrinsics.bxf_m / disparity_pixels;
                if (!std::isfinite(z) || z < 0.001F) {
                    continue;
                }
                const float x =
                    (static_cast<float>(column) - intrinsics.cx) * z /
                    intrinsics.fx;
                const float y =
                    (static_cast<float>(row) - intrinsics.cy) * z /
                    intrinsics.fy;
                if (!std::isfinite(x) || !std::isfinite(y) ||
                    z < box.box_min_z || z > box.box_max_z ||
                    x < box.box_min_x || x > box.box_max_x ||
                    y < box.box_min_y || y > box.box_max_y) {
                    continue;
                }

                out_sparse_cloud->emplace_back(
                    x, y, z, 0, 0, 0, -1, 0.0F);
                disp_vec.push_back(disparity_pixels);
                u_vec.push_back(static_cast<std::uint32_t>(column));
                v_vec.push_back(static_cast<std::uint32_t>(row));
                if (box.max_points > 0 &&
                    out_sparse_cloud->size() >= box.max_points) {
                    reached_limit = true;
                    break;
                }
            }
        }

        if (visualize && !disparity_for_display.empty()) {
            cv::Mat disparity_mat(height, width, CV_32FC1,
                                  disparity_for_display.data());
            cv::Mat disparity_gray;
            disparity_mat =
                (disparity_mat - min_disparity) /
                (max_disparity - min_disparity) * 255.0F;
            disparity_mat.convertTo(disparity_gray, CV_8UC1);

            sensor_msgs::msg::Image message;
            message.header.frame_id = "percep_demo";
            const auto timestamp_ns =
                disparity.key.camera_timestamp_ns > 0
                    ? disparity.key.camera_timestamp_ns
                    : disparity.key.transport_timestamp_ns;
            message.header.stamp = timestamp_ns > 0
                                       ? rclcpp::Time(timestamp_ns,
                                                      RCL_ROS_TIME)
                                       : rclcpp::Clock().now();
            message.width = static_cast<std::uint32_t>(width);
            message.height = static_cast<std::uint32_t>(height);
            message.encoding = "mono8";
            message.step = static_cast<std::uint32_t>(width);
            message.data.assign(disparity_gray.data,
                                disparity_gray.data +
                                    disparity_gray.total());
            disp_img_pub_.publish(message);
        }
    } catch (const std::bad_alloc&) {
        LOG_ERROR("[PclCommon] allocation failed while generating cloud");
        return false;
    }

    if (out_sparse_cloud->empty()) {
        std::cerr << "[PclCommon] sparse cloud generate empty\n";
        return false;
    }
    return true;
}
bool PercepDemo::initInterface()
{
    std::error_code output_error;
    if (output_dir.empty()) {
        LOG_ERROR("[PercepDemo] output_dir is empty");
        return false;
    }
    if (fs::exists(output_dir, output_error)) {
        if (output_error || !fs::is_directory(output_dir, output_error)) {
            LOG_ERROR("[PercepDemo] output_dir is not a usable directory: %s",
                      output_dir.c_str());
            return false;
        }
    } else if (!fs::create_directories(output_dir, output_error) &&
               output_error) {
        LOG_ERROR("[PercepDemo] create output_dir failed: %s (%s)",
                  output_dir.c_str(), output_error.message().c_str());
        return false;
    }
    // Existing captures and DNN overlays are intentionally retained. Each
    // producer applies its own naming/retention policy; startup must never
    // recursively delete a user-configured parent directory.
    cout << "数据目录可用: " << output_dir << endl;
    pclCommonPtr.reset(new PclCommon);
    ground_cloud.reset(new PointCloudT);
    no_ground_cloud.reset(new PointCloudT);
    obs_cloud.reset(new PointCloudT);
    last_semantic_cloud_.reset(new PointCloudT);
    aiFusionPtr.reset(new AiFusion(mapWidth,mapHeight,mapResolution,0.05,5,6,cali.x,cali.y,pcl_pub_cfg_.box_min_z));
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(daye::getHandle());
    visualizePtr.reset(new Visualize(mapWidth,mapHeight,mapResolution,output_dir,cali.x,cali.y));
    gdc_for_save_.reset();
    frame_rgb_buf_.reserve(rgb_width*rgb_height*3);
    // color_rgb_buf_.reserve(gdc_width*gdc_height*3);
    if (patchwork_cfg_.enabled) {
        patchwork::Params pw_params;
        pw_params.verbose       = patchwork_cfg_.verbose;
        pw_params.sensor_height = patchwork_cfg_.sensor_height;
        pw_params.min_range     = patchwork_cfg_.min_range;
        pw_params.max_range     = patchwork_cfg_.max_range;
        pw_params.th_dist       = patchwork_cfg_.th_dist;
        pw_params.th_seeds      = patchwork_cfg_.th_seeds;
        pw_params.enable_RNR    = false;  // RGBD 点云无强度列，强制关闭

        pw_params.num_iter = 3;
        pw_params.num_lpr = 20;
        pw_params.num_min_pts = 0;
        pw_params.th_seeds_v = 0.8;
        pw_params.th_dist_v = 0.05;
        pw_params.uprightness_thr = 0.101;
        pw_params.num_zones = 1;
        pw_params.num_rings_of_interest = 1;

        patchwork_.reset(new patchwork::PatchWorkpp(pw_params));
        cout << "[PercepDemo] PatchWork++ enabled"
            << " range=[" << pw_params.min_range << "," << pw_params.max_range << "]m"
            << " th_dist=" << pw_params.th_dist
            << " th_seeds=" << pw_params.th_seeds
            << " sensor_h=" << pw_params.sensor_height << "m"
            << " pub_ground=" << patchwork_cfg_.publish_ground
            << " pub_nonground=" << patchwork_cfg_.publish_nonground << endl;
    }

//-----------------sdk interface-------------------//      
    if (!offline) 
    {
        
        auto qos_cam = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        cam_params_sub_ = daye::CreateSubscribe<common_interface::msg::CameraParam>(kCameraParam, qos_cam,bind(&PercepDemo::onCamParams, this, placeholders::_1));
        camera_request_pub = daye::CreatePublish<std_msgs::msg::UInt8>("/camera/get_data", qos_cam);
    }
  //-----------------ros topic-------------------//
  rclcpp::SubscriptionOptionsWithAllocator<allocator<void>> sub_options;
  sub_options.callback_group = sub_callback_group_;
  // sub_ = daye::CreateSubscribe<std_msgs::msg::UInt8>("/percep_demo/test", rclcpp::SensorDataQoS(),bind(&PercepDemo::onMessage, this, placeholders::_1));
  grid_map_pub_ = daye::CreatePublish<common_interface::msg::GridMapInfo>(kGridMapInfo, rclcpp::SensorDataQoS());


  if (visualize)
  {
    ori_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_demo/pcl_cloud", rclcpp::QoS(5));   //Qos为BEST_EFFORT
    pclmap_pub_ =  daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_demo/pcl_map", rclcpp::QoS(5));
    aimap_pub_ =  daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_demo/ai_map", rclcpp::QoS(5));
    fusionmap_pub_ =  daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_demo/fusion_map", rclcpp::QoS(5));
    rgb_img_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>("/percep_demo/rgb_frame", rclcpp::QoS(5));//Reliable
    // gdc_img_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>("/percep_demo/gdc_frame", rclcpp::QoS(5));
    disp_img_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>("/percep_demo/disp_frame", rclcpp::QoS(5));
  }

  if (patchwork_cfg_.enabled) {
      if (patchwork_cfg_.publish_ground) {
          ground_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>(
              "/percep_demo/ground_cloud", rclcpp::QoS(5));
      }
      if (patchwork_cfg_.publish_nonground) {
          nonground_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>(
              "/percep_demo/nonground_cloud", rclcpp::QoS(5));
      }
  }

  if (!recharge_mark_config_path_.empty()) {
      recharge_host_ = RechargeHostConfig{};
      ai_recharge_ = AiRechargeConfig{};
      std::unique_ptr<Aiper_Recharge::RechargeMarkApi, RechargeApiDeleter>
          next_api(Aiper_Recharge::createRechargeMarkApi());
      if (!next_api || !next_api->init || !next_api->destroy) {
          LOG_ERROR("[PercepDemo] recharge_mark API factory/init/destroy unavailable");
      } else {
          const int rc = next_api->init(
              sdk_so_dir.c_str(), recharge_mark_config_path_.c_str());
          if (rc != 0) {
              cerr << "[PercepDemo] recharge_mark loadConfig failed path="
                   << recharge_mark_config_path_ << endl;
              LOG_ERROR("[PercepDemo] recharge_mark loadConfig failed path=%s rc=%d",
                        recharge_mark_config_path_.c_str(), rc);
          } else {
              // These entries form the minimum async API contract. Committing
              // a partial function table would postpone an ABI error until a
              // service command is already active.
              const bool async_api_complete =
                  next_api->destroy && next_api->getHostConfig &&
                  next_api->setResultCallback && next_api->startWorker &&
                  next_api->stopWorker && next_api->canSubmit &&
                  next_api->submitLeftNv12;
              if (!async_api_complete) {
                  LOG_ERROR("[PercepDemo] recharge_mark async API table incomplete");
              } else {
                  next_api->getHostConfig(&recharge_host_);
                  const bool ai_config_valid =
                      loadAiRechargeConfig(recharge_mark_config_path_);
                  if (!ai_config_valid) {
                      recharge_host_.yolo_assist_crop = false;
                      LOG_ERROR("[PercepDemo] invalid ai_recharge config; "
                                "disable detector-assisted crop");
                  } else if (!ai_recharge_.enabled &&
                      recharge_host_.yolo_assist_crop) {
                      ai_recharge_.enabled = true;
                  }
                  next_api->setResultCallback(
                      &PercepDemo::OnRechargeMarkResult, this);
                  // Transactional commit: every check above succeeded, so
                  // ownership becomes visible to service/timer code at once.
                  recharge_api_ = std::move(next_api);
              }
          }
      }
      if (recharge_host_.enabled && recharge_host_.publish_enabled) {
          recharge_mark_pub_ =
              daye::CreatePublish<common_interface::msg::DockPose>(
                  kRechargeMarkPoseTopic, rclcpp::SensorDataQoS());
          if (recharge_host_.publish_overlay) {
              const auto overlay_qos = rclcpp::QoS(rclcpp::KeepLast(5))
                                           .best_effort()
                                           .durability_volatile();
              recharge_mark_overlay_pub_ =
                  daye::CreatePublish<sensor_msgs::msg::Image>(
                      "/percep_demo/recharge_mark_overlay", overlay_qos);
          }
          cout << "[PercepDemo] recharge_mark ready (wait ToPercepCmd cmd=2) "
               << "topic=" << kRechargeMarkPoseTopic
               << " cfg=" << recharge_mark_config_path_
               << " min_interval_ms=" << recharge_host_.min_interval_ms
               << " use_hires=" << (recharge_host_.use_hires ? "on" : "off")
               << " yolo_assist=" << (recharge_host_.yolo_assist_crop ? "on" : "off")
               << " source_fallback="
               << (recharge_allow_source_fallback_ ? "on" : "off")
               << " charger=" << ai_recharge_.charger_class
               << endl;
          LOG_INFO("[PercepDemo] recharge_mark ready (wait ToPercepCmd cmd=2) "
                   "topic=%s cfg=%s min_interval_ms=%u use_hires=%s "
                   "yolo=%s source_fallback=%s",
                   kRechargeMarkPoseTopic, recharge_mark_config_path_.c_str(),
                   recharge_host_.min_interval_ms,
                   recharge_host_.use_hires ? "on" : "off",
                   recharge_host_.yolo_assist_crop ? "on" : "off",
                   recharge_allow_source_fallback_ ? "on" : "off");
      } else {
          cout << "[PercepDemo] recharge_mark disabled in "
               << recharge_mark_config_path_ << endl;
      }
  }
  cout << "[PercepDemo] image_pub resize="
       << (image_pub_cfg_.resize_enabled ? "on" : "off");
  if (image_pub_cfg_.resize_enabled) {
      cout << "[PercepDemo]  target=" << image_pub_cfg_.width << "x" << image_pub_cfg_.height;
  } else {
      cout << "[PercepDemo]  (original resolution)";
  }
  cout << endl;
  cout << "[PercepDemo] point_cloud_color="
       << (pcl_color_cfg_.enabled ? "on" : "off")
       << " use_gdc=" << (pcl_color_cfg_.use_gdc_image ? "yes" : "no")
       << " height_fallback="
       << (pcl_color_cfg_.height_color_fallback ? "on" : "off") << endl;
  cout << "[PercepDemo] point_cloud_pub max_points=" << pcl_pub_cfg_.max_points
       << " box_z=[" << pcl_pub_cfg_.box_min_z << "," << pcl_pub_cfg_.box_max_z
       << "] box_xy=[" << pcl_pub_cfg_.box_min_x << "," << pcl_pub_cfg_.box_max_x
       << "]" << endl;
  

#if defined(PERCEP_DNN_ENABLE)
  this->initDnnRuntime();
#endif
  return true;
}

bool PercepDemo::startOnlineRuntime() {
    const CameraShmLayout layout = cameraShmLayout();
    if (!layout.sdk_available) {
        LOG_ERROR("[PercepDemo] camera_struct SDK unavailable; "
                  "online input not started");
        return false;
    }
    // ChannelConfig carries a maximum frame capacity, not the exact wire
    // message size. A larger channel is valid; a smaller one would truncate
    // or reject the SDK object before the adapter can validate it.
    if (layout.rgbd_message_bytes >
            static_cast<std::size_t>(kFrameShmSize) ||
        layout.gdc_message_bytes >
            static_cast<std::size_t>(kGdcFrameShmSize)) {
        LOG_ERROR("[PercepDemo] camera SHM channel capacity too small "
                  "rgbd sdk=%zu channel=%zu gdc sdk=%zu channel=%zu",
                  layout.rgbd_message_bytes,
                  static_cast<std::size_t>(kFrameShmSize),
                  layout.gdc_message_bytes,
                  static_cast<std::size_t>(kGdcFrameShmSize));
        return false;
    }

    bool timer_started = false;
    try {
        // ShmSubscriber's callback thread is bound to the subscriber object's
        // address. Moving an already-running local subscriber stops that
        // source thread. Move only the unstarted channel holder, then register
        // callbacks directly on the final member objects.
        rgb_subscriber_ = daye::ShmSubscriber(
            daye::ChannelConfig{kCameraFrameShm, kFrameShmSize});
        gdc_subscriber_ = daye::ShmSubscriber(
            daye::ChannelConfig{kCameraGdcFrameShm, kGdcFrameShmSize});

        rgb_subscriber_.registerCallback(
            std::bind(&PercepDemo::onCameraFrame, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3),
            100);
        gdc_subscriber_.registerCallback(
            std::bind(&PercepDemo::onGdcFrame, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3),
            100);

        // Timer is deliberately started only after publishers, recharge,
        // models, channels and subscribers are all ready.
        timer_ = daye::CreateTimer("percep_demo_timer", 10.0,
                                  &PercepDemo::onTimer, this, true);
        timer_started = true;
        // This is the final potentially-throwing construction step. If thread
        // creation fails, the catch block stops all already-started inputs.
        if (save_log) {
            saveLogThreadHandle = std::thread([this]() noexcept {
                try {
                    saveLogLoop();
                } catch (const std::exception& error) {
                    LOG_ERROR("[PercepDemo] save thread stopped by exception: %s",
                              error.what());
                } catch (...) {
                    LOG_ERROR("[PercepDemo] save thread stopped by unknown exception");
                }
            });
        }
        return true;
    } catch (const std::exception& error) {
        if (timer_started) {
            timer_.stop();
        }
        rgb_subscriber_ = daye::ShmSubscriber{};
        gdc_subscriber_ = daye::ShmSubscriber{};
        LOG_ERROR("[PercepDemo] online runtime startup failed: %s",
                  error.what());
        return false;
    } catch (...) {
        if (timer_started) {
            timer_.stop();
        }
        rgb_subscriber_ = daye::ShmSubscriber{};
        gdc_subscriber_ = daye::ShmSubscriber{};
        LOG_ERROR("[PercepDemo] online runtime startup failed: unknown error");
        return false;
    }
}

bool PercepDemo::resolveRechargeMarkConfigPath(
    const std::string& start_config_path,
    const rapidjson::Document& ai_node_json) {
    recharge_mark_config_path_ = "/app/daye/config/ai_node/recharge_mark.json";
    if (!ai_node_json.HasMember("recharge_mark") ||
        !ai_node_json["recharge_mark"].IsObject()) {
        cout << "[PercepDemo] recharge_mark config_file default="
             << recharge_mark_config_path_ << endl;
        return true;
    }
    const rapidjson::Value& rm = ai_node_json["recharge_mark"];
    if (rm.HasMember("config_file") && rm["config_file"].IsString()) {
        recharge_mark_config_path_ =
            resolveConfigPath(start_config_path, rm["config_file"].GetString());
    }
    cout << "[PercepDemo] recharge_mark config_file="
         << recharge_mark_config_path_ << endl;
    return true;
}

bool PercepDemo::initParams()

{
    string extrinsics_config_dir = "/app/daye/config/ai_node/extrinsics.json";
    std::ifstream ifs_(extrinsics_config_dir, std::ios::binary);
    if (!ifs_.is_open()) {
        cerr << "[AutoCali err] 打开标定文件失败: " << extrinsics_config_dir << std::endl;
        return false;
    }
    std::stringstream ss_;
    ss_ << ifs_.rdbuf();
    string config = ss_.str();
    ifs_.close();  

    rapidjson::Document extrinsics_json;
    extrinsics_json.Parse(config.c_str());
    if (extrinsics_json.HasParseError()){
        cerr << "[AutoCali err] json解析失败: "<< std::endl;
        return false;
    } 
    
    if (extrinsics_json.HasMember("extrinsics") &&
        extrinsics_json["extrinsics"].IsObject()) {
        const rapidjson::Value& pp = extrinsics_json["extrinsics"];  
        cali.x = static_cast<float>(
            readJsonDouble(pp, "x", cali.x));    
        cali.y =static_cast<float>(
            readJsonDouble(pp, "y", cali.y)); 
        cali.z = static_cast<float>(
            readJsonDouble(pp, "z", cali.z));    
        cali.roll = angle_to_rad(static_cast<float>(
            readJsonDouble(pp, "roll", cali.roll)));    
        cali.pitch = angle_to_rad(static_cast<float>(
            readJsonDouble(pp, "pitch", cali.pitch))); 
        cali.yaw = angle_to_rad(static_cast<float>(
            readJsonDouble(pp, "yaw", cali.yaw)));     
    }

    string start_config_dir = "/app/daye/config/ai_node/start_config.json";
    std::ifstream ifs(start_config_dir, std::ios::binary);
    if (!ifs.is_open()) {
        cerr << "[PercepDemo err] 打开标定文件失败: " << start_config_dir << std::endl;
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    string start_config = ss.str();
    ifs.close();  
    rapidjson::Document ai_node_json;
    ai_node_json.Parse(start_config.c_str());
    if (ai_node_json.HasParseError()){
      cerr << "[PercepDemo err] json解析失败: "<< std::endl;
      return false;
    } 
    if (!ai_node_json.IsObject()) {
        cerr<<"[PercepDemo err] 不是JSON对象！"<<endl;
        return false; 
    }
    if (ai_node_json.HasMember("sdk_so_dir") && ai_node_json["sdk_so_dir"].IsString()) {
        sdk_so_dir = ai_node_json["sdk_so_dir"].GetString();
    }
    else{
      return false;
    }
    if (ai_node_json.HasMember("cfg_path") && ai_node_json["cfg_path"].IsString()) {
        cfg_path = ai_node_json["cfg_path"].GetString();
    }
    else{
      return false;
    }
    if (ai_node_json.HasMember("output_dir") && ai_node_json["output_dir"].IsString()) {
        output_dir = ai_node_json["output_dir"].GetString();
    }
    else{
      return false;
    }

    if (ai_node_json.HasMember("debug") && ai_node_json["debug"].IsObject()) {
        const rapidjson::Value& pd = ai_node_json["debug"];
        if (pd.HasMember("save_log") && pd["save_log"].IsBool()) {
            save_log = pd["save_log"].GetBool();
        }
        if (pd.HasMember("save_map") && pd["save_map"].IsBool()) {
            save_map = pd["save_map"].GetBool();
        }
        if (pd.HasMember("save_data") && pd["save_data"].IsBool()) {
            save_data = pd["save_data"].GetBool();
        }
        if (pd.HasMember("visualize") && pd["visualize"].IsBool()) {
            visualize = pd["visualize"].GetBool();
        }
    }

    if (ai_node_json.HasMember("auto_cali") && ai_node_json["auto_cali"].IsBool()) {
        auto_cali = ai_node_json["auto_cali"].GetBool();
    }
    else{
      return false;
    }
    if (ai_node_json.HasMember("offline") && ai_node_json["offline"].IsBool()) {
        offline = ai_node_json["offline"].GetBool();
    }
    else{
      return false;
    }
    if (ai_node_json.HasMember("interactive") && ai_node_json["interactive"].IsBool()) {
        interactive = ai_node_json["interactive"].GetBool();
    }
    else{
      return false;
    }
    if (ai_node_json.HasMember("self_test") && ai_node_json["self_test"].IsBool()) {
        self_test = ai_node_json["self_test"].GetBool();
    }
    else{
      return false;
    }
    if (ai_node_json.HasMember("patchwork") && ai_node_json["patchwork"].IsObject()) {
        const rapidjson::Value& pw = ai_node_json["patchwork"];
        if (pw.HasMember("enabled") && pw["enabled"].IsBool()) {
            patchwork_cfg_.enabled = pw["enabled"].GetBool();
        }
        if (pw.HasMember("verbose") && pw["verbose"].IsBool()) {
            patchwork_cfg_.verbose = pw["verbose"].GetBool();
        }
        if (pw.HasMember("publish_ground") && pw["publish_ground"].IsBool()) {
            patchwork_cfg_.publish_ground = pw["publish_ground"].GetBool();
        }
        if (pw.HasMember("publish_nonground") && pw["publish_nonground"].IsBool()) {
            patchwork_cfg_.publish_nonground = pw["publish_nonground"].GetBool();
        }
        patchwork_cfg_.sensor_height = readJsonDouble(pw, "sensor_height", patchwork_cfg_.sensor_height);
        patchwork_cfg_.min_range     = readJsonDouble(pw, "min_range",     patchwork_cfg_.min_range);
        patchwork_cfg_.max_range     = readJsonDouble(pw, "max_range",     patchwork_cfg_.max_range);
        patchwork_cfg_.th_dist       = readJsonDouble(pw, "th_dist",       patchwork_cfg_.th_dist);
        patchwork_cfg_.th_seeds      = readJsonDouble(pw, "th_seeds",      patchwork_cfg_.th_seeds);
        if (pw.HasMember("log_every_n") && pw["log_every_n"].IsInt()) {
            patchwork_cfg_.log_every_n = pw["log_every_n"].GetInt();
        }
    }

    if (ai_node_json.HasMember("image_pub") && ai_node_json["image_pub"].IsObject()) {
        const rapidjson::Value& img = ai_node_json["image_pub"];
        if (img.HasMember("resize_enabled") && img["resize_enabled"].IsBool()) {
            image_pub_cfg_.resize_enabled = img["resize_enabled"].GetBool();
        }
        image_pub_cfg_.width  = readJsonUInt(img, "width",  image_pub_cfg_.width);
        image_pub_cfg_.height = readJsonUInt(img, "height", image_pub_cfg_.height);
        if (image_pub_cfg_.width == 0 || image_pub_cfg_.height == 0) {
            cerr << "[PercepDemo err] image_pub width/height invalid, disable resize" << endl;
            image_pub_cfg_.resize_enabled = false;
        }
    }

    if (ai_node_json.HasMember("point_cloud_color") &&
        ai_node_json["point_cloud_color"].IsObject()) {
        const rapidjson::Value& pc = ai_node_json["point_cloud_color"];
        if (pc.HasMember("enabled") && pc["enabled"].IsBool()) {
            pcl_color_cfg_.enabled = pc["enabled"].GetBool();
        }
        if (pc.HasMember("use_gdc_image") && pc["use_gdc_image"].IsBool()) {
            pcl_color_cfg_.use_gdc_image = pc["use_gdc_image"].GetBool();
        }
        if (pc.HasMember("height_color_fallback") &&
            pc["height_color_fallback"].IsBool()) {
            pcl_color_cfg_.height_color_fallback =
                pc["height_color_fallback"].GetBool();
        }
    }

    if (ai_node_json.HasMember("point_cloud_pub") &&
        ai_node_json["point_cloud_pub"].IsObject()) {
        const rapidjson::Value& pp = ai_node_json["point_cloud_pub"];
        pcl_pub_cfg_.max_points =
            readJsonUInt(pp, "max_points", static_cast<uint32_t>(pcl_pub_cfg_.max_points));
        pcl_pub_cfg_.box_min_x = static_cast<float>(
            readJsonDouble(pp, "box_min_x", pcl_pub_cfg_.box_min_x));
        pcl_pub_cfg_.box_max_x = static_cast<float>(
            readJsonDouble(pp, "box_max_x", pcl_pub_cfg_.box_max_x));
        pcl_pub_cfg_.box_min_y = static_cast<float>(
            readJsonDouble(pp, "box_min_y", pcl_pub_cfg_.box_min_y));
        pcl_pub_cfg_.box_max_y = static_cast<float>(
            readJsonDouble(pp, "box_max_y", pcl_pub_cfg_.box_max_y));
        pcl_pub_cfg_.box_min_z = static_cast<float>(
            readJsonDouble(pp, "box_min_z", pcl_pub_cfg_.box_min_z));
        pcl_pub_cfg_.box_max_z = static_cast<float>(
            readJsonDouble(pp, "box_max_z", pcl_pub_cfg_.box_max_z));
    }

    if (ai_node_json.HasMember("map") &&
    ai_node_json["map"].IsObject()) {
        const rapidjson::Value& pp = ai_node_json["map"]; 
        mapWidth = readJsonDouble(pp, "MapWidth", mapWidth);
        mapHeight = readJsonDouble(pp, "MapHeight", mapHeight);
        mapResolution = readJsonDouble(pp, "MapResolution", mapResolution);
    }
    const auto grid_geometry = detail::TryMakeGridGeometry(
        mapWidth, mapHeight, mapResolution);
    const auto visualization_geometry = grid_geometry.has_value()
        ? detail::TryMakeGridVisualizationGeometry(*grid_geometry)
        : std::nullopt;
    if (!grid_geometry.has_value() || !visualization_geometry.has_value()) {
        cerr << "[PercepDemo err] invalid map geometry: dimensions/resolution "
                "must be finite and positive, rows/cols and resolution_mm must "
                "fit uint16, total cells must not exceed "
             << detail::kMaxFusionGridCells
             << ", and visualization pixels must not exceed "
             << detail::kMaxVisualizationPixels << endl;
        return false;
    }
    // Use the exact millimeter value advertised in GridMapInfo throughout all
    // projection and visualization code.
    mapResolution = grid_geometry->resolution_m;
    if (!resolveRechargeMarkConfigPath(start_config_dir, ai_node_json)) {
        cerr << "[PercepDemo err] 解析回充配置路径失败" << endl;
        return false;
    }

#if defined(PERCEP_DNN_ENABLE)
    if (!this->parseDnnConfig(ai_node_json)) {
        cerr << "[PercepDemo] parseDnnConfig failed" << endl;
        return false;
    }
    this->parseLabelFusionConfig(ai_node_json);
    cout << "[PercepDemo] dnn cfg enabled=" << (dnn_cfg_.enabled ? "on" : "off")
         << " detect=" << (dnn_cfg_.detect.enabled ? "on" : "off")
         << " model=" << dnn_cfg_.detect.model_path << endl;
#endif
    return true;
}

double PercepDemo::readJsonDouble(const rapidjson::Value& obj, const char* key, double default_val) {
    if (!obj.HasMember(key)) return default_val;
    if (obj[key].IsDouble()) return obj[key].GetDouble();
    if (obj[key].IsInt())    return static_cast<double>(obj[key].GetInt());
    return default_val;
}

uint32_t PercepDemo::readJsonUInt(const rapidjson::Value& obj, const char* key, uint32_t default_val) {
    if (!obj.HasMember(key)) return default_val;
    if (obj[key].IsUint())  return obj[key].GetUint();
    if (obj[key].IsInt() && obj[key].GetInt() >= 0) return static_cast<uint32_t>(obj[key].GetInt());
    return default_val;
}

bool PercepDemo::readJsonBool(const rapidjson::Value& obj, const char* key,
                              bool default_val) {
    if (!obj.HasMember(key) || !obj[key].IsBool()) return default_val;
    return obj[key].GetBool();
}

void PercepDemo::OnRechargeMarkResult(void* userdata,
                                      Aiper_Recharge::MarkPoseResult* result) {
    auto* self = static_cast<PercepDemo*>(userdata);
    if (!self) {
        return;
    }
    if (result) {
        self->publishRechargeMark(result);
    }
    self->mark_done_seq_.fetch_add(1, std::memory_order_release);
    self->mark_done_cv_.notify_all();
}

void PercepDemo::waitRechargeMarkSettled(uint64_t done_before, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mark_done_mtx_);
    const bool ok = mark_done_cv_.wait_for(
        lk, std::chrono::milliseconds(timeout_ms), [&] {
            return shutting_down_.load(std::memory_order_acquire) ||
                   mark_done_seq_.load(std::memory_order_acquire) > done_before;
        });
    if (!ok) {
        LOG_WARN("[PercepDemo] recharge_mark settle timeout %dms", timeout_ms);
    }
}

bool PercepDemo::loadAiRechargeConfig(const std::string& json_path) {
    AiRechargeConfig next;
    ai_recharge_ = next;
    if (json_path.empty()) return true;
    std::ifstream ifs(json_path, std::ios::binary);
    if (!ifs.is_open()) return true;
    std::stringstream ss;
    ss << ifs.rdbuf();
    rapidjson::Document doc;
    if (doc.Parse(ss.str().c_str()).HasParseError() || !doc.IsObject()) {
        return false;
    }
    const rapidjson::Value* root = &doc;
    if (doc.HasMember("recharge_mark") && doc["recharge_mark"].IsObject()) {
        root = &doc["recharge_mark"];
    }
    recharge_allow_source_fallback_ = readJsonBool(
        *root, "allow_source_fallback", recharge_allow_source_fallback_);
    if (!root->HasMember("ai_recharge") || !(*root)["ai_recharge"].IsObject()) {
        return true;
    }
    const rapidjson::Value& ar = (*root)["ai_recharge"];
    next.enabled = readJsonBool(ar, "enabled", next.enabled);
    if (ar.HasMember("charger_class") && ar["charger_class"].IsString()) {
        next.charger_class = ar["charger_class"].GetString();
    }
    if (ar.HasMember("score_min") && ar["score_min"].IsNumber()) {
        next.score_min = static_cast<float>(ar["score_min"].GetDouble());
    }
    if (ar.HasMember("pad_ratio") && ar["pad_ratio"].IsNumber()) {
        next.pad_ratio = static_cast<float>(ar["pad_ratio"].GetDouble());
    }
    if (ar.HasMember("pad_px") && ar["pad_px"].IsInt()) {
        next.pad_px = ar["pad_px"].GetInt();
    }
    if (ar.HasMember("min_box_w") && ar["min_box_w"].IsInt()) {
        next.min_box_w = ar["min_box_w"].GetInt();
    }
    if (ar.HasMember("min_box_h") && ar["min_box_h"].IsInt()) {
        next.min_box_h = ar["min_box_h"].GetInt();
    }
    const bool valid = !next.charger_class.empty() &&
        std::isfinite(next.score_min) && next.score_min >= 0.0F &&
        next.score_min <= 1.0F && std::isfinite(next.pad_ratio) &&
        next.pad_ratio >= 0.0F &&
        next.pad_ratio <= kMaxChargerCropPadRatio && next.pad_px >= 0 &&
        next.pad_px <= kMaxCameraFrameDimension && next.min_box_w > 0 &&
        next.min_box_h > 0 && next.min_box_w <= kMaxCameraFrameDimension &&
        next.min_box_h <= kMaxCameraFrameDimension;
    if (!valid) return false;
    ai_recharge_ = std::move(next);
    return true;
}

#if defined(PERCEP_DNN_ENABLE)
void PercepDemo::applyYoloAssistCropHint(const Nv12Image& image,
                                         const DnnFrameResult* result) {
    if (!recharge_api_ || !recharge_api_->setExternalCropHint) return;
    const bool want = recharge_host_.yolo_assist_crop && ai_recharge_.enabled;
    if (!want) {
        recharge_api_->setExternalCropHint(0, 0, 0, 0, 1);
        return;
    }
    const bool same_frame = result != nullptr && result->matches(image) &&
                            result->detection_succeeded;
    const int mark_w = image.width;
    const int mark_h = image.height;
    const int det_w = same_frame ? result->image_width : 0;
    const int det_h = same_frame ? result->image_height : 0;
    const DnnDetection* best = nullptr;
    if (det_w > 0 && det_h > 0) {
        for (const auto& d : result->detections) {
            const bool name_ok =
                !d.class_name.empty() && d.class_name == ai_recharge_.charger_class;
            if (!name_ok) continue;
            if (d.confidence < ai_recharge_.score_min) continue;
            if (!best || d.confidence > best->confidence) best = &d;
        }
    }
    int ox = 0, oy = 0, ow = 0, oh = 0;
    bool ok = false;
    if (best) {
        ok = buildChargerCropOnImage(
            best->x1, best->y1, best->x2, best->y2, det_w, det_h, mark_w,
            mark_h, ai_recharge_.pad_ratio, ai_recharge_.pad_px,
            ai_recharge_.min_box_w, ai_recharge_.min_box_h, &ox, &oy, &ow,
            &oh);
    }
    if (!ok) {
        // A crop is frame-specific. Reusing the previous frame's crop can
        // silently inspect the wrong region after the robot or target moves.
        recharge_api_->setExternalCropHint(0, 0, 0, 0, 1);
        if ((++yolo_crop_miss_log_tick_ % 20U) == 1U) {
            LOG_INFO("[PercepDemo] yolo_assist no charger box det=%dx%d dets=%zu "
                     "fallback=static_roi",
                     det_w, det_h,
                     result != nullptr ? result->detections.size() : 0U);
        }
        return;
    }
    recharge_api_->setExternalCropHint(ox, oy, ow, oh, 1);
    if ((++yolo_crop_hit_log_tick_ % 20U) == 1U) {
        LOG_INFO("[PercepDemo] yolo_assist crop=[%d,%d,%d,%d] score=%.2f "
                 "det=%dx%d mark=%dx%d",
                 ox, oy, ow, oh, best->confidence, det_w, det_h, mark_w,
                 mark_h);
        cout << "[PercepDemo] yolo_assist crop=[" << ox << "," << oy << ","
             << ow << "," << oh << "] score=" << best->confidence << endl;
    }
}
#endif

bool PercepDemo::startRechargeMarkWorker() {
    if (!recharge_host_.enabled || !recharge_host_.publish_enabled ||
        !recharge_api_ || !recharge_api_->startWorker) {
        LOG_ERROR("[PercepDemo] recharge_mark worker prerequisites missing");
        return false;
    }
    const int start_result = recharge_api_->startWorker();
    if (start_result != 0) {
        mark_runtime_on_.store(false, std::memory_order_release);
        LOG_ERROR("[PercepDemo] recharge_mark worker start failed rc=%d",
                  start_result);
        return false;
    }
    // Publish the running state only after the worker accepted startup.
    mark_runtime_on_.store(true, std::memory_order_release);
    cout << "[PercepDemo] recharge_mark worker=on (so async, min_interval_ms="
         << recharge_host_.min_interval_ms << ")" << endl;
    LOG_INFO("[PercepDemo] recharge_mark worker=on (so async, min_interval_ms=%u)",
             recharge_host_.min_interval_ms);
    return true;
}

void PercepDemo::stopRechargeMarkWorker() {
    mark_runtime_on_.store(false, std::memory_order_release);
    if (recharge_api_ && recharge_api_->stopWorker) {
        recharge_api_->stopWorker();
    }
    mark_done_cv_.notify_all();
}

int PercepDemo::submitRechargeMark(
    const Nv12Image& image,
    const std::optional<CameraIntrinsics>& calibration) {
    if (!mark_runtime_on_.load(std::memory_order_acquire) ||
        !recharge_host_.enabled || !recharge_host_.publish_enabled ||
        !recharge_api_ || !recharge_api_->submitLeftNv12 ||
        !image.valid()) {
        return 1;
    }

    if (std::strcmp(recharge_host_.intrinsics_source, "sdk_scaled") == 0) {
        if (!calibration || !recharge_api_->setCameraMatrix) {
            LOG_ERROR("[PercepDemo] recharge requires CameraParam but none is available");
            return 2;
        }
        const auto scaled = calibration->scaledTo(image.width, image.height);
        if (!scaled || !scaled->pinholeValid()) {
            LOG_ERROR("[PercepDemo] recharge CameraParam cannot scale to %dx%d",
                      image.width, image.height);
            return 2;
        }
        recharge_api_->setCameraMatrix(
            scaled->fx, scaled->fy, scaled->cx, scaled->cy);
    }

    const std::uint64_t frame_id = image.key.camera_frame_id != 0
                                       ? image.key.camera_frame_id
                                       : image.key.transport_sequence;
    const std::int64_t timestamp_ns = image.key.camera_timestamp_ns > 0
                                          ? image.key.camera_timestamp_ns
                                          : image.key.transport_timestamp_ns;
    Nv12Image normalized;
    const Nv12Image* nv12_image = &image;
    if (image.format == Yuv420Format::kI420) {
        if (!convertI420ToNv12(image, &normalized)) {
            LOG_ERROR("[PercepDemo] recharge I420 to NV12 conversion failed");
            return 3;
        }
        nv12_image = &normalized;
    }
    // This API explicitly guarantees that the SO deep-copies NV12 before
    // returning. Any normalized temporary therefore only needs to live
    // through this call, unlike the old opaque submitFromShm ABI.
    return recharge_api_->submitLeftNv12(
        nv12_image->bytes.data(), nv12_image->width, nv12_image->height,
        nv12_image->stride_y,
        frame_id, timestamp_ns);
}

void PercepDemo::publishRechargeMark(const Aiper_Recharge::MarkPoseResult* r) {
    if (!r || shutting_down_.load(std::memory_order_acquire) ||
        !mark_runtime_on_.load(std::memory_order_acquire) ||
        !recharge_host_.enabled || !recharge_host_.publish_enabled) {
        return;
    }

    const int w = r->image_w;
    const int h = r->image_h;
    const char* src = recharge_host_.use_hires
                          ? ((w >= gdc_width) ? "gdc1280" : "left640(fallback)")
                          : "left640";
    const int64_t stamp_ns = r->post.time_stamp;
    const rclcpp::Time stamp_ros(stamp_ns, RCL_ROS_TIME);
    const double mark_ms = r->process_ms;
    const uint64_t fid = r->frame_id;

    if (recharge_host_.publish_overlay && r->ok) {
        if (r->overlay_bgr && r->overlay_w > 0 && r->overlay_h > 0) {
            const int ow = r->overlay_w;
            const int oh = r->overlay_h;
            const int ostep =
                r->overlay_step > 0 ? r->overlay_step : ow * 3;
            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp_ros;
            msg.header.frame_id = "camera";
            msg.height = static_cast<uint32_t>(oh);
            msg.width = static_cast<uint32_t>(ow);
            msg.encoding = "rgb8";
            msg.is_bigendian = false;
            msg.step = static_cast<uint32_t>(ow * 3);
            msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
            for (int y = 0; y < oh; ++y) {
                const uint8_t* s =
                    r->overlay_bgr + static_cast<size_t>(y) * ostep;
                uint8_t* d =
                    msg.data.data() + static_cast<size_t>(y) * msg.step;
                for (int x = 0; x < ow; ++x) {
                    d[x * 3 + 0] = s[x * 3 + 2];
                    d[x * 3 + 1] = s[x * 3 + 1];
                    d[x * 3 + 2] = s[x * 3 + 0];
                }
            }
            recharge_mark_overlay_pub_.publish(msg);
        } else if ((++recharge_overlay_warn_tick_ % 30ull) == 1ull) {
            cerr << "[PercepDemo][recharge_mark] overlay enabled but empty"
                 << " (setBuildOverlay/g_build_overlay?)" << endl;
        }
    }

    common_interface::msg::DockPose msg;
    msg.header.stamp = stamp_ros;
    msg.header.frame_id = "camera";
    if (r->pose_ok) {
        msg.x = r->post.x;
        msg.y = r->post.y;
        msg.yaw = r->post.yaw;
    } else {
        msg.x = kDockPoseInvalidSentinel;
        msg.y = kDockPoseInvalidSentinel;
        msg.yaw = kDockPoseInvalidSentinel;
    }
    recharge_mark_pub_.publish(msg);

    if (r->pose_ok) {
        char logbuf[512];
        std::snprintf(logbuf, sizeof(logbuf),
            "[PercepDemo][recharge_mark] fid=%llu src=%s %dx%d mark_ms=%d "
            "cand=%d scaf=%d ov=%dx%d ok=1 pose_ok=1 pts=%d "
            "DockPose x=%.4f y=%.4f yaw=%.4f%s xf=%s crop=%s stamp_ns=%lld dist_mm=%.0f",
            static_cast<unsigned long long>(fid), src, w, h,
            static_cast<int>(mark_ms),
            r->debug_candidates, r->debug_scaffolds,
            r->overlay_w, r->overlay_h, r->detected_point_count,
            static_cast<double>(msg.x), static_cast<double>(msg.y),
            static_cast<double>(msg.yaw),
            r->yaw_jump_hold ? " yaw_hold=1" : "",
            r->x_filter_reason[0] ? r->x_filter_reason : "-",
            r->crop_src[0] ? r->crop_src : "-",
            static_cast<long long>(stamp_ns), r->distance_mm);
        cout << logbuf << endl;
        LOG_INFO("%s", logbuf);
    } else if (r->dist_jump_hold) {
        char logbuf[512];
        std::snprintf(logbuf, sizeof(logbuf),
            "[PercepDemo][recharge_mark] fid=%llu src=%s %dx%d mark_ms=%d "
            "cand=%d scaf=%d ov=%dx%d ok=1 pose_ok=0 filter=dist_jump "
            "raw_y=%.4f dist_mm=%.0f",
            static_cast<unsigned long long>(fid), src, w, h,
            static_cast<int>(mark_ms),
            r->debug_candidates, r->debug_scaffolds,
            r->overlay_w, r->overlay_h,
            static_cast<double>(r->post.y), r->distance_mm);
        cout << logbuf << endl;
        LOG_INFO("%s", logbuf);
    } else {
        char logbuf[640];
        int n = std::snprintf(logbuf, sizeof(logbuf),
            "[PercepDemo][recharge_mark] fid=%llu src=%s %dx%d mark_ms=%d "
            "cand=%d scaf=%d ov=%dx%d ok=%d pose_ok=0 pts=%d Pose=empty",
            static_cast<unsigned long long>(fid), src, w, h,
            static_cast<int>(mark_ms),
            r->debug_candidates, r->debug_scaffolds,
            r->overlay_w, r->overlay_h, r->ok ? 1 : 0,
            r->detected_point_count);
        if (n > 0 && n < static_cast<int>(sizeof(logbuf))) {
            if (r->error[0])
                n += std::snprintf(logbuf + n, sizeof(logbuf) - n,
                                   " err=%s", r->error);
            if (r->pnp_error[0])
                std::snprintf(logbuf + n, sizeof(logbuf) - n,
                              " pnp=%s", r->pnp_error);
        }
        cout << logbuf << endl;
        LOG_INFO("%s", logbuf);
    }
}

void PercepDemo::heightToRgb(float ros_z, float sensor_h,
                             uint8_t& r, uint8_t& g, uint8_t& b) {
    // 地面 ros_z ≈ -sensor_h，1.5m 以上视为高障碍
    const float z_ground = -sensor_h;
    const float z_high = 1.5f;
    float t = (ros_z - z_ground) / (z_high - z_ground);
    t = std::max(0.f, std::min(1.f, t));
    r = static_cast<uint8_t>(t * 255.f);
    g = static_cast<uint8_t>((1.f - t) * 255.f);
    b = 0;
}

void PercepDemo::postprocess(PointCloudT::Ptr& pcl_cloud,
                             const std::vector<uint8_t>& rgb_buf,
                             int img_w, int img_h,float fx,float fy,float cx,float cy)
{  
    if (pcl_cloud->empty()) return;
    if (visualize)
    {
        bool convert = false;
        int colored_pts = 0;
        if (pcl_color_cfg_.enabled && !rgb_buf.empty() && img_w > 0 && img_h > 0) {
            convert = pclCommonPtr->ProjectRGBFromBuffer(
                pcl_cloud, rgb_buf, img_w, img_h, fx, fy, cx, cy, &colored_pts);
        }
        this->publish3dTF(cali,pcl_cloud,"percep_demo",true);
        const bool use_height_color =!convert && pcl_color_cfg_.height_color_fallback;
        this->publish_ros_rgbcloud(pcl_cloud, ori_cloud_pub_, parent_frame,use_height_color);   
    }
    else{
        this->publish3dTF(cali,pcl_cloud,"percep_demo",false);
    }

    // pclCommonPtr->DownSample(pcl_cloud,0.02,"approximate");

    if (patchwork_cfg_.enabled) {
        runGroundSegmentation(pcl_cloud);
    }
}

// P3: 直接发布 PointCloudT（AiperPclPoint），保留 label/confidence 字段
void PercepDemo::publish_ros_rgbcloud(PointCloudT::Ptr &cloud_in, daye::RosPublisher& pub,
                                   string frame_id, bool height_color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    const float sensor_h = static_cast<float>(patchwork_cfg_.sensor_height);
    rgb_cloud->resize(cloud_in->size());
    auto& out_points = rgb_cloud->points;
    size_t idx = 0;
    Trans temp;
    temp.y  = mapWidth + 0.1;
    PointCloudT cloud = *cloud_in;
    Eigen::Matrix4f trans_cali = Eigen::Matrix4f::Identity();
    pclCommonPtr->EulerToMatrix(temp, trans_cali);
    // pcl::transformPointCloud(cloud, cloud, trans_cali);
    for (const auto& pt : cloud.points) {
        uint8_t r = pt.r, g = pt.g, b = pt.b;
        if (height_color) {
            heightToRgb(pt.z, sensor_h, r, g, b);
        }
        auto& out = out_points[idx];
        out.x = pt.x;
        out.y = pt.y;
        out.z = pt.z;
        // out.x= pt.z;     // 前
        // out.y = -pt.x;    // 左（SDK +x 为右）
        // out.z = -pt.y;    // 上（SDK +y 为下，取反）

        out.r = r;
        out.g = g;
        out.b = b;
        idx++;
    }
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*rgb_cloud, ros_msg);
    ros_msg.header.frame_id = frame_id;
    ros_msg.header.stamp = rclcpp::Clock().now();
    pub.publish(ros_msg);
}


void PercepDemo::publish3dTF(Trans &cali,const PointCloudT::Ptr& pcl_cloud,string frame_id,bool pubTf)
{
        
    Eigen::Matrix4f trans_cali = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q = pclCommonPtr->EulerToMatrix(cali, trans_cali);
    pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, trans_cali);
    if (pubTf)
    {
        tf2::Quaternion qtn;
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp=rclcpp::Clock().now();
        ts.header.frame_id=parent_frame; 
        ts.child_frame_id=frame_id;
        ts.transform.translation.x=cali.x;
        ts.transform.translation.y=cali.y;
        ts.transform.translation.z=cali.z;   
        qtn.setX(q.x());
        qtn.setY(q.y());
        qtn.setZ(q.z());
        qtn.setW(q.w());
        ts.transform.rotation.x=qtn.getX();
        ts.transform.rotation.y=qtn.getY();
        ts.transform.rotation.z=qtn.getZ();
        ts.transform.rotation.w=qtn.getW();
        broadcaster->sendTransform(ts);          
    }
 
}
void PercepDemo::convertGridMap(int count)
{
    const auto geometry =
        detail::TryMakeGridGeometry(mapWidth, mapHeight, mapResolution);
    if (!geometry.has_value()) {
        LOG_ERROR("[PercepDemo] skip invalid/oversized grid geometry");
        return;
    }
    const int cols = static_cast<int>(geometry->cols);  //列数
    const int rows = static_cast<int>(geometry->rows);  //行数 

    OccMapData fusion_data;
    cout<<"AI处理后点云个数："<<last_semantic_cloud_->size()<<endl;

    // Honor the configured grid-fusion policy. Previously `fuse_into_grid`
    // was parsed and logged but ignored, so semantic labels were always fed
    // into the occupancy fusion stage.
    PointCloudT::Ptr semantic_for_grid;
#if defined(PERCEP_DNN_ENABLE)
    if (label_fusion_cfg_.enabled && label_fusion_cfg_.fuse_into_grid) {
        semantic_for_grid = last_semantic_cloud_;
    }
#endif
    aiFusionPtr->initData(ground_cloud, obs_cloud, semantic_for_grid);
    aiFusionPtr->integrate(fusion_data);

    string time_str = frame_save_time_str_.empty()
                          ? std::to_string(daye::getSystemTimeMs())
                          : frame_save_time_str_;
    cout<<"current frame timestamp:"<<time_str<<endl;
    common_interface::msg::GridMapInfo grid_msg;
    grid_msg.header.stamp = rclcpp::Clock().now();
    grid_msg.header.frame_id = parent_frame;
    grid_msg.rows = geometry->rows;
    grid_msg.cols = geometry->cols;
    grid_msg.resolution_mm = geometry->resolution_mm;
    grid_msg.avg_h_cm = fusion_data.map_avg_h;
    grid_msg.min_h_cm = fusion_data.map_min_h;
    grid_msg.max_h_cm = fusion_data.map_max_h;
    grid_msg.type = fusion_data.type;
    grid_map_pub_.publish(grid_msg);

    if(visualize){
        this->publishColoredMap(fusion_data.pcl_map,"pcl",cols,rows,count);
        this->publishColoredMap(fusion_data.ai_map,"ai",cols,rows,count);
        this->publishColoredMap(fusion_data.fusion_map,"fusion",cols,rows,count);
    }
 
    if (save_log)
    {
        auto new_task = std::make_unique<SaveMapTask>();
        new_task->map_data = fusion_data;                 // vector深拷贝
        new_task->rgb = frame_rgb_buf_;   
        new_task->gdc = std::move(gdc_for_save_);
        new_task->disp = disp_vec;
        new_task->u = u_vec;
        new_task->v = v_vec;
        new_task->time_str = time_str;
        new_task->rows = rows;
        new_task->cols = cols;

        {
            std::lock_guard<std::mutex> lk(save_map_mtx_);
            // 如果存在未处理任务，直接丢弃旧任务，覆盖为最新帧
            if(pending_task_.has_value())
            {
                cout<<"IO busy, drop old save task, keep latest frame:"<<time_str.c_str()<<endl;
            }
            pending_task_ = std::move(new_task);
        }
        cv_save_map.notify_one();
    }
}
void PercepDemo::saveLogLoop()
{
    while (!save_map_exit_)
    {
        std::unique_lock<std::mutex> lk(save_map_mtx_);
        cv_save_map.wait(lk, [this]() {
            return save_map_exit_.load() || pending_task_.has_value();
        });

        if (save_map_exit_)
        {
            break;
        }

        // 取出任务，清空待处理
        std::unique_ptr<SaveMapTask> task = std::move(pending_task_.value());
        pending_task_.reset();
        lk.unlock(); // 提前解锁，不阻塞timer生产者
        if (save_data && task->gdc)
        {
            try
            {
                std::vector<uint8_t> frame_gdc_buf_;
                if (decodeYuv420ToRgb(task->gdc->left, frame_gdc_buf_)) {
                    visualizePtr->saveGDC(frame_gdc_buf_, task->time_str);
                } else {
                    LOG_ERROR("[PercepDemo] skip GDC save: YUV420 decode failed");
                }
                // visualizePtr->saveRGB(task->rgb, task->time_str);
                visualizePtr->saveDisp(task->u,task->v,task->disp,task->time_str);
                pclCommonPtr->keepNewestFiles(output_dir + "/data/gdc/", 30000);                
                // pclCommonPtr->keepNewestFiles(output_dir + "/data/rgb/", 30000);
                pclCommonPtr->keepNewestFiles(output_dir + "/data/disp/", 30000);
            }
            catch (...)
            {
                cerr<< "async save data exception frame:"<<task->time_str.c_str()<<endl;
            }
        }
        if (save_map)
        {
            try
            {
                visualizePtr->saveMap(task->map_data.pcl_map, "pcl", task->time_str, task->rows, task->cols);
                visualizePtr->saveMap(task->map_data.ai_map, "ai", task->time_str, task->rows, task->cols);
                visualizePtr->saveMap(task->map_data.fusion_map, "fusion", task->time_str, task->rows, task->cols);
                pclCommonPtr->keepNewestFiles(output_dir + "/map/pcl/", 30000);
                pclCommonPtr->keepNewestFiles(output_dir + "/map/ai/", 30000);
                pclCommonPtr->keepNewestFiles(output_dir + "/map/fusion/", 30000);
            }
            catch (...)
            {
                cerr<< "async save map exception frame:"<<task->time_str.c_str()<<endl;
            }            
        } 
    }
}
bool PercepDemo::saveCamParams(const common_interface::msg::CameraParam &params)
{
    fs::path dir(output_dir);
    if (!dir.empty() && !fs::exists(dir)) {
        fs::create_directories(dir);
    }
    string save_path = output_dir+"/camera_param.json";
    FILE* fp = fopen(save_path.c_str(), "w");
    if(nullptr == fp)
    {
        std::cerr << "[saveCameraParamToJson] open failed: " << save_path << std::endl;
        return false;
    }

    char writeBuf[1024];
    rapidjson::FileWriteStream os(fp, writeBuf, sizeof(writeBuf));
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(os);

    writer.StartObject();  // 唯一根对象

    writer.Key("intrinsics");
        writer.StartObject(); 
        writer.Key("fx"); writer.Double(roundDouble(params.fx,2));
        writer.Key("fy"); writer.Double(roundDouble(params.fy,2));
        writer.Key("cx"); writer.Double(roundDouble(params.cx,2));
        writer.Key("cy"); writer.Double(roundDouble(params.cy,2));
        writer.Key("bxf"); writer.Double(roundDouble(params.bxf,2));
        writer.Key("base_line"); writer.Double(roundDouble(params.base_line,2));
        writer.Key("width"); writer.Uint(roundDouble(params.width,2));
        writer.Key("height"); writer.Uint(roundDouble(params.height,2));
        writer.EndObject();
    
    writer.Key("extrinsics");
        writer.StartObject(); 
        writer.Key("roll"); writer.Double(roundDouble(rad_to_angle(cali.roll),2));
        writer.Key("pitch"); writer.Double(roundDouble(rad_to_angle(cali.pitch),2));
        writer.Key("yaw"); writer.Double(roundDouble(rad_to_angle(cali.yaw),2));
        writer.Key("x"); writer.Double(roundDouble(cali.x,2));
        writer.Key("y"); writer.Double(roundDouble(cali.y,2));
        writer.Key("z"); writer.Double(roundDouble(cali.z,2));
        writer.EndObject();
        
    writer.Key("interestFov");
        writer.StartObject(); 
        writer.Key("bbox_minX"); writer.Double(roundDouble(pcl_pub_cfg_.box_min_x,2));
        writer.Key("bbox_minY"); writer.Double(roundDouble(pcl_pub_cfg_.box_min_y,2));
        writer.Key("bbox_minZ"); writer.Double(roundDouble(pcl_pub_cfg_.box_min_z,2));
        writer.Key("bbox_maxX"); writer.Double(roundDouble(pcl_pub_cfg_.box_max_x,2));
        writer.Key("bbox_maxY"); writer.Double(roundDouble(pcl_pub_cfg_.box_max_y,2));
        writer.Key("bbox_maxZ"); writer.Double(roundDouble(pcl_pub_cfg_.box_max_z,2));
        writer.EndObject();       

    writer.Key("sample_step");writer.Int(sample_step);

    writer.EndObject();            // 闭合最外层
    fclose(fp);
    std::cout << "[saveCameraParamToJson] save camera param ok: " << save_path << std::endl; 
    return true;  
}
void PercepDemo::publishColoredMap(const std::vector<MapPoint>& map_points,string header,int cols,int rows,int index)
{
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
    {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = parent_frame;
        delete_marker.ns = header;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.push_back(delete_marker);
        if (header == "pcl"){pclmap_pub_.publish(*marker_array);}
        if (header == "ai"){aimap_pub_.publish(*marker_array);}
        if (header == "fusion"){fusionmap_pub_.publish(*marker_array);}
    }
    marker_array->markers.clear();
    // 创建一个Marker用于显示彩色点
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = rclcpp::Clock().now();
    marker.header.frame_id = parent_frame;
    marker.ns = header;
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1); 
    // 设置点的大小
    marker.scale.x = mapResolution;  // 点的大小与分辨率匹配
    marker.scale.y = mapResolution;
    marker.scale.z = 0.01;  // 稍微有点高度
    
   // 设置姿态（无旋转）
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    
    // 原点位置
    marker.pose.position.x = 0.0;
    if (header == "fusion"){marker.pose.position.y = mapWidth / 2.0;}
    if (header == "ai"){marker.pose.position.y = mapWidth / 2.0 + mapWidth + 0.1;}
    if (header == "pcl"){marker.pose.position.y = mapWidth / 2.0 - mapWidth - 0.1;}
    marker.pose.position.z = 0.0;


    std_msgs::msg::ColorRGBA colors[] = {
        this->createColorRGBA(255, 0, 0),     // 红色
        this->createColorRGBA(0, 255, 0),     // 绿色
        this->createColorRGBA(0, 0, 255),     // 蓝色
        this->createColorRGBA(255, 255, 0),   // 黄色
        this->createColorRGBA(255, 165, 0),   // 橙色
        this->createColorRGBA(255, 0, 255),   // 粉色
        this->createColorRGBA(255, 255, 255), // 白色
        this->createColorRGBA(0, 0, 0)        // 黑色
    };
    for (int i=0;i<map_points.size();i++) {
        int map_x = i % cols; //列索引
        int map_y = i / cols ;  //行索引

        geometry_msgs::msg::Point p;
        p.x = (map_y + 0.5) * mapResolution;
        p.y = -(map_x + 0.5) * mapResolution; 
        p.z = map_points[i].avg_h; 
        marker.points.push_back(p);
        int label = map_points[i].type;

        std_msgs::msg::ColorRGBA color;
        //0=未知, 1=非草, 2=其他障碍，3 =草，>=4 ai
        if (label == 0){color=colors[3];}  //"unknown"
        if (label == 1 ){color=colors[7];} //"nonegrass"
        if (label == 2 ){color=colors[0];} //"obs"
        if (label == 3){color=colors[1];} //"grass"
        if (label > 3) {color=colors[5];}
        marker.colors.push_back(color);
    }

    marker_array->markers.push_back(marker);
    if (header == "pcl"){pclmap_pub_.publish(*marker_array);}
    if (header == "ai"){aimap_pub_.publish(*marker_array);}
    if (header == "fusion"){fusionmap_pub_.publish(*marker_array);}
}
std_msgs::msg::ColorRGBA PercepDemo::createColorRGBA(uint8_t r, uint8_t g, uint8_t b)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r / 255.0f;
    color.g = g / 255.0f;
    color.b = b / 255.0f;
    color.a = 1.0f;
    return color;
}
void PercepDemo::publish_ros_cloud(PointCloudT::Ptr &cloud_in,daye::RosPublisher& pub,string frame_id)
{
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud_in, ros_msg);
    ros_msg.header.frame_id = frame_id;
    ros_msg.header.stamp = rclcpp::Clock().now();
    pub.publish(ros_msg);
}

void PercepDemo::callAppService(int32_t cmd, int32_t status,double timeout_us) 
{
  common_interface::srv::ToAppCmd::Request app_request;
  common_interface::srv::ToAppCmd::Response app_response;
  app_request.cmd = cmd;
  app_request.status = status;
  if (!appCmd_client_.call<common_interface::srv::ToAppCmd>(app_request, app_response, timeout_us)) {
    cerr<< "[PercepDemo err] call app failed " <<endl;
    return;
  }else{
    cout<<"[PercepDemo] received app result:"<<app_response.result<<endl;
  }
}

bool PercepDemo::decodeYuv420ToRgb(const Nv12Image& image,
                                   std::vector<uint8_t>& rgb_buf) {
    if (!pclCommonPtr || !image.valid()) {
        return false;
    }
    Nv12Image normalized;
    const Nv12Image* nv12_image = &image;
    if (image.format == Yuv420Format::kI420) {
        if (!convertI420ToNv12(image, &normalized)) {
            LOG_ERROR("[PercepDemo] I420 to NV12 conversion failed");
            return false;
        }
        nv12_image = &normalized;
    }
    if (!pclCommonPtr->yuv_nv12_to_rgb24(
            nv12_image->bytes.data(), nv12_image->width, nv12_image->height,
            nv12_image->stride_y, &rgb_buf)) {
        cerr << "[PercepDemo Err] YUV420 convert to rgb24 failed" << endl;
        return false;
    }

    std::size_t pixels = 0;
    std::size_t expected_bytes = 0;
    if (!checkedMultiply(static_cast<std::size_t>(image.width),
                         static_cast<std::size_t>(image.height), &pixels) ||
        !checkedMultiply(pixels, 3U, &expected_bytes) ||
        rgb_buf.size() < expected_bytes) {
        LOG_ERROR("[PercepDemo] RGB decoder returned a short buffer");
        return false;
    }
    return true;
}

void PercepDemo::fillRgbRosImage(const Nv12Image& image,
                                 const std::vector<uint8_t>& rgb_buf,
                                 sensor_msgs::msg::Image& ros_img) {
    const auto timestamp_ns = image.key.camera_timestamp_ns > 0
                                  ? image.key.camera_timestamp_ns
                                  : image.key.transport_timestamp_ns;
    ros_img.header.stamp = timestamp_ns > 0
                               ? rclcpp::Time(timestamp_ns, RCL_ROS_TIME)
                               : rclcpp::Clock().now();
    ros_img.width = static_cast<std::uint32_t>(image.width);
    ros_img.height = static_cast<std::uint32_t>(image.height);
    ros_img.encoding = "rgb8";
    ros_img.step = static_cast<std::uint32_t>(image.width * 3);
    ros_img.data.assign(rgb_buf.begin(), rgb_buf.end());
}

bool PercepDemo::ResizeRosImage(sensor_msgs::msg::Image& ros_img,std::vector<uint8_t> &publish_rgb_buf_,
                                uint32_t dst_w, uint32_t dst_h)
{
    std::lock_guard<std::mutex> lock(resize_mtx); // 函数生命周期独占锁
    if (ros_img.encoding != "rgb8" || ros_img.data.empty()) return false;
    if (ros_img.width == dst_w && ros_img.height == dst_h) return true;

    if (!pclCommonPtr->rgb8_resize(
            ros_img.data,    // 使用快照内存，不直接引用ros_img
            static_cast<int>(ros_img.width),
            static_cast<int>(ros_img.height),
            publish_rgb_buf_,
            static_cast<int>(dst_w),
            static_cast<int>(dst_h)))
    {
        std::cerr << "[PercepDemo Err] resize" << ros_img.width << "x" << ros_img.height
                   << "->" << dst_w << "x" << dst_h << "failed" << std::endl;
        return false;
    }

    ros_img.width = dst_w;
    ros_img.height = dst_h;
    ros_img.step = dst_w * 3;
    ros_img.data.assign(publish_rgb_buf_.begin(), publish_rgb_buf_.end());
    return true;
}


void PercepDemo::runGroundSegmentation(const PointCloudT::Ptr& raw_pts) 
{
    if (!patchwork_ || raw_pts->empty()) return;
    auto t_pw = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf cloud_mat(raw_pts->size(), 3);  
    for (size_t i = 0; i < raw_pts->size(); ++i) {
      cloud_mat.row(i) << raw_pts->points[i].x, raw_pts->points[i].y, raw_pts->points[i].z;
    }
    ground_cloud.reset(new PointCloudT);
    no_ground_cloud.reset(new PointCloudT);
    patchwork_->estimateGround(cloud_mat);
    Eigen::MatrixX3f ground = patchwork_->getGround();
    Eigen::MatrixX3f nonground = patchwork_->getNonground();
        
    vector<Eigen::VectorXf> ground_normals = patchwork_->getNormalsVec();
    vector<Eigen::VectorXf> ground_centers = patchwork_->getCentersVec();
    vector<double> d_vec = patchwork_->getDVec();    
    ground_cloud = pclCommonPtr->eigenMatrixToPointCloud(ground);
    no_ground_cloud = pclCommonPtr->eigenMatrixToPointCloud(nonground);
    double ground_dist = 0.05;
    if (!ground_normals.empty() && !ground_centers.empty())
    {
      
        double ground_pitch = std::acos(std::abs(ground_normals[0](2))) * 180.0 / M_PI;
        Eigen::Vector3f ground_normal = ground_normals[0].head<3>();
        Eigen::Vector3f ground_center = ground_centers[0].head<3>();  

        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
        PointCloudT::Ptr ground_cloud_extra(new PointCloudT);
        std::vector<float> distances;

        ground_indices = pclCommonPtr->extractPointPlaneDis(no_ground_cloud,ground_dist,ground_center,ground_normal,distances);
        pclCommonPtr->extractCloudFromIndices(no_ground_cloud,ground_indices,ground_cloud_extra);
        *ground_cloud = *ground_cloud + *ground_cloud_extra;

        // cout<<"地面提取成功，采用地面法向量滤波保护"<<endl;       
        if (!distances.empty()){
          float min_dist = *std::min_element(distances.begin(), distances.end());
          float max_dist = *std::max_element(distances.begin(), distances.end());
        //   cout<<"ground extract: min_dist:"<<min_dist<<",max_dist:"<<max_dist<<endl;      
        }  
      
    }
    else{
        // cout<<"地面提取失败，采用直通滤波滤波保护"<<endl;
        PointCloudT::Ptr ground_cloud_extra2 = pclCommonPtr->passThrough_filter(*no_ground_cloud,"z",ground_dist,100); 
        *ground_cloud = *ground_cloud + *ground_cloud_extra2;
    }

    {
      auto t_pw_end = std::chrono::high_resolution_clock::now();
      auto pw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          t_pw_end - t_pw);
      std::cout << "patchwork Time taken: " << pw_ms.count() << "ms"
                << std::endl;
    }

auto start = std::chrono::high_resolution_clock::now();    
    int MinClusterSize = 20;
    int MaxClusterSize = 20000;
    float cluster_tolerance = 0.1;
    pclCommonPtr->DownSample(no_ground_cloud,0.04,"uniform");
    cout<<"后处理滤波前的非地面点云数量："<<no_ground_cloud->points.size()<<endl;
    pclCommonPtr->RadiusOutlierRemoval(no_ground_cloud,0.1,10);
    pclCommonPtr->StatisticalOutlierRemoval(no_ground_cloud,20,1.0);   
        
    // pclCommonPtr->DownSample(ground_cloud,0.04,"uniform");
    // cout<<"后处理滤波前的地面点云数量："<<ground_cloud->points.size()<<endl;
    // pclCommonPtr->RadiusOutlierRemoval(ground_cloud,0.05,5);
    // pclCommonPtr->StatisticalOutlierRemoval(ground_cloud,20,1.0);   

    // auto obs_indices = pclCommonPtr->euclideanClustering(no_ground_cloud,cluster_tolerance,MinClusterSize,MaxClusterSize);
    // for (size_t i = 0; i < obs_indices.size(); ++i)
    // {
    //     pcl::PointIndices::Ptr indices_ptr = std::make_shared<pcl::PointIndices>(obs_indices[i]);
    //     PointCloudT::Ptr cluster_cloud(new PointCloudT);
    //     pclCommonPtr->extractCloudFromIndices(no_ground_cloud,indices_ptr,cluster_cloud);
    //     *obs_cloud += *cluster_cloud;
    // }
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end- start);
cout<<"filter Time taken: "<<duration.count()<<"ms"<<endl;        
    obs_cloud = no_ground_cloud;

    if (visualize)
    {
        if (patchwork_cfg_.publish_ground) {
            this->publish_ros_cloud(ground_cloud, ground_cloud_pub_,parent_frame);
        }
        if (patchwork_cfg_.publish_nonground) {
            this->publish_ros_cloud(no_ground_cloud, nonground_cloud_pub_,parent_frame);
        }        
    }


}   
void PercepDemo::autocali(PointCloudT::Ptr &pcl_cloud)
{
    PointCloudT ori_cloud = *pcl_cloud;
    // pcl_cloud = pclCommonPtr->passThrough_filter(*pcl_cloud,"z",1,100);
    Eigen::Matrix4f trans_ori = Eigen::Matrix4f::Identity();
    Trans default_trans;
    default_trans.x = 0.0;
    default_trans.y = 0.0;
    default_trans.z = 0.0;
    default_trans.roll = angle_to_rad(-90.0);
    default_trans.pitch = angle_to_rad(0.0);
    default_trans.yaw = angle_to_rad(-90.0);
    pclCommonPtr->EulerToMatrix(default_trans, trans_ori);
    pcl::transformPointCloud(*pcl_cloud,*pcl_cloud,trans_ori);

    float roll_ori,pitch_ori,yaw_ori;
    
    double cali_h = pclCommonPtr->calculate_by_normal(*pcl_cloud,roll_ori,pitch_ori,yaw_ori);
    float roll_cali = -rad_to_angle(pitch_ori)-90;
    float pitch_cali = rad_to_angle(roll_ori);
    float yaw_cali = -90;
    sum_roll = roll_cali + sum_roll;
    sum_pitch = pitch_cali + sum_pitch;
    sum_h = cali_h + sum_h;
    
    default_trans.roll = angle_to_rad(roll_cali);
    default_trans.pitch = angle_to_rad(pitch_cali);
    default_trans.yaw = angle_to_rad(yaw_cali);
    // cout<<"AutoCalibrated Roll:"<<roll_cali<<" Pitch: "<<pitch_cali<<" Yaw: "<<yaw_cali<<endl;
    Eigen::Matrix4f trans_cali = Eigen::Matrix4f::Identity();
    pclCommonPtr->EulerToMatrix(default_trans, trans_cali);
    pcl::transformPointCloud(ori_cloud,ori_cloud,trans_cali);
    cali_count++;
    if (cali_count>5)
    {
        string extrinsics_config_dir = "/app/daye/config/ai_node/extrinsics.json";
        std::ifstream ifs(extrinsics_config_dir, std::ios::binary);
        if (!ifs.is_open()) {
            cerr << "[AutoCali err] 打开文件失败: " << extrinsics_config_dir << std::endl;
            return ;
        }
        std::stringstream ss;
        ss << ifs.rdbuf();
        string config = ss.str();
        ifs.close();  

        rapidjson::Document ai_node_json;
        ai_node_json.Parse(config.c_str());
        if (ai_node_json.HasParseError()){
        cerr << "[AutoCali err] json解析失败: "<< std::endl;
        return ;
        } 
        double avg_roll = sum_roll / cali_count;
        double avg_pitch = sum_pitch / cali_count;
        double avg_h = sum_h / cali_count;
        
        if (ai_node_json.HasMember("extrinsics") &&
            ai_node_json["extrinsics"].IsObject()) {
            rapidjson::Value& pp = ai_node_json["extrinsics"];  
            pp["z"].SetDouble(roundDouble(avg_h,2));
            pp["roll"].SetDouble(roundDouble(avg_roll,2));   
            pp["pitch"].SetDouble(roundDouble(avg_pitch,2));
            pp["yaw"].SetDouble(roundDouble(yaw_cali,2));  
        }
        rapidjson::StringBuffer buffer;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        writer.SetIndent(' ', 4);
        ai_node_json.Accept(writer);

        std::ofstream ofs(extrinsics_config_dir, std::ios::binary);
        if(!ofs.is_open())
        {
            std::cerr << "[AutoCali err] 写配置文件失败 "<< extrinsics_config_dir << std::endl;
            return ;
        }
        ofs << buffer.GetString();
        ofs.close();
        std::cout << "标定参数已写入文件: Roll:" << avg_roll<<" ,Pitch:"<<avg_pitch<<" ,z:"<<avg_h<<std::endl;        
        cali_count = 0;sum_roll = 0;sum_pitch = 0; sum_h = 0;
        return;
    }

}

}  // namespace ai_node_percep_demo
