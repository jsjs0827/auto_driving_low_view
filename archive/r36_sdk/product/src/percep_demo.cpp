/**
 * @file percep_demo.cpp
 * @brief Minimal pub/sub/timer function example.
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved.
 */
#include "percep_demo/percep_demo.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <type_traits>
#include <utility>
#include "product/status_api.hpp"
#include "product/disparity_projection.hpp"
#include "product/command_contract.hpp"
#include "product/ros_io_identity.hpp"
static std::mutex resize_mtx; // 全局静态锁

namespace {

static_assert(std::is_trivially_copyable<CameraData>::value &&
              std::is_trivially_copyable<CameraGdcData>::value,
              "Original camera SHM messages must support complete byte copies");
template<class T> constexpr bool inlineByteArray = std::is_array<T>::value &&
    std::rank<T>::value == 1 && sizeof(typename std::remove_all_extents<T>::type) == 1;
static_assert(inlineByteArray<decltype(std::declval<CameraData>().left_data)> &&
              inlineByteArray<decltype(std::declval<CameraData>().right_data)> &&
              inlineByteArray<decltype(std::declval<CameraData>().disp_data)> &&
              inlineByteArray<decltype(std::declval<CameraGdcData>().left_data)>,
              "Original camera SHM payloads must be inline byte arrays, never serialized pointers");

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

#if PERCEPTION_PRODUCT_WITH_RECHARGE
// YOLO 检测图坐标 → 回充全图（GDC 1280 或 left 640）；扩边 + NV12 偶数对齐
bool BuildChargerCropOnMarkImage(float x1, float y1, float x2, float y2,
                                 int det_w, int det_h, int mark_w, int mark_h,
                                 float pad_ratio, int pad_px, int min_w,
                                 int min_h, int* ox, int* oy, int* ow,
                                 int* oh) {
    if (!ox || !oy || !ow || !oh || det_w <= 0 || det_h <= 0 || mark_w <= 0 ||
        mark_h <= 0) {
        return false;
    }
    const double sx = static_cast<double>(mark_w) / static_cast<double>(det_w);
    const double sy = static_cast<double>(mark_h) / static_cast<double>(det_h);
    double mx1 = x1 * sx;
    double my1 = y1 * sy;
    double mx2 = x2 * sx;
    double my2 = y2 * sy;
    if (mx2 < mx1) std::swap(mx1, mx2);
    if (my2 < my1) std::swap(my1, my2);
    const double bw = mx2 - mx1;
    const double bh = my2 - my1;
    if (bw < min_w || bh < min_h) return false;
    const double px =
        std::max(static_cast<double>(pad_px) * sx, bw * pad_ratio);
    const double py =
        std::max(static_cast<double>(pad_px) * sy, bh * pad_ratio);
    int x0 = static_cast<int>(std::floor(mx1 - px));
    int y0 = static_cast<int>(std::floor(my1 - py));
    int x1i = static_cast<int>(std::ceil(mx2 + px));
    int y1i = static_cast<int>(std::ceil(my2 + py));
    x0 = std::max(0, x0) & ~1;
    y0 = std::max(0, y0) & ~1;
    x1i = std::min(mark_w, (x1i + 1) & ~1);
    y1i = std::min(mark_h, (y1i + 1) & ~1);
    if (x1i - x0 < min_w || y1i - y0 < min_h) return false;
    *ox = x0;
    *oy = y0;
    *ow = x1i - x0;
    *oh = y1i - y0;
    return true;
}
#endif

}  // namespace


REGISTER_FUNCTION(factory) {
  cout << "[PercepDemo] register collect function, function_name:" << FUNCTION_NAME << endl;
  factory.registerCreater<ai_node_percep_demo::PercepDemo>(FUNCTION_NAME);
}

namespace ai_node_percep_demo {

PercepDemo::PercepDemo(const rapidjson::Value& json_conf) try : daye::Function(json_conf) {
    // Clean up started threads before member destruction on constructor failure.
    try {
    if(!this->initParams()) {
        r28::Status status; status.code=1004; status.detail="product configuration initialization failed";
        r28::product::setLastStatus(status); throw r28::Failure(status);
    }
    product_dag_ = std::make_unique<r28::product::ProductDag>(product_dag_path_,product_metrics_dir_);
    product_events_ = std::make_unique<r30::product::ProductEvents>(product_events_path_,product_metrics_dir_);
    sub_callback_group_ = daye::CreateCallbackGroup(daye::GroupType::MutuallyExclusive);
    global_json.CopyFrom(json_conf, global_json.GetAllocator());
    if(!this->initInterface()) {
        r28::Status status; status.code=1004; status.detail="product interfaces initialization failed";
        r28::product::setLastStatus(status); throw r28::Failure(status);
    }
    r31::product::writeRosIoReceiptFromEnvironment();
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
        percep_status = 0;        
    }
    else{
        //1.RGB 2.GDC 3.同时启动 4.全部停止
        std_msgs::msg::UInt8 camera_request;
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        camera_request.data = 3;
#else
        camera_request.data = 1;
#endif
        camera_request_pub.publish(camera_request);
        percep_map = true; 
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        percep_station = true;
        if(!startRechargeMarkWorker()) percep_status=-1;
#else
        percep_station = false;
        percep_status = 1;
#endif
    }
    } catch(...) {cleanupProductResources();throw;}
} catch(const std::exception& error) {
    r28::product::setLastStatus(r28::statusFromException(error,1004,"product.init"));
    throw;
}

PercepDemo::~PercepDemo() {
    cleanupProductResources();
}

void PercepDemo::cleanupProductResources() noexcept {
    shutting_down_.store(true,std::memory_order_release);
    percep_map=false;percep_station=false;g_running=false;
    try {if(timer_started_) {timer_.stop();timer_started_=false;}} catch(...) {}
    rgb_data_cond_.notify_all();gdc_data_cond_.notify_all();
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    mark_done_cv_.notify_all();
#endif
    try {
        std::lock_guard<std::mutex> wait_timer(timer_run_mtx_);
#if defined(PERCEP_DNN_ENABLE)
        dnn_runner_.reset();
#endif
    } catch(...) {}
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    try {stopRechargeMarkWorker();} catch(...) {}
    if(recharge_api_) {
        try {if(recharge_api_->destroy) recharge_api_->destroy(false);} catch(...) {}
        delete recharge_api_;recharge_api_=nullptr;
    }
#endif
    save_map_exit_=true;cv_save_map.notify_one();
    try {if(saveLogThreadHandle.joinable()) saveLogThreadHandle.join();} catch(...) {}
    try {subCamera=daye::ShmSubscriber{};subCameraGdc=daye::ShmSubscriber{};} catch(...) {}
}

void PercepDemo::onCameraFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info) {
    if (shutting_down_.load(std::memory_order_acquire)) return;
    if (data == nullptr || size != sizeof(CameraData)) {
    std::cerr << "invalid camera rgb frame size: expected=" << sizeof(CameraData)
                << ", actual=" << size << '\n';
    std::lock_guard<std::mutex> lock(rgb_mtx_);
    camera_status = -1;
    return;
    }
    std::cout << "Latest RGB current time_ms:" << daye::getSystemTimeMs() << std::endl;
    // 生产者：上锁直接覆盖最新帧，旧帧直接丢弃，不排队
    std::unique_lock<std::mutex> lock(rgb_mtx_);
    // while (rgb_buffer_queue.size() >= 12) rgb_buffer_queue.pop();

    CameraData latest_camera_data_{};
    std::memcpy(&latest_camera_data_, data, sizeof(CameraData));
    std::queue<CameraData>().swap(rgb_buffer_queue);
    rgb_buffer_queue.push(latest_camera_data_);
    rgb_data_cond_.notify_one(); 
    camera_status = 1;
    // cout<<"收到RGB最新数据!!!!!!!"<<endl;

    // std::cout << "sequence=" << info.sequence
    //         << ", shm_timestamp_ns=" << info.timestamp_ns
    //         << ", left={frame_id:" << latest_camera_data_.left.frame_id
    //         << ", timestamp_ns:" << latest_camera_data_.left.time_ns
    //         << ", resolution:" << latest_camera_data_.left.width << 'x' << latest_camera_data_.left.height
    //         << ", bytes:" << latest_camera_data_.left.bytes_used
    //         << ", exposure_ms:" << latest_camera_data_.left.exp_time
    //         << ", analog_gain:" << latest_camera_data_.left.analog_gain
    //         << "}, right={frame_id:" << latest_camera_data_.right.frame_id
    //         << ", timestamp_ns:" << latest_camera_data_.right.time_ns
    //         << ", resolution:" << latest_camera_data_.right.width << 'x' << latest_camera_data_.right.height
    //         << ", bytes:" << latest_camera_data_.right.bytes_used
    //         << ", exposure_ms:" << latest_camera_data_.right.exp_time
    //         << ", analog_gain:" << latest_camera_data_.right.analog_gain
    //         << "}, disparity_bytes=" << DISP_SIZE << '\n';
    // std::cout << "end time_ms:" << daye::getSystemTimeMs() << std::endl;
}

void PercepDemo::onGdcFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info) {
    if (shutting_down_.load(std::memory_order_acquire)) return;
    if (data == nullptr || size != sizeof(CameraGdcData)) {
        std::cerr << "invalid gdc frame size: expected=" << sizeof(CameraGdcData)
                  << ", actual=" << size << '\n';
        std::lock_guard<std::mutex> lock(gdc_mtx_);
        camera_status = -1;
        return;
    }

    std::cout << "Latest Gdc current time_ms:" << daye::getSystemTimeMs() << std::endl;
    std::unique_lock<std::mutex> lock(gdc_mtx_);
    // while (gdc_buffer_queue.size() >= 12) gdc_buffer_queue.pop();
    
    CameraGdcData latest_gdc_data_{};
    std::memcpy(&latest_gdc_data_, data, sizeof(CameraGdcData));
    std::queue<CameraGdcData>().swap(gdc_buffer_queue);
    gdc_buffer_queue.push(latest_gdc_data_);
    gdc_data_cond_.notify_one();     
    // cout<<"收到GDC最新数据!!!!!!!"<<endl;
    camera_status = 1;

    (void)info;
}

void PercepDemo::onCamParams(const common_interface::msg::CameraParam &params)
{
    r30::product::ProductEvents::Bindings bindings;
    bindings["params"] = [&] {
        if(params.width<=0||params.height<=0||!std::isfinite(params.fx)||!std::isfinite(params.fy)||
           !std::isfinite(params.cx)||!std::isfinite(params.cy)||!std::isfinite(params.bxf)||
           params.fx<=0||params.fy<=0||params.bxf<=0) {
            r28::Status status;status.code=1002;status.detail="camera parameters have invalid geometry or intrinsics";
            throw r28::Failure(status);
        }
        cout<<"收到相机参数：fx:"<<params.fx<<" fy:"<<params.fy<<" bxf:"<<params.bxf<<" base_line:"<<params.base_line<<" width:"<<params.width<<" height:"<<params.height<<endl;
        std::lock_guard<std::mutex> timer_lock(timer_run_mtx_);
        camera_params=params;receive_prams=true;
    };
    if(save_log) bindings["params_save"] = [&] {
        if(!saveCamParams(params)) throw std::runtime_error("camera parameter JSON write failed");
    };
    runProductEvent("camera_params",bindings);
}

bool PercepDemo::percepStatus_callback(common_interface::srv::GetPercepStatus::Request& req,common_interface::srv::GetPercepStatus::Response& res)
{
  //查询感知状态
    if (req.cmd == 1)
    {
        cout<<"[PercepDemo] 收到感知查询请求"<<endl;
        // -1 - 异常，0 - 待机中，1 - 感知地图发布中，2 - 基站二维码发布中 3 - 同时发布二维码和地图
        res.result = percep_status; 
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
    const auto code=runProductEvent("command",{{"command",[&] {
        std::lock_guard<std::mutex> timer_lock(timer_run_mtx_);
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        const bool recharge_ready=recharge_host_.enabled&&recharge_host_.publish_enabled&&
            recharge_api_&&recharge_api_->startWorker&&recharge_api_->stopWorker&&
            recharge_api_->canSubmit&&recharge_api_->submitFromShm&&recharge_api_->setResultCallback;
#else
        if(req.cmd==2||req.cmd==3) {
            r28::Status status;status.code=2004;status.feature="product.control.command";
            status.detail="recharge command is unsupported: product was built with PERCEPTION_PRODUCT_WITH_RECHARGE=OFF";
            throw r28::Failure(status);
        }
        const bool recharge_ready=false;
#endif
        const auto plan=r30::product::commandPlan(req.cmd,recharge_ready);
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        const bool previous_map=percep_map,previous_station=percep_station;
        const bool worker_was_running=mark_runtime_on_.load(std::memory_order_acquire);
        bool worker_started_here=false;
#endif
        bool stop_completed=false;
        const auto stop_recharge=[&] {
#if PERCEPTION_PRODUCT_WITH_RECHARGE
            stopRechargeMarkWorker();
#endif
            stop_completed=true;
        };
        try {
        r30::product::commandTransaction([&] {
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        if(plan.station) {
            if(recharge_api_->resetSession) recharge_api_->resetSession();
            const int start_rc=recharge_api_->startWorker();
            if(start_rc!=0) {
                r28::Status status;status.code=2004;status.native_code=start_rc;
                status.detail="recharge worker start failed";
                throw r28::Failure(status);
            }
            mark_runtime_on_.store(true,std::memory_order_release);
            worker_started_here=!worker_was_running;
        }
#endif
        // Keep the original stop order: disable work, request camera stop,
        // settle one second, stop the recharge worker, notify the application.
        percep_map=plan.map;percep_station=plan.station;
        if(req.cmd==1) stop_recharge();
        std_msgs::msg::UInt8 camera_request;camera_request.data=plan.camera;
        try {camera_request_pub.publish(camera_request);}
        catch(const std::exception& error) {
            throw r28::Failure(r28::statusFromException(error,2003,"product.control.command"));
        }
        if(req.cmd==21) {sleep(1);stop_recharge();}
        callAppService(plan.app,1,10000);
        },[&] {
#if PERCEPTION_PRODUCT_WITH_RECHARGE
            if(plan.station) {
                percep_map=previous_map;percep_station=previous_station;
                if(worker_started_here) stopRechargeMarkWorker();
            } else {
#endif
                // Camera/stop side effects cannot be reliably undone. Keep the
                // requested local stop and never restart the previous worker.
                percep_map=plan.map;percep_station=false;
                if(!stop_completed) stop_recharge();
#if PERCEPTION_PRODUCT_WITH_RECHARGE
            }
#endif
        });
        } catch(const std::exception& error) {
            if(!plan.station&&stop_completed) {
                auto status=r28::statusFromException(error,2003,"product.control.command");
                status.detail+="; local mode/worker stop already executed; subsequent camera/application notification failed";
                throw r28::Failure(status);
            }
            throw;
        }
    }}});
    res.result=static_cast<decltype(res.result)>(code);
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

    log_on = false;
    if (log_count / 200 ==1) {log_count = 0;log_on = true;}
    log_count++;

///////////////////////获取相机数据////////////////////////////
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    // cmd=2 只发二维码：YOLO 与结算必须 1:1。worker busy / min_interval 挡提交时
    // 不再取图、不再跑检测，否则日志里会连续多帧 YOLO 没有 mark。
    const bool mark_only_task =
        percep_station && !percep_map &&
        mark_runtime_on_.load(std::memory_order_acquire);
    if (mark_only_task) {
        if (camera_status != 1) {
            if (log_on) {
                cout << "相机未打开，感知待机中！" << endl;
                LOG_INFO("[percepDemo] 相机未打开，感知待机中！");
            }
            percep_status = 0;
            return;
        }
        if (!recharge_api_ || !recharge_api_->canSubmit ||
            !recharge_api_->canSubmit()) {
            static uint64_t skip_mark_yolo = 0;
            if ((++skip_mark_yolo % 30) == 1) {
                LOG_INFO("[PercepDemo] recharge_mark skip yolo "
                         "(busy or min_interval)");
            }
            return;
        }
    }
#endif

    CameraData latest_rgb{};
    CameraGdcData latest_gdc{};
    bool rgb_valid = false;
    bool gdc_valid = false;
    const bool need_gdc=percep_station||(save_log&&save_data);

    // -------- RGB 数据读取 --------
    // cmd=2 只发二维码时常无 640 RGB。阻塞等 1s 会把 10Hz onTimer 打成 ~1Hz，
    // 识别只要 70ms 日志却隔 1s+（0831/8）。地图任务仍阻塞等 RGB。
    {
        std::unique_lock<std::mutex> lock(rgb_mtx_);
        const bool wait_rgb = percep_map;
        bool got_rgb = !rgb_buffer_queue.empty();
        if (!got_rgb && wait_rgb) {
            got_rgb = rgb_data_cond_.wait_for(
                lock, std::chrono::milliseconds(1000), [this]() {
                    return shutting_down_.load(std::memory_order_acquire) ||
                           !rgb_buffer_queue.empty();
                });
        }
        if (shutting_down_.load(std::memory_order_acquire)) {
            std::queue<CameraData>().swap(rgb_buffer_queue);
            rgb_valid = false;
            return;
        }
        if (got_rgb && !rgb_buffer_queue.empty()) {
            latest_rgb = rgb_buffer_queue.back();
            std::queue<CameraData>().swap(rgb_buffer_queue);
            rgb_valid = true;
        } else {
            if (wait_rgb) {
                cerr << "超时,感知未收到rgb相机数据!" << endl;
                std::queue<CameraData>().swap(rgb_buffer_queue);
            }
            rgb_valid = false;
        }
    }


    // -------- GDC 数据读取，超时不return，仅置gdc_valid=false --------
    {
        std::unique_lock<std::mutex> lock(gdc_mtx_);
        if(!need_gdc) {
            std::queue<CameraGdcData>().swap(gdc_buffer_queue);
        } else if (!gdc_data_cond_.wait_for(lock, std::chrono::milliseconds(1000), [this]() {
            return shutting_down_.load(std::memory_order_acquire) || !gdc_buffer_queue.empty();
        })){
            if (log_on) {
                LOG_INFO("[percepDemo] 超时,感知未收到gdc相机数据");
            }
            std::queue<CameraGdcData>().swap(gdc_buffer_queue);
            gdc_valid = false;
        } else {
            if (shutting_down_.load(std::memory_order_acquire))
            {
                std::queue<CameraGdcData>().swap(gdc_buffer_queue);
                gdc_valid = false;
                return;
            }
            latest_gdc = gdc_buffer_queue.back();
            std::queue<CameraGdcData>().swap(gdc_buffer_queue);
            gdc_valid = true;
        }
        if (shutting_down_.load(std::memory_order_acquire))
        {
            std::queue<CameraGdcData>().swap(gdc_buffer_queue);
            gdc_valid = false;
            return;
        }
    }
    
    // 地图点云需要 CameraParam；回充 dataset 内参不依赖该话题（晚订阅会错过）
    const bool need_cam_params =
        percep_map
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        || (std::strcmp(recharge_host_.intrinsics_source, "sdk_scaled") == 0)
#endif
        ;
    if (need_cam_params && !receive_prams) {
        if (log_on) {
            cout << "未收到相机参数！！" << endl;
            LOG_INFO("[percepDemo] 未收到相机参数，等待 CameraParam");
        }
        return;
    }
    if(camera_status!=1) {
        if(log_on){cout<<"相机未打开，感知待机中！"<<endl;
        LOG_INFO("[percepDemo] 相机未打开，感知待机中！");}
        percep_status=0;return;}
    if(!percep_station && !percep_map) {
        if(log_on){cout<<"相机已打开，感知待机中！"<<endl;
        LOG_INFO("[percepDemo]相机已打开，感知待机中！");}
        percep_status = 0;return;}    
    if(percep_map && percep_station) {if(log_on){LOG_INFO("感知同时发布地图与二维码！");}percep_status = 3;}
    if(percep_map && !percep_station) {if(log_on){LOG_INFO("感知只发布地图！");}percep_status = 1;}
    if(percep_station && !percep_map) {if(log_on){LOG_INFO("感知只发布二维码！");}percep_status = 2;}  

    gdc_forsave.reset();
    if (save_log && save_data && gdc_valid) {gdc_forsave = std::make_unique<CameraGdcData>(); *gdc_forsave = latest_gdc;}
    r28::product::ProductDag::Bindings bindings;
#if defined(PERCEP_DNN_ENABLE)
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    const bool yolo_for_mark = percep_station && recharge_host_.yolo_assist_crop &&
                               ai_recharge_.enabled && dnn_cfg_.enabled && dnn_cfg_.detect.enabled &&
                               dnn_runner_;
#else
    const bool yolo_for_mark = false;
#endif
    const bool want_dnn = dnn_cfg_.enabled && dnn_runner_ &&
                          (percep_map || yolo_for_mark);
    if (want_dnn) bindings["dnn"] = [&] {
        auto start_1 = std::chrono::high_resolution_clock::now();
        bool ran = false;
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        const bool gdc_left_ok = gdc_valid && latest_gdc.left.bytes_used > 0 &&
                                 latest_gdc.left_data;
#endif
        const bool rgb_left_ok = rgb_valid && latest_rgb.left.bytes_used > 0 &&
                                 latest_rgb.left_data;
        // cmd=2 不要分割：回充只用 charger 框。cmd=3 地图仍跑 det+seg。
        const bool run_seg = percep_map;
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        if (yolo_for_mark && !percep_map) {
            if(recharge_host_.use_hires&&gdc_left_ok) {
                this->runDnnInference(latest_gdc.left,latest_gdc.left_data,run_seg);ran=true;
            } else if(!recharge_host_.use_hires&&rgb_left_ok) {
                this->runDnnInference(latest_rgb.left,latest_rgb.left_data,run_seg);ran=true;
            }
        } else
#endif
        if (rgb_left_ok) {
            this->runDnnInference(latest_rgb.left, latest_rgb.left_data,
                                  run_seg);
            ran = true;
        }
#if PERCEPTION_PRODUCT_WITH_RECHARGE
        else if (yolo_for_mark && gdc_left_ok) {
            this->runDnnInference(latest_gdc.left, latest_gdc.left_data,
                                  run_seg);
            ran = true;
        }
#endif
        auto end_1 = std::chrono::high_resolution_clock::now();
        auto duration1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_1 - start_1);
        if (ran && log_on) {
            cout << "1.runDnnInference Time taken: " << duration1.count()
                 << "ms" << endl;
        } else if (yolo_for_mark && !ran) {
            static uint64_t no_img_log = 0;
            if ((++no_img_log % 20) == 1) {
                LOG_WARN("[PercepDemo] yolo_assist skip: no gdc left for DNN");
            }
        }
        if (ran && !run_seg) {
            static bool seg_off_logged = false;
            if (!seg_off_logged) {
                seg_off_logged = true;
                LOG_INFO("[PercepDemo] recharge_mark dnn=det-only (seg off)");
            }
        }
    };
#endif
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    if (percep_station &&
        mark_runtime_on_.load(std::memory_order_acquire)) bindings["recharge"] = [&] {
        if(!recharge_api_||!recharge_api_->canSubmit||!recharge_api_->submitFromShm) {
            r28::Status status;status.code=2004;status.detail="recharge submission API unavailable";
            throw r28::Failure(status);
        }
        // One producer; do not replace an accepted job's callback context when
        // the SDK still owns it or the normal min_interval has not elapsed.
        if(!recharge_api_->canSubmit()) return;
        const PicData* selected=recharge_host_.use_hires
            ? (gdc_valid?&latest_gdc.left:nullptr) : (rgb_valid?&latest_rgb.left:nullptr);
        if(!selected||selected->width<=0||selected->height<=0||
           selected->width%2||selected->height%2||
           selected->bytes_used<static_cast<std::size_t>(selected->width)*selected->height*3/2) {
            r28::Status status;status.code=1001;
            status.detail="recharge selected NV12 source is missing or incomplete";throw r28::Failure(status);
        }
        applyYoloAssistCropHint();
        const uint64_t done_before =
            mark_done_seq_.load(std::memory_order_acquire);
        {
            std::lock_guard<std::mutex> lock(mark_done_mtx_);
            mark_pending_status_={};
            mark_submitted_frame_.id=selected->frame_id;
            mark_submitted_frame_.timestamp_ns=selected->time_ns;
            mark_submitted_run_id_=product_dag_->currentRunId();
        }
        const int sub_rc = submitRechargeMarkFromShm(
            rgb_valid ? &latest_rgb : nullptr, rgb_valid,
            gdc_valid ? &latest_gdc : nullptr, gdc_valid);
        if (sub_rc != 0 && sub_rc != 1) {
            r28::Status status; status.code=2004; status.native_code=sub_rc;
            status.detail="recharge submitFromShm failed"; throw r28::Failure(status);
        }
        // cmd=2：YOLO → 结算 → 发布，再进下一帧。cmd=3 还要跑点云，不阻塞。
        if (mark_only_task && sub_rc == 0) {
            waitRechargeMarkSettled(done_before, 2000);
        }
        if (log_on) {
            const int busy = (recharge_api_ && recharge_api_->canSubmit &&
                              !recharge_api_->canSubmit())
                                 ? 1
                                 : 0;
            LOG_INFO("[PercepDemo] recharge_mark tick src=%s gdc=%d rgb=%d "
                     "busy=%d sub=%d",
                     recharge_host_.use_hires ? "gdc1280" : "left640",
                     gdc_valid ? 1 : 0, rgb_valid ? 1 : 0, busy, sub_rc);
        }
    };
#endif

    PointCloudT::Ptr pcl_cloud(new PointCloudT);
    if (rgb_valid && percep_map) {
        bindings["rgb"] = [&] {
            if (latest_rgb.left.bytes_used==0 || latest_rgb.right.bytes_used==0 || latest_rgb.disp.bytes_used==0)
                throw std::runtime_error("map requires complete RGBD SHM frame");
            frame_save_time_str_ = std::to_string(daye::getSystemTimeMs());
            frame_rgb_buf_.clear();
            if (visualize) {
                sensor_msgs::msg::Image message;
                if (!NV12ToRgbRos(latest_rgb, message, frame_rgb_buf_))
                    throw std::runtime_error("NV12 to RGB conversion failed");
                message.header.frame_id="percep_demo"; rgb_img_pub_.publish(message);
            }
        };
        bindings["cloud"] = [&] {
            if (!dense2sparse_pointcloud(latest_rgb,sample_step,pcl_cloud,pcl_pub_cfg_))
                throw std::runtime_error("disparity to point cloud failed");
            ori_cloud = pcl_cloud;
        };
        if (auto_cali) {
            bindings["calibration"] = [&] {autocali(pcl_cloud); percep_status=0;};
        } else {
#if defined(PERCEP_DNN_ENABLE)
            bindings["labels"] = [&] {
                matchAndPublishDetCloud(pcl_cloud, camera_params.fx,camera_params.fy,camera_params.cx,camera_params.cy,rgb_width,rgb_height);
            };
#endif
            bindings["transform"] = [&] {
                postprocess(pcl_cloud,frame_rgb_buf_,rgb_width,rgb_height,camera_params.fx,camera_params.fy,camera_params.cx,camera_params.cy);
            };
            if (patchwork_cfg_.enabled) bindings["ground"] = [&] {runGroundSegmentation(pcl_cloud);};
            bindings["maps"] = [&] {buildProductMaps();};
            bindings["publish"] = [&] {publishProductMaps(frame_count_);};
            if (save_log) bindings["record"] = [&] {recordProductMaps();};
        }
    }
    perception::FrameInfo frame;
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    const bool frame_from_gdc=percep_station&&!percep_map&&recharge_host_.use_hires;
    const auto& source=frame_from_gdc?latest_gdc.left:latest_rgb.left;
#else
    const auto& source=latest_rgb.left;
#endif
    frame.id=source.frame_id;frame.timestamp_ns=source.time_ns;
    if(bindings.empty()) {
        r28::Status status;status.code=1001;status.frame=frame;status.feature="product.input.shm";
        status.detail="requested product work has no valid camera frame";r28::product::setLastStatus(status);return;
    }
    runProductBindings(bindings, frame);
    if(percep_map&&!rgb_valid) {
        r28::Status status;status.code=1001;status.frame=frame;status.feature="product.input.shm";
        status.detail="map RGBD acquisition timed out";r28::product::setLastStatus(status);
        product_dag_->recordExternalStatus(status,product_dag_->currentRunId());percep_status=-1;
    }
}

std::uint32_t PercepDemo::runProductBindings(const r28::product::ProductDag::Bindings& bindings, perception::FrameInfo frame) {
    product_frame_=frame;
    perception::runtime::GraphReport report; r28::Status status;
    auto code=product_dag_->run(bindings, frame, report, status);
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    {
        std::lock_guard<std::mutex> lock(mark_done_mtx_);
        if(!code&&mark_pending_status_.code&&mark_submitted_run_id_==product_dag_->currentRunId()) {
            status=mark_pending_status_;code=status.code;
        }
        r28::product::setLastStatus(status);
    }
#else
    r28::product::setLastStatus(status);
#endif
    if (code) {percep_status=-1; LOG_ERROR("[product.dag] %s",r28::statusJson(status).c_str()); return code;}
    if (bindings.count("publish")) ++frame_count_;
    return 0;
}

std::uint32_t PercepDemo::runProductEvent(const std::string& event,
    const r30::product::ProductEvents::Bindings& bindings,perception::FrameInfo frame,
    std::uint64_t parent_run_id,r28::Status* completion_status) {
    perception::runtime::GraphReport report;r28::Status status;
    if(!product_events_) {
        status.code=2001;status.frame=frame;status.feature="product.events";
        status.detail="product event graph is not initialized";
        if(completion_status) *completion_status=status;
        r28::product::setLastStatus(status);return status.code;
    }
    const auto code=product_events_->run(event,bindings,frame,parent_run_id,report,status);
    if(completion_status) *completion_status=status;
    if(code) {
        r28::product::setLastStatus(status);
        LOG_ERROR("[product.event] %s",r28::statusJson(status).c_str());
    }
    return code;
}

bool PercepDemo::dense2sparse_pointcloud(CameraData& camera_data, int sample_step, PointCloudT::Ptr &out_sparse_cloud,const PointCloudPubConfig& box)
{
 if (sample_step < 1)
    {
        std::cerr << "[PclCommon] disp frame null or invalid sample step\n";
        return false;
    }
    const int w = camera_data.disp.width;
    const int h = camera_data.disp.height;
    if(w!=rgb_width || h!=rgb_height || camera_data.disp.bytes_used<std::size_t(w)*h*sizeof(uint16_t))
        throw std::runtime_error("original product disparity requires complete 640x544 Q8.8 buffer");

    size_t reserve_num = (w / sample_step) * (h / sample_step);
    out_sparse_cloud->clear();
    out_sparse_cloud->reserve(reserve_num); // 预分配内存

    const uint16_t* disp_16 = reinterpret_cast<const uint16_t*>(camera_data.disp_data);

    disp_vec.clear();
    disp_vec.reserve(reserve_num);
    u_vec.clear();
    u_vec.reserve(reserve_num);
    v_vec.clear();
    v_vec.reserve(reserve_num);
    float fx = camera_params.fx;
    float fy = camera_params.fy;
    float cx = camera_params.cx;
    float cy = camera_params.cy;
    double bxf = camera_params.bxf /1000.0f;
    double baseline = camera_params.base_line ;

    // cout<<"fx:"<<fx<<" fy:"<<fy<<" cx:"<<cx<<" cy:"<<cy<<" bxf:"<<bxf<<" baseline:"<<baseline<<endl;
    
    for (int v = 0; v < h; v += sample_step)
    {
        for (int u = 0; u < w; u += sample_step)
        {
            size_t pix_idx = static_cast<size_t>(v) * w + u;
            uint16_t disp_raw_val = disp_16[pix_idx];
            
            r28::product::ProjectedDisparity projected;
            if(!r28::product::projectOriginalDisparity(disp_raw_val,u,v,fx,fy,cx,cy,bxf,projected)) continue;
            const float X_m=projected.x,Y_m=projected.y,Z_m=projected.z;
            if (Z_m < box.box_min_z || Z_m > box.box_max_z 
                || X_m < box.box_min_x || X_m > box.box_max_x 
                || Y_m < box.box_min_y || Y_m > box.box_max_y
                )
            {
                continue;
            }
            out_sparse_cloud->emplace_back(X_m, Y_m, Z_m, 0, 0, 0, -1, 0.f);
            if(save_log&&save_data) {
                disp_vec.push_back(projected.disparity);
                u_vec.push_back(static_cast<uint32_t>(u));
                v_vec.push_back(static_cast<uint32_t>(v));
            }
        }
    }
    if(visualize) {
    const float max_disp=144.0f,min_disp=1.0f;
    std::vector<float> disp_float(rgb_width*rgb_height);
    for(int i=0;i<rgb_width*rgb_height;++i) {
        const float d=static_cast<float>(disp_16[i])/256.0f;
        disp_float[i]=std::clamp(d,min_disp,max_disp);
    }
    // 归一化映射到0~255 uint8灰度
    cv::Mat disp_mat(rgb_height, rgb_width, CV_32FC1, disp_float.data());
    cv::Mat disp_gray;
    disp_mat = (disp_mat - min_disp) / (max_disp - min_disp) * 255.0f;
    disp_mat.convertTo(disp_gray, CV_8UC1);

    sensor_msgs::msg::Image disp_img_msg;
    // 封装ROS2 mono8图像消息
    disp_img_msg.header.frame_id = "percep_demo";
    disp_img_msg.header.stamp = rclcpp::Clock().now();
    disp_img_msg.width = rgb_width;
    disp_img_msg.height = rgb_height;
    disp_img_msg.encoding = "mono8";
    disp_img_msg.step = rgb_width;
    disp_img_msg.data.assign(disp_gray.data, disp_gray.data + disp_gray.total());
    disp_img_msg.header.frame_id = "percep_demo";
    disp_img_pub_.publish(disp_img_msg);   
    }

    if (out_sparse_cloud->empty())
    {
        std::cerr << "[PclCommon] sparse cloud generate empty\n";
        return false;
    }
    return true;
}
bool PercepDemo::initInterface()
{

    fs::create_directories(output_dir);
    pclCommonPtr.reset(new PclCommon);
    ground_cloud.reset(new PointCloudT);
    no_ground_cloud.reset(new PointCloudT);
    obs_cloud.reset(new PointCloudT);
    last_semantic_cloud_.reset(new PointCloudT);
    aiFusionPtr.reset(new AiFusion(mapWidth,mapHeight,mapResolution,0.05,5,6,cali.x,cali.y,pcl_pub_cfg_.box_min_z));
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(daye::getHandle());
    visualizePtr.reset(new Visualize(mapWidth,mapHeight,mapResolution,output_dir,cali.x,cali.y));
    gdc_forsave = std::make_unique<CameraGdcData>();
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

#if PERCEPTION_PRODUCT_WITH_RECHARGE
  if (!recharge_mark_config_path_.empty()) {
      if (!recharge_api_) {
          recharge_api_ = Aiper_Recharge::createRechargeMarkApi();
      }
      if(!recharge_api_ || !recharge_api_->init) {
          r28::Status status;status.code=2004;status.feature="product.recharge.init";
          status.detail="matched recharge SDK factory/init unavailable";throw r28::Failure(status);
      }
      if (recharge_api_ && recharge_api_->init) {
          const int rc = recharge_api_->init(
              sdk_so_dir.c_str(), recharge_mark_config_path_.c_str());
          if (rc != 0) {
              cerr << "[PercepDemo] recharge_mark loadConfig failed path="
                   << recharge_mark_config_path_ << endl;
              LOG_ERROR("[PercepDemo] recharge_mark loadConfig failed path=%s",
                        recharge_mark_config_path_.c_str());
              r28::Status status;status.code=2004;status.native_code=rc;
              status.feature="product.recharge.init";status.detail="recharge configuration initialization failed";
              throw r28::Failure(status);
          }
      }
      if (recharge_api_ && recharge_api_->getHostConfig) {
          recharge_api_->getHostConfig(&recharge_host_);
      }
      loadAiRechargeConfig(recharge_mark_config_path_);
      if (!ai_recharge_.enabled && recharge_host_.yolo_assist_crop) {
          ai_recharge_.enabled = true;
      }
      if (recharge_api_ && recharge_api_->setResultCallback) {
          recharge_api_->setResultCallback(&PercepDemo::OnRechargeMarkResult,
                                           this);
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
               << " charger=" << ai_recharge_.charger_class
               << endl;
          LOG_INFO("[PercepDemo] recharge_mark ready (wait ToPercepCmd cmd=2) "
                   "topic=%s cfg=%s min_interval_ms=%u use_hires=%s yolo=%s",
                   kRechargeMarkPoseTopic, recharge_mark_config_path_.c_str(),
                   recharge_host_.min_interval_ms,
                   recharge_host_.use_hires ? "on" : "off",
                   recharge_host_.yolo_assist_crop ? "on" : "off");
      } else {
          cout << "[PercepDemo] recharge_mark disabled in "
               << recharge_mark_config_path_ << endl;
      }
  }
#else
  LOG_INFO("[PercepDemo] recharge_mark excluded at build time; ToPercepCmd cmd=2/3 unsupported");
#endif
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
//-----------------sdk interface-------------------//      
    if (!offline) 
    {
        
        auto qos_cam = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        cam_params_sub_ = daye::CreateSubscribe<common_interface::msg::CameraParam>(kCameraParam, qos_cam,bind(&PercepDemo::onCamParams, this, placeholders::_1));
        const daye::ChannelConfig config{kCameraFrameShm, kFrameShmSize};
        subCamera = daye::ShmSubscriber(config);
        subCamera.registerCallback(std::bind(&PercepDemo::onCameraFrame, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3),100);
        const daye::ChannelConfig gdc_config{kCameraGdcFrameShm, kGdcFrameShmSize};
        subCameraGdc = daye::ShmSubscriber(gdc_config);
        subCameraGdc.registerCallback(std::bind(&PercepDemo::onGdcFrame, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3),100);

        camera_request_pub = daye::CreatePublish<std_msgs::msg::UInt8>("/camera/get_data", qos_cam);

        timer_ = daye::CreateTimer("percep_demo_timer", 10.0, &PercepDemo::onTimer, this, true);
        timer_started_=true;

        
    }

  if(save_log) saveLogThreadHandle=std::thread(&PercepDemo::saveLogLoop,this);
  return true;
}

#if PERCEPTION_PRODUCT_WITH_RECHARGE
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
#endif

bool PercepDemo::initParams()

{
    const char* generated=std::getenv("PERCEPTION_PRODUCT_CONFIG");
    const std::string product_config=generated&&*generated?generated:"/app/daye/config/ai_node/start_config.json";
    { std::ifstream file(product_config); if(!file) return false;
      std::string raw((std::istreambuf_iterator<char>(file)),{}); rapidjson::Document config;
      config.Parse(raw.c_str()); if(config.HasParseError()||!config.IsObject()) return false;
      if(config.HasMember("extrinsics_config")&&config["extrinsics_config"].IsString())
          product_extrinsics_path_=resolveConfigPath(product_config,config["extrinsics_config"].GetString());
      if(config.HasMember("product")&&config["product"].IsObject()) {
          const auto& p=config["product"];
          if(p.HasMember("dag_config")&&p["dag_config"].IsString()) product_dag_path_=resolveConfigPath(product_config,p["dag_config"].GetString());
          if(p.HasMember("events_config")&&p["events_config"].IsString()) product_events_path_=resolveConfigPath(product_config,p["events_config"].GetString());
          if(p.HasMember("exact_profiles")&&p["exact_profiles"].IsString()) product_exact_profiles_=resolveConfigPath(product_config,p["exact_profiles"].GetString());
          if(p.HasMember("metrics_dir")&&p["metrics_dir"].IsString()&&p["metrics_dir"].GetStringLength()) product_metrics_dir_=resolveConfigPath(product_config,p["metrics_dir"].GetString());
      }
    }

    // Offline images and disparity keep the original directories by default.
    // Configured paths are resolved against the selected start_config.json.
    { std::ifstream file(product_config);std::string raw((std::istreambuf_iterator<char>(file)),{});
      rapidjson::Document config;config.Parse(raw.c_str());
      if(!file||config.HasParseError()||!config.IsObject())
          throw std::invalid_argument("start_config changed or became unreadable while loading offline_input");
      if(config.HasMember("offline_input")) {
          const auto& input=config["offline_input"];
          if(!input.IsObject()) throw std::invalid_argument("offline_input must be an object");
          const auto read_path=[&](const char* key,std::string& destination) {
              if(!input.HasMember(key)) return;
              if(!input[key].IsString()||input[key].GetStringLength()==0)
                  throw std::invalid_argument(std::string("offline_input.")+key+" must be a nonempty path");
              destination=resolveConfigPath(product_config,input[key].GetString());
          };
          read_path("rgb_dir",offline_rgb_dir_);read_path("disparity_dir",offline_disparity_dir_);
      }
    }

    string extrinsics_config_dir = product_extrinsics_path_;
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

    string start_config_dir = product_config;
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
#if PERCEPTION_PRODUCT_WITH_RECHARGE
    if (!resolveRechargeMarkConfigPath(start_config_dir, ai_node_json)) {
        cerr << "[PercepDemo err] 解析回充配置路径失败" << endl;
        return false;
    }
#endif

#if defined(PERCEP_DNN_ENABLE)
    if (!this->parseDnnConfig(ai_node_json)) {
        cerr << "[PercepDemo] parseDnnConfig failed" << endl;
        return false;
    }
#if defined(PERCEP_DNN_ENABLE)
    this->parseLabelFusionConfig(ai_node_json);
#endif
    cout << "[PercepDemo] dnn cfg enabled=" << (dnn_cfg_.enabled ? "on" : "off")
         << " detect=" << (dnn_cfg_.detect.enabled ? "on" : "off")
         << " model=" << dnn_cfg_.detect.model_path << endl;
#endif
    if(offline&&save_log&&save_data) {
        r28::Status status;status.code=2001;status.feature="product.config.offline";
        status.detail="offline JPG/disparity replay supports save_map; raw GDC save_data requires online SHM";
        throw r28::Failure(status);
    }
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

#if PERCEPTION_PRODUCT_WITH_RECHARGE
void PercepDemo::OnRechargeMarkResult(void* userdata,
                                      Aiper_Recharge::MarkPoseResult* result) {
    auto* self = static_cast<PercepDemo*>(userdata);
    if (!self) {
        return;
    }
    bool notified=false;
    const auto notify=[&] {
        if(notified) return;
        notified=true;
        self->mark_done_seq_.fetch_add(1,std::memory_order_release);
        self->mark_done_cv_.notify_all();
    };
    r28::Status completion;
    try {
        std::uint64_t run_id=0;
        {std::lock_guard<std::mutex> lock(self->mark_done_mtx_);
         completion.frame=self->mark_submitted_frame_;run_id=self->mark_submitted_run_id_;}
        r28::Status event_status;
        const auto code=self->runProductEvent("recharge_result",{{"recharge_publish",[&] {
            try {
                if(!result) {
                    completion.code=2004;completion.detail="recharge SDK returned a null result";
                    throw r28::Failure(completion);
                }
                self->publishRechargeMark(result);
            } catch(const std::exception& error) {
                completion=r28::statusFromException(error,2003,"product.recharge.publish","recharge_publish",completion.frame);
            } catch(...) {
                completion.code=2003;completion.feature="product.recharge.publish";
                completion.task="recharge_publish";completion.detail="unknown recharge callback failure";
            }
            if(completion.code) {
                {std::lock_guard<std::mutex> lock(self->mark_done_mtx_);
                 self->mark_pending_status_=completion;}
                notify();
                throw r28::Failure(completion);
            }
            // Signal before ProductEvents writes its metrics. The waiting map
            // graph and this callback never acquire each other's graph mutex.
            notify();
        }}},completion.frame,run_id,&event_status);
        // A closed/missing event graph can fail before entering the binding.
        // Preserve its full status before waking a mark-only frame waiter.
        if(code) {
            std::lock_guard<std::mutex> lock(self->mark_done_mtx_);
            self->mark_pending_status_=event_status;
        }
    } catch(const std::exception& error) {
        try {
            completion=r28::statusFromException(error,2004,"product.recharge.callback","recharge_publish",completion.frame);
            {std::lock_guard<std::mutex> lock(self->mark_done_mtx_);self->mark_pending_status_=completion;}
            r28::product::setLastStatus(completion);
        } catch(...) {}
    } catch(...) {
        try {
            completion.code=2004;completion.feature="product.recharge.callback";
            completion.detail="unknown recharge callback event failure";
            {std::lock_guard<std::mutex> lock(self->mark_done_mtx_);self->mark_pending_status_=completion;}
            r28::product::setLastStatus(completion);
        } catch(...) {}
    } // No exception may cross the original SDK callback ABI.
    notify();
}

void PercepDemo::waitRechargeMarkSettled(uint64_t done_before, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mark_done_mtx_);
    const bool ok = mark_done_cv_.wait_for(
        lk, std::chrono::milliseconds(timeout_ms), [&] {
            return shutting_down_.load(std::memory_order_acquire) ||
                   mark_done_seq_.load(std::memory_order_acquire) > done_before;
        });
    if (!ok) {
        r28::Status status;status.code=2004;
        status.detail="recharge callback timed out after "+std::to_string(timeout_ms)+"ms";
        throw r28::Failure(status);
    }
    if(mark_pending_status_.code) throw r28::Failure(mark_pending_status_);
}

void PercepDemo::loadAiRechargeConfig(const std::string& json_path) {
    ai_recharge_ = AiRechargeConfig{};
    if (json_path.empty()) return;
    std::ifstream ifs(json_path, std::ios::binary);
    if (!ifs.is_open()) return;
    std::stringstream ss;
    ss << ifs.rdbuf();
    rapidjson::Document doc;
    if (doc.Parse(ss.str().c_str()).HasParseError() || !doc.IsObject()) return;
    const rapidjson::Value* root = &doc;
    if (doc.HasMember("recharge_mark") && doc["recharge_mark"].IsObject()) {
        root = &doc["recharge_mark"];
    }
    if (!root->HasMember("ai_recharge") || !(*root)["ai_recharge"].IsObject()) {
        return;
    }
    const rapidjson::Value& ar = (*root)["ai_recharge"];
    ai_recharge_.enabled = readJsonBool(ar, "enabled", ai_recharge_.enabled);
    if (ar.HasMember("charger_class") && ar["charger_class"].IsString()) {
        ai_recharge_.charger_class = ar["charger_class"].GetString();
    }
    if (ar.HasMember("score_min") && ar["score_min"].IsNumber()) {
        ai_recharge_.score_min = static_cast<float>(ar["score_min"].GetDouble());
    }
    if (ar.HasMember("pad_ratio") && ar["pad_ratio"].IsNumber()) {
        ai_recharge_.pad_ratio = static_cast<float>(ar["pad_ratio"].GetDouble());
    }
    if (ar.HasMember("pad_px") && ar["pad_px"].IsInt()) {
        ai_recharge_.pad_px = ar["pad_px"].GetInt();
    }
    if (ar.HasMember("min_box_w") && ar["min_box_w"].IsInt()) {
        ai_recharge_.min_box_w = ar["min_box_w"].GetInt();
    }
    if (ar.HasMember("min_box_h") && ar["min_box_h"].IsInt()) {
        ai_recharge_.min_box_h = ar["min_box_h"].GetInt();
    }
    ai_recharge_.require_charger =
        readJsonBool(ar, "require_charger", ai_recharge_.require_charger);
    ai_recharge_.fallback_static_roi =
        readJsonBool(ar, "fallback_static_roi", ai_recharge_.fallback_static_roi);
}

void PercepDemo::applyYoloAssistCropHint() {
    if (!recharge_api_ || !recharge_api_->setExternalCropHint) return;
    const bool want = recharge_host_.yolo_assist_crop && ai_recharge_.enabled;
    if (!want) {
        recharge_api_->setExternalCropHint(0, 0, 0, 0, 1);
        return;
    }
#if !defined(PERCEP_DNN_ENABLE)
    recharge_api_->setExternalCropHint(0, 0, 0, 0, 1);
    return;
#else
    const int mark_w = recharge_host_.use_hires ? gdc_width : rgb_width;
    const int mark_h = recharge_host_.use_hires ? gdc_height : rgb_height;
    const int det_w = last_dnn_img_w_;
    const int det_h = last_dnn_img_h_;
    const DnnDetection* best = nullptr;
    if (det_w > 0 && det_h > 0) {
        for (const auto& d : last_dnn_dets_) {
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
        ok = BuildChargerCropOnMarkImage(
            best->x1, best->y1, best->x2, best->y2, det_w, det_h, mark_w,
            mark_h, ai_recharge_.pad_ratio, ai_recharge_.pad_px,
            ai_recharge_.min_box_w, ai_recharge_.min_box_h, &ox, &oy, &ow,
            &oh);
    }
    if (!ok) {
        // 未检出不主动清空：worker 可能还没消费上一帧 hint；本帧会回退 follow/静态 ROI
        static uint64_t miss_log = 0;
        if ((++miss_log % 20) == 1) {
            LOG_INFO("[PercepDemo] yolo_assist no charger box det=%dx%d dets=%zu "
                     "fallback=static_roi",
                     det_w, det_h, last_dnn_dets_.size());
        }
        return;
    }
    recharge_api_->setExternalCropHint(ox, oy, ow, oh, 1);
    static uint64_t hit_log = 0;
    if ((++hit_log % 20) == 1) {
        LOG_INFO("[PercepDemo] yolo_assist crop=[%d,%d,%d,%d] score=%.2f "
                 "det=%dx%d mark=%dx%d",
                 ox, oy, ow, oh, best->confidence, det_w, det_h, mark_w,
                 mark_h);
        cout << "[PercepDemo] yolo_assist crop=[" << ox << "," << oy << ","
             << ow << "," << oh << "] score=" << best->confidence << endl;
    }
#endif
}

bool PercepDemo::startRechargeMarkWorker() {
    if (!recharge_host_.enabled || !recharge_host_.publish_enabled ||
        !recharge_api_ || !recharge_api_->startWorker) {
        r28::Status status;status.code=2004;status.feature="product.recharge.start";
        status.detail="recharge worker prerequisites unavailable";r28::product::setLastStatus(status);return false;
    }
    const int rc=recharge_api_->startWorker();
    if(rc!=0) {
        mark_runtime_on_.store(false,std::memory_order_release);
        r28::Status status;status.code=2004;status.native_code=rc;status.feature="product.recharge.start";
        status.detail="recharge worker start failed";r28::product::setLastStatus(status);return false;
    }
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

int PercepDemo::submitRechargeMarkFromShm(const CameraData* rgbd, bool rgb_valid,
                                          const CameraGdcData* gdc,
                                          bool gdc_valid) {
    if (!mark_runtime_on_.load(std::memory_order_acquire) ||
        !recharge_host_.enabled || !recharge_host_.publish_enabled ||
        !recharge_api_ || !recharge_api_->submitFromShm) {
        return 1;
    }
    Aiper_Recharge::RechargeCameraK k{};
    const Aiper_Recharge::RechargeCameraK* pk = nullptr;
    if (std::strcmp(recharge_host_.intrinsics_source, "sdk_scaled") == 0 &&
        camera_params.fx > 1.f && camera_params.fy > 1.f) {
        k.cal_width = camera_params.width;
        k.cal_height = camera_params.height;
        k.fx = camera_params.fx;
        k.fy = camera_params.fy;
        k.cx = camera_params.cx;
        k.cy = camera_params.cy;
        pk = &k;
    }
    return recharge_api_->submitFromShm(
        rgb_valid ? rgbd : nullptr, rgb_valid ? sizeof(CameraData) : 0,
        gdc_valid ? gdc : nullptr, gdc_valid ? sizeof(CameraGdcData) : 0, pk);
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
                          ? ((w >= GDC_WIDTH) ? "gdc1280" : "left640(fallback)")
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

#endif

void PercepDemo::scaleIntrinsics(int img_w, int img_h,
                                 const common_interface::msg::CameraParam& cam,
                                 float& fx, float& fy, float& cx, float& cy) {
    if (cam.width <= 0 || cam.height <= 0) {
        return;
    }
    if (cam.width == img_w && cam.height == img_h) {
        return;
    }
    const float sx = static_cast<float>(img_w) / cam.width;
    const float sy = static_cast<float>(img_h) / cam.height;
    fx *= sx;
    fy *= sy;
    cx *= sx;
    cy *= sy;
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
void PercepDemo::buildProductMaps() {
    product_maps_ = OccMapData{};
    aiFusionPtr->initData(ground_cloud,obs_cloud,last_semantic_cloud_);
    aiFusionPtr->integrate(product_maps_);
}
void PercepDemo::publishProductMaps(int count) {
    const int cols=static_cast<int>(mapWidth/mapResolution);
    const int rows=static_cast<int>(mapHeight/mapResolution);
    const auto& fusion_data=product_maps_;
    string time_str = frame_save_time_str_.empty()
                          ? std::to_string(daye::getSystemTimeMs())
                          : frame_save_time_str_;
    cout<<"current frame timestamp:"<<time_str<<endl;
    common_interface::msg::GridMapInfo grid_msg;
    grid_msg.header.stamp = rclcpp::Clock().now();
    grid_msg.header.frame_id = parent_frame;
    grid_msg.rows = static_cast<uint16_t>(mapHeight / mapResolution );
    grid_msg.cols = static_cast<uint16_t>(mapWidth / mapResolution );
    grid_msg.resolution_mm = static_cast<uint16_t>(mapResolution *1000);
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
 
}
void PercepDemo::recordProductMaps() {
    const int cols=static_cast<int>(mapWidth/mapResolution);
    const int rows=static_cast<int>(mapHeight/mapResolution);
    const auto& fusion_data=product_maps_;
    const string time_str=frame_save_time_str_.empty()?std::to_string(daye::getSystemTimeMs()):frame_save_time_str_;
    if (save_log)
    {
        auto new_task = std::make_unique<SaveMapTask>();
        new_task->frame=product_frame_;
        new_task->run_id=product_dag_->currentRunId();
        new_task->map_data = fusion_data;                 // vector深拷贝
        new_task->rgb = frame_rgb_buf_;   
        new_task->gdc = std::move(gdc_forsave);
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
void PercepDemo::convertGridMap(int count) {
    buildProductMaps(); publishProductMaps(count); recordProductMaps();
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
        runProductEvent("record_io",{{"record_io",[&] {
            if(save_data) {
                if(!task->gdc || !task->gdc->left.bytes_used)
                    throw std::runtime_error("requested GDC recording unavailable for this frame");
                sensor_msgs::msg::Image message;std::vector<uint8_t> rgb;
                if(!GdcToRgbRos(*task->gdc,message,rgb)) throw std::runtime_error("recording GDC conversion failed");
                visualizePtr->saveGDC(rgb,task->time_str);
                if(!visualizePtr->saveDisp(task->u,task->v,task->disp,task->time_str))
                    throw std::runtime_error("disparity recording failed");
                if(!pclCommonPtr->keepNewestFiles(output_dir+"/data/gdc/",30000)||
                   !pclCommonPtr->keepNewestFiles(output_dir+"/data/disp/",30000))
                    throw std::runtime_error("recording file retention failed");
            }
            if(save_map) {
                for(const auto& entry:{std::pair<const char*,std::vector<MapPoint>*>{"pcl",&task->map_data.pcl_map},
                    {"ai",&task->map_data.ai_map},{"fusion",&task->map_data.fusion_map}}) {
                    if(!visualizePtr->saveMap(*entry.second,entry.first,task->time_str,task->rows,task->cols))
                        throw std::runtime_error("map recording failed");
                    if(!pclCommonPtr->keepNewestFiles(output_dir+"/map/"+entry.first+"/",30000))
                        throw std::runtime_error("map file retention failed");
                }
            }
        }}},task->frame,task->run_id);
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
    os.Flush();
    const bool written=std::ferror(fp)==0;
    const int close_rc=fclose(fp);
    if(!written||close_rc!=0) return false;
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
    r28::Status failure;failure.code=2003;failure.feature="product.control.command";
    failure.detail="application command service call failed";
    throw r28::Failure(failure);
  }else{
    cout<<"[PercepDemo] received app result:"<<app_response.result<<endl;
  }
}

bool PercepDemo::GdcToRgbRos(const CameraGdcData& camera_data,
                                       sensor_msgs::msg::Image& ros_img,
                                       std::vector<uint8_t>& rgb_buf)
{

    const int w = camera_data.left.width;
    const int h = camera_data.left.height;

    if(w<=0||h<=0||!camera_data.left.bytes_used) return false;
    auto start1 = std::chrono::high_resolution_clock::now();
    const int stride_y = static_cast<int>(camera_data.left.bytes_used / (h * 1.5f));
    if (!pclCommonPtr->yuv_nv12_to_rgb24(static_cast<const uint8_t*>(camera_data.left_data), w, h, stride_y, &rgb_buf)) {
        cerr<<"[PercepDemo Err] nv12 convert to rgb24 failed"<<endl;
        return false;
    }
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1- start1);
    cout<<"Gdc nv12 to rgb Time taken: "<<duration1.count()<<"ms"<<endl;

    ros_img.header.stamp = rclcpp::Time(camera_data.left.time_ns, RCL_ROS_TIME);
    ros_img.width  = static_cast<uint32_t>(w);
    ros_img.height = static_cast<uint32_t>(h);
    ros_img.encoding = "rgb8";
    ros_img.step = static_cast<uint32_t>(w * 3);
    ros_img.data.assign(rgb_buf.begin(), rgb_buf.end());
    return true;
}

bool PercepDemo::NV12ToRgbRos(const CameraData& camera_data,
                                       sensor_msgs::msg::Image& ros_img,
                                       std::vector<uint8_t>& rgb_buf)
{

    const int w = camera_data.left.width;
    const int h = camera_data.left.height;

    auto start1 = std::chrono::high_resolution_clock::now();
    const int stride_y = static_cast<int>(camera_data.left.bytes_used / (h * 1.5f));
    if (!pclCommonPtr->yuv_nv12_to_rgb24(static_cast<const uint8_t*>(camera_data.left_data), w, h, stride_y, &rgb_buf)) {
        cerr<<"[PercepDemo Err] nv12 convert to rgb24 failed"<<endl;
        return false;
    }
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1- start1);
    cout<<"Rgb nv12 to rgb Time taken: "<<duration1.count()<<"ms"<<endl;

    ros_img.header.stamp = rclcpp::Time(camera_data.left.time_ns, RCL_ROS_TIME);
    ros_img.width  = static_cast<uint32_t>(w);
    ros_img.height = static_cast<uint32_t>(h);
    ros_img.encoding = "rgb8";
    ros_img.step = static_cast<uint32_t>(w * 3);
    ros_img.data.assign(rgb_buf.begin(), rgb_buf.end());
    return true;
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
        string extrinsics_config_dir = product_extrinsics_path_;
        std::ifstream ifs(extrinsics_config_dir, std::ios::binary);
        if (!ifs.is_open()) {
            cerr << "[AutoCali err] 打开文件失败: " << extrinsics_config_dir << std::endl;
            throw std::runtime_error("automatic calibration configuration I/O or JSON failure");
        }
        std::stringstream ss;
        ss << ifs.rdbuf();
        string config = ss.str();
        ifs.close();  

        rapidjson::Document ai_node_json;
        ai_node_json.Parse(config.c_str());
        if (ai_node_json.HasParseError()||!ai_node_json.IsObject()){
        cerr << "[AutoCali err] json解析失败: "<< std::endl;
        throw std::runtime_error("automatic calibration configuration JSON invalid");
        } 
        double avg_roll = sum_roll / cali_count;
        double avg_pitch = sum_pitch / cali_count;
        double avg_h = sum_h / cali_count;
        
        if (ai_node_json.HasMember("extrinsics") &&
            ai_node_json["extrinsics"].IsObject()&&
            ai_node_json["extrinsics"].HasMember("z")&&
            ai_node_json["extrinsics"].HasMember("roll")&&
            ai_node_json["extrinsics"].HasMember("pitch")&&
            ai_node_json["extrinsics"].HasMember("yaw")) {
            rapidjson::Value& pp = ai_node_json["extrinsics"];  
            pp["z"].SetDouble(roundDouble(avg_h,2));
            pp["roll"].SetDouble(roundDouble(avg_roll,2));   
            pp["pitch"].SetDouble(roundDouble(avg_pitch,2));
            pp["yaw"].SetDouble(roundDouble(yaw_cali,2));  
        } else throw std::runtime_error("automatic calibration extrinsics fields missing");
        rapidjson::StringBuffer buffer;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        writer.SetIndent(' ', 4);
        ai_node_json.Accept(writer);

        std::ofstream ofs(extrinsics_config_dir, std::ios::binary);
        if(!ofs.is_open())
        {
            std::cerr << "[AutoCali err] 写配置文件失败 "<< extrinsics_config_dir << std::endl;
            throw std::runtime_error("automatic calibration configuration I/O or JSON failure");
        }
        ofs << buffer.GetString();
        ofs.close();
        if(!ofs) throw std::runtime_error("automatic calibration configuration write failed");
        std::cout << "标定参数已写入文件: Roll:" << avg_roll<<" ,Pitch:"<<avg_pitch<<" ,z:"<<avg_h<<std::endl;        
        cali_count = 0;sum_roll = 0;sum_pitch = 0; sum_h = 0;
        return;
    }

}

}  // namespace ai_node_percep_demo
