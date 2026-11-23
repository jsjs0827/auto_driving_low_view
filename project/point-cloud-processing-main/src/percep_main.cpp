/**
 * @file percep_main.cpp
 * @brief Minimal pub/sub/timer function example.
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved.
 */
#include "percep_main/percep_main.h"

#include <malloc.h>
#include <cmath>


REGISTER_FUNCTION(factory) {
  cout << "[PercepMain] register collect function, function_name:" << FUNCTION_NAME << endl;
  LOG_WARN("[PercepMain] register collect function, function_name: %s",FUNCTION_NAME);
  factory.registerCreater<perception::PercepMain>(FUNCTION_NAME);
}

namespace perception {
PercepMain::PercepMain(const rapidjson::Value& json_conf) : daye::Function(json_conf) {
    LOG_INFO("========== System Version ==========");
    LOG_INFO("Git Commit: %s", GIT_COMMIT);
    LOG_INFO("Git Branch: %s", GIT_BRANCH);
    LOG_INFO("Build Time: %s %s", __DATE__, __TIME__);
    LOG_INFO("====================================");
    rapidjson::Document extrinsics_json,start_json;
    if (!this->loadExtrinsicsConfig(extrinsics_json)) {
        LOG_ERROR("[PercepMain ERR]标定配置丢失");
        healthCodeStatus.CalibrateMiss = true;
        return;}
    if(!this->initExtrinsicsParams(extrinsics_json)) {
        LOG_ERROR("[PercepMain ERR]标定配置异常，校验失败！");
        healthCodeStatus.CalibrateError = true;
        return; }
    if (!this->loadStartConfig(start_json)) {
        LOG_ERROR("[PercepMain ERR]启动配置丢失");
        healthCodeStatus.StartConfigMiss = true;
        return;}
    if(!this->initStartConfig(start_json)) {
        LOG_ERROR("[PercepMain ERR]启动配置异常，校验失败！");
        healthCodeStatus.StartConfigError = true;
        return; }
    
    // Initialize direct exposure/RAW processing before camera subscriptions.
    // It does not depend on map/recharge mode, GDC, or camera intrinsics.
    dark_scene_runtime_ = std::make_unique<DarkSceneRuntime>();
    std::string dark_scene_error;
    if (!dark_scene_runtime_->start(start_json, offline, &dark_scene_error)) {
        LOG_ERROR("[DarkScene] configuration/start failed: %s", dark_scene_error.c_str());
        healthCodeStatus.StartConfigError = true;
        return;
    }
    if (dark_scene_runtime_->enabled() && raw_lux_cfg_.enabled) {
        LOG_ERROR("[DarkScene] enable either dark_scene or legacy raw_lux, not both");
        dark_scene_runtime_->stop();
        healthCodeStatus.StartConfigError = true;
        return;
    }
    if(!this->initInterface()) {
        dark_scene_runtime_->stop();
        LOG_ERROR("[PercepMain ERR]接口初始化失败！");
        healthCodeStatus.StartConfigError = true;
        return; }

    // global_json.CopyFrom(json_conf, global_json.GetAllocator());
   
    //-----------------ros service-------------------//
    if (!self_test)
    {
        percepStatus_server_ = daye::CreateServer<common_interface::srv::GetPercepStatus>(kGetPercepStatus, &PercepMain::percepStatus_callback,this);  
        percepCmd_server_ = daye::CreateServer<common_interface::srv::ToPercepCmd>(kToPercepCmd, &PercepMain::percepCmd_callback,this);
        appCmd_client_ = daye::CreateClient<common_interface::srv::ToAppCmd>(kToApplicationCmd);  
               
    }
    else{
        if (v2_map_) switchV2Task(1);
        else percep_map.store(true); 
    }
}

PercepMain::~PercepMain() {
    LOG_WARN("[PercepMain] ~PercepMain");
    percep_map.store(false);
    percep_station.store(false);
    offline_running.store(false);
    running = false;
    if (dark_scene_runtime_) dark_scene_runtime_->stop();
    if (raw_lux_mailbox_) {
        std::lock_guard<std::mutex> lock(raw_lux_mailbox_->mutex);
        raw_lux_mailbox_->accepting = false;
        for (auto& frames : raw_lux_mailbox_->frames) frames.clear();
    }
    raw_lux_left_sub_.reset(); raw_lux_right_sub_.reset();
    if (!offline){
        // 先停 SHM 回调，避免析构队列/锁时回调仍在 push
        subCamera = daye::ShmSubscriber{};
        subCameraGdc = daye::ShmSubscriber{};        
        rgb_data_cond_.notify_all();
        gdc_data_cond_.notify_all();
#if PERCEP_ENABLE_RECHARGE
        if (recharge_) recharge_->Stop();
        // 智能指针句柄析构自动 Destroy(Stop→释放 api→关日志线程)，无需 destroyRechargeMarkHost
        recharge_.reset();
#endif
        
    }
    
    rgb_data_cond_.notify_all();
    gdc_data_cond_.notify_all();
    cv_save_map.notify_all();
    if (offlineThreadHandle.joinable()) {offlineThreadHandle.join();}
    if (algThreadHandle.joinable()) {algThreadHandle.join(); }
    RCLCPP_WARN(rclcpp::get_logger("exit"), "algThreadHandle joined!");
    if (saveLogThreadHandle.joinable()){saveLogThreadHandle.join();}
    RCLCPP_WARN(rclcpp::get_logger("exit"), "saveLogThreadHandle joined!");
    if (healthCodeHandle.joinable()) {healthCodeHandle.join(); }
    // Model handles outlive every worker that can use them.
    raw_lux_runner_.reset();
    raw_lux_mailbox_.reset();
    dark_scene_runtime_.reset();
    stereo_runner_.reset();
    dnn_runner_.reset();
    RCLCPP_WARN(rclcpp::get_logger("exit"), "perception exit!");
    LOG_WARN("[PercepMain] perception exit!");    
}

DarkSceneResult PercepMain::readDarkSceneResult() {
    if (dark_scene_runtime_) return dark_scene_runtime_->readResult();
    return {};
}

void PercepMain::processSingleCode(std::atomic<bool> &codeHappen,bool &reportedStatus,bool clear_cond,perception::HealthCode healthCode,daye::HealthStatus errLevel)
{
    if(codeHappen) 
    {
        if(!reportedStatus)
        {
            daye::pushCode(static_cast<int32_t>(healthCode), errLevel);
            reportedStatus = true;
            LOG_INFO("[PercepMain]发布错误码：%d",static_cast<int32_t>(healthCode));
        }
        else if (clear_cond) 
        {
            daye::pushCode(static_cast<int32_t>(healthCode), daye::HealthStatus::kNormal);
            codeHappen = false;
            reportedStatus= false;
            LOG_INFO("[PercepMain]清空错误码：%d",static_cast<int32_t>(healthCode));
        }            
    }
}
void PercepMain::healthCodeThread()
{
    const auto rate = std::chrono::milliseconds(100); //10HZ
    HealthCodeReportedStatus reportedStatus; 
    while(running)
    {
        //healthCodeStatus不允许外部线程置false,恢复在线程内解决
        //clear_cond为false表示无法判断恢复
        //相机相关

        this->processSingleCode(healthCodeStatus.CameraParamsMiss,reportedStatus.CameraParamsMiss,receive_params.load(),perception::HealthCode::CameraParamsMiss,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.CameraParamsError,reportedStatus.CameraParamsError,params_ok.load(),perception::HealthCode::CameraParamsError,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.RgbError,reportedStatus.RgbError,rgb_ok.load(),perception::HealthCode::RgbError,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.DispError,reportedStatus.DispError,disp_ok.load(),perception::HealthCode::DispError,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.GdcError,reportedStatus.GdcError,gdc_ok.load(),perception::HealthCode::GdcError,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.RgbNotReceived,reportedStatus.RgbNotReceived,rgb_rec.load(),perception::HealthCode::RgbNotReceived,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.DispNotReceived,reportedStatus.DispNotReceived,disp_rec.load(),perception::HealthCode::DispNotReceived,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.GdcNotReceived,reportedStatus.GdcNotReceived,gdc_rec.load(),perception::HealthCode::GdcNotReceived,daye::HealthStatus::kFailed);            
        

        this->processSingleCode(healthCodeStatus.LogWarn,reportedStatus.LogWarn,log_ok.load(),perception::HealthCode::LogWarn,daye::HealthStatus::kWarn);
        this->processSingleCode(healthCodeStatus.CameraBlockedWarn,reportedStatus.CameraBlockedWarn,cam_clear.load(),perception::HealthCode::CameraBlockedWarn,daye::HealthStatus::kWarn);
        this->processSingleCode(healthCodeStatus.CameraExposureWarn,reportedStatus.CameraExposureWarn,exposure_ok.load(),perception::HealthCode::CameraExposureWarn,daye::HealthStatus::kWarn);
        //初始化相关
        this->processSingleCode(healthCodeStatus.StartConfigMiss,reportedStatus.StartConfigMiss,false,perception::HealthCode::StartConfigMiss,daye::HealthStatus::kFailed);  
        this->processSingleCode(healthCodeStatus.StartConfigError,reportedStatus.StartConfigError,false,perception::HealthCode::StartConfigError,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.CalibrateMiss,reportedStatus.CalibrateMiss,false,perception::HealthCode::CalibrateMiss,daye::HealthStatus::kFailed);
        this->processSingleCode(healthCodeStatus.CalibrateError,reportedStatus.CalibrateError,false,perception::HealthCode::CalibrateError,daye::HealthStatus::kFailed);
        std::this_thread::sleep_for(rate);
    }
}
void PercepMain::onCameraFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info) {
    if (data == nullptr || size != sizeof(CameraData)) {
        LOG_ERROR("[PercepMain ERR] 接收RGB数据异常, invalid camera rgb frame size: expected= %d, actual= %d",sizeof(CameraData),size);
        healthCodeStatus.RgbError = true;
        healthCodeStatus.DispError = true;
        rgb_ok.store(false);
        disp_ok.store(false);
        return;
    }
    
    CameraData latest_camera_data_{};
    std::memcpy(&latest_camera_data_, data, sizeof(CameraData));
    // The host owns the camera ABI, exactly as for detection/segmentation.
    // The dark-scene SDK receives ordinary values and never decodes SHM bytes.
    // Evaluate exposure before taking the RGB queue lock. Same-process callers
    // of readDarkSceneResult() can retrieve the current decision on demand.
    const auto& exposure = latest_camera_data_.left;
    if (dark_scene_runtime_ && exposure.width == rgb_width && exposure.height == rgb_height &&
        exposure.time_ns && std::isfinite(exposure.exp_time) && exposure.exp_time > 0 &&
        std::isfinite(exposure.analog_gain) && exposure.analog_gain > 0) {
        (void)dark_scene_runtime_->evaluateExposure(exposure.frame_id, exposure.time_ns,
                                                    exposure.exp_time, exposure.analog_gain);
    }
    // 生产者：上锁直接覆盖最新帧，旧帧直接丢弃，不排队。
    // Dark-scene metadata ingestion above never holds the RGB queue lock.
    std::unique_lock<std::mutex> lock(rgb_mtx_);
    while (rgb_buffer_queue.size() >= 3) rgb_buffer_queue.pop();
    if(latest_camera_data_.left.bytes_used == 0 || !latest_camera_data_.left_data ){
        LOG_ERROR("[PercepMain ERR] 接收RGB数据异常, invalid camera rgb frame data!,bytes_used为0");
        healthCodeStatus.RgbError = true;
        rgb_ok.store(false);
        return;
    }
    if (stereo_cfg_.enabled) {
        // The map source is this packet's real left/right pair, not its old SDK disparity.
        std::string error;
        if (!validateStereoFrame(latest_camera_data_, &error)) {
            std::queue<CameraData>().swap(rgb_buffer_queue);
            LOG_ERROR("[Stereo] reject camera pair: %s", error.c_str());
            healthCodeStatus.RgbError = true;
            healthCodeStatus.DispError = true;
            rgb_ok.store(false);
            disp_ok.store(false);
            return;
        }
    } else if (latest_camera_data_.disp.bytes_used == 0 || !latest_camera_data_.disp_data) {
        LOG_ERROR("[PercepMain ERR] 接收DISP数据异常, invalid camera disp frame data!,bytes_used为0");
        healthCodeStatus.DispError = true;
        disp_ok.store(false);
        return;
    }
    rgb_ok.store(true);
    disp_ok.store(true);
    rgb_buffer_queue.push(latest_camera_data_);
    rgb_data_cond_.notify_one(); 

    LOG_DEBUG("[PercepMain]Latest RGB current time_ms: %s,left exposure_ms: %f, right exposure_ms:%f,left analog_gain:%f,right analog_gain:%f",
        ms_timestamp_to_str(daye::getSystemTimeMs()).c_str(),latest_camera_data_.left.exp_time,latest_camera_data_.right.exp_time,latest_camera_data_.left.analog_gain,latest_camera_data_.right.analog_gain);

}

void PercepMain::onGdcFrame(const uint8_t* data, std::size_t size, const daye::FrameInfo& info) {
    if (data == nullptr || size != sizeof(CameraGdcData)) {
        LOG_ERROR("[PercepMain ERR] 接收GDC数据异常, invalid camera gdc frame size: expected= %d, actual= %d",sizeof(CameraGdcData),size);
        gdc_ok.store(false);
        healthCodeStatus.GdcError = true;
        return;
    }

    std::unique_lock<std::mutex> lock(gdc_mtx_);
    while (gdc_buffer_queue.size() >= 3) gdc_buffer_queue.pop();
    LOG_DEBUG("[PercepMain]Latest Gdc current time_ms: %s",ms_timestamp_to_str(daye::getSystemTimeMs()).c_str());
    CameraGdcData latest_gdc_data_{};
    std::memcpy(&latest_gdc_data_, data, sizeof(CameraGdcData));
        
    if(latest_gdc_data_.left.bytes_used == 0 || !latest_gdc_data_.left_data){
        LOG_ERROR("[PercepMain ERR] 接收GDC数据异常, invalid camera gdc frame data!,bytes_used为0");
        gdc_ok.store(false);
        healthCodeStatus.GdcError = true;
        return;
    }
    gdc_ok.store(true);
    gdc_buffer_queue.push(latest_gdc_data_);
    gdc_data_cond_.notify_one();     

}

void PercepMain::onCamParams(const common_interface::msg::CameraParam &params)
{
    LOG_INFO("[PercepMain] 收到相机参数: fx: %f, fy: %f, bxf: %f, base_line: %f, width: %f, height: %f",params.fx,params.fy,params.bxf,params.base_line,params.width,params.height);
    {
        std::lock_guard<std::mutex> lock(camera_params_mtx_);
        camera_params = params;
        receive_params.store(true);
        if (!checkCamParams()) {
            LOG_ERROR("[PercepMain ERR] 相机内参校验不通过！");
            healthCodeStatus.CameraParamsError = true;
            params_ok.store(false);
            return;
        }
        params_ok.store(true);
    }
#if PERCEP_ENABLE_RECHARGE
    if (recharge_) {
        recharge_->setCameraK(params.width, params.height, params.fx, params.fy,
                              params.cx, params.cy);
    }
#endif
    if (save_log){
        this->saveCamParams(params);
    }
}

bool PercepMain::checkCamParams()
{
    double fx = camera_params.fx;
    double fy = camera_params.fy;
    double cx = camera_params.cx;
    double cy = camera_params.cy;
    int W = camera_params.width;
    int H = camera_params.height;
    double base_line = camera_params.base_line;
    double bxf = camera_params.bxf;
    //1.焦距必须大于0，且不能离谱
    if(fx <= 1e-3 || fx > W * 2.0) {return false;}
    if(fy <= 1e-3 || fy > H * 2.0) {return false;}

    //2.光心cx,cy必须在图像范围内，留少量容错（标定误差）
    if(cx < -10 || cx > W + 10) {return false;}
    if(cy < -10 || cy > H + 10) {return false;}

    //3.baseline基线，毫米，必须正数，机器人双目限定范围
    if(base_line <= 1e-3 || base_line > 200.0) {return false;}

    //4.校验bxf和fx*baseline一致性，浮点允许微小误差
    double calc_bxf = fx * base_line;
    const double eps = 2;
    if(fabs(bxf - calc_bxf) > eps)
    {
        return false;
    }

    return true;
}
bool PercepMain::percepStatus_callback(common_interface::srv::GetPercepStatus::Request& req,common_interface::srv::GetPercepStatus::Response& res)
{
  //查询感知状态
    if (req.cmd == 1)
    {
        LOG_INFO("[PercepMain] 收到感知查询请求");
        // -1 - 异常，0 - 待机中，1 - 感知地图发布中，2 - 基站二维码发布中 
        res.result = percep_status; 
        LOG_INFO("[PercepMain]当前感知状态：%d (-1 - 异常,0 - 待机中,1 - 感知地图发布中,2 - 基站二维码发布中)",res.result);
    }
    else
    {
       LOG_ERROR("[PercepMain ERR] 接收查询指令异常！");
    }
    return true;
}
bool PercepMain::percepCmd_callback(common_interface::srv::ToPercepCmd::Request& req,common_interface::srv::ToPercepCmd::Response& res)
{
    if (v2_map_ && (req.cmd == 1 || req.cmd == 2 || req.cmd == 21)) {
        res.result = 0;
        const int status = switchV2Task(req.cmd);
        const int app_command = req.cmd == 21 ? 2021 : 2000 + req.cmd;
        callAppService(app_command, status, 10000);
        return true;
    }
    if (req.cmd == 1)
    {
        LOG_INFO("[PercepMain] 收到启动感知地图发布请求");
        res.result = 0;
        percep_map.store(true);
        percep_station.store(false);
        stop_percep.store(false);
#if PERCEP_ENABLE_RECHARGE
        if (recharge_) recharge_->Stop();
#endif
        this->callAppService(2001,1,10000);
    }
    else if (req.cmd == 2)
    {
#if PERCEP_ENABLE_RECHARGE
        LOG_INFO("[PercepMain] 收到基站二维码发布模块启动指令");
        res.result = 0;
        percep_station.store(true);
        percep_map.store(false);
        stop_percep.store(false);
        if (!recharge_) {
            LOG_WARN("[PercepMain] recharge_mark: host not created");
            this->callAppService(2002, 2, 10000);
            return true;
        }
        if (!recharge_->Start()) {
            LOG_ERROR("[PercepMain ERR] recharge_mark: start failed");
            this->callAppService(2002, 2, 10000);
            return true;
        }
        this->callAppService(2002, 1, 10000);
#else
        res.result = -1;
        LOG_WARN("[PercepMain] recharge_mark is disabled in this build");
        this->callAppService(2002, 2, 10000);
#endif
    }
    else if (req.cmd == 21)
    {
        LOG_INFO("[PercepMain] 收到感知暂停工作请求");
        //停止工作
        res.result = 0;
        percep_map.store(false); 
        percep_station.store(false);
        stop_percep.store(true);
        sleep(1);
        this->releaseTaskResources();

        this->callAppService(2021,1,10000);  //上报感知停止工作完成           
        // this->callAppService(2021,2,10000);  //上报感知停止工作失败 
        
    }
    else if (req.cmd == 9002)
    {
        LOG_INFO("[PercepMain] 收到感知恢复出厂设置请求");
        if (fs::exists(output_dir)&& fs::is_directory(output_dir))
        {
        fs::remove_all(output_dir);
        LOG_INFO("[PercepInit]日志数据目录初始化完毕！");
        }
        this->callAppService(9002,1,10000);  //上报感知恢复出厂完成  
    }
    else{
      LOG_ERROR("[PercepMain ERR]接收感知执行指令异常！");
    }
    return true;
}

// cmd=21 暂停时的清理（最小范围）：只处理回充任务相关，
// 即 Stop worker + 清 GDC 残帧队列 + trim；其他（RGB/点云/DNN/存档）
// 一律不动，避免影响感知地图逻辑和任务。
void PercepMain::releaseTaskResources() {
#if PERCEP_ENABLE_RECHARGE
  if (recharge_) recharge_->Stop();
#endif
  // GDC 是回充主用帧：暂停时残帧必然过期，清掉只避免旧高水位
  // 被带到下一轮，不影响下一轮任务逻辑。
  {
    std::lock_guard<std::mutex> lk(gdc_mtx_);
    std::queue<CameraGdcData>().swap(gdc_buffer_queue);
  }
#if 0  // 对比测试阶段2再放开：全量堆积清理（存档任务/点云/DNN 中间缓存）
  {
    std::lock_guard<std::mutex> lk(save_map_mtx_);
    pending_task_.reset();
  }
  // 点云/DNN 中间缓存：清 size 并归还 capacity。
  auto reset_cloud = [](PointCloudT::Ptr& c) {
    if (c) {
      c->clear();
      c->points.shrink_to_fit();
      c->width = 0;
      c->height = 1;
    } else {
      c.reset(new PointCloudT);
    }
  };
  reset_cloud(ground_cloud);
  reset_cloud(no_ground_cloud);
  reset_cloud(obs_cloud);
  reset_cloud(ori_cloud);
  reset_cloud(ramp_cloud);
  reset_cloud(offline_sparse_cloud);
  reset_cloud(last_semantic_cloud_);
  last_dnn_dets_.clear();
  last_dnn_raw_dets_.clear();
  last_dnn_dets_.shrink_to_fit();
  last_dnn_raw_dets_.shrink_to_fit();
  last_dnn_seg_mask_.clear();
  last_dnn_seg_mask_.shrink_to_fit();
  dnn_overlay_buf_.clear();
  dnn_overlay_buf_.shrink_to_fit();
  last_dnn_overlay_save_.clear();
  last_dnn_overlay_save_.shrink_to_fit();
  frame_rgb_buf_.clear();
  frame_rgb_buf_.shrink_to_fit();
  pcl_proj_rgb_buf_.clear();
  pcl_proj_rgb_buf_.shrink_to_fit();
  disp_vec.clear();
  disp_vec.shrink_to_fit();
  u_vec.clear();
  u_vec.shrink_to_fit();
  v_vec.clear();
  v_vec.shrink_to_fit();
  gdc_forsave.reset();
#endif
  // glibc free 的内存默认不还 OS，top RES 看似不掉；trim 后高水位真实回落。
  ::malloc_trim(0);
  LOG_INFO("[PercepMain] task resources released (recharge stop + gdc queue trimmed)");
}

void PercepMain::convertGridMap(int count)
{
    OccMapData fusion_data;
    if (frame_skip_obstacle_ && exp_guard_cfg_.skip_ai_obstacle &&
        last_semantic_cloud_ && !last_semantic_cloud_->empty())
    {
        // 图像糊了检测框/掩码同样不准，连 AI 障碍一起跳过
        LOG_INFO("[PercepMain] skip AI obstacle for this frame!");
        last_semantic_cloud_.reset(new PointCloudT);
    }

    aiFusionPtr->initData(ground_cloud, obs_cloud, ramp_cloud,last_semantic_cloud_);
    aiFusionPtr->integrate(fusion_data);

    string time_str = frame_save_time_str_.empty()
                          ? ms_timestamp_to_str(daye::getSystemTimeMs())
                          : frame_save_time_str_;

    common_interface::msg::GridMapInfo grid_msg;
    grid_msg.header.stamp.sec = daye::getSteadyTimeMs() / 1000;
    grid_msg.header.stamp.nanosec = (daye::getSteadyTimeMs() % 1000) * 1000000;
    grid_msg.header.frame_id = parent_frame;
    grid_msg.rows = static_cast<uint16_t>(mapHeight / mapResolution );
    grid_msg.cols = static_cast<uint16_t>(mapWidth / mapResolution );
    grid_msg.resolution_mm = static_cast<uint16_t>(mapResolution *1000);
    grid_msg.avg_h_cm = fusion_data.map_avg_h;
    grid_msg.min_h_cm = fusion_data.map_min_h;
    grid_msg.max_h_cm = fusion_data.map_max_h;
    grid_msg.type = fusion_data.type;
    grid_map_pub_.publish(grid_msg);

    auto start = std::chrono::high_resolution_clock::now();
    static uint64_t grid_map_frame_id = 0;
    const uint64_t frame_id = ++grid_map_frame_id;
    const size_t expected_count =
        static_cast<size_t>(grid_msg.rows) * grid_msg.cols;

    if (fusion_data.map_avg_h.size() != expected_count ||
        fusion_data.map_min_h.size() != expected_count ||
        fusion_data.map_max_h.size() != expected_count ||
        fusion_data.type.size() != expected_count)
    {
        LOG_ERROR(
            "[GRIDMAP_ERROR] frame=%llu rows=%u cols=%u "
            "avg=%zu min=%zu max=%zu type=%zu expected=%zu",
            static_cast<unsigned long long>(frame_id),
            static_cast<unsigned int>(grid_msg.rows),
            static_cast<unsigned int>(grid_msg.cols),
            fusion_data.map_avg_h.size(),
            fusion_data.map_min_h.size(),
            fusion_data.map_max_h.size(),
            fusion_data.type.size(),
            expected_count);
    }
    else
    {
        size_t saved_count = 0;
        const std::string data = encodeSparseGridText(
            fusion_data.type,
            fusion_data.map_avg_h,
            fusion_data.map_min_h,
            fusion_data.map_max_h,
            saved_count);

        LOG_INFO(
            "[GM] f=%llu r=%u c=%u s=%u k=%zu d=%s",
            static_cast<unsigned long long>(frame_id),
            static_cast<unsigned int>(grid_msg.rows),
            static_cast<unsigned int>(grid_msg.cols),
            static_cast<unsigned int>(static_cast<uint16_t>(mapResolution *1000)),
            saved_count,
            data.c_str());
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end- start); 
    LOG_INFO("[PercepMain] writeMap Time taken:  %d ms",duration.count());

    if(visualize){
        this->publishColoredMap(fusion_data.pcl_map,"pcl",count);
        this->publishColoredMap(fusion_data.ai_map,"ai",count);
        this->publishColoredMap(fusion_data.fusion_map,"fusion",count);
    }
    
    if (save_log)
    {
        const uint64_t now_ms = daye::getSystemTimeMs();
        // 用 wall-clock 做保存节流：感知计算每帧照跑，保存只提交最新完成帧。
        // 旧版本 saveLogLoop 写盘慢时自然被“IO busy 覆盖”压到约1.5 FPS；
        // 这里用间隔明确复现实测低CPU的保存速率，而不是跳过感知帧。
        if (save_interval_ms > 0 && last_saved_frame_ms_ != 0 &&
            now_ms - last_saved_frame_ms_ <
                static_cast<uint64_t>(save_interval_ms)) {
            ++save_interval_drop_;
            return;
        }
        last_saved_frame_ms_ = now_ms;
        // 保存线程只有一个待处理槽；槽未空时直接丢弃本帧保存请求，
        // 避免先复制RGB/disp、遍历语义点云后再覆盖任务。
        std::unique_lock<std::mutex> save_lk(save_map_mtx_, std::try_to_lock);
        if (!save_lk.owns_lock() || pending_task_.has_value()) {
            ++save_busy_drop_;
            return;
        }
        ++save_submit_count_;
        auto new_task = std::make_unique<SaveMapTask>();
        new_task->map_data = fusion_data;
        new_task->rgb = frame_rgb_buf_;
        new_task->disp = disp_vec;
        new_task->gdc = std::move(gdc_forsave);
        new_task->u = u_vec;
        new_task->v = v_vec;
        new_task->ai_cloud_overlay_w = rgb_width;
        new_task->ai_cloud_overlay_h = rgb_height;
        if (last_semantic_cloud_ &&
            last_semantic_cloud_->size() == u_vec.size() &&
            u_vec.size() == v_vec.size()) {
            // 只保存与稀疏disp点一一对应的类别，不在板端构造640x544稠密mask。
            // 稠密投影由PC脚本结合disp中的u/v生成，减少每帧初始化和写盘约340 KiB。
            new_task->ai_cloud_overlay_classes.resize(last_semantic_cloud_->size(), 0);
            for (size_t i = 0; i < last_semantic_cloud_->size(); ++i) {
                const int label = last_semantic_cloud_->points[i].label;
                if (label < 0) continue;
                if (IsSegPointLabel(label)) {
                    const int seg_id = DecodeSegClassId(label);
                    const bool is_grass =
                        std::find(passable_seg_ids_.begin(), passable_seg_ids_.end(),
                                  seg_id) != passable_seg_ids_.end();
                    if (!is_grass) new_task->ai_cloud_overlay_classes[i] = 2;
                } else {
                    new_task->ai_cloud_overlay_classes[i] = 1;
                }
            }
        } else if (last_semantic_cloud_ && !last_semantic_cloud_->empty()) {
            LOG_ERROR("[PercepMain] skip ai_cloud_overlay raw: cloud=%llu u=%llu v=%llu",
                      static_cast<unsigned long long>(last_semantic_cloud_->size()),
                      static_cast<unsigned long long>(u_vec.size()),
                      static_cast<unsigned long long>(v_vec.size()));
        }
        // 只保存后续离线转换需要的AI原始结果，不在主循环生成overlay图片。
        new_task->seg_mask = last_dnn_seg_mask_;
        // 存原始检测结果（模型全部类别），保证离线可分析非 keep_classes 类别。
        new_task->raw_dets = last_dnn_raw_dets_;
        new_task->dets = last_dnn_dets_;
        new_task->dnn_img_w = last_dnn_img_w_;
        new_task->dnn_img_h = last_dnn_img_h_;
        new_task->time_str = time_str;
        new_task->rows = static_cast<int>(mapHeight /mapResolution);
        new_task->cols = static_cast<int>(mapWidth /mapResolution);

        pending_task_ = std::move(new_task);
        save_lk.unlock();
        cv_save_map.notify_one();
    }
}

void PercepMain::saveLogLoop()
{
    uint32_t saved_task_count = 0;
    while (running)
    {
        std::unique_lock<std::mutex> lk(save_map_mtx_);
        cv_save_map.wait(lk, [this]() {
            return !running.load() || pending_task_.has_value() || pending_v2_save_.has_value();
        });

        if (!running)
        {
            log_ok.store(false);
            healthCodeStatus.LogWarn = true;
            break;
        }
        if (pending_v2_save_) {
            auto task = std::move(*pending_v2_save_);
            pending_v2_save_.reset();
            lk.unlock();
            psdk_status status{};
            if (psdk_full_save_result(task.first.get(), task.second.c_str(), &status)) {
                LOG_WARN("[V2] save map: %s", status.detail);
            }
            continue;
        }


        // 取出任务，清空待处理
        std::unique_ptr<SaveMapTask> task = std::move(pending_task_.value());
        pending_task_.reset();
        lk.unlock(); // 提前解锁，不阻塞timer生产者

        const auto save_t0 = std::chrono::steady_clock::now();

        try
        {
            // sensor_msgs::msg::Image gdc_img_msg;
            // std::vector<uint8_t> frame_gdc_buf_;
            // frame_gdc_buf_.clear();
            // frame_gdc_buf_.reserve(gdc_width*gdc_height*3);
            // this->GdcToRgbRos(*(task->gdc), gdc_img_msg,frame_gdc_buf_);      
            // visualizePtr->saveGDC(frame_gdc_buf_, task->time_str);
            visualizePtr->saveRGB(task->rgb, task->time_str);
            visualizePtr->saveDisp(task->u, task->v, task->disp, task->time_str);
            visualizePtr->saveAiOverlayJson(task->raw_dets, task->seg_mask,
                                                task->dnn_img_w, task->dnn_img_h,
                                                task->time_str);
            visualizePtr->saveAiOverlayMaskRaw(task->seg_mask,
                                                task->dnn_img_w,
                                                task->dnn_img_h,
                                                task->time_str);
            if (!task->ai_cloud_overlay_classes.empty())
            {
                visualizePtr->saveAiCloudOverlayRaw(
                    task->ai_cloud_overlay_classes,
                    task->ai_cloud_overlay_w,
                    task->ai_cloud_overlay_h,
                    task->time_str);
            }
        }
        catch (...)
        {
            LOG_ERROR("[PercepMain ERR] async save data exception frame: %s",task->time_str.c_str());
            log_ok.store(false);
            healthCodeStatus.LogWarn = true;
        }
    

        try
        {
            // auto start = std::chrono::high_resolution_clock::now();
            // static uint64_t grid_map_frame_id = 0;
            // const uint64_t frame_id = ++grid_map_frame_id;
            // const size_t expected_count =
            //     static_cast<size_t>(task->rows) * task->cols;

            // if (task->map_data.map_avg_h.size() != expected_count ||
            //     task->map_data.map_min_h.size() != expected_count ||
            //     task->map_data.map_max_h.size() != expected_count ||
            //     task->map_data.type.size() != expected_count)
            // {
            //     LOG_ERROR(
            //         "[GRIDMAP_ERROR] frame=%llu rows=%u cols=%u "
            //         "avg=%zu min=%zu max=%zu type=%zu expected=%zu",
            //         static_cast<unsigned long long>(frame_id),
            //         static_cast<unsigned int>(task->rows),
            //         static_cast<unsigned int>(task->cols),
            //         task->map_data.map_avg_h.size(),
            //         task->map_data.map_min_h.size(),
            //         task->map_data.map_max_h.size(),
            //         task->map_data.type.size(),
            //         expected_count);
            // }
            // else
            // {
            //     size_t saved_count = 0;
            //     const std::string data = encodeSparseGridText(
            //         task->map_data.type,
            //         task->map_data.map_avg_h,
            //         task->map_data.map_min_h,
            //         task->map_data.map_max_h,
            //         saved_count);

            //     LOG_INFO(
            //         "[GM] f=%llu r=%u c=%u s=%u k=%zu d=%s",
            //         static_cast<unsigned long long>(frame_id),
            //         static_cast<unsigned int>(task->rows),
            //         static_cast<unsigned int>(task->cols),
            //         static_cast<unsigned int>(static_cast<uint16_t>(mapResolution *1000)),
            //         saved_count,
            //         data.c_str());
            // }
            // auto end = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end- start); 
            // LOG_INFO("[PercepMain] writeMap Time taken:  %d ms",duration.count());
            visualizePtr->saveMapRaw(task->map_data.pcl_map, "pcl",
                                        task->time_str, task->rows, task->cols);
            visualizePtr->saveMapRaw(task->map_data.ai_map, "ai",
                                        task->time_str, task->rows, task->cols);
            visualizePtr->saveMapRaw(task->map_data.fusion_map, "fusion",
                                        task->time_str, task->rows, task->cols);
            // visualizePtr->saveMap(task->map_data.pcl_map, "pcl", task->time_str, task->rows, task->cols);
            // visualizePtr->saveMap(task->map_data.ai_map, "ai", task->time_str, task->rows, task->cols);
            // visualizePtr->saveMap(task->map_data.fusion_map, "fusion", task->time_str, task->rows, task->cols);

        }
        catch (...)
        {
            LOG_ERROR("[PercepMain ERR] async save map exception frame: %s",task->time_str.c_str());
            log_ok.store(false);
            healthCodeStatus.LogWarn = true;
        }
        

        const uint64_t task_ms =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - save_t0).count());
        save_last_task_ms_.store(task_ms);
        save_total_task_ms_.fetch_add(task_ms);
        uint64_t prev_max = save_max_task_ms_.load();
        while (task_ms > prev_max &&
               !save_max_task_ms_.compare_exchange_weak(prev_max, task_ms)) {}
        const uint64_t done = ++saved_task_count;
        save_done_count_.store(done);
        if (done % 5 == 0) {
            const uint64_t avg = save_total_task_ms_.load() / done;
            LOG_INFO("[PercepMain] save perf: done=%llu last=%llums avg=%llums max=%llums "
                     "submit=%llu interval_drop=%llu busy_drop=%llu frame=%s",
                     static_cast<unsigned long long>(done),
                     static_cast<unsigned long long>(task_ms),
                     static_cast<unsigned long long>(avg),
                     static_cast<unsigned long long>(save_max_task_ms_.load()),
                     static_cast<unsigned long long>(save_submit_count_.load()),
                     static_cast<unsigned long long>(save_interval_drop_.load()),
                     static_cast<unsigned long long>(save_busy_drop_.load()),
                     task->time_str.c_str());
        }

        if (done % 10000 == 0) {
            pclCommonPtr->keepNewestFiles(output_dir + "/data/rgb/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/data/disp/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/data/ai_det/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/data/ai_mask/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/data/ai_cloud_overlay_raw/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/map/pcl/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/map/ai/", 10000);
            pclCommonPtr->keepNewestFiles(output_dir + "/map/fusion/", 10000);
        }
    }
}
bool PercepMain::saveCamParams(const common_interface::msg::CameraParam &params)
{
    fs::path dir(output_dir);
    if (!dir.empty() && !fs::exists(dir)) {
        fs::create_directories(dir);
    }
    string save_path = output_dir+"/camera_param.json";
    FILE* fp = fopen(save_path.c_str(), "w");
    if(nullptr == fp)
    {
        LOG_ERROR("[PercepMain ERR] saveCameraParamToJson open failed!");
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
        writer.Key("bbox_minX"); writer.Double(roundDouble(box_min_x,2));
        writer.Key("bbox_minY"); writer.Double(roundDouble(box_min_y,2));
        writer.Key("bbox_minZ"); writer.Double(roundDouble(box_min_z,2));
        writer.Key("bbox_maxX"); writer.Double(roundDouble(box_max_x,2));
        writer.Key("bbox_maxY"); writer.Double(roundDouble(box_max_y,2));
        writer.Key("bbox_maxZ"); writer.Double(roundDouble(box_max_z,2));
        writer.EndObject();       

    writer.Key("sample_step");writer.Int(sample_step);

    writer.EndObject();            // 闭合最外层
    fclose(fp);
    LOG_INFO("[PercepMain] save camera param SUCCESS!");
    return true;  
}
void PercepMain::publishColoredMap(const std::vector<MapPoint>& map_points,string header,int index)
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
    visualizePtr->createColoredMap(map_points,*marker_array,header,mapWidth,mapHeight,mapResolution,index);
    if (header == "pcl"){pclmap_pub_.publish(*marker_array);}
    if (header == "ai"){aimap_pub_.publish(*marker_array);}
    if (header == "fusion"){fusionmap_pub_.publish(*marker_array);}
}

void PercepMain::publish_ros_cloud(PointCloudT::Ptr &cloud_in,daye::RosPublisher& pub,string frame_id)
{
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud_in, ros_msg);
    ros_msg.header.frame_id = frame_id;
    ros_msg.header.stamp = rclcpp::Clock().now();
    pub.publish(ros_msg);
}
void PercepMain::publish_ros_rgbcloud(PointCloudT::Ptr &cloud_in, daye::RosPublisher& pub,string frame_id)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    rgb_cloud->resize(cloud_in->size());
    auto& out_points = rgb_cloud->points;
    size_t idx = 0;
    PointCloudT cloud = *cloud_in;

    for (const auto& pt : cloud.points) {
        uint8_t r = pt.r, g = pt.g, b = pt.b;
        auto& out = out_points[idx];
        out.x = pt.x;
        out.y = pt.y;
        out.z = pt.z;
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
void PercepMain::callAppService(int32_t cmd, int32_t status,double timeout_us) 
{
  common_interface::srv::ToAppCmd::Request app_request;
  common_interface::srv::ToAppCmd::Response app_response;
  app_request.cmd = cmd;
  app_request.status = status;
  if (!appCmd_client_.call<common_interface::srv::ToAppCmd>(app_request, app_response, timeout_us)) {
    LOG_ERROR("[PercepMain ERR] 上报中间件失败");
    return;
  }else{
    LOG_INFO("[PercepMain] 收到中间件回应: %d",app_response.result);
  }
}

}  // namespace perception
