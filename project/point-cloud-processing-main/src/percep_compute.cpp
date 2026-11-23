/**
 * @file percep_compute.cpp
 * @brief 感知主计算线程
 */
#include "percep_main/percep_main.h"


namespace perception {

void PercepMain::algThread() {
    const auto rate = std::chrono::milliseconds(10); //100HZ
    CameraData latest_rgb{};
    CameraGdcData latest_gdc{};
    while(running)
    {

        log_on = false;
        if (log_count / 200 ==1) {log_count = 0;log_on = true;}
        log_count++;
        if(percep_map ) {if(log_on){LOG_INFO("[PercepCompute]感知只发布地图！");}percep_status = 1;}
        if(percep_station) {if(log_on){LOG_INFO("[PercepCompute]感知只发布二维码！");}percep_status = 2;}    
        if(!percep_station && !percep_map) {if(log_on){LOG_INFO("[PercepCompute]感知待机中！");}percep_status = 0;}    
        // -------- RGB 数据读取 --------
        if(percep_map)
        {
            std::unique_lock<std::mutex> lock(rgb_mtx_);
            if (!rgb_data_cond_.wait_for(lock, std::chrono::milliseconds(1000), [this]() { return !rgb_buffer_queue.empty(); })){
                LOG_ERROR("[PercepCompute ERR] 超时,感知未收到rgb相机数据!");
                if(percep_status!=0)
                {
                    healthCodeStatus.RgbNotReceived = true;
                    healthCodeStatus.DispNotReceived = true;
                }
                rgb_rec.store(false);  //超时
                disp_rec.store(false);                    
            }else{
                latest_rgb = rgb_buffer_queue.back();
                std::queue<CameraData>().swap(rgb_buffer_queue);   
                rgb_rec.store(true);
                disp_rec.store(true);             
            }

        }

        // -------- GDC 数据读取--------
        {
            std::unique_lock<std::mutex> lock(gdc_mtx_);
            if (!gdc_data_cond_.wait_for(lock, std::chrono::milliseconds(1000), [this]() {return  !gdc_buffer_queue.empty();})){
                LOG_ERROR("[PercepCompute ERR] 超时,感知未收到gdc相机数据!");
                std::queue<CameraGdcData>().swap(gdc_buffer_queue);
                if(percep_status!=0)
                {
                    healthCodeStatus.GdcNotReceived = true;            
                }
                gdc_rec.store(false); 
            } else {
                latest_gdc = gdc_buffer_queue.back();
                std::queue<CameraGdcData>().swap(gdc_buffer_queue);
                gdc_rec.store(true);  
            }
        }

        if(stop_percep)
        {
            rgb_rec.store(true);
            disp_rec.store(true);
            gdc_rec.store(true);
            receive_params.store(true);
            // LOG_WARN("感知待机清空相机相关错误码！");
        }
        if (!receive_params ) {
            if (log_on) { LOG_ERROR("[PercepCompute ERR] 超时,感知未收到相机参数！！");}
            if (percep_status!=0)
            {
                healthCodeStatus.CameraParamsMiss = true;
            }
            std::this_thread::sleep_for(rate);
            continue;
        }
        
        if(percep_map || percep_station)
        {
            this->run_compute(latest_rgb,latest_gdc); 
        }

        std::this_thread::sleep_for(rate);
    }

}

void PercepMain::run_compute(CameraData &latest_rgb,CameraGdcData &latest_gdc)
{
    auto start_alg = std::chrono::high_resolution_clock::now();
    //启动二维码识别任务
    if (percep_station && recharge_ && gdc_rec) {
        // 未 Start/忙时 Tick 内部安全跳过并返回 1。
        frame_save_time_str_ = ms_timestamp_to_str(daye::getSystemTimeMs());
        const int tick_rc = recharge_->Tick(&latest_gdc, sizeof(CameraGdcData),
                                            (percep_station && !percep_map) ? 2000 : 0,
                                            frame_save_time_str_.c_str());
        if (tick_rc != 0 && log_on) {
            LOG_INFO("[PercepCompute] recharge_mark tick skip rc=%d", tick_rc);
        }
    }
    //启动感知地图任务
    if (percep_map && rgb_rec)
    {       
        frame_save_time_str_ = ms_timestamp_to_str(daye::getSystemTimeMs());
        LOG_INFO("[PercepMain]当前计算帧frame timestamp: %s",frame_save_time_str_.c_str());
        gdc_forsave = std::make_unique<CameraGdcData>(); *gdc_forsave = latest_gdc;
        auto start_1 = std::chrono::high_resolution_clock::now();
        // AI推理
        if (dnn_cfg_.enabled && dnn_runner_ ) {
            
            this->runDnnInference(latest_rgb.left, latest_rgb.left_data,true);
        }
        auto end_1 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_1 - start_1);
        
        //数据格式转换
        auto start_2 = std::chrono::high_resolution_clock::now();
        frame_rgb_buf_.clear();
        frame_rgb_buf_.reserve(rgb_width*rgb_height*3);
        const std::vector<uint8_t> *proj_rgb = &frame_rgb_buf_;
        sensor_msgs::msg::Image rgbd_img_msg;
        if (!this->NV12ToRgbRos(latest_rgb, rgbd_img_msg,frame_rgb_buf_))
        {
            LOG_ERROR("[PercepCompute ERR] latest_rgb Frame convert ros2 failed!");
            percep_status=-1;
            return;
        }
        proj_rgb = &frame_rgb_buf_; 

        if (visualize)
        {      
            rgbd_img_msg.header.frame_id = "percep_main";
            rgb_img_pub_.publish(rgbd_img_msg);   
        }
        auto end_2 = std::chrono::high_resolution_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_2- start_2);
        

        //点云生成
        auto start_3 = std::chrono::high_resolution_clock::now();
        PointCloudT::Ptr pcl_cloud(new PointCloudT);
        if (!offline)
        {
            if(!this->dense2sparse_pointcloud(latest_rgb,sample_step,pcl_cloud))
            {            
                LOG_ERROR("[PercepCompute ERR]点云生成失败！");
                percep_status=-1;
                return;
            }            
        }else{
            pcl_cloud = offline_sparse_cloud;
        }

        LOG_INFO("[PercepCompute] 原始点云个数：%d",pcl_cloud->size());

        if (pcl_cloud->size()<100)
        {
            LOG_WARN("[PercepCompute] 点云数量小于100，相机可能被遮挡！");
            healthCodeStatus.CameraBlockedWarn = true;
            cam_clear.store(false);
        }else{
            cam_clear.store(true);
        }
        auto end_3 = std::chrono::high_resolution_clock::now();
        auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_3- start_3);
        

        //自动标定（配置开关）          
        if (auto_cali) {this->autocali(pcl_cloud); percep_status=0;return;}
        cali = this->adaptGroundTrans(*pcl_cloud);
     
        //AI点云匹配投影
        ori_cloud = pcl_cloud;
        float fx = camera_params.fx, fy = camera_params.fy, cx = camera_params.cx, cy = camera_params.cy;
        auto start_4 = std::chrono::high_resolution_clock::now();
            this->matchAndPublishDetCloud(pcl_cloud, u_vec, v_vec, fx, fy, cx, cy,
                                          rgb_width, rgb_height);
        auto end_4 = std::chrono::high_resolution_clock::now();
        auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(end_4- start_4);
        

        //点云后处理
        auto start_5 = std::chrono::high_resolution_clock::now();
            this->postprocess(pcl_cloud, *proj_rgb, rgb_width, rgb_height,fx,fy,cx,cy);  
        auto end_5 = std::chrono::high_resolution_clock::now();
        auto duration5 = std::chrono::duration_cast<std::chrono::milliseconds>(end_5- start_5); 
            

        //地图融合
        auto start_6 = std::chrono::high_resolution_clock::now();
        this->convertGridMap(frame_count_);
        auto end_6 = std::chrono::high_resolution_clock::now();
        auto duration6 = std::chrono::duration_cast<std::chrono::milliseconds>(end_6- start_6);

        LOG_INFO("[PercepCompute] 1.runDnnInference Time taken: %d ms",duration1.count());
        LOG_INFO("[PercepCompute] 2.RGB convert Time taken: %d ms",duration2.count());
        LOG_INFO("[PercepCompute] 3.PointCloud generate Time taken: %d ms",duration3.count());
        LOG_INFO("[PercepCompute] 4.matchAndPublishDetCloud Time taken: %d ms",duration4.count());
        LOG_INFO("[PercepCompute] 5.postprocess Time taken: %d ms",duration5.count());   
        LOG_INFO("[PercepCompute] 6.convertGridMap Time taken:  %d ms",duration6.count());

        ++frame_count_;

    }
    auto end_alg = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_alg- start_alg);
    LOG_INFO("[PercepCompute] All Alg Time taken: %d ms",duration.count());
    LOG_INFO("\n");
}

// 依据本帧曝光时间/模拟增益评估运动模糊程度，刷新 frame_* 状态。
// exp_time 由相机 ISP 写进 SHM（PicData.exp_time，单位 ms），无需额外接口。
void PercepMain::evaluateFrameBlur(const CameraData& camera_data)
{
    const ExposureGuardConfig& c = exp_guard_cfg_;
    // disp 帧的曝光字段未必填充，取左右目较大值更保守
    frame_exp_ms_ = std::max(camera_data.left.exp_time, camera_data.right.exp_time);
    frame_gain_   = std::max(camera_data.left.analog_gain, camera_data.right.analog_gain);
    frame_blur_t_        = 0.f;
    frame_confidence_    = 1.f;
    frame_skip_obstacle_ = false;
    frame_drop_          = false;
    if (!c.enabled) return;

    // 线性斜坡：v<=lo → 0，v>=hi → 1
    auto ramp = [](float v, float lo, float hi) -> float {
        if (hi <= lo) return 0.f;
        return std::clamp((v - lo) / (hi - lo), 0.f, 1.f);
    };

    float t = 0.f;
    // exp_time<=0 说明这一路 SHM 没填该字段（老固件），跳过判定避免误杀
    if (frame_exp_ms_ > 0.f) {
        t = std::max(t, ramp(frame_exp_ms_, c.exp_soft_ms, c.exp_hard_ms));
    }
    if (frame_gain_ > 0.f) {
        t = std::max(t, ramp(frame_gain_, c.gain_soft, c.gain_hard));
    }
    frame_blur_t_ = t;
    // 降权：t=0 → 1.0，t=1 → min_confidence
    frame_confidence_ = 1.f - t * (1.f - std::clamp(c.min_confidence, 0.f, 1.f));

    if (t >= 1.f) {
        if (c.hard_action == "drop_frame") {
            frame_drop_ = true;
        } else if (c.hard_action == "skip_obstacle") {
            frame_skip_obstacle_ = true;
        }
        healthCodeStatus.CameraExposureWarn = true;
        exposure_ok.store(false);
    }else{
        exposure_ok.store(true);
    }

    if (c.log_every_n > 0 &&
        (frame_count_ % static_cast<uint64_t>(c.log_every_n)) == 0)
    {
        STREAM_INFO << "[PercepCompute] exp=" << frame_exp_ms_ << "ms gain=" << frame_gain_
             << " blur_t=" << frame_blur_t_
             << " conf=" << frame_confidence_
             << " skip_obstacle=" << frame_skip_obstacle_
             << " drop=" << frame_drop_ ;
    }
}
// 在 MoFrame 转点云逻辑处替换为方案B抽点逻辑
bool PercepMain::dense2sparse_pointcloud(CameraData& camera_data, int sample_step, PointCloudT::Ptr &out_sparse_cloud)
{
    if (sample_step < 1)
    {
        LOG_ERROR("[PercepCompute ERR] disp frame null or invalid sample step");
        return false;
    }
    const int w = camera_data.disp.width;
    const int h = camera_data.disp.height;
    if (w <= 0 || h <= 0 || camera_data.disp_data == nullptr)
    {
        LOG_ERROR("[PercepCompute ERR] disp frame invalid!");
        return false;
    }

    size_t reserve_num = (w / sample_step) * (h / sample_step);
    out_sparse_cloud->clear();
    out_sparse_cloud->reserve(reserve_num); // 预分配内存

    // 曝光自适应：先评估本帧模糊程度，后面用它收紧视差梯度阈值、给点云降权
    evaluateFrameBlur(camera_data);
    if (frame_drop_)
    {
        LOG_ERROR("[PercepCompute ERR] motion blur too high!, drop frame,exp= %f,gain= %f",frame_exp_ms_,frame_gain_);          
        return false;
    }
    // 模糊帧把梯度阈值按 blur 程度收紧：图像糊 → 边缘过渡区更宽 → 拖影更长
    const float blur_grad_scale =
        (exp_guard_cfg_.enabled && exp_guard_cfg_.blur_grad_scale > 0.f)
            ? (1.f - frame_blur_t_ * (1.f - exp_guard_cfg_.blur_grad_scale))
            : 1.f;

    const DispFilterConfig& dfc = disp_filter_cfg_;
    // dfc.enabled=false 时退化为原行为（仅 isfinite + box 裁剪）
    const float min_disp = dfc.enabled ? dfc.min_disp : 1.0f;
    const float max_disp = dfc.enabled ? dfc.max_disp : 144.0f;
    const bool  need_nb  = dfc.enabled && (dfc.grad_check || dfc.speckle_check);

    const uint16_t* disp_16 = reinterpret_cast<const uint16_t*>(camera_data.disp_data);
    const size_t pix_total = static_cast<size_t>(w) * static_cast<size_t>(h);

    // 全分辨率视差(px)，无效像素置 0。梯度/speckle 判据与调试图都基于它。
    // 注意不能像原来那样先 clamp 到 min_disp —— 那会把"无效"抹成"最远"，
    // 既看不出问题，也让梯度判据失效。
    std::vector<float> disp_float(pix_total);
    size_t valid_pix = 0;
    for (size_t i = 0; i < pix_total; i++)
    {
        const float d = static_cast<float>(disp_16[i]) / 256.0f;
        const bool ok = (d >= min_disp && d <= max_disp);
        disp_float[i] = ok ? d : 0.0f;
        if (ok) ++valid_pix;
    }

    // 帧级有效率：整帧有效视差过少说明这一帧本身不可信（暗光/运动模糊/镜头遮挡）。
    // 与其输出一片糊掉的点云去污染栅格图，不如直接丢帧。
    if (dfc.enabled && dfc.min_valid_ratio > 0.f &&
        static_cast<float>(valid_pix) < dfc.min_valid_ratio * static_cast<float>(pix_total))
    {
        LOG_ERROR("[PercepCompute ERR] disp valid ratio too low: %f < min_valid_ratio: %f",(static_cast<float>(valid_pix) / static_cast<float>(pix_total)),dfc.min_valid_ratio);
        return false;
    }

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

    size_t n_cut_range = 0, n_cut_grad = 0, n_cut_speckle = 0, n_cut_box = 0;

    for (int v = 0; v < h; v += sample_step)
    {
        for (int u = 0; u < w; u += sample_step)
        {
            size_t pix_idx = static_cast<size_t>(v) * w + u;
            const float disp = disp_float[pix_idx];

            // 1) 有效视差范围：无效像素在上面已置 0
            if (disp <= 0.0f)
            {
                ++n_cut_range;
                continue;
            }

            // 2) 视差梯度 + 邻域有效点数门控。邻域取自全分辨率视差图，
            //    所以 sample_step 抽稀不会削弱判据。抑制边缘拖影的核心一步。
            if (need_nb)
            {
                const int r  = dfc.grad_radius;
                const int u0 = std::max(0, u - r), u1 = std::min(w - 1, u + r);
                const int v0 = std::max(0, v - r), v1 = std::min(h - 1, v + r);
                // 近处视差大、斜面本身梯度也大，阈值带相对项防止误杀近处地面
                const float grad_th =
                    std::max(dfc.grad_max_px, disp * dfc.grad_max_ratio) *
                    blur_grad_scale;
                int  n_valid_nb = 0;
                bool on_edge    = false;
                for (int vv = v0; vv <= v1 && !on_edge; ++vv)
                {
                    const float* row = &disp_float[static_cast<size_t>(vv) * w];
                    for (int uu = u0; uu <= u1; ++uu)
                    {
                        if (vv == v && uu == u) continue;
                        const float dn = row[uu];
                        if (dn <= 0.0f) continue;   // 无效邻居不参与梯度判断
                        ++n_valid_nb;
                        if (dfc.grad_check && std::fabs(dn - disp) > grad_th)
                        {
                            on_edge = true;         // 落在遮挡边界上 → 拖影来源
                            break;
                        }
                    }
                }
                if (on_edge)
                {
                    ++n_cut_grad;
                    continue;
                }
                if (dfc.speckle_check && n_valid_nb < dfc.min_valid_neighbors)
                {
                    ++n_cut_speckle;                // 孤立噪点
                    continue;
                }
            }

            // 深度 Z 单位 m
            float Z_m = static_cast<float>(bxf)/ disp;
            if (!std::isfinite(Z_m) || Z_m < 0.001f)
                continue;

            // X Y 单位 m
            float X_m = (static_cast<float>(u) - cx) * Z_m / fx;
            float Y_m = (static_cast<float>(v) - cy) * Z_m / fy;
            if (!std::isfinite(X_m) || !std::isfinite(Y_m))
                continue;
            if (Z_m < box_min_z || Z_m > box_max_z 
                || X_m < box_min_x || X_m > box_max_x 
                || Y_m < box_min_y || Y_m > box_max_y
                )
            {
                ++n_cut_box;
                continue;
            }
            // confidence 带上本帧曝光评估的降权结果，供下游按需使用
            out_sparse_cloud->emplace_back(X_m, Y_m, Z_m, 0, 0, 0, -1,
                                           frame_confidence_);
            disp_vec.push_back(disp);
            u_vec.push_back(static_cast<uint32_t>(u));
            v_vec.push_back(static_cast<uint32_t>(v));
        }
    }

    if (dfc.log_every_n > 0 &&
        (frame_count_ % static_cast<uint64_t>(dfc.log_every_n)) == 0)
    {
        STREAM_INFO << "[PercepCompute] kept=" << out_sparse_cloud->size()
             << " cut_range=" << n_cut_range
             << " cut_grad=" << n_cut_grad
             << " cut_speckle=" << n_cut_speckle
             << " cut_box=" << n_cut_box
             << " valid_pix_ratio="
             << (static_cast<float>(valid_pix) / static_cast<float>(pix_total))
             << " exp=" << frame_exp_ms_ << "ms"
             << " blur_t=" << frame_blur_t_
             << " conf=" << frame_confidence_
             << " grad_scale=" << blur_grad_scale;
    }

    if (visualize)
    {
        // 归一化映射到0~255 uint8灰度；被门控判无效的像素为 0(纯黑)，可直接看效果
        cv::Mat disp_mat(h, w, CV_32FC1, disp_float.data());
        cv::Mat disp_gray;
        disp_mat = (disp_mat - min_disp) / (max_disp - min_disp) * 255.0f;
        disp_mat.convertTo(disp_gray, CV_8UC1);

        sensor_msgs::msg::Image disp_img_msg;
        // 封装ROS2 mono8图像消息
        disp_img_msg.header.frame_id = "percep_main";
        disp_img_msg.header.stamp = rclcpp::Clock().now();
        disp_img_msg.width = static_cast<uint32_t>(w);
        disp_img_msg.height = static_cast<uint32_t>(h);
        disp_img_msg.encoding = "mono8";
        disp_img_msg.step = static_cast<uint32_t>(w);
        disp_img_msg.data.assign(disp_gray.data, disp_gray.data + disp_gray.total());
        disp_img_msg.header.frame_id = "percep_main";
        disp_img_pub_.publish(disp_img_msg);           
    }

    if (out_sparse_cloud->empty())
    {
        LOG_ERROR("[PercepCompute ERR] sparse cloud generate empty");
        return false;
    }
    return true;
}


void PercepMain::postprocess(PointCloudT::Ptr& pcl_cloud,
                             const std::vector<uint8_t>& rgb_buf,
                             int img_w, int img_h,float fx,float fy,float cx,float cy)
{  
    if (pcl_cloud->empty()) {LOG_ERROR("[PercepCompute ERR] 后处理前点云为空！");return;}

    if (visualize)
    {
        if (!rgb_buf.empty() && img_w > 0 && img_h > 0) {
            if(!pclCommonPtr->AssignRGBByUV(pcl_cloud, rgb_buf, u_vec,v_vec,img_w, img_h)){LOG_ERROR("[PercepCompute ERR] ProjectRGBFromBuffer failed!");}
        }
       
        this->publish3dTF(cali,pcl_cloud,"percep_main",true);
        this->publish_ros_rgbcloud(pcl_cloud, ori_cloud_pub_, parent_frame);   
    }
    else{
        this->publish3dTF(cali,pcl_cloud,"percep_main",false);
    }
    // auto start = std::chrono::high_resolution_clock::now();
    // this->estimateGrassHeight();
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end- start);
    // LOG_INFO("[PercepCompute] estimateGrassHeight Time taken: %d ms",duration.count());
    this->runGroundSegmentation(pcl_cloud);
}

// 逐帧自适应地面。返回本帧用于 base_link 变换的外参。
//
// 静态外参只能代表"平均安装姿态"，而车体在行驶中俯仰，实测同一组静态值下
// 地面残余倾斜在 0.7~3.0deg 之间跨帧变化。逐帧估计能把这部分吃掉。
//
// 但逐帧估计不能裸用，两道保险：
//   1) 合理性门限 —— 单帧可能把墙面/大件家具当地面(旧 autocali 就这么跑飞的)，
//      偏离静态 cali 超过阈值、残差过大、内点过少，一律丢弃回退静态值；
//   2) EMA 平滑 —— 单帧 pitch 噪声约 ±3deg，直接用会让栅格地图逐帧抖动。
Trans PercepMain::adaptGroundTrans(const PointCloudT& cam_cloud)
{
    if (!gadapt_enable) return cali_default;

    Trans  est;
    double h = 0.0, tilt = 0.0, resid = 0.0;
    size_t inl = 0;
    const bool ok = solveExtrinsicsFromGround(cam_cloud, cali_max_range, est, h, tilt, resid, inl);

    const char* why = nullptr;
    if (!ok)                                                    why = "拟合失败";
    else if (static_cast<int>(inl) < gadapt_min_inliers)        why = "内点不足";
    else if (resid > gadapt_max_resid_cm)                       why = "残差过大";
    // 门限配置是度，Trans 是弧度，比较前统一到弧度
    else if (std::fabs(rad_to_angle(est.roll  - cali.roll))  > gadapt_max_roll_dev)  why = "roll 偏离过大";
    else if (std::fabs(rad_to_angle(est.pitch - cali.pitch)) > gadapt_max_pitch_dev) why = "pitch 偏离过大";
    // else if (std::fabs(rad_to_angle(est.yaw   - cali.yaw))   > gadapt_max_yaw_dev)   why = "yaw 偏离过大";
    else if (std::fabs(h - cali.z) > gadapt_max_h_dev)                               why = "高度偏离过大";

    if (why)
    {
        ++gadapt_rej_;
        if (gadapt_verbose)
            LOG_INFO("[PercepCompute] 拒绝(%s): roll=%.3f pitch=%.3f yaw=%.3f h=%.4f 内点=%zu 残差=%.2fcm"
                   " -> 回退", why, rad_to_angle(est.roll), rad_to_angle(est.pitch),
                   rad_to_angle(est.yaw), h, inl, resid);
        return cali_default;
    }

    ++gadapt_ok_;
    if (!gadapt_inited_)
    {
        gadapt_cur_ = cali;          // 从静态值起步，避免第一帧突跳
        gadapt_inited_ = true;
    }

    const double a = gadapt_ema_alpha;
    gadapt_cur_.roll  = (1.0 - a) * gadapt_cur_.roll  + a * est.roll;
    gadapt_cur_.pitch = (1.0 - a) * gadapt_cur_.pitch + a * est.pitch;
    // gadapt_cur_.yaw   = (1.0 - a) * gadapt_cur_.yaw   + a * est.yaw;
    gadapt_cur_.z     = (1.0 - a) * gadapt_cur_.z     + a * h;
    gadapt_cur_.x = cali.x;          // 前向/横向安装偏移观测不到，保持静态值
    gadapt_cur_.y = cali.y;

    if (gadapt_verbose)
        LOG_INFO("[PercepCompute] 本帧 roll=%.3f pitch=%.3f yaw=%.3f h=%.4f (内点=%zu 残差=%.2fcm)"
               " -> 平滑后 roll=%.3f pitch=%.3f yaw=%.3f z=%.4f  [采纳%zu/拒绝%zu]",
               rad_to_angle(est.roll), rad_to_angle(est.pitch), rad_to_angle(est.yaw),
               h, inl, resid,
               rad_to_angle(gadapt_cur_.roll), rad_to_angle(gadapt_cur_.pitch),
               rad_to_angle(gadapt_cur_.yaw), gadapt_cur_.z,
               gadapt_ok_, gadapt_rej_);
    return gadapt_cur_;
}
// 从地面反解相机安装外参。
//
// 做法：在相机系(X右 Y下 Z前)稳健拟合地面平面，得到向上的法向 up，
// 再用 (前, 左, 上) 三轴直接构造旋转矩阵 R —— base_link 的 X=前 Y=左 Z=上，
// 所以 R 的三个行向量就是这三个轴在相机系下的表示。最后把 R 分解成工程里
// EulerToMatrix 用的 Rz(yaw)·Ry(pitch)·Rx(roll)。
// 这样是精确构造，不做小角度近似，也不需要初值。
bool PercepMain::solveExtrinsicsFromGround(const PointCloudT& cam_cloud,double max_range,
                                               Trans& out,double& out_height,
                                               double& out_tilt_deg,double& out_resid_cm,
                                               size_t& out_inliers)
{
    out_inliers = 0;
    // 1) 近距离点。远处双目噪声大，且容易把墙面/天花板拉进来
    std::vector<Eigen::Vector3d> P;
    P.reserve(cam_cloud.size());
    for (const PointT& p : cam_cloud.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z < 0.3f || p.z > static_cast<float>(max_range)) continue;
        P.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y),
                       static_cast<double>(p.z));
    }
    if (P.size() < 500) { LOG_WARN("[PercepCompute] 近距离点太少(%zu)", P.size()); return false; }

    // 2) 种子：相机系 Y 向下，地面点 Y 偏大，取下半部
    std::vector<double> ys;
    ys.reserve(P.size());
    for (const auto& p : P) ys.push_back(p.y());
    std::nth_element(ys.begin(), ys.begin() + static_cast<long>(ys.size()/2), ys.end());
    const double y_mid = ys[ys.size()/2];
    std::vector<Eigen::Vector3d> sel;
    for (const auto& p : P) if (p.y() > y_mid) sel.push_back(p);
    if (sel.size() < 200) sel = P;

    // 3) 迭代重加权：PCA 拟平面 -> 收内点 -> 再拟
    Eigen::Vector3d nrm(0,-1,0);
    double d = 0.0;
    for (int it = 0; it < 10 && sel.size() >= 100; ++it)
    {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto& p : sel) c += p;
        c /= static_cast<double>(sel.size());
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        for (const auto& p : sel) { const Eigen::Vector3d q = p - c; C += q * q.transpose(); }
        C /= static_cast<double>(sel.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
        nrm = es.eigenvectors().col(0).normalized();   // 最小特征值 -> 平面法向
        d   = -nrm.dot(c);

        std::vector<double> dist;
        dist.reserve(P.size());
        for (const auto& p : P) dist.push_back(std::fabs(nrm.dot(p) + d));
        std::vector<double> sorted = dist;
        std::nth_element(sorted.begin(), sorted.begin() + static_cast<long>(sorted.size()*4/10),
                         sorted.end());
        const double thr = std::max(0.03, sorted[sorted.size()*4/10]);
        std::vector<Eigen::Vector3d> next;
        for (size_t i = 0; i < P.size(); ++i) if (dist[i] < thr) next.push_back(P[i]);
        if (next.size() < 100) break;
        sel.swap(next);
    }
    // 4) 强制法向朝上：相机系 Y 向下 => "上" 的 Y 分量必须为负
    if (nrm.y() > 0.0) { nrm = -nrm; d = -d; }

    out_height = std::fabs(d);
    double s = 0.0; size_t cnt = 0;
    for (const auto& p : P)
    {
        const double e = nrm.dot(p) + d;
        if (std::fabs(e) < 0.05) { s += e * e; ++cnt; }
    }
    out_inliers  = cnt;
    out_resid_cm = (cnt > 1) ? std::sqrt(s / static_cast<double>(cnt)) * 100.0 : 999.0;

    // 5) 构造 base_link 三轴并分解欧拉角
    const Eigen::Vector3d up = nrm;
    Eigen::Vector3d fwd(0,0,1);
    fwd = fwd - fwd.dot(up) * up;
    if (fwd.norm() < 1e-6) return false;
    fwd.normalize();
    const Eigen::Vector3d left = up.cross(fwd).normalized();
    Eigen::Matrix3d R;
    R.row(0) = fwd; R.row(1) = left; R.row(2) = up;

    // Trans 全项目约定用弧度(cali 也是 angle_to_rad 存的)，这里必须跟着用弧度，
    // 否则直接喂给 EulerToMatrix/publish3dTF 会得到完全错误的变换。
    out.pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2,0))));
    out.roll  = std::atan2(R(2,1), R(2,2));
    out.yaw   = std::atan2(R(1,0), R(0,0));
    out.x = cali.x; out.y = cali.y; out.z = out_height;

    // 构造法保证地面在 base_link 下严格水平，残余倾斜的真实校验由 autocali 回代完成
    out_tilt_deg = 0.0;
    return cnt > 200;
}
void PercepMain::publish3dTF(Trans &cali,const PointCloudT::Ptr& pcl_cloud,string frame_id,bool pubTf)
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

bool PercepMain::GdcToRgbRos(const CameraGdcData& camera_data,
                                       sensor_msgs::msg::Image& ros_img,
                                       std::vector<uint8_t>& rgb_buf)
{

    const int w = camera_data.left.width;
    const int h = camera_data.left.height;
    const int stride_y = static_cast<int>(camera_data.left.bytes_used / (h * 1.5f));
    if (!pclCommonPtr->yuv_nv12_to_rgb24(static_cast<const uint8_t*>(camera_data.left_data), w, h, stride_y, &rgb_buf)) {
        LOG_ERROR("[PercepCompute ERR] GdcToRgbRos nv12 convert to rgb24 failed");
        return false;
    }

    ros_img.header.stamp = rclcpp::Time(camera_data.left.time_ns, RCL_ROS_TIME);
    ros_img.width  = static_cast<uint32_t>(w);
    ros_img.height = static_cast<uint32_t>(h);
    ros_img.encoding = "rgb8";
    ros_img.step = static_cast<uint32_t>(w * 3);
    ros_img.data.assign(rgb_buf.begin(), rgb_buf.end());
    return true;
}

bool PercepMain::NV12ToRgbRos(const CameraData& camera_data,
                                       sensor_msgs::msg::Image& ros_img,
                                       std::vector<uint8_t>& rgb_buf)
{

    const int w = camera_data.left.width;
    const int h = camera_data.left.height;
    const int stride_y = static_cast<int>(camera_data.left.bytes_used / (h * 1.5f));
    if (!pclCommonPtr->yuv_nv12_to_rgb24(static_cast<const uint8_t*>(camera_data.left_data), w, h, stride_y, &rgb_buf)) {
        LOG_ERROR("[PercepCompute ERR] NV12ToRgbRos nv12 convert to rgb24 failed!");
        return false;
    }

    ros_img.header.stamp = rclcpp::Time(camera_data.left.time_ns, RCL_ROS_TIME);
    ros_img.width  = static_cast<uint32_t>(w);
    ros_img.height = static_cast<uint32_t>(h);
    ros_img.encoding = "rgb8";
    ros_img.step = static_cast<uint32_t>(w * 3);
    ros_img.data.assign(rgb_buf.begin(), rgb_buf.end());
    return true;
}

void PercepMain::estimateGrassHeight()
{
    if (!last_semantic_cloud_ || last_semantic_cloud_->empty())return;
    constexpr int kGrassLabel = 101;
    // 草坪点最少数量。
    constexpr size_t kMinGrassPoints = 80;
    // EMA 平滑系数,越小越平滑，越大响应越快。
    constexpr float kEmaAlpha = 0.50f;
    // 单帧最大变化量
    constexpr float kMaxStep = 0.02f;   // 2cm
    //取topN高度个点平均值进行候选
    constexpr size_t kTopN = 3000;
    // 候选高度邻域范围
    constexpr float kHeightRange = 0.01f;
    // 候选高度邻域点个数
    constexpr size_t kMinNearbyPoints = 50;
    std::vector<float> grass_heights,grass_x,grass_y;
    grass_heights.reserve(last_semantic_cloud_->size());
    grass_x.reserve(last_semantic_cloud_->size());
    grass_y.reserve(last_semantic_cloud_->size());
    for (const auto& p : last_semantic_cloud_->points)
    {
        if (p.label != kGrassLabel)continue;
        if (!std::isfinite(p.z))continue;
        grass_heights.push_back(p.z);
        grass_x.push_back(p.x);
        grass_y.push_back(p.y);
    }

    const size_t grass_num = grass_heights.size();
    if (grass_num < kMinGrassPoints)
    {
        LOG_INFO("草坪点：%d < kMinGrassPoints: %d",grass_num,kMinGrassPoints);
        return;
    }
    std::sort(grass_heights.begin(),grass_heights.end(),std::greater<float>());
    const size_t top_n = std::min(kTopN, grass_heights.size());

    double valid_height_sum = 0.0;
    size_t valid_height_num = 0;
    PointCloudT::Ptr grass(new PointCloudT);
    for (size_t i = 0; i < top_n; ++i)
    {
        const float h = grass_heights[i];
        size_t nearby_count = 0;
        for (const float z : grass_heights)
        {
            if (std::fabs(z - h) <= kHeightRange)
            {
                ++nearby_count;
            }
        }

        if (nearby_count < kMinNearbyPoints)
        {
            continue;
        }

        valid_height_sum += h;
        ++valid_height_num;
        PointT p;
        p.x = grass_x[i];
        p.y = grass_y[i];
        p.z = h;
        p.r = 0;p.g = 255;p.b = 0;
        grass->points.push_back(p);

    }
    if (valid_height_num == 0)
    {
        LOG_INFO("TopN中没有满足邻域点数要求的高度！");
        return;
    }
    if(visualize)
    {
        this->publish_ros_rgbcloud(grass, grass_cloud_pub_,parent_frame);
    }
    

    float grass_height =static_cast<float>(valid_height_sum / static_cast<double>(valid_height_num));

    // =========================
    // 单帧限幅过滤
    // =========================
    const float diff = grass_height - obs_height_thr;
    if (diff > kMaxStep)
    {
        grass_height = obs_height_thr + kMaxStep;
    }
    else if (diff < -kMaxStep)
    {
        grass_height = obs_height_thr - kMaxStep;
    }
    // =========================
    // EMA 平滑
    // =========================
    // obs_height_thr =(1.0f - kEmaAlpha) * obs_height_thr +kEmaAlpha * grass_height;
    obs_height_thr = grass_height;
    patchwork_->updateDist(obs_height_thr);
    LOG_INFO(
        "[PercepCompute] GrassHeight: "
        "grass=%zu valid_height_num=%zu "
        "top_max=%.4f top_min=%.4f grass_height=%.4f "
        "obs_height_thr=%.4f",
        grass_num,
        valid_height_num,
        grass_heights.front(),
        grass_heights[top_n - 1],
        grass_height,
        obs_height_thr);
}
void PercepMain::runGroundSegmentation(const PointCloudT::Ptr& raw_pts) 
{
    if (!patchwork_ || raw_pts->empty()) {LOG_ERROR("[PercepCompute ERR] 点云为空，地面分割失败！");return;}
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

    PointCloudT::Ptr ground_cloud_extra = pclCommonPtr->passThrough_filter(*no_ground_cloud,"z",obs_height_thr,100); 
    *ground_cloud = *ground_cloud + *ground_cloud_extra;

    auto t_pw_end = std::chrono::high_resolution_clock::now();
    auto pw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_pw_end - t_pw);
    LOG_INFO("[PercepCompute] patchwork Time taken: %d ms",pw_ms.count());
    LOG_INFO("[PercepCompute] 非地面点云数量：%d,地面点云数量：%d",no_ground_cloud->points.size(),ground_cloud->points.size());
    if (visualize)
    {
        this->publish_ros_cloud(ground_cloud, ground_cloud_pub_,parent_frame);
        this->publish_ros_cloud(no_ground_cloud, nonground_cloud_pub_,parent_frame);
    }

     
    if(no_ground_cloud->size()>3000)
    {
        LOG_INFO("[PercepCompute]非地面点云数量过多，触发降采样!");
        pclCommonPtr->DownSample(no_ground_cloud,0.04,"uniform");
    }

    if (ramp_detect)
    {
        auto start = std::chrono::high_resolution_clock::now();
        RampRecovery::Result recovery_result;
        recoveryPtr->process(no_ground_cloud,ground_cloud,recovery_result);
        // PointCloudT::Ptr step_cloud = recovery_result.step; 
        ramp_cloud = recovery_result.ramp;
        if (visualize)
        {
            // for (auto &p: *step_cloud) {p.r = 255;p.g = 255;p.b = 0;}
            for (auto &p: *ramp_cloud) {p.r = 0;p.g = 0;p.b = 255;}
            // this->publish_ros_rgbcloud(step_cloud, step_cloud_pub_,parent_frame);
            this->publish_ros_rgbcloud(ramp_cloud, ramp_cloud_pub_,parent_frame);
        }
        no_ground_cloud =recovery_result.obstacle;
        if (recovery_result.slope_angle<5.0 && recovery_result.slope_angle>=0)
        {
            *ground_cloud +=*ramp_cloud; 
            ramp_cloud.reset(new PointCloudT);
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
        LOG_INFO("[PercepCompute] ramp_detect all Time taken: %d ms",duration.count());
    }

    auto t_filter = std::chrono::high_resolution_clock::now();   
    pclCommonPtr->StatisticalOutlierRemoval(no_ground_cloud,20,1.0);   
    auto t_filter_end = std::chrono::high_resolution_clock::now();
    auto filter_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_filter_end- t_filter);    
    LOG_INFO("[PercepCompute] filter Time taken: %d ms",filter_ms.count());

    obs_cloud = no_ground_cloud;
    if (visualize)
    {
        for (auto &p: *obs_cloud) {p.r = 255;p.g = 0;p.b = 0;}
        this->publish_ros_rgbcloud(obs_cloud, obs_cloud_pub_,parent_frame);
      
    }
    if (frame_skip_obstacle_)
    {
        // 运动模糊帧：非地面点的深度不可信，输出障碍容易误刹/误绕。
        obs_cloud.reset(new PointCloudT);
        LOG_WARN("[PercepCompute]运动模糊，跳过此帧,本帧曝光：%f ms,frame_blur_t_:%f",frame_exp_ms_,frame_blur_t_);
    }
}   

void PercepMain::autocali(PointCloudT::Ptr &pcl_cloud)
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
    Eigen::Matrix4f trans_cali = Eigen::Matrix4f::Identity();
    pclCommonPtr->EulerToMatrix(default_trans, trans_cali);
    pcl::transformPointCloud(ori_cloud,ori_cloud,trans_cali);
    cali_count++;
    if (cali_count>5)
    {
        string extrinsics_config_dir = "/app/daye/config/ai_node/extrinsics.json";
        std::ifstream ifs(extrinsics_config_dir, std::ios::binary);
        if (!ifs.is_open()) {
            LOG_ERROR("[PercepCompute ERR] 打开文件失败: %s",extrinsics_config_dir);
            return ;
        }
        std::stringstream ss;
        ss << ifs.rdbuf();
        string config = ss.str();
        ifs.close();  

        rapidjson::Document ai_node_json;
        ai_node_json.Parse(config.c_str());
        if (ai_node_json.HasParseError()){
            LOG_ERROR("[PercepCompute ERR] json解析失败: ");
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
            LOG_ERROR("[PercepCompute ERR] 写配置文件失败,路径：%s",extrinsics_config_dir.c_str());
            return ;
        }
        ofs << buffer.GetString();
        ofs.close();
        LOG_INFO("[PercepCompute]标定参数已写入文件: Roll:%f,Pitch:%f,z:%f",avg_roll,avg_pitch,avg_h);        
        cali_count = 0;sum_roll = 0;sum_pitch = 0; sum_h = 0;
        return;
    }

}

}  // namespace perception
