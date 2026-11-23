/**
 * @file percep_init.cpp
 * @brief 感知模块初始化与校验
 */
#include "percep_main/percep_main.h"


namespace perception {
namespace{
    double readJsonDouble(const rapidjson::Value& obj, const char* key, double default_val) {
    if (!obj.HasMember(key)) return default_val;
    if (obj[key].IsDouble()) return obj[key].GetDouble();
    if (obj[key].IsInt())    return static_cast<double>(obj[key].GetInt());
    return default_val;
    }

    uint32_t readJsonUInt(const rapidjson::Value& obj, const char* key, uint32_t default_val) {
        if (!obj.HasMember(key)) return default_val;
        if (obj[key].IsUint())  return obj[key].GetUint();
        if (obj[key].IsInt() && obj[key].GetInt() >= 0) return static_cast<uint32_t>(obj[key].GetInt());
        return default_val;
    }

    bool readJsonBool(const rapidjson::Value& obj, const char* key,
                                bool default_val) {
        if (!obj.HasMember(key) || !obj[key].IsBool()) return default_val;
        return obj[key].GetBool();
    }
}
bool PercepMain::initInterface()
{

    if (fs::exists(output_dir)&& fs::is_directory(output_dir))
    {
      fs::remove_all(output_dir);
      LOG_INFO("[PercepInit]日志数据目录初始化完毕！");
    }

    ground_cloud.reset(new PointCloudT);
    no_ground_cloud.reset(new PointCloudT);
    ramp_cloud.reset(new PointCloudT);
    obs_cloud.reset(new PointCloudT);
    last_semantic_cloud_.reset(new PointCloudT);
    offline_sparse_cloud.reset(new PointCloudT);
    frame_rgb_buf_.reserve(rgb_width*rgb_height*3);
    gdc_forsave = std::make_unique<CameraGdcData>();

    pclCommonPtr.reset(new PclCommon);
    aiFusionPtr.reset(new AiFusion(mapWidth,mapHeight,mapResolution,0.05,5,6,cali.x,cali.y,box_min_z));
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(daye::getHandle());
    visualizePtr.reset(new Visualize(mapWidth,mapHeight,mapResolution,output_dir,cali.x,cali.y));
    recoveryPtr.reset(new RampRecovery()); 
    recoveryPtr->setVisualize(visualize);
//-----------------sdk interface-------------------//      
    if (!offline) 
    {
        recharge_ = Aiper_Recharge::createRechargeMarkHost();
        auto qos_cam = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        cam_params_sub_ = daye::CreateSubscribe<common_interface::msg::CameraParam>(kCameraParam, qos_cam,bind(&PercepMain::onCamParams, this, placeholders::_1));
        const daye::ChannelConfig config{kCameraFrameShm, kFrameShmSize};
        subCamera = daye::ShmSubscriber(config);
        subCamera.registerCallback(std::bind(&PercepMain::onCameraFrame, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3),100);
        const daye::ChannelConfig gdc_config{kCameraGdcFrameShm, kGdcFrameShmSize};
        subCameraGdc = daye::ShmSubscriber(gdc_config);
        subCameraGdc.registerCallback(std::bind(&PercepMain::onGdcFrame, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3),100);

        algThreadHandle = thread(&PercepMain::algThread, this);
    }else{
        LOG_WARN("[PercepMain] 进入离线测试模式!");
        offlineThreadHandle = thread(&PercepMain::offline_loop, this);

    }

    healthCodeHandle = thread(&PercepMain::healthCodeThread, this);
    if (save_log)
    {
        log_ok.store(true);
        saveLogThreadHandle = std::thread(&PercepMain::saveLogLoop, this);
    }else{
        RCLCPP_WARN(rclcpp::get_logger("warn"), "本次程序感知日志未启用！");
        LOG_WARN("本次程序感知日志未启用！");
        healthCodeStatus.LogWarn = true;
        log_ok.store(false);
    }
  //-----------------ros topic-------------------//
    //感知地图接口
    grid_map_pub_ = daye::CreatePublish<common_interface::msg::GridMapInfo>(kGridMapInfo, rclcpp::SensorDataQoS());

    //可视化接口
    ori_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/pcl_cloud", rclcpp::QoS(5));   
    pclmap_pub_ =  daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_main/pcl_map", rclcpp::QoS(5));
    aimap_pub_ =  daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_main/ai_map", rclcpp::QoS(5));
    fusionmap_pub_ =  daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_main/fusion_map", rclcpp::QoS(5));
    rgb_img_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>("/percep_main/rgb_frame", rclcpp::QoS(5));//Reliable
    disp_img_pub_ = daye::CreatePublish<sensor_msgs::msg::Image>("/percep_main/disp_frame", rclcpp::QoS(5));
    ground_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/ground_cloud", rclcpp::QoS(5));
    nonground_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/nonground_cloud", rclcpp::QoS(5));
    obs_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/obs_cloud", rclcpp::QoS(5));
    ramp_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/ramp_cloud", rclcpp::QoS(5));
    step_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/step_cloud", rclcpp::QoS(5));
    grass_cloud_pub_ = daye::CreatePublish<sensor_msgs::msg::PointCloud2>("/percep_main/grass_cloud", rclcpp::QoS(5));
    if (recharge_) {
        if (recharge_->Init(sdk_so_dir.c_str(), nullptr,&daye::getHandle()) != 0) {
            LOG_ERROR("[PercepInit ERR] recharge_mark Init failed path=%s",recharge_->configPath());
            return false;
        }
        // 回充结果图与主仓日志统一受 debug.save_log 控制。
        recharge_->setSaveResultEnabled(save_log);
    }

    this->initDnnRuntime();

    return true;
}

bool PercepMain::loadExtrinsicsConfig(rapidjson::Document &extrinsics_json)
{
    string extrinsics_config_dir = "/app/daye/config/ai_node/extrinsics.json";
    std::ifstream ifs_(extrinsics_config_dir, std::ios::binary);
    if (!ifs_.is_open()) {
        LOG_ERROR("[PercepInit ERR] 打开标定文件失败: %s",extrinsics_config_dir.c_str());
        
        return false;
    }
    std::stringstream ss_;
    ss_ << ifs_.rdbuf();
    string config = ss_.str();
    ifs_.close();  

    extrinsics_json.Parse(config.c_str());
    if (extrinsics_json.HasParseError()){
        LOG_ERROR("[PercepInit ERR] 外参json解析失败: %s",extrinsics_config_dir.c_str());
        return false;
    }     
    return true;
}
bool PercepMain::initExtrinsicsParams(rapidjson::Document &extrinsics_json)
{
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
    //外参校验
    if (cali.x<=0 || cali.x >1 || cali.y>1 || cali.y<-1 || cali.z<=0 || cali.z> 0.3
        || rad_to_angle(cali.roll)<-105 || rad_to_angle(cali.roll)> -90 || rad_to_angle(cali.pitch) <-5 
        || rad_to_angle(cali.pitch)>5 || rad_to_angle(cali.yaw) < -95 || rad_to_angle(cali.yaw) > -85)
    {
        return false;
    }
    cali_default = cali;
    return true;
}
bool PercepMain::loadStartConfig(rapidjson::Document &start_json)
{

    string start_config_dir = "/app/daye/config/ai_node/start_config.json";
    std::ifstream ifs(start_config_dir, std::ios::binary);
    if (!ifs.is_open()) {
        LOG_ERROR("[PercepInit ERR] 打开配置文件失败: %s",start_config_dir.c_str());
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    string start_config = ss.str();
    ifs.close();  
    start_json.Parse(start_config.c_str());
    if (start_json.HasParseError()){
      LOG_ERROR("[PercepInit ERR] 配置文件json解析失败: %s",start_config_dir.c_str());
      return false;
    } 
    if (!start_json.IsObject()) {
        LOG_ERROR("[PercepInit ERR] 配置文件不是JSON对象！");
        return false; 
    }
    return true;
}
bool PercepMain::initStartConfig(rapidjson::Document &ai_node_json)
{
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
        if (pd.HasMember("save_interval_ms") && pd["save_interval_ms"].IsInt()) {
            save_interval_ms = std::max(0, pd["save_interval_ms"].GetInt());
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
    if (ai_node_json.HasMember("ramp_detect") && ai_node_json["ramp_detect"].IsBool()) {
        ramp_detect = ai_node_json["ramp_detect"].GetBool();
    }
    else{
      return false;
    }

    if (ai_node_json.HasMember("offline") && ai_node_json["offline"].IsObject()) {
        const rapidjson::Value& pd = ai_node_json["offline"];
        if (pd.HasMember("enable") && pd["enable"].IsBool()) {
            offline = pd["enable"].GetBool();
        }
        if (pd.HasMember("type") && pd["type"].IsString()) {
            type = pd["type"].GetString();
        }
        if (pd.HasMember("auto_loop") && pd["auto_loop"].IsBool()) {
            auto_loop = pd["auto_loop"].GetBool();
        }
        if (pd.HasMember("interactive") && pd["interactive"].IsBool()) {
            interactive = pd["interactive"].GetBool();
        }
        if (pd.HasMember("camparams_dir") && pd["camparams_dir"].IsString()) {
            camparams_dir = pd["camparams_dir"].GetString();
        }
        if (pd.HasMember("disp_dir") && pd["disp_dir"].IsString()) {
            disp_dir = pd["disp_dir"].GetString();
        }
        if (pd.HasMember("rgb_dir") && pd["rgb_dir"].IsString()) {
            rgb_dir = pd["rgb_dir"].GetString();
        }
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
    if (ai_node_json.HasMember("obs_height_thr") && ai_node_json["obs_height_thr"].IsDouble()) {
        obs_height_thr = ai_node_json["obs_height_thr"].GetDouble();
    }else{
        return false;
    }
    patchwork::Params pw_params;
    pw_params.verbose       = false;
    pw_params.sensor_height = cali.z;
    pw_params.min_range     = 0.1;
    pw_params.max_range     = 3.0;
    pw_params.th_dist       = obs_height_thr;
    pw_params.th_seeds      = 0.025;
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

    if (ai_node_json.HasMember("interestFov") &&
        ai_node_json["interestFov"].IsObject()) {
        const rapidjson::Value& pp = ai_node_json["interestFov"];
        box_min_x = static_cast<float>(
            readJsonDouble(pp, "box_min_x", box_min_x));
        box_max_x = static_cast<float>(
            readJsonDouble(pp, "box_max_x", box_max_x));
        box_min_y = static_cast<float>(
            readJsonDouble(pp, "box_min_y", box_min_y));
        box_max_y = static_cast<float>(
            readJsonDouble(pp, "box_max_y", box_max_y));
        box_min_z = static_cast<float>(
            readJsonDouble(pp, "box_min_z", box_min_z));
        box_max_z = static_cast<float>(
            readJsonDouble(pp, "box_max_z", box_max_z));
    }else{
        return false;
    }

    if (ai_node_json.HasMember("disp_filter") &&
        ai_node_json["disp_filter"].IsObject()) {
        const rapidjson::Value& df = ai_node_json["disp_filter"];
        DispFilterConfig& c = disp_filter_cfg_;
        c.enabled     = readJsonBool(df, "enabled", c.enabled);
        c.min_disp    = static_cast<float>(readJsonDouble(df, "min_disp", c.min_disp));
        c.max_disp    = static_cast<float>(readJsonDouble(df, "max_disp", c.max_disp));
        c.grad_check  = readJsonBool(df, "grad_check", c.grad_check);
        c.grad_radius = static_cast<int>(readJsonDouble(df, "grad_radius", c.grad_radius));
        c.grad_max_px = static_cast<float>(readJsonDouble(df, "grad_max_px", c.grad_max_px));
        c.grad_max_ratio =
            static_cast<float>(readJsonDouble(df, "grad_max_ratio", c.grad_max_ratio));
        c.speckle_check = readJsonBool(df, "speckle_check", c.speckle_check);
        c.min_valid_neighbors = static_cast<int>(
            readJsonDouble(df, "min_valid_neighbors", c.min_valid_neighbors));
        c.min_valid_ratio =
            static_cast<float>(readJsonDouble(df, "min_valid_ratio", c.min_valid_ratio));
        c.log_every_n = static_cast<int>(readJsonDouble(df, "log_every_n", c.log_every_n));
        if (c.grad_radius < 1) c.grad_radius = 1;
        if (c.min_disp < 0.f) c.min_disp = 0.f;
    }else{
        return false;
    }

    if (ai_node_json.HasMember("exposure_guard") &&
        ai_node_json["exposure_guard"].IsObject()) {
        const rapidjson::Value& eg = ai_node_json["exposure_guard"];
        ExposureGuardConfig& c = exp_guard_cfg_;
        c.enabled     = readJsonBool(eg, "enabled", c.enabled);
        c.exp_soft_ms = static_cast<float>(readJsonDouble(eg, "exp_soft_ms", c.exp_soft_ms));
        c.exp_hard_ms = static_cast<float>(readJsonDouble(eg, "exp_hard_ms", c.exp_hard_ms));
        c.gain_soft   = static_cast<float>(readJsonDouble(eg, "gain_soft", c.gain_soft));
        c.gain_hard   = static_cast<float>(readJsonDouble(eg, "gain_hard", c.gain_hard));
        c.min_confidence =
            static_cast<float>(readJsonDouble(eg, "min_confidence", c.min_confidence));
        if (eg.HasMember("hard_action") && eg["hard_action"].IsString()) {
            c.hard_action = eg["hard_action"].GetString();
        }
        c.skip_ai_obstacle = readJsonBool(eg, "skip_ai_obstacle", c.skip_ai_obstacle);
        c.blur_grad_scale =
            static_cast<float>(readJsonDouble(eg, "blur_grad_scale", c.blur_grad_scale));
        c.log_every_n = static_cast<int>(readJsonDouble(eg, "log_every_n", c.log_every_n));
        if (c.hard_action != "none" && c.hard_action != "skip_obstacle" &&
            c.hard_action != "drop_frame") {
            STREAM_INFO << "[PercepInit] unknown exposure_guard.hard_action='"
                 << c.hard_action << "', fallback to skip_obstacle" << endl;
            c.hard_action = "skip_obstacle";
        }
    }else{
        return false;
    }
    STREAM_INFO << "[PercepInit] interestFov  box_z=[" << box_min_z 
       << "," << box_max_z<< "] box_xy=[" << box_min_x << "," 
       << box_max_x<< "]" << endl;
    // STREAM_INFO << "[PercepInit] exposure_guard enabled=" << exp_guard_cfg_.enabled
    //      << " exp_ms=[" << exp_guard_cfg_.exp_soft_ms
    //      << "," << exp_guard_cfg_.exp_hard_ms << "]"
    //      << " gain=[" << exp_guard_cfg_.gain_soft
    //      << "," << exp_guard_cfg_.gain_hard << "]"
    //      << " min_conf=" << exp_guard_cfg_.min_confidence
    //      << " hard_action=" << exp_guard_cfg_.hard_action
    //      << " skip_ai=" << exp_guard_cfg_.skip_ai_obstacle
    //      << " blur_grad_scale=" << exp_guard_cfg_.blur_grad_scale << endl;
    // STREAM_INFO << "[PercepInit] disp_filter enabled=" << disp_filter_cfg_.enabled
    //      << " disp=[" << disp_filter_cfg_.min_disp
    //      << "," << disp_filter_cfg_.max_disp << "]"
    //      << " grad=" << disp_filter_cfg_.grad_check
    //      << "(r=" << disp_filter_cfg_.grad_radius
    //      << ",px=" << disp_filter_cfg_.grad_max_px
    //      << ",ratio=" << disp_filter_cfg_.grad_max_ratio << ")"
    //      << " speckle=" << disp_filter_cfg_.speckle_check
    //      << "(minNb=" << disp_filter_cfg_.min_valid_neighbors << ")"
    //      << " min_valid_ratio=" << disp_filter_cfg_.min_valid_ratio << endl;

    if (ai_node_json.HasMember("map") &&
    ai_node_json["map"].IsObject()) {
        const rapidjson::Value& pp = ai_node_json["map"]; 
        mapWidth = readJsonDouble(pp, "MapWidth", mapWidth);
        mapHeight = readJsonDouble(pp, "MapHeight", mapHeight);
        mapResolution = readJsonDouble(pp, "MapResolution", mapResolution);
    }else{
        return false;
    }

    if (!this->parseDnnConfig(ai_node_json)) {
        LOG_ERROR("[PercepInit ERR] parseDnnConfig failed!");
        return false;
    }
    this->parseLabelFusionConfig(ai_node_json);

    STREAM_INFO << "[PercepInit] dnn cfg enabled=" << (dnn_cfg_.enabled ? "on" : "off")
         << " detect=" << (dnn_cfg_.detect.enabled ? "on" : "off")
         << " model=" << dnn_cfg_.detect.model_path << endl;

    return true;
}
}  // namespace perception
