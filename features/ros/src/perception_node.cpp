#include "perception_ros_so_demo/perception_node.hpp"
#include "perception_ros_so_demo/json_text.hpp"
#include "perception_ros_so_demo/image_messages.hpp"
#include "intrinsics_file.hpp"

namespace ros_so_demo {

PerceptionNode::PerceptionNode():Node("perception_ros_so_demo") {
        const auto config=declare_parameter<std::string>("sdk_config","");
        const auto det=declare_parameter<std::string>("detection_model","");
        const auto seg=declare_parameter<std::string>("segmentation_model","");
        const auto mode=declare_parameter<std::string>("mode","infer");
        if(config.empty())throw std::invalid_argument("Set sdk_config to an absolute generated full-SDK JSON path");
        if(mode!="infer"&&mode!="map"&&mode!="both"&&mode!="recharge")throw std::invalid_argument("mode: infer/map/both/recharge");
        mapping_=declare_parameter<bool>("mapping_enabled",mode=="map"||mode=="both");
        recharge_=mode=="recharge"||mode=="both";
        input_mode_=declare_parameter<std::string>("input_mode","ros");
        if(input_mode_!="offline"&&input_mode_!="camera"&&input_mode_!="ros")throw std::invalid_argument("input_mode: offline/camera/ros");
        const auto disparity_source=declare_parameter<std::string>("disparity_source","original_q8");
        if(disparity_source!="original_q8"&&disparity_source!="raft_stereo")throw std::invalid_argument("disparity_source: original_q8/raft_stereo");
        new_stereo_=disparity_source=="raft_stereo";
        disparity_=declare_parameter<bool>("disparity_enabled",mapping_||new_stereo_);
        if(mapping_&&!disparity_)throw std::invalid_argument("mapping_enabled requires disparity_enabled");
        if(new_stereo_&&!disparity_)throw std::invalid_argument("raft_stereo requires disparity_enabled");
        use_camera_info_=declare_parameter<bool>("use_camera_info",false);
        if(use_camera_info_&&input_mode_!="ros")throw std::invalid_argument("use_camera_info requires input_mode=ros");
        frame_name_=declare_parameter<std::string>("frame_id","camera_optical_frame");
        if(frame_name_.empty())throw std::invalid_argument("frame_id must not be empty");
        if(input_mode_=="offline")configureOffline();
        const auto capacity=declare_parameter<int>("pair_capacity",4);
        if(capacity<1||capacity>16)throw std::invalid_argument("pair_capacity must be1..16");pair_capacity_=std::size_t(capacity);
        publish_enabled_=declare_parameter<bool>("publish_enabled",true);
        const bool record_enabled=declare_parameter<bool>("record_enabled",true);
        metrics_enabled_=declare_parameter<bool>("metrics_enabled",true);
        publish_every_=declare_parameter<int>("publish_every_n",1);save_every_=declare_parameter<int>("save_every_n",0);
        if(!record_enabled)save_every_=0;
        if(publish_every_<1||save_every_<0)throw std::invalid_argument("Invalid publication/save interval");
        save_root_=declare_parameter<std::string>("save_root","");
        if(save_every_&&save_root_.empty())throw std::invalid_argument("Set save_root when save_every_n>0");
        publish_maps_=declare_parameter<bool>("publish_maps",false);publish_mask_=declare_parameter<bool>("publish_mask",true);
        publish_input_=declare_parameter<bool>("publish_input",false);
        publish_clouds_=declare_parameter<bool>("publish_clouds",false);
        publish_disparity_=declare_parameter<bool>("publish_disparity",true);
        save_clouds_ply_=declare_parameter<bool>("save_clouds_ply",false)&&record_enabled;
        if(save_clouds_ply_&&!save_every_)throw std::invalid_argument("save_clouds_ply requires save_every_n>0 and save_root");
        robot_frame_=declare_parameter<std::string>("robot_frame_id","base_link");
        if(robot_frame_.empty())throw std::invalid_argument("robot_frame_id must not be empty");
        const auto rviz_config=declare_parameter<std::string>("rviz_config","");
#ifdef R36_ROS_WITH_RVIZ
        if(!rviz_config.empty())rviz_stream_=std::make_unique<perception::rviz::SdkPublisher>(fs::u8path(rviz_config));
#else
        if(!rviz_config.empty())throw std::invalid_argument("Rebuild the ROS application with R36_BUILD_RVIZ_STREAM=ON");
#endif
        const auto odom_topic=declare_parameter<std::string>("wheel_odom_topic","");
        motion_wait_timeout_ms_=declare_parameter<int>("wheel_odom_wait_timeout_ms",50);
        if(motion_wait_timeout_ms_<0||motion_wait_timeout_ms_>2000)throw std::invalid_argument("wheel_odom_wait_timeout_ms must be 0..2000");
        if(!odom_topic.empty())motion_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic,rclcpp::SensorDataQoS().keep_last(100),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr message){wheelOdometry(std::move(message));});
        slip_pub_=create_publisher<Text>("~/slip",1);
        // Calibration has an explicit source. Camera values are filled from the
        // SDK below; CameraInfo values are attached to each synchronized frame.
        intrinsics_={640,544,declare_parameter<double>("fx",0),declare_parameter<double>("fy",0),
            declare_parameter<double>("cx",0),declare_parameter<double>("cy",0),declare_parameter<double>("bxf",0)};
        const auto intrinsic_file=declare_parameter<std::string>("intrinsics_file","");
        if((mapping_||disparity_)&&input_mode_=="offline") {
            if(intrinsic_file.empty())throw std::invalid_argument("Offline geometry requires intrinsics_file; configure features/offline/config.json");
            const auto path=fs::absolute(fs::u8path(intrinsic_file)).lexically_normal();
            const auto k=v3::loadIntrinsicsFile(path,true);
            const auto value=[&](const char* key){return r33::json::number(r33::json::required(k,key));};
            intrinsics_={640,544,value("fx"),value("fy"),value("cx"),value("cy"),value("bxf")};
            RCLCPP_INFO(get_logger(),"Offline rectified calibration: %s",path.u8string().c_str());
        }
        frame_calibration_required_=mapping_||disparity_||(recharge_&&input_mode_=="ros"&&
            v3::rechargeUsesFrameIntrinsics(fs::absolute(fs::u8path(config))));
        const bool explicit_calibration=frame_calibration_required_&&input_mode_=="ros"&&!use_camera_info_;
        if(explicit_calibration&&(!std::isfinite(intrinsics_.fx)||!std::isfinite(intrinsics_.fy)||!std::isfinite(intrinsics_.bxf)||
           !std::isfinite(intrinsics_.cx)||!std::isfinite(intrinsics_.cy)||intrinsics_.fx<=0||intrinsics_.fy<=0||intrinsics_.bxf<=0||
           intrinsics_.cx<0||intrinsics_.cx>=640||intrinsics_.cy<0||intrinsics_.cy>=544))
            throw std::invalid_argument("Provide verified rectified 640x544 fx/fy/cx/cy/bxf (bxf in px*mm), or enable ROS CameraInfo");
        if(new_stereo_) {
            if(!declare_parameter<bool>("stereo_rectified",input_mode_=="camera"))
                throw std::invalid_argument("Set stereo_rectified=true only for calibrated, rectified left/right input");
            if(!declare_parameter<bool>("calibration_confirmed",input_mode_=="camera"))
                throw std::invalid_argument("New Stereo requires calibration_confirmed=true and genuine rectified intrinsics");
            ros_so_demo::StereoSettings settings;
            settings.model=declare_parameter<std::string>("stereo_model","");
            settings.input_mode=declare_parameter<std::string>("stereo_input_mode","resize");
            const int sw=declare_parameter<int>("stereo_input_width",640),sh=declare_parameter<int>("stereo_input_height",352);
            if(sw<0||sh<0||((sw==0)!=(sh==0))||(sw&1)||(sh&1))
                throw std::invalid_argument("Stereo dimensions must both be zero for metadata, or both positive even values");
            settings.input_width=unsigned(sw);settings.input_height=unsigned(sh);
            stereo_=std::make_unique<ros_so_demo::StereoProvider>(settings);
        }
        result_pub_=create_publisher<Text>("~/result",1);history_pub_=create_publisher<Text>("~/history",1);
        metrics_pub_=create_publisher<Text>("~/metrics",1);
        const auto metrics_every=declare_parameter<int>("metrics_every_n",1);
        if(metrics_every<1)throw std::invalid_argument("metrics_every_n must be positive");metrics_every_=uint64_t(metrics_every);
        const auto metrics_path=declare_parameter<std::string>("metrics_path","");
        if(metrics_enabled_&&!metrics_path.empty()) {
            const auto path=fs::absolute(fs::u8path(metrics_path));
            if(!path.parent_path().empty())fs::create_directories(path.parent_path());
            metrics_file_.open(path,std::ios::binary|std::ios::app);
            if(!metrics_file_)throw std::runtime_error("Cannot open metrics_path: "+metrics_path);
        }
        error_pub_=create_publisher<Text>("~/errors",10);code_pub_=create_publisher<Code>("~/error_code",10);
        mask_pub_=create_publisher<Image>("~/mask",rclcpp::SensorDataQoS().keep_last(1));
        disparity_pub_=create_publisher<Image>("~/disparity_q8",rclcpp::SensorDataQoS().keep_last(1));
        input_pub_=create_publisher<Image>("~/input_image",rclcpp::QoS(4).reliable());
        pcl_pub_=create_publisher<Image>("~/pcl_map",rclcpp::SensorDataQoS().keep_last(1));
        ai_pub_=create_publisher<Image>("~/ai_map",rclcpp::SensorDataQoS().keep_last(1));
        fusion_pub_=create_publisher<Image>("~/fusion_map",rclcpp::SensorDataQoS().keep_last(1));
        if(publish_clouds_)for(const auto* name:{"camera","robot","ground","obstacle","semantic"})
            cloud_pubs_.emplace(name,create_publisher<PointCloud>(std::string("~/clouds/")+name,rclcpp::SensorDataQoS().keep_last(1)));
        auto status=ros_so_demo::newStatus();
        if(algorithm_.open(config.c_str(),status,det.empty()?nullptr:det.c_str(),seg.empty()?nullptr:seg.c_str(),stereo_?stereo_->hooks():nullptr)){
            report(status);throw std::runtime_error("SO initialization failed: "+std::string(status.detail));}
        const int command=mode=="both"?3:mode=="recharge"?2:1;
        if(algorithm_.command(command,status)){report(status);throw std::runtime_error(status.detail);}
        if(input_mode_=="ros") {
            const uint64_t required=(1u<<Left)|(new_stereo_?(1u<<Right):0u)|
                (disparity_&&!new_stereo_?(1u<<Depth):0u)|(recharge_?(1u<<Hires):0u)|
                (use_camera_info_?((1u<<LeftInfo)|(1u<<RightInfo)):0u);
            synchronizer_=std::make_unique<ros_so_demo::ExactFrames<Payload,6>>(required,pair_capacity_);
            auto qos=rclcpp::SensorDataQoS().keep_last(capacity);
            const auto reliability=declare_parameter<std::string>("input_reliability","best_effort");
            if(reliability=="reliable")qos.reliable();
            else if(reliability!="best_effort")throw std::invalid_argument("input_reliability must be best_effort or reliable");
            left_sub_=create_subscription<Image>(declare_parameter<std::string>("left_topic","/camera/rgbd_left_nv12"),qos,
                [this](Image::ConstSharedPtr m){input(Left,std::move(m));});
            if(new_stereo_)right_sub_=create_subscription<Image>(declare_parameter<std::string>("right_topic","/camera/rgbd_right_nv12"),qos,
                [this](Image::ConstSharedPtr m){input(Right,std::move(m));});
            if(disparity_&&!new_stereo_)q8_sub_=create_subscription<Image>(declare_parameter<std::string>("q8_topic","/camera/disparity_q8"),qos,
                [this](Image::ConstSharedPtr m){input(Depth,std::move(m));});
            if(recharge_)gdc_sub_=create_subscription<Image>(declare_parameter<std::string>("gdc_topic","/camera/gdc_left_nv12"),qos,
                [this](Image::ConstSharedPtr m){input(Hires,std::move(m));});
            if(use_camera_info_) {
                left_info_sub_=create_subscription<CameraInfo>(declare_parameter<std::string>("left_camera_info_topic","/stereo/left/camera_info"),qos,
                    [this](CameraInfo::ConstSharedPtr m){cameraInfo(LeftInfo,std::move(m));});
                right_info_sub_=create_subscription<CameraInfo>(declare_parameter<std::string>("right_camera_info_topic","/stereo/right/camera_info"),qos,
                    [this](CameraInfo::ConstSharedPtr m){cameraInfo(RightInfo,std::move(m));});
            }
        } else if(input_mode_=="camera") {
            raw::CameraOptions options;options.need_q8=disparity_&&!new_stereo_;options.need_gdc=recharge_;
            options.disparity_mode=declare_parameter<int>("camera_disparity_mode",new_stereo_?1:2);
            options.sdk=declare_parameter<std::string>("camera_sdk",options.sdk);
            options.config=declare_parameter<std::string>("camera_config",options.config);
            options.timeout_ms=declare_parameter<int>("acquire_timeout_ms",3000);
            options.retry_ms=declare_parameter<int>("retry_interval_ms",5);
            camera_=std::make_unique<raw::Camera>(options);
            const auto p=camera_->parameters();
            if(!((p.width==640&&p.height==544)||(p.width==1280&&p.height==1088)))
                throw std::runtime_error("SDK rectified reference geometry is unverified; expected 640x544 or 1280x1088");
            const auto& k=p.rectified;const double sx=640.0/p.width,sy=544.0/p.height;
            intrinsics_={640,544,k.fx*sx,k.fy*sy,k.cx*sx,k.cy*sy,p.bxf*sx};
            if(!std::isfinite(intrinsics_.fx)||!std::isfinite(intrinsics_.fy)||!std::isfinite(intrinsics_.bxf)||
               !std::isfinite(intrinsics_.cx)||!std::isfinite(intrinsics_.cy)||
               intrinsics_.fx<=0||intrinsics_.fy<=0||intrinsics_.bxf<=0||
               intrinsics_.cx<0||intrinsics_.cx>=640||intrinsics_.cy<0||intrinsics_.cy>=544)
                throw std::runtime_error("Camera SDK returned invalid rectified calibration");
        }
        else {
            source_timer_=create_wall_timer(std::chrono::nanoseconds(int64_t(1e9/offline_fps_)),[this]{sourceStep();});
        }
        timer_=create_wall_timer(std::chrono::seconds(5),[this]{
            const auto timeout=input_mode_=="offline"?std::max(5.0,3.0/offline_fps_):5.0;
            const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();
            if(!offline_done_&&!offline_waiting_&&(now-last_success_ns_.load())/1e9>timeout)
                invalid("No successful input: check configured input_mode, source, exact stamps and image formats; received_images="+
                    std::to_string(received_.load())+" paired_frames="+std::to_string(paired_.load())+
                    " processed_frames="+std::to_string(processed_.load())+" dropped_pending="+std::to_string(pending_.replaced())+
                    " last_stamp="+std::to_string(last_stamp_.load()),1001);});
        metrics_started_=Steady::now();metrics_cpu_started_=cpuMs();last_success_ns_=std::chrono::duration_cast<std::chrono::nanoseconds>(metrics_started_.time_since_epoch()).count();
        try {
            worker_=std::thread([this]{workerLoop();});
            if(input_mode_=="camera")capture_thread_=std::thread([this]{
                try{while(running_&&rclcpp::ok())sourceStep();}
                catch(const std::exception& e){if(rclcpp::ok())invalid(e.what(),1001);running_=false;pending_.close();rclcpp::shutdown();}
            });
        }catch(...){running_=false;pending_.close();
            if(worker_.joinable())worker_.join();throw;}
        RCLCPP_INFO(get_logger(),"SO ready; mode=%s input_mode=%s disparity_source=%s; latest-frame worker",mode.c_str(),input_mode_.c_str(),disparity_source.c_str());
    }

PerceptionNode::~PerceptionNode() {try{close();}catch(const std::exception& e){std::cerr<<"Input shutdown: "<<e.what()<<'\n';}}

void PerceptionNode::close() {
        running_=false;motion_ready_.notify_all();if(source_timer_)source_timer_->cancel();if(timer_)timer_->cancel();pending_.close();
        if(capture_thread_.joinable())capture_thread_.join();
        if(worker_.joinable())worker_.join();
        if(camera_)camera_->close();
        auto status=ros_so_demo::newStatus();algorithm_.close(status);
    }

} // namespace ros_so_demo
