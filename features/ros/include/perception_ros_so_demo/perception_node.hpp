#pragma once
#include "perception_ros_so_demo/sdk_caller.hpp"
#include "perception_ros_so_demo/mo_camera.hpp"
#include "perception_ros_so_demo/offline_image.hpp"
#include "perception_ros_so_demo/stereo_calibration.hpp"
#include "perception_ros_so_demo/cloud_output.hpp"
#include "perception_ros_so_demo/stereo_provider.hpp"
#include "perception_ros_so_demo/latest_frame.hpp"
#include "perception_temporal_sdk.h"
#ifdef R36_ROS_WITH_RVIZ
#include "rviz_sdk_output.hpp"
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deque>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <variant>
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "perception_ros_so_demo/source_info.hpp"
#include "perception_ros_so_demo/process_metrics.hpp"

namespace ros_so_demo {
using Image=sensor_msgs::msg::Image;
using CameraInfo=sensor_msgs::msg::CameraInfo;
using PointCloud=sensor_msgs::msg::PointCloud2;
using Text=std_msgs::msg::String;
using Code=std_msgs::msg::UInt32;
namespace fs=std::filesystem;
// The node owns scheduling and ROS transport; algorithm work lives in features.
class PerceptionNode final:public rclcpp::Node {
    using Payload=std::variant<Image::ConstSharedPtr,CameraInfo::ConstSharedPtr>;
    enum Slot:std::size_t {Left,Right,Depth,Hires,LeftInfo,RightInfo};
    struct Pending {
        Image::ConstSharedPtr left,right,depth,hires;
        uint64_t id=0,stamp=0;
        SourceInfo info;
        psdk_camera_intrinsics calibration{};
        std::string calibration_source="not_required";
        double load_ms=0;
        Steady::time_point received=Steady::now();
    };
    std::unique_ptr<ros_so_demo::StereoProvider> stereo_;
    ros_so_demo::Caller algorithm_; // Destroy engine before its borrowed provider hooks.
#ifdef R36_ROS_WITH_RVIZ
    std::unique_ptr<perception::rviz::SdkPublisher> rviz_stream_;
#endif
    std::unique_ptr<ros_so_demo::ExactFrames<Payload,6>> synchronizer_;
    ros_so_demo::LatestFrame<Pending> pending_;
    std::mutex sync_mutex_;
    std::mutex motion_mutex_;
    std::condition_variable motion_ready_;
    int motion_wait_timeout_ms_=50;
    std::deque<psdk_motion_sample> motion_pending_;
    uint64_t last_motion_stamp_=0;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_sub_;
    rclcpp::Publisher<Text>::SharedPtr slip_pub_;
    std::thread worker_,capture_thread_;
    std::atomic<bool> running_{true};
    std::size_t pair_capacity_=4;
    uint64_t sequence_=0;
    std::atomic<uint64_t> last_stamp_{0},received_{0},processed_{0},paired_{0};
    std::atomic<int64_t> last_success_ns_{0};
    bool new_stereo_=false;
    bool mapping_=false,disparity_=false,recharge_=false,publish_maps_=false,publish_mask_=true;
    bool frame_calibration_required_=false;
    bool publish_enabled_=true,metrics_enabled_=true;
    bool publish_input_=false,offline_loop_=false,offline_done_=false;
    bool offline_wait_for_subscribers_=false,offline_waiting_=false;
    bool use_camera_info_=false,publish_clouds_=false,publish_disparity_=true,save_clouds_ply_=false;
    int publish_every_=1,save_every_=0;
    std::string save_root_,input_mode_,frame_name_,offline_resize_,offline_format_;
    std::string robot_frame_;
    std::vector<std::string> offline_files_,offline_right_files_,offline_q8_files_;
    std::size_t offline_index_=0;
    int offline_max_frames_=0;
    double offline_fps_=2.0;
    std::unique_ptr<raw::Camera> camera_;
    psdk_camera_intrinsics intrinsics_{};
    Steady::time_point metrics_started_=Steady::now();
    double metrics_cpu_started_=0;
    uint64_t metrics_every_=1;
    std::ofstream metrics_file_;
    rclcpp::Subscription<Image>::SharedPtr left_sub_,right_sub_,q8_sub_,gdc_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr left_info_sub_,right_info_sub_;
    rclcpp::Publisher<Image>::SharedPtr mask_pub_,pcl_pub_,ai_pub_,fusion_pub_,input_pub_,disparity_pub_;
    rclcpp::Publisher<Text>::SharedPtr result_pub_,history_pub_,error_pub_,metrics_pub_;
    rclcpp::Publisher<Code>::SharedPtr code_pub_;
    std::map<std::string,rclcpp::Publisher<PointCloud>::SharedPtr> cloud_pubs_;
    rclcpp::TimerBase::SharedPtr timer_,source_timer_;

    static Image::ConstSharedPtr fromPlane(raw::Plane&& plane,const std::string& name);
    void configureOffline();
    void sourceStep();

    void report(const psdk_status& s);
    void invalid(const std::string& detail,uint32_t code=1002,uint64_t stamp=0);
    void input(std::size_t slot,Image::ConstSharedPtr image);
    void cameraInfo(std::size_t slot,CameraInfo::ConstSharedPtr info);
    void synchronize(std::size_t slot,uint64_t stamp,Payload value);
    static std::string calibrationJson(const psdk_camera_intrinsics& k,const char* source);
    void image(const rclcpp::Publisher<Image>::SharedPtr& pub,const std_msgs::msg::Header& header,
               const uint8_t* data,uint64_t bytes,uint32_t width,uint32_t height,const char* encoding,uint32_t step);
    void publish(const psdk_full_result_view& v,const std_msgs::msg::Header& header,
                 const ros_so_demo::Result& owner,const SourceInfo& info,
                 const psdk_camera_intrinsics& calibration,const char* calibration_source);
    void processFrame(const Pending& work);
    void wheelOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
    void metrics(const Pending& work,const psdk_full_result_view& v,double compute,double save,double total,double cpu,double queue_ms);
    void workerLoop();
public:
    PerceptionNode();
    ~PerceptionNode()override;
    void close();
};
} // namespace ros_so_demo
