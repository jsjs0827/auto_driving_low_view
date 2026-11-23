#include "perception_ros_so_demo/mo_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using Image=sensor_msgs::msg::Image;
using CameraInfo=sensor_msgs::msg::CameraInfo;
using Text=std_msgs::msg::String;
using Code=std_msgs::msg::UInt32;
namespace raw=ros_so_demo::raw;

class RawCamera final:public rclcpp::Node {
    std::unique_ptr<raw::Camera> camera_;
    rclcpp::Publisher<Image>::SharedPtr left_,right_,q8_,gdc_;
    rclcpp::Publisher<CameraInfo>::SharedPtr left_info_,right_info_;
    CameraInfo left_calibration_,right_calibration_;
    rclcpp::Publisher<Text>::SharedPtr capture_,errors_;
    rclcpp::Publisher<Code>::SharedPtr code_;
    std::string frame_name_,right_frame_name_;
    std::uint64_t last_stamp_=0, count_=0;
    static Image message(raw::Plane&& plane,const std::string& name) {
        raw::checkStamp(plane.capture.ns);
        Image out; out.header.frame_id=name;
        out.header.stamp.sec=static_cast<std::int32_t>(plane.capture.ns/1000000000ULL);
        out.header.stamp.nanosec=static_cast<std::uint32_t>(plane.capture.ns%1000000000ULL);
        out.width=plane.width; out.height=plane.height; out.step=plane.step;
        out.encoding=std::move(plane.encoding); out.is_bigendian=0; out.data=std::move(plane.bytes);
        return out;
    }
public:
    RawCamera():Node("perception_raw_camera") {
        const auto mode=declare_parameter<std::string>("mode","infer");
        if (mode!="infer" && mode!="map" && mode!="both" && mode!="recharge")
            throw std::invalid_argument("mode must be infer/map/both/recharge");
        raw::CameraOptions options;
        options.sdk=declare_parameter<std::string>("camera_sdk",options.sdk);
        options.config=declare_parameter<std::string>("camera_config",options.config);
        options.timeout_ms=declare_parameter<int>("acquire_timeout_ms",3000);
        options.retry_ms=declare_parameter<int>("retry_interval_ms",5);
        options.need_q8=mode=="map" || mode=="both";
        options.need_gdc=mode=="recharge" || mode=="both";
        options.disparity_mode=declare_parameter<int>("camera_disparity_mode",options.need_q8?2:1);
        frame_name_=declare_parameter<std::string>("frame_id","camera_optical_frame");
        right_frame_name_=declare_parameter<std::string>("right_frame_id","camera_right_optical_frame");
        if (frame_name_.empty()||right_frame_name_.empty()) throw std::invalid_argument("frame_id/right_frame_id must not be empty");
        auto qos=rclcpp::SensorDataQoS().keep_last(4);
        const auto reliability=declare_parameter<std::string>("input_reliability","best_effort");
        if(reliability=="reliable")qos.reliable();
        else if(reliability!="best_effort")
            throw std::invalid_argument("input_reliability must be reliable or best_effort");
        left_=create_publisher<Image>(declare_parameter<std::string>("left_topic","/camera/rgbd_left_nv12"),qos);
        right_=create_publisher<Image>(declare_parameter<std::string>("right_topic","/camera/rgbd_right_nv12"),qos);
        const bool publish_info=declare_parameter<bool>("publish_camera_info",true);
        if(publish_info) {
            left_info_=create_publisher<CameraInfo>(declare_parameter<std::string>("left_camera_info_topic","/stereo/left/camera_info"),qos);
            right_info_=create_publisher<CameraInfo>(declare_parameter<std::string>("right_camera_info_topic","/stereo/right/camera_info"),qos);
        }
        if (options.need_q8)
            q8_=create_publisher<Image>(declare_parameter<std::string>("q8_topic","/camera/disparity_q8"),qos);
        if (options.need_gdc)
            gdc_=create_publisher<Image>(declare_parameter<std::string>("gdc_topic","/camera/gdc_left_nv12"),qos);
        capture_=create_publisher<Text>("~/capture",qos);
        errors_=create_publisher<Text>("~/errors",10); code_=create_publisher<Code>("~/error_code",10);
        RCLCPP_INFO(get_logger(),"Opening camera mode=%s sdk=%s config=%s; the old ai_node must be stopped",
            mode.c_str(),options.sdk.c_str(),options.config.c_str());
        camera_=std::make_unique<raw::Camera>(options);
        if(publish_info) {
            const auto p=camera_->parameters();
            if(!((p.width==640&&p.height==544)||(p.width==1280&&p.height==1088)))
                throw std::runtime_error("SDK rectified reference must be verified 640x544 or 1280x1088; cannot guess crop calibration");
            const double sx=640.0/p.width,sy=544.0/p.height;
            const double fx=p.rectified.fx*sx,fy=p.rectified.fy*sy,cx=p.rectified.cx*sx,cy=p.rectified.cy*sy;
            left_calibration_.width=640;left_calibration_.height=544;
            left_calibration_.distortion_model="plumb_bob";left_calibration_.d.assign(5,0.0);
            left_calibration_.k={fx,0,cx,0,fy,cy,0,0,1};
            left_calibration_.r={1,0,0,0,1,0,0,0,1};
            left_calibration_.p={fx,0,cx,0,0,fy,cy,0,0,0,1,0};
            right_calibration_=left_calibration_;
            // ROS horizontal projection Tx is -fx*baseline_metres; SDK bxf is px*mm.
            right_calibration_.p[3]=-p.bxf*sx/1000.0;
        }
    }
    void step() {
        try {
            auto frame=camera_->next([] { return rclcpp::ok(); });
            const auto id=frame.left.capture;
            if (id.ns<=last_stamp_) throw std::runtime_error("Duplicate/backward capture timestamp; frame was not published");
            last_stamp_=id.ns;
            // Prepare all messages before publishing any plane. Data copies already
            // outlive the SDK release, and every plane has the original capture stamp.
            auto left=message(std::move(frame.left),frame_name_);
            auto right=message(std::move(frame.right),right_frame_name_);
            Image disparity,high;
            if (q8_) disparity=message(std::move(frame.disparity),frame_name_);
            if (gdc_) high=message(std::move(frame.gdc),frame_name_);
            if(left_info_) {
                left_calibration_.header=left.header;right_calibration_.header=right.header;
                left_info_->publish(left_calibration_);right_info_->publish(right_calibration_);
            }
            left_->publish(left); right_->publish(right);
            if (q8_) q8_->publish(disparity); if (gdc_) gdc_->publish(high);
            Text meta; meta.data="{\"frame_id\":"+std::to_string(id.id)+",\"timestamp_ns\":"+
                std::to_string(id.ns)+",\"frame_id_source\":\"camera_sdk\",\"right\":true,\"q8\":"+
                (q8_?"true":"false")+",\"gdc\":"+(gdc_?"true":"false")+"}";
            capture_->publish(meta); Code ok; ok.data=0; code_->publish(ok);
            if (++count_==1) RCLCPP_INFO(get_logger(),"First raw frame: id=%llu stamp=%llu, NV12 640x544",
                static_cast<unsigned long long>(id.id),static_cast<unsigned long long>(id.ns));
        } catch (const raw::FatalCameraError&) {
            throw; // Failed buffer release is fatal: close before any further acquisition.
        } catch (const std::exception& e) {
            if (!rclcpp::ok()) return;
            Text detail; detail.data=e.what(); errors_->publish(detail); Code error; error.data=1001; code_->publish(error);
            RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,"%s",e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    void close() { if (camera_) camera_->close(); }
};
int main(int argc,char** argv) {
    rclcpp::init(argc,argv); int status=0;
    try {
        auto node=std::make_shared<RawCamera>();
        while (rclcpp::ok()) { rclcpp::spin_some(node); node->step(); }
        node->close();
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; status=1; }
    rclcpp::shutdown(); return status;
}
