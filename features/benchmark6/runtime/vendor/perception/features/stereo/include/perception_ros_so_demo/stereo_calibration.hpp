#ifndef PERCEPTION_ROS_SO_DEMO_STEREO_CALIBRATION_HPP
#define PERCEPTION_ROS_SO_DEMO_STEREO_CALIBRATION_HPP
#include "perception_full_sdk.h"
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ros_so_demo {
inline bool calibrationNear(double a,double b) {
    return std::abs(a-b)<=1e-8*std::max({1.0,std::abs(a),std::abs(b)});
}
inline bool sameStamp(const std_msgs::msg::Header& a,const std_msgs::msg::Header& b) {
    return a.stamp.sec==b.stamp.sec&&a.stamp.nanosec==b.stamp.nanosec;
}
// P describes the delivered RECTIFIED images, not the original distorted K.
// ROS P[3] is fx*translation_m; the SDK takes fx*baseline_mm.
inline psdk_camera_intrinsics stereoIntrinsics(
        const sensor_msgs::msg::CameraInfo& left,const sensor_msgs::msg::CameraInfo& right,
        const std_msgs::msg::Header& image,uint32_t width,uint32_t height) {
    if(!sameStamp(left.header,image)||!sameStamp(right.header,image))
        throw std::invalid_argument("Stereo CameraInfo must have the exact left-image capture stamp");
    if(image.frame_id.empty()||left.header.frame_id!=image.frame_id||right.header.frame_id.empty())
        throw std::invalid_argument("Left CameraInfo frame must match the left image; right frame must be explicit");
    if(width!=640||height!=544||left.width!=width||left.height!=height||right.width!=width||right.height!=height)
        throw std::invalid_argument("Stereo CameraInfo and rectified images must all be 640x544");
    auto validate=[](const sensor_msgs::msg::CameraInfo& info,const char* side) {
        auto finite=[&](const auto& values) {for(double v:values)if(!std::isfinite(v))
            throw std::invalid_argument(std::string(side)+" CameraInfo contains non-finite calibration");};
        finite(info.p);finite(info.k);finite(info.r);finite(info.d);
        const auto& p=info.p;
        if(p[0]<=0||p[5]<=0||p[2]<0||p[6]<0||p[2]>=info.width||p[6]>=info.height)
            throw std::invalid_argument(std::string(side)+" rectified P has invalid focal length/principal point");
        if(!calibrationNear(p[1],0)||!calibrationNear(p[4],0)||!calibrationNear(p[7],0)||
           !calibrationNear(p[8],0)||!calibrationNear(p[9],0)||!calibrationNear(p[10],1)||!calibrationNear(p[11],0))
            throw std::invalid_argument(std::string(side)+" P must describe a canonical horizontal rectified camera");
        if(info.binning_x>1||info.binning_y>1)
            throw std::invalid_argument("Stereo CameraInfo binning must already be incorporated into P and delivered dimensions");
    };
    validate(left,"Left");validate(right,"Right");
    const auto& l=left.p;const auto& r=right.p;
    if(!calibrationNear(l[0],r[0])||!calibrationNear(l[5],r[5])||
       !calibrationNear(l[2],r[2])||!calibrationNear(l[6],r[6]))
        throw std::invalid_argument("Stereo P matrices require matching fx/fy/cx/cy (CALIB_ZERO_DISPARITY)");
    if(!calibrationNear(l[3],0)||r[3]>=0)
        throw std::invalid_argument("Stereo P requires left Tx=0 and right Tx=-fx*positive_baseline_m");
    const double baseline_m=-r[3]/r[0];
    const double bxf=-r[3]*1000.0;
    if(!std::isfinite(baseline_m)||!std::isfinite(bxf)||baseline_m<=0||bxf<=0)
        throw std::invalid_argument("Stereo baseline/bxf must be finite and positive");
    return {width,height,l[0],l[5],l[2],l[6],bxf};
}
} // namespace ros_so_demo
#endif
