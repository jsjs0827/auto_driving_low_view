#pragma once
/* Optional real ROS2 transport adapter. Core .so and C headers do not require ROS.
 * Build a ROS caller with rclcpp, sensor_msgs and visualization_msgs.
 */
#include "det_cloud.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace det_cloud_ros2 {
class Publisher {
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
public:
    Publisher(rclcpp::Node::SharedPtr node,const std::string& cloud_topic="/percep_demo/det_cloud",
              const std::string& marker_topic="/percep_demo/det_markers") :node_(std::move(node)) {
        cloud_=node_->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic,rclcpp::QoS(1));
        markers_=node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic,rclcpp::QoS(1));
    }
    void publish(const ds_cloud_view& v) {
        const auto stamp=rclcpp::Time(static_cast<int64_t>(v.timestamp_ns),RCL_ROS_TIME);
        if(v.publish_cloud) {
            if(v.point_count>std::numeric_limits<uint32_t>::max()/24)throw std::length_error("ROS cloud is too large");
            sensor_msgs::msg::PointCloud2 msg;msg.header.frame_id=v.output_frame;msg.header.stamp=stamp;
            msg.width=static_cast<uint32_t>(v.point_count);msg.height=1;msg.is_dense=false;msg.is_bigendian=false;msg.point_step=24;msg.row_step=msg.width*24;
            const char* names[]={"x","y","z","rgb","label","confidence"};
            for(uint32_t i=0;i<6;++i) {
                sensor_msgs::msg::PointField f;f.name=names[i];f.offset=i*4;f.count=1;
                f.datatype=i==4?sensor_msgs::msg::PointField::INT32:sensor_msgs::msg::PointField::FLOAT32;
                msg.fields.push_back(f);
            }
            msg.data.resize(msg.row_step);
            for(size_t i=0;i<v.point_count;++i) {
                const auto& p=v.points[i];auto* b=msg.data.data()+i*24;
                std::memcpy(b,&p.x,4);std::memcpy(b+4,&p.y,4);std::memcpy(b+8,&p.z,4);
                const uint32_t rgb=(uint32_t(p.r)<<16)|(uint32_t(p.g)<<8)|p.b;
                std::memcpy(b+12,&rgb,4);std::memcpy(b+16,&p.label,4);std::memcpy(b+20,&p.confidence,4);
            }
            cloud_->publish(msg);
        }
        // Original semantic-fusion branch returns before marker publication.
        if(v.clear_markers) {
            visualization_msgs::msg::MarkerArray msg;
            visualization_msgs::msg::Marker clear;clear.header.frame_id=v.output_frame;clear.header.stamp=stamp;
            clear.ns="det";clear.id=0;clear.action=visualization_msgs::msg::Marker::DELETEALL;msg.markers.push_back(clear);
            for(size_t i=0;i<v.marker_count;++i) {
                const auto& p=v.markers[i];auto sphere=clear;
                sphere.id=p.detection_index+1;sphere.type=visualization_msgs::msg::Marker::SPHERE;sphere.action=visualization_msgs::msg::Marker::ADD;
                sphere.pose.position.x=p.x;sphere.pose.position.y=p.y;sphere.pose.position.z=p.z;sphere.pose.orientation.w=1;
                sphere.scale.x=sphere.scale.y=sphere.scale.z=.12;
                sphere.color.r=p.r/255.f;sphere.color.g=p.g/255.f;sphere.color.b=p.b/255.f;sphere.color.a=.85f;
                sphere.lifetime=rclcpp::Duration::from_seconds(.5);msg.markers.push_back(sphere);
                auto text=sphere;text.id=p.detection_index+1000;text.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text.pose.position.z+=.15;text.scale.x=text.scale.y=0;text.scale.z=.12;text.text=p.text;msg.markers.push_back(text);
            }
            markers_->publish(msg);
        }
    }
    static int callback(void* userdata,const ds_cloud_view* view) noexcept {
        try {if(!userdata||!view)return 1;static_cast<Publisher*>(userdata)->publish(*view);return 0;}catch(...){return 1;}
    }
};
} // namespace det_cloud_ros2
