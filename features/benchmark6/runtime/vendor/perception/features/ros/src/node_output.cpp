#include "perception_ros_so_demo/perception_node.hpp"
#include "perception_ros_so_demo/json_text.hpp"
#include "perception_ros_so_demo/image_messages.hpp"

namespace ros_so_demo {

void PerceptionNode::report(const psdk_status& s) {
        Code c;c.data=s.code;code_pub_->publish(c);
        Text m;m.data="{\"code\":"+std::to_string(s.code)+",\"native_code\":"+std::to_string(s.native_code)+
            ",\"frame_id\":"+std::to_string(s.frame_id)+",\"timestamp_ns\":"+std::to_string(s.timestamp_ns)+
            ",\"feature\":"+quote(s.feature)+",\"task\":"+quote(s.task)+",\"detail\":"+quote(s.detail)+"}";
        error_pub_->publish(m);if(s.code)RCLCPP_ERROR(get_logger(),"%s",m.data.c_str());
    }

void PerceptionNode::invalid(const std::string& detail,uint32_t code,uint64_t stamp) {
        auto s=ros_so_demo::newStatus();s.code=code;s.timestamp_ns=stamp;
        std::strncpy(s.feature,"ros_demo.input",sizeof(s.feature)-1);
        std::strncpy(s.detail,detail.c_str(),sizeof(s.detail)-1);report(s);
    }

std::string PerceptionNode::calibrationJson(const psdk_camera_intrinsics& k,const char* source) {
        return "\"calibration_source\":"+quote(source)+",\"intrinsics\":{\"width\":"+std::to_string(k.width)+
            ",\"height\":"+std::to_string(k.height)+",\"fx\":"+number(k.fx)+",\"fy\":"+number(k.fy)+
            ",\"cx\":"+number(k.cx)+",\"cy\":"+number(k.cy)+",\"bxf\":"+number(k.bxf)+
            ",\"bxf_unit\":\"px_mm\",\"baseline_m\":"+number(k.bxf/k.fx/1000.0)+"}";
    }

void PerceptionNode::image(const rclcpp::Publisher<Image>::SharedPtr& pub,const std_msgs::msg::Header& header,
               const uint8_t* data,uint64_t bytes,uint32_t width,uint32_t height,const char* encoding,uint32_t step) {
        if(!data||!bytes)return;
        Image out;out.header=header;out.width=width;out.height=height;out.encoding=encoding;
        out.is_bigendian=0;out.step=step;out.data.assign(data,data+bytes);pub->publish(out);
    }

void PerceptionNode::publish(const psdk_full_result_view& v,const std_msgs::msg::Header& header,
                 const ros_so_demo::Result& owner,const SourceInfo& info,
                 const psdk_camera_intrinsics& calibration,const char* calibration_source) {
        Text result;std::ostringstream o;
        o<<"{\"frame_id\":"<<v.frame_id<<",\"frame_id_source\":"<<quote(info.id_source)<<",\"timestamp_ns\":"<<v.timestamp_ns
         <<",\"timestamp_source\":"<<quote(info.timestamp_source)<<",\"input_mode\":"<<quote(input_mode_)
         <<",\"disparity_source\":"<<quote(new_stereo_?"raft_stereo":"original_q8")
         <<",\"source_image\":{\"file\":"<<quote(info.file)<<",\"width\":"<<info.width<<",\"height\":"<<info.height
         <<",\"right_file\":"<<quote(info.right_file)<<",\"resize\":"<<quote(info.resize)<<"},\"input_width\":640,\"input_height\":544"
         <<','<<calibrationJson(calibration,calibration_source)
         <<",\"ros_frame\":"<<quote(header.frame_id)<<",\"has_inference\":"<<(v.has_inference?"true":"false")
         <<",\"has_disparity\":"<<(v.disparity.data&&v.disparity.bytes?"true":"false")
         <<",\"graph_ms\":"<<number(v.graph_ms)<<",\"process_ms\":"<<number(v.process_ms)<<",\"detections\":[";
        for(uint64_t i=0;i<v.inference.detection_count;++i){const auto& d=v.inference.detections[i];if(i)o<<',';
            o<<"{\"class_id\":"<<d.class_id<<",\"name\":"<<quote(d.class_name)<<",\"score\":"<<number(d.confidence)
             <<",\"x1\":"<<number(d.x1)<<",\"y1\":"<<number(d.y1)<<",\"x2\":"<<number(d.x2)<<",\"y2\":"<<number(d.y2)<<'}';}
        const auto& t=v.inference.timings;
        o<<"],\"inference_ms\":{\"det_pre\":"<<number(t.detection_pre_ms)<<",\"det_infer\":"<<number(t.detection_infer_ms)
         <<",\"det_post\":"<<number(t.detection_post_ms)<<",\"seg_pre\":"<<number(t.segmentation_pre_ms)
         <<",\"seg_infer\":"<<number(t.segmentation_infer_ms)<<",\"seg_post\":"<<number(t.segmentation_post_ms)
         <<"},\"map_count\":"<<v.map_count<<",\"nearest_obstacle\":{\"valid\":"<<v.nearest_obstacle_valid
         <<",\"distance_m\":"<<number(v.nearest_obstacle_distance_m)<<"},\"recharge\":{\"executed\":"<<v.recharge.executed
         <<",\"mark_ok\":"<<v.recharge.mark_ok<<",\"pose_ok\":"<<v.recharge.pose_ok<<",\"x_m\":"<<number(v.recharge.x_m)
         <<",\"y_m\":"<<number(v.recharge.y_m)<<",\"yaw_rad\":"<<number(v.recharge.yaw_rad)<<",\"error\":"<<quote(v.recharge.error)<<"}}";
        result.data=o.str();result_pub_->publish(result);
        if(publish_disparity_&&v.disparity.data&&v.disparity.bytes)
            image(disparity_pub_,header,reinterpret_cast<const uint8_t*>(v.disparity.data),
                  v.disparity.bytes,v.disparity.width,v.disparity.height,"16UC1",v.disparity.stride_bytes);
        if(publish_mask_&&v.has_inference)
            image(mask_pub_,header,v.inference.segmentation_mask,v.inference.segmentation_bytes,
                  v.inference.width,v.inference.height,"mono8",v.inference.width);
        if(publish_maps_)for(uint32_t i=0;i<v.map_count;++i){const auto& m=v.maps[i];
            const std::string name=m.name;
            auto pub=name=="pcl"?pcl_pub_:name=="ai"?ai_pub_:fusion_pub_;
            image(pub,header,m.rgb,m.rgb_bytes,m.image_width,m.image_height,"rgb8",m.image_width*3);}
        if(publish_clouds_) {
            if(v.cloud_count&&!v.clouds)throw std::invalid_argument("SDK cloud views are null");
            for(uint32_t i=0;i<v.cloud_count;++i) {
                const auto& cloud=v.clouds[i];
                cloud_pubs_.at(ros_so_demo::cloudName(cloud))->publish(
                    ros_so_demo::makePointCloud2(cloud,header,robot_frame_));
            }
        }
        psdk_temporal_view h{};h.struct_size=sizeof(h);h.abi_version=PSDK_TEMPORAL_ABI_VERSION;
        auto status=ros_so_demo::newStatus();
        if(psdk_full_result_get_temporal_view(owner.get(),&h,&status))report(status);
        else {Text history;history.data=h.analysis_json?h.analysis_json:"{}";history_pub_->publish(history);}
    }

} // namespace ros_so_demo
