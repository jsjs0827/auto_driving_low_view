#include "perception_ros_so_demo/perception_node.hpp"
#include "perception_ros_so_demo/json_text.hpp"
#include "perception_ros_so_demo/image_messages.hpp"

namespace ros_so_demo {
void PerceptionNode::wheelOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message) {
    try {
        if(!running_||!message)return;
        if(message->child_frame_id!=robot_frame_)throw std::invalid_argument("Wheel odometry twist must use configured robot_frame_id as child_frame_id");
        const auto stamp=timestamp(message->header);const auto& twist=message->twist.twist;
        if(!stamp||!std::isfinite(twist.linear.x)||!std::isfinite(twist.angular.z))throw std::invalid_argument("Wheel odometry requires capture-clock stamp and finite velocity");
        psdk_motion_sample sample{};sample.struct_size=sizeof(sample);sample.abi_version=PSDK_SLIP_ABI_VERSION;
        sample.timestamp_ns=stamp;sample.linear_speed_mps=twist.linear.x;sample.yaw_rate_radps=twist.angular.z;
        {std::lock_guard<std::mutex> lock(motion_mutex_);
         if(stamp<=last_motion_stamp_)return;
         last_motion_stamp_=stamp;motion_pending_.push_back(sample);
         while(motion_pending_.size()>4096)motion_pending_.pop_front();}
        motion_ready_.notify_one();
    }catch(const std::exception& e){invalid(e.what(),1002);}
}

void PerceptionNode::processFrame(const Pending& work) {
        const auto started=Steady::now();const double cpu_started=cpuMs();
        const auto queue_ms=std::max(0.0,std::chrono::duration<double,std::milli>(started-work.received).count());
        const auto& left=work.left;const auto& depth=work.depth;const auto& hires=work.hires;
        const auto id=work.id,stamp=work.stamp;const auto& info=work.info;
        const auto* calibration_source=work.calibration_source.c_str();
        psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
        frame.left=imageView(*left,id,stamp);frame.intrinsics=work.calibration;
        if(left->header.frame_id.empty())throw std::invalid_argument("Left image frame_id must be explicit");
        if(frame.left.width!=640||frame.left.height!=544)
            throw std::invalid_argument("Original RGBD-left input must remain 640x544; no resize is performed here");
        psdk_frame high{};psdk_disparity_q8 q{};std::vector<uint16_t> aligned_q8;
        if(hires){high=imageView(*hires,id,stamp);
            if(!ros_so_demo::sameStamp(hires->header,left->header)||hires->header.frame_id!=left->header.frame_id)
                throw std::invalid_argument("GDC and left image must have the same stamp/frame_id");
            if(high.format!=PSDK_FORMAT_NV12||high.width!=1280||high.height!=1088)
                throw std::invalid_argument("Recharge input must be same-capture 1280x1088 GDC-left NV12");
            frame.recharge_hires=&high;}
        if(depth){
            if(!ros_so_demo::sameStamp(depth->header,left->header)||depth->header.frame_id!=left->header.frame_id)
                throw std::invalid_argument("Q8 and left image must have the same stamp/frame_id");
            if(depth->encoding!="16UC1"||depth->is_bigendian||depth->width!=frame.left.width||
               depth->height!=frame.left.height||depth->step<depth->width*2||(depth->step&1u))
                throw std::invalid_argument("Expected same-capture little-endian 16UC1 Q8 disparity, not depth-mm/float disparity");
            const auto bytes=uint64_t(depth->step)*depth->height;
            if(depth->data.size()<bytes)throw std::invalid_argument("Q8 storage is shorter than stride*height");
            aligned_q8.resize(std::size_t(bytes/2));std::memcpy(aligned_q8.data(),depth->data.data(),std::size_t(bytes));
            q.struct_size=sizeof(q);q.abi_version=PSDK_FULL_ABI_VERSION;q.width=depth->width;q.height=depth->height;
            q.stride_bytes=depth->step;q.data=aligned_q8.data();q.bytes=bytes;q.frame_id=id;q.timestamp_ns=stamp;frame.disparity=&q;
        }
        psdk_frame right{};
        if(new_stereo_) {
            if(!work.right)throw std::invalid_argument("New Stereo requires the exact-capture right image");
            if(!ros_so_demo::sameStamp(work.right->header,left->header)||work.right->header.frame_id.empty())
                throw std::invalid_argument("Left/right images require exact capture stamps and explicit optical frame IDs");
            right=imageView(*work.right,id,stamp);
            frame.right=&right; // Same packet becomes input.right in the product DAG.
            stereo_->setPair(frame.left,right);
            frame.disparity=nullptr; // A supplied SDK Q8 would bypass the provider.
        }
        struct PairGuard {ros_so_demo::StereoProvider* p;~PairGuard(){if(p)p->clearPair();}} pair_guard{stereo_.get()};
        if(publish_enabled_&&publish_input_)input_pub_->publish(*left);
        ros_so_demo::Result owner;auto status=ros_so_demo::newStatus();
        std::vector<psdk_motion_sample> motion;
        {std::unique_lock<std::mutex> lock(motion_mutex_);
         // Wait only on the worker. ROS callbacks keep receiving samples; never
         // extrapolate beyond the latest measured wheel sample. Timeout still
         // processes the frame, with an explicit unavailable reference if needed.
         if(motion_sub_&&motion_wait_timeout_ms_>0)
             motion_ready_.wait_for(lock,std::chrono::milliseconds(motion_wait_timeout_ms_),
                 [&]{return !running_||last_motion_stamp_>=stamp;});
         if(!running_)return;
         while(!motion_pending_.empty()){
             auto sample=motion_pending_.front();motion_pending_.pop_front();motion.push_back(sample);
             if(sample.timestamp_ns>stamp)break;
         }}
        if(!motion.empty()&&algorithm_.pushMotion(motion.data(),static_cast<uint32_t>(motion.size()),status)){report(status);return;}
        // The entire original DAG/Task/Feature implementation is inside this call.
        const auto compute_started=Steady::now();
        if(algorithm_.process(frame,owner,status)){report(status);return;}
        const auto compute_ms=milliseconds(compute_started);
        if(new_stereo_&&!stereo_->called())throw std::runtime_error("SDK did not request Stereo disparity; enable the disparity task in SDK configuration");
        psdk_full_result_view v{};
        if(owner.view(v,status)){report(status);return;}
        if(v.frame_id!=id||v.timestamp_ns!=stamp){invalid("SO result identity differs from submitted capture",1002,stamp);return;}
#ifdef R36_ROS_WITH_RVIZ
        if(publish_enabled_&&rviz_stream_)try {
            psdk_slip_view slip{};slip.struct_size=sizeof(slip);slip.abi_version=PSDK_SLIP_ABI_VERSION;
            const psdk_slip_view* motion=nullptr;
            if(rviz_stream_->wantsSlip()&&!psdk_full_result_get_slip_view(owner.get(),&slip,&status))motion=&slip;
            rviz_stream_->publish(frame.left,v,left->header.frame_id,robot_frame_,motion);
        }catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"RViz display output: %s",e.what());}
#endif
        ++processed_;last_success_ns_=std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();
        Code success;success.data=0;code_pub_->publish(success);
        if(publish_enabled_&&(processed_-1)%uint64_t(publish_every_)==0)
            publish(v,left->header,owner,info,frame.intrinsics,calibration_source);
        if(publish_enabled_) {
            psdk_slip_view slip{};slip.struct_size=sizeof(slip);slip.abi_version=PSDK_SLIP_ABI_VERSION;
            if(psdk_full_result_get_slip_view(owner.get(),&slip,&status))report(status);
            else if(slip.enabled){Text message;message.data=slip.details_json?slip.details_json:"{}";slip_pub_->publish(message);}
        }
        const auto save_started=Steady::now();bool saved=false;
        if(save_every_&&(processed_-1)%uint64_t(save_every_)==0){
            saved=true;
            const auto path=save_root_+"/frame-"+std::to_string(id)+"-"+std::to_string(stamp);
            if(psdk_full_save_result(owner.get(),path.c_str(),&status))report(status);
            else {
                std::ofstream source(fs::u8path(path)/"source.json",std::ios::binary);
                source<<"{\"input_mode\":"<<quote(input_mode_)<<",\"frame_id\":"<<id
                      <<",\"disparity_source\":"<<quote(new_stereo_?"raft_stereo":"original_q8")
                      <<",\"frame_id_source\":"<<quote(info.id_source)<<",\"timestamp_ns\":"<<stamp
                      <<",\"timestamp_source\":"<<quote(info.timestamp_source)<<",\"file\":"<<quote(info.file)
                      <<",\"right_file\":"<<quote(info.right_file)
                      <<",\"source_width\":"<<info.width<<",\"source_height\":"<<info.height
                      <<",\"resize\":"<<quote(info.resize)<<",\"input_width\":640,\"input_height\":544,"
                      <<calibrationJson(frame.intrinsics,calibration_source)<<"}\n";
                source.close();if(!source)invalid("Cannot save source.json",2002,stamp);
                if(save_clouds_ply_)try {
                    ros_so_demo::savePlyClouds(v,fs::u8path(path),left->header,robot_frame_);
                }catch(const std::exception& e){invalid(e.what(),2002,stamp);}
            }
        }
        const auto save_ms=saved?milliseconds(save_started):0.0;
        if(metrics_enabled_)metrics(work,v,compute_ms,save_ms,milliseconds(started),cpuMs()-cpu_started,queue_ms);
        // owner releases result after publishing copies; borrowed pointers are not cached.
    }

void PerceptionNode::workerLoop() {
        Pending work;
        while(running_&&pending_.take(work)) {
            try{processFrame(work);}catch(const std::exception& e){if(rclcpp::ok())invalid(e.what(),1002,work.stamp);}
        }
    }

} // namespace ros_so_demo
