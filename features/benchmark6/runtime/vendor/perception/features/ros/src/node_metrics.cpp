#include "perception_ros_so_demo/perception_node.hpp"
#include "perception_ros_so_demo/json_text.hpp"
#include "perception_ros_so_demo/image_messages.hpp"

namespace ros_so_demo {

void PerceptionNode::metrics(const Pending& work,const psdk_full_result_view& v,double compute,double save,double total,double cpu,double queue_ms) {
        const auto done=processed_.load();if((done-1)%metrics_every_)return;
        uint64_t unmatched=0,stale=0;std::size_t waiting=0;
        if(synchronizer_){std::lock_guard<std::mutex> guard(sync_mutex_);const auto s=synchronizer_->stats();unmatched=s.evicted;stale=s.stale_messages;waiting=s.pending;}
        const auto elapsed=std::chrono::duration<double>(Steady::now()-metrics_started_).count();
        std::ostringstream out;
        out<<"{\"frame_id\":"<<work.id<<",\"timestamp_ns\":"<<work.stamp
           <<",\"input_mode\":"<<quote(input_mode_)<<",\"disparity_source\":"<<quote(new_stereo_?"raft_stereo":"original_q8")
           <<",\"received_images_or_captures\":"<<received_.load()<<",\"paired_frames\":"<<paired_.load()
           <<",\"processed_frames\":"<<done<<",\"dropped_pending_frames\":"<<pending_.replaced()
           <<",\"dropped_unmatched_captures\":"<<unmatched<<",\"stale_messages\":"<<stale<<",\"sync_pending_captures\":"<<waiting
           <<",\"load_or_acquire_ms\":"<<number(work.load_ms)<<",\"queue_wait_ms\":"<<number(queue_ms)
           <<",\"compute_ms\":"<<number(compute)<<",\"graph_ms\":"<<number(v.graph_ms)
           <<",\"stereo_ms\":"<<(stereo_?number(stereo_->elapsedMs()):"null")<<",\"save_ms\":"<<number(save)
           <<",\"worker_frame_ms\":"<<number(total)<<",\"process_cpu_ms\":"<<number(cpu)
           <<",\"process_cpu_percent_over_worker_frame\":"<<number(total>0?100*cpu/total:0)
           <<",\"rss_kib\":"<<rssKib()<<",\"completed_fps_since_ready\":"<<number(elapsed>0?done/elapsed:0)
           <<",\"process_cpu_percent_since_ready\":"<<number(elapsed>0?(cpuMs()-metrics_cpu_started_)/(elapsed*10):0)
           <<",\"det_pre_ms\":"<<number(v.inference.timings.detection_pre_ms)
           <<",\"det_infer_ms\":"<<number(v.inference.timings.detection_infer_ms)
           <<",\"det_post_ms\":"<<number(v.inference.timings.detection_post_ms)
           <<",\"seg_pre_ms\":"<<number(v.inference.timings.segmentation_pre_ms)
           <<",\"seg_infer_ms\":"<<number(v.inference.timings.segmentation_infer_ms)
           <<",\"seg_post_ms\":"<<number(v.inference.timings.segmentation_post_ms)
           <<",\"task_trace\":"<<(v.task_trace_json?v.task_trace_json:"null")
           <<",\"cpu_scope\":\"whole_process_including_camera_DDS_threads_during_worker_frame\""
           <<",\"fps_scope\":\"completed_frames_per_wall_second_since_node_ready_including_wait_and_save\"";
        if(stereo_)out<<",\"stereo_model\":"<<stereo_->metadataJson();
        out<<'}';Text value;value.data=out.str();metrics_pub_->publish(value);
        if(metrics_file_.is_open()){metrics_file_<<value.data<<'\n';metrics_file_.flush();if(!metrics_file_)throw std::runtime_error("Cannot append metrics_path");}
    }

} // namespace ros_so_demo
