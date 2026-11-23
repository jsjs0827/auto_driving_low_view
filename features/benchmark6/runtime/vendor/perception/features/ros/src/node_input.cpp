#include "perception_ros_so_demo/perception_node.hpp"
#include "perception_ros_so_demo/json_text.hpp"
#include "perception_ros_so_demo/image_messages.hpp"

namespace ros_so_demo {

Image::ConstSharedPtr PerceptionNode::fromPlane(raw::Plane&& plane,const std::string& name) {
        raw::checkStamp(plane.capture.ns);
        auto out=std::make_shared<Image>();out->header.frame_id=name;
        out->header.stamp.sec=static_cast<int32_t>(plane.capture.ns/1000000000ULL);
        out->header.stamp.nanosec=static_cast<uint32_t>(plane.capture.ns%1000000000ULL);
        out->width=plane.width;out->height=plane.height;out->step=plane.step;
        out->encoding=std::move(plane.encoding);out->is_bigendian=0;out->data=std::move(plane.bytes);
        return out;
    }

void PerceptionNode::configureOffline() {
        const auto path=declare_parameter<std::string>("offline_path","");
        offline_files_=declare_parameter<std::vector<std::string>>("offline_files",std::vector<std::string>{});
        offline_right_files_=declare_parameter<std::vector<std::string>>("offline_right_files",std::vector<std::string>{});
        offline_resize_=declare_parameter<std::string>("offline_resize","letterbox");
        offline_format_=declare_parameter<std::string>("offline_format","nv12");
        offline_fps_=declare_parameter<double>("offline_fps",2.0);
        offline_loop_=declare_parameter<bool>("offline_loop",false);
        offline_wait_for_subscribers_=declare_parameter<bool>("offline_wait_for_subscribers",false);
        offline_max_frames_=declare_parameter<int>("offline_max_frames",0);
        if(recharge_||(disparity_&&!new_stereo_))throw std::invalid_argument("Offline disparity requires raft_stereo paired images; original Q8 requires recording mode; recharge requires a same-capture GDC input");
        if(!std::isfinite(offline_fps_)||offline_fps_<=0||offline_fps_>120||offline_max_frames_<0)
            throw std::invalid_argument("offline_fps must be (0,120], offline_max_frames >= 0");
        if(offline_resize_!="letterbox"&&offline_resize_!="stretch")throw std::invalid_argument("offline_resize: letterbox/stretch");
        if(offline_format_!="nv12"&&offline_format_!="i420")throw std::invalid_argument("offline_format: nv12/i420");
        if(!path.empty()&&!offline_files_.empty())throw std::invalid_argument("Use offline_path OR offline_files, not both");
        if(!path.empty()) {
            const auto source=fs::u8path(path);
            if(fs::is_directory(source)) {
                for(const auto& entry:fs::directory_iterator(source))if(entry.is_regular_file()) {
                    auto extension=entry.path().extension().u8string();
                    std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char c){return char(std::tolower(c));});
                    if(extension==".jpg"||extension==".jpeg"||extension==".png"||extension==".bmp"||
                       extension==".ppm"||extension==".pnm"||extension==".pgm"||extension==".nv12"||extension==".i420"||extension==".yuv")
                        offline_files_.push_back(entry.path().u8string());
                }
                std::sort(offline_files_.begin(),offline_files_.end());
            } else offline_files_.push_back(path);
        }
        if(offline_files_.empty())throw std::invalid_argument("Set offline_path to a picture/directory or offline_files to a nonempty list");
        for(auto& file:offline_files_) {
            const auto source=fs::absolute(fs::u8path(file));
            if(!fs::is_regular_file(source))throw std::invalid_argument("Offline picture not found: "+source.u8string());
            file=source.u8string();
        }
        if(new_stereo_&&offline_right_files_.size()!=offline_files_.size())
            throw std::invalid_argument("raft_stereo offline requires offline_right_files paired one-to-one with offline_files");
        if(!new_stereo_&&!offline_right_files_.empty())throw std::invalid_argument("offline_right_files requires raft_stereo");
        for(auto& file:offline_right_files_) {
            const auto source=fs::absolute(fs::u8path(file));
            if(!fs::is_regular_file(source))throw std::invalid_argument("Offline right picture not found: "+source.u8string());
            file=source.u8string();
        }
    }

void PerceptionNode::sourceStep() {
        if(input_mode_=="offline") {
            if(offline_done_||pending_.hasPending())return; // Offline playback applies backpressure instead of dropping files.
            offline_waiting_=publish_enabled_&&offline_wait_for_subscribers_&&
                (!result_pub_->get_subscription_count()||
                 (publish_mask_&&!mask_pub_->get_subscription_count())||
                 (publish_input_&&!input_pub_->get_subscription_count()));
            if(offline_waiting_) {
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for configured offline output subscribers");
                return;
            }
            const auto load_started=Steady::now();
            const auto& file=offline_files_[offline_index_];
            auto converted=ros_so_demo::offline::load(file,offline_resize_,offline_format_);
            const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if(wall<=0)throw std::runtime_error("Invalid host wall clock for offline input");
            const auto stamp=std::max(uint64_t(wall),last_stamp_.load()+1);last_stamp_=stamp;
            raw::Plane plane;plane.capture={++sequence_,stamp};plane.width=640;plane.height=544;plane.step=640;
            plane.encoding=offline_format_;plane.bytes=std::move(converted.bytes);
            SourceInfo info;info.id_source="offline_sequence";info.timestamp_source="offline_host_wall_clock";
            info.file=file;info.resize=converted.resize;info.width=converted.original_width;info.height=converted.original_height;
            Pending work;work.left=fromPlane(std::move(plane),frame_name_);work.id=sequence_;work.stamp=stamp;
            work.info=info;work.calibration=intrinsics_;
            work.calibration_source=(mapping_||disparity_)?"intrinsics_file":"not_required";
            if(new_stereo_) {
                auto right=ros_so_demo::offline::load(offline_right_files_[offline_index_],offline_resize_,offline_format_);
                if(converted.original_width!=640||converted.original_height!=544||right.original_width!=640||right.original_height!=544)
                    throw std::invalid_argument("Offline stereo pairs must already be rectified 640x544 matching explicit calibration; do not letterbox rectified pairs");
                raw::Plane rp;rp.capture={sequence_,stamp};rp.width=640;rp.height=544;rp.step=640;
                rp.encoding=offline_format_;rp.bytes=std::move(right.bytes);
                work.right=fromPlane(std::move(rp),frame_name_+"_right");
                work.info.right_file=offline_right_files_[offline_index_];
            }
            work.load_ms=milliseconds(load_started);work.received=Steady::now();++paired_;pending_.put(std::move(work));
            ++offline_index_;
            if((offline_max_frames_&&sequence_>=uint64_t(offline_max_frames_))||
               (offline_index_==offline_files_.size()&&!offline_loop_)) {
                offline_done_=true;source_timer_->cancel();
                RCLCPP_INFO(get_logger(),"Offline input finished after %llu frames; node remains available until Ctrl+C",
                            static_cast<unsigned long long>(sequence_));
            } else if(offline_index_==offline_files_.size())offline_index_=0;
            return;
        }
        try {
            const auto acquired=Steady::now();
            auto capture=camera_->next([this] {return running_&&rclcpp::ok();});
            const auto id=capture.left.capture;
            if(id.ns<=last_stamp_)throw std::runtime_error("Duplicate/backward SDK capture timestamp");
            last_stamp_=id.ns;SourceInfo info;info.id_source="camera_sdk";info.timestamp_source="camera_sdk";
            const auto left=fromPlane(std::move(capture.left),frame_name_);
            const auto right=new_stereo_?fromPlane(std::move(capture.right),frame_name_+"_right"):Image::ConstSharedPtr{};
            const auto depth=disparity_&&!new_stereo_?fromPlane(std::move(capture.disparity),frame_name_):Image::ConstSharedPtr{};
            const auto high=recharge_?fromPlane(std::move(capture.gdc),frame_name_):Image::ConstSharedPtr{};
            Pending work;work.left=left;work.right=right;work.depth=depth;work.hires=high;work.id=id.id;work.stamp=id.ns;
            work.info=info;work.calibration=intrinsics_;work.calibration_source="camera_sdk_rectified_scaled";
            work.load_ms=milliseconds(acquired);work.received=Steady::now();++received_;++paired_;pending_.put(std::move(work));
        } catch(const raw::FatalCameraError&) {throw;}
        catch(const std::exception& e) {if(running_&&rclcpp::ok())invalid(e.what(),1001);}
    }

void PerceptionNode::input(std::size_t slot,Image::ConstSharedPtr image) {
        uint64_t stamp=0;
        try{
            ++received_;stamp=timestamp(image->header);
            synchronize(slot,stamp,Payload(std::move(image)));
        }catch(const std::exception& e){invalid(e.what(),1002,stamp);}
    }

void PerceptionNode::cameraInfo(std::size_t slot,CameraInfo::ConstSharedPtr info) {
        uint64_t stamp=0;
        try {
            stamp=timestamp(info->header);
            synchronize(slot,stamp,Payload(std::move(info)));
        }catch(const std::exception& e){invalid(e.what(),1002,stamp);}
    }

void PerceptionNode::synchronize(std::size_t slot,uint64_t stamp,Payload value) {
        std::lock_guard<std::mutex> guard(sync_mutex_);
        auto match=synchronizer_->push(slot,stamp,std::move(value));
        if(!match)return;
        auto image=[&](std::size_t i){return match->values[i]?std::get<Image::ConstSharedPtr>(*match->values[i]):Image::ConstSharedPtr{};};
        Pending work;work.left=image(Left);work.right=image(Right);work.depth=image(Depth);work.hires=image(Hires);
        work.stamp=stamp;work.id=++sequence_;work.calibration=intrinsics_;
        if(frame_calibration_required_&&!use_camera_info_)work.calibration_source="ros_parameters";
        if(use_camera_info_) {
            const auto& li=*std::get<CameraInfo::ConstSharedPtr>(*match->values[LeftInfo]);
            const auto& ri=*std::get<CameraInfo::ConstSharedPtr>(*match->values[RightInfo]);
            work.calibration=ros_so_demo::stereoIntrinsics(li,ri,work.left->header,work.left->width,work.left->height);
            if(work.right&&ri.header.frame_id!=work.right->header.frame_id)
                throw std::invalid_argument("Right CameraInfo frame must match right image frame");
            work.calibration_source="camera_info_rectified_P";
        }
        last_stamp_=stamp;++paired_;work.received=Steady::now();pending_.put(std::move(work));
    }

} // namespace ros_so_demo
