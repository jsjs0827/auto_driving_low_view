// This translation unit is built only against the real, matching V0 framework
// and common_interface package. No substitute transport ABI is provided.
#include "perception_v0_host/controller.hpp"
#include "perception_ros_so_demo/stereo_provider.hpp"
#include "base_framework/function_factory.h"
#include "base_framework/node_factory.h"
#include "base_framework/daye_shm.h"
#include "base_framework/base_utils.h"
#include "shm_struct/camera_struct.h"
#include "topic_macros.h"
#include "common_interface/msg/camera_param.hpp"
#include "common_interface/msg/grid_map_info.hpp"
#include "common_interface/msg/dock_pose.hpp"
#include "common_interface/srv/to_percep_cmd.hpp"
#include "common_interface/srv/get_percep_status.hpp"
#include "common_interface/srv/to_app_cmd.hpp"
#include "std_msgs/msg/string.hpp"
#include "rapidjson/istreamwrapper.h"
#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>

namespace perception::v0_host {
namespace {
const auto logger=rclcpp::get_logger("percep_main_v2");
std::string requiredString(const rapidjson::Value& json,const char* key) {
    if (!json.IsObject() || !json.HasMember(key) || !json[key].IsString() || !json[key].GetStringLength())
        throw std::runtime_error(std::string("percep_main requires ")+key);
    return json[key].GetString();
}
std::string selectedString(const rapidjson::Value& function,const rapidjson::Value& host,
                           const char* key,const char* fallback,bool allow_empty=false) {
    const rapidjson::Value* selected=nullptr;
    if (function.IsObject() && function.HasMember(key)) selected=&function[key];
    else if (host.IsObject() && host.HasMember(key)) selected=&host[key];
    if (selected && !selected->IsString())
        throw std::runtime_error(std::string("percep_main requires a string for ")+key);
    const std::string value=selected?std::string(selected->GetString(),selected->GetStringLength()):fallback;
    if ((!allow_empty && value.empty()) || value.find('\0')!=std::string::npos)
        throw std::runtime_error(std::string("percep_main requires a valid ")+key);
    return value;
}
template<class Packet> std::unique_ptr<Packet> ownedPacket(const uint8_t* data,size_t size) {
    static_assert(std::is_trivially_copyable_v<Packet>,"V0 packet must be trivially copyable");
    if (!data || size!=sizeof(Packet)) throw std::runtime_error("V0 SHM packet size differs from matching camera_struct.h");
    auto copy=std::make_unique<Packet>();std::memcpy(copy.get(),data,size);return copy;
}
template<class Meta,size_t N> Nv12Image imageCopy(const Meta& meta,const uint8_t (&data)[N]) {
    // Array references deliberately reject a pointer-based guessed SHM ABI.
    if (meta.stream_type!=0 || meta.bits_per_pixel!=8 || !meta.height || !meta.bytes_used ||
        meta.bytes_used>N || (uint64_t(meta.bytes_used)*2u)%(uint64_t(meta.height)*3u))
        throw std::runtime_error("V0 producer did not provide a complete NV12 image");
    const uint64_t stride=uint64_t(meta.bytes_used)*2u/(uint64_t(meta.height)*3u);
    if (stride>std::numeric_limits<uint32_t>::max()) throw std::runtime_error("NV12 stride overflow");
    Nv12Image result;result.width=meta.width;result.height=meta.height;result.stride=uint32_t(stride);
    result.frame_id=meta.frame_id;result.timestamp_ns=meta.time_ns;
    result.bytes.assign(data,data+meta.bytes_used);return result;
}
template<class Meta,size_t N> Q8Disparity disparityCopy(const Meta& meta,const uint8_t (&data)[N]) {
    if (!meta.height || !meta.bytes_used || meta.bytes_used>N || (meta.bytes_used&1u) ||
        meta.bytes_used%meta.height || meta.bits_per_pixel!=16)
        throw std::runtime_error("V0 producer did not provide complete SDK Q8 disparity");
    Q8Disparity result;result.width=meta.width;result.height=meta.height;
    result.stride_bytes=meta.bytes_used/meta.height;result.frame_id=meta.frame_id;result.timestamp_ns=meta.time_ns;
    result.samples.resize(meta.bytes_used/2u);std::memcpy(result.samples.data(),data,meta.bytes_used);return result;
}
template<class Packet,class=void> struct HasRightImage:std::false_type {};
template<class Packet> struct HasRightImage<Packet,std::void_t<
    decltype(std::declval<Packet>().right_data),decltype(std::declval<Packet>().right.width),
    decltype(std::declval<Packet>().right.height),decltype(std::declval<Packet>().right.time_ns),
    decltype(std::declval<Packet>().right.frame_id),decltype(std::declval<Packet>().right.stream_type),
    decltype(std::declval<Packet>().right.bytes_used),decltype(std::declval<Packet>().right.bits_per_pixel)>>:
    std::bool_constant<std::is_array_v<decltype(std::declval<Packet>().right_data)>> {};
template<class Packet> Nv12Image rightImageCopy(const Packet& packet) {
    if constexpr (HasRightImage<Packet>::value) return imageCopy(packet.right,packet.right_data);
    else throw std::runtime_error("raft_stereo missing_input: this upstream SHM ABI has no right image; supply a real richer input binder");
}
template<class Header> void stamp(Header& header,uint64_t ns,const std::string& frame) {
    if (ns/1000000000ull>uint64_t(std::numeric_limits<int32_t>::max()))
        throw std::runtime_error("producer timestamp exceeds ROS Header time range");
    header.stamp.sec=int32_t(ns/1000000000ull);header.stamp.nanosec=uint32_t(ns%1000000000ull);
    header.frame_id=frame;
}
} // namespace

class PercepMain final:public daye::Function {
    // Session is destroyed before the provider whose hooks it borrows.
    std::unique_ptr<ros_so_demo::StereoProvider> stereo_;
    entry::Session session_;
    BoardController controller_{session_};
    std::string parent_frame_,reset_directory_,dock_pose_topic_,grid_header_clock_;
    std::string disparity_source_,algorithm_contract_sha256_;
    std::atomic<bool> running_{true};
    daye::RosPublisher grid_pub_,dock_pub_,algorithm_pub_;
    daye::RosSubscriber camera_params_sub_;
    daye::RosServer command_server_,status_server_;
    daye::RosClient app_client_;
    daye::ShmSubscriber rgbd_sub_,gdc_sub_;
    std::thread worker_;

    void report(const char* where,const std::exception& e) {
        RCLCPP_ERROR(logger,"%s: %s",where,e.what());
    }
    void notifyApp(int command,bool success) {
        common_interface::srv::ToAppCmd::Request request;
        common_interface::srv::ToAppCmd::Response response;
        request.cmd=command;request.status=success?1:2;
        if (!app_client_.call<common_interface::srv::ToAppCmd>(request,response,10000))
            RCLCPP_ERROR(logger,"application acknowledgement failed for cmd=%d",command);
    }
    void rgbd(const uint8_t* data,size_t size,const daye::FrameInfo&) {
        if (!running_) return;
        try {
            auto packet=ownedPacket<CameraData>(data,size);
            auto left=imageCopy(packet->left,packet->left_data);
            std::string error;
            const bool accepted=disparity_source_=="raft_stereo"?
                controller_.onStereo(std::move(left),rightImageCopy(*packet),error):
                controller_.onRgbd(std::move(left),disparityCopy(packet->disp,packet->disp_data),error);
            if (!accepted && !error.empty())
                RCLCPP_ERROR(logger,"RGBD rejected: %s",error.c_str());
        } catch (const std::exception& e) {report("RGBD callback",e);}
    }
    void gdc(const uint8_t* data,size_t size,const daye::FrameInfo&) {
        if (!running_) return;
        try {
            auto packet=ownedPacket<CameraGdcData>(data,size);std::string error;
            if (!controller_.onGdc(imageCopy(packet->left,packet->left_data),error) && !error.empty())
                RCLCPP_ERROR(logger,"GDC rejected: %s",error.c_str());
        } catch (const std::exception& e) {report("GDC callback",e);}
    }
    void cameraParameters(const common_interface::msg::CameraParam& message) {
        if (!running_) return;
        CameraParameters p;p.width=message.width;p.height=message.height;p.fx=message.fx;p.fy=message.fy;
        p.cx=message.cx;p.cy=message.cy;p.baseline_mm=message.base_line;p.bxf=message.bxf;
        std::string error;if (!controller_.setIntrinsics(p,error))
            RCLCPP_ERROR(logger,"CameraParam rejected: %s",error.c_str());
    }
    bool status(common_interface::srv::GetPercepStatus::Request& request,
                common_interface::srv::GetPercepStatus::Response& response) {
        response.result=running_ && request.cmd==1?controller_.status():-1;return true;
    }
    bool resetHostLogs() {
        // 9002 is a host filesystem action. It never reaches the SDK command ABI.
        // No implicit /app/daye path is deleted. The host must name its log leaf.
        namespace fs=std::filesystem;
        if (reset_directory_.empty()) return false;
        fs::path path(reset_directory_);
        if (!path.is_absolute() || path.lexically_normal()!=path || path==path.root_path() ||
            std::distance(path.begin(),path.end())<4) return false;
        fs::path current;
        for (const auto& part:path) {
            current/=part;if (fs::is_symlink(fs::symlink_status(current))) return false;
        }
        if (!fs::exists(path)) return true;
        if (!fs::is_directory(path)) return false;
        for (const auto& item:fs::directory_iterator(path)) fs::remove_all(item.path());
        return true;
    }
    bool command(common_interface::srv::ToPercepCmd::Request& request,
                 common_interface::srv::ToPercepCmd::Response& response) {
        const bool supported=request.cmd==1 || request.cmd==2 || request.cmd==21 || request.cmd==9002;
        // V0 returns synchronous result=0 for recognized commands. Execution
        // completion/failure belongs to ToAppCmd status=1/2, not this response.
        response.result=supported?0:-1;
        if (!supported) {RCLCPP_ERROR(logger,"unsupported command=%d",request.cmd);return true;}
        auto result=ros_so_demo::newStatus();
        bool success=false;
        if (running_) {
            try {
                success=request.cmd==9002?
                    !controller_.onCommand(21,result) && resetHostLogs():
                    !controller_.onCommand(request.cmd,result);
            } catch (const std::exception& e) {report("command execution",e);}
        }
        notifyApp(request.cmd==9002?9002:(request.cmd==21?2021:2000+request.cmd),success);
        if (!success) RCLCPP_ERROR(logger,"command=%d failed: %s",request.cmd,result.detail);
        return true;
    }
    void publish(const psdk_full_result_view& view) {
        if (view.grid_count) {
            const psdk_topview* map=nullptr;
            for (uint32_t i=0;view.maps && i<view.map_count;++i)
                if (std::strncmp(view.maps[i].name,"fusion",sizeof(view.maps[i].name))==0) map=&view.maps[i];
            if (!map || map->rows>65535 || map->cols>65535 || map->resolution_mm>65535 ||
                uint64_t(map->rows)*map->cols!=view.grid_count || !view.avg_h_cm ||
                !view.min_h_cm || !view.max_h_cm || !view.grid_types)
                throw std::runtime_error("SDK fusion grid cannot be represented by GridMapInfo");
            common_interface::msg::GridMapInfo message;
            uint64_t header_ns=view.timestamp_ns;
            if (grid_header_clock_=="legacy_steady_ms") {
                // Preserve V0's external Header clock and millisecond precision.
                // The SDK frame/result retains its original capture timestamp.
                const auto milliseconds=daye::getSteadyTimeMs();
                if (milliseconds<0 || uint64_t(milliseconds)>std::numeric_limits<uint64_t>::max()/1000000ull)
                    throw std::runtime_error("legacy steady clock exceeds GridMapInfo Header range");
                header_ns=uint64_t(milliseconds)*1000000ull;
            }
            stamp(message.header,header_ns,parent_frame_);
            message.rows=uint16_t(map->rows);message.cols=uint16_t(map->cols);message.resolution_mm=uint16_t(map->resolution_mm);
            message.avg_h_cm.assign(view.avg_h_cm,view.avg_h_cm+view.grid_count);
            message.min_h_cm.assign(view.min_h_cm,view.min_h_cm+view.grid_count);
            message.max_h_cm.assign(view.max_h_cm,view.max_h_cm+view.grid_count);
            message.type.assign(view.grid_types,view.grid_types+view.grid_count);grid_pub_.publish(message);
        }
        if (view.recharge.executed && view.recharge.mark_ok && view.recharge.pose_ok) {
            if (view.recharge.timestamp_ns<=0) throw std::runtime_error("SDK DockPose has no original timestamp");
            common_interface::msg::DockPose message;stamp(message.header,uint64_t(view.recharge.timestamp_ns),parent_frame_);
            message.x=view.recharge.x_m;message.y=view.recharge.y_m;message.yaw=view.recharge.yaw_rad;
            dock_pub_.publish(message);
        }
    }
    void work() {
        while (running_) {
            if (!controller_.waitForWork(std::chrono::milliseconds(100))) continue;
            try {
                entry::Result result;auto status=ros_so_demo::newStatus();
                if (controller_.step(result,status,[this,&result](const auto& view){
                    publish(view);
                    // All selected algorithm results, including missing_input
                    // and valid:false, remain visible for optional input slots.
                    const char* json=nullptr;auto diagnostics=ros_so_demo::newStatus();
                    if (psdk_full_result_get_algorithms_json(result.get(),&json,&diagnostics))
                        throw std::runtime_error(diagnostics.detail);
                    std_msgs::msg::String message;message.data=json?json:"{}";algorithm_pub_.publish(message);
                }) && status.code)
                    RCLCPP_ERROR(logger,"SDK process failed code=%u: %s",status.code,status.detail);
            } catch (const std::exception& e) {report("SDK worker",e);}
        }
    }
public:
    explicit PercepMain(const rapidjson::Value& json):daye::Function(json) {
        const auto host_config=requiredString(json,"v2_host_config");
        std::ifstream host_stream(host_config);
        if (!host_stream) throw std::runtime_error("cannot read v2_host_config");
        rapidjson::IStreamWrapper wrapper(host_stream);rapidjson::Document host;
        host.ParseStream(wrapper);
        if (host.HasParseError() || !host.IsObject()) throw std::runtime_error("invalid v0_host.json");
        if (!host.HasMember("schema_version") || !host["schema_version"].IsInt() || host["schema_version"].GetInt()!=1)
            throw std::runtime_error("unsupported v0_host.json schema_version");
        const auto pipeline=requiredString(host,"sdk_config");
        algorithm_contract_sha256_=requiredString(host,"algorithm_contract_sha256");
        dock_pose_topic_=selectedString(json,host,"dock_pose_topic","");
        grid_header_clock_=selectedString(json,host,"grid_header_clock","legacy_steady_ms");
        if (grid_header_clock_!="legacy_steady_ms" && grid_header_clock_!="capture_ns")
            throw std::runtime_error("grid_header_clock must be legacy_steady_ms or capture_ns");
        disparity_source_=requiredString(host,"disparity_source");
        if (disparity_source_!="original_q8" && disparity_source_!="raft_stereo")
            throw std::runtime_error("v0_host.json disparity_source must be original_q8 or raft_stereo");
        if (!host.HasMember("initial_command") || !host["initial_command"].IsInt() || host["initial_command"].GetInt()!=21)
            throw std::runtime_error("V0 plugin initial_command must be 21 (external commands own work)");
        if (disparity_source_=="raft_stereo") {
            if constexpr (!HasRightImage<CameraData>::value)
                throw std::runtime_error("raft_stereo missing_input: matched upstream CameraData ABI has no right image");
            if (!host.HasMember("stereo") || !host["stereo"].IsObject()) throw std::runtime_error("missing StereoProvider settings");
            const auto& configured=host["stereo"];ros_so_demo::StereoSettings settings;
            settings.model=requiredString(configured,"model");settings.input_mode=requiredString(configured,"input_mode");
            if (!configured.HasMember("input_width") || !configured["input_width"].IsInt() ||
                !configured.HasMember("input_height") || !configured["input_height"].IsInt())
                throw std::runtime_error("missing StereoProvider input dimensions");
            settings.input_width=configured["input_width"].GetInt();settings.input_height=configured["input_height"].GetInt();
            stereo_=std::make_unique<ros_so_demo::StereoProvider>(settings);
            controller_.setProcessBinding([this](const psdk_full_frame& frame){
                if (frame.disparity || frame.recharge_hires) return;
                if (!frame.right) throw std::runtime_error("raft_stereo missing_input: original same-frame right image");
                stereo_->setPair(frame.left,*frame.right);
            },[this]{stereo_->clearPair();});
        }
        parent_frame_=selectedString(json,host,"parent_frame","base_link");
        reset_directory_=selectedString(json,host,"factory_reset_directory","",true);
        auto status=ros_so_demo::newStatus();
        if (session_.open(pipeline.c_str(),status,nullptr,nullptr,stereo_?stereo_->hooks():nullptr))
            throw std::runtime_error(status.detail);
        RCLCPP_INFO(logger,"shared SDK configuration contract sha256=%s",algorithm_contract_sha256_.c_str());
        // The external camera producer owns startup and both V0 SHM channels.
        // This plugin only subscribes; it does not reset channels or launch it.
        if (controller_.onCommand(21,status)) throw std::runtime_error(status.detail);
        grid_pub_=daye::CreatePublish<common_interface::msg::GridMapInfo>(kGridMapInfo,rclcpp::SensorDataQoS());
        dock_pub_=daye::CreatePublish<common_interface::msg::DockPose>(dock_pose_topic_,rclcpp::SensorDataQoS());
        algorithm_pub_=daye::CreatePublish<std_msgs::msg::String>("/percep_main/algorithm_results",rclcpp::QoS(5));
        app_client_=daye::CreateClient<common_interface::srv::ToAppCmd>(kToApplicationCmd);
        auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        camera_params_sub_=daye::CreateSubscribe<common_interface::msg::CameraParam>(kCameraParam,qos,
            [this](const common_interface::msg::CameraParam& p){cameraParameters(p);});
        command_server_=daye::CreateServer<common_interface::srv::ToPercepCmd>(kToPercepCmd,&PercepMain::command,this);
        status_server_=daye::CreateServer<common_interface::srv::GetPercepStatus>(kGetPercepStatus,&PercepMain::status,this);
        rgbd_sub_=daye::ShmSubscriber(daye::ChannelConfig{kCameraFrameShm,kFrameShmSize,false},
            [this](const uint8_t* d,size_t n,const auto& i){rgbd(d,n,i);},100);
        gdc_sub_=daye::ShmSubscriber(daye::ChannelConfig{kCameraGdcFrameShm,kGdcFrameShmSize,false},
            [this](const uint8_t* d,size_t n,const auto& i){gdc(d,n,i);},100);
        worker_=std::thread(&PercepMain::work,this);
    }
    ~PercepMain() override {
        running_=false;
        // Stop transport callbacks before destroying owned queue storage/session.
        rgbd_sub_=daye::ShmSubscriber{};gdc_sub_=daye::ShmSubscriber{};
        camera_params_sub_=daye::RosSubscriber{};command_server_=daye::RosServer{};status_server_=daye::RosServer{};
        controller_.shutdown();if (worker_.joinable()) worker_.join();
        auto status=ros_so_demo::newStatus();session_.close(status);
    }
};
} // namespace perception::v0_host

REGISTER_FUNCTION(factory) {
    factory.registerCreater<perception::v0_host::PercepMain>("percep_main");
}
