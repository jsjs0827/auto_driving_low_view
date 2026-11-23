#include "full_pipeline_api.hpp"
#include "perception_full_sdk.h"
#include "perception_temporal.hpp"
#include "config.hpp"
#include "full_graph.hpp"
#include "json.hpp"
#include "stereo_inputs.hpp"
#include "r33/mapping.hpp"
#include "bridge.hpp"
#include "percep_demo/charger_crop.hpp"
#include "product/command_contract.hpp"
#include "visualization.hpp"
#include "runtime_status.hpp"
#include "perception/frame_input.hpp"
#include "perception/frame_resources.hpp"
#include "visual_motion.hpp"
#include "motion_reference.hpp"
#include "slip_detection.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <system_error>

namespace {
using Clock=std::chrono::steady_clock;
namespace fs=std::filesystem;
std::mutex full_gate;
thread_local bool in_callback=false;
struct CallbackGuard {bool previous=in_callback;CallbackGuard(){in_callback=true;}~CallbackGuard(){in_callback=previous;}};
double milliseconds(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
template<std::size_t N>void copyText(char(&dst)[N],const std::string& s){const auto n=std::min(N-1,s.size());std::memcpy(dst,s.data(),n);dst[n]=0;}
template<std::size_t N>void copyText(char(&dst)[N],const char* s) noexcept {std::size_t n=0;if(s)while(n+1<N&&s[n]){dst[n]=s[n];++n;}dst[n]=0;}
std::uint32_t rawStatus(psdk_status* out,std::uint32_t code,int native,perception::FrameInfo frame,const char* feature,const char* task,const char* detail) noexcept {
    if(out){*out={};out->struct_size=sizeof(*out);out->abi_version=PSDK_ABI_VERSION;out->code=code;out->native_code=native;out->frame_id=frame.id;out->timestamp_ns=frame.timestamp_ns;copyText(out->feature,feature);copyText(out->task,task);copyText(out->detail,detail);}return code;
}
std::uint32_t caught(psdk_status* out,const std::exception& ex,std::uint32_t fallback,const char* feature,perception::FrameInfo frame={}) noexcept {
    if(const auto* f=dynamic_cast<const r28::Failure*>(&ex)){const auto& s=f->status();return rawStatus(out,s.code?s.code:fallback,s.native_code,(s.frame.id||s.frame.timestamp_ns)?s.frame:frame,s.feature.empty()?feature:s.feature.c_str(),s.task.c_str(),s.detail.c_str());}
    if(const auto* f=dynamic_cast<const perception::PerceptionException*>(&ex)){const auto& s=f->info();return rawStatus(out,static_cast<std::uint32_t>(s.code)?static_cast<std::uint32_t>(s.code):fallback,0,(s.frame.id||s.frame.timestamp_ns)?s.frame:frame,s.component.empty()?feature:s.component.c_str(),"",s.detail.c_str());}
    const auto* system=dynamic_cast<const std::system_error*>(&ex);return rawStatus(out,fallback,system?system->code().value():0,frame,feature,"",ex.what());
}
std::uint32_t status(psdk_status* out,const r28::Status& s={}) noexcept {
    if(out){*out={};out->struct_size=sizeof(*out);out->abi_version=PSDK_ABI_VERSION;out->code=s.code;out->native_code=s.native_code;
        out->frame_id=s.frame.id;out->timestamp_ns=s.frame.timestamp_ns;copyText(out->feature,s.feature);copyText(out->task,s.task);copyText(out->detail,s.detail);}
    return s.code;
}
r28::Status fromPublic(const psdk_status& s,const char* feature,perception::FrameInfo frame) {
    auto bounded=[](const char* text,std::size_t size){const auto* end=static_cast<const char*>(std::memchr(text,0,size));return std::string(text,end?std::size_t(end-text):size);};
    return {s.code,s.native_code,s.feature[0]?bounded(s.feature,sizeof(s.feature)):feature,bounded(s.task,sizeof(s.task)),bounded(s.detail,sizeof(s.detail)),frame};
}
[[noreturn]]void fail(std::uint32_t code,const std::string& text,const char* feature="full.sdk",perception::FrameInfo frame={}){throw r28::Failure({code,0,feature,{},text,frame});}
void check(std::uint32_t code,const psdk_status& s,const char* feature,perception::FrameInfo frame={}){if(code){auto v=fromPublic(s,feature,frame);v.code=code;throw r28::Failure(v);}}
void reentry(){if(in_callback)fail(2001,"Callbacks must not reenter full SDK calls","full.callback");}
template<class T>bool abi(const T* p){return p&&p->struct_size==sizeof(*p)&&p->abi_version==PSDK_FULL_ABI_VERSION;}
perception::ImageView image(const psdk_frame& f) {
    if(f.struct_size!=sizeof(f)||f.abi_version!=PSDK_ABI_VERSION||!f.data||!f.width||!f.height||f.width>8192||f.height>8192||f.stride<f.width||f.stride>16*1024*1024u||(f.width|f.height|f.stride)&1u||
       (f.format!=PSDK_FORMAT_NV12&&f.format!=PSDK_FORMAT_I420)||f.bytes<uint64_t(f.stride)*f.height*3/2||f.bytes>std::numeric_limits<std::size_t>::max())fail(1002,"Invalid contiguous NV12/I420 frame","full.input");
    return {f.data,std::size_t(f.bytes),f.stride,int(f.width),int(f.height),f.format==PSDK_FORMAT_NV12?perception::PixelFormat::NV12:perception::PixelFormat::I420};
}
void binary(const fs::path& p,const void* ptr,std::size_t bytes){std::ofstream f(p,std::ios::binary);if(!f)fail(2002,"Cannot create output file: "+p.u8string(),"full.record");if(bytes)f.write(static_cast<const char*>(ptr),std::streamsize(bytes));f.close();if(!f)fail(2002,"Output write/close failed: "+p.u8string(),"full.record");}
void text(const fs::path& p,const std::string& value){binary(p,value.data(),value.size());}
struct JsonScalar {double value;};
JsonScalar jsonScalar(double value){return {value};}
std::ostream& operator<<(std::ostream& out,JsonScalar value){if(std::isfinite(value.value))out<<value.value;else out<<"null";return out;}
}

struct psdk_full_result {
    perception::motion::VisualResult visual_motion;
    perception::motion::ReferenceResult motion_reference;
    perception::motion::SlipResult slip;
    bool motion_enabled=false,slip_enabled=false;
    double visual_ms=0,reference_ms=0,slip_ms=0;
    std::string slip_json="{}";
    std::shared_ptr<const r36::temporal::Window> history;
    r36::temporal::Analysis temporal;
    std::vector<psdk_temporal_frame> history_views;
    std::string temporal_json="{}";
    psdk_result* inference=nullptr;
    psdk_result* detection=nullptr;
    psdk_result* segmentation=nullptr;
    psdk_result_view inference_view{};
    psdk_result_view detection_view{},segmentation_view{};
    std::array<double,2> inference_begin_ms{},inference_end_ms{};
    perception::FrameInfo frame;
    std::uint64_t run_id=0;
    int width=0,height=0;
    std::shared_ptr<const perception::RgbFrame> rgb;
    std::vector<std::uint8_t> disparity_gray,raw_input,recharge_rgb;
    std::shared_ptr<const std::vector<std::uint16_t>> disparity;
    std::shared_ptr<const r33::MappingResult> mapping;
    std::array<std::vector<psdk_grid_cell>,3> map_cells;
    std::array<psdk_topview,3> map_views{};
    std::array<std::vector<psdk_cloud_point>,5> cloud_points;
    std::array<psdk_cloud_view,5> cloud_views{};
    psdk_dock_pose pose{};
    std::string pose_details="{}",trace="{}",effective;
    psdk_calibration_result calibration{};
    bool overlay=true,save_inputs=false,finalized=false;
    double graph_ms=0,process_ms=0;
    ~psdk_full_result(){psdk_result_release(inference);psdk_result_release(detection);psdk_result_release(segmentation);}
};
struct psdk_full_engine {
    perception::motion::VisualMotion visual_motion;
    perception::motion::MotionBuffer motion_reference;
    perception::motion::SlipDetector slip_detector;
    r36::temporal::History history;
    psdk_temporal_callback temporal_callback=nullptr;
    void* temporal_userdata=nullptr;
    r33::full::Config config;
    r33::full::Graph graph;
    psdk_engine* inference=nullptr;
    psdk_engine* detection=nullptr;
    psdk_engine* segmentation=nullptr;
    psdk_full_hooks hooks{};
    std::uint64_t runs=0,ai_tick=0,recharge_tick=0;
    int ai_every_n=1;
    Clock::time_point last_recharge{};
    Aiper_Recharge::RechargeHostConfig recharge_host;
#ifdef R33_WITH_MAPPING
    std::unique_ptr<r33::MappingPipeline> mapping;
#endif
#ifdef R33_WITH_RECHARGE
    std::unique_ptr<r33::recharge::Engine> recharge;
#endif
    ~psdk_full_engine(){psdk_status ignored{};psdk_destroy(&inference,&ignored);psdk_destroy(&detection,&ignored);psdk_destroy(&segmentation,&ignored);}
};

namespace {
bool hasInference(const psdk_full_result& out) {
    return out.inference||out.detection||out.segmentation;
}
const std::vector<std::uint8_t>& rgbPixels(const psdk_full_result& out) {
    static const std::vector<std::uint8_t> empty;
    return out.rgb?out.rgb->pixels:empty;
}
// Read only after dependencies have joined all active semantic producers.
// Each lane result retains its original immutable SDK owner; this view does
// not copy masks or boxes and remains valid until full_result_release.
psdk_result_view inferenceView(const psdk_full_result& out) {
    if(out.inference)return out.inference_view;
    if(!out.detection&&!out.segmentation)return {};
    psdk_result_view view{};view.struct_size=sizeof(view);view.abi_version=PSDK_ABI_VERSION;
    view.width=out.width;view.height=out.height;view.frame_id=out.frame.id;view.timestamp_ns=out.frame.timestamp_ns;
    if(out.detection){const auto& d=out.detection_view;view.detections=d.detections;view.detection_count=d.detection_count;
        view.timings.detection_pre_ms=d.timings.detection_pre_ms;view.timings.detection_infer_ms=d.timings.detection_infer_ms;view.timings.detection_post_ms=d.timings.detection_post_ms;
        view.timings.pack_ms+=d.timings.pack_ms;}
    if(out.segmentation){const auto& s=out.segmentation_view;view.segmentation_mask=s.segmentation_mask;view.segmentation_bytes=s.segmentation_bytes;
        view.timings.segmentation_pre_ms=s.timings.segmentation_pre_ms;view.timings.segmentation_infer_ms=s.timings.segmentation_infer_ms;view.timings.segmentation_post_ms=s.timings.segmentation_post_ms;
        view.timings.pack_ms+=s.timings.pack_ms;}
    if(out.detection||out.segmentation){
        const auto first=out.detection?out.inference_begin_ms[0]:out.inference_begin_ms[1];
        const auto begin=out.segmentation?std::min(first,out.inference_begin_ms[1]):first;
        view.timings.graph_ms=std::max(out.inference_end_ms[0],out.inference_end_ms[1])-begin;
        view.timings.process_ms=view.timings.graph_ms;
    }
    return view;
}
void refreshHistory(psdk_full_result& out) {
    out.history_views.clear();
    if(out.history){for(const auto& f:out.history->frames)out.history_views.push_back(f->view());
        out.temporal_json=r36::temporal::json(*out.history,out.temporal);}
}
psdk_temporal_view temporalView(const psdk_full_result& out) {
    if(!out.history)throw std::invalid_argument("Result has no temporal snapshot");
    return r36::temporal::view(*out.history,out.temporal,out.history_views,out.temporal_json);
}
psdk_slip_view slipView(const psdk_full_result& out) {
    psdk_slip_view v{};v.struct_size=sizeof(v);v.abi_version=PSDK_SLIP_ABI_VERSION;
    v.enabled=out.motion_enabled;v.state=out.motion_enabled?PSDK_SLIP_UNAVAILABLE:PSDK_SLIP_DISABLED;
    const auto& m=out.visual_motion;const auto& r=out.motion_reference;const auto& s=out.slip;
    v.visual_valid=m.available;v.reference_valid=r.available;v.body_calibration_confirmed=m.body_confirmed;
    v.start_timestamp_ns=m.t1_ns?m.t0_ns:r.t0_ns;v.end_timestamp_ns=m.t1_ns?m.t1_ns:r.t1_ns;
    v.dt_s=m.t1_ns?m.dt_s:r.dt_s;v.frame_id=out.frame.id;
    if(out.history&&out.history->frames.size()>=2)v.previous_frame_id=out.history->frames[out.history->frames.size()-2]->left.metadata.frame_id;
    v.visual_forward_m=m.forward_m;v.visual_lateral_m=m.lateral_m;v.visual_yaw_rad=m.yaw_rad;
    v.reference_forward_m=r.forward_m;v.reference_lateral_m=r.lateral_m;v.reference_yaw_rad=r.yaw_rad;
    v.visual_speed_mps=m.available&&m.dt_s>0?m.forward_m/m.dt_s:std::numeric_limits<double>::quiet_NaN();
    v.reference_speed_mps=r.available&&r.dt_s>0?r.forward_m/r.dt_s:std::numeric_limits<double>::quiet_NaN();
    v.slip_ratio=s.available?s.slip_ratio:std::numeric_limits<double>::quiet_NaN();
    v.visual_quality=m.quality;v.visual_rmse_m=m.residual_m;v.matches=m.matches;v.inliers=m.inliers;
    v.visual_ms=out.visual_ms;v.reference_ms=out.reference_ms;v.slip_ms=out.slip_ms;
    std::string state_name=out.motion_enabled?"unavailable":"disabled",reason=out.motion_enabled?"slip_disabled":"motion_disabled";
    if(out.slip_enabled) {
        state_name=perception::motion::stateName(s.state);reason=s.reason;
        switch(s.state){
          case perception::motion::SlipState::Normal:v.state=PSDK_SLIP_NORMAL;break;
          case perception::motion::SlipState::Suspected:v.state=PSDK_SLIP_SUSPECTED;break;
          case perception::motion::SlipState::Slipping:v.state=PSDK_SLIP_SLIPPING;break;
          case perception::motion::SlipState::Recovering:v.state=PSDK_SLIP_RECOVERING;break;
          default:break;
        }
    } else if(!m.available&&!m.reason.empty()&&out.motion_enabled)reason=m.reason;
    if(out.motion_enabled&&out.history&&out.history->frames.size()<2){v.state=PSDK_SLIP_INITIALIZING;state_name="initializing";reason="insufficient_history";}
    copyText(v.state_name,state_name);copyText(v.reason,reason);v.details_json=out.slip_json.c_str();return v;
}
void refreshSlip(psdk_full_result& out) {
    const auto v=slipView(out);std::ostringstream o;o<<std::setprecision(12);
    o<<"{\"schema_version\":1,\"enabled\":"<<(v.enabled?"true":"false")
     <<",\"state\":"<<r33::json::quote(v.state_name)<<",\"reason\":"<<r33::json::quote(v.reason)
     <<",\"frame_id\":"<<v.frame_id<<",\"previous_frame_id\":"<<v.previous_frame_id
     <<",\"t0_ns\":"<<v.start_timestamp_ns<<",\"t1_ns\":"<<v.end_timestamp_ns<<",\"dt_s\":"<<jsonScalar(v.dt_s)
     <<",\"visual_valid\":"<<(v.visual_valid?"true":"false")<<",\"reference_valid\":"<<(v.reference_valid?"true":"false")
     <<",\"body_calibration_confirmed\":"<<(v.body_calibration_confirmed?"true":"false")
     <<",\"visual_reason\":"<<r33::json::quote(out.visual_motion.reason)<<",\"reference_reason\":"<<r33::json::quote(out.motion_reference.reason)
     <<",\"visual_forward_m\":"<<jsonScalar(v.visual_forward_m)<<",\"visual_lateral_m\":"<<jsonScalar(v.visual_lateral_m)
     <<",\"visual_yaw_rad\":"<<jsonScalar(v.visual_yaw_rad)<<",\"reference_forward_m\":"<<jsonScalar(v.reference_forward_m)
     <<",\"reference_lateral_m\":"<<jsonScalar(v.reference_lateral_m)<<",\"reference_yaw_rad\":"<<jsonScalar(v.reference_yaw_rad)
     <<",\"visual_speed_mps\":"<<jsonScalar(v.visual_speed_mps)<<",\"reference_speed_mps\":"<<jsonScalar(v.reference_speed_mps)
     <<",\"slip_ratio\":"<<(out.slip.available?jsonScalar(v.slip_ratio):jsonScalar(std::numeric_limits<double>::quiet_NaN()))
     <<",\"quality\":"<<jsonScalar(v.visual_quality)<<",\"rmse_m\":"<<jsonScalar(v.visual_rmse_m)
     <<",\"matches\":"<<v.matches<<",\"inliers\":"<<v.inliers<<",\"visual_ms\":"<<jsonScalar(v.visual_ms)
     <<",\"reference_ms\":"<<jsonScalar(v.reference_ms)<<",\"slip_ms\":"<<jsonScalar(v.slip_ms)
     <<",\"enter_elapsed_ms\":"<<jsonScalar(out.slip.enter_elapsed_ms)<<",\"exit_elapsed_ms\":"<<jsonScalar(out.slip.exit_elapsed_ms)<<'}';
    out.slip_json=o.str();
}
void refreshMaps(psdk_full_result& out) {
    if(!out.mapping)return;
    const auto& r=*out.mapping;const auto& c=r.calibration;
    out.calibration={std::uint32_t(c.active),std::uint32_t(c.completed),std::uint32_t(c.accumulated_frames),0,c.x,c.y,c.z,c.roll_degrees,c.pitch_degrees,c.yaw_degrees};
    if(r.has_maps) {
    const std::vector<r33::MappingCell>* cells[]={&r.pcl_map,&r.ai_map,&r.fusion_map};
    const r33::MappingRgbImage* images[]={&r.pcl_rgb,&r.ai_rgb,&r.fusion_rgb};
    const char* names[]={"pcl","ai","fusion"};
    for(int i=0;i<3;++i){auto& dst=out.map_cells[i];dst.reserve(cells[i]->size());for(const auto& v:*cells[i])dst.push_back({v.min_h,v.max_h,v.avg_h,v.type});
        auto& view=out.map_views[i];copyText(view.name,names[i]);view.rows=r.rows;view.cols=r.cols;view.resolution_mm=r.resolution_mm;
        view.cell_count=dst.size();view.cells=dst.data();view.image_width=images[i]->width;view.image_height=images[i]->height;
        view.rgb=images[i]->pixels.data();view.rgb_bytes=images[i]->pixels.size();}
    }
    const std::vector<r33::MappingPoint>* clouds[]={&r.camera_cloud,&r.robot_cloud,&r.ground_cloud,&r.obstacle_cloud,&r.semantic_cloud};
    const char* cloud_names[]={"camera","robot","ground","obstacle","semantic"};
    for(int i=0;i<5;++i){auto& dst=out.cloud_points[i];dst.reserve(clouds[i]->size());for(const auto& p:*clouds[i])dst.push_back({p.x,p.y,p.z,p.confidence,p.label,p.r,p.g,p.b,0});
        copyText(out.cloud_views[i].name,cloud_names[i]);out.cloud_views[i].points=dst.data();out.cloud_views[i].point_count=dst.size();}
}
psdk_full_result_view viewOf(const psdk_full_result& out) {
    psdk_full_result_view v{};v.struct_size=sizeof(v);v.abi_version=PSDK_FULL_ABI_VERSION;
    v.frame_id=out.frame.id;v.timestamp_ns=out.frame.timestamp_ns;v.run_id=out.run_id;
    v.has_inference=hasInference(out)?1:0;v.inference=inferenceView(out);
    const auto& rgb=rgbPixels(out);v.rgb=rgb.data();v.rgb_bytes=rgb.size();v.image_width=out.width;v.image_height=out.height;
    if(out.disparity){v.disparity={sizeof(psdk_disparity_q8),1,std::uint32_t(out.width),std::uint32_t(out.height),std::uint32_t(out.width*2),0,out.disparity->data(),out.disparity->size()*2,out.frame.id,out.frame.timestamp_ns};}
    v.recharge=out.pose;v.recharge.details_json=out.pose_details.c_str();v.recharge.overlay_rgb=out.recharge_rgb.data();v.recharge.overlay_bytes=out.recharge_rgb.size();
    v.calibration=out.calibration;v.graph_ms=out.graph_ms;v.process_ms=out.process_ms;v.task_trace_json=out.trace.c_str();v.effective_config_json=out.effective.c_str();
    if(out.mapping){const auto& r=*out.mapping;v.map_count=r.has_maps?3:0;v.maps=out.map_views.data();v.grid_count=r.type.size();
        v.avg_h_cm=r.avg_h_cm.data();v.min_h_cm=r.min_h_cm.data();v.max_h_cm=r.max_h_cm.data();v.grid_types=r.type.data();
        v.clouds=out.cloud_views.data();v.cloud_count=5;v.nearest_obstacle_valid=r.nearest_obstacle_valid;v.nearest_obstacle_index=r.nearest_obstacle_index;
        v.nearest_obstacle_x_m=r.nearest_obstacle_x_m;v.nearest_obstacle_y_m=r.nearest_obstacle_y_m;v.nearest_obstacle_distance_m=r.nearest_obstacle_distance_m;}
    return v;
}
std::string resultJson(const psdk_full_result& out) {
    using r33::json::quote;const auto v=viewOf(out);std::ostringstream o;o<<std::setprecision(12);
    o<<"{\"frame_id\":"<<v.frame_id<<",\"timestamp_ns\":"<<v.timestamp_ns<<",\"run_id\":"<<v.run_id
     <<",\"has_inference\":"<<(v.has_inference?"true":"false")<<",\"has_disparity\":"<<(v.disparity.data&&v.disparity.bytes?"true":"false")<<",\"map_count\":"<<v.map_count<<",\"history\":"<<out.temporal_json<<",\"timings_finalized\":"<<(out.finalized?"true":"false")
     <<",\"graph_ms\":"<<jsonScalar(v.graph_ms)<<",\"process_ms\":"<<jsonScalar(v.process_ms)
     <<",\"rgb_conversion_ms\":"<<jsonScalar(out.rgb?out.rgb->conversion_ms:0)
     <<",\"inference_execution\":"<<quote(out.detection||out.segmentation?"independent_lanes_serialized_legacy_exact_workspace":out.inference?"legacy_combined":"disabled_or_skipped")
     <<",\"detections\":[";
    for(std::uint64_t i=0;i<v.inference.detection_count;++i){const auto& d=v.inference.detections[i];if(i)o<<',';o<<"{\"class_id\":"<<d.class_id<<",\"name\":"<<quote(d.class_name)<<",\"score\":"<<jsonScalar(d.confidence)<<",\"x1\":"<<jsonScalar(d.x1)<<",\"y1\":"<<jsonScalar(d.y1)<<",\"x2\":"<<jsonScalar(d.x2)<<",\"y2\":"<<jsonScalar(d.y2)<<'}';}
    o<<"],\"recharge\":{\"executed\":"<<(v.recharge.executed?"true":"false")<<",\"ok\":"<<(v.recharge.mark_ok?"true":"false")<<",\"pose_ok\":"<<(v.recharge.pose_ok?"true":"false")
     <<",\"frame_id\":"<<v.recharge.frame_id<<",\"timestamp_ns\":"<<v.recharge.timestamp_ns<<",\"x\":"<<jsonScalar(v.recharge.x_m)<<",\"y\":"<<jsonScalar(v.recharge.y_m)<<",\"yaw\":"<<jsonScalar(v.recharge.yaw_rad)
     <<",\"error\":"<<quote(v.recharge.error)<<",\"details\":"<<out.pose_details<<"},\"calibration\":{\"active\":"<<v.calibration.active<<",\"completed\":"<<v.calibration.completed<<",\"accumulated_frames\":"<<v.calibration.accumulated_frames
     <<",\"x\":"<<jsonScalar(v.calibration.x_m)<<",\"y\":"<<jsonScalar(v.calibration.y_m)<<",\"z\":"<<jsonScalar(v.calibration.z_m)<<",\"roll\":"<<jsonScalar(v.calibration.roll_deg)<<",\"pitch\":"<<jsonScalar(v.calibration.pitch_deg)<<",\"yaw\":"<<jsonScalar(v.calibration.yaw_deg)<<"},\"grid\":{\"count\":"<<v.grid_count;
    if(v.map_count)o<<",\"rows\":"<<v.maps[0].rows<<",\"cols\":"<<v.maps[0].cols<<",\"resolution_mm\":"<<v.maps[0].resolution_mm;
    o<<",\"axes\":\"row:+robot_x, col:-robot_y; RGB rows reversed\"},\"nearest_obstacle\":{\"valid\":"<<v.nearest_obstacle_valid<<",\"index\":"<<v.nearest_obstacle_index
     <<",\"x_m\":"<<jsonScalar(v.nearest_obstacle_x_m)<<",\"y_m\":"<<jsonScalar(v.nearest_obstacle_y_m)<<",\"distance_m\":"<<jsonScalar(v.nearest_obstacle_distance_m)<<"}}\n";return o.str();
}
void saveResult(const psdk_full_result& out,const fs::path& directory) {
    if(fs::exists(directory))fail(2002,"Output directory already exists: "+directory.u8string(),"full.record",out.frame);
    if(!directory.parent_path().empty())fs::create_directories(directory.parent_path());
    if(!fs::create_directory(directory))fail(2002,"Cannot create NEW output directory","full.record",out.frame);
    const auto v=viewOf(out);text(directory/"result.json",resultJson(out));text(directory/"task_trace.json",out.trace+'\n');text(directory/"effective_config.json",out.effective+'\n');
    if(out.motion_enabled)text(directory/"slip.json",out.slip_json+'\n');
    text(directory/"temporal.json",out.temporal_json+'\n');
    if(v.has_inference)binary(directory/"mask.bin",v.inference.segmentation_mask,std::size_t(v.inference.segmentation_bytes));
    if(out.disparity)binary(directory/"disparity_q8.u16",out.disparity->data(),out.disparity->size()*2);
    if(!out.disparity_gray.empty()) {
        std::vector<std::uint8_t> rgb(out.disparity_gray.size()*3);for(std::size_t i=0;i<out.disparity_gray.size();++i)rgb[3*i]=rgb[3*i+1]=rgb[3*i+2]=out.disparity_gray[i];
        demo_vis::saveRgbPng(directory/"disparity.png",rgb,out.width,out.height);
    }
    const auto& rgb=rgbPixels(out);
    if(!rgb.empty())demo_vis::saveRgbPng(directory/"rgb.png",rgb,out.width,out.height);
    if(out.overlay&&hasInference(out)&&!rgb.empty()) {
        perception::DetectionResult det;perception::SegmentationResult seg;demo_vis::Style style;det.width=out.width;det.height=out.height;
        for(std::uint64_t i=0;i<v.inference.detection_count;++i){const auto& d=v.inference.detections[i];det.detections.push_back({d.class_id,d.confidence,d.x1,d.y1,d.x2,d.y2});}
        seg.width=out.width;seg.height=out.height;if(v.inference.segmentation_bytes)seg.mask.assign(v.inference.segmentation_mask,v.inference.segmentation_mask+v.inference.segmentation_bytes);
        auto overlay=rgb;demo_vis::overlayRgb(overlay,out.width,out.height,&det,&seg,style);demo_vis::saveRgbPng(directory/"overlay.png",overlay,out.width,out.height);
    }
    for(std::uint32_t i=0;i<v.map_count;++i){const auto& m=v.maps[i];const auto base=directory/m.name;
        binary(fs::u8path(base.u8string()+".cells.bin"),m.cells,std::size_t(m.cell_count)*sizeof(psdk_grid_cell));
        if(m.rgb_bytes){std::vector<std::uint8_t> pixels(m.rgb,m.rgb+std::size_t(m.rgb_bytes));demo_vis::saveRgbPng(fs::u8path(base.u8string()+".png"),pixels,m.image_width,m.image_height);}}
    if(v.grid_count){binary(directory/"grid_type.u8",v.grid_types,std::size_t(v.grid_count));binary(directory/"grid_avg_h_cm.i16",v.avg_h_cm,std::size_t(v.grid_count)*2);
        binary(directory/"grid_min_h_cm.i16",v.min_h_cm,std::size_t(v.grid_count)*2);binary(directory/"grid_max_h_cm.i16",v.max_h_cm,std::size_t(v.grid_count)*2);}
    for(std::uint32_t i=0;i<v.cloud_count;++i)if(v.clouds[i].point_count)binary(directory/(std::string(v.clouds[i].name)+".cloud.bin"),v.clouds[i].points,std::size_t(v.clouds[i].point_count)*sizeof(psdk_cloud_point));
    if(!out.recharge_rgb.empty())demo_vis::saveRgbPng(directory/"recharge.png",out.recharge_rgb,out.pose.overlay_width,out.pose.overlay_height);
    if(out.save_inputs&&!out.raw_input.empty())binary(directory/"source.yuv",out.raw_input.data(),out.raw_input.size());
    if(out.finalized)text(directory/"complete.json","{\"completed\":true,\"frame_id\":"+std::to_string(out.frame.id)+",\"run_id\":"+std::to_string(out.run_id)+"}\n");
}
void loadHostPolicies(psdk_full_engine& e) {
    using namespace r33::json;
    if(!e.config.mapping_config.empty()) {
        const auto original=load(e.config.mapping_config);
        if(auto d=optional(original,"dnn")){if(auto v=optional(*d,"enabled"))e.config.inference=e.config.inference&&boolean(*v);if(auto v=optional(*d,"run_every_n"))e.ai_every_n=integer(*v,1,1000000);}
    }
    if(!e.config.recharge)return;

}
#ifdef R33_WITH_RECHARGE
void runRecharge(psdk_full_engine& e,const psdk_full_frame& frame,psdk_full_result& out) {
    const auto tick=e.recharge_tick++;const auto now=Clock::now();
    if(tick%std::max(1u,e.recharge_host.run_every_n)||(e.last_recharge!=Clock::time_point{}&&milliseconds(e.last_recharge)<e.recharge_host.min_interval_ms))return;
    const psdk_frame* src=&frame.left;
    if(e.recharge_host.use_hires){if(frame.recharge_hires)src=frame.recharge_hires;else if(!e.config.allow_recharge_source_fallback)fail(1002,"Original use_hires requires same-frame GDC left; pass recharge_hires or explicitly allow_source_fallback","product.recharge_mark",out.frame);}
    const auto recharge_image=image(*src);if(frame.recharge_hires&&src==frame.recharge_hires)r33::stereo::validateHires(recharge_image,{src->frame_id,src->timestamp_ns},out.frame);if(src->format!=PSDK_FORMAT_NV12||src->frame_id!=frame.left.frame_id||src->timestamp_ns!=frame.left.timestamp_ns)fail(1002,"Recharge source must be same-frame NV12","product.recharge_mark",out.frame);
    if(std::string(e.recharge_host.intrinsics_source)!="dataset") {
        const auto& k=frame.intrinsics;if(!k.width||!k.height)fail(2001,"Scaled SDK recharge intrinsics require calibration geometry");
        std::string error;if(!e.recharge->setCameraMatrix(k.fx*src->width/k.width,k.fy*src->height/k.height,k.cx*src->width/k.width,k.cy*src->height/k.height,error))fail(2004,error,"product.recharge_mark",out.frame);
    }
    // charing-apriltag owns its static ROI; no YOLO-assisted external crop.
    r33::recharge::Output result;std::string error;
    e.last_recharge=now;
    if(!e.recharge->processLeftNv12(src->data,std::size_t(src->bytes),int(src->width),int(src->height),int(src->stride),src->frame_id,std::int64_t(src->timestamp_ns),nullptr,result,error))fail(2004,error,"product.recharge_mark",out.frame);
    const auto& p=result.pose;auto& v=out.pose;v.executed=1;v.mark_ok=p.ok;v.pose_ok=p.pose_ok;v.detected_point_count=p.detected_point_count;v.frame_id=p.frame_id;v.timestamp_ns=p.post.time_stamp;
    v.x_m=p.post.x;v.y_m=p.post.y;v.yaw_rad=p.post.yaw;v.score=p.score;v.distance_mm=p.distance_mm;v.reprojection_rmse_px=p.reprojection_rmse_px;
    std::copy(p.tvec_mm,p.tvec_mm+3,v.tvec_mm);std::copy(p.rvec,p.rvec+3,v.rvec);copyText(v.error,p.error);copyText(v.pnp_error,p.pnp_error);copyText(v.crop_source,p.crop_src);
    v.overlay_width=p.overlay_w;v.overlay_height=p.overlay_h;
    if(!result.overlay_bgr.empty()){out.recharge_rgb.resize(std::size_t(p.overlay_w)*p.overlay_h*3);for(int y=0;y<p.overlay_h;++y)for(int x=0;x<p.overlay_w;++x){const auto from=std::size_t(y)*p.overlay_step+x*3,to=(std::size_t(y)*p.overlay_w+x)*3;
        out.recharge_rgb[to]=result.overlay_bgr.at(from+2);out.recharge_rgb[to+1]=result.overlay_bgr.at(from+1);out.recharge_rgb[to+2]=result.overlay_bgr.at(from);}}
    using r33::json::quote;std::ostringstream details;details<<std::setprecision(12)<<"{\"center_u\":"<<jsonScalar(p.center_u)<<",\"center_v\":"<<jsonScalar(p.center_v)<<",\"image_angle_deg\":"<<jsonScalar(p.image_angle_deg)<<",\"process_ms\":"<<jsonScalar(p.process_ms)
      <<",\"debug_candidates\":"<<jsonScalar(p.debug_candidates)<<",\"debug_scaffolds\":"<<jsonScalar(p.debug_scaffolds)<<",\"trap_sign\":"<<jsonScalar(p.trap_sign)<<",\"cam_x_mm\":"<<jsonScalar(p.cam_x_mm)<<",\"raw_x_m\":"<<jsonScalar(p.raw_x_m)
      <<",\"x_filter_reason\":"<<quote(p.x_filter_reason)<<",\"roll_x_deg\":"<<jsonScalar(p.roll_x_deg)<<",\"pitch_y_deg\":"<<jsonScalar(p.pitch_y_deg)<<",\"yaw_z_deg\":"<<jsonScalar(p.yaw_z_deg)<<",\"heading_y_deg\":"<<jsonScalar(p.heading_y_deg)
      <<",\"tilt_x_deg\":"<<jsonScalar(p.tilt_x_deg)<<",\"image_roll_z_deg\":"<<jsonScalar(p.image_roll_z_deg)<<",\"corners\":[["<<jsonScalar(p.tl_u)<<','<<jsonScalar(p.tl_v)<<"],["<<jsonScalar(p.tr_u)<<','<<jsonScalar(p.tr_v)<<"],["<<jsonScalar(p.br_u)<<','<<jsonScalar(p.br_v)<<"],["<<jsonScalar(p.bl_u)<<','<<jsonScalar(p.bl_v)
      <<"]],\"has_corners\":["<<jsonScalar(p.has_tl)<<','<<jsonScalar(p.has_tr)<<','<<jsonScalar(p.has_br)<<','<<jsonScalar(p.has_bl)<<"],\"top\":["<<jsonScalar(p.top_u)<<','<<jsonScalar(p.top_v)<<"],\"bottom\":["<<jsonScalar(p.bot_u)<<','<<jsonScalar(p.bot_v)<<"],\"has_top\":"<<jsonScalar(p.has_top)<<",\"has_bottom\":"<<jsonScalar(p.has_bot)<<'}';out.pose_details=details.str();
}
#endif
}

namespace perception::sdk_internal {
std::uint32_t impl_psdk_full_capabilities(void){auto c=psdk_build_capabilities()|PSDK_CAP_RGBD_INPUT|PSDK_CAP_TEMPORAL|PSDK_CAP_SLIP;
#ifdef R33_WITH_MAPPING
    c|=PSDK_CAP_MAPPING;
#endif
#ifdef R33_WITH_RECHARGE
    c|=PSDK_CAP_RECHARGE;
#endif
    return c;
}
std::uint32_t impl_psdk_full_create(const psdk_full_config* config,psdk_full_engine** engine,psdk_status* error) {
    if(engine)*engine=nullptr;
    try {reentry();if(!abi(config)||!engine||!config->json_file)fail(2001,"Invalid full config ABI/JSON path");std::lock_guard<std::mutex> lock(full_gate);
        auto e=std::make_unique<psdk_full_engine>();e->config=r33::full::loadConfig(config->json_file);
        e->graph.configure(e->config.graph);
        e->history=r36::temporal::History(e->config.history);
        e->visual_motion=perception::motion::VisualMotion(e->config.visual_motion);
        e->motion_reference=perception::motion::MotionBuffer(e->config.motion_reference);
        e->slip_detector=perception::motion::SlipDetector(e->config.slip_detection);
        if(config->detection_model)e->config.det_model=config->detection_model;if(config->segmentation_model)e->config.seg_model=config->segmentation_model;
        if(config->hooks){if(!abi(config->hooks))fail(2001,"Invalid hooks ABI");e->hooks=*config->hooks;}
        loadHostPolicies(*e);
        if(e->config.mapping){
#ifdef R33_WITH_MAPPING
            r33::MappingConfig m;m.start_config_path=e->config.mapping_config;m.extrinsics_path=e->config.extrinsics;m.sample_step=e->config.sample_step;m.retain_clouds=e->config.clouds;m.render_maps=e->config.render_maps;m.calibration_output_path=e->config.calibration_output;
            if(e->config.graph.configured) {
                m.run_calibration=e->config.graph.enabled("calibration");m.run_labels=e->config.graph.enabled("labels");
                m.run_transform=e->config.graph.enabled("transform");m.run_ground=e->config.graph.enabled("ground");m.run_maps=e->config.graph.enabled("maps");
            }
            e->mapping=std::make_unique<r33::MappingPipeline>(m);
#else
            fail(2001,"Mapping was not compiled; build with R33_WITH_MAPPING=ON and real PCL/OpenCV/Eigen","full.build");
#endif
        }
        if(e->config.recharge){
#ifdef R33_WITH_RECHARGE
            e->recharge=std::make_unique<r33::recharge::Engine>();std::string message;if(!e->recharge->initialize(e->config.recharge_config.c_str(),message))fail(2004,message,"product.recharge_mark");
            e->recharge->getHostConfig(e->recharge_host);
#else
            fail(2001,"Recharge was not compiled; build with R33_WITH_RECHARGE=ON and shared OpenCV","full.build");
#endif
        }
        if(e->config.inference){
            const auto create_lane=[&](const std::string& dag,psdk_engine** engine,const char* feature){
                if(dag.empty())fail(2001,"A split inference instance requires its generated lane DAG",feature);
                psdk_config ai{};ai.struct_size=sizeof(ai);ai.abi_version=PSDK_ABI_VERSION;ai.backend=e->config.backend=="bpu"?PSDK_BACKEND_BPU:PSDK_BACKEND_CPU_ORT;
                ai.original_profile=e->config.original_profile.c_str();ai.dag_profile=dag.c_str();ai.cpu_profile=e->config.cpu_profile.empty()?nullptr:e->config.cpu_profile.c_str();
                ai.detection_model=e->config.det_model.empty()?nullptr:e->config.det_model.c_str();ai.segmentation_model=e->config.seg_model.empty()?nullptr:e->config.seg_model.c_str();
                psdk_status s{};check(psdk_create(&ai,engine,&s),s,feature);
            };
            if(e->config.graph.configured&&(e->config.graph.enabled("det")||e->config.graph.enabled("seg"))){
                if(e->config.graph.enabled("det"))create_lane(e->config.detection_dag,&e->detection,"product.ai.detection");
                if(e->config.graph.enabled("seg"))create_lane(e->config.segmentation_dag,&e->segmentation,"product.ai.segmentation");
            }else create_lane(e->config.ai_dag,&e->inference,"product.ai.exact");
        }
        *engine=e.release();return status(error);
    }catch(const std::exception& ex){return caught(error,ex,2001,"full.create",{});}catch(...){return rawStatus(error,2001,0,{},"full.create","","Unknown creation exception");}
}
std::uint32_t impl_psdk_full_process(psdk_full_engine* e,const psdk_full_frame* frame,psdk_full_result** result,psdk_status* error) {
    if(result)*result=nullptr;perception::FrameInfo identity{};
    try {reentry();if(!e||!result||!frame)fail(1002,"Invalid full frame/engine/result ABI");std::lock_guard<std::mutex> lock(full_gate);const auto start=Clock::now();
        perception::input::FrameBundle bundle;
        try{bundle=perception::input::FrameBundle::borrowed(*frame);}
        catch(const std::exception& ex){fail(1002,ex.what(),"full.input");}
        identity={frame->left.frame_id,frame->left.timestamp_ns};const auto source=image(frame->left);if(!identity.timestamp_ns||identity.timestamp_ns>std::uint64_t(std::numeric_limits<std::int64_t>::max()))fail(1002,"Full pipeline requires explicit nonzero same-frame timestamp","full.input",identity);
        if(e->config.command==21)fail(2001,"Perception stopped by command 21","product.control.command",identity);
        const bool map=(e->config.command==1||e->config.command==3)&&e->config.mapping,charge=(e->config.command==2||e->config.command==3)&&e->config.recharge;
        auto out=std::make_unique<psdk_full_result>();out->frame=identity;out->run_id=++e->runs;out->width=source.width;out->height=source.height;out->overlay=e->config.save_overlay;out->save_inputs=e->config.save_inputs;
        auto resources=std::make_shared<perception::FrameResources>(source,identity,[](const perception::ImageView& input){
            std::vector<std::uint8_t> pixels;demo_vis::convertRgbInto(input,pixels);return pixels;});
        std::once_flag rgb_once;
        const auto ensureRgb=[&]{std::call_once(rgb_once,[&]{
            const auto& rgb=resources->leftRgb();out->rgb=std::shared_ptr<const perception::RgbFrame>(resources,&rgb);
        });};
        const auto capture_start=Clock::now();r36::temporal::History::Plan history_plan;
        const bool history_enabled=!e->config.graph.configured||e->config.graph.enabled("history");
        try{if(history_enabled)history_plan=e->history.prepare(*frame);
            else{history_plan.window.current_id=identity.id;history_plan.window.current_timestamp=identity.timestamp_ns;history_plan.window.reset_reason="history disabled in graph";}}
        catch(const std::exception& ex){fail(1002,ex.what(),"product.temporal.history",identity);}
        out->temporal.capture_ms=milliseconds(capture_start);
        out->history=std::make_shared<const r36::temporal::Window>(history_plan.window);
        out->motion_enabled=e->config.graph.enabled("visual_motion")||e->config.graph.enabled("motion_reference")||e->config.graph.enabled("slip_detection");
        out->slip_enabled=e->config.graph.enabled("slip_detection");
        auto slip_candidate=e->slip_detector;
        auto reference_candidate=e->motion_reference;
        if(!history_plan.window.reset_reason.empty()&&history_plan.window.reset_reason!="duplicate_ignored")slip_candidate.reset();
        if(history_plan.window.reset_reason=="timestamp_discontinuity")reference_candidate.clear();
        if(out->save_inputs)out->raw_input.assign(frame->left.data,frame->left.data+std::size_t(frame->left.bytes));
        out->effective=r33::full::effectiveJson(e->config);
        r33::stereo::Intrinsics k{int(frame->intrinsics.width),int(frame->intrinsics.height),float(frame->intrinsics.fx),float(frame->intrinsics.fy),float(frame->intrinsics.cx),float(frame->intrinsics.cy),frame->intrinsics.bxf};
        r33::full::Graph::Bindings work;
        r33::full::Graph::ContextBindings contextual;
        // The new recharge donor has no YOLO crop dependency. Command 2 is
        // GDC-only, so inference remains on original RGBD inputs in modes 1/3.
        const bool run_ai=(e->inference||e->detection||e->segmentation)&&(e->config.command==1||e->config.command==3)&&
            (e->ai_tick++%std::uint64_t(e->ai_every_n)==0);
        if(run_ai&&e->inference)work["dnn"]=[&]{psdk_status s{};check(psdk_process(e->inference,&frame->left,&out->inference,&s),s,"product.ai.exact",identity);
            out->inference_view.struct_size=sizeof(psdk_result_view);out->inference_view.abi_version=1;check(psdk_result_get_view(out->inference,&out->inference_view,&s),s,"product.ai.exact",identity);};
        const auto bind_lane=[&](int lane,const char* key,psdk_engine* engine,psdk_result** result,psdk_result_view* view,const char* feature){
            if(!run_ai||!engine)return;
            contextual[key]=[&,lane,engine,result,view,feature](perception::runtime::FeatureContext& context){
                const auto input=context.get<psdk_frame>("input.left");psdk_status s{};
                out->inference_begin_ms[lane]=milliseconds(start);
                check(psdk_process(engine,input.get(),result,&s),s,feature,identity);
                view->struct_size=sizeof(*view);view->abi_version=PSDK_ABI_VERSION;
                check(psdk_result_get_view(*result,view,&s),s,feature,identity);
                out->inference_end_ms[lane]=milliseconds(start);
            };
        };
        bind_lane(0,"det",e->detection,&out->detection,&out->detection_view,"product.ai.detection");
        bind_lane(1,"seg",e->segmentation,&out->segmentation,&out->segmentation_view,"product.ai.segmentation");
#ifdef R33_WITH_RECHARGE
        if(charge)work["recharge"]=[&]{runRecharge(*e,*frame,*out);};
#endif
        const bool disparity=(e->config.command==1||e->config.command==3)&&
            (map||(e->config.graph.configured&&e->config.graph.enabled("disparity")));
        if(disparity){contextual["disparity"]=[&](perception::runtime::FeatureContext& context){
            const auto left=context.get<psdk_frame>("input.left");
            psdk_disparity_q8 supplied{};std::shared_ptr<const psdk_disparity_q8> original;
            if(context.contains("input.disparity_q8"))original=context.get<psdk_disparity_q8>("input.disparity_q8");
            const auto* d=original.get();
            if(e->config.disparity_source=="raft_stereo"&&d)
                fail(1002,"raft_stereo rejects supplied original Q8; provide left/right and a Stereo model provider","product.stereo.disparity",identity);
            if(e->config.disparity_source=="original_q8"&&!d)
                fail(1002,"original_q8 requires genuine same-frame input.disparity_q8; model fallback is forbidden","product.stereo.disparity",identity);
            if(e->config.disparity_source=="raft_stereo"&&!e->hooks.disparity_provider)
                fail(1002,"raft_stereo requires a Stereo model provider","product.stereo.disparity",identity);
            if(!d&&(e->config.disparity_source=="raft_stereo"||e->config.graph.stereo_inference)){
                if(!context.contains("input.right"))fail(1002,"Stereo inference requires a same-frame right image input","product.stereo.disparity",identity);
                const auto right=context.get<psdk_frame>("input.right");
                if(right->frame_id!=left->frame_id||right->timestamp_ns!=left->timestamp_ns||right->width!=left->width||right->height!=left->height)
                    fail(1002,"Stereo input artifact pair has mismatched identity or geometry","product.stereo.disparity",identity);
            }
            if(!d&&e->hooks.disparity_provider){supplied.struct_size=sizeof(supplied);supplied.abi_version=1;psdk_status s{};CallbackGuard callback;check(e->hooks.disparity_provider(e->hooks.userdata,left.get(),&supplied,&s),s,"product.stereo.disparity",identity);d=&supplied;}
            if(!abi(d)||d->reserved||d->bytes>std::numeric_limits<std::size_t>::max())fail(1002,"Disparity stage requires genuine same-frame uint16 Q8 disparity","product.stereo.disparity",identity);
            r33::stereo::Q8View view{d->data,std::size_t(d->bytes),d->stride_bytes,int(d->width),int(d->height),{d->frame_id,d->timestamp_ns}};
            out->disparity=r33::stereo::copyDisparity(source,identity,view,k);
            out->disparity_gray.resize(out->disparity->size());for(std::size_t i=0;i<out->disparity->size();++i){const float value=std::clamp(float((*out->disparity)[i])/256.f,1.f,144.f);out->disparity_gray[i]=std::uint8_t(std::clamp(std::nearbyint((value-1.f)/143.f*255.f),0.f,255.f));}
        };}
        // Kept only for legacy explicit rgb graphs. New graphs request this
        // common frame resource from its actual consumers instead.
        if(e->config.graph.configured&&e->config.graph.enabled("rgb"))work["rgb"]=ensureRgb;
#ifdef R33_WITH_MAPPING
        std::string mapping_last="maps";
        if(e->config.graph.configured)for(const auto* key:{"cloud","calibration","labels","transform","ground","maps"})
            if(e->config.graph.enabled(key))mapping_last=key;
        const auto collect_mapping=[&](const char* stage){if(mapping_last==stage){out->mapping=e->mapping->result();refreshMaps(*out);
            out->effective="{\"sdk\":"+r33::full::effectiveJson(e->config)+",\"mapping\":"+e->mapping->effectiveConfigJson()+"}";}};
        if(map){work["cloud"]=[&]{ensureRgb();r33::MappingFrame m;m.frame_id=identity.id;m.width=source.width;m.height=source.height;m.disparity=out->disparity->data();m.disparity_bytes=out->disparity->size()*2;m.disparity_stride_samples=source.width;
            const auto& rgb=rgbPixels(*out);
            m.intrinsics={k.fx,k.fy,k.cx,k.cy,float(k.bxf/1000.0f)};m.rgb=rgb.data();m.rgb_bytes=rgb.size();
            e->mapping->beginFrame(m);e->mapping->cloud();collect_mapping("cloud");};
            work["calibration"]=[&]{e->mapping->calibration();collect_mapping("calibration");};
            work["labels"]=[&]{
                r33::MappingFrame semantic;semantic.frame_id=identity.id;semantic.width=source.width;semantic.height=source.height;
                semantic.semantic_result_valid=hasInference(*out);semantic.semantic_frame_id=identity.id;
                const auto ai=inferenceView(*out);semantic.segmentation_mask=ai.segmentation_mask;semantic.segmentation_bytes=std::size_t(ai.segmentation_bytes);
                semantic.segmentation_width=source.width;semantic.segmentation_height=source.height;
                for(std::uint64_t i=0;i<ai.detection_count;++i){const auto& d=ai.detections[i];semantic.detections.push_back({d.class_id,d.confidence,d.x1,d.y1,d.x2,d.y2});}
                e->mapping->setSemantic(semantic);e->mapping->labels();collect_mapping("labels");};
            work["transform"]=[&]{e->mapping->transform();collect_mapping("transform");};
            work["ground"]=[&]{e->mapping->ground();collect_mapping("ground");};
            work["maps"]=[&]{e->mapping->maps();collect_mapping("maps");};}
#endif
        if(e->hooks.publication)work["publish"]=[&]{if(out->overlay)ensureRgb();refreshHistory(*out);refreshSlip(*out);const auto view=viewOf(*out);psdk_status s{};CallbackGuard callback;check(e->hooks.publication(e->hooks.userdata,&view,&s),s,"product.output.application",identity);};
        fs::path recorded;
        if(e->config.record_every_n&&(out->run_id-1)%std::uint64_t(e->config.record_every_n)==0)work["record"]=[&]{if(out->overlay)ensureRgb();refreshHistory(*out);refreshSlip(*out);recorded=fs::u8path(e->config.output_directory)/("frame-"+std::to_string(out->run_id)+"-"+std::to_string(identity.id)+"-"+std::to_string(identity.timestamp_ns));saveResult(*out,recorded);};
        for(const auto& job:work)contextual[job.first]=[f=job.second](perception::runtime::FeatureContext&){f();};
        if(history_enabled)contextual["history"]=[&](perception::runtime::FeatureContext& context){
            const auto capture=Clock::now();auto disparity=out->disparity;
            if(history_plan.window.sampled&&!disparity&&frame->disparity){
                const auto* d=frame->disparity;
                if(!abi(d)||d->reserved||d->bytes>std::numeric_limits<std::size_t>::max())fail(1002,"Invalid temporal disparity ABI","product.temporal.history",identity);
                r33::stereo::Q8View v{d->data,std::size_t(d->bytes),d->stride_bytes,int(d->width),int(d->height),{d->frame_id,d->timestamp_ns}};
                disparity=r33::stereo::copyDisparity(source,identity,v,k);
            }
            const auto ai=inferenceView(*out);
            e->history.complete(history_plan,hasInference(*out)?&ai:nullptr,std::move(disparity));
            out->history=std::make_shared<const r36::temporal::Window>(history_plan.window);
            out->temporal.capture_ms+=milliseconds(capture);refreshHistory(*out);
            // A task-local transaction publishes this immutable completed
            // window to all dependent Features only after this Task succeeds.
            context.set(r36::temporal::artifact_key,*out->history);
        };
        if(e->config.graph.enabled("visual_motion"))contextual["visual_motion"]=[&](perception::runtime::FeatureContext& context){
            const auto start=Clock::now();const auto window=context.get<r36::temporal::Window>(r36::temporal::artifact_key);
            out->visual_motion=e->visual_motion.estimate(*window);out->visual_ms=milliseconds(start);
            context.set("motion.visual",out->visual_motion);
        };
        if(e->config.graph.enabled("motion_reference"))contextual["motion_reference"]=[&](perception::runtime::FeatureContext& context){
            const auto start=Clock::now();const auto window=context.get<r36::temporal::Window>(r36::temporal::artifact_key);
            if(!e->config.motion_same_clock_confirmed)out->motion_reference.reason="reference_clock_not_confirmed";
            else if(window->sampled&&window->frames.size()>=2) {
                const auto n=window->frames.size();out->motion_reference=reference_candidate.integrate(window->frames[n-2]->left.metadata.timestamp_ns,window->frames[n-1]->left.metadata.timestamp_ns);
            }else out->motion_reference.reason="insufficient_history_or_not_sampled";
            out->reference_ms=milliseconds(start);context.set("motion.reference",out->motion_reference);
        };
        if(out->slip_enabled)contextual["slip_detection"]=[&](perception::runtime::FeatureContext& context){
            const auto start=Clock::now();const auto visual=context.get<perception::motion::VisualResult>("motion.visual");
            const auto reference=context.get<perception::motion::ReferenceResult>("motion.reference");
            if(history_plan.window.sampled)out->slip=slip_candidate.update(*visual,*reference);
            else out->slip.reason="not_sampled";
            out->slip_ms=milliseconds(start);refreshSlip(*out);context.set("slip.result",out->slip);
        };
        if(history_plan.window.sampled&&e->config.history.capacity>1&&e->config.history.difference)
            contextual["temporal_difference"]=[&](perception::runtime::FeatureContext& context){
                auto window=context.get<r36::temporal::Window>(r36::temporal::artifact_key);
                r36::temporal::difference(*window,e->config.history,out->temporal);
            };
        if(history_plan.window.sampled&&e->config.history.capacity>1&&e->config.history.match)
            contextual["temporal_match"]=[&](perception::runtime::FeatureContext& context){
                auto window=context.get<r36::temporal::Window>(r36::temporal::artifact_key);
                r36::temporal::match(*window,e->config.history,out->temporal);
            };
        // The optional extension joins analyses before invoking its callback.
        // Output serialization also refreshes when this node is disabled.
        contextual["temporal_extension"]=[&](perception::runtime::FeatureContext&){
            refreshHistory(*out);
            if(history_plan.window.sampled&&e->temporal_callback){const auto v=temporalView(*out);psdk_status s{};
                CallbackGuard callback;check(e->temporal_callback(e->temporal_userdata,&v,&s),s,"product.temporal.extension",identity);}
        };
        perception::runtime::GraphReport report;r28::Status failure;
        const auto code=e->graph.runContext(contextual,identity,report,failure,out->history,[&](perception::runtime::ArtifactStore& artifacts){bundle.seed(artifacts);});out->graph_ms=report.elapsed_ms;out->trace=r33::full::traceJson(report,out->run_id);out->finalized=true;
        if(code)return status(error,failure);
        if(out->overlay)ensureRgb();
        out->process_ms=milliseconds(start);
        // Output serialization must also be current when the optional extension
        // node is disabled or no publication/record hook is installed.
        refreshHistory(*out);
        if(!recorded.empty()){text(recorded/"final_task_trace.json",out->trace+'\n');text(recorded/"complete.json","{\"completed\":true,\"frame_id\":"+std::to_string(identity.id)+",\"run_id\":"+std::to_string(out->run_id)+",\"graph_ms\":"+std::to_string(out->graph_ms)+",\"process_ms\":"+std::to_string(out->process_ms)+"}\n");}
        refreshSlip(*out);
        if(history_enabled)e->history.commit(history_plan);
        if(out->motion_enabled){e->slip_detector=std::move(slip_candidate);e->motion_reference=std::move(reference_candidate);}
        *result=out.release();return status(error,{0,0,{},{},{},identity});
    }catch(const std::exception& ex){return caught(error,ex,1011,"full.process",identity);}catch(...){return rawStatus(error,1011,0,identity,"full.process","","Unknown processing exception");}
}
std::uint32_t impl_psdk_full_result_get_view(const psdk_full_result* result,psdk_full_result_view* view,psdk_status* error){
    try{if(!result||!abi(view))fail(1012,"Invalid full result view ABI");*view=viewOf(*result);return status(error,{0,0,{},{},{},result->frame});}catch(const std::exception& ex){return caught(error,ex,1012,"full.result",{});}catch(...){return rawStatus(error,1012,0,{},"full.result","","Unknown view exception");}}
std::uint32_t impl_psdk_full_command(psdk_full_engine* e,std::int32_t command,psdk_status* error){
    try{reentry();if(!e)fail(2001,"Engine required");std::lock_guard<std::mutex> lock(full_gate);r30::product::commandPlan(command,e->config.recharge);
        e->config.command=command;e->ai_tick=0;e->recharge_tick=0;e->last_recharge={};e->history.clear();e->slip_detector.reset();e->motion_reference.clear();
#ifdef R33_WITH_RECHARGE
        if(e->recharge)e->recharge->resetSession();
#endif
        return status(error);
    }catch(const std::exception& ex){return caught(error,ex,2001,"product.control.command");}catch(...){return rawStatus(error,2001,0,{},"product.control.command","","Unknown command exception");}}
std::uint32_t impl_psdk_full_save_result(const psdk_full_result* result,const char* directory,psdk_status* error){
    try{if(!result||!directory||!*directory)fail(2002,"Result and NEW output directory required");saveResult(*result,fs::u8path(directory));return status(error,{0,0,{},{},{},result->frame});}catch(const std::exception& ex){return caught(error,ex,2002,"full.record",result?result->frame:perception::FrameInfo{});}catch(...){return rawStatus(error,2002,0,{},"full.record","","Unknown save exception");}}
void impl_psdk_full_result_release(psdk_full_result* result){try{delete result;}catch(...){}}
std::uint32_t impl_psdk_full_result_get_temporal_view(const psdk_full_result* result,psdk_temporal_view* view,psdk_status* error){
    try{if(!result||!view||view->struct_size!=sizeof(*view)||view->abi_version!=PSDK_TEMPORAL_ABI_VERSION)fail(1012,"Invalid temporal result view ABI");
        *view=temporalView(*result);return status(error,{0,0,{},{},{},result->frame});
    }catch(const std::exception& ex){return caught(error,ex,1012,"full.temporal.result");}catch(...){return rawStatus(error,1012,0,{},"full.temporal.result","","Unknown temporal view exception");}}
std::uint32_t impl_psdk_full_set_temporal_callback(psdk_full_engine* e,psdk_temporal_callback callback,void* userdata,psdk_status* error){
    try{reentry();if(!e)fail(2001,"Engine required");std::lock_guard<std::mutex> lock(full_gate);e->temporal_callback=callback;e->temporal_userdata=userdata;return status(error);
    }catch(const std::exception& ex){return caught(error,ex,2001,"full.temporal.callback");}catch(...){return rawStatus(error,2001,0,{},"full.temporal.callback","","Unknown callback exception");}}
std::uint32_t impl_psdk_full_reset_history(psdk_full_engine* e,psdk_status* error){
    try{reentry();if(!e)fail(2001,"Engine required");std::lock_guard<std::mutex> lock(full_gate);e->history.clear();e->slip_detector.reset();e->motion_reference.clear();return status(error);
    }catch(const std::exception& ex){return caught(error,ex,2001,"full.temporal.reset");}catch(...){return rawStatus(error,2001,0,{},"full.temporal.reset","","Unknown history reset exception");}}
std::uint32_t impl_psdk_full_destroy(psdk_full_engine** engine,psdk_status* error){
    try{reentry();if(!engine)fail(2001,"Engine output pointer required");std::lock_guard<std::mutex> lock(full_gate);std::unique_ptr<psdk_full_engine> owned(*engine);*engine=nullptr;if(!owned)return status(error);
        psdk_status first{};std::uint32_t code=0;
        for(auto** lane:{&owned->inference,&owned->detection,&owned->segmentation}) {
            psdk_status current{};const auto failure=psdk_destroy(lane,&current);
            if(failure&&!code){code=failure;first=current;}
        }
        owned.reset();return code?status(error,fromPublic(first,"full.destroy",{})):status(error);
    }catch(const std::exception& ex){return caught(error,ex,2006,"full.destroy",{});}catch(...){return rawStatus(error,2006,0,{},"full.destroy","","Unknown destroy exception");}}
std::uint32_t impl_psdk_full_push_motion_samples(psdk_full_engine* e,const psdk_motion_sample* samples,std::uint32_t count,psdk_status* error) {
    try {reentry();if(!e||(!samples&&count)||count>8192)fail(1002,"Engine and valid motion sample batch (<=8192) required","full.motion.input");
        std::lock_guard<std::mutex> lock(full_gate);auto candidate=e->motion_reference;
        for(std::uint32_t i=0;i<count;++i){const auto& s=samples[i];
            if(s.struct_size!=sizeof(s)||s.abi_version!=PSDK_SLIP_ABI_VERSION||s.reserved[0]||s.reserved[1]||
                !s.timestamp_ns||s.timestamp_ns>std::uint64_t(std::numeric_limits<std::int64_t>::max())||
                !std::isfinite(s.linear_speed_mps)||!std::isfinite(s.yaw_rate_radps))fail(1002,"Invalid motion sample ABI/timestamp/finite rates","full.motion.input");
            std::string reason;if(!candidate.push({s.timestamp_ns,s.linear_speed_mps,s.yaw_rate_radps},&reason))fail(1002,reason,"full.motion.input");
        }
        e->motion_reference=std::move(candidate);return status(error);
    }catch(const std::exception& ex){return caught(error,ex,1002,"full.motion.input");}catch(...){return rawStatus(error,1002,0,{},"full.motion.input","","Unknown motion input exception");}
}
std::uint32_t impl_psdk_full_result_get_slip_view(const psdk_full_result* result,psdk_slip_view* view,psdk_status* error) {
    try {if(!result||!view||view->struct_size!=sizeof(*view)||view->abi_version!=PSDK_SLIP_ABI_VERSION)fail(1012,"Invalid slip result view ABI","full.slip.result");
        *view=slipView(*result);return status(error,{0,0,{},{},{},result->frame});
    }catch(const std::exception& ex){return caught(error,ex,1012,"full.slip.result");}catch(...){return rawStatus(error,1012,0,{},"full.slip.result","","Unknown slip view exception");}
}
}
