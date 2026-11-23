#include "slip_timeline.hpp"
#include "native_pipeline.hpp"
#include "intrinsics_file.hpp"
#include "perception_slip_sdk.h"
#include "platform_metrics.hpp"
#include "bpu_metrics.hpp"
#ifdef R36_SLIP_WITH_CAMERA
#include "camera_capture_util.hpp"
#endif
#ifdef R36_SLIP_WITH_HORIZON
#include "perception_ros_so_demo/stereo_provider.hpp"
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace fs=std::filesystem;
namespace app=perception::slip_app;
using Clock=std::chrono::steady_clock;
namespace {
volatile std::sig_atomic_t interrupted=0;
void stop(int){interrupted=1;}
struct Interrupted:std::runtime_error {Interrupted():std::runtime_error("Run interrupted") {}};
const v3::Json& get(const v3::Json& object,const char* name){return r33::json::required(object,name);}
std::string text(const v3::Json& object,const char* name,const std::string& fallback={}) {
    const auto* value=r33::json::optional(object,name);
    if(!value||value->kind==v3::Json::Kind::Null)return fallback;
    if(value->kind!=v3::Json::Kind::String||value->text.find('\0')!=std::string::npos)
        throw std::invalid_argument(std::string("Expected JSON string: ")+name);
    return value->text;
}
int integer(const v3::Json& object,const char* name,int fallback) {
    const auto* value=r33::json::optional(object,name);
    return value?r33::json::integer(*value,0,1000000):fallback;
}
std::string quote(const std::string& value) {
    v3::Json out;out.kind=v3::Json::Kind::String;out.text=value;return v3::jsonText(out,0);
}
std::string number(double value,bool valid=true) {
    if(!valid||!std::isfinite(value))return "null";
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(15)<<value;return out.str();
}
double ms(Clock::time_point start) {return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
psdk_status statusValue(){psdk_status value{};value.struct_size=sizeof(value);value.abi_version=PSDK_ABI_VERSION;return value;}
std::string fixed(const char* data,std::size_t capacity){const auto* end=std::find(data,data+capacity,'\0');return {data,end};}
void check(std::uint32_t code,const psdk_status& status,const char* label) {
    if(code)throw std::runtime_error(std::string(label)+" failed code="+std::to_string(code)+
        " native="+std::to_string(status.native_code)+" feature="+fixed(status.feature,sizeof(status.feature))+
        " task="+fixed(status.task,sizeof(status.task))+" detail="+fixed(status.detail,sizeof(status.detail)));
}
void write(const fs::path& file,const std::string& contents) {
    std::ofstream stream(file,std::ios::binary);stream<<contents<<'\n';stream.close();
    if(!stream)throw std::runtime_error("Cannot write "+file.u8string());
}
struct Engine {psdk_full_engine* value=nullptr;~Engine(){psdk_full_destroy(&value,nullptr);}};
struct Result {psdk_full_result* value=nullptr;~Result(){psdk_full_result_release(value);}};
struct Options {fs::path config,output;bool check_only=false;};
Options options(int argc,char** argv) {
    Options out;std::set<std::string> seen;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(!seen.insert(key).second)throw std::invalid_argument("Duplicate argument: "+key);
        if(key=="--check"){out.check_only=true;continue;}
        if((key!="--config"&&key!="--output")||i+1==argc)throw std::invalid_argument("Expected --config MAIN.json --output NEW_DIRECTORY [--check]");
        const auto value=fs::u8path(argv[++i]);if(value.empty())throw std::invalid_argument("Empty argument: "+key);
        (key=="--config"?out.config:out.output)=value;
    }
    if(out.config.empty()||out.output.empty())throw std::invalid_argument("--config and --output are required");
    out.config=fs::absolute(out.config).lexically_normal();out.output=fs::absolute(out.output).lexically_normal();
    if(!fs::is_regular_file(out.config))throw std::invalid_argument("Main JSON does not exist");
    if(fs::exists(out.output)||fs::is_symlink(out.output))throw std::invalid_argument("Output must be a NEW directory");
    return out;
}
psdk_camera_intrinsics intrinsics(const v3::Json& value) {
    psdk_camera_intrinsics k{};
    k.width=std::uint32_t(r33::json::integer(get(value,"width"),2,4096));
    k.height=std::uint32_t(r33::json::integer(get(value,"height"),2,4096));
    k.fx=r33::json::number(get(value,"fx"));k.fy=r33::json::number(get(value,"fy"));
    k.cx=r33::json::number(get(value,"cx"));k.cy=r33::json::number(get(value,"cy"));k.bxf=r33::json::number(get(value,"bxf"));
    if(k.width!=640||k.height!=544||k.fx<=0||k.fy<=0||k.bxf<=0||k.cx<0||k.cy<0||k.cx>=640||k.cy>=544)
        throw std::invalid_argument("Slip input requires explicit valid rectified 640x544 intrinsics");
    return k;
}
psdk_frame image(const std::vector<std::uint8_t>& bytes,std::uint64_t id,std::uint64_t timestamp) {
    psdk_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_ABI_VERSION;
    frame.format=PSDK_FORMAT_NV12;frame.width=640;frame.height=544;frame.stride=640;
    frame.data=bytes.data();frame.bytes=bytes.size();frame.frame_id=id;frame.timestamp_ns=timestamp;return frame;
}
std::string slipJson(const psdk_slip_view& s,const std::string& source,std::uint64_t capture_timestamp) {
    const bool evaluated=s.state==PSDK_SLIP_NORMAL||s.state==PSDK_SLIP_SUSPECTED||
        s.state==PSDK_SLIP_SLIPPING||s.state==PSDK_SLIP_RECOVERING;
    std::ostringstream out;out<<"{\"frame_id\":"<<s.frame_id<<",\"timestamp_ns\":"<<capture_timestamp
       <<",\"frame_id_source\":"<<quote(source)<<",\"previous_frame_id\":"<<s.previous_frame_id
       <<",\"start_timestamp_ns\":"<<s.start_timestamp_ns<<",\"end_timestamp_ns\":"<<s.end_timestamp_ns
       <<",\"state\":"<<s.state<<",\"state_name\":"<<quote(fixed(s.state_name,sizeof(s.state_name)))
       <<",\"enabled\":"<<(s.enabled?"true":"false")<<",\"visual_valid\":"<<(s.visual_valid?"true":"false")
       <<",\"reference_valid\":"<<(s.reference_valid?"true":"false")
       <<",\"body_calibration_confirmed\":"<<(s.body_calibration_confirmed?"true":"false")
       <<",\"dt_s\":"<<number(s.dt_s)<<",\"visual_forward_m\":"<<number(s.visual_forward_m,s.visual_valid)
       <<",\"visual_lateral_m\":"<<number(s.visual_lateral_m,s.visual_valid)<<",\"visual_yaw_rad\":"<<number(s.visual_yaw_rad,s.visual_valid)
       <<",\"reference_forward_m\":"<<number(s.reference_forward_m,s.reference_valid)
       <<",\"reference_lateral_m\":"<<number(s.reference_lateral_m,s.reference_valid)<<",\"reference_yaw_rad\":"<<number(s.reference_yaw_rad,s.reference_valid)
       <<",\"visual_speed_mps\":"<<number(s.dt_s>0?s.visual_forward_m/s.dt_s:0,s.visual_valid&&s.dt_s>0)
       <<",\"reference_speed_mps\":"<<number(s.dt_s>0?s.reference_forward_m/s.dt_s:0,s.reference_valid&&s.dt_s>0)
       <<",\"slip_ratio\":"<<number(s.slip_ratio,evaluated)
       <<",\"visual_quality\":"<<number(s.visual_quality,s.visual_valid)<<",\"visual_rmse_m\":"<<number(s.visual_rmse_m,s.visual_valid)
       <<",\"matches\":"<<s.matches<<",\"inliers\":"<<s.inliers
       <<",\"visual_ms\":"<<number(s.visual_ms)<<",\"reference_ms\":"<<number(s.reference_ms)<<",\"slip_ms\":"<<number(s.slip_ms)
       <<",\"reason\":"<<quote(fixed(s.reason,sizeof(s.reason)))<<",\"details\":"<<(s.details_json?s.details_json:"null")<<'}';return out.str();
}
int run(const Options& o) {
    // Validate before creating output or opening camera/model resources.
    auto prepared=v3::prepare(o.config,o.output,false);
    if(prepared.mode!="camera"&&prepared.mode!="recording")throw std::invalid_argument("Slip runner supports camera or timestamped recording input only");
    const bool original=prepared.method=="original_q8";
    if(!original&&prepared.method!="raft_stereo")throw std::invalid_argument("Unknown disparity source");
#ifndef R36_SLIP_WITH_HORIZON
    if(!original)throw std::invalid_argument("This slip runner was built without real Horizon Stereo; select original_q8 or rebuild with R36_SLIP_WITH_HORIZON=ON");
#endif
#ifndef R36_SLIP_WITH_CAMERA
    if(prepared.mode=="camera")throw std::invalid_argument("Camera requires the Linux camera SDK build; Windows supports timestamped recordings");
#endif
    std::vector<app::TimelineRow> timeline;std::vector<app::MotionRow> motion;
    psdk_camera_intrinsics k{};
    if(prepared.mode=="recording") {
        const auto file=text(prepared.slip_input,"timeline_csv");
        if(file.empty())throw std::invalid_argument("Recording requires input.timeline_csv with original capture timestamps");
        app::TimelineOptions parsing;parsing.original_q8=original;
        timeline=app::readTimeline(fs::u8path(file),parsing);
        const auto calibration=text(prepared.recording,"intrinsics_file");
        if(calibration.empty())throw std::invalid_argument("Recording requires explicit recording.intrinsics_file");
        k=intrinsics(v3::loadIntrinsicsFile(fs::u8path(calibration),true));
    }
    const auto motion_file=text(prepared.slip_input,"motion_reference_csv");
    if(!motion_file.empty()) {
        const auto* confirmed=r33::json::optional(prepared.slip_input,"same_clock_confirmed");
        if(!confirmed||!r33::json::boolean(*confirmed)||text(prepared.slip_input,"clock_domain").empty())
            throw std::invalid_argument("Motion CSV requires clock_domain and same_clock_confirmed=true; receipt/playback clocks are forbidden");
        motion=app::readMotionCsv(fs::u8path(motion_file));
    }
    const int max_frames=prepared.mode=="recording"?integer(prepared.recording,"limit",100):integer(prepared.slip_input,"max_frames",100);
    if(o.check_only) {
        std::cout<<"Slip configuration valid; capture_rows="<<timeline.size()<<" motion_samples="<<motion.size()
            <<" input="<<prepared.mode<<" disparity="<<prepared.method<<"; no model/camera opened\n";return 0;
    }
    auto generated=v3::prepare(o.config,o.output,true);
    if(v3::jsonText(generated.requested)!=v3::jsonText(prepared.requested)||
       v3::jsonText(generated.full)!=v3::jsonText(prepared.full)||
       v3::jsonText(generated.parameters)!=v3::jsonText(prepared.parameters)||
       v3::jsonText(generated.recording)!=v3::jsonText(prepared.recording)||
       v3::jsonText(generated.stereo)!=v3::jsonText(prepared.stereo)||
       v3::jsonText(generated.slip_input)!=v3::jsonText(prepared.slip_input))
        throw std::runtime_error("Configuration changed during input validation; start a new run with one fixed configuration");
    prepared=std::move(generated);
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    std::uint64_t duplicate_captures=0;
#ifdef R36_SLIP_WITH_CAMERA
    std::unique_ptr<ros_so_demo::raw::Camera> camera;
    ros_so_demo::raw::CameraOptions camera_options;
    std::optional<ros_so_demo::raw::Identity> previous;
    if(prepared.mode=="camera") {
        camera_options.sdk=text(prepared.parameters,"camera_sdk");camera_options.config=text(prepared.parameters,"camera_config");
        camera_options.need_q8=original;camera_options.need_gdc=false;
        camera_options.disparity_mode=integer(prepared.parameters,"camera_disparity_mode",original?2:1);
        if(camera_options.disparity_mode!=(original?2:1))throw std::invalid_argument("original_q8 requires camera mode2, new Stereo requires mode1; no fallback");
        camera_options.timeout_ms=integer(prepared.parameters,"acquire_timeout_ms",3000);
        camera_options.retry_ms=integer(prepared.parameters,"retry_interval_ms",5);
        if(!camera_options.timeout_ms||!camera_options.retry_ms||camera_options.retry_ms>camera_options.timeout_ms)
            throw std::invalid_argument("Invalid camera timeout/retry");
        camera=std::make_unique<ros_so_demo::raw::Camera>(camera_options);
        const auto actual=r36compare::runner::cameraIntrinsics(camera->parameters());
        k={640,544,actual.fx,actual.fy,actual.cx,actual.cy,actual.bxf};
    }
#endif
#ifdef R36_SLIP_WITH_HORIZON
    std::unique_ptr<ros_so_demo::StereoProvider> stereo;
    if(!original) {
        ros_so_demo::StereoSettings settings;settings.model=text(prepared.stereo,"model");
        settings.input_mode=text(prepared.stereo,"input_mode","crop");
        settings.input_width=integer(prepared.stereo,"input_width",0);settings.input_height=integer(prepared.stereo,"input_height",0);
        stereo=std::make_unique<ros_so_demo::StereoProvider>(settings);
    }
#endif
    const auto full_file=(o.output/"effective_full.json").u8string();
    psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;config.json_file=full_file.c_str();
#ifdef R36_SLIP_WITH_HORIZON
    if(stereo)config.hooks=stereo->hooks();
#endif
    Engine engine;auto status=statusValue();check(psdk_full_create(&config,&engine.value,&status),status,"full_create");
    check(psdk_full_command(engine.value,1,&status),status,"full_command");
    const auto metadata=[&] {
        return std::string("{\"input_mode\":")+quote(prepared.mode)+",\"disparity_source\":"+quote(prepared.method)+
            ",\"timeline_csv\":"+quote(text(prepared.slip_input,"timeline_csv"))+
            ",\"motion_reference_csv\":"+quote(motion_file)+",\"clock_domain\":"+quote(text(prepared.slip_input,"clock_domain"))+
            ",\"motion_reference_input_present\":"+(motion.empty()?"false":"true")+
            ",\"frame_identity_policy\":\"recorded_frame_id_if_present_else_original_source_timestamp_ns; camera=SDK_id_and_stamp\""
            ",\"timestamp_policy\":\"original_capture_time; never_playback_or_receipt_time\""
            ",\"playback_policy\":\"recording_unpaced_with_original_timestamps; camera_synchronous_without_ROS_or_latest_frame_queue\""
            ",\"session_policy\":\"one_monotonic_capture_epoch_one_fixed_model_ROI_calibration; new_run_resets_engine_history\""
            ",\"reference_policy\":\"measured_body_forward_mps_and_yaw_radps; incremental_bracketing_samples; no_IMU_fusion\""
            ",\"intrinsics_source\":"+quote(prepared.mode=="camera"?"camera_sdk_rectified_reference":"recording_intrinsics_file")+
            ",\"intrinsics\":{\"width\":640,\"height\":544,\"fx\":"+number(k.fx)+",\"fy\":"+number(k.fy)+
            ",\"cx\":"+number(k.cx)+",\"cy\":"+number(k.cy)+",\"bxf\":"+number(k.bxf)+"}"
            ",\"metrics_scope\":\"same_process_all_threads; loop includes acquire_load_motion_push_process_json_save; 100_percent=one_logical_CPU\""
            ",\"per_frame_total_scope\":\"acquisition_through_slip_JSONL_and_resource_read; metrics_JSONL_and_stdout_excluded; full_loop_includes_all\""
            ",\"warmup_frames\":0,\"first_frame_policy\":\"initializing_frame_included; state_history_never_reset_between_frames\""
            ",\"bpu_scope\":\"device_shared_snapshot_window; no_per_model_attribution; missing_values_are_null\""
            ",\"original_disparity_generation_ms\":null,\"source_camera_mode\":"+(prepared.mode=="camera"?(original?"2":"1"):"null")+"}";
    };
    write(o.output/"run.json",metadata());
    std::ofstream slip(o.output/"slip.jsonl",std::ios::binary),metrics(o.output/"metrics.jsonl",std::ios::binary);
    if(!slip||!metrics)throw std::runtime_error("Cannot create slip/metrics JSONL");
    r29::BpuSamplerOptions bpu_options;
    if(const auto* bpu=r33::json::optional(prepared.recording,"bpu")) {
        if(const auto* enabled=r33::json::optional(*bpu,"enabled"))bpu_options.enabled=r33::json::boolean(*enabled);
        bpu_options.sample_interval_ms=unsigned(integer(*bpu,"sample_interval_ms",100));
        if(const auto* paths=r33::json::optional(*bpu,"paths"))for(const auto& path:paths->array)
            bpu_options.paths.push_back(r33::json::string(path));
    }
    r29::BpuSampler bpu(bpu_options);bpu.begin((o.output/"bpu_samples.jsonl").u8string());
    const auto start=Clock::now();const double cpu_start=r29::processCpuMs();
    std::size_t count=0,motion_cursor=0;bool motion_initialized=false,stopped=false;
    std::size_t initializing_frames=0,visual_valid_frames=0,reference_valid_frames=0,evaluated_frames=0,slipping_frames=0;
    double compute_sum=0,acquire_sum=0;const int save_every=integer(prepared.parameters,"save_every_n",0);
    try {
      while(!interrupted&&(max_frames==0||count<std::size_t(max_frames))&&
            (prepared.mode=="camera"||count<timeline.size())) {
        const auto frame_start=Clock::now();const double cpu_before=r29::processCpuMs();
        std::vector<std::uint8_t> left_bytes,right_bytes;std::vector<std::uint16_t> q8;
        std::uint64_t timestamp=0,id=0;std::string identity_source;
        const auto acquire_start=Clock::now();
        if(prepared.mode=="recording") {
            const auto& row=timeline[count];id=row.frame_id;timestamp=row.timestamp_ns;identity_source=row.identity_source;
            left_bytes=app::readExactBytes(row.left_nv12,640*544*3/2);
            if(original)q8=app::readQ8LittleEndian(row.q8_u16,640*544);
            else right_bytes=app::readExactBytes(row.right_nv12,640*544*3/2);
        }
#ifdef R36_SLIP_WITH_CAMERA
        else {
            ros_so_demo::raw::Capture capture;const auto deadline=acquire_start+std::chrono::milliseconds(camera_options.timeout_ms);
            for(;;) {
                if(interrupted)throw Interrupted();
                if(Clock::now()>=deadline)throw std::runtime_error("No unique camera capture before timeout");
                try{capture=camera->next([]{return !interrupted;});}
                catch(const std::exception&){if(interrupted)throw Interrupted();throw;}
                if(Clock::now()>=deadline)throw std::runtime_error("Unique camera acquisition exceeded timeout");
                if(r36compare::runner::validateCameraCapture(capture,original,previous))break;
                ++duplicate_captures;std::this_thread::sleep_for(std::chrono::milliseconds(camera_options.retry_ms));
            }
            previous=capture.left.capture;id=previous->id;timestamp=previous->ns;identity_source="camera_sdk";
            left_bytes=std::move(capture.left.bytes);
            if(original) {
                q8.resize(640*544);for(std::size_t i=0;i<q8.size();++i)
                    q8[i]=std::uint16_t(capture.disparity.bytes[2*i])|(std::uint16_t(capture.disparity.bytes[2*i+1])<<8);
            } else right_bytes=std::move(capture.right.bytes);
        }
#endif
        const double acquire_ms=ms(acquire_start),acquire_cpu_ms=r29::processCpuMs()-cpu_before;
        const auto motion_start=Clock::now();
        if(!motion.empty()) {
            if(!motion_initialized){motion_cursor=app::initialMotionIndex(motion,timestamp);motion_initialized=true;}
            const auto end=app::motionEndpoint(motion,motion_cursor,timestamp);
            while(motion_cursor<end) {
                const auto n=std::min(std::size_t(8192),end-motion_cursor);std::vector<psdk_motion_sample> samples(n);
                for(std::size_t j=0;j<n;++j) {
                    const auto& m=motion[motion_cursor+j];auto& sample=samples[j];
                    sample.struct_size=sizeof(sample);sample.abi_version=PSDK_SLIP_ABI_VERSION;
                    sample.timestamp_ns=m.timestamp_ns;sample.linear_speed_mps=m.linear_speed_mps;sample.yaw_rate_radps=m.yaw_rate_radps;
                }
                check(psdk_full_push_motion_samples(engine.value,samples.data(),std::uint32_t(n),&status),status,"push_motion_samples");
                motion_cursor+=n;
            }
        }
        const double motion_push_ms=ms(motion_start);
        psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
        frame.left=image(left_bytes,id,timestamp);frame.intrinsics=k;
        const auto right=image(right_bytes,id,timestamp);
        psdk_disparity_q8 disparity{};disparity.struct_size=sizeof(disparity);disparity.abi_version=PSDK_FULL_ABI_VERSION;
        disparity.width=640;disparity.height=544;disparity.stride_bytes=1280;disparity.data=q8.data();disparity.bytes=q8.size()*2;
        disparity.frame_id=id;disparity.timestamp_ns=timestamp;
        if(original)frame.disparity=&disparity;
        else {
            frame.right=&right;
#ifdef R36_SLIP_WITH_HORIZON
            stereo->setPair(frame.left,right);
#endif
        }
        Result result;const double compute_cpu_before=r29::processCpuMs();const auto compute_start=Clock::now();
        check(psdk_full_process(engine.value,&frame,&result.value,&status),status,"full_process");
#ifdef R36_SLIP_WITH_HORIZON
        if(stereo){stereo->clearPair();if(!stereo->called())throw std::runtime_error("Configured Stereo disparity task did not execute");}
#endif
        const double compute_ms=ms(compute_start),compute_cpu_ms=r29::processCpuMs()-compute_cpu_before;
        psdk_full_result_view view{};view.struct_size=sizeof(view);view.abi_version=PSDK_FULL_ABI_VERSION;
        check(psdk_full_result_get_view(result.value,&view,&status),status,"result_get_view");
        psdk_slip_view slip_view{};slip_view.struct_size=sizeof(slip_view);slip_view.abi_version=PSDK_SLIP_ABI_VERSION;
        check(psdk_full_result_get_slip_view(result.value,&slip_view,&status),status,"result_get_slip_view");
        if(view.frame_id!=id||view.timestamp_ns!=timestamp||slip_view.frame_id!=id||
           (slip_view.end_timestamp_ns&&slip_view.end_timestamp_ns!=timestamp)||!slip_view.enabled)
            throw std::runtime_error("SDK returned missing/disabled or mismatched same-capture slip output");
        const auto save_start=Clock::now();
        if(save_every&&count%std::size_t(save_every)==0)
            check(psdk_full_save_result(result.value,(o.output/("frame-"+std::to_string(id)+"-"+std::to_string(timestamp))).u8string().c_str(),&status),status,"save_result");
        const double save_ms=ms(save_start);
        slip<<slipJson(slip_view,identity_source,timestamp)<<'\n';slip.flush();if(!slip)throw std::runtime_error("Cannot write slip JSONL");
        const auto resources=r29::processResources();const auto rss=resources.find("VmRSS:"),peak=resources.find("VmHWM:");
        metrics<<"{\"ordinal\":"<<count<<",\"frame_id\":"<<id<<",\"timestamp_ns\":"<<timestamp<<",\"frame_id_source\":"<<quote(identity_source)
           <<",\"acquire_load_ms\":"<<number(acquire_ms)<<",\"acquire_cpu_ms\":"<<number(acquire_cpu_ms)<<",\"motion_push_ms\":"<<number(motion_push_ms)
           <<",\"compute_ms\":"<<number(compute_ms)<<",\"compute_cpu_ms\":"<<number(compute_cpu_ms)<<",\"save_ms\":"<<number(save_ms)
           <<",\"total_ms\":"<<number(ms(frame_start))<<",\"total_cpu_ms\":"<<number(r29::processCpuMs()-cpu_before)
           <<",\"rss_kib\":"<<(rss==resources.end()?"null":std::to_string(rss->second))
           <<",\"peak_rss_kib\":"<<(peak==resources.end()?"null":std::to_string(peak->second))
           <<",\"graph_ms\":"<<number(view.graph_ms)<<",\"process_ms\":"<<number(view.process_ms)
           <<",\"visual_ms\":"<<number(slip_view.visual_ms)<<",\"reference_ms\":"<<number(slip_view.reference_ms)<<",\"slip_ms\":"<<number(slip_view.slip_ms)
           <<",\"bpu\":"<<bpu.latestJson()<<",\"original_disparity_generation_ms\":null,\"task_trace\":"<<(view.task_trace_json?view.task_trace_json:"null")<<"}\n";
        metrics.flush();if(!metrics)throw std::runtime_error("Cannot write metrics JSONL");
        ++count;compute_sum+=compute_ms;acquire_sum+=acquire_ms;
        initializing_frames+=slip_view.state==PSDK_SLIP_INITIALIZING;
        visual_valid_frames+=bool(slip_view.visual_valid);reference_valid_frames+=bool(slip_view.reference_valid);
        evaluated_frames+=slip_view.state==PSDK_SLIP_NORMAL||slip_view.state==PSDK_SLIP_SUSPECTED||
                          slip_view.state==PSDK_SLIP_SLIPPING||slip_view.state==PSDK_SLIP_RECOVERING;
        slipping_frames+=slip_view.state==PSDK_SLIP_SLIPPING;
        std::cout<<"frame="<<id<<" capture_ns="<<timestamp<<" state="<<fixed(slip_view.state_name,sizeof(slip_view.state_name))
                 <<" compute_ms="<<number(compute_ms)<<" reason="<<fixed(slip_view.reason,sizeof(slip_view.reason))<<'\n';
      }
    } catch(const Interrupted&){stopped=true;}
    stopped=stopped||interrupted;
    const double loop_ms=ms(start),loop_cpu_ms=r29::processCpuMs()-cpu_start;
    bpu.end();
    slip.close();metrics.close();if(!slip||!metrics)throw std::runtime_error("Cannot close JSONL outputs");
#ifdef R36_SLIP_WITH_CAMERA
    if(camera)camera->close();
#endif
    check(psdk_full_destroy(&engine.value,&status),status,"full_destroy");
    stopped=stopped||interrupted;
    const auto completion=[&] {
        return std::string("{\"completed\":")+(stopped?"false":"true")+",\"interrupted\":"+(stopped?"true":"false")+
          ",\"frames\":"+std::to_string(count)+",\"duplicate_capture_count\":"+std::to_string(duplicate_captures)+
          ",\"initializing_frames\":"+std::to_string(initializing_frames)+",\"visual_valid_frames\":"+std::to_string(visual_valid_frames)+
          ",\"reference_valid_frames\":"+std::to_string(reference_valid_frames)+",\"evaluated_frames\":"+std::to_string(evaluated_frames)+
          ",\"slipping_frames\":"+std::to_string(slipping_frames)+
          ",\"loop_ms\":"+number(loop_ms)+",\"loop_cpu_ms\":"+number(loop_cpu_ms)+
          ",\"loop_cpu_percent\":"+number(loop_cpu_ms/loop_ms*100)+",\"pipeline_fps\":"+number(count*1000.0/loop_ms)+
          ",\"compute_fps\":"+number(count*1000.0/compute_sum)+",\"acquire_load_ms_total\":"+number(acquire_sum)+
          ",\"compute_ms_total\":"+number(compute_sum)+",\"input_mode\":"+quote(prepared.mode)+
          ",\"bpu\":"+bpu.summaryJson()+
          ",\"measurement_scope\":\"slip_session_loop_including_acquisition_motion_push_processing_and_output; initialization_shutdown_excluded\"}";
    };
    auto final=completion();if(interrupted&&!stopped){stopped=true;final=completion();}
    write(o.output/(stopped?"partial.json":"complete.json"),final);return stopped?130:0;
}
}
int main(int argc,char** argv) {
    if(argc==2&&(std::string(argv[1])=="--help"||std::string(argv[1])=="-h")) {
        std::cout<<"r36_slip_runner --config slip_recording.json|slip_camera.json --output NEW_DIRECTORY [--check]\n"
          "Native C++ source -> SDK DAG -> visual motion/wheel reference/slip; no ROS transport or Python.\n"
          "Recording CSV: timestamp_ns,left_nv12,q8_u16[,right_nv12][,frame_id]; Stereo needs right_nv12 and may omit q8_u16.\n"
          "Motion CSV: timestamp_ns,linear_speed_mps,yaw_rate_radps, in the explicitly confirmed capture clock.\n"
          "A run has one fixed epoch/model/ROI/calibration; split epochs into separate runs to reset history.\n"
          "runtime.max_frames=0 runs camera until SIGINT; recording stops at its final row.\n";return 0;
    }
    try{return run(options(argc,argv));}
    catch(const std::exception& error){std::cerr<<"Slip runner error: "<<error.what()<<'\n';return interrupted?130:1;}
}
