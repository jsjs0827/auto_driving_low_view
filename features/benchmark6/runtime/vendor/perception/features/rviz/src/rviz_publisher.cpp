#include "rviz_sdk_output.hpp"
#include "native_pipeline.hpp"
#include "intrinsics_file.hpp"
#include "slip_timeline.hpp"
#include "camera_capture_util.hpp"
#ifdef R36_RVIZ_WITH_HORIZON
#include "perception_ros_so_demo/stereo_provider.hpp"
#endif
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <thread>

namespace {
namespace fs=std::filesystem;
namespace view=perception::rviz;
namespace app=perception::slip_app;
namespace raw=ros_so_demo::raw;
volatile std::sig_atomic_t stopped=0;
void stop(int){stopped=1;}
const v3::Json& field(const v3::Json& value,const char* key){return r33::json::required(value,key);}
std::string text(const v3::Json& value,const char* key,const std::string& fallback={}){const auto* v=r33::json::optional(value,key);return !v||v->kind==v3::Json::Kind::Null?fallback:r33::json::string(*v);}
int integer(const v3::Json& value,const char* key,int fallback){const auto* v=r33::json::optional(value,key);return v?r33::json::integer(*v,0,1000000):fallback;}
bool flag(const v3::Json& value,const char* key,bool fallback=false){const auto* v=r33::json::optional(value,key);return v?r33::json::boolean(*v):fallback;}
void write(const fs::path& path,const std::string& contents){std::ofstream out(path,std::ios::binary);out<<contents<<'\n';out.close();if(!out)throw std::runtime_error("Cannot write "+path.u8string());}
psdk_status status(){psdk_status s{};s.struct_size=sizeof(s);s.abi_version=PSDK_ABI_VERSION;return s;}
void check(std::uint32_t code,const psdk_status& s){if(code)throw std::runtime_error("SDK "+std::to_string(code)+" "+s.feature+": "+s.detail);}
struct Engine{psdk_full_engine* value=nullptr;~Engine(){psdk_full_destroy(&value,nullptr);}};
struct Result{psdk_full_result* value=nullptr;~Result(){psdk_full_result_release(value);}};
psdk_frame frame(const std::vector<std::uint8_t>& bytes,std::uint64_t id,std::uint64_t ns,unsigned w=640,unsigned h=544){return {sizeof(psdk_frame),PSDK_ABI_VERSION,PSDK_FORMAT_NV12,w,h,w,bytes.data(),bytes.size(),id,ns};}
struct Options{fs::path pipeline,rviz,output;bool check=false;};
Options parse(int argc,char** argv){
  Options o;std::set<std::string> seen;
  for(int i=1;i<argc;++i){const std::string key=argv[i];if(!seen.insert(key).second)throw std::invalid_argument("Duplicate option: "+key);
    if(key=="--help"){std::cout<<"r36_rviz_publisher --config PIPELINE.json [--rviz RVIZ.json] --output NEW [--check]\nX5 camera/recording -> bounded TCP display publisher; no ROS required.\n";std::exit(0);}
    if(key=="--check"){o.check=true;continue;}if(i+1>=argc)throw std::invalid_argument("Missing option value");const auto path=fs::u8path(argv[++i]);
    if(key=="--config")o.pipeline=path;else if(key=="--rviz")o.rviz=path;else if(key=="--output")o.output=path;else throw std::invalid_argument("Unknown option: "+key);}
  if(o.pipeline.empty()||o.output.empty())throw std::invalid_argument("--config and --output are required");o.pipeline=fs::absolute(o.pipeline).lexically_normal();o.output=fs::absolute(o.output).lexically_normal();
  if(o.rviz.empty()){
    const auto root=v3::readJson(o.pipeline);
    const auto* configured=r33::json::optional(root,"rviz_config");
    o.rviz=configured?fs::u8path(v3::resolvePath(*configured,o.pipeline.parent_path(),"rviz_config","file")):o.pipeline.parent_path()/"rviz.json";
  }
  o.rviz=fs::absolute(o.rviz).lexically_normal();if(fs::exists(o.output)||fs::is_symlink(o.output))throw std::invalid_argument("Output must be a new directory");return o;
}
psdk_camera_intrinsics intrinsics(const v3::Json& value){
  psdk_camera_intrinsics k{std::uint32_t(integer(value,"width",0)),std::uint32_t(integer(value,"height",0)),r33::json::number(field(value,"fx")),r33::json::number(field(value,"fy")),r33::json::number(field(value,"cx")),r33::json::number(field(value,"cy")),r33::json::number(field(value,"bxf"))};
  if(k.width!=640||k.height!=544||k.fx<=0||k.fy<=0||k.bxf<=0||k.cx<0||k.cy<0||k.cx>=640||k.cy>=544)throw std::invalid_argument("Recording needs actual rectified 640x544 intrinsics");return k;
}
int run(const Options& o){
  const auto visual=view::loadSettings(o.rviz);auto prepared=v3::prepare(o.pipeline,o.output,false);
  if(prepared.mode!="camera"&&prepared.mode!="recording")throw std::invalid_argument("Native RViz publisher accepts camera or timestamped recording; ROS/offline sources use ROS demo rviz_config instead");
  const bool original=prepared.method=="original_q8",disparity=flag(prepared.parameters,"disparity_enabled"),recharge=text(prepared.parameters,"mode")=="recharge"||text(prepared.parameters,"mode")=="both";
  if(!flag(prepared.parameters,"publish_enabled"))throw std::invalid_argument("RViz publisher requires graph.publish enabled");
#ifndef R36_RVIZ_WITH_HORIZON
  if(disparity&&!original)throw std::invalid_argument("raft_stereo requires an X5 build with real Horizon provider");
#endif
  std::vector<app::TimelineRow> timeline;std::vector<app::MotionRow> motion;psdk_camera_intrinsics k{};
  if(prepared.mode=="recording"){
    if(recharge)throw std::invalid_argument("RViz recording timeline does not contain same-capture GDC input");
    const auto path=v3::resolvePath(field(field(prepared.requested,"input"),"timeline_csv"),o.pipeline.parent_path(),"input.timeline_csv","file");
    app::TimelineOptions t;t.original_q8=original;timeline=app::readTimeline(fs::u8path(path),t);k=intrinsics(v3::loadIntrinsicsFile(fs::u8path(text(prepared.recording,"intrinsics_file")),true));
  }
  const auto motion_path=text(prepared.slip_input,"motion_reference_csv");if(!motion_path.empty())motion=app::readMotionCsv(fs::u8path(motion_path));
  if(o.check){std::cout<<"RViz publisher configuration valid; input="<<prepared.mode<<", TCP="<<visual.transport.bind_address<<':'<<visual.transport.remote_port<<", captures="<<timeline.size()<<"; no camera/model/socket opened\n";return 0;}
  if(!visual.enabled){std::cout<<"RViz disabled by JSON; no camera/model/socket opened\n";return 0;}
  prepared=v3::prepare(o.pipeline,o.output,true);write(o.output/"rviz_resolved.json",view::settingsJson(visual));
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);view::SdkPublisher publisher(o.rviz);
  std::unique_ptr<raw::Camera> camera;std::optional<raw::Identity> previous;
#ifdef R36_RVIZ_WITH_HORIZON
  std::unique_ptr<ros_so_demo::StereoProvider> stereo;
  if(disparity&&!original){ros_so_demo::StereoSettings s;s.model=text(prepared.stereo,"model");s.input_mode=text(prepared.stereo,"input_mode","crop");s.input_width=integer(prepared.stereo,"input_width",0);s.input_height=integer(prepared.stereo,"input_height",0);stereo=std::make_unique<ros_so_demo::StereoProvider>(s);}
#endif
  Engine engine;auto s=status();const auto path=(o.output/"effective_full.json").u8string();psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;config.json_file=path.c_str();
#ifdef R36_RVIZ_WITH_HORIZON
  if(stereo)config.hooks=stereo->hooks();
#endif
  check(psdk_full_create(&config,&engine.value,&s),s);
  raw::CameraOptions camera_options;
  if(prepared.mode=="camera"){
    camera_options.sdk=text(prepared.parameters,"camera_sdk");camera_options.config=text(prepared.parameters,"camera_config");camera_options.need_q8=disparity&&original;camera_options.need_gdc=recharge;
    camera_options.disparity_mode=integer(prepared.parameters,"camera_disparity_mode",original?2:1);camera_options.timeout_ms=integer(prepared.parameters,"acquire_timeout_ms",3000);camera_options.retry_ms=integer(prepared.parameters,"retry_interval_ms",5);
    camera=std::make_unique<raw::Camera>(camera_options);const auto actual=r36compare::runner::cameraIntrinsics(camera->parameters());k={640,544,actual.fx,actual.fy,actual.cx,actual.cy,actual.bxf};
  }else{
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(visual.transport.connect_timeout_ms);
    while(!stopped&&!publisher.connected()&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if(!publisher.connected())throw std::runtime_error("No RViz client before recording playback; start Ubuntu receiver first or increase transport.connect_timeout_ms");
  }
  const auto* runtime=r33::json::optional(prepared.requested,"runtime");const int limit=prepared.mode=="recording"?integer(prepared.recording,"limit",100):(runtime?integer(*runtime,"max_frames",0):0);
  const auto optical=text(prepared.parameters,"frame_id","camera_optical_frame"),body=text(prepared.parameters,"robot_frame_id","base_link");
  std::uint64_t processed=0,display_frames=0;std::size_t motion_cursor=0;bool motion_initialized=false;
  std::cout<<"X5 RViz publisher listening "<<visual.transport.bind_address<<':'<<visual.transport.remote_port<<"; no ROS/DDS required on this side\n";
  try{
    while(!stopped&&(!limit||processed<std::uint64_t(limit))&&(camera||processed<timeline.size())){
      const auto begin=std::chrono::steady_clock::now();raw::Capture capture;std::uint64_t id=0,ns=0;std::vector<std::uint16_t> q;
      if(camera){const auto deadline=begin+std::chrono::milliseconds(camera_options.timeout_ms);for(;;){capture=camera->next([]{return !stopped;});if(r36compare::runner::validateCameraCapture(capture,camera_options.need_q8,previous))break;
          if(stopped)break;if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("No unique camera frame before timeout");std::this_thread::sleep_for(std::chrono::milliseconds(camera_options.retry_ms));}
        if(stopped)break;previous=capture.left.capture;id=previous->id;ns=previous->ns;
        if(camera_options.need_q8){q.resize(640*544);for(std::size_t i=0;i<q.size();++i)q[i]=std::uint16_t(capture.disparity.bytes[2*i])|(std::uint16_t(capture.disparity.bytes[2*i+1])<<8);}
      }else{const auto& row=timeline[processed];id=row.frame_id;ns=row.timestamp_ns;capture.left.bytes=app::readExactBytes(row.left_nv12,640*544*3/2);if(original)q=app::readQ8LittleEndian(row.q8_u16,640*544);else capture.right.bytes=app::readExactBytes(row.right_nv12,640*544*3/2);}
      if(!motion.empty()){
        if(!motion_initialized){motion_cursor=app::initialMotionIndex(motion,ns);motion_initialized=true;}
        const auto end=app::motionEndpoint(motion,motion_cursor,ns);while(motion_cursor<end){const auto count=std::min(std::size_t(8192),end-motion_cursor);std::vector<psdk_motion_sample> samples(count);
          for(std::size_t i=0;i<count;++i){const auto& m=motion[motion_cursor+i];samples[i]={sizeof(psdk_motion_sample),PSDK_SLIP_ABI_VERSION,m.timestamp_ns,m.linear_speed_mps,m.yaw_rate_radps,{0,0}};}
          check(psdk_full_push_motion_samples(engine.value,samples.data(),std::uint32_t(count),&s),s);motion_cursor+=count;}
      }
      psdk_full_frame input{};input.struct_size=sizeof(input);input.abi_version=PSDK_FULL_ABI_VERSION;input.left=frame(capture.left.bytes,id,ns);input.intrinsics=k;
      const auto right=frame(capture.right.bytes,id,ns),high=frame(capture.gdc.bytes,id,ns,1280,1088);
      psdk_disparity_q8 depth{sizeof(psdk_disparity_q8),PSDK_FULL_ABI_VERSION,640,544,1280,0,q.data(),q.size()*2,id,ns};
      if(disparity&&original)input.disparity=&depth;if(recharge)input.recharge_hires=&high;
#ifdef R36_RVIZ_WITH_HORIZON
      if(stereo){input.right=&right;stereo->setPair(input.left,right);}
#endif
      Result result;check(psdk_full_process(engine.value,&input,&result.value,&s),s);
#ifdef R36_RVIZ_WITH_HORIZON
      if(stereo){stereo->clearPair();if(!stereo->called())throw std::runtime_error("Stereo provider was not invoked by the DAG");}
#endif
      psdk_full_result_view output{};output.struct_size=sizeof(output);output.abi_version=PSDK_FULL_ABI_VERSION;check(psdk_full_result_get_view(result.value,&output,&s),s);
      psdk_slip_view slip{};slip.struct_size=sizeof(slip);slip.abi_version=PSDK_SLIP_ABI_VERSION;const psdk_slip_view* slip_ptr=nullptr;
      if(publisher.wantsSlip()){
        check(psdk_full_result_get_slip_view(result.value,&slip,&s),s);slip_ptr=&slip;
        if(!processed)std::cout<<"SDK slip enabled="<<slip.enabled<<" state="<<slip.state_name<<" reason="<<slip.reason<<'\n';
      }
      if(publisher.publish(input.left,output,optical,body,slip_ptr))++display_frames;++processed;
      if(processed==1||processed%100==0)std::cout<<"processed="<<processed<<" display_submitted="<<display_frames<<" frame="<<id<<" timestamp_ns="<<ns<<'\n';
      // Recorded capture times are preserved; pacing only gives the display a
      // chance to observe a finite replay and never enters motion integration.
      if(!camera){const auto until=begin+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1./visual.publish.max_fps));while(!stopped&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    }
    if(camera)camera->close();const auto stats=publisher.stats();write(o.output/(stopped?"partial.json":"complete.json"),"{\"completed\":"+std::string(stopped?"false":"true")+",\"processed_frames\":"+std::to_string(processed)+",\"display_submitted\":"+std::to_string(display_frames)+",\"display_sent\":"+std::to_string(stats.sent)+",\"display_replaced\":"+std::to_string(stats.replaced)+",\"display_rejected\":"+std::to_string(stats.rejected)+"}");
  }catch(const std::exception& e){write(o.output/"failed.json","{\"completed\":false,\"error\":"+r33::json::quote(e.what())+"}");throw;}
  return 0;
}
} // namespace
int main(int argc,char** argv){try{return run(parse(argc,argv));}catch(const std::exception& e){std::cerr<<"RViz publisher error: "<<e.what()<<'\n';return 1;}}
