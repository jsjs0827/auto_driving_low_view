#include "native_pipeline.hpp"
#include "image_input.hpp"
#include "perception/preprocess.hpp"
#include "perception_full_sdk.h"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
namespace {
const v3::Json& field(const v3::Json& value,const char* key) {
  return r33::json::required(value,key);
}
std::string quoted(const std::string& text) {
  v3::Json value;value.kind=v3::Json::Kind::String;value.text=text;
  return v3::jsonText(value,0);
}
double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
std::string stamp() {
  return std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}
void check(std::uint32_t code,const psdk_status& status,const char* operation) {
  if(!code)return;
  throw std::runtime_error(std::string(operation)+" failed code="+std::to_string(code)+
    " native="+std::to_string(status.native_code)+" feature="+status.feature+
    " task="+status.task+" detail="+status.detail);
}
struct Engine {
  psdk_full_engine* value=nullptr;
  ~Engine(){psdk_full_destroy(&value,nullptr);}
};
struct Result {
  psdk_full_result* value=nullptr;
  ~Result(){psdk_full_result_release(value);}
};
struct Options {fs::path config="windows_debug.json",output;};
Options options(int argc,char** argv) {
  Options out;std::set<std::string> seen;
  for(int i=1;i<argc;++i) {
    const std::string key=argv[i];
    if((key!="--config"&&key!="--output")||!seen.insert(key).second||i+1==argc)
      throw std::invalid_argument("Expected unique --config FILE and optional --output NEW_DIRECTORY");
    const auto path=fs::u8path(argv[++i]);
    if(path.empty())throw std::invalid_argument("Empty path for "+key);
    if(key=="--config")out.config=path;else out.output=path;
  }
  out.config=fs::absolute(out.config).lexically_normal();
  if(!fs::is_regular_file(out.config))throw std::invalid_argument("Configuration not found: "+out.config.u8string());
  if(out.output.empty())out.output=out.config.parent_path()/"output"/("windows-debug-"+stamp());
  out.output=fs::absolute(out.output).lexically_normal();
  if(fs::exists(out.output)||fs::is_symlink(out.output))
    throw std::invalid_argument("Output must be a NEW directory: "+out.output.u8string());
  return out;
}
example::Image load(const std::string& file,const std::string& configured_format) {
  auto extension=fs::u8path(file).extension().u8string();
  std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char c){return char(std::tolower(c));});
  example::ImageInputOptions input;
  // Encoded files are sniffed and decoded by the existing stb input adapter.
  // Raw camera-shaped files retain their bytes, layout and 640x544 geometry.
  if(extension==".nv12"||extension==".i420"||extension==".yuv") {
    input.format=configured_format;
    if(input.format!="nv12"&&input.format!="i420")
      throw std::invalid_argument("A .yuv file needs offline.format=nv12 or i420");
    if((extension==".nv12"&&input.format!="nv12")||(extension==".i420"&&input.format!="i420"))
      throw std::invalid_argument("Raw extension conflicts with offline.format; select its actual NV12/I420 layout");
    input.width=640;input.height=544;input.stride=640;
  } else input.format="image";
  auto image=example::loadImage(file,input);
  if(input.format=="image") {
    const auto start=Clock::now();
    // Reuse the SDK RGB -> camera-shaped YUV conversion before its YUV-only
    // C ABI boundary; model normalization remains inside CpuStages.
    const auto prepared=perception::prepare(image.view(),640,544,false,0);
    image.bytes=perception::rgbToNv12(prepared.rgb,640,544);
    constexpr std::size_t expected_yuv_bytes=std::size_t(640)*544*3/2;
    if(image.bytes.size()!=expected_yuv_bytes)
      throw std::runtime_error("RGB to NV12 conversion must produce exactly522240 bytes at640x544");
    image.width=640;image.height=544;image.stride=640;
    image.format=perception::PixelFormat::NV12;
    if(configured_format=="i420") {
      const std::size_t plane=640*544,quarter=plane/4;
      std::vector<std::uint8_t> planar(image.bytes.size());
      std::copy_n(image.bytes.begin(),plane,planar.begin());
      for(std::size_t i=0;i<quarter;++i){planar[plane+i]=image.bytes[plane+2*i];planar[plane+quarter+i]=image.bytes[plane+2*i+1];}
      image.bytes=std::move(planar);image.format=perception::PixelFormat::I420;
    }
    if(image.bytes.size()!=expected_yuv_bytes)
      throw std::runtime_error("Prepared NV12/I420 storage must contain exactly522240 bytes");
    image.decoded_format+="_to_640x544_"+configured_format;
    image.decode_ms+=milliseconds(start);
  }
  return image;
}
std::uint32_t pixelFormat(perception::PixelFormat format) {
  switch(format) {
    case perception::PixelFormat::RGB:return PSDK_FORMAT_RGB;
    case perception::PixelFormat::BGR:return PSDK_FORMAT_BGR;
    case perception::PixelFormat::NV12:return PSDK_FORMAT_NV12;
    case perception::PixelFormat::I420:return PSDK_FORMAT_I420;
  }
  throw std::invalid_argument("Unsupported image format");
}
int run(const Options& o) {
  const auto root=v3::readJson(o.config);
  if(r33::json::string(field(field(root,"input"),"mode"))!="offline")
    throw std::invalid_argument("Windows debug entry requires input.mode=offline");
  // Prepare configuration in this process. v3::launch deliberately rejects
  // Linux/ROS executables on Windows and is not used by this debug entry.
  auto prepared=v3::prepare(o.config,o.output,true);
  if(prepared.mode!="offline"||!prepared.stereo.object.empty()||
     r33::json::boolean(field(field(prepared.full,"mapping"),"enabled"))||
     r33::json::boolean(field(field(prepared.full,"recharge"),"enabled")))
    throw std::invalid_argument("Windows debug supports offline detection/segmentation; disable Stereo, mapping and recharge");
  if(r33::json::string(field(field(prepared.full,"inference"),"backend"))!="cpu")
    throw std::invalid_argument("Windows debug requires the real CPU ONNX backend");
  // The debugger entry saves inspectable images alongside its JSON and mask.
  // This changes only this run's effective output policy, not source profiles.
  auto& output_policy=prepared.full.object["output"].object;
  output_policy["save_overlay"].kind=v3::Json::Kind::Boolean;
  output_policy["save_overlay"].boolean=true;
  output_policy["save_inputs"].kind=v3::Json::Kind::Boolean;
  output_policy["save_inputs"].boolean=true;
  {
    // prepare() has just created this file in our exclusively new run directory.
    std::ofstream effective(o.output/"effective_full.json",std::ios::binary|std::ios::trunc);
    effective<<v3::jsonText(prepared.full)<<'\n';
    effective.close();
    if(!effective)throw std::runtime_error("Cannot write debugger output policy");
  }
  const auto& files=field(prepared.parameters,"offline_files").array;
  if(files.empty())throw std::invalid_argument("offline.left_files must contain at least one image");
  const auto max_frames=r33::json::integer(field(prepared.parameters,"offline_max_frames"),0,1000000);
  const bool loop=r33::json::boolean(field(prepared.parameters,"offline_loop"));
  if(loop&&!max_frames)throw std::invalid_argument("Offline debug loop requires a finite max_frames");
  const auto count=loop?std::size_t(max_frames):max_frames?std::min(files.size(),std::size_t(max_frames)):files.size();
  const auto input_format=r33::json::string(field(prepared.parameters,"offline_format"));
  const auto full_file=(o.output/"effective_full.json").u8string();
  psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;
  config.json_file=full_file.c_str();
  Engine engine;psdk_status status{};
  check(psdk_full_create(&config,&engine.value,&status),status,"psdk_full_create");
  check(psdk_full_command(engine.value,1,&status),status,"psdk_full_command");
  std::ofstream rows(o.output/"frames.jsonl",std::ios::binary);rows<<std::setprecision(12);
  if(!rows)throw std::runtime_error("Cannot create frames.jsonl");
  std::uint64_t last_stamp=0;double total_compute=0;
  std::cout<<"Windows offline debug: "<<o.config.u8string()<<"\nOutput: "<<o.output.u8string()<<'\n';
  for(std::size_t i=0;i<count;++i) {
    const auto& file=r33::json::string(files[i%files.size()]);
    auto image=load(file,input_format);
    const auto now=std::uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
    last_stamp=std::max(last_stamp+1,now);
    psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    auto& left=frame.left;left.struct_size=sizeof(left);left.abi_version=PSDK_ABI_VERSION;
    left.format=pixelFormat(image.format);left.width=std::uint32_t(image.width);left.height=std::uint32_t(image.height);
    left.stride=std::uint32_t(image.stride);left.data=image.bytes.data();left.bytes=image.bytes.size();
    left.frame_id=i+1;left.timestamp_ns=last_stamp;
    frame.intrinsics.width=left.width;frame.intrinsics.height=left.height;
    Result result;const auto start=Clock::now();
    // Set breakpoints here, then step into FullPipeline -> Graph -> Task ->
    // Feature -> CpuStages. All code is compiled from this v7 source tree.
    check(psdk_full_process(engine.value,&frame,&result.value,&status),status,"psdk_full_process");
    const auto compute=milliseconds(start);total_compute+=compute;
    psdk_full_result_view view{};view.struct_size=sizeof(view);view.abi_version=PSDK_FULL_ABI_VERSION;
    check(psdk_full_result_get_view(result.value,&view,&status),status,"psdk_full_result_get_view");
    if(!view.has_inference||view.frame_id!=left.frame_id||view.timestamp_ns!=last_stamp)
      throw std::runtime_error("SDK did not return matching same-frame inference");
    const auto directory=o.output/("frame-"+std::to_string(i+1));
    const auto save_start=Clock::now();
    check(psdk_full_save_result(result.value,directory.u8string().c_str(),&status),status,"psdk_full_save_result");
    const auto save_ms=milliseconds(save_start);
    const auto& t=view.inference.timings;
    rows<<"{\"frame_id\":"<<left.frame_id<<",\"source\":"<<quoted(file)
      <<",\"decode_and_convert_ms\":"<<image.decode_ms<<",\"input_format\":"<<quoted(image.decoded_format)
      <<",\"compute_ms\":"<<compute<<",\"save_ms\":"<<save_ms
      <<",\"graph_ms\":"<<view.graph_ms<<",\"det_pre_ms\":"<<t.detection_pre_ms
      <<",\"det_infer_ms\":"<<t.detection_infer_ms<<",\"det_post_ms\":"<<t.detection_post_ms
      <<",\"seg_pre_ms\":"<<t.segmentation_pre_ms<<",\"seg_infer_ms\":"<<t.segmentation_infer_ms
      <<",\"seg_post_ms\":"<<t.segmentation_post_ms<<",\"task_trace\":"
      <<(view.task_trace_json?view.task_trace_json:"null")<<"}\n";
    rows.flush();if(!rows)throw std::runtime_error("Cannot write frame metrics");
    std::cout<<"frame="<<left.frame_id<<" compute_ms="<<compute<<" det="<<view.inference.detection_count
      <<" mask_bytes="<<view.inference.segmentation_bytes<<" saved="<<directory.u8string()<<'\n';
  }
  rows.close();if(!rows)throw std::runtime_error("Cannot close frame metrics");
  std::ofstream complete(o.output/"complete.json",std::ios::binary);
  complete<<"{\"completed\":true,\"frames\":"<<count<<",\"compute_ms_total\":"<<std::setprecision(12)<<total_compute
    <<",\"backend\":\"cpu_onnx\",\"evidence\":\"offline_debug_not_X5_performance\"}\n";
  complete.close();if(!complete)throw std::runtime_error("Cannot write completion metadata");
  check(psdk_full_destroy(&engine.value,&status),status,"psdk_full_destroy");
  return 0;
}
} // namespace
int main(int argc,char** argv) {
  if(argc==2&&(std::string(argv[1])=="--help"||std::string(argv[1])=="-h")) {
    std::cout<<"perception_windows_debug [--config windows_debug.json] [--output NEW_DIRECTORY]\n"
      "Run v7 configuration, FullPipeline/DAG and real CPU ONNX in this process.\n"
      "Default output uses a fresh timestamp; PNG/JPEG/PPM and tight640x544 NV12/I420 are supported.\n";return 0;
  }
  try{return run(options(argc,argv));}
  catch(const std::exception& error){std::cerr<<"Windows offline debug error: "<<error.what()<<'\n';return 1;}
}
