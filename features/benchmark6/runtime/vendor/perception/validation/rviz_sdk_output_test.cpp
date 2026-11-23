#include "rviz_sdk_output.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
namespace {
namespace r=perception::rviz;
void need(bool b,const char* message){if(!b)throw std::runtime_error(message);}
const r::Packet& packet(const r::Frame& f,const std::string& key){for(const auto& p:f.packets)if(p.topic==key)return p;throw std::runtime_error("Missing "+key);}
r::RvizSettings only(std::initializer_list<std::string> keys){
  auto settings=r::defaultSettings();for(auto& stream:settings.publish.streams)stream.second=false;
  for(const auto& key:keys){settings.publish.streams.at(key)=true;settings.rviz.displays.at(key).enabled=true;}
  settings.publish.every_n=1;settings.publish.point_stride=1;return settings;
}
std::vector<std::uint8_t> wire(const r::Frame& frame,const r::RvizSettings& settings){
  r::Limits limits;limits.max_body_bytes=std::uint32_t(settings.transport.max_frame_bytes-r::kWireHeaderSize);
  limits.max_image_bytes=limits.max_body_bytes;limits.max_points=std::uint32_t(settings.publish.max_points);
  std::vector<std::uint8_t> bytes;std::string error;need(r::encode_frame(frame,bytes,&error,limits),error.c_str());return bytes;
}
template<class Fn>void budgetError(Fn call,const std::string& stream){
  bool rejected=false;try{call();}catch(const std::invalid_argument& error){const std::string text=error.what();rejected=text.find("transport.max_frame_bytes")!=std::string::npos&&text.find(stream)!=std::string::npos;}
  need(rejected,"Oversized fixed payload must expose the specific stream/budget before copying");
}
void budgetTests(){
  std::vector<std::uint8_t> pixels(16*8*3,128),map_rgb(8*8*3,77);
  psdk_frame input{sizeof(input),PSDK_ABI_VERSION,PSDK_FORMAT_RGB,16,8,48,pixels.data(),pixels.size(),41,1000000};
  psdk_full_result_view result{};result.struct_size=sizeof(result);result.abi_version=PSDK_FULL_ABI_VERSION;result.frame_id=input.frame_id;result.timestamp_ns=input.timestamp_ns;
  auto settings=only({"input_rgb"});const std::string long_frame(1024,'c');
  settings.transport.max_frame_bytes=1513; // 64 + record(32 + 9 + 1024 + 384).
  auto exact=r::sdkFrame(settings,input,result,long_frame,"base_link");
  need(wire(exact,settings).size()==1513,"Image budget includes exact wire header, record and identifiers");
  --settings.transport.max_frame_bytes;
  budgetError([&]{r::sdkFrame(settings,input,result,long_frame,"base_link");},"input_rgb");

  auto huge=input;huge.width=4096;huge.height=4096;huge.stride=4096*3;huge.data=nullptr;huge.bytes=0;
  settings.transport.max_frame_bytes=1024;
  budgetError([&]{r::sdkFrame(settings,huge,result,"camera_optical_frame","base_link");},"input_rgb");
  // All three individual images fit, their aggregate does not. Null RGB storage
  // proves preflight reaches the aggregate budget error before input conversion.
  auto null_input=input;null_input.data=nullptr;null_input.bytes=0;
  result.has_inference=1;std::vector<std::uint8_t> mask(16*8,1);
  result.inference.width=16;result.inference.height=8;result.inference.segmentation_mask=mask.data();result.inference.segmentation_bytes=mask.size();
  settings=only({"input_rgb","overlay","mask_rgb"});settings.transport.max_frame_bytes=1024;
  budgetError([&]{r::sdkFrame(settings,null_input,result,"camera_optical_frame","base_link");},"mask_rgb");
  result.has_inference=0;result.inference={};
  psdk_topview map{};std::memcpy(map.name,"pcl",4);map.image_width=4096;map.image_height=4096;map.rgb=map_rgb.data();map.rgb_bytes=std::uint64_t(4096)*4096*3;
  result.maps=&map;result.map_count=1;settings=only({"input_rgb","maps/pcl"});settings.transport.max_frame_bytes=1024;
  budgetError([&]{r::sdkFrame(settings,null_input,result,"camera_optical_frame","base_link");},"maps/pcl");

  // Non-point image/map/text records are reserved before either cloud. Compare
  // against the real encoder's measured fixed size, not a duplicate size formula.
  map.image_width=8;map.image_height=8;map.rgb_bytes=map_rgb.size();
  std::vector<psdk_cloud_point> points(100);for(unsigned i=0;i<points.size();++i)points[i]={float(i+1),2,3,.7f,2,11,22,33,0};
  psdk_cloud_view clouds[2]{};std::memcpy(clouds[0].name,"camera",7);std::memcpy(clouds[1].name,"robot",6);
  clouds[0].points=points.data();clouds[1].points=points.data();result.clouds=clouds;result.cloud_count=2;
  psdk_slip_view slip{};slip.enabled=1;slip.frame_id=input.frame_id;slip.details_json="{}";
  settings=only({"input_rgb","maps/pcl","clouds/camera","clouds/robot","slip"});settings.transport.max_frame_bytes=1024;settings.publish.max_points=100;
  const auto fixed=wire(r::sdkFrame(settings,input,result,"camera_optical_frame","base_link",&slip),settings).size();
  const auto available=(1024-fixed)/24;need(available>0&&available<points.size(),"Fixture leaves a genuine partial-cloud budget");
  clouds[0].point_count=points.size();clouds[1].point_count=points.size();
  auto sampled=r::sdkFrame(settings,input,result,"camera_optical_frame","base_link",&slip);
  const auto& camera=std::get<r::PointCloud>(packet(sampled,"clouds/camera").payload);
  const auto& robot=std::get<r::PointCloud>(packet(sampled,"clouds/robot").payload);
  need(camera.points.size()==available&&robot.points.empty(),"Clouds use actual bytes remaining after all fixed payloads");
  const auto stride=(points.size()+available-1)/available;
  for(std::size_t i=0;i<camera.points.size();++i)need(camera.points[i].x==points[i*stride].x&&camera.points[i].rgb==0x0b1621,"Budget sampling preserves selected point coordinates/colors");
  need(wire(sampled,settings).size()==fixed+available*24&&wire(sampled,settings).size()<=1024,"Real encoded frame remains within configured total size");
  need(std::get<r::Text>(packet(sampled,"slip").payload).data=="{}"&&std::get<r::Image>(packet(sampled,"maps/pcl").payload).data==map_rgb,"Later map/text payloads survive point budget pressure");
  std::vector<psdk_cloud_point> invalid=points;for(auto& point:invalid)point.x=std::numeric_limits<float>::quiet_NaN();clouds[0].points=invalid.data();
  sampled=r::sdkFrame(settings,input,result,"camera_optical_frame","base_link",&slip);
  const auto& empty=std::get<r::PointCloud>(packet(sampled,"clouds/camera").payload);
  need(empty.points.empty()&&empty.points.capacity()==0,"Nonfinite cloud retains no unused max_points allocation");
  need(std::get<r::PointCloud>(packet(sampled,"clouds/robot").payload).points.size()==available,"Filtered points return their real unused byte budget to later streams");
  need(wire(sampled,settings).size()<=1024&&pixels.front()==128&&points.front().x==1,"Budget handling is bounded and never modifies source buffers");
}
#if defined(__linux__) || defined(__APPLE__)
void failedAttemptThrottleTest(){
  const int probe=::socket(AF_INET,SOCK_STREAM,0);need(probe>=0,"Free-port socket");sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  need(::bind(probe,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"Free-port bind");socklen_t length=sizeof(address);need(getsockname(probe,reinterpret_cast<sockaddr*>(&address),&length)==0,"Free-port name");::close(probe);
  auto settings=only({"input_rgb"});settings.transport.remote_port=ntohs(address.sin_port);settings.transport.max_frame_bytes=1024;settings.publish.max_fps=.1;
  struct Config{std::filesystem::path path;~Config(){std::error_code error;std::filesystem::remove(path,error);}} config{std::filesystem::temp_directory_path()/("rviz_budget_"+std::to_string(::getpid())+"_"+std::to_string(r::system_time_ns())+".json")};
  {std::ofstream out(config.path);out<<r::settingsJson(settings);need(bool(out),"Write publisher test settings");}
  r::SdkPublisher publisher(config.path);psdk_frame input{sizeof(input),PSDK_ABI_VERSION,PSDK_FORMAT_RGB,4096,4096,4096*3,nullptr,0,41,1000000};
  psdk_full_result_view result{};result.struct_size=sizeof(result);result.abi_version=PSDK_FULL_ABI_VERSION;result.frame_id=input.frame_id;result.timestamp_ns=input.timestamp_ns;
  need(!publisher.publish(input,result,"camera_optical_frame","base_link"),"No client skips conversion even for oversized source");
  r::ClientOptions options;options.port=settings.transport.remote_port;r::Client client(options);r::Frame frame;r::ReceiveInfo info;
  need(client.receive(frame,info,2000)==r::ReceiveResult::Heartbeat&&publisher.connected(),"Real client enables publisher conversion");
  budgetError([&]{publisher.publish(input,result,"camera_optical_frame","base_link");},"input_rgb");
  need(!publisher.publish(input,result,"camera_optical_frame","base_link"),"Failed budget attempt advances display throttle instead of throwing/converting every frame");
  need(publisher.stats().published==0,"Rejected preflight never queues an oversized frame");client.stop();
}
#endif
}
int main(){try{
  auto settings=r::defaultSettings();for(auto& s:settings.publish.streams)s.second=true;for(auto& d:settings.rviz.displays)d.second.enabled=true;settings.publish.point_stride=1;settings.publish.max_points=2;
  constexpr unsigned w=16,h=8;std::vector<std::uint8_t> yuv(w*h*3/2,128),mask(w*h);for(unsigned i=0;i<mask.size();++i)mask[i]=i%3;
  std::vector<std::uint16_t> q((w+1)*h,8192);q[0]=0;const auto yuv_original=yuv;const auto q_original=q;
  psdk_frame input{sizeof(input),PSDK_ABI_VERSION,PSDK_FORMAT_NV12,w,h,w,yuv.data(),yuv.size(),8123,1000123456};
  psdk_detection d{};d.class_id=13;d.confidence=.9f;d.x1=2;d.y1=2;d.x2=12;d.y2=7;std::memcpy(d.class_name,"fence",6);
  psdk_cloud_point points[5]={{1,2,3,.7f,2,11,22,33,0},{2,3,4,.8f,1,44,55,66,0},{3,4,5,.9f,0,77,88,99,0},{4,5,6,1,2,10,20,30,0},{5,6,7,1,2,30,40,50,0}};
  psdk_cloud_view cloud{};std::memcpy(cloud.name,"camera",7);cloud.points=points;cloud.point_count=5;
  psdk_full_result_view result{};result.struct_size=sizeof(result);result.abi_version=PSDK_FULL_ABI_VERSION;result.frame_id=input.frame_id;result.timestamp_ns=input.timestamp_ns;
  result.has_inference=1;result.inference.width=w;result.inference.height=h;result.inference.detections=&d;result.inference.detection_count=1;result.inference.segmentation_mask=mask.data();result.inference.segmentation_bytes=mask.size();
  result.disparity={sizeof(psdk_disparity_q8),PSDK_FULL_ABI_VERSION,w,h,(w+1)*2,0,q.data(),q.size()*2,input.frame_id,input.timestamp_ns};result.cloud_count=1;result.clouds=&cloud;
  const auto out=r::sdkFrame(settings,input,result,"camera_optical_frame","base_link");
  need(out.frame_number==input.frame_id&&out.capture_timestamp_ns==input.timestamp_ns,"SDK display capture identity preserved");
  const auto& raw=std::get<r::Image>(packet(out,"disparity_q8").payload);need(raw.step==w*2&&raw.data.size()==w*h*2&&raw.data[0]==0&&raw.data[1]==0&&raw.data[2]==0&&raw.data[3]==32,"Stride-aware exact little-endian Q8, no depth conversion");
  const auto& rgb=std::get<r::Image>(packet(out,"input_rgb").payload);need(rgb.data.size()==w*h*3&&rgb.data[0]>=129&&rgb.data[0]<=131,"NV12 is actually converted to RGB, not relabelled");
  const auto& colors=std::get<r::Image>(packet(out,"mask_rgb").payload);need(colors.data[0]==235&&colors.data[1]==90&&colors.data[2]==70&&colors.data[3]==35&&colors.data[4]==205&&colors.data[5]==95,"Raw classes use the established segmentation palette");
  const auto& disp=std::get<r::Image>(packet(out,"disparity_rgb").payload);need(disp.data[0]==0&&disp.data[1]==0&&disp.data[2]==0&&disp.data[5]>0,"Zero Q8 remains invalid black; valid disparities are visible");
  const auto& pc=std::get<r::PointCloud>(packet(out,"clouds/camera").payload);need(pc.points.size()==2&&pc.points[0].rgb==0x0b1621&&pc.points[0].label==2&&pc.points[1].x==4,"Deterministic bounded point sampling preserves fields");
  const auto& overlay=std::get<r::Image>(packet(out,"overlay").payload);need(overlay.data!=rgb.data&&overlay.data.size()==rgb.data.size(),"Actual detection/segmentation overlay rendered on owned display copy");
  need(yuv==yuv_original&&q==q_original,"Display processing never mutates model/camera inputs");
  std::fill(yuv.begin(),yuv.end(),0);std::fill(q.begin(),q.end(),0);need(rgb.data[0]>=129&&raw.data[3]==32,"Packets own payload after SDK/camera buffer reuse");
  settings.enabled=false;need(r::sdkFrame(settings,input,result,"camera_optical_frame","base_link").packets.empty(),"Disabled display does not make payload copies");settings.enabled=true;
  input.timestamp_ns++;bool rejected=false;try{r::sdkFrame(settings,input,result,"camera_optical_frame","base_link");}catch(const std::exception&){rejected=true;}need(rejected,"Different capture identities rejected");
  budgetTests();
#if defined(__linux__) || defined(__APPLE__)
  failedAttemptThrottleTest();
#endif
  std::cout<<"SDK display contracts passed: owned RGB/mask/overlay/Q8/cloud, exact pre-copy budgets, failure throttle, true time/frame, no input mutation\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
