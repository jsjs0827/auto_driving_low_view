#include "rviz_sdk_output.hpp"
#include "visualization.hpp"
#include "json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace perception::rviz {
namespace {
void require(bool ok,const char* reason){if(!ok)throw std::invalid_argument(reason);}
std::string bounded(const char* p,std::size_t n){const auto end=std::find(p,p+n,'\0');require(end!=p+n,"SDK display string is not terminated");return {p,end};}
using Color=std::array<std::uint8_t,3>;
Color palette(unsigned id){
  if(id==0)return {235,90,70};if(id==1)return {35,205,95};if(id==2)return {80,140,245};
  Color color{};unsigned bits=id+1;for(unsigned shift=0;shift<8;++shift)for(unsigned c=0;c<3;++c){color[c]|=std::uint8_t((bits&1)<<(7-shift));bits>>=1;}return color;
}
perception::ImageView imageView(const psdk_frame& frame){
  perception::PixelFormat format;
  switch(frame.format){case PSDK_FORMAT_NV12:format=perception::PixelFormat::NV12;break;case PSDK_FORMAT_I420:format=perception::PixelFormat::I420;break;
    case PSDK_FORMAT_RGB:format=perception::PixelFormat::RGB;break;case PSDK_FORMAT_BGR:format=perception::PixelFormat::BGR;break;default:throw std::invalid_argument("Unsupported display input format");}
  require(frame.width&&frame.height&&frame.width<=4096&&frame.height<=4096&&frame.bytes<=std::numeric_limits<std::size_t>::max(),"Invalid display input geometry");
  return {frame.data,std::size_t(frame.bytes),frame.stride,int(frame.width),int(frame.height),format};
}
Image rgbImage(unsigned w,unsigned h,std::vector<std::uint8_t> pixels){return {w,h,w*3,ImageEncoding::Rgb8,std::move(pixels)};}
std::string resultJson(const psdk_full_result_view& result){
  std::ostringstream out;out<<std::setprecision(12)<<"{\"frame_id\":"<<result.frame_id<<",\"timestamp_ns\":"<<result.timestamp_ns
    <<",\"has_inference\":"<<(result.has_inference?"true":"false")<<",\"graph_ms\":";
  if(std::isfinite(result.graph_ms))out<<result.graph_ms;else out<<"null";
  out<<",\"process_ms\":";if(std::isfinite(result.process_ms))out<<result.process_ms;else out<<"null";
  out<<",\"detection_count\":"<<result.inference.detection_count<<",\"cloud_count\":"<<result.cloud_count<<",\"map_count\":"<<result.map_count<<'}';return out.str();
}
Limits wireLimits(const RvizSettings& settings){
  require(settings.transport.max_frame_bytes>=1024&&settings.transport.max_frame_bytes<=64*1024*1024,"Invalid display frame byte limit");
  require(settings.publish.max_points>0&&settings.publish.max_points<=1000000&&settings.publish.point_stride>0,"Invalid display point limits");
  Limits limits;limits.max_body_bytes=std::uint32_t(settings.transport.max_frame_bytes-kWireHeaderSize);
  limits.max_image_bytes=limits.max_body_bytes;limits.max_points=std::uint32_t(settings.publish.max_points);return limits;
}
// Reserve every fixed record before reading/copying any image, map or point.
// Wire v1 has a 4-byte record length and 12 bytes of shared packet metadata.
// Image adds 16 bytes; cloud/text adds 4 bytes. Header bytes are already excluded.
class FrameBudget {
 public:
  explicit FrameBudget(const Limits& limits):remaining_(limits.max_body_bytes),limits_(limits){}
  void record(const std::string& key,const std::string& frame,std::uint64_t typed_header,std::uint64_t payload){
    require(!key.empty()&&key.size()<=limits_.max_string_bytes&&frame.size()<=limits_.max_string_bytes&&
      key.find('\0')==std::string::npos&&frame.find('\0')==std::string::npos,"Invalid display stream/frame identifier");
    require(++packets_<=limits_.max_packets,"Display packet count exceeds wire limit");
    take(key,16+key.size()+frame.size()+typed_header+payload);
  }
  void take(const std::string& key,std::uint64_t bytes){
    if(bytes>remaining_)throw std::invalid_argument("RViz stream '"+key+"' exceeds transport.max_frame_bytes budget; requires "+std::to_string(bytes)+" bytes, remaining "+std::to_string(remaining_));
    remaining_-=bytes;
  }
  std::uint64_t remaining()const{return remaining_;}
  std::size_t packets()const{return packets_;}
 private:
  std::uint64_t remaining_;const Limits& limits_;std::size_t packets_=0;
};
std::size_t jsonLength(const char* value){
  for(std::size_t size=0;size<=64*1024;++size)if(!value[size])return size;
  throw std::invalid_argument("Display slip JSON exceeds limit");
}
} // namespace
Frame sdkFrame(const RvizSettings& settings,const psdk_frame& input,const psdk_full_result_view& result,
               const std::string& optical_frame,const std::string& body_frame,const psdk_slip_view* slip){
  require(result.struct_size==sizeof(result)&&result.abi_version==PSDK_FULL_ABI_VERSION,"Invalid SDK display result ABI");
  require(input.struct_size==sizeof(input)&&input.abi_version==PSDK_ABI_VERSION&&input.frame_id==result.frame_id&&input.timestamp_ns==result.timestamp_ns&&result.timestamp_ns,"Display frame/result identity mismatch");
  Frame out;out.frame_number=result.frame_id;out.capture_timestamp_ns=result.timestamp_ns;
  if(!settings.enabled)return out;
  require(!optical_frame.empty()&&!body_frame.empty(),"Display coordinate frames must be explicit");
  const auto limits=wireLimits(settings);FrameBudget budget(limits);
  const auto add=[&](const std::string& key,const std::string& frame,auto payload){if(streamEnabled(settings,key))out.packets.push_back({key,frame,std::move(payload)});};
  const unsigned w=input.width,h=input.height;require(w&&h&&w<=4096&&h<=4096,"Invalid display image geometry");const std::size_t pixels=std::size_t(w)*h;
  const bool input_rgb=streamEnabled(settings,"input_rgb");
  const bool overlay=streamEnabled(settings,"overlay")&&result.has_inference;
  const auto& ai=result.inference;const bool mask=result.has_inference&&ai.segmentation_mask&&ai.segmentation_bytes;
  if(mask)require(ai.width==w&&ai.height==h&&ai.segmentation_bytes==pixels,"Display segmentation must match source image");
  const bool mask_rgb=mask&&streamEnabled(settings,"mask_rgb");
  const auto& d=result.disparity;
  const bool disparity_rgb=d.data&&d.bytes&&streamEnabled(settings,"disparity_rgb");
  const bool disparity_q8=d.data&&d.bytes&&streamEnabled(settings,"disparity_q8");
  if(disparity_rgb||disparity_q8)require(d.frame_id==result.frame_id&&d.timestamp_ns==result.timestamp_ns&&d.width==w&&d.height==h&&d.stride_bytes>=w*2&&!(d.stride_bytes&1)&&d.bytes>=std::uint64_t(d.stride_bytes)*h,"Display Q8 identity/geometry mismatch");
  if(input_rgb)budget.record("input_rgb",optical_frame,16,pixels*3);
  if(overlay)budget.record("overlay",optical_frame,16,pixels*3);
  if(mask_rgb)budget.record("mask_rgb",optical_frame,16,pixels*3);
  if(disparity_rgb)budget.record("disparity_rgb",optical_frame,16,pixels*3);
  if(disparity_q8)budget.record("disparity_q8",optical_frame,16,pixels*2);
  struct CloudPlan{const psdk_cloud_view* source;std::string key,frame;};std::vector<CloudPlan> clouds;
  require(result.cloud_count<=5&&(!result.cloud_count||result.clouds),"Invalid SDK display cloud list");
  for(unsigned i=0;i<result.cloud_count;++i){const auto& c=result.clouds[i];const auto name=bounded(c.name,sizeof(c.name));const auto key="clouds/"+name;if(!streamEnabled(settings,key))continue;
    require((!c.point_count||c.points)&&c.point_count<=20000000,"Invalid display cloud storage");const auto& coordinate=name=="camera"?optical_frame:body_frame;
    budget.record(key,coordinate,4,0);clouds.push_back({&c,key,coordinate});}
  struct MapPlan{const psdk_topview* source;std::string key;};std::vector<MapPlan> maps;
  require(result.map_count<=3&&(!result.map_count||result.maps),"Invalid SDK display map list");
  for(unsigned i=0;i<result.map_count;++i){const auto& map=result.maps[i];const auto key="maps/"+bounded(map.name,sizeof(map.name));if(!streamEnabled(settings,key)||!map.rgb||!map.rgb_bytes)continue;
    require(map.image_width&&map.image_height&&map.image_width<=4096&&map.image_height<=4096&&map.rgb_bytes==std::uint64_t(map.image_width)*map.image_height*3,"Invalid display RGB map storage");
    budget.record(key,optical_frame,16,map.rgb_bytes);maps.push_back({&map,key});}
  std::string result_text;if(streamEnabled(settings,"result")){result_text=resultJson(result);budget.record("result",optical_frame,4,result_text.size());}
  std::size_t slip_size=0;const char* slip_json=nullptr;
  if(slip&&slip->enabled&&streamEnabled(settings,"slip")){
    require(slip->frame_id==result.frame_id,"Display slip identity mismatch");if(slip->details_json){slip_json=slip->details_json;slip_size=jsonLength(slip_json);budget.record("slip",body_frame,4,slip_size);}}
  out.packets.reserve(budget.packets());
  // All non-point payloads and all packet headers now fit. No SDK/camera pointer
  // is retained in the returned Frame; plans only live during this synchronous call.
  std::vector<std::uint8_t> rgb;
  if(input_rgb||overlay){
    if(result.rgb&&result.image_width==w&&result.image_height==h&&result.rgb_bytes==pixels*3)rgb.assign(result.rgb,result.rgb+pixels*3);
    else demo_vis::convertRgbInto(imageView(input),rgb);
    if(input_rgb){if(overlay)add("input_rgb",optical_frame,rgbImage(w,h,rgb));else add("input_rgb",optical_frame,rgbImage(w,h,std::move(rgb)));}
  }
  if(mask_rgb){
    std::vector<std::uint8_t> colors(pixels*3);for(std::size_t i=0;i<pixels;++i){const auto c=palette(ai.segmentation_mask[i]);std::copy(c.begin(),c.end(),colors.data()+i*3);}add("mask_rgb",optical_frame,rgbImage(w,h,std::move(colors)));
  }
  if(overlay){
    perception::DetectionResult detection;detection.width=int(w);detection.height=int(h);
    demo_vis::Style style;style.mask_alpha=float(settings.publish.overlay_alpha);style.segmentation_names={"no_grass","grass","flagstone"};
    require(ai.detection_count<=4096&&(!ai.detection_count||ai.detections),"Invalid display detection storage");
    for(std::uint64_t i=0;i<ai.detection_count;++i){const auto& d=ai.detections[i];require(d.class_id>=0&&d.class_id<=4095,"Display class ID exceeds bound");
      detection.detections.push_back({d.class_id,d.confidence,d.x1,d.y1,d.x2,d.y2});
      if(style.detection_names.size()<=std::size_t(d.class_id))style.detection_names.resize(std::size_t(d.class_id)+1);
      style.detection_names[d.class_id]=bounded(d.class_name,sizeof(d.class_name));}
    perception::SegmentationResult segmentation;if(mask){segmentation.width=int(w);segmentation.height=int(h);segmentation.mask.assign(ai.segmentation_mask,ai.segmentation_mask+pixels);}
    demo_vis::overlayRgb(rgb,int(w),int(h),&detection,mask?&segmentation:nullptr,style);add("overlay",optical_frame,rgbImage(w,h,std::move(rgb)));
  }
  if(disparity_rgb||disparity_q8){
    std::vector<std::uint8_t> raw,colors;if(disparity_q8)raw.resize(pixels*2);if(disparity_rgb)colors.resize(pixels*3);
    for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){std::uint16_t q;std::memcpy(&q,reinterpret_cast<const std::uint8_t*>(d.data)+std::size_t(y)*d.stride_bytes+x*2,2);const auto i=std::size_t(y)*w+x;
      if(!raw.empty()){raw[i*2]=std::uint8_t(q);raw[i*2+1]=std::uint8_t(q>>8);}
      if(!colors.empty()&&q){const double t=std::clamp((q/256.-settings.publish.disparity_min_px)/(settings.publish.disparity_max_px-settings.publish.disparity_min_px),0.,1.);
        for(unsigned c=0;c<3;++c){const double value=std::clamp(1.5-std::abs(4*t-(3.-c)),0.,1.);colors[i*3+c]=std::uint8_t(std::lround(value*255));}}
    }
    if(!raw.empty())add("disparity_q8",optical_frame,Image{w,h,w*2,ImageEncoding::Mono16,std::move(raw)});
    if(!colors.empty())add("disparity_rgb",optical_frame,rgbImage(w,h,std::move(colors)));
  }
  for(const auto& plan:clouds){const auto& c=*plan.source;PointCloud cloud;
    const auto limit=std::min(std::size_t(settings.publish.max_points),std::size_t(budget.remaining()/24));
    const auto stride=std::max(std::size_t(settings.publish.point_stride),limit?std::size_t(c.point_count/limit+(c.point_count%limit!=0)):std::size_t(c.point_count+1));
    // Count the selected finite points before reserving. This avoids retaining
    // max_points capacity for empty/NaN clouds and lets later clouds use real remaining bytes.
    std::size_t count=0;for(std::uint64_t j=0;j<c.point_count&&count<limit;j+=stride){const auto& p=c.points[j];if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))++count;}
    budget.take(plan.key,std::uint64_t(count)*24);cloud.points.reserve(count);
    for(std::uint64_t j=0;j<c.point_count&&cloud.points.size()<count;j+=stride){const auto& p=c.points[j];if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      cloud.points.push_back({p.x,p.y,p.z,(std::uint32_t(p.r)<<16)|(std::uint32_t(p.g)<<8)|p.b,p.confidence,p.label});}
    add(plan.key,plan.frame,std::move(cloud));
  }
  for(const auto& plan:maps){const auto& map=*plan.source;
    add(plan.key,optical_frame,rgbImage(map.image_width,map.image_height,{map.rgb,map.rgb+map.rgb_bytes}));
  }
  if(streamEnabled(settings,"result"))add("result",optical_frame,Text{std::move(result_text)});
  if(slip_json)add("slip",body_frame,Text{std::string(slip_json,slip_size)});
  std::string error;if(!out.packets.empty()&&!validate_frame(out,limits,&error))throw std::invalid_argument("RViz display frame: "+error);
  return out;
}
SdkPublisher::SdkPublisher(const std::filesystem::path& config):settings_(loadSettings(config)){
  if(!settings_.enabled)return;PublisherOptions options;options.bind_address=settings_.transport.bind_address;options.port=settings_.transport.remote_port;
  options.send_timeout_ms=settings_.transport.send_timeout_ms;options.limits=wireLimits(settings_);
  publisher_=std::make_unique<AsyncPublisher>(options);std::string error;if(!publisher_->start(&error))throw std::runtime_error("RViz publisher: "+error);
}
SdkPublisher::~SdkPublisher()=default;
bool SdkPublisher::wantsSlip()const{return settings_.enabled&&streamEnabled(settings_,"slip");}
bool SdkPublisher::connected()const{return publisher_&&publisher_->hasClient();}
PublisherStats SdkPublisher::stats()const{return publisher_?publisher_->stats():PublisherStats{};}
bool SdkPublisher::publish(const psdk_frame& input,const psdk_full_result_view& result,const std::string& optical_frame,const std::string& body_frame,const psdk_slip_view* slip){
  ++seen_;if(!publisher_||!publisher_->hasClient()||(seen_-1)%std::uint64_t(settings_.publish.every_n))return false;
  const auto now=std::chrono::steady_clock::now();if(last_.time_since_epoch().count()&&std::chrono::duration<double>(now-last_).count()<1./settings_.publish.max_fps)return false;
  last_=now; // Failed conversions/rejections consume this attempt's display-rate slot too.
  auto frame=sdkFrame(settings_,input,result,optical_frame,body_frame,slip);if(frame.packets.empty())return false;
  std::string error;const bool sent=publisher_->publish(std::move(frame),&error);
  if(!sent&&publisher_->hasClient())throw std::runtime_error("RViz display publish rejected: "+error);
  return sent; // A client disappearing during conversion is normal; the transport can reconnect.
}
} // namespace perception::rviz
