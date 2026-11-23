#include "perception_temporal.hpp"
#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>
namespace r36::temporal {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
bool sameCalibration(const psdk_camera_intrinsics&a,const psdk_camera_intrinsics&b){
    return a.width==b.width&&a.height==b.height&&a.fx==b.fx&&a.fy==b.fy&&a.cx==b.cx&&a.cy==b.cy&&a.bxf==b.bxf;
}
bool sameGeometry(const psdk_frame&a,const psdk_frame&b){return a.width==b.width&&a.height==b.height&&a.stride==b.stride&&a.format==b.format;}
Image copyImage(const psdk_frame&f,std::uint64_t budget){
    if(f.struct_size!=sizeof(f)||f.abi_version!=PSDK_ABI_VERSION||!f.data||!f.width||!f.height||
       f.width>8192||f.height>8192||f.stride<f.width||((f.width|f.height|f.stride)&1)||
       (f.format!=PSDK_FORMAT_NV12&&f.format!=PSDK_FORMAT_I420))throw std::invalid_argument("Invalid temporal YUV frame");
    const std::uint64_t bytes=std::uint64_t(f.stride)*f.height*3/2;
    if(bytes>f.bytes||bytes>budget||bytes>std::numeric_limits<std::size_t>::max())throw std::invalid_argument("Temporal image exceeds storage/budget");
    Image out;out.metadata=f;out.metadata.data=nullptr;out.metadata.bytes=bytes;
    out.storage=std::make_shared<const std::vector<std::uint8_t>>(f.data,f.data+std::size_t(bytes));return out;
}
const Frame* previous(const Window&w){return w.sampled&&w.frames.size()>=2?w.frames[w.frames.size()-2].get():nullptr;}
float iou(const psdk_detection&a,const psdk_detection&b){
    const float intersection=std::max(0.f,std::min(a.x2,b.x2)-std::max(a.x1,b.x1))*std::max(0.f,std::min(a.y2,b.y2)-std::max(a.y1,b.y1));
    const float area=std::max(0.f,a.x2-a.x1)*std::max(0.f,a.y2-a.y1)+std::max(0.f,b.x2-b.x1)*std::max(0.f,b.y2-b.y1)-intersection;
    return area>0?intersection/area:0;
}
}
std::uint64_t Frame::payloadBytes()const {
    return (left.storage?left.storage->size():0)+(hires.storage&&hires.storage!=left.storage?hires.storage->size():0)+
        (disparity?disparity->size()*sizeof(std::uint16_t):0)+mask.size()+detections.size()*sizeof(psdk_detection);
}
psdk_temporal_frame Frame::view()const {
    psdk_temporal_frame v{};v.sequence=sequence;v.left=left.view();v.hires=hires.view();v.intrinsics=intrinsics;
    v.has_hires=hires.storage?1:0;v.has_disparity=disparity?1:0;v.has_inference=has_inference;
    if(disparity)v.disparity={sizeof(psdk_disparity_q8),PSDK_FULL_ABI_VERSION,left.metadata.width,left.metadata.height,left.metadata.width*2,0,
        disparity->data(),disparity->size()*2,left.metadata.frame_id,left.metadata.timestamp_ns};
    v.inference=inference;v.inference.detections=detections.data();v.inference.detection_count=detections.size();
    v.inference.segmentation_mask=mask.data();v.inference.segmentation_bytes=mask.size();return v;
}
History::History(Settings s):settings_(s){
    if(s.capacity<1||s.capacity>32||s.frame_interval<1||s.frame_interval>1000000||s.max_memory_mb<1||s.max_memory_mb>4096||
       s.difference_threshold>255||s.match_max_detections<1||s.match_max_detections>1024||!std::isfinite(s.match_iou_threshold)||
       s.match_iou_threshold<=0||s.match_iou_threshold>1)throw std::invalid_argument("Invalid history settings");
    clear();
}
void History::clear(){stored_={};stored_.capacity=settings_.capacity;stored_.interval=settings_.capacity==1?1:settings_.frame_interval;
    unique_count_=last_id_=last_timestamp_=0;has_last_=false;intrinsics_={};}
void History::trim(Window&w)const {
    w.payload_bytes=0;for(const auto& f:w.frames)w.payload_bytes+=f->payloadBytes();
    const auto budget=std::uint64_t(settings_.max_memory_mb)*1024*1024;
    if(!w.frames.empty()&&w.frames.back()->payloadBytes()>budget)throw std::invalid_argument("One temporal capture exceeds history.max_memory_mb");
    while(w.frames.size()>settings_.capacity||w.payload_bytes>budget){
        if(w.payload_bytes>budget)w.memory_limited=true;
        w.payload_bytes-=w.frames.front()->payloadBytes();w.frames.erase(w.frames.begin());
    }
}
History::Plan History::prepare(const psdk_full_frame&f)const {
    Plan p;p.window=stored_;p.window.sampled=false;p.window.memory_limited=false;p.window.reset_reason.clear();
    p.window.current_id=f.left.frame_id;p.window.current_timestamp=f.left.timestamp_ns;
    p.unique_count=unique_count_;p.last_id=last_id_;p.last_timestamp=last_timestamp_;p.intrinsics=intrinsics_;
    if(!f.left.timestamp_ns)throw std::invalid_argument("Temporal capture timestamp is required");
    if(has_last_){
        if(f.left.frame_id==last_id_&&f.left.timestamp_ns==last_timestamp_&&sameCalibration(f.intrinsics,intrinsics_)&&
           !stored_.frames.empty()&&sameGeometry(f.left,stored_.frames.back()->left.metadata)&&
           bool(f.recharge_hires)==bool(stored_.frames.back()->hires.storage)&&
           (!f.recharge_hires||sameGeometry(*f.recharge_hires,stored_.frames.back()->hires.metadata))){p.window.reset_reason="duplicate_ignored";return p;}
        std::string reason;
        if(f.left.timestamp_ns<=last_timestamp_)reason="timestamp_discontinuity";
        else if(!stored_.frames.empty()&&(!sameGeometry(f.left,stored_.frames.back()->left.metadata)||
                bool(f.recharge_hires)!=bool(stored_.frames.back()->hires.storage)||
                (f.recharge_hires&&!sameGeometry(*f.recharge_hires,stored_.frames.back()->hires.metadata))))reason="geometry_changed";
        else if(!sameCalibration(f.intrinsics,intrinsics_))reason="calibration_changed";
        if(!reason.empty()){p.window.frames.clear();p.window.payload_bytes=0;p.window.reset_reason=reason;p.unique_count=0;}
    }
    p.new_capture=true;p.last_id=f.left.frame_id;p.last_timestamp=f.left.timestamp_ns;p.intrinsics=f.intrinsics;
    p.window.sampled=p.unique_count%p.window.interval==0;++p.unique_count;
    if(!p.window.sampled)return p;
    auto current=std::make_shared<Frame>();current->sequence=p.unique_count;current->intrinsics=f.intrinsics;
    const auto budget=std::uint64_t(settings_.max_memory_mb)*1024*1024;
    current->left=copyImage(f.left,budget);
    if(f.recharge_hires){const auto& h=*f.recharge_hires;
        if(h.frame_id!=f.left.frame_id||h.timestamp_ns!=f.left.timestamp_ns)throw std::invalid_argument("Temporal GDC/left capture identity differs");
        if(h.data==f.left.data&&sameGeometry(h,f.left))current->hires=current->left;
        else current->hires=copyImage(h,budget-current->left.storage->size());
    }
    p.window.frames.push_back(current);trim(p.window);return p;
}
void History::complete(Plan&p,const psdk_result_view*ai,std::shared_ptr<const std::vector<std::uint16_t>>d)const {
    if(!p.window.sampled)return;
    auto completed=std::make_shared<Frame>(*p.window.frames.back());completed->disparity=std::move(d);
    if(ai){
        if(ai->frame_id!=p.last_id||ai->timestamp_ns!=p.last_timestamp)throw std::invalid_argument("Temporal inference capture identity differs");
        if((ai->detection_count&&!ai->detections)||(ai->segmentation_bytes&&!ai->segmentation_mask))throw std::invalid_argument("Missing temporal inference storage");
        const auto budget=std::uint64_t(settings_.max_memory_mb)*1024*1024;
        if(ai->segmentation_bytes>budget||ai->detection_count>budget/sizeof(psdk_detection)||
           ai->detection_count*sizeof(psdk_detection)+ai->segmentation_bytes+completed->payloadBytes()>budget)
            throw std::invalid_argument("Completed temporal capture exceeds history.max_memory_mb");
        completed->has_inference=true;completed->inference=*ai;completed->inference.detections=nullptr;completed->inference.segmentation_mask=nullptr;
        if(ai->detection_count)completed->detections.assign(ai->detections,ai->detections+ai->detection_count);
        if(ai->segmentation_bytes)completed->mask.assign(ai->segmentation_mask,ai->segmentation_mask+ai->segmentation_bytes);
    }
    p.window.frames.back()=std::move(completed);trim(p.window);
}
void History::commit(const Plan&p){stored_=p.window;if(p.new_capture){unique_count_=p.unique_count;last_id_=p.last_id;last_timestamp_=p.last_timestamp;intrinsics_=p.intrinsics;has_last_=true;}}
void difference(const Window&w,const Settings&s,Analysis&a){
    const auto start=Clock::now();const auto* old=previous(w);if(!old)return;
    const auto& cur=*w.frames.back();const auto x=old->left.view(),y=cur.left.view();
    if(x.width!=y.width||x.height!=y.height)return;
    std::uint64_t total=0;for(unsigned row=0;row<y.height;++row)for(unsigned col=0;col<y.width;++col){
        const auto d=unsigned(std::abs(int(x.data[std::size_t(row)*x.stride+col])-int(y.data[std::size_t(row)*y.stride+col])));
        total+=d;if(d>s.difference_threshold)++a.changed_luma;
    }
    a.difference_valid=true;a.previous_id=x.frame_id;a.previous_timestamp=x.timestamp_ns;a.mean_abs_luma=double(total)/(std::uint64_t(y.width)*y.height);
    if(old->has_inference&&cur.has_inference&&!old->mask.empty()&&old->mask.size()==cur.mask.size()&&old->inference.width==cur.inference.width&&old->inference.height==cur.inference.height){
        a.mask_pixels=cur.mask.size();for(std::size_t i=0;i<cur.mask.size();++i)if(old->mask[i]!=cur.mask[i])++a.changed_mask;
    }a.difference_ms=elapsed(start);
}
void match(const Window&w,const Settings&s,Analysis&a){
    const auto start=Clock::now();const auto* old=previous(w);if(!old)return;
    const auto& cur=*w.frames.back();if(!old->has_inference||!cur.has_inference)return;
    const auto& p=old->detections;const auto& c=cur.detections;
    const auto pn=std::min<std::size_t>(p.size(),s.match_max_detections),cn=std::min<std::size_t>(c.size(),s.match_max_detections);
    a.matching_valid=true;a.matching_truncated=pn!=p.size()||cn!=c.size();a.previous_id=old->left.metadata.frame_id;a.previous_timestamp=old->left.metadata.timestamp_ns;
    std::vector<psdk_temporal_match> candidates;
    for(std::size_t i=0;i<pn;++i)for(std::size_t j=0;j<cn;++j)if(p[i].class_id==c[j].class_id){const float score=iou(p[i],c[j]);
        if(score>=s.match_iou_threshold)candidates.push_back({unsigned(i),unsigned(j),p[i].class_id,score});}
    std::sort(candidates.begin(),candidates.end(),[](const auto&x,const auto&y){if(x.iou!=y.iou)return x.iou>y.iou;if(x.previous_index!=y.previous_index)return x.previous_index<y.previous_index;return x.current_index<y.current_index;});
    std::vector<bool> used_p(pn),used_c(cn);for(const auto& x:candidates)if(!used_p[x.previous_index]&&!used_c[x.current_index]){a.matches.push_back(x);used_p[x.previous_index]=true;used_c[x.current_index]=true;}
    a.match_ms=elapsed(start);
}
std::string json(const Window&w,const Analysis&a){
    std::ostringstream o;o.precision(12);o<<"{\"capacity\":"<<w.capacity<<",\"frame_interval\":"<<w.interval<<",\"count\":"<<w.frames.size()
      <<",\"sampled\":"<<(w.sampled?"true":"false")<<",\"ready\":"<<(w.frames.size()==w.capacity?"true":"false")<<",\"payload_bytes\":"<<w.payload_bytes
      <<",\"memory_limited\":"<<(w.memory_limited?"true":"false")<<",\"reset_reason\":"<<r33::json::quote(w.reset_reason)<<",\"current_frame_id\":"<<w.current_id
      <<",\"current_timestamp_ns\":"<<w.current_timestamp<<",\"previous_frame_id\":"<<a.previous_id<<",\"previous_timestamp_ns\":"<<a.previous_timestamp
      <<",\"capture_ms\":"<<a.capture_ms<<",\"difference_ms\":"<<a.difference_ms<<",\"match_ms\":"<<a.match_ms
      <<",\"difference_valid\":"<<(a.difference_valid?"true":"false")<<",\"mean_abs_luma\":"<<a.mean_abs_luma<<",\"changed_luma_pixels\":"<<a.changed_luma
      <<",\"compared_mask_pixels\":"<<a.mask_pixels<<",\"changed_mask_pixels\":"<<a.changed_mask<<",\"matching_valid\":"<<(a.matching_valid?"true":"false")
      <<",\"matching_truncated\":"<<(a.matching_truncated?"true":"false")<<",\"frames\":[";
    bool first=true;for(const auto& f:w.frames){if(!first)o<<',';first=false;o<<"{\"frame_id\":"<<f->left.metadata.frame_id<<",\"timestamp_ns\":"<<f->left.metadata.timestamp_ns<<",\"sequence\":"<<f->sequence<<'}';}
    o<<"],\"matches\":[";first=true;for(const auto&m:a.matches){if(!first)o<<',';first=false;o<<"{\"previous_index\":"<<m.previous_index<<",\"current_index\":"<<m.current_index<<",\"class_id\":"<<m.class_id<<",\"iou\":"<<m.iou<<'}';}o<<"]}";return o.str();
}
psdk_temporal_view view(const Window&w,const Analysis&a,const std::vector<psdk_temporal_frame>&frames,const std::string&text){
    psdk_temporal_view v{};v.struct_size=sizeof(v);v.abi_version=PSDK_TEMPORAL_ABI_VERSION;v.capacity=w.capacity;v.frame_interval=w.interval;
    v.frame_count=unsigned(frames.size());v.sampled=w.sampled;v.ready=w.frames.size()==w.capacity;v.difference_valid=a.difference_valid;
    v.current_frame_id=w.current_id;v.current_timestamp_ns=w.current_timestamp;v.payload_bytes=w.payload_bytes;v.frames=frames.data();
    v.previous_frame_id=a.previous_id;v.previous_timestamp_ns=a.previous_timestamp;v.changed_luma_pixels=a.changed_luma;v.mean_abs_luma=a.mean_abs_luma;
    v.compared_mask_pixels=a.mask_pixels;v.changed_mask_pixels=a.changed_mask;v.matches=a.matches.data();v.match_count=unsigned(a.matches.size());
    v.matching_valid=a.matching_valid;v.matching_truncated=a.matching_truncated;v.memory_limited=w.memory_limited;
    v.capture_ms=a.capture_ms;v.difference_ms=a.difference_ms;v.match_ms=a.match_ms;v.reset_reason=w.reset_reason.c_str();v.analysis_json=text.c_str();return v;
}
}
