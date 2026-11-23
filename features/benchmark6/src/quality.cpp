#include "benchmark6.hpp"
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <numeric>
#include <set>

namespace b6 {
namespace {
using Rows=std::map<std::string,Json>;
Rows rows(const fs::path &path,bool jsonl) {
  Rows out;
  auto add=[&](Json j){const auto stamp=text(get(j,"timestamp_ns"),"timestamp_ns");if(!identity(stamp))throw std::invalid_argument("Zero capture timestamp");
    if(!out.emplace(stamp,std::move(j)).second)throw std::invalid_argument("Repeated capture timestamp in "+path.u8string());};
  if(jsonl){std::ifstream f(path);if(!f)throw std::invalid_argument("Missing manifest: "+path.u8string());std::string line;
    while(std::getline(f,line)){if(line.empty())continue;add(r33::json::JsonParser(line,path.u8string()).parse());}}
  else {const auto j=v3::readJson(path);const auto &frames=get(j,"frames");if(frames.kind!=Json::Kind::Array)throw std::invalid_argument("frames must be an array");for(const auto &f:frames.array)add(f);}
  return out;
}
std::vector<std::uint8_t> mask(const fs::path &path,int width,int height) {
  if(path.extension()!=".png")return readBytes(path,std::size_t(width)*height);
  const auto bytes=readBytes(path);int w=0,h=0,c=0;
  if(!stbi_info_from_memory(bytes.data(),int(bytes.size()),&w,&h,&c)||w!=width||h!=height||c!=1)
    throw std::invalid_argument("Label PNG must be a same-size single-channel class-ID image");
  if(stbi_is_16_bit_from_memory(bytes.data(),int(bytes.size())))throw std::invalid_argument("Segmentation labels must be 8-bit class IDs");
  auto *data=stbi_load_from_memory(bytes.data(),int(bytes.size()),&w,&h,&c,1);
  if(!data)throw std::runtime_error("Cannot decode label PNG");
  std::vector<std::uint8_t> out(data,data+std::size_t(w)*h);stbi_image_free(data);return out;
}
std::vector<float> floats(const fs::path &path,std::size_t pixels) {
  static_assert(sizeof(float)==4,"32-bit IEEE float required");const auto b=readBytes(path,pixels*4);std::vector<float> out(pixels);
  for(std::size_t i=0;i<pixels;++i){std::uint32_t bits=std::uint32_t(b[i*4])|(std::uint32_t(b[i*4+1])<<8)|(std::uint32_t(b[i*4+2])<<16)|(std::uint32_t(b[i*4+3])<<24);std::memcpy(&out[i],&bits,4);}return out;
}
Json ratio(double numerator,double denominator){return denominator>0?num(numerator/denominator):Json{};}
fs::path pathField(const fs::path &base,const Json &row,const char *key){return resolved(base,text(get(row,key),key));}
int dimension(const Json &row,const char *key){return count(get(row,key),key,2,4096);}
bool present(const Json &row,const char *key){return explicitValue(row,key);}
struct Event {double score;bool tp;};
double ap(const std::vector<Event> &input,std::size_t gt) {
  if(!gt)return 0;auto events=input;std::stable_sort(events.begin(),events.end(),[](const auto &a,const auto &b){return a.score>b.score;});
  std::vector<double> recall,precision;double tp=0,fp=0;
  for(const auto &e:events){e.tp?++tp:++fp;recall.push_back(tp/gt);precision.push_back(tp/(tp+fp));}
  double value=0;for(int k=0;k<=100;++k){double best=0;for(std::size_t i=0;i<recall.size();++i)if(recall[i]>=k/100.0)best=std::max(best,precision[i]);value+=best;}return value/101;
}
} // namespace

Json evaluate(const fs::path &dir,const fs::path &truth,int classes,Roi roi,bool require_digest) {
  auto out=object();out.object["kind"]=str("absolute_accuracy_against_independent_ground_truth");
  if(truth.empty()){out.object["status"]=str("unavailable_no_ground_truth");out.object["detection"]=Json{};out.object["segmentation"]=Json{};out.object["disparity"]=Json{};return out;}
  if(classes<1||classes>255)throw std::invalid_argument("segmentation_classes must be 1..255");
  const auto pred=rows(dir/"predictions.jsonl",true),gt=rows(truth,truth.extension()==".jsonl");
  std::map<int,std::size_t> gt_count;std::map<int,std::vector<Event>> events;
  std::vector<std::uint64_t> confusion(std::size_t(classes)*classes);
  std::size_t matched=0,det_frames=0,seg_frames=0,disp_frames=0,tp=0,fp=0,fn=0;
  std::size_t valid_gt=0,valid_both=0,bad3=0,d1=0;double absolute_error=0,squared_error=0;
  for(const auto &item:pred){const auto found=gt.find(item.first);if(found==gt.end())continue;const auto &p=item.second;const auto &g=found->second;
    if(require_digest&&!present(g,"left_digest"))continue;
    if(present(g,"left_digest")&&text(get(g,"left_digest"),"left_digest")!=text(get(p,"left_digest"),"left_digest"))continue;
    ++matched;const int w=dimension(p,"width"),h=dimension(p,"height");
    if(roi.x<0||roi.y<0||roi.width<1||roi.height<1||roi.x+roi.width>w||roi.y+roi.height>h)throw std::invalid_argument("Evaluation ROI outside image");
    if(present(g,"detections")){
      ++det_frames;auto detection=boxes(get(p,"detections"));const auto target=boxes(get(g,"detections"));std::vector<bool> used(target.size());
      for(const auto &b:target)++gt_count[b.label];
      std::stable_sort(detection.begin(),detection.end(),[](const Box&a,const Box&b){return a.score>b.score;});
      for(const auto &b:detection){double best=.5;std::size_t index=target.size();for(std::size_t i=0;i<target.size();++i)
        if(!used[i]&&target[i].label==b.label&&iou(b,target[i])>=best){best=iou(b,target[i]);index=i;}
        const bool ok=index!=target.size();events[b.label].push_back({b.score,ok});if(ok){used[index]=true;++tp;}else ++fp;}
      fn+=std::count(used.begin(),used.end(),false);
    }
    if(present(g,"segmentation_mask")){
      ++seg_frames;const auto a=mask(pathField(dir,p,"segmentation_mask"),w,h),b=mask(pathField(truth.parent_path(),g,"segmentation_mask"),w,h);
      for(std::size_t i=0;i<a.size();++i){if(b[i]==255)continue;if(a[i]>=classes||b[i]>=classes)throw std::invalid_argument("Segmentation class ID outside declared classes");++confusion[std::size_t(b[i])*classes+a[i]];}
    }
    if(present(g,"disparity_f32")){
      ++disp_frames;const auto a=readQ8(pathField(dir,p,"disparity_q8"),std::size_t(w)*h);const auto b=floats(pathField(truth.parent_path(),g,"disparity_f32"),std::size_t(w)*h);
      std::vector<std::uint8_t> valid;if(present(g,"disparity_valid_mask"))valid=mask(pathField(truth.parent_path(),g,"disparity_valid_mask"),w,h);
      for(int y=roi.y;y<roi.y+roi.height;++y)for(int x=roi.x;x<roi.x+roi.width;++x){auto i=std::size_t(y)*w+x;
        if(!std::isfinite(b[i])||b[i]<=0||(!valid.empty()&&!valid[i]))continue;++valid_gt;
        if(!a[i]){++bad3;++d1;continue;}++valid_both;const double e=std::abs(a[i]/256.0-b[i]);absolute_error+=e;squared_error+=e*e;
        if(e>3)++bad3;if(e>3&&e/b[i]>.05)++d1;}
    }
  }
  out.object["status"]=str(matched?"evaluated_with_reported_label_coverage":"unavailable_no_matching_ground_truth");
  out.object["prediction_frames"]=num(double(pred.size()));out.object["matched_ground_truth_frames"]=num(double(matched));out.object["label_coverage"]=ratio(double(matched),double(pred.size()));
  out.object["fully_labeled"]=boolean(!pred.empty()&&det_frames==pred.size()&&seg_frames==pred.size()&&disp_frames==pred.size());
  auto det=object();det.object["frames"]=num(double(det_frames));det.object["tp"]=num(double(tp));det.object["fp"]=num(double(fp));det.object["fn"]=num(double(fn));
  det.object["precision_iou50"]=det_frames?ratio(double(tp),double(tp+fp)):Json{};det.object["recall_iou50"]=det_frames?ratio(double(tp),double(tp+fn)):Json{};
  double sum_ap=0;auto per_class=array();for(const auto &entry:gt_count){auto c=object();c.object["class_id"]=integer(entry.first);c.object["ground_truth_boxes"]=num(double(entry.second));const double a=ap(events[entry.first],entry.second);c.object["ap50"]=num(a);sum_ap+=a;per_class.array.push_back(c);}
  det.object["map50_101point"]=gt_count.empty()?Json{}:num(sum_ap/gt_count.size());det.object["classes"]=per_class;
  det.object["scope"]=str("full left image; emitted post-threshold boxes; IoU>=0.5; 101-point interpolated AP; no crowd/area filtering");out.object["detection"]=det;
  auto seg=object();double correct=0,total=std::accumulate(confusion.begin(),confusion.end(),0.0),miou=0;int included=0;auto ious=array();
  for(int c=0;c<classes;++c){double row=0,col=0;for(int j=0;j<classes;++j){row+=confusion[std::size_t(c)*classes+j];col+=confusion[std::size_t(j)*classes+c];}double intersection=double(confusion[std::size_t(c)*classes+c]),u=row+col-intersection;correct+=intersection;ious.array.push_back(ratio(intersection,u));if(u){miou+=intersection/u;++included;}}
  seg.object["frames"]=num(double(seg_frames));seg.object["valid_pixels"]=num(total);seg.object["pixel_accuracy"]=ratio(correct,total);seg.object["miou"]=included?num(miou/included):Json{};seg.object["iou_per_class"]=ious;seg.object["scope"]=str("full left image; ground-truth class 255 ignored; absent-union classes excluded from mIoU");out.object["segmentation"]=seg;
  auto disp=object();disp.object["frames"]=num(double(disp_frames));disp.object["valid_gt_pixels"]=num(double(valid_gt));disp.object["valid_prediction_coverage"]=ratio(double(valid_both),double(valid_gt));
  disp.object["epe_valid_prediction_px"]=ratio(absolute_error,double(valid_both));disp.object["rmse_valid_prediction_px"]=valid_both?num(std::sqrt(squared_error/valid_both)):Json{};
  disp.object["bad3_rate"]=ratio(double(bad3),double(valid_gt));disp.object["d1_rate"]=ratio(double(d1),double(valid_gt));
  disp.object["scope"]=str("same crop ROI; independent float32 disparity in source-pixel units; invalid prediction counts as bad; EPE/RMSE only on valid predictions");out.object["disparity"]=disp;return out;
}

Json comparePredictions(const fs::path &left,const fs::path &right,Roi roi) {
  auto out=object();out.object["kind"]=str("same_input_output_consistency_not_absolute_accuracy");
  if(!fs::is_regular_file(left/"predictions.jsonl")||!fs::is_regular_file(right/"predictions.jsonl")){out.object["status"]=str("unavailable_missing_predictions");return out;}
  const auto a=rows(left/"predictions.jsonl",true),b=rows(right/"predictions.jsonl",true);
  std::size_t pairs=0,mask_equal=0,boxes_equal=0,n=0;double error=0;
  for(const auto &entry:a){const auto it=b.find(entry.first);if(it==b.end())continue;const auto &l=entry.second;const auto &r=it->second;
    if(text(get(l,"left_digest"),"left_digest")!=text(get(r,"left_digest"),"left_digest")||text(get(l,"right_digest"),"right_digest")!=text(get(r,"right_digest"),"right_digest"))continue;
    if(v3::jsonText(get(l,"intrinsics"))!=v3::jsonText(get(r,"intrinsics")))continue;
    ++pairs;const int w=dimension(l,"width"),h=dimension(l,"height");
    if(w!=dimension(r,"width")||h!=dimension(r,"height"))throw std::invalid_argument("Paired prediction shape differs");
    if(roi.x<0||roi.y<0||roi.width<1||roi.height<1||roi.x+roi.width>w||roi.y+roi.height>h)throw std::invalid_argument("Comparison ROI outside image");
    if(mask(pathField(left,l,"segmentation_mask"),w,h)==mask(pathField(right,r,"segmentation_mask"),w,h))++mask_equal;
    if(v3::jsonText(get(l,"detections"))==v3::jsonText(get(r,"detections")))++boxes_equal;
    const auto q=readQ8(pathField(left,l,"disparity_q8"),std::size_t(w)*h),p=readQ8(pathField(right,r,"disparity_q8"),std::size_t(w)*h);
    for(int y=roi.y;y<roi.y+roi.height;++y)for(int x=roi.x;x<roi.x+roi.width;++x){auto i=std::size_t(y)*w+x;if(q[i]&&p[i]){++n;error+=std::abs(double(q[i])-p[i])/256;}}
  }
  out.object["status"]=str(pairs?"paired":"unpaired_live_acquisitions_or_missing_identity");out.object["same_input_pairs"]=num(double(pairs));
  out.object["exact_segmentation_equal_pairs"]=num(double(mask_equal));out.object["exact_detection_equal_pairs"]=num(double(boxes_equal));
  out.object["disparity_mae_against_original_reference_px"]=ratio(error,double(n));out.object["valid_intersection_pixels"]=num(double(n));return out;
}
} // namespace b6
