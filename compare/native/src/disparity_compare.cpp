#include "disparity_compare.hpp"
#include <array>
#include <cstring>
#include <limits>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

namespace perception_compare::detail {
namespace {
void validate(const ComparisonRoi& r) {
  if(r.source_width<2||r.source_height<2||r.source_width>4096||r.source_height>4096||
     r.x<0||r.y<0||r.width<2||r.height<2||r.width>r.source_width||r.height>r.source_height||
     r.x>r.source_width-r.width||r.y>r.source_height-r.height||
     ((r.x|r.y|r.width|r.height|r.source_width|r.source_height)&1))
    throw std::runtime_error("Comparison ROI must be even, inside the full rectified source");
}
int dimension(const Json& j,const char* key) {
  const auto n=uint64(field(j,key),std::string("comparison_roi.")+key);
  if(n>4096)throw std::runtime_error("Comparison dimension exceeds 4096");
  return int(n);
}
ComparisonRoi roiFromRun(const Json& metadata) {
  const auto& j=field(metadata,"comparison_roi");
  if(j.kind==Json::Kind::Object) {
    const auto& k=field(metadata,"intrinsics");
    ComparisonRoi r{dimension(j,"x"),dimension(j,"y"),dimension(j,"width"),dimension(j,"height"),
      field(j,"source_width").kind==Json::Kind::Null?dimension(k,"width"):dimension(j,"source_width"),
      field(j,"source_height").kind==Json::Kind::Null?dimension(k,"height"):dimension(j,"source_height")};
    if(k.kind==Json::Kind::Object&&(r.source_width!=dimension(k,"width")||r.source_height!=dimension(k,"height")))
      throw std::runtime_error("Comparison ROI source dimensions differ from rectified intrinsics");
    validate(r);return r;
  }
  const auto& k=field(metadata,"intrinsics");
  const int w=dimension(k,"width"),h=dimension(k,"height");
  ComparisonRoi r{0,0,w,h,w,h};validate(r);return r;
}
fs::path within(const fs::path& root,const fs::path& base,const std::string& name) {
  const auto p=fs::weakly_canonical(base/fs::u8path(name));
  if(!contained(root,p))throw std::runtime_error("Disparity artifact path escapes comparison root");
  return p;
}
std::vector<std::uint16_t> readQ8(const fs::path& path,std::size_t pixels) {
  if(!fs::is_regular_file(path)||fs::file_size(path)!=pixels*2)
    throw std::runtime_error("Saved disparity must contain exact full-source little-endian uint16 Q8: "+path.u8string());
  std::ifstream f(path,std::ios::binary);std::vector<std::uint16_t> values(pixels);
  for(auto& v:values){const int a=f.get(),b=f.get();if(a<0||b<0)throw std::runtime_error("Truncated disparity");v=std::uint16_t(unsigned(a)|(unsigned(b)<<8));}
  return values;
}
void writeQ8(const fs::path& path,const std::vector<std::uint16_t>& values) {
  std::ofstream f(path,std::ios::binary|std::ios::trunc);
  for(auto v:values){f.put(char(v&255));f.put(char(v>>8));}
  f.close();if(!f)throw std::runtime_error("Cannot write cropped Q8");
}
void writeNpy(const fs::path& path,const std::vector<float>& values,int width,int height) {
  if(values.size()!=std::size_t(width)*height)throw std::runtime_error("NPY shape mismatch");
  std::string header="{'descr': '<f4', 'fortran_order': False, 'shape': ("+
    std::to_string(height)+", "+std::to_string(width)+"), }";
  const auto padding=(64-((10+header.size()+1)%64))%64;
  header.append(padding,' ');header+='\n';
  if(header.size()>65535)throw std::runtime_error("NPY header excessive");
  std::ofstream f(path,std::ios::binary|std::ios::trunc);
  f.write("\x93NUMPY\x01\x00",8);f.put(char(header.size()&255));f.put(char(header.size()>>8));
  f.write(header.data(),std::streamsize(header.size()));
  for(float v:values){std::uint32_t bits=0;std::memcpy(&bits,&v,4);for(int b=0;b<4;++b)f.put(char((bits>>(8*b))&255));}
  f.close();if(!f)throw std::runtime_error("Cannot write float32 NPY");
}
struct Accumulator {
  std::size_t total=0,reference_valid=0,candidate_valid=0,joint=0,gt1=0,gt2=0,gt3=0;
  double abs_sum=0,square_sum=0,signed_sum=0;
  // Absolute Q8 differences are discrete 0..65535: exact quantiles need a
  // bounded histogram, not one double per pixel across all saved captures.
  std::array<std::uint64_t,65536> absolute_counts{};
  void append(const std::vector<std::uint16_t>& a,const std::vector<std::uint16_t>& b) {
    if(a.size()!=b.size())throw std::runtime_error("Disparity vectors differ in shape");
    total+=a.size();
    for(std::size_t i=0;i<a.size();++i){reference_valid+=a[i]>0;candidate_valid+=b[i]>0;
      if(!a[i]||!b[i])continue;
      ++joint;const double diff=(double(b[i])-double(a[i]))/256.0,ad=std::abs(diff);
      abs_sum+=ad;square_sum+=diff*diff;signed_sum+=diff;gt1+=ad>1;gt2+=ad>2;gt3+=ad>3;
      ++absolute_counts[std::size_t(std::abs(int(b[i])-int(a[i])))];
    }
  }
  Json finish() {
    const auto quantile=[&](double q) {
      const double at=double(joint-1)*q;const auto lo=std::size_t(at),hi=std::min(lo+1,joint-1);
      std::size_t cumulative=0;double a=0,b=0;bool found=false;
      for(std::size_t v=0;v<absolute_counts.size();++v){cumulative+=absolute_counts[v];
        if(!found&&cumulative>lo){a=double(v)/256;found=true;}
        if(cumulative>hi){b=double(v)/256;break;}}
      return a*(1-(at-lo))+b*(at-lo);
    };
    return obj({{"roi_pixels",integer(total)},{"original_valid_pixels",integer(reference_valid)},
      {"stereo_valid_pixels",integer(candidate_valid)},{"both_valid_pixels",integer(joint)},
      {"original_coverage_ratio",total?number(double(reference_valid)/total):Json{}},
      {"stereo_coverage_ratio",total?number(double(candidate_valid)/total):Json{}},
      {"both_valid_coverage_ratio",total?number(double(joint)/total):Json{}},
      {"mae_px",joint?number(abs_sum/joint):Json{}},{"rmse_px",joint?number(std::sqrt(square_sum/joint)):Json{}},
      {"median_abs_difference_px",joint?number(quantile(.5)):Json{}},
      {"p95_abs_difference_px",joint?number(quantile(.95)):Json{}},
      {"bias_stereo_minus_original_px",joint?number(signed_sum/joint):Json{}},
      {"abs_difference_gt_1px_ratio",joint?number(double(gt1)/joint):Json{}},
      {"abs_difference_gt_2px_ratio",joint?number(double(gt2)/joint):Json{}},
      {"abs_difference_gt_3px_ratio",joint?number(double(gt3)/joint):Json{}}});
  }
};
using Saved=std::map<std::string,Json>;
bool accepted(const Json& j) {
  if(field(j,"warmup").boolean)return false;
  if(field(j,"success").kind==Json::Kind::Boolean&&!field(j,"success").boolean)return false;
  const auto status=textValue(field(j,"status"));if(status=="failed"||status=="error")return false;
  const auto& error=field(j,"error");if(error.kind!=Json::Kind::Null&&
    !(error.kind==Json::Kind::String&&error.text.empty())&&
    !(error.kind==Json::Kind::Object&&error.object.empty()))return false;
  const auto& code=field(j,"code");if(code.kind!=Json::Kind::Null&&(!numeric(code)||*numeric(code)!=0))return false;
  return true;
}
Saved savedRows(const fs::path& root,const char* method) {
  Saved result;std::ifstream f(root/method/"frames.jsonl");if(!f)return result;
  std::string line;while(std::getline(f,line)){if(line.empty())continue;auto j=parse(line,"disparity saved row");
    if(!accepted(j))continue;
    const auto stem=textValue(field(j,"stem"));
    if(stem.empty())throw std::runtime_error("Disparity pair requires stable stem");
    const auto repeat=field(j,"repeat").kind==Json::Kind::Number?field(j,"repeat").text:"0";
    if(!result.emplace(stem+'\0'+repeat,std::move(j)).second)throw std::runtime_error("Duplicate disparity stem/repeat");
  }return result;
}
std::string csv(const std::string& s){std::string out="\"";for(char c:s){if(c=='\"')out+='\"';out+=c;}return out+'\"';}
std::vector<float> pixels(const std::vector<std::uint16_t>& q) {
  std::vector<float> out(q.size());for(std::size_t i=0;i<q.size();++i)out[i]=q[i]?float(q[i])/256.f:std::numeric_limits<float>::quiet_NaN();return out;
}
void png(const fs::path& path,const cv::Mat& image) {
  if(!cv::imwrite(path.u8string(),image))throw std::runtime_error("Cannot write disparity visualization");
}
void colored(const fs::path& path,const std::vector<float>& v,int w,int h,double maximum) {
  cv::Mat gray(h,w,CV_8UC1,cv::Scalar(0)),valid(h,w,CV_8UC1,cv::Scalar(0)),color;
  for(std::size_t i=0;i<v.size();++i)if(std::isfinite(v[i])){
    gray.data[i]=static_cast<unsigned char>(std::lround(std::clamp(double(v[i])/maximum,0.0,1.0)*255));valid.data[i]=255;}
  cv::applyColorMap(gray,color,cv::COLORMAP_TURBO);color.setTo(cv::Scalar(0,0,0),valid==0);png(path,color);
}
Json images(const fs::path& root,const fs::path& dir,const ComparisonRoi& r,
            const std::vector<std::uint16_t>& a,const std::vector<std::uint16_t>& b,
            const fs::path& left_png) {
  auto artifacts=object();auto add=[&](const char* key,const char* filename){artifacts.object[key]=str(fs::relative(dir/filename,root).generic_u8string());};
  writeQ8(dir/"original_q8.u16",a);writeQ8(dir/"stereo_q8.u16",b);add("original_q8","original_q8.u16");add("stereo_q8","stereo_q8.u16");
  const auto af=pixels(a),bf=pixels(b);std::vector<float> delta(a.size(),std::numeric_limits<float>::quiet_NaN());
  cv::Mat av(r.height,r.width,CV_8UC1,cv::Scalar(0)),bv=av.clone(),both=av.clone();
  for(std::size_t i=0;i<a.size();++i){if(a[i])av.data[i]=255;if(b[i])bv.data[i]=255;
    if(a[i]&&b[i]){both.data[i]=255;delta[i]=std::abs(bf[i]-af[i]);}}
  writeNpy(dir/"original_px.npy",af,r.width,r.height);writeNpy(dir/"stereo_px.npy",bf,r.width,r.height);
  writeNpy(dir/"abs_difference_px.npy",delta,r.width,r.height);
  add("original_npy","original_px.npy");add("stereo_npy","stereo_px.npy");add("abs_difference_npy","abs_difference_px.npy");
  colored(dir/"original_disparity.png",af,r.width,r.height,128);colored(dir/"stereo_disparity.png",bf,r.width,r.height,128);
  colored(dir/"abs_difference.png",delta,r.width,r.height,10);png(dir/"original_valid.png",av);png(dir/"stereo_valid.png",bv);png(dir/"both_valid.png",both);
  add("original_png","original_disparity.png");add("stereo_png","stereo_disparity.png");add("abs_difference_png","abs_difference.png");
  add("original_valid_png","original_valid.png");add("stereo_valid_png","stereo_valid.png");add("both_valid_png","both_valid.png");
  if(fs::is_regular_file(left_png)){
    auto left=cv::imread(left_png.u8string(),cv::IMREAD_COLOR);
    if(left.cols!=r.source_width||left.rows!=r.source_height)throw std::runtime_error("Prepared left image dimensions differ from source ROI");
    png(dir/"left_roi.png",left(cv::Rect(r.x,r.y,r.width,r.height)));add("left_roi_png","left_roi.png");
  }
  return artifacts;
}
void html(const fs::path& root,const Json& report) {
  const std::string before=R"HTML(<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>同ROI视差差异</title><style>body{font:16px system-ui;margin:24px;background:#101820;color:#e3edf4}a{color:#8ed9ff}select{max-width:100%;padding:8px}main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}figure{margin:0}img{width:100%;image-rendering:pixelated;background:#000}pre{white-space:pre-wrap;font-size:13px}.notice{background:#203142;padding:14px}figcaption{padding:8px}#files a{display:inline-block;margin:8px}</style><h1>同一源坐标 ROI：原 Q8 与新 Stereo</h1><p class="notice">原始 Q8 仅作为 reference，不是 Ground Truth。仅双方非零的有效交集计算差异；无交集时指标为 null。差异大小不能独立证明哪种算法更准确。全部质量分析位于 runner 测量窗口之外。</p><p><a href="index.html">返回性能与点云</a> · <a href="disparity_comparison.json">JSON</a> · <a href="disparity_comparison.csv">CSV</a></p><p id="status"></p><pre id="roi"></pre><p>两组视差共享固定色标 0–128 px；绝对差异固定 0–10 px，超范围仅在 PNG 中饱和。黑色表示无效，原始 Q8 与 float32 NPY 保留数值。mask 白色=有效。</p><select id="pair"></select><pre id="metrics"></pre><main id="images"></main><div id="files"></div><script>const report=)HTML";
  const std::string after=R"HTML(;const $=id=>document.getElementById(id);$('status').textContent=`状态：${report.status}；已分析保存配对 ${report.analyzed_pair_count} / ${report.saved_pair_count}；${report.message}`;$('roi').textContent=JSON.stringify({roi:report.comparison_roi,aggregate:report.aggregate},null,2);const entries=report.pairs.filter(p=>p.artifacts);for(let i=0;i<entries.length;i++){const o=document.createElement('option');o.value=i;o.textContent=`${entries[i].stem} / repeat ${entries[i].repeat}`;$('pair').append(o)}function show(){const p=entries[Number($('pair').value)];$('images').replaceChildren();$('files').replaceChildren();if(!p)return;$('metrics').textContent=JSON.stringify(p.metrics,null,2);for(const [key,label]of [['left_roi_png','左图 ROI'],['both_valid_png','双方有效交集'],['original_png','原 Q8 reference'],['stereo_png','新 Stereo'],['abs_difference_png','绝对差异（交集）'],['original_valid_png','原 Q8 有效区域'],['stereo_valid_png','新 Stereo 有效区域']]){if(!p.artifacts[key])continue;const f=document.createElement('figure'),im=document.createElement('img'),c=document.createElement('figcaption');im.src=p.artifacts[key];c.textContent=label;f.append(c,im);$('images').append(f)}for(const[key,path]of Object.entries(p.artifacts)){if(!key.includes('npy')&&!key.includes('q8'))continue;const a=document.createElement('a');a.href=path;a.textContent=key;a.download='';$('files').append(a)}}$('pair').onchange=show;show();</script></html>)HTML";
  writeText(root/"disparity_compare.html",before+dump(report)+after);
}
} // namespace

ComparisonRoi comparisonRoi(const std::string& mode,int model_width,int model_height,const Intrinsics& k) {
  ComparisonRoi r{0,0,k.width,k.height,k.width,k.height};
  if(mode=="crop") {
    if(model_width<=0||model_height<=0)throw std::runtime_error("Crop comparison requires declared model input_width/input_height; original mode does not load the Stereo model");
    if(model_width!=k.width||model_height>k.height)
      throw std::runtime_error("Crop comparison requires equal source/model width and model height <= source height; resize is forbidden");
    r.y=k.height-model_height;r.width=model_width;r.height=model_height;
  } else if(mode!="resize"&&mode!="strict")throw std::runtime_error("Unknown comparison input mode");
  validate(r);return r;
}
Json comparisonRoiJson(const ComparisonRoi& r,const Intrinsics& k) {
  validate(r);
  const auto intrinsics=[&](bool crop){return obj({{"width",integer(crop?r.width:k.width)},
    {"height",integer(crop?r.height:k.height)},{"fx",number(k.fx)},{"fy",number(k.fy)},
    {"cx",number(k.cx-(crop?r.x:0))},{"cy",number(k.cy-(crop?r.y:0))},{"bxf",number(k.bxf)}});};
  return obj({{"x",integer(r.x)},{"y",integer(r.y)},{"width",integer(r.width)},{"height",integer(r.height)},
    {"source_width",integer(r.source_width)},{"source_height",integer(r.source_height)},
    {"coordinate_frame",str("source_rectified_left")},
    {"outside_roi_policy",str(r.x||r.y||r.width!=r.source_width||r.height!=r.source_height?"invalid_q8_zero":"full_frame")},
    {"resized",boolean(false)},{"source_intrinsics",intrinsics(false)},{"crop_intrinsics",intrinsics(true)}});
}
Json comparisonRoiIdentity(const Json& run) {
  if(field(run,"comparison_roi").kind!=Json::Kind::Object&&field(run,"intrinsics").kind!=Json::Kind::Object)return {};
  const auto r=roiFromRun(run);
  auto result=obj({{"x",integer(r.x)},{"y",integer(r.y)},{"width",integer(r.width)},{"height",integer(r.height)},
    {"source_width",integer(r.source_width)},{"source_height",integer(r.source_height)},
    {"outside_roi_policy",str(r.x||r.y||r.width!=r.source_width||r.height!=r.source_height?"invalid_q8_zero":"full_frame")}});
  const auto& explicit_roi=field(run,"comparison_roi");
  if(explicit_roi.kind==Json::Kind::Object){
    if(field(explicit_roi,"resized").boolean)throw std::runtime_error("Comparison ROI must not resize source geometry");
    const auto policy=textValue(field(explicit_roi,"outside_roi_policy"));
    if(!policy.empty()&&policy!=textValue(field(result,"outside_roi_policy")))throw std::runtime_error("Comparison ROI mask policy differs from source geometry");
  }
  return result;
}
std::vector<std::uint16_t> sliceDisparity(const std::vector<std::uint16_t>& source,const ComparisonRoi& r) {
  validate(r);if(source.size()!=std::size_t(r.source_width)*r.source_height)throw std::runtime_error("Disparity source shape differs from comparison ROI");
  std::vector<std::uint16_t> out(std::size_t(r.width)*r.height);
  for(int y=0;y<r.height;++y)std::copy_n(source.begin()+std::size_t(y+r.y)*r.source_width+r.x,r.width,out.begin()+std::size_t(y)*r.width);
  return out;
}
Json compareDisparity(const std::vector<std::uint16_t>& a,const std::vector<std::uint16_t>& b) {
  Accumulator stats;stats.append(a,b);return stats.finish();
}
Json generateDisparityComparison(const fs::path& root,const Json& metadata) {
  const auto original=savedRows(root,"original"),stereo=savedRows(root,"stereo");
  auto report=obj({{"schema_version",integer(1)},{"status",str("not_saved")},
    {"message",str("未保存双方同帧 disparity_q8.u16；save_every_n=0 时不能计算质量差异。")},
    {"comparison_roi",field(metadata,"comparison_roi")},{"analysis_scope",str("after_all_runner_measurement_windows")},
    {"reference",str("original_q8_reference_not_ground_truth")},{"ground_truth",boolean(false)},
    {"validity",str("Q8 > 0; differences only on both-valid intersection")},
    {"units",str("pixel_disparity = uint16_Q8 / 256; signed bias = stereo - original")},
    {"disparity_png_scale_px",obj({{"min",number(0)},{"max",number(128)}})},
    {"abs_difference_png_scale_px",obj({{"min",number(0)},{"max",number(10)}})},
    {"raw_encoding",str("little_endian_uint16_Q8; NPY little_endian_float32, invalid=NaN")},
    {"common_measured_pairs",integer(0)},{"saved_pair_count",integer(0)},{"analyzed_pair_count",integer(0)},
    {"missing_artifact_pairs",integer(0)},{"aggregate",Json{}},{"pairs",array()}});
  std::size_t common=0,saved=0,analyzed=0,missing=0;Accumulator aggregate;
  std::optional<ComparisonRoi> roi;
  std::ostringstream table;table<<"stem,repeat,status,roi_pixels,original_valid_pixels,stereo_valid_pixels,both_valid_pixels,original_coverage_ratio,stereo_coverage_ratio,both_valid_coverage_ratio,mae_px,rmse_px,median_abs_difference_px,p95_abs_difference_px,bias_stereo_minus_original_px,abs_difference_gt_1px_ratio,abs_difference_gt_2px_ratio,abs_difference_gt_3px_ratio\n";
  for(const auto& [key,a]:original){const auto it=stereo.find(key);if(it==stereo.end())continue;++common;const auto& b=it->second;
    const auto ad=textValue(field(a,"output_dir")),bd=textValue(field(b,"output_dir"));if(ad.empty()||bd.empty())continue;++saved;
    auto pair=obj({{"stem",field(a,"stem")},{"repeat",field(a,"repeat")},{"status",str("missing_artifact")},{"metrics",Json{}}});
    try {
      if(!roi)roi=roiFromRun(metadata);
      const auto& r=*roi;
      for(const auto* row:{&a,&b})if(field(*row,"comparison_roi").kind==Json::Kind::Object) {
        auto row_metadata=metadata;row_metadata.object["comparison_roi"]=field(*row,"comparison_roi");
        if(dump(comparisonRoiIdentity(row_metadata))!=dump(comparisonRoiIdentity(metadata)))
          throw std::runtime_error("Saved pair ROI differs from comparison run identity");
      }
      const auto af=within(root,root/"original",ad)/"disparity_q8.u16",bf=within(root,root/"stereo",bd)/"disparity_q8.u16";
      const auto aq=sliceDisparity(readQ8(af,std::size_t(r.source_width)*r.source_height),r),bq=sliceDisparity(readQ8(bf,std::size_t(r.source_width)*r.source_height),r);
      const auto dir=root/"disparity_roi"/("pair-"+std::to_string(saved));fs::create_directories(dir);
      const auto left=within(root,root/"prepared",textValue(field(a,"stem")))/"left.png";
      const auto artifacts=images(root,dir,r,aq,bq,left);auto metrics=compareDisparity(aq,bq);
      aggregate.append(aq,bq);++analyzed;pair.object["metrics"]=std::move(metrics);pair.object["artifacts"]=artifacts;
      pair.object["status"]=str(uint64(field(pair.object["metrics"],"both_valid_pixels"),"both_valid_pixels")?"available":"no_joint_valid_pixels");
    } catch(const std::exception& e){++missing;pair.object["error"]=str(e.what());}
    table<<csv(textValue(field(pair,"stem")))<<','<<field(pair,"repeat").text<<','<<textValue(field(pair,"status"));
    for(const auto* metric:{"roi_pixels","original_valid_pixels","stereo_valid_pixels","both_valid_pixels","original_coverage_ratio","stereo_coverage_ratio","both_valid_coverage_ratio","mae_px","rmse_px","median_abs_difference_px","p95_abs_difference_px","bias_stereo_minus_original_px","abs_difference_gt_1px_ratio","abs_difference_gt_2px_ratio","abs_difference_gt_3px_ratio"}){
      table<<',';const auto& value=field(field(pair,"metrics"),metric);if(value.kind==Json::Kind::Number)table<<value.text;
    }table<<'\n';report.object["pairs"].array.push_back(std::move(pair));
  }
  report.object["common_measured_pairs"]=integer(common);report.object["saved_pair_count"]=integer(saved);
  report.object["analyzed_pair_count"]=integer(analyzed);report.object["missing_artifact_pairs"]=integer(missing);
  if(analyzed){report.object["aggregate"]=aggregate.finish();report.object["status"]=str(missing?"partial":"available");
    report.object["message"]=str("同位置 ROI 视差差异；原 Q8 仅 reference，不是 GT，不据此判定精度优劣。所有统计仅使用双方有效交集。");}
  else if(saved){report.object["status"]=str("missing_artifacts");report.object["message"]=str("存在保存行但缺少完整有效的视差文件/ROI元数据，差异指标保持 null。");}
  writeJson(root/"disparity_comparison.json",report);writeText(root/"disparity_comparison.csv",table.str());html(root,report);
  return report;
}
} // namespace perception_compare::detail
