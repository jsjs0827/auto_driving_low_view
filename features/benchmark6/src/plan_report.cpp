#include "benchmark6.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
namespace b6 {
namespace {
std::string boardPath(const Json &j,const char *key){auto s=text(get(j,key),key);
  if(!std::regex_match(s,std::regex("/[a-zA-Z0-9_./-]+"))||s.find("..")!=std::string::npos||s=="/")throw std::invalid_argument(std::string(key)+" must be a safe absolute board path");
  while(s.size()>1&&s.back()=='/')s.pop_back();return s;}
Json arrayOf(std::initializer_list<const char*> values){auto a=array();for(auto s:values)a.array.push_back(str(s));return a;}
Json pick(Json j,const std::string &path){std::size_t begin=0;while(begin<path.size()){auto e=path.find('.',begin);auto it=j.object.find(path.substr(begin,e-begin));if(it==j.object.end())return Json{};auto next=it->second;j=std::move(next);if(e==std::string::npos)break;begin=e+1;}return j;}
std::string cell(const Json &j){if(j.kind==Json::Kind::Null)return "NA";if(j.kind==Json::Kind::String)return j.text;return v3::jsonText(j,0);}
std::string escape(const std::string&s){std::string out;for(char c:s){if(c=='&')out+="&amp;";else if(c=='<')out+="&lt;";else if(c=='>')out+="&gt;";else if(c=='\"')out+="&quot;";else out+=c;}return out;}
Json error(const std::exception &e){auto j=object();j.object["status"]=str("error");j.object["error"]=str(e.what());return j;}
}
Suite loadSuite(const fs::path &file){Suite s;s.file=fs::absolute(file);s.project=s.file.parent_path();s.requested=v3::readJson(file);auto &j=s.requested;
  keys(j,{"schema_version","frames","warmup","quality_frames","timeout_ms","width","height","segmentation_classes","stereo_roi","board_root","board_output_base","camera","ros","ground_truth","visualization"},"benchmark6");
  count(get(j,"schema_version"),"schema_version",1,1);
  s.frames=count(get(j,"frames"),"frames",1,100000);s.warmup=count(get(j,"warmup"),"warmup",0,10000);
  s.quality_frames=count(get(j,"quality_frames"),"quality_frames",1,100000);s.timeout_ms=count(get(j,"timeout_ms"),"timeout_ms",100,600000);
  if(std::max(s.frames,s.quality_frames)+s.warmup>6000)throw std::invalid_argument("capture frames + warmup must be <=6000 for the bounded JSON manifest reader");
  s.width=count(get(j,"width"),"width",2,4096);s.height=count(get(j,"height"),"height",2,4096);
  s.segmentation_classes=count(get(j,"segmentation_classes"),"segmentation_classes",1,255);
  auto r=get(j,"stereo_roi");keys(r,{"x","y","width","height"},"stereo_roi");s.roi={count(get(r,"x"),"x",0,4096),count(get(r,"y"),"y",0,4096),count(get(r,"width"),"width",2,4096),count(get(r,"height"),"height",2,4096)};
  if((s.width|s.height|s.roi.y|s.roi.height)&1||s.roi.x||s.roi.width!=s.width||s.roi.y+s.roi.height!=s.height)throw std::invalid_argument("ROI must be an even bottom full-width crop");
  boardPath(j,"board_root");boardPath(j,"board_output_base");
  keys(get(j,"camera"),{"sdk_library","vendor_config"},"camera");boardPath(get(j,"camera"),"sdk_library");boardPath(get(j,"camera"),"vendor_config");
  const auto ros=get(j,"ros");keys(ros,{"bridge_port","left_topic","right_topic","q8_topic","left_info_topic","right_info_topic"},"ros");count(get(ros,"bridge_port"),"bridge_port",1024,65535);
  for(auto k:{"left_topic","right_topic","q8_topic","left_info_topic","right_info_topic"})topic(get(ros,k),k);
  keys(get(j,"ground_truth"),{"camera","offline","ros"},"ground_truth");
  auto vis=get(j,"visualization");keys(vis,{"save_rosbag","replay_hz","cloud_stride","disparity_max_px"},"visualization");flag(get(vis,"save_rosbag"),"save_rosbag");number(get(vis,"replay_hz"),"replay_hz",.1,100);count(get(vis,"cloud_stride"),"cloud_stride",1,64);number(get(vis,"disparity_max_px"),"disparity_max_px",1,256);
  return s;
}
Json makePlan(const Suite&s,const fs::path &out){auto p=s.requested;const auto host=fs::absolute(out).lexically_normal();const auto name=host.filename().u8string();
  if(!std::regex_match(name,std::regex("[a-zA-Z0-9_-]+")))throw std::invalid_argument("Output directory basename must use letters, digits, _ or -");
  p.object["host_output"]=str(host.u8string());p.object["board_root"]=str(boardPath(s.requested,"board_root"));p.object["board_output"]=str(boardPath(s.requested,"board_output_base")+"/"+name);
  p.object["suite_file"]=str(s.file.u8string());p.object["group_count"]=integer(6);p.object["measurement_schedule"]=str("det+seg in parallel, then disparity import or crop stereo; no mapping or display inside performance pass");
  auto gt=get(p,"ground_truth");for(auto source:sources())if(explicitValue(gt,source))gt.object[source]=str(resolved(s.file.parent_path(),text(get(gt,source),source)).u8string());p.object["ground_truth"]=gt;return p;
}
void preparePlan(const Suite&s,const fs::path &output){auto p=makePlan(s,output);auto out=fs::absolute(output);if(fs::exists(out))throw std::invalid_argument("Choose a NEW output directory");
  fs::create_directories(out/"board-config");fs::create_directories(out/"host-config");fs::create_directories(out/"logs");
  const auto root=text(get(p,"board_root"),"board_root"),board=text(get(p,"board_output"),"board_output");
  auto runtime=v3::readJson(s.project/"features/benchmark6/runtime/config/ros.json");
  runtime.object["models"].object["detection"]=str(root+"/models/x5/mmyolo0828.bin");runtime.object["models"].object["segmentation"]=str(root+"/models/x5/pidnet_0825_3classes.bin");runtime.object["models"].object["stereo"]=str(root+"/models/x5/r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin");
  auto &pipe=runtime.object["pipeline"];pipe.object["mapping_config"]=str(root+"/config/mapping.json");pipe.object["extrinsics"]=str(root+"/config/extrinsics.json");pipe.object["retain_clouds"]=boolean(false);pipe.object["render_maps"]=boolean(false);pipe.object["run_mapping"]=boolean(false);
  auto &in=runtime.object["input"];in.object["camera"]=get(s.requested,"camera");in.object["width"]=integer(s.width);in.object["height"]=integer(s.height);in.object["offline"].object["manifest"]=str(board+"/capture/manifest.json");in.object["offline"].object["intrinsics"]=str(board+"/capture/intrinsics.json");
  auto &ros=runtime.object["ros"];merge(ros,get(s.requested,"ros"));ros.object["acknowledged_replay"]=boolean(true);ros.object["ack_topic"]=str("/benchmark6/replay_ack");ros.object["reliable"]=boolean(true);ros.object["publish_hz"]=num(1000);ros.object["transport"]=str("tcp");
  auto &ex=runtime.object["experiment"];ex.object["methods"]=arrayOf({"optimized_q8","optimized_stereo"});ex.object["suites"]=arrayOf({"matched_roi"});ex.object["warmup"]=integer(s.warmup);ex.object["timeout_ms"]=integer(s.timeout_ms);ex.object["stereo_roi"]=get(s.requested,"stereo_roi");
  for(const auto &source:sources())for(const auto &pass:{std::string("performance"),std::string("quality")}){
    auto config=runtime;config.object["input"].object["source"]=str(source);config.object["experiment"].object["frames"]=integer(pass=="quality"?s.quality_frames:s.frames);config.object["experiment"].object["save_every_n"]=integer(pass=="quality"?1:0);
    v3::writeJson(out/"board-config"/(source+"_"+pass+".json"),config);
    if(source=="ros"){auto &off=config.object["input"].object["offline"];off.object["manifest"]=str((out/"capture/manifest.json").u8string());off.object["intrinsics"]=str((out/"capture/intrinsics.json").u8string());v3::writeJson(out/"host-config"/("ros_"+pass+".json"),config);}
  }
  runtime.object["input"].object["source"]=str("camera");ex.object["frames"]=integer(std::max(s.frames,s.quality_frames));ex.object["save_every_n"]=integer(0);v3::writeJson(out/"board-config/capture.json",runtime);
  v3::writeJson(out/"plan.json",p);std::ofstream(out/"STATUS.txt")<<"Prepared only. No device results exist until the remote Ubuntu script completes.\n";
}
Json report(const fs::path &output){auto out=fs::absolute(output);auto p=v3::readJson(out/"plan.json");auto report=object();auto groups=array();int complete=0;
  const auto r=get(p,"stereo_roi");Roi roi{count(get(r,"x"),"x",0),count(get(r,"y"),"y",0),count(get(r,"width"),"width"),count(get(r,"height"),"height")};
  const int classes=count(get(p,"segmentation_classes"),"segmentation_classes");
  for(auto source:sources())for(auto method:methods()){auto row=object();row.object["source"]=str(source);row.object["disparity"]=str(method);const auto base=out/"runs"/source/method;
    for(auto pass:{"performance","quality"}){try{const auto file=base/pass/"summary.json";auto value=fs::is_regular_file(file)?v3::readJson(file):object();value.object["status"]=str(fs::is_regular_file(file)&&flag(get(value,"complete",boolean(false)),"complete")?"complete":"missing_or_failed");if(fs::is_regular_file(base/pass/"failure.json"))value.object["failure"]=v3::readJson(base/pass/"failure.json");row.object[pass]=value;}catch(const std::exception&e){row.object[pass]=error(e);}}
    const bool ok=text(get(row.object["performance"],"status"),"status")=="complete"&&text(get(row.object["quality"],"status"),"status")=="complete";row.object["complete"]=boolean(ok);if(ok)++complete;
    try{const auto gt=get(get(p,"ground_truth"),source);row.object["accuracy"]=evaluate(base/"quality",gt.kind==Json::Kind::Null?fs::path{}:fs::u8path(text(gt,source)),classes,roi,source=="camera");}catch(const std::exception&e){row.object["accuracy"]=error(e);}
    row.object["original_disparity_generation_ms"]=Json{};row.object["visualization_directory"]=str("runs/"+source+"/"+method+"/quality");groups.array.push_back(row);
  }
  report.object["groups"]=groups;report.object["complete_groups"]=integer(complete);report.object["expected_groups"]=integer(6);report.object["status"]=str(complete==6?"complete":"partial_or_not_run");report.object["plan"]=p;
  auto comparisons=object();for(auto source:sources()){try{comparisons.object[source]=comparePredictions(out/"runs"/source/"original_q8/quality",out/"runs"/source/"stereo_crop/quality",roi);}catch(const std::exception&e){comparisons.object[source]=error(e);}}
  report.object["same_input_consistency"]=comparisons;
  auto cross=array();for(auto method:methods()){auto result=object();result.object["method"]=str(method);try{result.object["offline_vs_ros"]=comparePredictions(out/"runs/offline"/method/"quality",out/"runs/ros"/method/"quality",roi);}catch(const std::exception&e){result.object["offline_vs_ros"]=error(e);}cross.array.push_back(result);}report.object["transport_consistency"]=cross;
  report.object["measurement_notes"]=arrayOf({"No labels means accuracy unavailable, not zero. Original Q8 is a reference, not independent ground truth.","CPU 100% = one core; worker includes camera SDK threads. Host ROS publisher/bridge and visualizer resources excluded.","ROS uses actual Ubuntu DDS topics then a C++ bridge over ADB TCP to X5; these are not native board ROS subscription resource metrics.","BPU is device-wide driver-window utilization; unavailable samples are null.","Original disparity generation latency cannot be isolated from SDK; offline/ROS original Q8 generation happened before the run.","Performance and quality are separate runs; only performance metrics are tabulated. Camera runs acquire different live frames.","Clouds are derived after measurement from saved Q8 and calibration, not additional model inference."});
  const auto serialized=v3::jsonText(report)+"\n";
  writeBytes(out/"summary.json",serialized.data(),serialized.size());
  const std::vector<std::pair<std::string,std::string>> columns={{"source","source"},{"disparity","disparity"},{"perf_status","performance.status"},{"frames","performance.measured_frames"},{"FPS","performance.pipeline_fps"},{"compute_mean_ms","performance.metrics.compute_wall.mean"},{"compute_p95_ms","performance.metrics.compute_wall.p95"},{"acquire_mean_ms","performance.metrics.acquire.mean"},{"det_infer_mean_ms","performance.metrics.det_infer_wait.mean"},{"seg_infer_mean_ms","performance.metrics.seg_infer_wait.mean"},{"disparity_mean_ms","performance.metrics.disparity_prepare_or_stereo.mean"},{"CPU_pct","performance.process_cpu_percent"},{"BPU_mean_pct","performance.bpu.mean_pct"},{"BPU_p95_pct","performance.bpu.p95_pct"},{"RSS_mean_KiB","performance.metrics.rss_kib.mean"},{"RSS_max_KiB","performance.metrics.rss_kib.max"},{"AP50","accuracy.detection.map50_101point"},{"mIoU","accuracy.segmentation.miou"},{"EPE_px","accuracy.disparity.epe_valid_prediction_px"},{"D1_rate","accuracy.disparity.d1_rate"}};
  std::ofstream csv(out/"summary.csv"),md(out/"REPORT_zh.md"),html(out/"index.html");
  html<<"<!doctype html><meta charset='utf-8'><title>Benchmark6</title><style>body{font:14px sans-serif;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:6px}img{max-width:360px}</style><h1>Benchmark6</h1><p>"<<complete<<"/6 groups complete. NA = unavailable. Performance pass only. Accuracy requires independent labels.</p><table><tr>";
  md<<"# 六组测试报告\n\n已完成 "<<complete<<"/6 组。缺失值为 NA，详细阶段统计见 summary.json；无独立标注时不报告绝对精度。\n\n";
  for(std::size_t i=0;i<columns.size();++i){if(i)csv<<',';csv<<columns[i].first;html<<"<th>"<<columns[i].first<<"</th>";md<<"|"<<columns[i].first;}csv<<'\n';html<<"</tr>";md<<"|\n";for(auto c:columns)md<<"|---";md<<"|\n";
  for(const auto &g:groups.array){html<<"<tr>";for(std::size_t i=0;i<columns.size();++i){auto value=cell(pick(g,columns[i].second));if(i)csv<<',';csv<<'\"'<<value<<'\"';md<<'|'<<value;html<<"<td>"<<escape(value)<<"</td>";}csv<<'\n';md<<"|\n";html<<"</tr>";}html<<"</table><h2>Saved visualizations</h2>";
  for(auto source:sources())for(auto method:methods()){auto relative="runs/"+source+"/"+method+"/quality";html<<"<h3>"<<source<<" / "<<method<<"</h3><a href='"<<relative<<"/view.rviz'>RViz config</a> | <a href='visualization/bag/metadata.yaml'>rosbag metadata</a><br>";
    if(fs::is_directory(out/relative))for(auto entry:fs::directory_iterator(out/relative))if(entry.is_directory()&&entry.path().filename().u8string().find("frame-")==0){for(auto name:{"detection.png","segmentation_overlay.png","disparity_color.png"}){auto path=fs::relative(entry.path()/name,out).generic_u8string();if(fs::is_regular_file(out/path))html<<"<a href='"<<escape(path)<<"'><img loading='lazy' src='"<<escape(path)<<"'></a>";}break;}}
  md<<"\n精度与一致性口径、BPU/CPU/RSS口径及未完成组的原因见 summary.json。可视化由已保存的结果生成，不计入性能采样。\n";html<<"<p><a href='summary.json'>Detailed JSON + measurement scope</a></p>";
  return report;
}
} // namespace b6
