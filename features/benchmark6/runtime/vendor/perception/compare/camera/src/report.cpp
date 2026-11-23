#include "camera_compare.hpp"
#include "native_config_detail.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>

namespace camera_compare {
namespace {
namespace fs=std::filesystem;
using v3::Json;
using namespace v3::detail;
const Json& field(const Json& parent,const std::string& key) {
  static const Json missing;
  const auto found=parent.object.find(key);
  return found==parent.object.end()?missing:found->second;
}
std::optional<double> value(const Json& j) {
  if(j.kind!=Json::Kind::Number)return {};
  try{std::size_t used=0;const auto n=std::stod(j.text,&used);if(used==j.text.size()&&std::isfinite(n)&&n>=0)return n;}catch(const std::exception&){}
  return {};
}
std::string jsonString(const Json& j,const std::string& fallback={}){return j.kind==Json::Kind::String?j.text:fallback;}
Json statistics(std::vector<double> values,std::size_t expected) {
  auto result=object();result.object["count"]=num(double(values.size()));result.object["missing"]=num(double(expected-values.size()));
  for(const auto* key:{"mean","p50","p95","p99","min","max"})result.object[key]=Json{};
  if(values.empty())return result;
  std::sort(values.begin(),values.end());double sum=0;for(double v:values)sum+=v;
  result.object["mean"]=num(sum/values.size());result.object["min"]=num(values.front());result.object["max"]=num(values.back());
  for(const auto& p:std::map<std::string,double>{{"p50",.5},{"p95",.95},{"p99",.99}}){const double at=(values.size()-1)*p.second;const auto lo=std::size_t(at),hi=std::min(lo+1,values.size()-1);
    result.object[p.first]=num(values[lo]*(1-(at-lo))+values[hi]*(at-lo));}
  return result;
}
bool failed(const Json& row) {
  if(has(row,"success")&&field(row,"success").kind==Json::Kind::Boolean&&!field(row,"success").boolean)return true;
  const auto status=jsonString(field(row,"status"));if(status=="failed"||status=="error")return true;
  const auto& error=field(row,"error");if(error.kind!=Json::Kind::Null&&
    !(error.kind==Json::Kind::String&&error.text.empty())&&!(error.kind==Json::Kind::Object&&error.object.empty()))return true;
  const auto& code=field(row,"code");if(code.kind!=Json::Kind::Null&&(!value(code)||*value(code)!=0))return true;
  for(const auto& task:field(field(row,"task_trace"),"tasks").array)if(jsonString(field(task,"state"))=="failed")return true;
  return false;
}
const Json& metric(const Json& row,const std::string& name) {
  if(name=="acquire_ms"&&!has(row,name))return field(row,"load_ms");
  for(const auto* prefix:{"det_pre","det_infer","det_post","seg_pre","seg_infer","seg_post"})
    if(name==std::string(prefix)+"_ms"&&!has(row,name))return field(field(row,"inference_ms"),prefix);
  return field(row,name);
}
Json savedArtifacts(const fs::path& root,const fs::path& method,const Json& row) {
  const auto name=jsonString(field(row,"output_dir"));if(name.empty())return {};
  const auto dir=fs::weakly_canonical(method/fs::u8path(name));
  auto a=root.begin(),b=dir.begin();for(;a!=root.end();++a,++b)if(b==dir.end()||*a!=*b)throw std::runtime_error("Saved live artifact escapes experiment output");
  auto item=object();item.object["frame_id"]=field(row,"frame_id");item.object["timestamp_ns"]=field(row,"timestamp_ns");
  item.object["directory"]=str(fs::relative(dir,root).generic_u8string());item.object["files"]=object();
  for(const auto* filename:{"rgb.png","disparity.png","robot.ply","camera.ply","ground.ply","obstacle.ply","semantic.ply","disparity_q8.u16","result.json"})
    if(fs::is_regular_file(dir/filename))item.object["files"].object[filename]=str(fs::relative(dir/filename,root).generic_u8string());
  return item;
}
Json summarize(const fs::path& root,const std::string& method,const Json& suite) {
  const auto directory=root/method;auto complete=object(),run=object();
  if(fs::is_regular_file(directory/"complete.json"))complete=v3::readJson(directory/"complete.json");
  if(fs::is_regular_file(directory/"run.json"))run=v3::readJson(directory/"run.json");
  std::ifstream input(directory/"frames.jsonl");std::vector<Json> accepted;std::size_t warmup=0,failures=0,total=0;
  std::set<std::string> captures;std::string line;
  while(std::getline(input,line)){if(line.empty())continue;++total;
    auto row=r33::json::JsonParser(line,"live camera frame metrics").parse();
    if(field(row,"warmup").boolean){++warmup;continue;}
    if(failed(row)){++failures;continue;}
    const auto source=jsonString(field(row,"source_mode"),jsonString(field(row,"input_mode")));
    if(source!="camera")throw std::runtime_error("Live summary refuses non-camera frame metrics");
    for(const auto* name:{"frame_id","timestamp_ns"}) {
      const auto& identity=field(row,name);
      if(identity.kind!=Json::Kind::Number||identity.text.empty()||identity.text.find_first_not_of("0123456789")!=std::string::npos)
        throw std::runtime_error("Live frame identity must contain exact unsigned SDK integers");
      const auto n=std::stoull(identity.text);
      if(std::string(name)=="timestamp_ns"&&!n)throw std::runtime_error("Live capture timestamp must be positive");
    }
    const auto key=v3::jsonText(field(row,"frame_id"),0)+"/"+v3::jsonText(field(row,"timestamp_ns"),0);
    if(!captures.insert(key).second)throw std::runtime_error("Duplicate successful SDK capture identity inside one live method");
    accepted.push_back(std::move(row));
  }
  const std::vector<std::string> names{"acquire_ms","acquire_cpu_ms","load_ms","compute_wall_ms","sdk_call_ms","graph_ms","process_ms",
    "cpu_ms","cpu_wall_ms","total_wall_ms","cpu_total_ms","rss_kib","peak_rss_kib","save_ms","original_roi_mask_ms", "stereo_ms","stereo_pre_ms","stereo_backend_ms","stereo_post_ms",
    "det_pre_ms","det_infer_ms","det_post_ms","seg_pre_ms","seg_infer_ms","seg_post_ms"};
  auto metrics=object(),tasks=object(),saved=array();std::map<std::string,std::vector<double>> task_values;
  for(const auto& name:names){std::vector<double> values;for(const auto& row:accepted)if(const auto n=value(metric(row,name)))values.push_back(*n);
    metrics.object[name]=statistics(std::move(values),accepted.size());}
  for(const auto& row:accepted){for(const auto& task:field(field(row,"task_trace"),"tasks").array){const auto state=jsonString(field(task,"state"));if(state!="succeeded"&&state!="success")continue;
      if(const auto n=value(field(task,"ms")))task_values[jsonString(field(task,"task"))].push_back(*n);}
    auto item=savedArtifacts(root,directory,row);if(item.kind!=Json::Kind::Null)saved.array.push_back(std::move(item));}
  for(auto& item:task_values)tasks.object[item.first]=statistics(std::move(item.second),accepted.size());
  auto sum=[&](const char* name){double result=0;std::size_t count=0;for(const auto& row:accepted)if(const auto n=value(metric(row,name))){result+=*n;++count;}return std::pair{result,count};};
  const auto compute=sum("compute_wall_ms"),acquire=sum("acquire_ms"),total_wall=sum("total_wall_ms");
  double compute_cpu=0,matching_cpu_wall=0;std::size_t compute_cpu_samples=0;
  for(const auto& row:accepted) {
    const auto cpu=value(metric(row,"cpu_ms")),wall=value(metric(row,"cpu_wall_ms"));
    if(cpu&&wall&&*wall>0){compute_cpu+=*cpu;matching_cpu_wall+=*wall;++compute_cpu_samples;}
  }
  const auto elapsed=value(field(complete,"measured_wall_ms")),whole_cpu=value(field(complete,"measured_cpu_ms")),cores=value(field(complete,"online_cpus"));
  const auto claimed=value(field(complete,"measured_frames")),requested=value(field(field(suite,"requested"),"frames"));
  const bool coverage=field(complete,"complete").boolean&&claimed&&*claimed==accepted.size()&&accepted.size()>0&&failures==0&&(!requested||*requested==accepted.size());
  auto result=object();result.object["complete"]=boolean(coverage);result.object["runner_claimed_complete"]=field(complete,"complete");
  result.object["measured_frames"]=num(double(accepted.size()));result.object["warmup_frames"]=num(double(warmup));
  result.object["failed_frames"]=num(double(failures));result.object["input_rows"]=num(double(total));result.object["comparison_cohort"]=str("unpaired_live_runs");
  result.object["metrics"]=metrics;result.object["tasks_ms"]=tasks;result.object["runner_metadata"]=run;result.object["saved_frames"]=saved;
  result.object["compute_fps"]=compute.first>0?num(compute.second*1000.0/compute.first):Json{};
  result.object["acquire_interval_fps"]=acquire.first>0?num(acquire.second*1000.0/acquire.first):Json{};
  result.object["load_compute_save_fps"]=total_wall.first>0?num(total_wall.second*1000.0/total_wall.first):Json{};
  result.object["pipeline_fps"]=coverage&&elapsed&&*elapsed>0?num(accepted.size()*1000.0/ *elapsed):Json{};
  result.object["end_to_end_fps"]=result.object["pipeline_fps"];
  result.object["compute_cpu_percent"]=matching_cpu_wall>0?num(compute_cpu*100/matching_cpu_wall):Json{};
  result.object["compute_cpu_samples"]=num(double(compute_cpu_samples));
  result.object["window_cpu_percent"]=coverage&&elapsed&&*elapsed>0&&whole_cpu?num(*whole_cpu*100/ *elapsed):Json{};
  result.object["window_cpu_machine_percent"]=coverage&&elapsed&&*elapsed>0&&whole_cpu&&cores&&*cores>0?num(*whole_cpu*100/ *elapsed/ *cores):Json{};
  result.object["measured_wall_ms"]=coverage?field(complete,"measured_wall_ms"):Json{};
  result.object["measured_process_cpu_ms"]=coverage?field(complete,"measured_cpu_ms"):Json{};
  result.object["rss_p95_kib"]=field(field(metrics,"rss_kib"),"p95");result.object["rss_max_kib"]=field(field(metrics,"rss_kib"),"max");
  result.object["peak_rss_max_kib"]=field(field(metrics,"peak_rss_kib"),"max");result.object["bpu"]=field(complete,"bpu");
  if(!accepted.empty()){result.object["first_capture_frame_id"]=field(accepted.front(),"frame_id");result.object["first_capture_timestamp_ns"]=field(accepted.front(),"timestamp_ns");
    result.object["last_capture_frame_id"]=field(accepted.back(),"frame_id");result.object["last_capture_timestamp_ns"]=field(accepted.back(),"timestamp_ns");}
  return result;
}
std::string csv(const std::string& value){std::string s="\"";for(char c:value){if(c=='\"')s+='\"';s+=c;}return s+'\"';}
std::string display(const Json& j){const auto n=value(j);if(!n)return "—";std::ostringstream out;out<<std::fixed<<std::setprecision(3)<<*n;return out.str();}
void write(const fs::path& path,const std::string& text){std::ofstream out(path,std::ios::binary|std::ios::trunc);out<<text;out.close();if(!out)throw std::runtime_error("Cannot write live summary "+path.u8string());}
void html(const fs::path& root,const Json& summary) {
  auto data=v3::jsonText(summary,0);std::string safe;for(char c:data)if(c=='<')safe+="\\u003c";else safe+=c;
  const auto before=R"HTML(<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>X5 相机实时实验</title><style>body{font:15px/1.65 system-ui;margin:24px;background:#111b29;color:#e0eaf5}a{color:#92dacf}h1{font-size:25px}table{border-collapse:collapse;width:100%}th,td{padding:8px 12px;border-bottom:1px solid #34465b;text-align:right}th:first-child,td:first-child{text-align:left}.note{padding:14px;background:#213347;color:#d7e9ff}pre{white-space:pre-wrap;font-size:12px}details{margin:18px 0}img{max-width:640px;width:100%;display:block}section{margin:20px 0}#saved a{margin-right:10px}</style><h1>X5 开相机实时实验</h1><p class="note">每组独立打开相机、顺序运行，数据为 unpaired_live_runs。图像和采集时间不同，不按帧序号配对，不计算同帧质量差异。pipeline FPS 是包含等待、SDK线程、处理、配置保存和日志的实际测量循环吞吐量；compute FPS 单独显示。</p><p><a href="summary.json">JSON</a> · <a href="summary.csv">CSV</a> · <a href="report_zh.md">中文报告</a> · <a href="run_suite.json">实验配置与命令</a></p><p id="status"></p><div id="table"></div><p class="note">CPU 单核满载=100%，可以超过100%；compute 区间的进程CPU也包含并行运行的相机SDK线程。BPU是设备共享快照，不归属于单个模型。初始化/暖机不进入正式FPS和CPU测量，但进程峰值RSS包含初始化/暖机。原视差内部独立计算耗时未知，acquire_ms不能等同于其计算耗时。阶段可以重叠，不相加当作整帧时间。</p><div id="stages"></div><h2>保存结果：按组查看，各帧独立采集</h2><div id="saved"></div><details><summary>测量定义与实际模型/标定</summary><pre id="metadata"></pre></details><script>const report=)HTML";
  const auto after=R"HTML(;const $=id=>document.getElementById(id),methods=report.method_order;const fmt=v=>v==null?'—':typeof v==='number'?v.toFixed(3):String(v);$('status').textContent='完成状态：'+report.all_selected_methods_complete+'；范围：'+report.comparison_cohort;function table(fields,parent){const t=document.createElement('table'),head=document.createElement('tr');for(const label of ['指标',...methods]){const th=document.createElement('th');th.textContent=label;head.append(th)}t.append(head);for(const [label,fn]of fields){const tr=document.createElement('tr'),td=document.createElement('td');td.textContent=label;tr.append(td);for(const m of methods){const cell=document.createElement('td');cell.textContent=fmt(fn(report.methods[m]));tr.append(cell)}t.append(tr)}parent.append(t)}table([['成功测量帧',m=>m.measured_frames],['实际在线 pipeline FPS',m=>m.pipeline_fps],['算法 compute FPS',m=>m.compute_fps],['采集区间 FPS（不是整链路）',m=>m.acquire_interval_fps],['load/compute/save FPS',m=>m.load_compute_save_fps],['测量窗口进程 CPU %',m=>m.window_cpu_percent],['compute 区间进程 CPU %',m=>m.compute_cpu_percent],['测量窗口 CPU ms',m=>m.measured_process_cpu_ms],['compute区间 CPU ms/帧',m=>m.metrics.cpu_ms.mean],['RSS p95 KiB',m=>m.rss_p95_kib],['进程峰值RSS KiB',m=>m.peak_rss_max_kib],['BPU设备 mean %',m=>m.bpu&&m.bpu.mean_pct],['BPU设备 p95 %',m=>m.bpu&&m.bpu.p95_pct]],$('table'));const stageFields=['acquire_ms','compute_wall_ms','total_wall_ms','original_roi_mask_ms','det_pre_ms','det_infer_ms','det_post_ms','seg_pre_ms','seg_infer_ms','seg_post_ms','stereo_ms','stereo_pre_ms','stereo_backend_ms','stereo_post_ms','graph_ms','save_ms'];table(stageFields.map(key=>[key+'（mean/p50/p95 ms）',m=>{const v=m.metrics[key];return v?[v.mean,v.p50,v.p95].map(fmt).join(' / '):null}]),$('stages'));for(const name of methods){const m=report.methods[name],section=document.createElement('section'),title=document.createElement('h3');title.textContent=name;section.append(title);if(!m.saved_frames.length){const p=document.createElement('p');p.textContent='未保存结果（性能实验默认 save_every_n=0）；不生成质量分数。';section.append(p)}for(const frame of m.saved_frames){const d=document.createElement('details'),s=document.createElement('summary');s.textContent='capture '+frame.frame_id+' / '+frame.timestamp_ns;d.append(s);for(const [label,path]of Object.entries(frame.files)){const a=document.createElement('a');a.href=path;a.textContent=label;d.append(a);if(label==='rgb.png'||label==='disparity.png'){const im=document.createElement('img');im.src=path;im.alt=name+' '+label;d.append(im)}}section.append(d)}$('saved').append(section)}$('metadata').textContent=JSON.stringify({definitions:report.definitions,methods:Object.fromEntries(methods.map(m=>[m,report.methods[m].runner_metadata]))},null,2);</script></html>)HTML";
  write(root/"index.html",before+safe+after);
}
} // namespace

v3::Json report(const std::filesystem::path& output) {
  const auto root=std::filesystem::canonical(output);const auto suite=v3::readJson(root/"run_suite.json");
  auto result=object();result.object["schema_version"]=integer(1);result.object["comparison_cohort"]=str("unpaired_live_runs");
  result.object["paired_frame_count"]=Json{};result.object["quality_comparison"]=str("not_performed_unpaired_live_runs");
  result.object["method_order"]=field(suite,"method_order");result.object["methods"]=object();result.object["run_suite"]=suite;
  bool complete=!field(suite,"method_order").array.empty();
  for(const auto& name:field(suite,"method_order").array){auto method=summarize(root,name.text,suite);complete=complete&&field(method,"complete").boolean;result.object["methods"].object[name.text]=std::move(method);}
  result.object["all_selected_methods_complete"]=boolean(complete);
  result.object["definitions"]=object();auto& definitions=result.object["definitions"];
  definitions.object["pipeline_fps"]=str("successful_measured_frames*1000/measured_wall_ms; real synchronous live camera acquisition+processing+configured_save+logging+result_release; excludes initialization/warmup");
  definitions.object["compute_fps"]=str("valid_compute_samples*1000/sum(compute_wall_ms); excludes acquisition/save; not actual online throughput");
  definitions.object["cpu"]=str("process CPU milliseconds / matching wall milliseconds *100; 100%=one core; includes camera SDK and scheduler threads; never add overlapping phase CPU percentages");
  definitions.object["bpu"]=str("device_shared snapshots after warmup; driver averaging window not reset; mean/p95 are complete runner group measurements, not model attribution");
  definitions.object["rss"]=str("per-frame process RSS KiB; peak RSS includes process initialization and warmup");
  definitions.object["quality"]=str("different live captures across methods; no same-frame pairing, no ground truth, no disparity difference score; saved outputs are grouped previews only");
  definitions.object["legacy_disparity"]=str("original camera mode2 enables SDK Q8 producer, included in whole-process measurement; its isolated internal compute time is unavailable; acquire time includes SDK wait/copy/release");
  write(root/"summary.json",v3::jsonText(result)+"\n");
  std::ostringstream csv_out;csv_out<<"cohort,method,metric,count,missing,mean,p50,p95,p99,min,max\n";
  for(const auto& name:field(result,"method_order").array){const auto& method=field(field(result,"methods"),name.text);
    for(const auto& item:field(method,"metrics").object){csv_out<<"unpaired_live_runs,"<<name.text<<','<<csv(item.first);
      for(const auto* key:{"count","missing","mean","p50","p95","p99","min","max"}){csv_out<<',';const auto& j=field(item.second,key);if(j.kind==Json::Kind::Number)csv_out<<j.text;}csv_out<<'\n';}
    for(const auto* key:{"pipeline_fps","compute_fps","acquire_interval_fps","load_compute_save_fps","window_cpu_percent","compute_cpu_percent","window_cpu_machine_percent","measured_process_cpu_ms"}){
      csv_out<<"unpaired_live_runs,"<<name.text<<','<<key<<",,,";const auto& n=field(method,key);if(n.kind==Json::Kind::Number)csv_out<<n.text;csv_out<<",,,,,\n";}
    for(const auto* key:{"mean_pct","p95_pct","max_pct"}){csv_out<<"unpaired_live_runs,"<<name.text<<",bpu_device_"<<key<<",,,";const auto& n=field(field(method,"bpu"),key);if(n.kind==Json::Kind::Number)csv_out<<n.text;csv_out<<",,,,,\n";}
  }
  write(root/"summary.csv",csv_out.str());
  std::ostringstream markdown;markdown<<"# X5 相机实时实验\n\n范围：`unpaired_live_runs`。每组独立打开相机、顺序运行；不同采集帧不按序号配对，不计算同帧质量差异。\n\n"
    <<"| 方法 | 成功帧 | 实际在线 pipeline FPS | compute FPS | 测量窗口 CPU % | 每帧CPU均值 ms | RSS p95 KiB | BPU设备均值 % | BPU设备p95 % |\n|---|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for(const auto& name:field(result,"method_order").array){const auto& m=field(field(result,"methods"),name.text);markdown<<'|'<<name.text<<'|'<<display(field(m,"measured_frames"))<<'|'<<display(field(m,"pipeline_fps"))<<'|'<<display(field(m,"compute_fps"))<<'|'<<display(field(m,"window_cpu_percent"))<<'|'<<display(field(field(field(m,"metrics"),"cpu_ms"),"mean"))<<'|'<<display(field(m,"rss_p95_kib"))<<'|'<<display(field(field(m,"bpu"),"mean_pct"))<<'|'<<display(field(field(m,"bpu"),"p95_pct"))<<"|\n";}
  markdown<<"\n实际在线FPS包含SDK等待、处理、配置的保存、指标日志和结果释放；初始化/暖机不计入。CPU为整个进程，包含相机SDK和调度线程；单核100%，可以超过100%。阶段可以重叠，不相加为整帧耗时。BPU为设备共享快照，不能归因到某个模型。RSS逐帧采样，进程峰值包含初始化/暖机。原视差内部独立计算时间未知，acquire_ms不能等同于其计算时间。\n\n"
    <<"[交互查看各组指标与保存结果](index.html) · [JSON](summary.json) · [CSV](summary.csv) · [配置与命令](run_suite.json)。\n\nsave_every_n=0只做性能测量，不生成同帧质量分数；保存的PLY/视差/RGB按方法分组查看，各组帧不同。\n";
  write(root/"report_zh.md",markdown.str());html(root,result);return result;
}
} // namespace camera_compare
