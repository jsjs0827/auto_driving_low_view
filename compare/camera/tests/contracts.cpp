#include "camera_compare.hpp"
#include "native_config_detail.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <sys/wait.h>
#include <unistd.h>

namespace {
namespace fs=std::filesystem;
using v3::Json;
using namespace v3::detail;
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
template<class F>void rejects(F fn,const char* message){bool rejected=false;try{fn();}catch(const std::exception&){rejected=true;}check(rejected,message);}
double numericStat(const Json& j){check(j.kind==Json::Kind::Number,"Expected numeric statistic");return std::stod(j.text);}
void near(double a,double b,const char* why){check(std::abs(a-b)<1e-8,why);}
void file(const fs::path& path,const std::string& value){fs::create_directories(path.parent_path());std::ofstream f(path,std::ios::binary);f<<value;if(!f)throw std::runtime_error("Fixture write failed");}
int fakeRunner(int argc,char** argv){
  std::map<std::string,std::string> options;for(int i=1;i+1<argc;i+=2)options[argv[i]]=argv[i+1];
  for(const auto* forbidden:{"--fx","--fy","--cx","--cy","--bxf","--width","--height","--manifest"})
    check(!options.count(forbidden),"Camera controller passed manual intrinsics or recording input");
  const auto method=options.at("--method");check(options.at("--camera-mode")== (method=="original"?"2":"1"),"Wrong camera SDK mode");
  check(options.at("--comparison-roi")=="0,192,640,352"&&options.at("--repeat")=="1","ROI/repeat dispatch mismatch");
  check(options.count("--model")==std::size_t(method=="stereo"),"Baseline/original must not receive a Stereo model");
  const auto out=fs::path(options.at("--output"));fs::create_directory(out);
  if(std::getenv("CAMERA_COMPARE_TEST_SLEEP")){file(out/"ready","ready");sleep(10);return 1;}
  file(out/"run.json","{\"source_mode\":\"camera\",\"model_metadata\":\"synthetic_contract_only\"}\n");
  auto frame=[&](int id,bool warm,double wall,double cpu){auto r=object();
    r.object["source_mode"]=str("camera");r.object["input_mode"]=str("camera");r.object["comparison_cohort"]=str("unpaired_live_runs");
    r.object["method"]=str(method);r.object["frame_id"]=integer(id);r.object["timestamp_ns"]=integer(id*100);
    r.object["warmup"]=boolean(warm);r.object["compute_wall_ms"]=num(wall);r.object["cpu_ms"]=num(cpu);r.object["cpu_wall_ms"]=num(wall);
    r.object["total_wall_ms"]=num(wall+5);r.object["acquire_ms"]=num(5);r.object["load_ms"]=num(5);r.object["acquire_cpu_ms"]=num(1);
    r.object["rss_kib"]=integer(id==1?512:1536);r.object["peak_rss_kib"]=integer(2048);r.object["output_dir"]=Json{};
    r.object["inference_ms"]=object();r.object["inference_ms"].object["det_infer"]=num(3);r.object["task_trace"]=object();r.object["task_trace"].object["tasks"]=array();return r;};
  file(out/"frames.jsonl",v3::jsonText(frame(9,true,1000,1000),0)+"\n"+v3::jsonText(frame(1,false,10,10),0)+"\n"+v3::jsonText(frame(2,false,30,90),0)+"\n");
  auto complete=object();complete.object["complete"]=boolean(true);complete.object["measured_frames"]=integer(2);complete.object["warmup_frames"]=integer(1);
  complete.object["measured_wall_ms"]=num(100);complete.object["measured_cpu_ms"]=num(200);complete.object["online_cpus"]=integer(8);
  complete.object["bpu"]=object();complete.object["bpu"].object["scope"]=str("device_shared");complete.object["bpu"].object["mean_pct"]=num(0);
  complete.object["bpu"].object["p95_pct"]=Json{};v3::writeJson(out/"complete.json",complete);return 0;
}
struct Fixture{fs::path root,suite;};
Fixture fixture(const fs::path& project,const fs::path& output,const fs::path& runner){
  Fixture f{output/"fixture",output/"fixture/suite.json"};fs::create_directories(f.root);
  fs::copy(project/"features",f.root/"features",fs::copy_options::recursive);
  fs::copy_file(project/"visualization.json",f.root/"visualization.json");
  fs::create_directories(f.root/"sdk/config");fs::copy(project/"scheduling/config/sdk_templates",f.root/"sdk/config",fs::copy_options::recursive);
  fs::copy(project/"scheduling/config/profiles",f.root/"sdk/profiles",fs::copy_options::recursive);
  setenv("PERCEPTION_PROJECT_ROOT",f.root.c_str(),1);setenv("SDK_ROOT",(f.root/"sdk").c_str(),1);
  file(f.root/"camera/libMoGeneralSDK.so","configuration fixture only");file(f.root/"camera/vendor.cfg","configuration fixture only");
  auto camera=object();camera.object["sdk_library"]=str((f.root/"camera/libMoGeneralSDK.so").string());camera.object["vendor_config"]=str((f.root/"camera/vendor.cfg").string());camera.object["disparity_mode"]=str("auto");v3::writeJson(f.root/"camera.json",camera);
  for(const auto* model:{"mmyolo0828.bin","pidnet_0825_3classes.bin","r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin"})file(f.root/"models/x5"/model,"configuration fixture only, never infer");
  auto base=v3::normalizePipeline(v3::readJson(project/"pipeline.json"));base.object["input"].object["mode"]=str("camera");base.object["sdk"].object["root"]=str((f.root/"sdk").string());
  for(const auto* method:{"detseg_only","original","stereo"}){
    auto main=base;main.object["input"].object["disparity_source"]=str(std::string(method)=="stereo"?"raft_stereo":"original_q8");
    for(auto& node:main.object["graph"].object["nodes"].array){const auto id=get(node,"id").text;
      if(std::string(method)=="detseg_only"&&id!="det"&&id!="seg")node.object["enabled"]=boolean(false);
      if(id=="publish"||id=="record"||id=="recharge"||id=="history"||id=="difference"||id=="match"||id=="extension")node.object["enabled"]=boolean(false);}
    v3::writeJson(f.root/(std::string(method)+".json"),main);
  }
  auto suite=object();suite.object["schema_version"]=integer(1);suite.object["configurations"]=object();suite.object["order"]=array();
  for(const auto* method:{"detseg_only","original","stereo"}){suite.object["configurations"].object[method]=str(std::string(method)+".json");suite.object["order"].array.push_back(str(method));}
  suite.object["frames"]=integer(2);suite.object["warmup_frames"]=integer(1);suite.object["save_every_n"]=integer(0);
  suite.object["comparison_roi"]=object();for(const auto& item:std::map<std::string,int>{{"x",0},{"y",192},{"width",640},{"height",352}})suite.object["comparison_roi"].object[item.first]=integer(item.second);
  suite.object["runner"]=str(runner.string());suite.object["bpu"]=object();suite.object["bpu"].object["enabled"]=boolean(true);suite.object["bpu"].object["sample_interval_ms"]=integer(100);suite.object["bpu"].object["paths"]=array();
  v3::writeJson(f.suite,suite);return f;
}
void runTests(const fs::path& project,const fs::path& output,const fs::path& runner){
  const auto f=fixture(project,output,runner);const auto model=f.root/"models/x5/r34_crop_stage2_i12_cheb5_max1_tail_int16_352x640_20261012.bin";
  fs::rename(model,f.root/"stereo.hidden");
  for(const auto* method:{"detseg_only","original"}){camera_compare::Options options{f.suite,output/(std::string("prepared_")+method),method,true};
    check(camera_compare::run(options)==0,"Original/baseline prepare unexpectedly required Stereo model");
    check(fs::is_regular_file(options.output/"configurations"/method/"effective_full.json"),"Prepared config snapshot missing");
    check(!fs::exists(options.output/method)&&!fs::exists(options.output/"summary.json"),"Prepare-only opened runner or invented metrics");}
  fs::rename(f.root/"stereo.hidden",model);
  camera_compare::Options options{f.suite,output/"all","all",false};check(camera_compare::run(options)==0,"Three live method dispatch failed");
  const auto summary=v3::readJson(options.output/"summary.json");
  check(get(summary,"comparison_cohort").text=="unpaired_live_runs"&&get(summary,"paired_frame_count").kind==Json::Kind::Null,"Live sequence IDs must not become paired-frame evidence");
  check(get(summary,"quality_comparison").text=="not_performed_unpaired_live_runs"&&!fs::exists(options.output/"disparity_comparison.json"),"Live camera suite generated invalid same-frame quality comparison");
  check(get(summary,"all_selected_methods_complete").boolean,"Completed live methods not recognized");
  for(const auto* name:{"detseg_only","original","stereo"}){const auto& m=get(get(summary,"methods"),name);
    near(numericStat(get(m,"measured_frames")),2,"Warmup leaked into live measured frames");near(numericStat(get(m,"pipeline_fps")),20,"Pipeline FPS must use real measured loop wall");
    near(numericStat(get(m,"compute_fps")),50,"Compute FPS must use accumulated intervals");near(numericStat(get(m,"compute_cpu_percent")),250,"Compute CPU requires weighted ratio, not frame percentage mean");
    near(numericStat(get(m,"window_cpu_percent")),200,"Whole-process online CPU must use complete measured window");near(numericStat(get(m,"acquire_interval_fps")),200,"Acquire-only FPS scope wrong");
    near(numericStat(get(get(m,"bpu"),"mean_pct")),0,"Real BPU zero must be preserved");check(get(get(m,"bpu"),"p95_pct").kind==Json::Kind::Null,"Missing BPU percentile invented");
    check(get(m,"saved_frames").array.empty(),"save0 generated saved frame previews");}
  const auto executions=v3::readJson(options.output/"executions.json");check(executions.array.size()==3,"Not all three methods executed");
  for(const auto& item:executions.array){const auto& args=get(item,"arguments").array;std::set<std::string> tokens;for(const auto& arg:args)tokens.insert(arg.text);
    check(tokens.count("--camera-sdk")&&tokens.count("--camera-config")&&tokens.count("--frames")&&tokens.count("--comparison-roi"),"Camera runner command protocol incomplete");
    check(!tokens.count("--fx")&&!tokens.count("--width")&&!tokens.count("--manifest"),"Controller must use SDK intrinsics and live source");}
  // The same controller also supports inference-only SDKs with no PCL/mapping.
  // Keep the full mapping fixtures above as an independent regression case.
  auto inference_suite=v3::readJson(f.suite);
  for(const auto* method:{"original","stereo"}) {
    auto pipeline=v3::readJson(f.root/(std::string(method)+".json"));
    auto& nodes=pipeline.object["graph"].object["nodes"].array;
    for(auto& node:nodes) {
      const auto id=get(node,"id").text;
      if(id!="det"&&id!="seg"&&id!="stereo")node.object["enabled"]=boolean(false);
    }
    const auto filename=std::string(method)+"_inference.json";
    v3::writeJson(f.root/filename,pipeline);
    inference_suite.object["configurations"].object[method]=str(filename);
  }
  v3::writeJson(f.root/"inference_suite.json",inference_suite);
  options={f.root/"inference_suite.json",output/"inference_only","all",false};
  check(camera_compare::run(options)==0,"Disparity-only camera workloads unexpectedly required cloud/mapping");
  for(const auto* method:{"original","stereo"}) {
    const auto full=v3::readJson(options.output/"configurations"/method/"effective_full.json");
    check(!get(get(full,"mapping"),"enabled").boolean,"Inference-only configuration enabled mapping");
  }
  auto no_disparity=v3::readJson(f.root/"original_inference.json");
  for(auto& node:no_disparity.object["graph"].object["nodes"].array)
    if(get(node,"id").text=="stereo")node.object["enabled"]=boolean(false);
  v3::writeJson(f.root/"original_no_disparity.json",no_disparity);
  inference_suite.object["configurations"].object["original"]=str("original_no_disparity.json");
  v3::writeJson(f.root/"no_disparity_suite.json",inference_suite);
  camera_compare::Options missing_disparity{f.root/"no_disparity_suite.json",output/"missing_disparity","original",true};
  rejects([&]{camera_compare::run(missing_disparity);},"Original workload accepted a graph without disparity");
  check(!fs::exists(missing_disparity.output),"Invalid disparity graph created partial experiment output");
  auto bad=v3::readJson(f.suite);bad.object["configurations"].object["detseg_only"]=str("original.json");v3::writeJson(f.root/"bad.json",bad);
  camera_compare::Options invalid{f.root/"bad.json",output/"bad","detseg_only",true};
  rejects([&]{camera_compare::run(invalid);},"Declared baseline accepted geometry configuration");check(!fs::exists(invalid.output),"Bad graph created partial experiment output before validation");
  bad=v3::readJson(f.suite);bad.object["comparison_roi"].object["y"]=integer(190);v3::writeJson(f.root/"bad_crop.json",bad);
  invalid={f.root/"bad_crop.json",output/"bad_crop","all",true};rejects([&]{camera_compare::run(invalid);},"Suite ROI differs from Stereo crop but was accepted");
  const auto interrupted_output=output/"interrupted";setenv("CAMERA_COMPARE_TEST_SLEEP","1",1);
  const auto watcher=fork();if(watcher<0)throw std::runtime_error("signal watcher fork failed");
  if(!watcher){const auto parent=getppid();for(int i=0;i<1000;++i){if(fs::exists(interrupted_output/"detseg_only/ready")){kill(parent,SIGINT);_exit(0);}usleep(10000);}_exit(1);}
  options={f.suite,interrupted_output,"all",false};const auto code=camera_compare::run(options);unsetenv("CAMERA_COMPARE_TEST_SLEEP");int status=0;waitpid(watcher,&status,0);
  check(code==130&&WIFEXITED(status)&&WEXITSTATUS(status)==0,"SIGINT was not propagated to active runner");
  check(!fs::exists(interrupted_output/"complete.json")&&!fs::exists(interrupted_output/"original"),"Interrupted suite falsely completed or opened next camera process");
}
} // namespace
int main(int argc,char** argv){try{
  if(argc>1&&std::string(argv[1])=="--sdk-config")return fakeRunner(argc,argv);
  if(argc!=3)throw std::invalid_argument("Provide product source and test output parent");
  const auto output=fs::absolute(argv[2])/("camera-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));fs::create_directories(output);
  runTests(fs::canonical(argv[1]),output,fs::canonical(argv[0]));
  std::cout<<"PASS: live method/camera-mode/ROI dispatch, inference-only disparity without mapping, prepare-only no models for original/baseline, weighted CPU/FPS, unpaired report, no invented save0 quality, graph rejection and SIGINT forwarding.\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
