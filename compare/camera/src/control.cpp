#include "camera_compare.hpp"
#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <set>
#include <sys/wait.h>
#include <unistd.h>

namespace camera_compare {
namespace {
namespace fs=std::filesystem;
using namespace v3::detail;
using v3::Json;
struct Suite {
  Json requested,bpu,roi;
  std::vector<std::string> order;
  std::map<std::string,fs::path> configurations;
  fs::path runner;
  int frames=1000,warmup=10,save_every=0;
};
void saveJson(const fs::path& path,const Json& value) {
  // Only the controller's newly created output contains mutable progress files.
  // Config snapshots retain v3::prepare's exclusive-create contract.
  const auto temporary=fs::path(path.u8string()+".tmp");
  std::ofstream stream(temporary,std::ios::binary|std::ios::trunc);
  stream<<v3::jsonText(value)<<'\n';stream.close();
  if(!stream)throw std::runtime_error("Cannot write live experiment progress "+path.u8string());
  fs::rename(temporary,path);
}
fs::path siblingRunner(){char buf[8192];const auto n=readlink("/proc/self/exe",buf,sizeof(buf)-1);
  if(n<0)throw std::runtime_error("Cannot find sibling camera runner");
  buf[n]=0;return fs::path(buf).parent_path()/"r36_compare_runner";}
Suite load(const Options& options) {
  Suite suite;const auto config=fs::canonical(options.configuration);suite.requested=v3::readJson(config);
  keys(suite.requested,{"schema_version","configurations","order","frames","warmup_frames","save_every_n","comparison_roi","bpu","runner"},"camera benchmark");
  if(count(get(suite.requested,"schema_version"),"schema_version",1,1)!=1)throw std::invalid_argument("Camera benchmark schema_version must be 1");
  choice(str(options.method),{"all","detseg_only","original","stereo"},"method");
  const auto& configs=get(suite.requested,"configurations");keys(configs,{"detseg_only","original","stereo"},"configurations");
  const auto& order=get(suite.requested,"order");
  if(order.kind!=Json::Kind::Array||order.array.size()!=3)throw std::invalid_argument("order must contain detseg_only, original and stereo exactly once");
  std::set<std::string> unique;
  for(const auto& item:order.array){const auto method=choice(item,{"detseg_only","original","stereo"},"order method");
    if(!unique.insert(method).second)throw std::invalid_argument("Duplicate live method in order");
    text(get(configs,method),"configurations."+method);
    if(options.method=="all"||options.method==method){suite.order.push_back(method);
      suite.configurations.emplace(method,fs::u8path(v3::resolvePath(get(configs,method),config.parent_path(),"configurations."+method,"file")));}}
  suite.frames=count(get(suite.requested,"frames",integer(1000)),"frames",1);
  suite.warmup=count(get(suite.requested,"warmup_frames",integer(10)),"warmup_frames",0);
  suite.save_every=count(get(suite.requested,"save_every_n",integer(0)),"save_every_n",0);
  suite.roi=get(suite.requested,"comparison_roi");keys(suite.roi,{"x","y","width","height"},"comparison_roi");
  const int x=count(get(suite.roi,"x"),"ROI x",0,640),y=count(get(suite.roi,"y"),"ROI y",0,544),
    w=count(get(suite.roi,"width"),"ROI width",2,640),h=count(get(suite.roi,"height"),"ROI height",2,544);
  if(((x|y|w|h)&1)||x>640-w||y>544-h)throw std::invalid_argument("comparison_roi must be even and inside unchanged 640x544 source");
  suite.roi.object["x"]=integer(x);suite.roi.object["y"]=integer(y);
  suite.roi.object["width"]=integer(w);suite.roi.object["height"]=integer(h);
  suite.bpu=get(suite.requested,"bpu",object());keys(suite.bpu,{"enabled","sample_interval_ms","paths"},"bpu");
  suite.bpu.object["enabled"]=boolean(flag(get(suite.bpu,"enabled",boolean(true)),"bpu.enabled"));
  suite.bpu.object["sample_interval_ms"]=integer(count(get(suite.bpu,"sample_interval_ms",integer(100)),"bpu.sample_interval_ms",10,60000));
  auto paths=get(suite.bpu,"paths",array());if(paths.kind!=Json::Kind::Array||paths.array.size()>16)throw std::invalid_argument("bpu.paths must be an array with at most 16 paths");
  for(const auto& path:paths.array)text(path,"bpu path");
  suite.bpu.object["paths"]=std::move(paths);
  suite.runner=explicitValue(suite.requested,"runner")?fs::u8path(v3::resolvePath(get(suite.requested,"runner"),config.parent_path(),"runner")):siblingRunner();
  if(!options.prepare_only&&(!fs::is_regular_file(suite.runner)||access(suite.runner.c_str(),X_OK)))throw std::invalid_argument("Camera runner is missing or not executable: "+suite.runner.u8string());
  return suite;
}
void validate(const v3::PreparedPipeline& p,const std::string& method,const Suite& suite) {
  if(p.mode!="camera")throw std::invalid_argument(method+" configuration must select input.mode=camera");
  const auto graph=r33::full::parseProductGraph(p.graph);
  const bool det=graph.enabled("det")||graph.enabled("dnn"),seg=graph.enabled("seg")||graph.enabled("dnn");
  if(!det||!seg||!get(get(p.full,"inference"),"enabled").boolean||
     !get(get(p.features,"detection"),"enabled",boolean(true)).boolean||
     !get(get(p.features,"segmentation"),"enabled",boolean(true)).boolean)
    throw std::invalid_argument(method+" requires active detection and segmentation");
  if(graph.enabled("recharge")||graph.enabled("history")||graph.enabled("temporal_difference")||graph.enabled("temporal_match")||graph.enabled("temporal_extension"))
    throw std::invalid_argument("Camera experiment excludes recharge/temporal graph work; configure a separate experiment for those features");
  const bool geometry=get(p.parameters,"mapping_enabled").boolean||get(p.parameters,"disparity_enabled").boolean;
  if(method=="detseg_only") {
    if(geometry||graph.enabled("cloud")||graph.enabled("labels")||graph.enabled("maps"))throw std::invalid_argument("detseg_only configuration must disable all geometry/fusion tasks");
  } else {
    // Disparity-only inference is a complete workload. Cloud/fusion stages are
    // optional and must not force a PCL/mapping dependency into this runner.
    if(!graph.enabled("disparity"))throw std::invalid_argument(method+" requires an enabled disparity task");
    const auto expected=method=="original"?"original_q8":"raft_stereo";
    if(p.method!=expected||get(p.parameters,"disparity_source").text!=expected)throw std::invalid_argument(method+" configuration has the wrong disparity_source");
  }
  const int mode=count(get(p.parameters,"camera_disparity_mode"),"camera mode",1,2);
  if(mode!=(method=="original"?2:1))throw std::invalid_argument("Live camera mode must be original=2, detseg_only/stereo=1 to avoid hidden SDK disparity production");
  if(method=="stereo"&&get(p.stereo,"input_mode").text=="crop") {
    const int w=count(get(p.stereo,"input_width"),"stereo.input_width",2,640),h=count(get(p.stereo,"input_height"),"stereo.input_height",2,544);
    if(w!=640||get(suite.roi,"x").text!="0"||count(get(suite.roi,"y"),"ROI y",0)!=544-h||
       count(get(suite.roi,"width"),"ROI width")!=w||count(get(suite.roi,"height"),"ROI height")!=h)
      throw std::invalid_argument("Suite comparison ROI must equal the native bottom Stereo crop");
  }
}
void option(std::vector<std::string>& args,const std::string& key,const std::string& value){args.push_back("--"+key);args.push_back(value);}
std::vector<std::string> arguments(const Suite& suite,const v3::PreparedPipeline& p,
                                   const std::string& method,const fs::path& output) {
  std::vector<std::string> args{suite.runner.u8string()};
  option(args,"sdk-config",(fs::u8path(p.run_directory)/"effective_full.json").u8string());
  option(args,"camera-sdk",text(get(p.parameters,"camera_sdk"),"camera_sdk"));
  option(args,"camera-config",text(get(p.parameters,"camera_config"),"camera_config"));
  for(const auto& item:std::map<std::string,std::string>{{"camera-mode","camera_disparity_mode"},{"camera-timeout-ms","acquire_timeout_ms"},{"camera-retry-ms","retry_interval_ms"}})
    option(args,item.first,v3::jsonText(get(p.parameters,item.second),0));
  option(args,"method",method);option(args,"output",output.u8string());
  option(args,"frames",std::to_string(suite.frames));option(args,"warmup",std::to_string(suite.warmup));
  option(args,"repeat","1");option(args,"save-every",std::to_string(suite.save_every));
  option(args,"comparison-roi",get(suite.roi,"x").text+","+get(suite.roi,"y").text+","+get(suite.roi,"width").text+","+get(suite.roi,"height").text);
  option(args,"bpu-enabled",get(suite.bpu,"enabled").boolean?"1":"0");
  option(args,"bpu-interval-ms",get(suite.bpu,"sample_interval_ms").text);
  for(const auto& path:get(suite.bpu,"paths").array)option(args,"bpu-path",path.text);
  if(method=="stereo") {
    option(args,"model",text(get(p.stereo,"model"),"stereo.model"));
    option(args,"stereo-input-mode",text(get(p.stereo,"input_mode"),"stereo.input_mode"));
    option(args,"stereo-input-width",get(p.stereo,"input_width").text);
    option(args,"stereo-input-height",get(p.stereo,"input_height").text);
  }
  return args;
}
volatile std::sig_atomic_t child_pid=0,interrupted=0;
void forwardSignal(int value){interrupted=value;if(child_pid>0)kill(-child_pid,value);}
int execute(const std::vector<std::string>& args,const std::map<std::string,std::string>& env,const fs::path& log) {
  const int fd=open(log.c_str(),O_CREAT|O_EXCL|O_WRONLY,0644);if(fd<0)throw std::runtime_error("Cannot exclusively create camera runner log");
  const auto previous_int=std::signal(SIGINT,forwardSignal),previous_term=std::signal(SIGTERM,forwardSignal);
  const auto pid=fork();if(pid<0){close(fd);std::signal(SIGINT,previous_int);std::signal(SIGTERM,previous_term);throw std::runtime_error("fork camera runner failed");}
  if(!pid){setpgid(0,0);dup2(fd,STDOUT_FILENO);dup2(fd,STDERR_FILENO);close(fd);
    std::signal(SIGINT,SIG_DFL);std::signal(SIGTERM,SIG_DFL);
    for(const auto& item:env)if(setenv(item.first.c_str(),item.second.c_str(),1))_exit(126);
    std::vector<char*> argv;for(const auto& value:args)argv.push_back(const_cast<char*>(value.c_str()));argv.push_back(nullptr);
    execv(argv[0],argv.data());_exit(127);
  }
  close(fd);setpgid(pid,pid);child_pid=pid;if(interrupted)kill(-pid,interrupted);
  int status=0;while(waitpid(pid,&status,0)<0){if(errno!=EINTR){child_pid=0;std::signal(SIGINT,previous_int);std::signal(SIGTERM,previous_term);throw std::runtime_error("waitpid camera runner failed");}}
  child_pid=0;std::signal(SIGINT,previous_int);std::signal(SIGTERM,previous_term);
  return interrupted?128+interrupted:WIFEXITED(status)?WEXITSTATUS(status):128+(WIFSIGNALED(status)?WTERMSIG(status):0);
}
} // namespace

int run(const Options& options) {
  const auto suite=load(options);const auto out=fs::absolute(options.output).lexically_normal();
  if(fs::exists(out))throw std::invalid_argument("Output must be new: "+out.u8string());
  std::map<std::string,v3::PreparedPipeline> prepared;
  for(const auto& method:suite.order){auto p=v3::prepare(suite.configurations.at(method),out/"configurations"/method,false);validate(p,method,suite);prepared.emplace(method,std::move(p));}
  fs::create_directories(out.parent_path());if(!fs::create_directory(out))throw std::runtime_error("Cannot exclusively create experiment output");
  auto metadata=object();metadata.object["schema_version"]=integer(1);metadata.object["comparison_cohort"]=str("unpaired_live_runs");
  metadata.object["configuration"]=str(fs::canonical(options.configuration).u8string());metadata.object["requested"]=suite.requested;
  metadata.object["status"]=str(options.prepare_only?"prepared_only":"running");metadata.object["method_order"]=array();metadata.object["methods"]=object();
  metadata.object["intrinsics_source"]=str("camera_sdk_at_runtime_no_user_intrinsics");metadata.object["quality_comparison"]=str("not_performed_unpaired_live_runs");
  for(const auto& method:suite.order){prepared[method]=v3::prepare(suite.configurations.at(method),out/"configurations"/method,true);validate(prepared.at(method),method,suite);
    const auto args=arguments(suite,prepared.at(method),method,out/method);auto argv=array();for(const auto& value:args)argv.array.push_back(str(value));
    metadata.object["method_order"].array.push_back(str(method));metadata.object["methods"].object[method]=object();auto& record=metadata.object["methods"].object[method];
    record.object["configuration"]=str(suite.configurations.at(method).u8string());record.object["arguments"]=argv;
    record.object["effective_full"]=str((out/"configurations"/method/"effective_full.json").u8string());
    record.object["camera_mode"]=get(prepared.at(method).parameters,"camera_disparity_mode");record.object["sdk_root"]=str(prepared.at(method).sdk_root);
  }
  saveJson(out/"run_suite.json",metadata);
  if(options.prepare_only){std::cout<<"Prepared live camera experiments only; no camera/model opened: "<<out<<'\n';return 0;}
  interrupted=0;auto executions=array();
  for(const auto& method:suite.order){
    if(interrupted)break;
    const auto args=arguments(suite,prepared.at(method),method,out/method);
    std::cout<<"Live "<<method<<": warmup="<<suite.warmup<<" measure="<<suite.frames<<"; exclusive camera process\n";
    const auto start=std::chrono::steady_clock::now();const int code=execute(args,prepared.at(method).environment,out/(method+".log"));
    auto execution=object();execution.object["method"]=str(method);execution.object["returncode"]=integer(code);
    execution.object["process_wall_s"]=num(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count());
    execution.object["arguments"]=get(get(metadata.object["methods"],method),"arguments");executions.array.push_back(std::move(execution));saveJson(out/"executions.json",executions);
    if(code){metadata.object["status"]=str(code==130||code==143?"interrupted":"failed");saveJson(out/"run_suite.json",metadata);report(out);return code;}
    const auto complete=v3::readJson(out/method/"complete.json");
    if(!get(complete,"complete").boolean||count(get(complete,"measured_frames"),"complete.measured_frames")!=suite.frames)
      throw std::runtime_error(method+" runner did not confirm the requested measured frame count");
  }
  metadata.object["status"]=str("complete");saveJson(out/"run_suite.json",metadata);
  if(!get(report(out),"all_selected_methods_complete").boolean) {
    metadata.object["status"]=str("failed_incomplete_metrics");saveJson(out/"run_suite.json",metadata);
    throw std::runtime_error("Camera runner completion does not match actual successful metrics coverage");
  }
  auto complete=object();complete.object["complete"]=boolean(true);complete.object["comparison_cohort"]=str("unpaired_live_runs");
  complete.object["methods"]=get(metadata,"method_order");complete.object["frames_per_method"]=integer(suite.frames);saveJson(out/"complete.json",complete);
  return 0;
}
} // namespace camera_compare
