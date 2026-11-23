#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>

namespace {
using namespace v3;
using namespace v3::detail;
int checks=0;
void require(bool condition,const std::string& message) {
  ++checks;if(!condition)throw std::runtime_error(message);
}
bool samePath(const std::string& utf8,const Path& expected) {
  const auto actual=std::filesystem::u8path(utf8);
  if(!actual.is_absolute())return false;
  const auto a=actual.lexically_normal();
  const auto b=std::filesystem::absolute(expected).lexically_normal();
  if(a==b)return true;
  std::error_code ec;
  if(!std::filesystem::equivalent(a,b,ec)||ec)return false;
  return std::filesystem::equivalent(a.parent_path(),b.parent_path());
}
Path checkedFixtureRoot(const Path& root,const Path& parent) {
  const std::string prefix="camera-benchmark-config-",name=root.filename().u8string();
  require(root.is_absolute()&&name.size()>prefix.size()&&name.rfind(prefix,0)==0&&
      std::all_of(name.begin()+prefix.size(),name.end(),[](unsigned char c){return c>='0'&&c<='9';}),"unsafe temporary cleanup name");
  require(!std::filesystem::is_symlink(std::filesystem::symlink_status(root))&&std::filesystem::is_directory(root),"unsafe temporary cleanup root type");
  const auto resolved=std::filesystem::canonical(root),base=std::filesystem::canonical(parent);
  require(std::filesystem::equivalent(root.parent_path(),base)&&std::filesystem::equivalent(resolved.parent_path(),base)&&
      resolved.filename()==root.filename(),"unsafe temporary cleanup path");
  return resolved;
}
void fixtureFile(const Path& path,const char* value) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path,std::ios::binary);file<<value;
  if(!file)throw std::runtime_error("Cannot write fixture "+path.u8string());
}
void env(const char* name,const Path& path) {
#ifdef _WIN32
  _putenv_s(name,path.u8string().c_str());
#else
  setenv(name,path.u8string().c_str(),1);
#endif
}
}

int main(int argc,char** argv) {
  Path temp;
  try {
    if(argc!=2)throw std::invalid_argument("camera_benchmark_config_test PROJECT_ROOT");
    const auto project=std::filesystem::absolute(argv[1]);
    const auto tick=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto temp_parent=std::filesystem::canonical(std::filesystem::temp_directory_path());
    temp=temp_parent/("camera-benchmark-config-"+std::to_string(tick));
    require(std::filesystem::create_directory(temp),"fixture directory must be new");
    temp=checkedFixtureRoot(temp,temp_parent);
    std::filesystem::create_directories(temp/"scheduling");
    // Only configuration files and inert fake model/vendor files are needed.
    // v3::prepare never dlopens these fixtures or runs a model/camera.
    for(const auto* feature:{"ros","offline","stereo","inference","detection","segmentation",
                             "pointcloud","fusion","metrics","temporal","recharge"}) {
      std::filesystem::create_directories(temp/"features"/feature);
      std::filesystem::copy_file(project/"features"/feature/"config.json",temp/"features"/feature/"config.json");
    }
    std::filesystem::copy_file(project/"features/inference/cpu_profile.json",temp/"features/inference/cpu_profile.json");
    std::filesystem::create_directories(temp/"sdk/config");
    std::filesystem::copy(project/"scheduling/config/sdk_templates",temp/"sdk/config",std::filesystem::copy_options::recursive);
    std::filesystem::copy(project/"scheduling/config/profiles",temp/"sdk/profiles",std::filesystem::copy_options::recursive);
    for(const auto* file:{"camera_detseg.json","camera_original_q8.json","camera_stereo.json",
                         "camera_benchmark.json","camera_benchmark_visualization.json"})
      std::filesystem::copy_file(project/file,temp/file);
    auto camera=readJson(project/"camera.json");
    camera.object["sdk_library"]=str("mock_camera/libMockCamera.so");
    camera.object["vendor_config"]=str("mock_camera/vendor.cfg");
    writeJson(temp/"camera.json",camera);
    fixtureFile(temp/"mock_camera/libMockCamera.so","inert camera library fixture; do not load");
    fixtureFile(temp/"mock_camera/vendor.cfg","inert camera configuration fixture");
    for(const auto* feature:{"detection","segmentation","stereo"}) {
      const auto relative=get(readJson(temp/"features"/feature/"config.json"),"model").text;
      require(!relative.empty()&&!std::filesystem::u8path(relative).is_absolute(),"Managed model path must remain relative");
      fixtureFile(temp/std::filesystem::u8path(relative),"inert model fixture; do not infer");
    }
    env("PERCEPTION_PROJECT_ROOT",temp);env("SDK_ROOT",temp/"sdk");
    const auto suite=readJson(temp/"camera_benchmark.json");
    require(get(suite,"schema_version").text=="1","Camera suite schema must be version1");
    require(get(suite,"frames").text=="1000"&&get(suite,"warmup_frames").text=="10","Default camera measurement/warmup frame counts changed");
    require(get(suite,"save_every_n").text=="0","Performance suite must not save frame artifacts");
    const auto& roi=get(suite,"comparison_roi");
    require(get(roi,"x").text=="0"&&get(roi,"y").text=="192"&&get(roi,"width").text=="640"&&get(roi,"height").text=="352","All methods must share the native bottom crop ROI");
    require(get(get(suite,"bpu"),"enabled").boolean&&get(get(suite,"bpu"),"sample_interval_ms").text=="100","BPU sampler must be enabled at100ms");
    const auto& order=get(suite,"order").array;
    require(order.size()==3&&order[0].text=="detseg_only"&&order[1].text=="original"&&order[2].text=="stereo","Default method order changed");
    const auto& configs=get(suite,"configurations");
    require(configs.object.size()==3&&get(configs,"detseg_only").text=="camera_detseg.json"&&
      get(configs,"original").text=="camera_original_q8.json"&&get(configs,"stereo").text=="camera_stereo.json","Suite does not reference the three main camera JSON files");
    auto original_request=readJson(temp/"camera_original_q8.json");
    auto stereo_request=readJson(temp/"camera_stereo.json");
    original_request.object["input"].object["disparity_source"]=str("raft_stereo");
    require(jsonText(original_request)==jsonText(stereo_request),"Original and Stereo main configs must differ only in disparity_source");
    const auto visual=readJson(temp/"camera_benchmark_visualization.json");
    require(get(visual,"save_root").text.empty()&&get(visual,"save_every_n").text=="0"&&!get(visual,"save_ply").boolean,"Benchmark visualization must disable disk/PLY output");
    for(const auto* field:{"publish_clouds","publish_maps","publish_mask","publish_disparity","publish_input"})
      require(!get(visual,field).boolean,std::string("Performance output should be disabled: ")+field);
    require(get(visual,"render_maps").boolean,"Keep render_maps for comparison with v6 perf workload");

    const auto stereo_file=temp/std::filesystem::u8path(get(readJson(temp/"features/stereo/config.json"),"model").text);
    const auto hidden=temp/"stereo-model.hidden";std::filesystem::rename(stereo_file,hidden);
    for(const auto* method:{"detseg_only","original"}) {
      const auto config=temp/std::filesystem::u8path(get(configs,method).text);
      const auto prepared=v3::prepare(config,temp/(std::string("run-")+method));
      const auto plan=r33::full::parseProductGraph(prepared.graph);
      require(prepared.mode=="camera","Experiment must use direct camera input");
      require(get(prepared.parameters,"camera_disparity_mode").text==(std::string(method)=="original"?"2":"1"),"auto camera mode must be detseg1/original2");
      require(!has(prepared.parameters,"stereo_model")&&prepared.stereo.object.empty(),"Original/detseg experiments must not load the missing Stereo model");
      require(plan.enabled("det")&&plan.enabled("seg")&&!plan.enabled("publish")&&!plan.enabled("record"),"Algorithm graph/output selection changed");
      const bool geometry=std::string(method)=="original";
      require(plan.enabled("disparity")==geometry&&plan.enabled("cloud")==geometry&&
              get(prepared.parameters,"mapping_enabled").boolean==geometry,"detseg must disable geometry; original must keep it");
      require(get(prepared.parameters,"save_every_n").text=="0"&&!get(prepared.parameters,"publish_enabled").boolean,"Generated parameters must disable publish/save");
      require(!get(get(prepared.full,"history"),"difference").boolean&&
              !get(get(prepared.full,"history"),"match").boolean,"History analysis must remain disabled");
      require(get(get(prepared.full,"mapping"),"render_maps").boolean,"Map rendering setting lost during prepare");
      if(!geometry)require(get(prepared.graph,"nodes").array.size()==2,"detseg graph must contain only the two semantic lanes");
    }
    std::filesystem::rename(hidden,stereo_file);
    const auto prepared=v3::prepare(temp/"camera_stereo.json",temp/"run-stereo");
    const auto plan=r33::full::parseProductGraph(prepared.graph);
    require(prepared.mode=="camera"&&prepared.method=="raft_stereo","Stereo experiment source changed");
    require(get(prepared.parameters,"camera_disparity_mode").text=="1","New Stereo must not request camera old Q8 generation");
    require(get(prepared.stereo,"input_mode").text=="crop"&&get(prepared.stereo,"input_width").text=="640"&&get(prepared.stereo,"input_height").text=="352","Stereo must use native640x352 crop");
    require(samePath(get(prepared.stereo,"model").text,stereo_file),"Model resolution is not relative to the main configuration");
    for(const auto* stage:{"det","seg","disparity","cloud","labels","transform","ground","maps"})
      require(plan.enabled(stage),std::string("Fusion benchmark missing stage: ")+stage);
    for(const auto* stage:{"calibration","recharge","history","temporal_difference","temporal_match","temporal_extension","publish","record"})
      require(!plan.enabled(stage),std::string("Unexpected extra benchmark stage: ")+stage);
    require(get(prepared.graph,"worker_threads").text=="3","Product graph must retain three workers");
    // Minimal det+seg+disparity experiments must work with an inference-only
    // SDK. Preparing these graphs must not enable any mapping/PCL stages.
    for(const auto* method:{"original_q8","stereo"}) {
      auto request=v3::normalizePipeline(readJson(temp/(std::string("camera_")+method+".json")));
      for(auto& node:request.object["graph"].object["nodes"].array) {
        const auto id=get(node,"id").text;
        if(id!="det"&&id!="seg"&&id!="stereo")node.object["enabled"]=boolean(false);
      }
      const auto config=temp/(std::string("inference-")+method+".json");
      writeJson(config,request);
      const auto minimal=v3::prepare(config,temp/(std::string("run-inference-")+method));
      const auto selected=r33::full::parseProductGraph(minimal.graph);
      for(const auto* stage:{"det","seg","disparity"})
        require(selected.enabled(stage),std::string("Inference-only graph lost stage: ")+stage);
      for(const auto* stage:{"cloud","calibration","labels","transform","ground","maps"})
        require(!selected.enabled(stage),std::string("Inference-only graph enabled mapping stage: ")+stage);
      require(!get(minimal.parameters,"mapping_enabled").boolean&&
              !get(get(minimal.full,"mapping"),"enabled").boolean,
              "Inference-only camera graph requires a mapping SDK");
      require(get(minimal.parameters,"disparity_enabled").boolean,
              "Inference-only camera graph lost its disparity output");
      require(get(minimal.parameters,"camera_disparity_mode").text==
              (std::string(method)=="original_q8"?"2":"1"),
              "Inference-only graph changed the required camera SDK disparity mode");
    }
    const auto cleanup_root=checkedFixtureRoot(temp,temp_parent);
    std::filesystem::remove_all(cleanup_root);temp.clear();
    std::cout<<"camera benchmark config contract: "<<checks<<" checks passed; no camera/model opened\n";
    return 0;
  }catch(const std::exception& error) {
    if(!temp.empty())std::cerr<<"Retained failing fixture: "<<temp<<'\n';
    std::cerr<<"camera benchmark configuration failed: "<<error.what()<<'\n';return 1;
  }
}
