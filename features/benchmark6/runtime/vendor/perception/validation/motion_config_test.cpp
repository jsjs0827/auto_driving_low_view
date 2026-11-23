#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include "motion_settings_json.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace {
using namespace v3;
using namespace v3::detail;
int checks=0;
Json settings(Json value){value.object.erase("_help");return value;}
bool samePath(const std::string& value,const Path& expected){
  const auto actual=std::filesystem::u8path(value);
  if(!actual.is_absolute())return false;
  if(actual.lexically_normal()==std::filesystem::absolute(expected).lexically_normal())return true;
  std::error_code error;return std::filesystem::equivalent(actual,expected,error)&&!error;
}
void require(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
template<class F>void rejected(F action,const char* expected){
  try{action();}catch(const std::exception& e){require(std::string(e.what()).find(expected)!=std::string::npos,e.what());return;}
  throw std::runtime_error(std::string("Expected rejection: ")+expected);
}
void file(const Path& path,const std::string& contents){
  std::filesystem::create_directories(path.parent_path());std::ofstream out(path,std::ios::binary);out<<contents;
  if(!out)throw std::runtime_error("Fixture write failed");
}
void save(const Path& path,const Json& value){file(path,jsonText(value)+"\n");}
void environment(const char* key,const Path& value){
#ifdef _WIN32
  _putenv_s(key,value.u8string().c_str());
#else
  setenv(key,value.u8string().c_str(),1);
#endif
}
}
int main(int argc,char** argv){
  Path temp;
  try{
    if(argc!=2)throw std::invalid_argument("motion_config_test PROJECT_ROOT");
    const auto project=std::filesystem::absolute(argv[1]);
    temp=std::filesystem::temp_directory_path()/("motion-config-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    require(std::filesystem::create_directory(temp),"New fixture directory required");
    const auto fixture=std::filesystem::canonical(temp);temp=fixture;
    std::filesystem::create_directories(temp/"scheduling");
    for(const auto* feature:{"ros","offline","stereo","inference","detection","segmentation","pointcloud","fusion","metrics","temporal","recharge","visual_motion","motion_reference","slip_detection"}){
      std::filesystem::create_directories(temp/"features"/feature);
      std::filesystem::copy_file(project/"features"/feature/"config.json",temp/"features"/feature/"config.json");
    }
    std::filesystem::copy_file(project/"features/inference/cpu_profile.json",temp/"features/inference/cpu_profile.json");
    std::filesystem::create_directories(temp/"sdk/config");
    std::filesystem::copy(project/"scheduling/config/sdk_templates",temp/"sdk/config",std::filesystem::copy_options::recursive);
    std::filesystem::copy(project/"scheduling/config/profiles",temp/"sdk/profiles",std::filesystem::copy_options::recursive);
    std::filesystem::create_directories(temp/"config/slip");
    std::filesystem::copy_file(project/"config/slip/temporal.json",temp/"config/slip/temporal.json");
    for(const auto* name:{"slip_camera.json","slip_recording.json","slip_ros.json","camera_benchmark_visualization.json","camera_original_q8.json"})
      std::filesystem::copy_file(project/name,temp/name);
    auto camera=readJson(project/"camera.json");camera.object["sdk_library"]=str("mock/libCamera.so");camera.object["vendor_config"]=str("mock/vendor.cfg");
    save(temp/"camera.json",camera);file(temp/"mock/libCamera.so","inert");file(temp/"mock/vendor.cfg","inert");
    for(const auto* feature:{"detection","segmentation","stereo"})
      file(temp/std::filesystem::u8path(get(readJson(temp/"features"/feature/"config.json"),"model").text),"inert");
    environment("PERCEPTION_PROJECT_ROOT",temp);environment("SDK_ROOT",temp/"sdk");
    file(temp/"capture/timeline.csv","timestamp_ns,left_nv12,q8_u16\n1000000000,left.nv12,disp.u16\n");
    file(temp/"capture/metadata.json","{\"width\":640,\"height\":544,\"fx\":420,\"fy\":420,\"cx\":320,\"cy\":272,\"bxf\":30000}");
    environment("CAPTURE_ROOT",temp/"capture");
    const auto prepared=prepare(temp/"slip_camera.json",temp/"camera-run",false);
    const auto plan=r33::full::parseProductGraph(prepared.graph);
    require(plan.enabled("history")&&plan.enabled("visual_motion")&&plan.enabled("motion_reference")&&plan.enabled("slip_detection"),"Three independent motion stages required");
    require(!plan.enabled("det")&&!plan.enabled("seg")&&!plan.enabled("cloud"),"Default motion graph must remain lightweight");
    require(get(get(prepared.full,"history"),"capacity").text=="5"&&get(get(prepared.full,"history"),"frame_interval").text=="1","Default five-frame window changed");
    require(get(prepared.slip_input,"max_frames").text=="100"&&get(prepared.slip_input,"motion_reference_csv").text.empty(),"Default source/frame limit changed");
    require(get(prepared.parameters,"camera_disparity_mode").text=="2","Original Q8 requires camera mode2");
    require(prepared.argv.front().find("r36_slip_runner")!=std::string::npos,"Native motion must run native slip entry");
    require(!std::filesystem::exists(temp/"camera-run"),"Preparation-only must not write output");
    const auto ros=prepare(temp/"slip_ros.json",temp/"ros-run",false);
    require(ros.argv.size()>2&&ros.argv[1]=="--ros-args","ROS motion must retain the ROS demo entry");
    require(get(ros.parameters,"use_camera_info").boolean&&get(ros.parameters,"wheel_odom_topic").text.empty(),"ROS calibration/source defaults changed");
    const auto recorded=prepare(temp/"slip_recording.json",temp/"record-run",false);
    require(samePath(get(recorded.slip_input,"timeline_csv").text,temp/"capture/timeline.csv"),"Timeline must resolve relative/environment path");
    require(samePath(get(recorded.recording,"directory").text,temp/"capture"),"Recording directory must derive from timeline");
    auto unlimited=readJson(temp/"slip_recording.json");unlimited.object["recording"].object["limit"]=integer(0);save(temp/"unlimited.json",unlimited);
    require(get(prepare(temp/"unlimited.json",temp/"unlimited-run",false).recording,"limit").text=="0","Motion recording zero limit means all captures");
    unlimited.object["recording"].object["repeat"]=integer(2);save(temp/"repeated.json",unlimited);
    rejected([&]{prepare(temp/"repeated.json",temp/"repeated-run",false);},"one original capture epoch");
    auto request=readJson(temp/"slip_recording.json");request.object["input"].object.erase("timeline_csv");save(temp/"missing-timeline.json",request);
    rejected([&]{prepare(temp/"missing-timeline.json",temp/"bad-timeline",false);},"input.timeline_csv");
    auto temporal=readJson(temp/"config/slip/temporal.json");temporal.object["capacity"]=integer(1);save(temp/"config/slip/temporal.json",temporal);
    rejected([&]{prepare(temp/"slip_camera.json",temp/"bad-capacity",false);},"capacity >= 2");
    temporal.object["capacity"]=integer(5);save(temp/"config/slip/temporal.json",temporal);
    auto reference=readJson(temp/"features/motion_reference/config.json");const auto reference_default=reference;
    file(temp/"wheel.csv","timestamp_ns,linear_speed_mps,yaw_rate_radps\n1000000000,0.1,0\n");reference.object["input_csv"]=str("wheel.csv");
    save(temp/"features/motion_reference/config.json",reference);
    rejected([&]{prepare(temp/"slip_camera.json",temp/"bad-clock",false);},"same_clock_confirmed");
    reference.object["same_clock_confirmed"]=boolean(true);reference.object["clock_domain"]=str("camera_monotonic_ns");save(temp/"features/motion_reference/config.json",reference);
    const auto wheel=prepare(temp/"slip_camera.json",temp/"wheel-run",false);
    require(samePath(get(wheel.slip_input,"motion_reference_csv").text,temp/"wheel.csv"),"Reference CSV must resolve against main JSON");
    rejected([&]{prepare(temp/"slip_ros.json",temp/"ros-csv",false);},"ROS motion input does not consume CSV");
    reference.object["odom_topic"]=str("/wheel/odom");save(temp/"features/motion_reference/config.json",reference);
    rejected([&]{prepare(temp/"slip_camera.json",temp/"two-sources",false);},"mutually exclusive");
    reference=reference_default;reference.object["odom_topic"]=str("wheel/odom");save(temp/"features/motion_reference/config.json",reference);
    rejected([&]{prepare(temp/"slip_ros.json",temp/"bad-topic",false);},"absolute ROS topic");
    reference.object["odom_topic"]=str("/wheel/odom");save(temp/"features/motion_reference/config.json",reference);
    rejected([&]{prepare(temp/"slip_ros.json",temp/"bad-odom-clock",false);},"shared clock_domain");
    reference.object["same_clock_confirmed"]=boolean(true);reference.object["clock_domain"]=str("ros_time");save(temp/"features/motion_reference/config.json",reference);
    const auto odom=prepare(temp/"slip_ros.json",temp/"odom-run",false);
    require(get(odom.parameters,"wheel_odom_topic").text=="/wheel/odom"&&get(odom.parameters,"motion_same_clock_confirmed").boolean,"Odometry adapter parameters missing");
    require(get(odom.parameters,"wheel_odom_wait_timeout_ms").text=="50","ROS reference endpoint wait must have a bounded default");
    rejected([&]{prepare(temp/"slip_camera.json",temp/"native-odom",false);},"requires ROS input");
    save(temp/"features/motion_reference/config.json",reference_default);
    auto bad=settings(readJson(temp/"features/visual_motion/config.json"));bad.object["patch_radius"]=integer(0);
    rejected([&]{r33::full::visualSettingsFromJson(bad);},"ranges");
    bad=settings(readJson(temp/"features/slip_detection/config.json"));bad.object["exit_ratio"]=num(.7);
    rejected([&]{r33::full::slipSettingsFromJson(bad);},"slip");
    bad=object();bad.object["unknown"]=integer(1);rejected([&]{r33::full::referenceSettingsFromJson(bad);},"Unknown");
    bad=object();bad.object["wait_timeout_ms"]=integer(2001);rejected([&]{r33::full::referenceSettingsFromJson(bad);},"range");
    auto graph=readJson(temp/"slip_camera.json").object.at("graph");
    for(auto& node:graph.object["nodes"].array)if(get(node,"id").text=="history")node.object["depends_on"]=array();
    rejected([&]{r33::full::parseProductGraph(graph);},"requires enabled ancestor disparity");
    // Old v7 fixtures have no new optional feature folders. Their graph remains valid.
    for(const auto* name:{"visual_motion","motion_reference","slip_detection"})std::filesystem::rename(temp/"features"/name,temp/(std::string("hidden-")+name));
    const auto legacy=prepare(temp/"camera_original_q8.json",temp/"legacy-run",false);
    require(!r33::full::parseProductGraph(legacy.graph).enabled("visual_motion"),"Old graph unexpectedly enabled motion");
    require(get(legacy.full,"visual_motion").object.empty(),"Missing optional settings must use SDK defaults");
    // Cleanup only the exact canonical temporary directory created by this test.
    require(temp==fixture&&temp.filename().u8string().rfind("motion-config-",0)==0&&!std::filesystem::is_symlink(temp),"Unsafe fixture cleanup");
    std::filesystem::remove_all(temp);
    std::cout<<"Motion configuration contracts passed: "<<checks<<" checks\n";return 0;
  }catch(const std::exception& e){std::cerr<<"Motion config test failed: "<<e.what()<<"; fixture="<<temp.u8string()<<'\n';return 1;}
}
