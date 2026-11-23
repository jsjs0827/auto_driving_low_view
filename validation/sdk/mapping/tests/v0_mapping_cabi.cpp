// Actual shared-library C ABI smoke: synthetic NV12 + Q8, no model/camera.
// This is a geometry-only integration test, not labeled accuracy or a speed run.
#include "perception_full_sdk.h"
#include "json.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#if defined(__linux__)
#include <dlfcn.h>
#endif
namespace {
namespace fs=std::filesystem;
using namespace r33::json;
unsigned checks=0;
void require(bool ok,const std::string& reason){++checks;if(!ok)throw std::runtime_error(reason);}
void checked(std::uint32_t code,const psdk_status& status){require(code==0,"C ABI error="+std::to_string(code)+" feature="+status.feature+" detail="+status.detail);}
void text(const fs::path& file,const std::string& value){std::ofstream out(file);out<<value;out.close();require(bool(out),"Cannot write test config "+file.string());}
struct Cleanup{fs::path path;~Cleanup(){std::error_code ignored;fs::remove_all(path,ignored);}};
struct Publication{unsigned calls=0;bool valid=true;};
std::uint32_t publish(void* userdata,const psdk_full_result_view* view,psdk_status*){
    auto& observation=*static_cast<Publication*>(userdata);++observation.calls;
    observation.valid=observation.valid&&view&&view->frame_id==12345&&view->map_count==3&&!view->has_inference;
    return 0;
}
std::uint64_t observed(const psdk_topview& map){
    std::uint64_t count=0;
    for(std::uint64_t i=0;i<map.cell_count;++i){const auto& c=map.cells[i];
        if(std::isfinite(c.min_h_m)&&std::isfinite(c.max_h_m)&&std::isfinite(c.avg_h_m)&&c.min_h_m<=c.max_h_m)++count;
    }return count;
}
const char* fullJson=R"JSON({
"schema_version":1,"mode":"map","input":{"disparity_source":"original_q8"},
"inference":{"enabled":false,"backend":"bpu","original_profile":"intentionally_absent.cfg","dag":"intentionally_absent.json"},
"mapping":{"enabled":true,"original_config":"start.json","extrinsics":"extrinsics.json","sample_step":1,"retain_clouds":true,"render_maps":false},
"recharge":{"enabled":false},"output":{"every_n":0,"save_overlay":false,"save_inputs":false},
"history":{"capacity":1,"difference":false,"match":false},
"graph":{"worker_threads":2,"nodes":[
 {"id":"disparity","feature":"disparity"},
 {"id":"cloud","feature":"cloud","depends_on":["disparity"]},
 {"id":"transform","feature":"transform","depends_on":["cloud"]},
 {"id":"ground","feature":"ground","depends_on":["transform"]},
 {"id":"maps","feature":"maps","depends_on":["ground"]},
 {"id":"publish","feature":"publish","depends_on":["maps"]}]}
})JSON";
const char* startJson=R"JSON({
"auto_cali":false,"obs_height_thr":0.01,
"map":{"MapWidth":1.6,"MapHeight":2.0,"MapResolution":0.02},
"interestFov":{"box_min_x":-3,"box_max_x":3,"box_min_y":-0.4,"box_max_y":3,"box_min_z":0.1,"box_max_z":3},
"label_fusion":{"enabled":true,"fuse_into_grid":false,"passable_classes":"grass"},
"dnn":{"enabled":false}
})JSON";
const char* extrinsicsJson=R"JSON({"extrinsics":{"x":0.45,"y":0,"z":0.2,"roll":-90,"pitch":0,"yaw":-90}})JSON";
}
int main(){
    psdk_full_engine* engine=nullptr;psdk_full_result* result=nullptr;psdk_status status{};
    try{
        require((psdk_full_capabilities()&PSDK_CAP_MAPPING)!=0,"Actual shared SDK was built without mapping");
        std::string loaded_sdk="linked perception_sdk";
#if defined(__linux__)
        Dl_info loaded{};
        require(dladdr(reinterpret_cast<const void*>(&psdk_full_process),&loaded)!=0&&loaded.dli_fname,"Cannot identify loaded C ABI shared library");
        loaded_sdk=loaded.dli_fname;
        require(loaded_sdk.find("libperception_sdk")!=std::string::npos,"C ABI smoke did not resolve through libperception_sdk");
#endif
        const auto directory=fs::temp_directory_path()/("v0_map_cabi_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        require(fs::create_directory(directory),"Cannot reserve new smoke directory");Cleanup cleanup{directory};
        text(directory/"full.json",fullJson);text(directory/"start.json",startJson);text(directory/"extrinsics.json",extrinsicsJson);
        require(!fs::exists(directory/"intentionally_absent.cfg")&&!fs::exists(directory/"intentionally_absent.json"),"Disabled-model proof paths unexpectedly exist");
        Publication publications;psdk_full_hooks hooks{};hooks.struct_size=sizeof(hooks);hooks.abi_version=PSDK_FULL_ABI_VERSION;
        hooks.userdata=&publications;hooks.publication=publish;
        const auto config_path=(directory/"full.json").string();psdk_full_config config{};
        config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;config.json_file=config_path.c_str();config.hooks=&hooks;
        checked(psdk_full_create(&config,&engine,&status),status);require(engine!=nullptr,"No C ABI engine");
        const unsigned width=160,height=120;std::vector<std::uint8_t> left(width*height*3/2,128);
        std::vector<std::uint16_t> q8(width*height,0);
        // Camera Y=.2m ground plane, with Q8 rounding retained. Intrinsics bxf
        // is10000 px*mm =>10 px*m; this is synthetic geometry, no camera claim.
        for(unsigned y=69;y<height;++y)for(unsigned x=0;x<width;++x){
            const float depth=.2f*120/(float(y)-60.f);
            q8[y*width+x]=std::uint16_t(std::round(10/depth*256));
        }
        psdk_disparity_q8 disparity{};disparity.struct_size=sizeof(disparity);disparity.abi_version=1;
        disparity.width=width;disparity.height=height;disparity.stride_bytes=width*2;
        disparity.data=q8.data();disparity.bytes=q8.size()*2;disparity.frame_id=12345;disparity.timestamp_ns=1000000000;
        psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
        auto& image=frame.left;image.struct_size=sizeof(image);image.abi_version=PSDK_ABI_VERSION;image.format=PSDK_FORMAT_NV12;
        image.width=width;image.height=height;image.stride=width;image.data=left.data();image.bytes=left.size();image.frame_id=12345;image.timestamp_ns=1000000000;
        frame.disparity=&disparity;frame.intrinsics={width,height,120,120,80,60,10000};
        checked(psdk_full_process(engine,&frame,&result,&status),status);require(result!=nullptr,"No C ABI result");
        psdk_full_result_view view{};view.struct_size=sizeof(view);view.abi_version=PSDK_FULL_ABI_VERSION;
        checked(psdk_full_result_get_view(result,&view,&status),status);
        require(view.frame_id==12345&&view.timestamp_ns==1000000000&&view.run_id==1,"C ABI frame identity changed");
        require(!view.has_inference&&view.inference.detection_count==0&&view.inference.segmentation_bytes==0,"Disabled detector/segmenter produced inference");
        require(publications.calls==1&&publications.valid,"Publication did not see completed same-frame maps");
        require(view.disparity.bytes==q8.size()*2&&view.disparity.data&&std::memcmp(view.disparity.data,q8.data(),q8.size()*2)==0,"Original supplied Q8 was transformed or lost");
        require(view.map_count==3&&view.maps&&view.grid_count==8000,"Actual shared SDK did not expose all3 real maps");
        std::uint64_t pcl_observed=0,fusion_observed=0;
        for(unsigned i=0;i<3;++i){const auto& map=view.maps[i];const char* expected[]={"pcl","ai","fusion"};
            require(std::strcmp(map.name,expected[i])==0,"C ABI map order/name contract changed");
            require(map.rows==100&&map.cols==80&&map.resolution_mm==20&&map.cell_count==8000&&map.cells,"C ABI map geometry/storage mismatch");
            require(map.rgb_bytes==0,"render_maps=false unexpectedly rendered BEV");
            if(i==0)pcl_observed=observed(map);
            if(i==2)fusion_observed=observed(map);
            if(i==1)for(std::uint64_t k=0;k<map.cell_count;++k)require(map.cells[k].type==0,"Geometry-only test invented AI labels");
        }
        require(pcl_observed>0&&fusion_observed>0,"Map structures exist but lack real geometry observations");
        std::uint64_t ground_points=0;
        for(unsigned i=0;i<view.cloud_count;++i){const auto& cloud=view.clouds[i];
            if(std::strcmp(cloud.name,"ground")==0)ground_points=cloud.point_count;
            if(std::strcmp(cloud.name,"semantic")==0)require(cloud.point_count==0,"No-inference smoke generated semantic points");
        }
        require(ground_points>3000,"Actual shared cloud/adapt/ground chain did not produce ground");
        require(!view.calibration.active&&std::abs(view.calibration.z_m-.2)<.001&&std::abs(view.calibration.roll_deg+90)<.01&&std::abs(view.calibration.pitch_deg)<.01,"Applied ordinary adaptive pose not exposed through C ABI");
        require(view.effective_config_json&&view.task_trace_json,"C ABI omitted effective numeric config/trace");
        const auto effective_text=std::string(view.effective_config_json),trace_text=std::string(view.task_trace_json);
        const auto effective=JsonParser(effective_text,"C ABI effective").parse();
        const auto& sdk=required(effective,"sdk");const auto& mapping=required(effective,"mapping");
        require(!boolean(required(sdk,"inference_enabled"))&&boolean(required(sdk,"mapping_enabled")),"Effective config does not describe maps-only execution");
        require(string(required(sdk,"disparity_source"))=="original_q8","Smoke accidentally selected a Stereo model provider");
        require(std::abs(number(required(mapping,"effective_obs_height_thr"))-.01)<1e-8,"Shared SDK lost1104 obs_height_thr");
        const auto& pw=required(mapping,"effective_patchwork");
        require(boolean(required(pw,"enabled"))&&std::abs(number(required(pw,"sensor_height"))-.2)<1e-7&&std::abs(number(required(pw,"th_dist"))-.01)<1e-8,"Shared SDK default Patchwork/e.z/1cm contract missing");
        require(std::abs(number(required(required(mapping,"effective_fov"),"box_min_y"))+.4)<1e-7,"Shared SDK ignored1104 interestFov");
        require(string(required(mapping,"semantic_grid_policy"))=="v0_unconditional_produced_cloud","Shared SDK is still using the previous semantic handoff policy");
        require(number(required(required(mapping,"ground_adaptation"),"accepted_frames"))==1,"Ordinary adapt did not execute through SO without calibration node");
        const auto trace=JsonParser(trace_text,"C ABI task trace").parse();std::set<std::string> tasks;
        for(const auto& task:required(trace,"tasks").array){tasks.insert(string(required(task,"task")));require(string(required(task,"state"))=="succeeded","C ABI graph task did not succeed");}
        require(tasks==std::set<std::string>({"disparity","cloud","transform","ground","maps","publish"}),"Maps-only graph executed unexpected model/history work");
        const auto saved=directory/"saved";checked(psdk_full_save_result(result,saved.string().c_str(),&status),status);
        for(const auto* name:{"pcl.cells.bin","ai.cells.bin","fusion.cells.bin"})require(fs::file_size(saved/name)==8000*sizeof(psdk_grid_cell),"C ABI result saving lost a map");
        // Invalid capture identity must fail before publishing another result.
        disparity.timestamp_ns+=1;psdk_full_result* rejected=nullptr;
        require(psdk_full_process(engine,&frame,&rejected,&status)!=0&&!rejected&&publications.calls==1,"Cross-capture Q8 was silently accepted");
        const auto retained_q8=view.disparity.data[69*width+80];
        std::fill(left.begin(),left.end(),0);std::fill(q8.begin(),q8.end(),0);
        checked(psdk_full_destroy(&engine,&status),status);require(engine==nullptr,"Destroy left engine handle live");
        view={};view.struct_size=sizeof(view);view.abi_version=PSDK_FULL_ABI_VERSION;
        checked(psdk_full_result_get_view(result,&view,&status),status);
        require(view.map_count==3&&view.disparity.data[69*width+80]==retained_q8&&observed(view.maps[0])==pcl_observed,"SO result did not own its data after caller mutation/engine destruction");
        psdk_full_result_release(result);result=nullptr;
        std::cout<<"PASS v0_mapping_cabi checks="<<checks<<" maps=3 ground_points="<<ground_points<<" pcl_observed="<<pcl_observed<<" fusion_observed="<<fusion_observed
                 <<" models_disabled=true scope=synthetic_geometry_only\nloaded_sdk="<<loaded_sdk
                 <<"\ntask_trace_json="<<trace_text<<"\neffective_config_json="<<effective_text<<'\n';
        return 0;
    }catch(const std::exception& e){if(result)psdk_full_result_release(result);if(engine)psdk_full_destroy(&engine,&status);std::cerr<<"V0_MAP_CABI_FAILED: "<<e.what()<<'\n';return 1;}
}
