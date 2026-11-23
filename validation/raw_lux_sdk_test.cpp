#include "perception_full_sdk.h"
#include "features/raw_lux/backend/include/percep_main/raw_lux_runner.hpp"
#include "json.hpp"
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace {
namespace fs=std::filesystem;
int checks=0;
void require(bool ok,const std::string& why){++checks;if(!ok)throw std::runtime_error(why);}
void check(std::uint32_t code,const psdk_status& status){require(code==0,"SDK failure "+std::to_string(code)+": "+status.detail);}
std::vector<std::uint8_t> rawFile(const fs::path& path){std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f),"Cannot open fixture "+path.string());require(f.tellg()==2785280,"Wrong RAW fixture size");f.seekg(0);std::vector<std::uint8_t> b(2785280);f.read(reinterpret_cast<char*>(b.data()),b.size());require(bool(f),"Incomplete fixture");return b;}
std::uint32_t published(void* userdata,const psdk_full_result_view* view,psdk_status*){
    if(!view)return 1002;
    ++*static_cast<unsigned*>(userdata);return 0;
}
struct Cleanup{fs::path path;~Cleanup(){std::error_code e;fs::remove_all(path,e);}};
const char* configJson=R"JSON({
"schema_version":1,"mode":"map",
"inference":{"enabled":false,"backend":"bpu","original_profile":"unused.cfg","dag":"unused.json"},
"mapping":{"enabled":false},"recharge":{"enabled":false},
"output":{"every_n":0,"save_overlay":false,"save_inputs":false},
"history":{"capacity":1,"difference":false,"match":false},
"graph":{"worker_threads":2,"nodes":[
 {"id":"lux","feature":"raw_lux"},
 {"id":"publish","feature":"publish","depends_on":["lux"]}]},
"algorithms":{"lux":{"algorithm":"raw_lux","parameters":{
 "model_name":"raw_hist_org_logistic","metadata_source":"org_proxy","include_features":true}}}
})JSON";
}
int main(int argc,char** argv){
    psdk_full_engine* engine=nullptr;psdk_full_result* retained=nullptr;psdk_status status{};
    try{
        require(argc==2,"raw_lux_sdk_test RAW_EXAMPLES_DIRECTORY");
        const auto root=fs::temp_directory_path()/("v2_raw_lux_sdk_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        Cleanup cleanup{root};fs::create_directories(root);const auto config_path=root/"full.json";{std::ofstream out(config_path);out<<configJson;}
        unsigned publications=0;psdk_full_hooks hooks{};hooks.struct_size=sizeof(hooks);hooks.abi_version=PSDK_FULL_ABI_VERSION;
        hooks.userdata=&publications;hooks.publication=published;
        const auto config_name=config_path.string();psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;config.json_file=config_name.c_str();config.hooks=&hooks;
        check(psdk_full_create(&config,&engine,&status),status);require(engine!=nullptr,"No SDK engine");
        auto l=rawFile(fs::path(argv[1])/"left.raw"),r=rawFile(fs::path(argv[1])/"right.raw");std::vector<std::uint8_t> yuv(12,128);
        psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
        auto& f=frame.left;f.struct_size=sizeof(f);f.abi_version=PSDK_ABI_VERSION;f.format=PSDK_FORMAT_NV12;
        f.data=yuv.data();f.bytes=yuv.size();f.width=4;f.height=2;f.stride=4;f.frame_id=1;f.timestamp_ns=1000;
        psdk_raw_stereo_input raw{};raw.struct_size=sizeof(raw);raw.abi_version=PSDK_RAW_INPUT_ABI_VERSION;raw.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
        auto& left=raw.left;left.struct_size=sizeof(left);left.abi_version=PSDK_RAW_INPUT_ABI_VERSION;left.width=1088;left.height=1280;left.stride_bytes=2176;
        left.format=PSDK_RAW_FORMAT_UINT16_LE;left.valid_bits=10;left.bayer=PSDK_RAW_BAYER_BGGR;left.data=l.data();left.bytes=l.size();left.frame_id=1;left.timestamp_ns=1000;
        raw.right=left;raw.right.data=r.data();auto& org=raw.left_org;org.struct_size=sizeof(org);org.abi_version=PSDK_RAW_INPUT_ABI_VERSION;org.present=1;
        org.exposure_ms=10;org.analog_gain=2.95365238;org.frame_id=1;org.timestamp_ns=1000;raw.right_org=org;frame.raw_light=&raw;
        check(psdk_full_process(engine,&frame,&retained,&status),status);require(retained&&publications==1,"Publish hook did not join real RAW feature");
        const char* json=nullptr;check(psdk_full_result_get_algorithms_json(retained,&json,&status),status);
        using namespace r33::json;const auto original_json=std::string(json);const auto parsed=JsonParser(original_json,"public SDK algorithms").parse();const auto& value=required(parsed,"lux");
        require(boolean(required(value,"valid"))&&string(required(value,"predicted_class"))=="LOW_LIGHT","Public SDK RAW fixture classification failed");
        perception::RawLuxRunner direct;perception::RawLuxConfig direct_config;direct_config.metadata_source="org_proxy";require(direct.init(direct_config),"Direct runner init");
        perception::RawLuxImageView dl,dr;dl.data=l.data();dl.bytes=l.size();dl.frame_id=1;dl.timestamp_ns=1000;dl.exposure_us=10000;dl.analog_gain=org.analog_gain;dr=dl;dr.data=r.data();perception::RawLuxResult reference;
        require(direct.predictPair("1",dl,dr,&reference),"Direct fixture prediction failed");
        for(std::size_t i=0;i<3;++i){const double got=number(required(required(value,"probabilities"),reference.class_names[i].c_str()));require(std::memcmp(&got,&reference.probabilities[i],sizeof(double))==0,"C ABI probability differs from direct model");}
        for(std::size_t i=0;i<reference.features.size();++i){const double got=number(required(required(value,"features"),reference.feature_names[i].c_str()));require(std::memcmp(&got,&reference.features[i],sizeof(double))==0,"C ABI feature differs from direct model");}
        const auto saved_path=root/"saved";check(psdk_full_save_result(retained,saved_path.string().c_str(),&status),status);
        require(fs::is_regular_file(saved_path/"result.json"),"SDK save_result did not save JSON");
        const auto saved=load((saved_path/"result.json").string());
        require(string(required(required(required(saved,"algorithms"),"lux"),"predicted_class"))=="LOW_LIGHT","Saved result omitted RAW algorithm");
        const auto missing=[&](std::uint32_t size,bool null_pointer){
            ++f.frame_id;f.timestamp_ns+=1000;frame.struct_size=size;frame.raw_light=null_pointer?nullptr:reinterpret_cast<const psdk_raw_stereo_input*>(1);
            psdk_full_result* result=nullptr;check(psdk_full_process(engine,&frame,&result,&status),status);require(result!=nullptr,"Missing RAW failed full SDK");
            const char* text=nullptr;check(psdk_full_result_get_algorithms_json(result,&text,&status),status);const auto all=JsonParser(text,"missing RAW C ABI result").parse();const auto& entry=required(all,"lux");
            require(!boolean(required(entry,"valid"))&&string(required(entry,"status"))=="missing_input","Missing RAW did not return explicit status");
            psdk_full_result_release(result);
        };
        missing(sizeof(frame),true);missing(PSDK_FULL_FRAME_V3_SIZE,false);missing(PSDK_FULL_FRAME_V2_SIZE,false);missing(PSDK_FULL_FRAME_V1_SIZE,false);
        require(publications==5,"Publish hook was blocked by absent/old-ABI RAW");
        check(psdk_full_destroy(&engine,&status),status);require(engine==nullptr,"Destroy did not clear engine");
        check(psdk_full_result_get_algorithms_json(retained,&json,&status),status);require(original_json==json,"Owned RAW result became invalid after engine destruction");
        psdk_full_result_release(retained);retained=nullptr;
        std::cout<<"PASS "<<checks<<" public RAW SDK checks; create/process/publish/get/save, direct-model bits, old ABI missing-RAW and result ownership passed\n";return 0;
    }catch(const std::exception& e){if(retained)psdk_full_result_release(retained);if(engine)psdk_full_destroy(&engine,&status);std::cerr<<e.what()<<'\n';return 1;}
}
