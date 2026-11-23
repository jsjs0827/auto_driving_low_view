#include "perception_full_sdk.h"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>
#include <unistd.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

static void check(bool v,const char* m){if(!v)throw std::runtime_error(m);}
static std::uint32_t publish(void* u,const psdk_full_result_view* v,psdk_status*){
    check(v&&v->recharge.executed,"publication must follow actual recharge Feature, including warmup validity");
    check(v->frame_id==v->recharge.frame_id&&v->timestamp_ns==std::uint64_t(v->recharge.timestamp_ns),"publication identity");
    ++*static_cast<int*>(u);return 0;
}
static std::uint64_t hash(const std::uint8_t* bytes,std::uint64_t count){
    std::uint64_t h=1469598103934665603ULL;
    for(std::uint64_t i=0;i<count;++i)h=(h^bytes[i])*1099511628211ULL;
    return h;
}
int main(int argc,char** argv){
    psdk_full_engine* engine=nullptr;psdk_full_result *first=nullptr,*second=nullptr;
    try{
        check(argc==3||argc==4,"usage: r35_recharge_cabi full_recharge.json dual.ppm [--gdc-only]");
        std::ifstream f(argv[2],std::ios::binary);std::string magic;int w,h,max;
        f>>magic>>w>>h>>max;f.get();check(magic=="P6"&&w==1280&&h==1088&&max==255,"actual full-size dual fixture");
        std::vector<std::uint8_t> rgb(std::size_t(w)*h*3),hires(std::size_t(w)*h*3/2,128),left(640*544*3/2,128);
        check(bool(f.read(reinterpret_cast<char*>(rgb.data()),rgb.size())),"full fixture read");
        for(std::size_t i=0;i<std::size_t(w)*h;++i)hires[i]=std::uint8_t(16+((220*int(rgb[3*i])+128)>>8));
        int calls=0;psdk_full_hooks hooks{};hooks.struct_size=sizeof(hooks);hooks.abi_version=1;hooks.userdata=&calls;hooks.publication=&publish;
        // Only I/O paths are changed for this owned contract run. All algorithm
        // options (including the original three-frame gate) come from V0 config.
        namespace fs=std::filesystem;
        const auto owned=fs::temp_directory_path()/("recharge-v0-cabi-"+std::to_string(getpid()));
        check(!fs::exists(owned),"contract directory must be new");fs::create_directories(owned);
        auto read_json=[](const fs::path&p){std::ifstream f(p);check(bool(f),"read contract configuration");std::string t((std::istreambuf_iterator<char>(f)),{});rapidjson::Document d;d.Parse(t.c_str());check(!d.HasParseError()&&d.IsObject(),"valid contract JSON");return d;};
        auto write_json=[](const fs::path&p,const rapidjson::Document&d){rapidjson::StringBuffer b;rapidjson::Writer<rapidjson::StringBuffer>w(b);d.Accept(w);std::ofstream f(p);f<<b.GetString();check(bool(f),"write owned contract JSON");};
        auto full=read_json(argv[1]);
        // Public SDK source-RGB ownership is exercised without the selected
        // original SO's independently reproduced broken optional overlay path.
        full["output"]["save_overlay"].SetBool(true);
        if(full.HasMember("inference")&&full["inference"].IsObject()){
            for(const char* key:{"original_profile","dag"}){
                auto& inf=full["inference"];
                if(inf.HasMember(key)&&inf[key].IsString()&&inf[key].GetStringLength()){
                    const auto absolute=fs::absolute(fs::path(argv[1]).parent_path()/inf[key].GetString()).lexically_normal().string();
                    inf[key].SetString(absolute.c_str(),full.GetAllocator());
                }
            }
        }
        const auto source_cfg=fs::absolute(fs::path(argv[1]).parent_path()/full["recharge"]["config"].GetString());
        auto algorithm=read_json(source_cfg);auto&aa=algorithm.GetAllocator();
        algorithm["save_raw"]["enabled"].SetBool(false);algorithm["save_raw"]["dir"].SetString((owned/"raw").c_str(),aa);
        algorithm["save_result"]["dir"].SetString((owned/"result").c_str(),aa);
        algorithm["save_result"]["enabled"].SetBool(false);
        write_json(owned/"recharge.json",algorithm);
        full["recharge"]["config"].SetString((owned/"recharge.json").c_str(),full.GetAllocator());
        write_json(owned/"full.json",full);const auto full_path=(owned/"full.json").string();
        psdk_full_config c{};c.struct_size=sizeof(c);c.abi_version=1;c.json_file=full_path.c_str();c.hooks=&hooks;psdk_status status{};
        check(!psdk_full_create(&c,&engine,&status),status.detail);
        psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=1;
        frame.left={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,640,544,640,left.data(),left.size(),901,1234500000};
        psdk_frame high={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,1280,1088,1280,hires.data(),hires.size(),901,1234500001};
        frame.recharge_hires=&high;frame.intrinsics={640,544,298.4,298.4,313.8,274.3,17935.5};
        check(psdk_full_process(engine,&frame,&first,&status)==1002&&!first&&!calls,"mismatched GDC timestamp must fail with original error code");
        high.timestamp_ns=frame.left.timestamp_ns;
        if(argc==4){check(std::string(argv[3])=="--gdc-only","unknown option");
            check(psdk_full_capabilities()&PSDK_CAP_CPU_ORT,"cmd2 engine-initialized regression requires actual ORT build");
            frame.left=high;}
        // Exercise the real original continuous gate without changing it.
        // Invalid warmup frames are processed and released, not called valid.
        psdk_full_result_view a{};a.struct_size=sizeof(a);a.abi_version=1;
        for(int i=0;i<3;++i){
            check(!psdk_full_process(engine,&frame,&first,&status),status.detail);
            check(!psdk_full_result_get_view(first,&a,&status),status.detail);
            check(a.recharge.executed&&a.recharge.mark_ok,"real V0 detects the unchanged fixture");
            if(i<2){check(!a.recharge.pose_ok,"original V0 three-frame gate is retained");psdk_full_result_release(first);first=nullptr;++frame.left.frame_id;++frame.left.timestamp_ns;high.frame_id=frame.left.frame_id;high.timestamp_ns=frame.left.timestamp_ns;}
        }
        check(a.recharge.mark_ok&&a.recharge.pose_ok&&a.recharge.detected_point_count==8,"real original V0 dual pose after its continuous gate through full C ABI");
        check(!a.has_inference&&!a.map_count&&a.recharge.overlay_bytes==0,"default V0 configuration does not request its optional vendor overlay");
        check(a.rgb&&a.rgb_bytes>0&&a.image_width==frame.left.width&&a.image_height==frame.left.height&&a.rgb_bytes==std::uint64_t(a.image_width)*a.image_height*3,"public SDK returns actual nonempty owned source RGB at the input geometry");
        check(a.recharge.details_json&&*a.recharge.details_json,"public SDK returns real pose details");
        check(std::string(a.task_trace_json).find("recharge")!=std::string::npos,"original TaskGraph executes recharge");
        check(std::string(a.task_trace_json).find("\"task\":\"dnn\"")==std::string::npos,"command2 must skip inference even when models are initialized");
        const auto held=hash(a.rgb,a.rgb_bytes);const auto x=a.recharge.x_m,y=a.recharge.y_m,yaw=a.recharge.yaw_rad;
        const std::string details=a.recharge.details_json;
        // The next input differs, so an accidentally borrowed/reused RGB
        // buffer would change the earlier result. The GDC pose source stays
        // unchanged in the normal paired-input contract.
        auto& mutable_input=frame.left.data==hires.data()?hires:left;
        std::fill(mutable_input.begin(),mutable_input.begin()+std::size_t(frame.left.width)*frame.left.height,64);
        ++frame.left.frame_id;++frame.left.timestamp_ns;high.frame_id=frame.left.frame_id;high.timestamp_ns=frame.left.timestamp_ns;
        const int previous_calls=calls;
        check(!psdk_full_process(engine,&frame,&second,&status)&&calls==previous_calls+1,status.detail);
        check(hash(a.rgb,a.rgb_bytes)==held&&a.recharge.x_m==x&&a.recharge.y_m==y&&a.recharge.yaw_rad==yaw&&std::string(a.recharge.details_json)==details,"previous source RGB, pose and details stay owned across next frame");
        check(!psdk_full_command(engine,21,&status),status.detail);
        check(!psdk_full_destroy(&engine,&status),status.detail);
        check(hash(a.rgb,a.rgb_bytes)==held&&a.recharge.x_m==x&&a.recharge.y_m==y&&a.recharge.yaw_rad==yaw&&std::string(a.recharge.details_json)==details,"owned source RGB, pose and details outlive engine");
        psdk_full_result_release(first);first=nullptr;psdk_full_result_release(second);second=nullptr;
        std::cout<<"PASS full C ABI default V0 overlay=false: real gated dual pose, DAG publication, same-frame rejection, source RGB/pose/details ownership across frames/destroy; vendor overlay=true remains a separately recorded upstream failure\n";
        return 0;
    }catch(const std::exception&e){psdk_full_result_release(first);psdk_full_result_release(second);psdk_full_destroy(&engine,nullptr);std::cerr<<e.what()<<'\n';return 1;}
}
