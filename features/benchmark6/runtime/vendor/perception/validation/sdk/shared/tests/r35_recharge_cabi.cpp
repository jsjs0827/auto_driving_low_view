#include "perception_full_sdk.h"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static void check(bool v,const char* m){if(!v)throw std::runtime_error(m);}
static std::uint32_t publish(void* u,const psdk_full_result_view* v,psdk_status*){
    check(v&&v->recharge.executed&&v->recharge.pose_ok,"publication must follow actual recharge Feature");
    check(v->frame_id==v->recharge.frame_id&&v->timestamp_ns==std::uint64_t(v->recharge.timestamp_ns),"publication identity");
    ++*static_cast<int*>(u);return 0;
}
static std::uint64_t hash(const psdk_dock_pose& p){
    std::uint64_t h=1469598103934665603ULL;
    for(std::uint64_t i=0;i<p.overlay_bytes;++i)h=(h^p.overlay_rgb[i])*1099511628211ULL;
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
        psdk_full_config c{};c.struct_size=sizeof(c);c.abi_version=1;c.json_file=argv[1];c.hooks=&hooks;psdk_status status{};
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
        check(!psdk_full_process(engine,&frame,&first,&status),status.detail);
        psdk_full_result_view a{};a.struct_size=sizeof(a);a.abi_version=1;
        check(!psdk_full_result_get_view(first,&a,&status),status.detail);
        check(a.recharge.mark_ok&&a.recharge.pose_ok&&a.recharge.detected_point_count==8,"real ArUco dual PnP through full C ABI");
        check(!a.has_inference&&!a.map_count&&a.recharge.overlay_bytes>0,"recharge-only returns real owned overlay");
        check(std::string(a.task_trace_json).find("recharge")!=std::string::npos,"original TaskGraph executes recharge");
        check(std::string(a.task_trace_json).find("\"task\":\"dnn\"")==std::string::npos,"command2 must skip inference even when models are initialized");
        auto held=hash(a.recharge);auto x=a.recharge.x_m;
        ++frame.left.frame_id;++frame.left.timestamp_ns;high.frame_id=frame.left.frame_id;high.timestamp_ns=frame.left.timestamp_ns;
        check(!psdk_full_process(engine,&frame,&second,&status)&&calls==2,status.detail);
        check(hash(a.recharge)==held&&a.recharge.x_m==x,"previous owned result stays valid across next frame");
        check(!psdk_full_command(engine,21,&status),status.detail);
        check(!psdk_full_destroy(&engine,&status),status.detail);
        check(hash(a.recharge)==held,"owned overlay outlives engine");
        psdk_full_result_release(first);first=nullptr;psdk_full_result_release(second);second=nullptr;
        std::cout<<"PASS full C ABI: actual ArUco dual PnP, DAG publication, same-frame rejection, status1002, owned overlay across frames/destroy\n";
        return 0;
    }catch(const std::exception&e){psdk_full_result_release(first);psdk_full_result_release(second);psdk_full_destroy(&engine,nullptr);std::cerr<<e.what()<<'\n';return 1;}
}
