#include "perception_full_sdk.h"
#include "json.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
namespace {
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
void check(std::uint32_t code,const psdk_status& status){if(code)throw std::runtime_error(std::to_string(code)+": "+status.detail);}
}
int main(int argc,char** argv){
 psdk_full_engine* engine=nullptr;psdk_full_result* result=nullptr;
 try{
    require(argc==2,"algorithm_runtime_test EFFECTIVE_FULL_JSON");
    psdk_status status{};psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;config.json_file=argv[1];
    check(psdk_full_create(&config,&engine,&status),status);
    const int width=16,height=16;std::vector<std::uint8_t> bytes(width*height*3/2,82);
    for(std::size_t i=width*height;i<bytes.size();i+=2){bytes[i]=90;bytes[i+1]=240;}
    psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    frame.left.struct_size=sizeof(frame.left);frame.left.abi_version=PSDK_ABI_VERSION;frame.left.format=PSDK_FORMAT_NV12;
    frame.left.data=bytes.data();frame.left.bytes=bytes.size();frame.left.width=width;frame.left.height=height;frame.left.stride=width;
    frame.left.frame_id=1;frame.left.timestamp_ns=1000000;
    check(psdk_full_process(engine,&frame,&result,&status),status);
    const char* data=nullptr;check(psdk_full_result_get_algorithms_json(result,&data,&status),status);
    using namespace r33::json;const auto first=JsonParser(data,"algorithm results").parse();
    require(first.object.size()==2,"Two independent classifiers did not execute");
    for(const auto& item:first.object){const auto& top=required(item.second,"top_k");require(integer(required(top.array.front(),"class_id"),0,2)==0,"Red input did not select the red class");}
    const auto retained=std::string(data);
    // The same real model must respond to changed pixels, not return a stub.
    std::fill(bytes.begin(),bytes.begin()+width*height,41);
    for(std::size_t i=width*height;i<bytes.size();i+=2){bytes[i]=240;bytes[i+1]=110;}
    psdk_full_result* blue=nullptr;frame.left.frame_id=2;frame.left.timestamp_ns=2000000;
    auto right=frame.left,rear=frame.left;rear.frame_id=20;rear.timestamp_ns=1900000;
    psdk_imu_sample imu_sample{};imu_sample.timestamp_ns=1900000;
    psdk_imu_input imu{sizeof(imu),PSDK_SENSOR_INPUT_ABI_VERSION,&imu_sample,1,0};
    psdk_motion_sample motion{};motion.struct_size=sizeof(motion);motion.abi_version=PSDK_SENSOR_INPUT_ABI_VERSION;
    motion.timestamp_ns=1900000;motion.linear_speed_mps=.5;
    psdk_wheel_input wheel{sizeof(wheel),PSDK_SENSOR_INPUT_ABI_VERSION,&motion,1,0};
    psdk_radar_point point{1,0,0,0,0};
    psdk_radar_input radar{sizeof(radar),PSDK_SENSOR_INPUT_ABI_VERSION,0,0,1900000,&point,1};
    frame.right=&right;frame.rear=&rear;frame.imu=&imu;frame.wheel=&wheel;frame.radar=&radar;
    rear.bytes=1;
    require(psdk_full_process(engine,&frame,&blue,&status)==1002&&blue==nullptr,
        "SDK did not reject an invalid rear input before executing algorithms");
    rear.bytes=bytes.size();
    check(psdk_full_process(engine,&frame,&blue,&status),status);
    check(psdk_full_result_get_algorithms_json(blue,&data,&status),status);
    const auto second=JsonParser(data,"algorithm results").parse();
    for(const auto& item:second.object){const auto& top=required(item.second,"top_k");require(integer(required(top.array.front(),"class_id"),0,2)==2,"Blue input did not select the blue class");}
    psdk_full_result_release(blue);
    check(psdk_full_destroy(&engine,&status),status);
    check(psdk_full_result_get_algorithms_json(result,&data,&status),status);
    require(retained==data,"Algorithm result did not retain ownership after engine destruction");
    psdk_full_result_release(result);result=nullptr;
    require(psdk_full_result_get_algorithms_json(nullptr,&data,&status)!=0&&data==nullptr,"Invalid getter did not clear its output");
    std::cout<<"Real ONNX classifier changed with input; two instances, ownership and C ABI passed\n";
    return 0;
 }catch(const std::exception& e){if(result)psdk_full_result_release(result);psdk_status ignored{};if(engine)psdk_full_destroy(&engine,&ignored);std::cerr<<e.what()<<'\n';return 1;}
}
