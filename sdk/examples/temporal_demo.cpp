#include "perception_temporal_sdk.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
// TSV: capture_frame_id<TAB>capture_timestamp_ns<TAB>NV12_path
// Captures must retain their actual identity; this demo does not relabel them.
int main(int argc,char**argv){psdk_full_engine* engine=nullptr;try{
    if(argc!=3)throw std::runtime_error("usage: perception_temporal_demo SDK.json frames.tsv (640x544 NV12; mapping=false, inference-only or temporal-only config)");
    auto check=[](uint32_t c,const psdk_status&s){if(c)throw std::runtime_error(std::to_string(c)+":"+s.detail);};
    psdk_status status{};psdk_full_config cfg{sizeof(cfg),1,argv[1],nullptr,nullptr,nullptr};check(psdk_full_create(&cfg,&engine,&status),status);
    check(psdk_full_command(engine,1,&status),status);
    std::ifstream list(argv[2]);if(!list)throw std::runtime_error("Cannot open frames.tsv");std::string line;unsigned count=0;
    while(std::getline(list,line)){
        if(line.empty()||line[0]=='#')continue;
        std::istringstream row(line);std::string id,stamp,name;
        if(!std::getline(row,id,'\t')||!std::getline(row,stamp,'\t')||!std::getline(row,name))throw std::runtime_error("Invalid capture TSV");
        auto path=std::filesystem::u8path(name);if(path.is_relative())path=std::filesystem::u8path(argv[2]).parent_path()/path;
        constexpr unsigned bytes=640*544*3/2;std::vector<uint8_t> input(bytes);std::ifstream file(path,std::ios::binary);
        if(std::filesystem::file_size(path)!=bytes||!file.read(reinterpret_cast<char*>(input.data()),bytes))throw std::runtime_error("Require full640x544 tight NV12 capture");
        psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=1;
        frame.left={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,640,544,640,input.data(),input.size(),std::stoull(id),std::stoull(stamp)};
        frame.intrinsics={640,544,298.404816,298.404816,313.862183,274.359955,17935.5};
        psdk_full_result* result=nullptr;check(psdk_full_process(engine,&frame,&result,&status),status);
        psdk_temporal_view view{};view.struct_size=sizeof(view);view.abi_version=PSDK_TEMPORAL_ABI_VERSION;
        const auto code=psdk_full_result_get_temporal_view(result,&view,&status);
        if(code){psdk_full_result_release(result);check(code,status);}
        std::cout<<view.analysis_json<<'\n';psdk_full_result_release(result);++count;
    }
    if(!count)throw std::runtime_error("No captures in TSV");check(psdk_full_destroy(&engine,&status),status);return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';psdk_full_destroy(&engine,nullptr);return 1;}}
