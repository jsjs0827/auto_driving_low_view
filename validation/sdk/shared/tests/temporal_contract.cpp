#include "perception_temporal_sdk.h"
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
void need(bool v,const char*s){if(!v)throw std::runtime_error(s);}
void ok(uint32_t code,const psdk_status&s){if(code)throw std::runtime_error(std::to_string(code)+":"+s.detail);}
using Result=std::unique_ptr<psdk_full_result,decltype(&psdk_full_result_release)>;
struct Engine{psdk_full_engine*p=nullptr;~Engine(){psdk_full_destroy(&p,nullptr);}};
struct State{psdk_full_engine*engine=nullptr;unsigned calls=0;bool fail=false;};
uint32_t extension(void* user,const psdk_temporal_view*w,psdk_status*error){
    auto&s=*static_cast<State*>(user);++s.calls;need(w->sampled&&w->frame_count,"extension gets current sampled snapshot");
    const auto&f=w->frames[w->frame_count-1];need(f.left.frame_id==w->current_frame_id&&f.left.timestamp_ns==w->current_timestamp_ns,"capture identity retained");
    psdk_status nested{};need(psdk_full_reset_history(s.engine,&nested)==2001,"extension mutation reentry rejected promptly");
    if(s.fail){error->native_code=-77;return 0xf123abcdu;}return 0;
}
psdk_temporal_view get(const Result&r){psdk_temporal_view v{};v.struct_size=sizeof(v);v.abi_version=1;psdk_status s{};ok(psdk_full_result_get_temporal_view(r.get(),&v,&s),s);return v;}
int main(int argc,char**argv){try{
    need(argc==2,"config required");Engine engine;psdk_full_config c{sizeof(c),1,argv[1],nullptr,nullptr,nullptr};psdk_status error{};
    ok(psdk_full_create(&c,&engine.p,&error),error);ok(psdk_full_command(engine.p,1,&error),error);
    State state{engine.p};ok(psdk_full_set_temporal_callback(engine.p,extension,&state,&error),error);
    std::vector<uint8_t> input(64*48*3/2);psdk_full_frame f{};f.struct_size=sizeof(f);f.abi_version=1;
    f.left={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,64,48,64,input.data(),input.size(),0,0};
    std::vector<Result> results;
    for(unsigned i=1;i<=8;++i){std::fill(input.begin(),input.end(),uint8_t(i));f.left.frame_id=i;f.left.timestamp_ns=i*1000;
        psdk_full_result*r=nullptr;ok(psdk_full_process(engine.p,&f,&r,&error),error);results.emplace_back(r,psdk_full_result_release);}
    auto v=get(results.back());need(v.capacity==4&&v.frame_count==4&&v.ready&&!v.sampled&&state.calls==4,"capacity/interval/warmup callback counts");
    for(unsigned i=0;i<4;++i)need(v.frames[i].left.frame_id==2*i+1&&v.frames[i].left.data[0]==2*i+1,"bounded sampled input bytes");
    auto first=get(results.front());need(first.frames[0].left.data[0]==1,"old result owns source despite input reuse");
    std::fill(input.begin(),input.end(),9);f.left.frame_id=9;f.left.timestamp_ns=9000;state.fail=true;psdk_full_result*r=nullptr;
    need(psdk_full_process(engine.p,&f,&r,&error)==0xf123abcdu&&!r&&error.native_code==-77&&std::string(error.task)=="temporal_extension","full uint32 extension status and failed output");
    need(get(results.back()).frames[3].left.frame_id==7,"failed frame cannot mutate prior results");
    state.fail=false;ok(psdk_full_process(engine.p,&f,&r,&error),error);results.emplace_back(r,psdk_full_result_release);v=get(results.back());
    need(v.frames[0].left.frame_id==3&&v.frames[3].left.frame_id==9&&v.difference_valid&&v.mean_abs_luma==2,"retry committed once and true frame difference");
    psdk_full_result_view full{};full.struct_size=sizeof(full);full.abi_version=1;ok(psdk_full_result_get_view(r,&full,&error),error);
    const std::string trace=full.task_trace_json;need(trace.find("temporal_difference")!=std::string::npos&&trace.find("temporal_match")!=std::string::npos,"actual temporal Feature tasks traced");
    ok(psdk_full_process(engine.p,&f,&r,&error),error);results.emplace_back(r,psdk_full_result_release);v=get(results.back());need(!v.sampled&&std::string(v.reset_reason)=="duplicate_ignored","same capture never becomes a second temporal sample");
    ok(psdk_full_reset_history(engine.p,&error),error);f.left.frame_id=10;f.left.timestamp_ns=10000;ok(psdk_full_process(engine.p,&f,&r,&error),error);results.emplace_back(r,psdk_full_result_release);
    need(get(results.back()).frame_count==1,"explicit history reset");
    ok(psdk_full_command(engine.p,21,&error),error);need(psdk_full_process(engine.p,&f,&r,&error)==2001&&!r,"stop still rejects processing");
    ok(psdk_full_command(engine.p,1,&error),error);ok(psdk_full_process(engine.p,&f,&r,&error),error);results.emplace_back(r,psdk_full_result_release);need(get(results.back()).frame_count==1,"mode transition resets history");
    ok(psdk_full_destroy(&engine.p,&error),error);need(get(results.front()).frames[0].left.data[0]==1,"history result outlives engine");
    psdk_temporal_view bad{};need(psdk_full_result_get_temporal_view(results.front().get(),&bad,&error)==1012,"new API validates struct ABI");
    std::cout<<"Public temporal C ABI passed: actual DAG, owned history, interval, full error return, failed-frame rollback, reentry, duplicate, command/reset and destruction. No ROS/hardware/accuracy claim.\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
