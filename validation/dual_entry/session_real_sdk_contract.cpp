#include "perception_entry/session.hpp"
#include "perception_v0_host/controller.hpp"
#include "perception_temporal_sdk.h"
#include "json.hpp"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Runs the real SDK temporal/DAG implementation with synthetic descriptors.
// It does not test a real upstream producer, IPC/SHM, camera, BPU or model.
namespace {
using namespace perception::v0_host;
unsigned checks=0;
void need(bool valid,const std::string& detail){++checks;if(!valid)throw std::runtime_error(detail);}
void ok(std::uint32_t code,const psdk_status& status){need(code==0,"SDK code="+std::to_string(code)+" "+status.detail);}
struct PublicationState {perception::entry::Session* session=nullptr;unsigned count=0;std::uint32_t reentry_code=0;};
std::uint32_t published(void* user,const psdk_full_result_view* view,psdk_status*) {
    if(!view)return 1002;
    auto& state=*static_cast<PublicationState*>(user);++state.count;
    // The bridge lock must not deadlock same-thread SDK callback reentry. The
    // SDK itself rejects the mutation and the bridge must retain its old mode.
    auto error=ros_so_demo::newStatus();state.reentry_code=state.session->command(21,error);
    return 0;
}
std::string normalizedTrace(const char* text) {
    using namespace r33::json;
    need(text,"missing actual DAG trace");const auto trace=JsonParser(text,"real SDK trace").parse();
    std::string result;
    for(const auto& task:required(trace,"tasks").array) {
        const auto id=string(required(task,"task"));const auto state=string(required(task,"state"));
        const auto& error=required(task,"error");
        need(state=="succeeded","real DAG task did not succeed: "+id+" "+state);
        result+=id+":"+state+":"+error.text+"\n";
    }
    need(result.find("history:")!=std::string::npos&&result.find("difference:")!=std::string::npos&&
         result.find("publish:")!=std::string::npos,"expected configured real DAG tasks absent");
    return result;
}
psdk_temporal_view temporal(const perception::entry::Result& result) {
    psdk_temporal_view view{};view.struct_size=sizeof(view);view.abi_version=PSDK_TEMPORAL_ABI_VERSION;
    auto status=ros_so_demo::newStatus();ok(psdk_full_result_get_temporal_view(result.get(),&view,&status),status);return view;
}
}
int main(int argc,char** argv) try {
    need(argc==2,"pass synthetic temporal-only JSON");
    // Offline API spelling remains valid, and board transport consumes the
    // shared Session API. Both run the identical public SDK and JSON graph.
    ros_so_demo::Caller offline;perception::entry::Session live;
    BoardController board(live);auto status=ros_so_demo::newStatus();
    PublicationState offline_publications{&offline},board_publications{&live};
    psdk_full_hooks a{sizeof(psdk_full_hooks),PSDK_FULL_ABI_VERSION,&offline_publications,nullptr,published};
    psdk_full_hooks b{sizeof(psdk_full_hooks),PSDK_FULL_ABI_VERSION,&board_publications,nullptr,published};
    ok(offline.open(argv[1],status,nullptr,nullptr,&a),status);ok(live.open(argv[1],status,nullptr,nullptr,&b),status);
    ok(offline.command(1,status),status);ok(board.onCommand(1,status),status);
    std::string error;need(board.setIntrinsics({1280,1088,600,620,640,544,60,36000},error),error);
    perception::entry::Result first_offline,first_board,last_offline,last_board;
    for(std::uint64_t id=501;id<=504;++id) {
        Nv12Image input;input.width=640;input.height=544;input.stride=640;input.frame_id=id;input.timestamp_ns=id*1000000;
        input.bytes.assign(640*544*3/2,static_cast<std::uint8_t>(id-500));
        Q8Disparity disparity;disparity.width=640;disparity.height=544;disparity.stride_bytes=1280;
        disparity.frame_id=id;disparity.timestamp_ns=input.timestamp_ns;disparity.samples.assign(640*544,256);
        InputFrame offline_input;offline_input.command=1;offline_input.left=input;offline_input.disparity=disparity;
        offline_input.intrinsics={640,544,300,310,320,272,18000};
        psdk_frame high{};psdk_disparity_q8 q{};const auto submitted=offline_input.view(high,q);
        ok(offline.process(submitted,last_offline,status),status);
        need(board.onRgbd(input,disparity,error),"board enqueue: "+error);
        need(board.waitForWork(std::chrono::milliseconds(0)),"board frame not available");
        unsigned external_publications=0;
        need(board.step(last_board,status,[&](const psdk_full_result_view& view){++external_publications;
            need(view.frame_id==id&&view.timestamp_ns==input.timestamp_ns,"board publication identity changed");}),"board did not process queued frame");
        ok(status.code,status);need(external_publications==1,"board published a frame more than once");
        psdk_full_result_view av{},bv{};ok(last_offline.view(av,status),status);ok(last_board.view(bv,status),status);
        need(av.frame_id==id&&bv.frame_id==id&&av.timestamp_ns==input.timestamp_ns&&bv.timestamp_ns==input.timestamp_ns,"entry result identity differs");
        need(av.effective_config_json&&bv.effective_config_json&&std::string(av.effective_config_json)==bv.effective_config_json,"entries used different effective SDK configuration");
        need(normalizedTrace(av.task_trace_json)==normalizedTrace(bv.task_trace_json),"entries executed different real DAG provenance");
        need(av.run_id==id-500&&bv.run_id==id-500,"entry run count changed");
        need(!av.has_inference&&!bv.has_inference&&!av.map_count&&!bv.map_count&&!av.recharge.executed&&!bv.recharge.executed,"synthetic temporal test unexpectedly executed model/map/recharge");
        const auto at=temporal(last_offline),bt=temporal(last_board);
        need(at.frame_count==id-500&&bt.frame_count==id-500,"same-mode command reset temporal state");
        need(at.frames[at.frame_count-1].left.data[0]==id-500&&bt.frames[bt.frame_count-1].left.data[0]==id-500,"owned history differs between entries");
        const char *algorithms_a=nullptr,*algorithms_b=nullptr;
        ok(psdk_full_result_get_algorithms_json(last_offline.get(),&algorithms_a,&status),status);
        ok(psdk_full_result_get_algorithms_json(last_board.get(),&algorithms_b,&status),status);
        need(algorithms_a&&algorithms_b&&std::string(algorithms_a)==algorithms_b,"entry algorithm JSON differs");
        if(id==501){first_offline=std::move(last_offline);first_board=std::move(last_board);}
        if(id==502){ok(offline.command(1,status),status);ok(board.onCommand(1,status),status);}
    }
    need(offline_publications.count==4&&board_publications.count==4,"actual SDK publication task was skipped or repeated");
    need(offline_publications.reentry_code==2001&&board_publications.reentry_code==2001&&
         offline.currentCommand()==1&&live.currentCommand()==1,"SDK callback reentry was not promptly rejected without mode mutation");
    ok(offline.command(21,status),status);ok(board.onCommand(21,status),status);
    need(!board.waitForWork(std::chrono::milliseconds(0)),"stop left queued board frame");
    board.shutdown();ok(offline.close(status),status);ok(live.close(status),status);
    const auto old_a=temporal(first_offline),old_b=temporal(first_board);
    need(old_a.frame_count==1&&old_b.frame_count==1&&old_a.frames[0].left.data[0]==1&&old_b.frames[0].left.data[0]==1,
         "first result did not retain owned history after producer-buffer reuse and engine close");
    const auto final_a=temporal(last_offline),final_b=temporal(last_board);
    need(final_a.frame_count==4&&final_b.frame_count==4&&final_a.frames[3].left.data[0]==4&&final_b.frames[3].left.data[0]==4,
         "last result invalid after engine close");
    first_offline.reset();first_board.reset();last_offline.reset();last_board.reset();
    std::cout<<"PASS "<<checks<<" real SDK entry-parity checks; synthetic NV12/Q8, temporal/history DAG only; no real SHM/camera/BPU/model/accuracy claim\n";
    return 0;
} catch(const std::exception& exception){std::cerr<<exception.what()<<'\n';return 1;}
