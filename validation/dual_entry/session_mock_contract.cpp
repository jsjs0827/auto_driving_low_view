#include "perception_entry/session.hpp"
#include "perception_v0_host/controller.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// Synthetic C ABI transport fixture only. This test exercises the real bridge
// implementation, but neither the SDK algorithms nor camera/SHM transport.
struct psdk_full_engine { int command=-1; };
struct psdk_full_result {
    std::uint64_t frame_id=0,stamp=0;
    std::vector<std::uint8_t> pixels;
};
namespace {
struct Fixture {
    std::mutex mutex;
    std::condition_variable ready;
    unsigned creates=0,destroys=0,commands=0,processes=0,motions=0,releases=0,live_results=0;
    std::uint32_t next_command_error=0,next_process_error=0;
    bool block_process=false,process_entered=false,release_process=false;
} fixture;
unsigned checks=0;
void need(bool valid,const char* detail){++checks;if(!valid)throw std::runtime_error(detail);}
std::uint32_t status(psdk_status* s,std::uint32_t code,const char* detail="") {
    if(s){*s={};s->struct_size=sizeof(*s);s->abi_version=PSDK_ABI_VERSION;s->code=code;
        std::strncpy(s->feature,"synthetic.mock.C_ABI",sizeof(s->feature)-1);
        std::strncpy(s->detail,detail,sizeof(s->detail)-1);}
    return code;
}
psdk_full_frame frame(std::vector<std::uint8_t>& data,std::uint64_t id=71) {
    psdk_full_frame f{};f.struct_size=sizeof(f);f.abi_version=PSDK_FULL_ABI_VERSION;
    f.left={sizeof(psdk_frame),PSDK_ABI_VERSION,PSDK_FORMAT_NV12,4,2,4,data.data(),data.size(),id,id*1000};
    return f;
}
void unblock() {
    {std::lock_guard<std::mutex> lock(fixture.mutex);fixture.release_process=true;}
    fixture.ready.notify_all();
}
struct UnblockOnExit{~UnblockOnExit(){unblock();}};
void checkView(perception::entry::Result& result,std::uint64_t id,std::uint8_t pixel) {
    psdk_full_result_view view{};auto error=ros_so_demo::newStatus();
    need(result.view(view,error)==0,"result view failed");
    need(view.frame_id==id&&view.timestamp_ns==id*1000,"result identity changed");
    need(view.rgb_bytes==12&&view.rgb&&view.rgb[0]==pixel,"result lost its owned bytes");
}
}
extern "C" {
std::uint32_t psdk_full_create(const psdk_full_config* c,psdk_full_engine** e,psdk_status* s) {
    std::lock_guard<std::mutex> lock(fixture.mutex);++fixture.creates;
    if(!c||!c->json_file||std::string(c->json_file)=="fail-create")return status(s,2002,"injected create failure");
    *e=new psdk_full_engine;return status(s,0);
}
std::uint32_t psdk_full_destroy(psdk_full_engine** e,psdk_status* s) {
    std::lock_guard<std::mutex> lock(fixture.mutex);
    if(e&&*e){++fixture.destroys;delete *e;*e=nullptr;}
    return status(s,0);
}
std::uint32_t psdk_full_command(psdk_full_engine* e,std::int32_t command,psdk_status* s) {
    std::lock_guard<std::mutex> lock(fixture.mutex);++fixture.commands;
    if(!e)return status(s,2001,"closed engine");
    if(fixture.next_command_error){auto code=fixture.next_command_error;fixture.next_command_error=0;return status(s,code,"injected command failure");}
    if(command!=1&&command!=2&&command!=3&&command!=21)return status(s,2001,"invalid command");
    e->command=command;return status(s,0);
}
std::uint32_t psdk_full_process(psdk_full_engine* e,const psdk_full_frame* f,psdk_full_result** out,psdk_status* s) {
    std::unique_lock<std::mutex> lock(fixture.mutex);++fixture.processes;
    if(!e)return status(s,2001,"closed engine");
    fixture.process_entered=true;fixture.ready.notify_all();
    if(fixture.block_process)fixture.ready.wait(lock,[]{return fixture.release_process;});
    if(fixture.next_process_error){auto code=fixture.next_process_error;fixture.next_process_error=0;return status(s,code,"injected process failure");}
    auto* value=new psdk_full_result;value->frame_id=f->left.frame_id;value->stamp=f->left.timestamp_ns;
    value->pixels.assign(f->left.data,f->left.data+f->left.bytes);*out=value;++fixture.live_results;
    return status(s,0);
}
std::uint32_t psdk_full_result_get_view(const psdk_full_result* result,psdk_full_result_view* view,psdk_status* s) {
    if(!result)return status(s,2001,"null result");
    view->frame_id=result->frame_id;view->timestamp_ns=result->stamp;view->rgb=result->pixels.data();view->rgb_bytes=result->pixels.size();
    return status(s,0);
}
void psdk_full_result_release(psdk_full_result* result) {
    if(!result)return;
    std::lock_guard<std::mutex> lock(fixture.mutex);++fixture.releases;--fixture.live_results;delete result;
}
std::uint32_t psdk_full_push_motion_samples(psdk_full_engine* e,const psdk_motion_sample*,std::uint32_t,psdk_status* s) {
    std::lock_guard<std::mutex> lock(fixture.mutex);++fixture.motions;return status(s,e?0:2001);
}
}
int main() try {
    static_assert(std::is_same_v<perception::entry::Session,ros_so_demo::Caller>,"Session must preserve the old caller type");
    static_assert(std::is_same_v<perception::entry::Result,ros_so_demo::Result>,"Result ownership must be shared");
    using perception::entry::Session;using perception::entry::Result;
    auto error=ros_so_demo::newStatus();
    Session session;
    need(!session.isOpen()&&session.currentCommand()==-1,"new session state");
    need(session.open("fail-create",error)==2002&&!session.isOpen(),"failed open changed state");
    need(session.open("synthetic.json",error)==0&&session.isOpen(),"open did not recover");
    const auto creates=fixture.creates;
    need(session.open("synthetic.json",error)==2001&&fixture.creates==creates,"duplicate open reached C ABI");
    need(session.command(1,error)==0&&session.currentCommand()==1,"map command state");
    const auto first_command=fixture.commands;
    error.code=999;std::strcpy(error.detail,"stale error");
    need(session.command(1,error)==0&&fixture.commands==first_command&&error.code==0,"same command was not an idempotent success");
    need(session.restartCommand(1,error)==0&&fixture.commands==first_command+1,"forced restart did not reach SDK");
    fixture.next_command_error=2003;
    need(session.command(2,error)==2003&&session.currentCommand()==1,"failed command changed mode");
    const auto after_failure=fixture.commands;
    need(session.command(2,error)==0&&fixture.commands==after_failure+1&&session.currentCommand()==2,"failed command was cached as applied");
    fixture.next_command_error=2004;
    need(session.restartCommand(1,error)==2004&&session.currentCommand()==2,"failed restart changed mode");
    need(session.command(99,error)==2001&&session.currentCommand()==2,"invalid command changed mode");
    need(session.pushMotion(nullptr,0,error)==0&&fixture.motions==1,"old caller motion forwarding changed");

    std::vector<std::uint8_t> pixels(12,19);auto input=frame(pixels);
    Result retained;
    need(session.process(input,retained,error)==0&&retained.get(),"process did not own result");
    std::fill(pixels.begin(),pixels.end(),99);checkView(retained,71,19);
    Result moved=std::move(retained);need(!retained.get()&&moved.get(),"move construction did not transfer result");
    Result replaced;need(session.process(input,replaced,error)==0,"replacement fixture process failed");
    const auto releases=fixture.releases;replaced=std::move(moved);
    need(!moved.get()&&fixture.releases==releases+1,"move assignment did not release prior result once");
    checkView(replaced,71,19);
    fixture.next_process_error=1011;
    need(session.process(input,replaced,error)==1011&&!replaced.get()&&fixture.live_results==0,"failed process retained stale result");

    fixture.block_process=true;fixture.process_entered=false;fixture.release_process=false;
    UnblockOnExit release_guard;
    Result concurrent_result;auto process_error=ros_so_demo::newStatus(),close_error=ros_so_demo::newStatus();
    auto process=std::async(std::launch::async,[&]{return session.process(input,concurrent_result,process_error);});
    bool entered=false;
    {std::unique_lock<std::mutex> lock(fixture.mutex);entered=fixture.ready.wait_for(lock,std::chrono::seconds(2),[]{return fixture.process_entered;});}
    if(!entered){unblock();process.get();throw std::runtime_error("process did not reach blocking fixture");}
    std::promise<void> close_started;auto close_has_started=close_started.get_future();
    auto close=std::async(std::launch::async,[&]{close_started.set_value();return session.close(close_error);});
    close_has_started.wait();const bool waited=close.wait_for(std::chrono::milliseconds(100))==std::future_status::timeout;
    unsigned destroys_while_active=0;{std::lock_guard<std::mutex> lock(fixture.mutex);destroys_while_active=fixture.destroys;}
    unblock();const auto process_code=process.get(),close_code=close.get();
    need(waited&&destroys_while_active==0,"close destroyed an active process engine");
    need(process_code==0&&close_code==0&&!session.isOpen()&&session.currentCommand()==-1,"serialized process/close did not complete");
    need(fixture.destroys==1,"engine destroyed more than once");
    pixels.clear();pixels.shrink_to_fit();checkView(concurrent_result,71,99);
    need(session.close(error)==0&&fixture.destroys==1,"close is not idempotent");
    need(session.command(1,error)!=0&&session.currentCommand()==-1,"closed command used stale cached success");
    const auto released_before=fixture.releases;concurrent_result.reset();concurrent_result.reset();
    need(fixture.releases==released_before+1&&fixture.live_results==0,"result release not exactly once");
    need(session.open("synthetic.json",error)==0&&session.currentCommand()==-1,"reopen retained mode");
    const auto before_reopen_command=fixture.commands;
    need(session.command(2,error)==0&&fixture.commands==before_reopen_command+1,"reopen suppressed first command");
    need(session.close(error)==0,"final close failed");
    fixture.block_process=false;
    Session managed;need(managed.open("synthetic.json",error)==0,"controller fixture open");
    perception::v0_host::BoardController board(managed);std::string input_error;
    need(board.setIntrinsics({1280,1088,600,620,640,544,60,36000},input_error),"controller calibration");
    need(board.onCommand(1,error)==0&&board.status()==1,"controller map command");
    perception::v0_host::Nv12Image rgbd;rgbd.width=640;rgbd.height=544;rgbd.stride=640;
    rgbd.frame_id=82;rgbd.timestamp_ns=82000;rgbd.bytes.assign(640*544*3/2,82);
    perception::v0_host::Q8Disparity disparity;disparity.width=640;disparity.height=544;disparity.stride_bytes=1280;
    disparity.frame_id=82;disparity.timestamp_ns=82000;disparity.samples.assign(640*544,256);
    need(board.onRgbd(rgbd,disparity,input_error),"controller enqueue");
    const auto heartbeat_commands=fixture.commands;
    need(board.onCommand(1,error)==0&&fixture.commands==heartbeat_commands&&board.waitForWork(std::chrono::milliseconds(0)),
         "controller heartbeat reset pending frame");
    need(board.onCommand(3,error)==1002&&fixture.commands==heartbeat_commands,"unsupported board command reached SDK");
    fixture.next_command_error=2003;
    need(board.onCommand(2,error)==2003&&managed.currentCommand()==1&&board.waitForWork(std::chrono::milliseconds(0)),
         "failed mode command discarded old pending capture");
    unsigned publications=0;Result board_result;
    need(board.step(board_result,error,[&](const psdk_full_result_view& v){++publications;need(v.frame_id==82,"controller publication identity");})&&error.code==0,
         "controller failed to recover after command rejection");
    need(publications==1&&board.status()==1,"controller published duplicate or wrong status");
    need(board.onRgbd(rgbd,disparity,input_error),"controller failure fixture enqueue");
    fixture.next_process_error=1011;
    need(board.step(board_result,error,[&](const psdk_full_result_view&){++publications;})&&error.code==1011&&
         !board_result.get()&&publications==1&&board.status()==-1,"controller published failed result");

    need(board.onRgbd(rgbd,disparity,input_error),"controller concurrency fixture enqueue");
    fixture.block_process=true;fixture.process_entered=false;fixture.release_process=false;
    std::promise<void> publication_entered,publication_release;
    auto publication_has_entered=publication_entered.get_future();auto publication_can_finish=publication_release.get_future().share();
    auto board_error=ros_so_demo::newStatus(),stop_error=ros_so_demo::newStatus();
    auto step=std::async(std::launch::async,[&]{return board.step(board_result,board_error,[&](const psdk_full_result_view&){
        publication_entered.set_value();publication_can_finish.wait();++publications;});});
    {std::unique_lock<std::mutex> lock(fixture.mutex);entered=fixture.ready.wait_for(lock,std::chrono::seconds(2),[]{return fixture.process_entered;});}
    if(!entered){unblock();publication_release.set_value();step.get();throw std::runtime_error("controller process did not enter");}
    std::string async_input_error;
    auto queued_during_process=std::async(std::launch::async,[&]{return board.onRgbd(rgbd,disparity,async_input_error);});
    const bool producer_remained_responsive=queued_during_process.wait_for(std::chrono::milliseconds(100))==std::future_status::ready;
    auto status_during_process=std::async(std::launch::async,[&]{return board.status();});
    const bool status_remained_responsive=status_during_process.wait_for(std::chrono::milliseconds(100))==std::future_status::ready;
    std::promise<void> stop_started;auto stop_has_started=stop_started.get_future();
    auto stop=std::async(std::launch::async,[&]{stop_started.set_value();return board.onCommand(21,stop_error);});
    stop_has_started.wait();const bool stop_waited_for_process=stop.wait_for(std::chrono::milliseconds(100))==std::future_status::timeout;
    unblock();const bool published_before_stop=publication_has_entered.wait_for(std::chrono::seconds(2))==std::future_status::ready;
    const bool stop_waited_for_publish=stop.wait_for(std::chrono::milliseconds(100))==std::future_status::timeout;
    publication_release.set_value();const auto stepped=step.get();const auto stopped=stop.get();
    need(stop_waited_for_process&&published_before_stop&&stop_waited_for_publish&&stepped&&stopped==0,
         "stop returned before an active old-mode process/publication finished");
    const bool queued=queued_during_process.get();(void)status_during_process.get();
    need(producer_remained_responsive&&status_remained_responsive&&queued,"SDK process blocked producer enqueue or status lookup");
    need(board.status()==0&&publications==2&&!board.waitForWork(std::chrono::milliseconds(0)),"stop retained old-mode work");
    board.shutdown();need(board.onCommand(1,error)==1002&&!board.onRgbd(rgbd,disparity,input_error),"shutdown accepted new work");
    board_result.reset();need(managed.close(error)==0&&fixture.live_results==0,"controller leaked result or engine");
    std::cout<<"PASS "<<checks<<" session bridge checks; synthetic C ABI fixture only, no algorithms, camera, SHM or model execution\n";
    return 0;
} catch(const std::exception& exception) {unblock();std::cerr<<exception.what()<<'\n';return 1;}
