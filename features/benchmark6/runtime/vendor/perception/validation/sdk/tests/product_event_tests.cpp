#include "product/product_events.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
namespace fs=std::filesystem;
namespace rt=perception::runtime;
using Events=r30::product::ProductEvents;
using namespace std::chrono_literals;
void check(bool condition,const char* detail) {if(!condition)throw std::runtime_error(detail);}
template<class T> T bounded(std::future<T>& future,const char* detail) {
    if(future.wait_for(5s)!=std::future_status::ready) {
        // std::async future destruction may itself wait forever after a broken
        // graph lock. Fail the test process before destructing such a future.
        std::cerr<<"Bounded event test timed out: "<<detail<<std::endl;
        std::_Exit(2);
    }
    return future.get();
}
struct Temporary {
    fs::path root=fs::temp_directory_path()/
        ("r30_product_events_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Temporary(){fs::create_directories(root);}
    ~Temporary(){std::error_code error;fs::remove_all(root,error);}
};
std::string read(const fs::path& path) {
    std::ifstream stream(path,std::ios::binary);check(bool(stream),"Test output file absent");
    return std::string(std::istreambuf_iterator<char>(stream),{});
}
void write(const fs::path& path,const std::string& data) {
    std::ofstream stream(path,std::ios::binary);stream.exceptions(std::ios::failbit|std::ios::badbit);
    stream.write(data.data(),std::streamsize(data.size()));stream.close();
}
std::string replaceOnce(std::string value,const std::string& before,const std::string& after) {
    const auto index=value.find(before);check(index!=std::string::npos,"Bad-config fixture anchor absent");
    value.replace(index,before.size(),after);return value;
}
const rt::TaskReport& task(const rt::GraphReport& report,const std::string& id) {
    const auto found=std::find_if(report.tasks.begin(),report.tasks.end(),[&](const auto& row){return row.id==id;});
    check(found!=report.tasks.end(),"Expected event task report absent");return *found;
}
std::vector<std::string> columns(const std::string& line) {
    std::istringstream stream(line);std::vector<std::string> result;std::string field;
    while(std::getline(stream,field,','))result.push_back(field);return result;
}
std::uint64_t integer(const std::string& line,const std::string& key) {
    const auto index=line.find('"'+key+'"'+':');check(index!=std::string::npos,"Status JSON correlation key absent");
    return std::stoull(line.substr(index+key.size()+3));
}
struct Result {std::uint32_t code=0;rt::GraphReport report;r28::Status status;};
Result run(const std::shared_ptr<Events>& events,const std::string& event,const Events::Bindings& bindings,
           perception::FrameInfo frame,std::uint64_t parent) {
    Result result;result.code=events->run(event,bindings,frame,parent,result.report,result.status);return result;
}
}

int main(int argc,char** argv) {try {
    check(argc==2,"Pass product/config/product_events.json");const std::string config_path=argv[1];
    Temporary temp;const auto metrics=temp.root/"metrics";
    auto events=std::make_shared<Events>(config_path,metrics.string());
    check(events->registeredFeatures().size()==6,"Product event Feature inventory changed");
    const perception::FrameInfo frame{42,987654321};
    std::vector<std::string> order;bool accepted=false;
    Events::Bindings params;
    params["params"]=[&]{accepted=true;order.push_back("accept");};
    params["params_save"]=[&]{check(accepted,"Parameters persisted before acceptance");
        order.push_back("save");write(temp.root/"camera_params.json","{\"fx\":359.375}\n");};
    auto result=run(events,"camera_params",params,frame,70);
    check(result.code==0&&result.report.ok()&&order==std::vector<std::string>({"accept","save"}),"Actual params/save graph order failed");
    check(read(temp.root/"camera_params.json")=="{\"fx\":359.375}\n","Camera parameters were not persisted");
    check(task(result.report,"params").state==rt::TaskState::Succeeded&&
          task(result.report,"params_save").state==rt::TaskState::Succeeded,"Params/save task reports incorrect");
    bool saved_after_failure=false;
    params["params"]=[]{throw r28::Failure({1002,-2147483000,"","","camera parameters rejected",{}});};
    params["params_save"]=[&]{saved_after_failure=true;};
    result=run(events,"camera_params",params,frame,71);
    check(result.code==1002&&result.status.native_code==-2147483000&&
          result.status.frame.id==42&&result.status.frame.timestamp_ns==987654321&&
          result.status.feature=="product.input.camera_params"&&result.status.task=="params",
          "Parameter failure lost business/native code or identity");
    check(!saved_after_failure&&task(result.report,"params").state==rt::TaskState::Failed&&
          task(result.report,"params_save").state==rt::TaskState::Skipped,"Parameter rejection did not skip persistence");
    params.erase("params_save");params["params"]=[]{};
    result=run(events,"camera_params",params,frame,72);
    check(result.code==0&&result.report.tasks.size()==1,"Acceptance-only event incorrectly requires saving");

    const std::string payload("real\0record\xff",12);
    Events::Bindings record{{"record_io",[&]{write(temp.root/"record.bin",payload);}}};
    result=run(events,"record_io",record,frame,80);
    check(result.code==0&&read(temp.root/"record.bin")==payload&&
          task(result.report,"record_io").state==rt::TaskState::Succeeded,"record_io did not execute real file I/O");
    write(temp.root/"blocked_parent","regular file, not a directory");
    int native_io=0;
    record["record_io"]=[&]{
        try {write(temp.root/"blocked_parent"/"record.bin",payload);}
        catch(const std::system_error& error){native_io=error.code().value();throw;}
    };
    result=run(events,"record_io",record,frame,81);
    check(result.code==2002&&native_io!=0&&result.status.native_code==native_io&&
          result.status.feature=="product.output.record_io"&&result.status.task=="record_io",
          "Actual recording failure did not preserve2002/native context");

    // Command waits for a separate callback event, just as a vendor command
    // waits for recharge completion. Its own wait is bounded even if graph
    // isolation regresses, so the callback can subsequently unwind.
    std::promise<void> entered_promise,completed_promise;
    auto entered=entered_promise.get_future();auto completed=completed_promise.get_future().share();
    auto command=std::async(std::launch::async,[events,&entered_promise,completed]{
        return run(events,"command",{{"command",[&entered_promise,completed]{
            entered_promise.set_value();
            if(completed.wait_for(2s)!=std::future_status::ready)
                throw std::runtime_error("recharge_result graph blocked behind command");
        }}},{51,151},91);
    });
    bounded(entered,"command did not enter its Feature");
    auto callback=std::async(std::launch::async,[events,&completed_promise]{
        return run(events,"recharge_result",{{"recharge_publish",[&completed_promise]{completed_promise.set_value();}}},{52,152},92);
    });
    const auto callback_result=bounded(callback,"independent recharge_result could not complete");
    const auto command_result=bounded(command,"command wait did not settle");
    check(callback_result.code==0&&command_result.code==0,"Command and recharge callback graphs are not independent");
    check(callback_result.status.frame.id==52&&command_result.status.frame.id==51,"Independent event identities crossed");

    // Real concurrent runs include repeated runs of the same callback type.
    // The published metrics must preserve each run's frame, parent and event
    // even while its scheduler is queuing behind a prior same-type invocation.
    struct Identity {std::uint64_t parent,timestamp;std::string event;bool fail;};
    std::map<std::uint64_t,Identity> expected;
    std::vector<std::future<Result>> pending;
    std::promise<void> start_promise;const auto start=start_promise.get_future().share();
    std::atomic<int> executed{0};
    const std::vector<std::pair<std::string,std::string>> kinds{
        {"command","command"},{"camera_params","params"},{"record_io","record_io"},
        {"recharge_result","recharge_publish"},{"offline_read","offline_read"}};
    for(int group=0;group<6;++group)for(const auto& kind:kinds) {
        const std::uint64_t id=1000+expected.size(),parent=7000+id,timestamp=900000+id;
        const bool fail=group%3==1;
        expected.emplace(id,Identity{parent,timestamp,kind.first,fail});
        pending.push_back(std::async(std::launch::async,[events,start,&executed,id,parent,timestamp,kind,fail]{
            start.wait();Events::Bindings binding;
            binding[kind.second]=[&executed,id,fail]{
                ++executed;std::this_thread::sleep_for(1ms);
                if(fail)throw r28::Failure({2003,-int(id),"","","concurrent-frame-"+std::to_string(id),{}});
            };
            return run(events,kind.first,binding,{id,timestamp},parent);
        }));
    }
    start_promise.set_value();
    for(std::size_t index=0;index<pending.size();++index) {
        const auto concurrent=bounded(pending[index],"concurrent event invocation did not settle");
        const auto id=1000+index;const auto& identity=expected.at(id);
        check(concurrent.code==(identity.fail?2003U:0U)&&concurrent.status.frame.id==id&&
              concurrent.status.frame.timestamp_ns==identity.timestamp,"Concurrent event returned another frame's status");
        if(identity.fail)check(concurrent.status.native_code==-int(id)&&
            concurrent.status.detail=="concurrent-frame-"+std::to_string(id),"Concurrent typed failure context crossed invocations");
    }
    check(executed==int(expected.size()),"Concurrent event binding duplicated or lost");pending.clear();

    for(const auto& invalid:std::vector<std::pair<std::string,Events::Bindings>>{
            {"command",{}},{"record_io",{{"record_io",{}}}},
            {"camera_params",{{"params_save",[]{}}}},{"unknown_event",{{"command",[]{}}}},
            {"command",{{"params",[]{}}}}}) {
        result=run(events,invalid.first,invalid.second,frame,99);
        check(result.code==2001&&result.report.tasks.empty()&&result.status.frame.id==42,
              "Unknown/empty/missing event binding accepted or identity lost");
    }
    result=run(events,"command",{{"command",[]{throw r28::Failure({0xF1234567U,-2147483000,"","","full32bit",{}});}}},frame,100);
    check(result.code==0xF1234567U&&result.status.native_code==-2147483000,"Fulluint32/native code was truncated");
    events->close();events->close();
    result=run(events,"command",{{"command",[]{throw std::runtime_error("closed graph executed");}}},frame,101);
    check(result.code==2001&&result.report.tasks.empty(),"Closed product event runner accepted work");events.reset();

    std::set<std::uint64_t> metric_event_ids,status_event_ids,seen_frames;
    std::map<std::uint64_t,std::uint64_t> frame_event_ids;
    std::istringstream csv(read(metrics/"product_event_metrics.csv"));std::string line;
    check(bool(std::getline(csv,line))&&line.find("event_id,parent_run_id,event,frame_id,timestamp_ns,")==0,
          "Event metric correlation header missing");
    while(std::getline(csv,line)) {
        const auto cells=columns(line);check(cells.size()==13,"Event metrics malformed");
        const auto event_id=std::stoull(cells[0]),id=std::stoull(cells[3]);metric_event_ids.insert(event_id);
        if(!expected.count(id))continue;const auto& identity=expected.at(id);
        check(std::stoull(cells[1])==identity.parent&&cells[2]==identity.event&&
              std::stoull(cells[4])==identity.timestamp&&std::stoul(cells[9])==(identity.fail?2003U:0U),
              "Concurrent metric frame/event/parent identity crossed");
        const auto inserted=frame_event_ids.emplace(id,event_id);
        check(inserted.second||inserted.first->second==event_id,"One frame produced inconsistent event IDs");seen_frames.insert(id);
    }
    check(seen_frames.size()==expected.size(),"Concurrent metric frame missing");
    std::istringstream statuses(read(metrics/"product_event_status.jsonl"));
    while(std::getline(statuses,line)) {
        const auto event_id=integer(line,"event_id"),id=integer(line,"frame_id");
        check(status_event_ids.insert(event_id).second,"Event ID reused across invocations");
        if(!expected.count(id))continue;const auto& identity=expected.at(id);
        check(integer(line,"parent_run_id")==identity.parent&&integer(line,"timestamp_ns")==identity.timestamp&&
              event_id==frame_event_ids.at(id)&&line.find("\"event\":\""+identity.event+"\"")!=std::string::npos,
              "Status JSON frame/event/parent correlation crossed");
    }
    check(metric_event_ids==status_event_ids,"Event metrics and status IDs disagree");
    const auto valid=read(config_path);
    const std::vector<std::string> invalid_configs{
        replaceOnce(valid,"\"worker_threads\": 1","\"worker_threads\": 2"),
        replaceOnce(valid,"{\"sequence\": [\"params\", \"params_save\"]}","{\"parallel\": [\"params\", \"params_save\"]}"),
        replaceOnce(valid,"\"dag\": {\"parallel\": [","\"dag\": {\"sequence\": ["),
        replaceOnce(valid,"product.output.record_io","product.output.unknown"),
        replaceOnce(valid,"\"record_io\": {\"features\"","\"record_io\": {\"enabled\": false, \"features\"")};
    for(std::size_t index=0;index<invalid_configs.size();++index) {
        const auto path=temp.root/("invalid_"+std::to_string(index)+".json");write(path,invalid_configs[index]);
        bool refused=false;try {Events bad(path.string());}catch(const std::exception&){refused=true;}
        check(refused,"Invalid/cross-callback/unordered event graph accepted");
    }
    std::cout<<"Actual ProductEvents TaskGraph: params/save dependency and failure skip, full business/native codes, real record I/O, "
        "independent command/recharge callback,30 concurrent frame/parent correlations, invalid bindings/configs and close passed.\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
