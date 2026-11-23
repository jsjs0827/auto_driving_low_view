#include "full_graph.hpp"
#include "json.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
namespace rt=perception::runtime;
using Graph=r33::full::Graph;
void require(bool good,const char* detail){if(!good)throw std::runtime_error(detail);}
rt::TaskState state(const rt::GraphReport& report,const std::string& id){
    const auto found=std::find_if(report.tasks.begin(),report.tasks.end(),[&](const rt::TaskReport& task){return task.id==id;});
    require(found!=report.tasks.end(),"Required task absent from actual report");return found->state;
}
void success(const rt::GraphReport& report,std::size_t count){
    require(report.ok()&&report.tasks.size()==count,"Actual graph report is not complete");
    for(const auto& task:report.tasks)require(task.state==rt::TaskState::Succeeded,"Enabled native Feature did not succeed");
}
}
int main(){try{
    Graph graph;rt::GraphReport report;r28::Status status;
    std::mutex mutex;std::condition_variable ready;std::set<std::string> completed;
    bool dnn_entered=false,disparity_entered=false;std::thread::id dnn_thread,disparity_thread;
    Graph::Bindings all;
    const std::vector<std::string> sequence={"dnn","recharge","rgb","cloud","calibration","labels","transform","ground","maps","publish","record"};
    const auto enter=[&](const std::string& key){
        std::unique_lock<std::mutex> lock(mutex);
        if(key=="dnn"||key=="disparity"){
            if(key=="dnn"){dnn_entered=true;dnn_thread=std::this_thread::get_id();}
            else{disparity_entered=true;disparity_thread=std::this_thread::get_id();}
            ready.notify_all();
            require(ready.wait_for(lock,std::chrono::seconds(4),[&]{return dnn_entered&&disparity_entered;}),"Independent disparity and DNN tasks did not execute concurrently");
        }
        const auto found=std::find(sequence.begin(),sequence.end(),key);
        if(found!=sequence.end()&&found!=sequence.begin())require(completed.count(*(found-1))==1,"Original dependent Feature order changed");
        if(key=="cloud")require(completed.count("disparity")==1,"Point cloud ran before disparity validation completed");
        completed.insert(key);
    };
    for(const auto& name:sequence)all[name]=[&,name]{enter(name);};all["disparity"]=[&]{enter("disparity");};
    require(graph.run(all,{101,1001},report,status)==0&&status.code==0,"Complete native graph failed");
    success(report,12);require(dnn_thread!=disparity_thread,"Parallel roots executed on one worker");
    const auto trace=r33::json::JsonParser(r33::full::traceJson(report,7),"graph trace").parse();
    require(trace.object.at("run_id").text=="7"&&trace.object.at("tasks").array.size()==12,"Trace lost task/run correlation");

    // AI-only overlay output still consumes the RGB Feature's owned buffer.
    // Without map/recharge tasks, publication must still wait for RGB.
    std::promise<void> rgb_started_promise,rgb_release_promise,publish_started_promise;
    auto rgb_started_future=rgb_started_promise.get_future();
    auto rgb_release_future=rgb_release_promise.get_future().share();
    auto publish_started_future=publish_started_promise.get_future();
    std::atomic<bool> rgb_finished{false},overlay_published{false},overlay_recorded{false};
    Graph::Bindings overlay;
    overlay["dnn"]=[]{};
    overlay["rgb"]=[&]{
        rgb_started_promise.set_value();
        require(rgb_release_future.wait_for(std::chrono::seconds(4))==std::future_status::ready,"Controller did not release RGB Feature");
        rgb_finished.store(true,std::memory_order_release);
    };
    overlay["publish"]=[&]{
        publish_started_promise.set_value();
        require(rgb_finished.load(std::memory_order_acquire),"AI-only publication raced RGB buffer production");
        overlay_published.store(true,std::memory_order_release);
    };
    overlay["record"]=[&]{
        require(rgb_finished.load(std::memory_order_acquire)&&overlay_published.load(std::memory_order_acquire),"Recording read unfinished RGB/publication output");
        overlay_recorded.store(true,std::memory_order_release);
    };
    auto overlay_run=std::async(std::launch::async,[&]{return graph.run(overlay,{107,1007},report,status);});
    const bool rgb_started=rgb_started_future.wait_for(std::chrono::seconds(4))==std::future_status::ready;
    const bool publication_waited=publish_started_future.wait_for(std::chrono::milliseconds(100))==std::future_status::timeout;
    rgb_release_promise.set_value();
    const auto overlay_code=overlay_run.get();
    require(rgb_started&&publication_waited,"AI-only publication was not held behind the blocked RGB Feature");
    require(overlay_code==0&&overlay_recorded.load(std::memory_order_acquire),"AI-only overlay graph did not finish after RGB release");
    success(report,4);

    // Removing optional Features must preserve transitive dependencies through
    // those Features; a failed disparity cannot reach maps/publication/record.
    int published=0,recorded=0,cloud=0;
    Graph::Bindings reduced;
    reduced["disparity"]=[]{throw r28::Failure({1011,-902,"vendor.stereo",{},"Injected native rejection",{}});};
    reduced["cloud"]=[&]{++cloud;};reduced["publish"]=[&]{++published;};reduced["record"]=[&]{++recorded;};
    require(graph.run(reduced,{102,1002},report,status)==1011,"Full native failure code changed");
    require(status.native_code==-902&&status.frame.id==102&&status.frame.timestamp_ns==1002&&status.task=="disparity"&&status.feature=="vendor.stereo","Native error/task/Feature/frame metadata lost");
    require(cloud==0&&published==0&&recorded==0,"Dependent side effects ran after a failed frame");
    require(state(report,"disparity")==rt::TaskState::Failed&&state(report,"cloud")==rt::TaskState::Skipped&&state(report,"publish")==rt::TaskState::Skipped&&state(report,"record")==rt::TaskState::Skipped,"Transitive failures not represented in TaskGraph report");

    // Reuse the cached reduced graph with a new closure. It must invoke the
    // current frame's binding, clear the old failure and publish only once.
    std::uint64_t observed=0;const perception::FrameInfo current{103,1003};
    reduced["disparity"]=[&]{observed=current.id;};
    reduced["cloud"]=[&]{require(observed==103,"Cached graph invoked stale frame binding");++cloud;};
    reduced["publish"]=[&]{require(observed==current.id,"Publication saw stale frame");++published;};
    require(graph.run(reduced,current,report,status)==0&&status.native_code==0&&status.detail.empty(),"Failure poisoned the next cached graph run");
    success(report,4);require(cloud==1&&published==1&&recorded==1,"Recovered frame side effects missing or repeated");

    // A failure in the publication boundary must suppress recording too.
    reduced["publish"]=[]{throw std::runtime_error("Injected application publication failure");};
    require(graph.run(reduced,{104,1004},report,status)==2003&&status.frame.id==104&&status.task=="publish"&&status.feature=="product.output.application","Publication failure lost its business boundary");
    require(recorded==1&&state(report,"record")==rt::TaskState::Skipped,"Recording ran after failed publication");

    Graph::Bindings recharge;
    std::vector<std::string> calls;
    recharge["dnn"]=[&]{calls.push_back("dnn");};recharge["recharge"]=[&]{calls.push_back("recharge");};
    require(graph.run(recharge,{105,1005},report,status)==0,"Recharge-only native graph failed");
    success(report,2);require(calls==std::vector<std::string>({"dnn","recharge"}),"Recharge did not depend on same-frame detection");
    recharge["recharge"]=[]{throw 42;};
    require(graph.run(recharge,{106,1006},report,status)==2004&&status.frame.id==106&&status.task=="recharge"&&status.feature=="product.recharge_mark","Nonstandard Feature exception escaped without proper status");
    require(state(report,"recharge")==rt::TaskState::Failed,"Unknown exception absent from scheduler evidence");

    // Rejected invocations must not return the previous success/failure report.
    for(int mode=0;mode<3;++mode){Graph::Bindings invalid;
        if(mode==1)invalid["unknown"]=[&]{throw std::logic_error("Must not execute unknown binding");};
        if(mode==2)invalid["dnn"]={};
        require(graph.run(invalid,{std::uint64_t(110+mode),1010},report,status)==2001,"Invalid graph binding accepted");
        require(report.tasks.empty()&&report.elapsed_ms==0&&status.frame.id==std::uint64_t(110+mode),"Rejected frame retained stale graph report or identity");
    }
    Graph::Bindings final;
    final["dnn"]=[&]{observed=200;};final["publish"]=[&]{require(observed==200,"Invalid bindings leaked prior closure");};
    require(graph.run(final,{200,1200},report,status)==0,"Graph did not recover after invalid binding");success(report,2);
    std::cout<<"Full native TaskGraph: parallel roots, RGB publication barrier, dependency expansion, full errors, side-effect suppression, new-frame closure and trace reset passed. No camera, neural inference, mapping algorithm or recharge algorithm executed.\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
