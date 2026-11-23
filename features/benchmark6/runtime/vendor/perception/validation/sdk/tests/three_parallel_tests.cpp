#include "dag_pipeline.hpp"
#include "original_input_contract.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
void require(bool value,const char* message) {
    if(!value)throw std::runtime_error(message);
}
using Clock=std::chrono::steady_clock;

// All three branches must be executing before any infer/depth call may return.
// A serialized or two-worker implementation fails within two seconds instead
// of hanging the test. The recorded real thread IDs also prove independence.
class ThreeWayGate {
    std::mutex mutex_;
    std::condition_variable cv_;
    int arrived_=0;
    bool broken_=false;
    std::array<bool,3> seen_{};
    std::array<std::thread::id,3> threads_{};
public:
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        arrived_=0;broken_=false;seen_={};threads_={};
    }
    void arrive(int branch) {
        std::unique_lock<std::mutex> lock(mutex_);
        if(branch<0 || branch>2 || seen_[branch])throw std::runtime_error("Duplicate concurrency barrier branch");
        seen_[branch]=true;threads_[branch]=std::this_thread::get_id();++arrived_;
        if(arrived_==3)cv_.notify_all();
        if(!cv_.wait_for(lock,std::chrono::seconds(2),[&]{return arrived_==3 || broken_;})) {
            broken_=true;cv_.notify_all();
            throw std::runtime_error("Three-branch concurrency barrier timed out");
        }
        if(broken_)throw std::runtime_error("Three-branch concurrency barrier was broken");
    }
    std::array<std::thread::id,3> threads() {
        std::lock_guard<std::mutex> lock(mutex_);
        require(arrived_==3 && !broken_,"Not all three calls overlapped");
        return threads_;
    }
};

struct MockStages final:r25::Stages {
    ThreeWayGate& gate;
    std::array<std::atomic<int>,2> phase{};
    std::array<const std::uint8_t*,2> inputs{};
    std::atomic<unsigned> active_infers{0},aborts{0},closed{0};
    std::atomic<std::uint64_t> expected_id{0};
    std::atomic<unsigned> det_posts{0},seg_posts{0};
    explicit MockStages(ThreeWayGate& source):gate(source) {}
    void preprocess(int lane,const perception::ImageView& image) override {
        r26::validateInput(image);
        require(phase[lane].fetch_add(1)==0,"Previous frame/lane was still active");
        require(image.data[0]==std::uint8_t(expected_id.load()),"Wrong source frame reached preprocessing");
        inputs[lane]=image.data;
    }
    void infer(int lane,ai_node_percep_demo::DnnInferTiming&) override {
        require(phase[lane].fetch_add(1)==1,"Backend executed before preprocessing");
        struct Active {
            std::atomic<unsigned>& count;
            explicit Active(std::atomic<unsigned>& value):count(value){++count;}
            ~Active(){--count;}
        } active(active_infers);
        gate.arrive(lane);
        std::this_thread::sleep_for(std::chrono::milliseconds(lane?8:6));
    }
    std::vector<ai_node_percep_demo::DnnDetection> detection() override {
        require(phase[0].exchange(0)==2,"Detection postprocess phase was invalid");++det_posts;
        const float x=static_cast<float>(expected_id.load());
        return {{18,.75f,x,2,x+3,4,"charger"}};
    }
    std::vector<std::uint8_t> segmentation() override {
        require(phase[1].exchange(0)==2,"Segmentation postprocess phase was invalid");++seg_posts;
        return std::vector<std::uint8_t>(640*544,std::uint8_t(expected_id.load()%3));
    }
    int width(int lane) const override {return lane?832:864;}
    int height(int) const override {return 480;}
    void abortFrame() noexcept override {++aborts;phase[0]=0;phase[1]=0;}
    void close() override {++closed;}
};

enum class DepthBehavior { Success, Throw, WrongId, FailCode, InvalidGeometry };

void verifySuccessfulFrame(const r25::Output& output,std::uint64_t id,std::uint64_t timestamp,
                           MockStages& mock,ThreeWayGate& gate) {
    require(output.frame.ok() && output.frame.frame.id==id && output.frame.frame.timestamp_ns==timestamp,
            "Frame identity or timestamp changed");
    require(output.frame.detection && output.frame.segmentation,"Final typed AI artifacts missing");
    require(output.det.size()==1 && output.det[0].class_id==18 && output.det[0].class_name=="charger" &&
            output.det[0].x1==static_cast<float>(id),"Detection result belongs to another frame");
    require(output.mask().size()==640*544 &&
            std::all_of(output.mask().begin(),output.mask().end(),[&](std::uint8_t pixel){return pixel==id%3;}),
            "Segmentation snapshot is not the current frame result");
    require(output.depth.requested && output.depth.frame_id==id && output.depth.code==0 &&
            output.depth.width==640 && output.depth.height==544 && output.depth.pixels==640*544,
            "Depth result did not preserve the requested frame/cache key");
    require(output.frame.artifacts->keys().size()==4,"Borrowed/transient input artifacts retained");
    require(mock.inputs[0]==mock.inputs[1],"Detection and segmentation used different source storage");
    require(mock.phase[0]==0 && mock.phase[1]==0 && mock.active_infers==0,"Frame returned before AI lanes completed");

    const auto threads=gate.threads();
    const std::set<std::thread::id> independent(threads.begin(),threads.end());
    require(independent.size()==3,"Inference and depth calls did not use three independent workers");
    for(int branch=0;branch<3;++branch) {
        const int feature=branch==0?2:branch==1?5:7;
        require(output.features[feature].thread==std::hash<std::thread::id>{}(threads[branch]),
                "Feature trace thread does not match executing call");
    }
    for(const auto& feature:output.features) {
        require(feature.succeeded && feature.end_ms>=feature.start_ms,"Incomplete or negative Feature trace");
        require(feature.end_ms<=output.graph_ms+0.001,"Graph completion omitted a branch");
    }
    const auto& detection=output.features[2];const auto& segmentation=output.features[5];const auto& depth=output.features[7];
    require(std::max({detection.start_ms,segmentation.start_ms,depth.start_ms})<
            std::min({detection.end_ms,segmentation.end_ms,depth.end_ms}),
            "Feature traces do not show a three-way overlap");
    const double ai_finished=std::max(output.features[3].end_ms,output.features[6].end_ms);
    require(output.wall_ms>=0 && output.wall_ms<=output.graph_ms+0.001,"AI wall scope is negative or exceeds full graph");
    require(std::abs(output.wall_ms-ai_finished)<0.000001,
            "AI wall must use its completion endpoint, not subtract overlapping depth time");
    require(output.graph_ms>=depth.end_ms && output.graph_ms>=ai_finished,"Full graph did not include every branch");
    require(output.frame.schedule.tasks.size()==4,"Three-parallel graph task count changed");
    require(output.features[2].task=="detection" && output.features[5].task=="segmentation" &&
            output.features[7].task=="disparity","Features were not in independent DAG tasks");
}
}

int main(int argc,char** argv) {
    try {
        require(argc==2,"Profiles directory required");
        ThreeWayGate gate;auto mock=std::make_shared<MockStages>(gate);
        {
            r25::Pipeline pipeline(std::string(argv[1])+"/dag_exact_depth_parallel.json",mock);
            std::vector<std::uint8_t> raw(640*544*3/2,127);
            perception::ImageView image{raw.data(),raw.size(),640,640,544,perception::PixelFormat::NV12};
            r25::RunOptions options;options.optimized=false;options.depth_parallel=true;
            std::atomic<unsigned> depth_calls{0},depth_active{0};
            std::atomic<DepthBehavior> behavior{DepthBehavior::Success};
            options.depth=[&](std::uint64_t id) {
                struct Active {
                    std::atomic<unsigned>& count;
                    explicit Active(std::atomic<unsigned>& value):count(value){++count;}
                    ~Active(){--count;}
                } active(depth_active);
                require(id==mock->expected_id.load(),"Depth callback received another frame ID");
                ++depth_calls;gate.arrive(2);
                if(behavior.load()==DepthBehavior::Throw)throw std::runtime_error("depth failure sentinel");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                r25::DepthResult result{true,id+(behavior.load()==DepthBehavior::WrongId),0,640,544,640*544,20,0};
                if(behavior.load()==DepthBehavior::FailCode)result.code=-2;
                if(behavior.load()==DepthBehavior::InvalidGeometry)result.pixels=1;
                return result;
            };
            auto run=[&](std::uint64_t id) {
                gate.reset();mock->expected_id=id;raw[0]=std::uint8_t(id);
                return pipeline.run(image,{id,id+1000},options);
            };

            auto first=run(0);verifySuccessfulFrame(first,0,1000,*mock,gate);
            require(depth_calls==1 && depth_active==0,"Depth call missing or still active after graph return");
            const auto retained=first.frame;const auto first_mask=first.mask();
            auto second=run(7);verifySuccessfulFrame(second,7,1007,*mock,gate);
            require(depth_calls==2 && retained.frame.id==0 && retained.segmentation->mask==first_mask &&
                    retained.detection->detections[0].x1==0,"Previous immutable frame result was overwritten");

            for(auto failure:{DepthBehavior::Throw,DepthBehavior::WrongId,DepthBehavior::FailCode,DepthBehavior::InvalidGeometry}) {
                behavior=failure;bool rejected=false;
                try{(void)run(9);}catch(const std::exception& e) {
                    const auto error=std::string(e.what());
                    const char* wanted=failure==DepthBehavior::Throw?"depth failure sentinel":
                        failure==DepthBehavior::WrongId?"Depth frame identity mismatch":
                        failure==DepthBehavior::FailCode?"Requested disparity failed, code -2":"Requested disparity geometry invalid";
                    rejected=error.find("DAG failed")!=std::string::npos && error.find(wanted)!=std::string::npos;
                }
                require(rejected,"Depth error was not propagated through the DAG");
                require(mock->phase[0]==0 && mock->phase[1]==0 && mock->active_infers==0 && depth_active==0,
                        "Failed depth left AI/depth work active or lane phases stale");
                require(mock->aborts>0,"Depth failure did not abort/reset native stages");
                behavior=DepthBehavior::Success;
                auto recovered=run(12);verifySuccessfulFrame(recovered,12,1012,*mock,gate);
                require(retained.segmentation->mask==first_mask,"Failure/recovery changed retained prior result");
            }
            require(mock->closed==0,"Stages closed during normal frame processing");
        }
        require(mock->closed==1,"Pipeline did not close stage ownership exactly once");
        std::cout<<"PASS three same-frame DAG branches: barrier overlap, independent workers, identity, immutable results, depth abort/recovery and AI/full-graph timing scopes\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
