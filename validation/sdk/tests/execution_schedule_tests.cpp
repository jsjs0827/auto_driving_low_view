#include "percep_demo/execution_schedule.hpp"
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <vector>
static void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
int main(){try{
    r21::PersistentPair workers;
    std::thread::id ids[2];
    for(int frame=0;frame<300;++frame){
        const int first=frame%2;
        r21::FrameGate gate(first);
        std::atomic<int> active{0},calls{0};int output[2]{-1,-1};
        const auto job=[&](int lane){double wait=0;
            if(frame==0)ids[lane]=std::this_thread::get_id();
            else require(ids[lane]==std::this_thread::get_id(),"Workers were recreated");
            const bool ok=gate.run(lane,[&]{
                require(active.fetch_add(1)==0,"Ordered BPU tasks overlap");
                require(calls.fetch_add(1)==(lane==first?0:1),"Wrong BPU order");
                output[lane]=frame*2+lane;active.fetch_sub(1);return true;
            },wait);gate.branchFinished(lane,ok);require(ok,"Task failed");
        };
        workers.run([&]{job(0);},[&]{job(1);});
        require(output[0]==frame*2 && output[1]==frame*2+1,"Wrong frame/result ownership");
    }
    for(int first=0;first<2;++first){
        for(bool throwing:{false,true}){
            r21::FrameGate gate(first);std::atomic<int> submitted{0};
            const auto job=[&](int lane){
                if(lane==first){gate.branchFinished(lane,false);if(throwing)throw std::runtime_error("prepare failed");return;}
                double wait=0;require(!gate.run(lane,[&]{++submitted;return true;},wait),"Aborted lane was submitted");
            };
            bool threw=false;try{workers.run([&]{job(0);},[&]{job(1);});}catch(const std::runtime_error&){threw=true;}
            require(threw==throwing && submitted==0,"Failure did not drain both workers");
        }
    }
    {r21::FrameGate gate(0);bool second=false;bool threw=false;
        try{workers.run([&]{double t;gate.run(0,[]{throw std::runtime_error("submit failed");return true;},t);},[&]{double t;second=gate.run(1,[]{return true;},t);});}catch(...){threw=true;}
        require(threw && !second,"BPU exception left another task running");}
    // Free mode permits concurrency; use a rendezvous instead of timing assumptions.
    {r21::FrameGate gate(-1);std::mutex m;std::condition_variable cv;int started=0;
        const auto job=[&](int lane){double t;require(gate.run(lane,[&]{std::unique_lock<std::mutex> lock(m);++started;cv.notify_all();cv.wait(lock,[&]{return started==2;});return true;},t),"Free task failed");};
        workers.run([&]{job(0);},[&]{job(1);});}
    const int old=std::fegetround();
    for(int rounding:{FE_DOWNWARD,FE_UPWARD,FE_TONEAREST}){
        require(std::fesetround(rounding)==0,"Cannot set FP rounding");int actual[2]{};
        workers.run([&]{actual[0]=std::fegetround();},[&]{actual[1]=std::fegetround();});
        require(actual[0]==rounding && actual[1]==rounding,"FP environment changed");
    }
    std::fesetround(old);
    std::cout<<"300 frames: ordered single-frame BPU, persistent workers, correct ownership; free overlap, exception/prepare abort, FP inheritance passed\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
