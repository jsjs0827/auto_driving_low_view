#include "percep_demo/execution_schedule.hpp"
#include <chrono>
#include <stdexcept>
#include <utility>
namespace r21 {
bool FrameGate::run(int lane,const std::function<bool()>& task,double& gate_ms) {
    const auto start=std::chrono::steady_clock::now();
    {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock,[&]{return aborted_ || first_<0 || lane==first_ || retired_[first_];});
        gate_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if(aborted_) return false;
        if(used_[lane]) throw std::logic_error("More than one BPU task for a lane/frame");
        used_[lane]=true;
    }
    bool ok=false;
    try {ok=task();} catch(...) {branchFinished(lane,false);throw;}
    {
        std::lock_guard<std::mutex> lock(mutex_);
        retired_[lane]=true;
        if(!ok) aborted_=true;
    }
    ready_.notify_all();
    return ok;
}
void FrameGate::branchFinished(int lane,bool ok) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        retired_[lane]=true;
        if(!ok) aborted_=true;
    }
    ready_.notify_all();
}
PersistentPair::PersistentPair() {
    try {for(int lane=0;lane<2;++lane) threads_[lane]=std::thread([this,lane]{loop(lane);});}
    catch(...) {
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
        start_.notify_all();for(auto& thread:threads_)if(thread.joinable())thread.join();throw;
    }
}
PersistentPair::~PersistentPair() {
    std::lock_guard<std::mutex> dispatch(dispatch_);
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
    start_.notify_all();for(auto& thread:threads_)if(thread.joinable())thread.join();
}
void PersistentPair::loop(int lane) {
    unsigned observed=0;
    for(;;) {
        std::function<void()> job;std::fenv_t environment;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            start_.wait(lock,[&]{return stopping_ || generation_!=observed;});
            if(stopping_)return;
            observed=generation_;job=std::move(jobs_[lane]);environment=environment_;
        }
        std::exception_ptr error;
        // Per-frame threads inherited the caller's FP environment. Preserve that contract.
        try {std::fesetenv(&environment);job();}catch(...){error=std::current_exception();}
        job={}; // Drop all frame references before reporting completion to the caller.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            errors_[lane]=error;--remaining_;
        }
        finished_.notify_one();
    }
}
void PersistentPair::run(std::function<void()> detection,std::function<void()> segmentation) {
    std::lock_guard<std::mutex> dispatch(dispatch_);
    std::unique_lock<std::mutex> lock(mutex_);
    if(stopping_)throw std::logic_error("Worker pair stopped");
    jobs_[0]=std::move(detection);jobs_[1]=std::move(segmentation);
    std::fegetenv(&environment_);errors_={};remaining_=2;++generation_;
    start_.notify_all();finished_.wait(lock,[&]{return remaining_==0;});
    auto errors=errors_;lock.unlock();
    for(const auto& error:errors)if(error)std::rethrow_exception(error);
}
}
