#pragma once
#include <array>
#include <cfenv>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>

namespace r21 {
struct BpuTiming {
    double gate_ms=0,submit_ms=0,wait_ms=0,release_ms=0;
};
// Each gate belongs to one frame. Only BPU calls are ordered; CPU work remains parallel.
class FrameGate {
    int first_;
    std::mutex mutex_;
    std::condition_variable ready_;
    bool retired_[2]{false,false},used_[2]{false,false},aborted_=false;
public:
    explicit FrameGate(int first):first_(first){}
    bool run(int lane,const std::function<bool()>& task,double& gate_ms);
    void branchFinished(int lane,bool ok);
};
// One synchronous dispatch at a time; there is no frame queue or second input buffer.
class PersistentPair {
    std::array<std::thread,2> threads_;
    std::array<std::function<void()>,2> jobs_;
    std::array<std::exception_ptr,2> errors_;
    std::mutex mutex_,dispatch_;
    std::condition_variable start_,finished_;
    std::fenv_t environment_{};
    unsigned generation_=0;
    int remaining_=0;
    bool stopping_=false;
    void loop(int lane);
public:
    PersistentPair();
    ~PersistentPair();
    PersistentPair(const PersistentPair&)=delete;
    PersistentPair& operator=(const PersistentPair&)=delete;
    void run(std::function<void()> detection,std::function<void()> segmentation);
};
}
