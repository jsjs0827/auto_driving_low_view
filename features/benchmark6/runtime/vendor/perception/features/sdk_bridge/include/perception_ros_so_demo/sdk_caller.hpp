#pragma once
#include "perception_full_sdk.h"
#include "perception_slip_sdk.h"

namespace ros_so_demo {
psdk_status newStatus();

// Owns one SDK result; view pointers expire when this owner resets or is destroyed.
class Result {
    psdk_full_result* result_=nullptr;
public:
    Result()=default;
    ~Result();
    Result(const Result&)=delete;
    Result& operator=(const Result&)=delete;
    Result(Result&& other) noexcept;
    Result& operator=(Result&& other) noexcept;
    void reset();
    psdk_full_result** output();
    psdk_full_result* get() const;
    uint32_t view(psdk_full_result_view& value,psdk_status& status) const;
};

// A synchronous bridge: acquisition, conversion, and scheduling belong to callers.
class Caller {
    psdk_full_engine* engine_=nullptr;
public:
    Caller()=default;
    ~Caller();
    Caller(const Caller&)=delete;
    Caller& operator=(const Caller&)=delete;
    uint32_t open(const char* json,psdk_status& status,
                  const char* detection_model=nullptr,const char* segmentation_model=nullptr,
                  const psdk_full_hooks* hooks=nullptr);
    uint32_t command(int command,psdk_status& status);
    uint32_t process(const psdk_full_frame& frame,Result& result,psdk_status& status);
    uint32_t pushMotion(const psdk_motion_sample* samples,uint32_t count,psdk_status& status);
    uint32_t close(psdk_status& status);
};
} // namespace ros_so_demo
