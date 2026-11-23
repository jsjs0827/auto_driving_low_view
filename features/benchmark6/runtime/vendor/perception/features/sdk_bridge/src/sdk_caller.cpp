#include "perception_ros_so_demo/sdk_caller.hpp"
#include <cstring>
#include <utility>

namespace ros_so_demo {
psdk_status newStatus() {
    psdk_status value{};value.struct_size=sizeof(value);value.abi_version=PSDK_ABI_VERSION;return value;
}
Result::~Result(){reset();}
Result::Result(Result&& other) noexcept:result_(std::exchange(other.result_,nullptr)){}
Result& Result::operator=(Result&& other) noexcept {
    if(this!=&other){reset();result_=std::exchange(other.result_,nullptr);}return *this;
}
void Result::reset(){psdk_full_result_release(std::exchange(result_,nullptr));}
psdk_full_result** Result::output(){reset();return &result_;}
psdk_full_result* Result::get() const{return result_;}
uint32_t Result::view(psdk_full_result_view& value,psdk_status& status) const {
    value={};value.struct_size=sizeof(value);value.abi_version=PSDK_FULL_ABI_VERSION;
    return psdk_full_result_get_view(result_,&value,&status);
}
Caller::~Caller(){auto status=newStatus();close(status);}
uint32_t Caller::open(const char* json,psdk_status& status,const char* detection_model,
                      const char* segmentation_model,const psdk_full_hooks* hooks) {
    if(engine_){status=newStatus();status.code=2001;std::strcpy(status.feature,"ros_demo.create");
        std::strcpy(status.detail,"Caller already initialized");return status.code;}
    psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;
    config.json_file=json;config.detection_model=detection_model;
    config.segmentation_model=segmentation_model;config.hooks=hooks;
    return psdk_full_create(&config,&engine_,&status);
}
uint32_t Caller::command(int command,psdk_status& status){return psdk_full_command(engine_,command,&status);}
uint32_t Caller::process(const psdk_full_frame& frame,Result& result,psdk_status& status) {
    return psdk_full_process(engine_,&frame,result.output(),&status);
}
uint32_t Caller::close(psdk_status& status){return psdk_full_destroy(&engine_,&status);}
uint32_t Caller::pushMotion(const psdk_motion_sample* samples,uint32_t count,psdk_status& status){return psdk_full_push_motion_samples(engine_,samples,count,&status);}
} // namespace ros_so_demo
