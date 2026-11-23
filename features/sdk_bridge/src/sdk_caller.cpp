#include "perception_ros_so_demo/sdk_caller.hpp"
#include <cstring>
#include <utility>
#include <cstdio>

namespace ros_so_demo {
psdk_status newStatus() {
    psdk_status value{};value.struct_size=sizeof(value);value.abi_version=PSDK_ABI_VERSION;return value;
}
namespace {
thread_local const Caller* callback_owner=nullptr;
struct CallbackScope {
    const Caller* previous=callback_owner;
    explicit CallbackScope(const Caller* value){callback_owner=value;}
    ~CallbackScope(){callback_owner=previous;}
};
bool rejectCallback(psdk_status& status) {
    if(!callback_owner)return false;
    status=newStatus();status.code=2001;
    std::snprintf(status.feature,sizeof(status.feature),"full.callback");
    std::snprintf(status.detail,sizeof(status.detail),"Callbacks must not reenter Session mutation methods");
    return true;
}
}
uint32_t Caller::disparityHook(void* opaque,const psdk_frame* frame,psdk_disparity_q8* output,psdk_status* status) {
    auto& self=*static_cast<Caller*>(opaque);CallbackScope scope(&self);
    return self.client_hooks_.disparity_provider(self.client_hooks_.userdata,frame,output,status);
}
uint32_t Caller::publicationHook(void* opaque,const psdk_full_result_view* view,psdk_status* status) {
    auto& self=*static_cast<Caller*>(opaque);CallbackScope scope(&self);
    return self.client_hooks_.publication(self.client_hooks_.userdata,view,status);
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
    if(rejectCallback(status))return status.code;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(engine_){status=newStatus();status.code=2001;std::strcpy(status.feature,"ros_demo.create");
        std::strcpy(status.detail,"Caller already initialized");return status.code;}
    psdk_full_config config{};config.struct_size=sizeof(config);config.abi_version=PSDK_FULL_ABI_VERSION;
    config.json_file=json;config.detection_model=detection_model;
    config.segmentation_model=segmentation_model;
    if(hooks) {
        if(hooks->struct_size!=sizeof(*hooks)||hooks->abi_version!=PSDK_FULL_ABI_VERSION) {
            status=newStatus();status.code=2001;
            std::snprintf(status.feature,sizeof(status.feature),"entry.create");
            std::snprintf(status.detail,sizeof(status.detail),"Invalid hooks ABI");return status.code;
        }
        client_hooks_=*hooks;sdk_hooks_={};sdk_hooks_.struct_size=sizeof(sdk_hooks_);
        sdk_hooks_.abi_version=PSDK_FULL_ABI_VERSION;sdk_hooks_.userdata=this;
        if(hooks->disparity_provider)sdk_hooks_.disparity_provider=&Caller::disparityHook;
        if(hooks->publication)sdk_hooks_.publication=&Caller::publicationHook;
        config.hooks=&sdk_hooks_;
    }
    const auto code=psdk_full_create(&config,&engine_,&status);
    if(!code){command_=-1;opened_=true;}
    return code;
}
uint32_t Caller::command(int command,psdk_status& status){
    if(rejectCallback(status))return status.code;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(engine_&&command_==command){status=newStatus();return 0;}
    return restartCommand(command,status);
}
uint32_t Caller::restartCommand(int command,psdk_status& status){
    if(rejectCallback(status))return status.code;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto code=psdk_full_command(engine_,command,&status);
    if(!code)command_=command;
    return code;
}
int Caller::currentCommand() const {return command_.load();}
bool Caller::isOpen() const {return opened_.load();}
uint32_t Caller::process(const psdk_full_frame& frame,Result& result,psdk_status& status) {
    if(rejectCallback(status))return status.code;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return psdk_full_process(engine_,&frame,result.output(),&status);
}
uint32_t Caller::close(psdk_status& status){
    if(rejectCallback(status))return status.code;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto code=psdk_full_destroy(&engine_,&status);if(!code){command_=-1;opened_=false;client_hooks_={};sdk_hooks_={};}return code;
}
uint32_t Caller::pushMotion(const psdk_motion_sample* samples,uint32_t count,psdk_status& status){
    if(rejectCallback(status))return status.code;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return psdk_full_push_motion_samples(engine_,samples,count,&status);
}
} // namespace ros_so_demo
