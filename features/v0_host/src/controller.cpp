#include "perception_v0_host/controller.hpp"
#include <cstdio>
#include <utility>

namespace perception::v0_host {
uint32_t BoardController::onCommand(int command,psdk_status& status) {
    std::lock_guard<std::mutex> execution(execution_mutex_);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || (command!=1 && command!=2 && command!=21)) {
            status=ros_so_demo::newStatus();status.code=1002;
            std::snprintf(status.detail,sizeof(status.detail),"unsupported board command or stopped controller");
            return status.code;
        }
    }
    const auto rc=session_.command(command,status);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!rc) {input_.setCommand(command);status_=command==21?0:command;ready_.notify_all();}
    else status_=-1;
    return rc;
}
bool BoardController::setIntrinsics(const CameraParameters& p,std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);if (closed_) return false;
    return input_.setIntrinsics(p,error);
}
bool BoardController::onRgbd(Nv12Image left,Q8Disparity q8,std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);if (closed_) return false;
    const bool accepted=input_.onRgbd(std::move(left),std::move(q8),error);
    if (accepted) ready_.notify_one();
    return accepted;
}
bool BoardController::onGdc(Nv12Image left,std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);if (closed_) return false;
    const bool accepted=input_.onGdc(std::move(left),error);
    if (accepted) ready_.notify_one();
    return accepted;
}
bool BoardController::onStereo(Nv12Image left,Nv12Image right,std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);if (closed_) return false;
    const bool accepted=input_.onStereo(std::move(left),std::move(right),error);
    if (accepted) ready_.notify_one();
    return accepted;
}
bool BoardController::onFullFrame(psdk_full_frame frame,std::shared_ptr<const void> owner,std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);if (closed_) return false;
    const bool accepted=input_.onFullFrame(frame,std::move(owner),error);
    if (accepted) ready_.notify_one();
    return accepted;
}
void BoardController::setProcessBinding(std::function<void(const psdk_full_frame&)> before,
                                       std::function<void()> after) {
    std::lock_guard<std::mutex> execution(execution_mutex_);
    before_process_=std::move(before);after_process_=std::move(after);
}
bool BoardController::step(entry::Result& result,psdk_status& status,const Publication& publish) {
    std::lock_guard<std::mutex> execution(execution_mutex_);
    std::optional<InputFrame> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);if (closed_) return false;
        pending=input_.take();if (!pending) return false;
    }
    psdk_frame hires{};psdk_disparity_q8 q8{};const auto frame=pending->view(hires,q8);
    struct Cleanup {const std::function<void()>& call;~Cleanup(){if(call)call();}} cleanup{after_process_};
    try {
        if (before_process_) before_process_(frame);
        auto rc=session_.process(frame,result,status);
        if (!rc) {
            psdk_full_result_view view{};rc=result.view(view,status);
            if (!rc && publish) publish(view);
        }
        std::lock_guard<std::mutex> lock(mutex_);status_=rc?-1:pending->command;return true;
    } catch (...) {std::lock_guard<std::mutex> lock(mutex_);status_=-1;throw;}
}
bool BoardController::waitForWork(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait_for(lock,timeout,[this]{return closed_ || input_.hasPending();});
    return !closed_ && input_.hasPending();
}
int BoardController::status() const {std::lock_guard<std::mutex> lock(mutex_);return status_;}
void BoardController::shutdown() {
    std::lock_guard<std::mutex> execution(execution_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);closed_=true;input_.clear();ready_.notify_all();
}
} // namespace perception::v0_host
