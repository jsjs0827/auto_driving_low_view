#pragma once
#include "perception_v0_host/input.hpp"
#include "perception_entry/session.hpp"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>

namespace perception::v0_host {
class BoardController {
    entry::Session& session_;
    std::mutex execution_mutex_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    BoardInput input_;
    bool closed_=false;
    int status_=0;
    std::function<void(const psdk_full_frame&)> before_process_;
    std::function<void()> after_process_;
public:
    using Publication=std::function<void(const psdk_full_result_view&)>;
    explicit BoardController(entry::Session& session):session_(session) {}
    uint32_t onCommand(int command,psdk_status& status);
    bool setIntrinsics(const CameraParameters&,std::string& error);
    bool onRgbd(Nv12Image,Q8Disparity,std::string& error);
    bool onStereo(Nv12Image,Nv12Image,std::string& error);
    bool onGdc(Nv12Image,std::string& error);
    bool onFullFrame(psdk_full_frame,std::shared_ptr<const void>,std::string& error);
    // Install once before starting transport callbacks. StereoProvider's pair
    // registration executes under the same session lock as process/commands.
    void setProcessBinding(std::function<void(const psdk_full_frame&)> before,
                           std::function<void()> after);
    // Execution is serialized with commands; reception/status use a separate
    // short-held mutex. A stop response cannot precede an old publication.
    bool step(entry::Result& result,psdk_status& status,const Publication& publish={});
    bool waitForWork(std::chrono::milliseconds timeout);
    int status() const;
    void shutdown();
};
} // namespace perception::v0_host
