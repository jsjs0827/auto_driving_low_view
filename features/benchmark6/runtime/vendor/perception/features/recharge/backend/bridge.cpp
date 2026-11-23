#include "bridge.hpp"

#include "backend_lifecycle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <exception>
#include <limits>
#include <mutex>

namespace r33::recharge {
namespace {

std::mutex engine_mutex;
const Engine* initialized_engine = nullptr;

std::int64_t frameStamp(std::int64_t timestamp_ns) {
    if (timestamp_ns > 0) return timestamp_ns;
#if defined(CLOCK_BOOTTIME) || defined(CLOCK_MONOTONIC)
    timespec stamp{};
#if defined(CLOCK_BOOTTIME)
    if (clock_gettime(CLOCK_BOOTTIME, &stamp) == 0) {
        return static_cast<std::int64_t>(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec;
    }
#endif
#if defined(CLOCK_MONOTONIC)
    if (clock_gettime(CLOCK_MONOTONIC, &stamp) == 0) {
        return static_cast<std::int64_t>(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec;
    }
#endif
#endif
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool validateNv12(const std::uint8_t* data, std::size_t bytes,
                  int width, int height, int stride, std::string& error) {
    if (!data || width <= 0 || height <= 0 || width > 4096 || height > 4096 ||
        (width & 1) || (height & 1) || stride < width) {
        error = "recharge requires non-null even NV12 dimensions <=4096 and stride>=width";
        return false;
    }
    const auto rows = static_cast<std::size_t>(height) * 3u / 2u;
    if (static_cast<std::size_t>(stride) > std::numeric_limits<std::size_t>::max() / rows ||
        bytes < static_cast<std::size_t>(stride) * rows) {
        error = "recharge NV12 buffer is smaller than stride*height*3/2";
        return false;
    }
    return true;
}

}  // namespace

struct Engine::Impl {
    std::unique_ptr<Aiper_Recharge::RechargeMarkApi> api;
    bool initialized = false;
};

Engine::Engine() : impl_(std::make_unique<Impl>()) {}

Engine::~Engine() {
    std::lock_guard<std::mutex> lock(engine_mutex);
    if (initialized_engine == this) {
        if (impl_->api) impl_->api->destroy(false);
        r35_recharge_log_shutdown();
        initialized_engine = nullptr;
    }
}

bool Engine::initialize(const char* config_path, std::string& error) {
    std::lock_guard<std::mutex> lock(engine_mutex);
    error.clear();
    if (initialized_engine && initialized_engine != this) {
        error = "charing-apriltag uses process-global state: only one recharge Engine may be initialized";
        return false;
    }
    try {
        if (!impl_->api) impl_->api.reset(Aiper_Recharge::createRechargeMarkApi());
        if (!impl_->api || impl_->api->loadConfig(config_path) != 0) {
            error = std::string("failed to load charing-apriltag configuration: ") + (config_path ? config_path : "<null>");
            return false;
        }
        impl_->api->useSdkCameraMatrix();  // Clears an earlier camera override.
        impl_->api->resetSession();
#if !defined(_WIN32)
        r35_recharge_log_init();
#endif
        impl_->initialized = true;
        initialized_engine = this;
        return true;
    } catch (const std::exception& exception) {
        error = std::string("recharge initialization exception: ") + exception.what();
        return false;
    }
}

void Engine::getHostConfig(Aiper_Recharge::RechargeHostConfig& out) const {
    std::lock_guard<std::mutex> lock(engine_mutex);
    out = Aiper_Recharge::RechargeHostConfig{};
    if (impl_->initialized && initialized_engine == this) impl_->api->getHostConfig(&out);
}

void Engine::resetSession() {
    std::lock_guard<std::mutex> lock(engine_mutex);
    if (impl_->initialized && initialized_engine == this) impl_->api->resetSession();
}

bool Engine::setCameraMatrix(double fx, double fy, double cx, double cy,
                             std::string& error) {
    std::lock_guard<std::mutex> lock(engine_mutex);
    error.clear();
    if (!impl_->initialized || initialized_engine != this) {
        error = "recharge Engine has not been initialized";
        return false;
    }
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
        !std::isfinite(cy) || fx <= 1.0 || fy <= 1.0) {
        error = "recharge camera intrinsics require finite fx/fy>1 and finite cx/cy";
        return false;
    }
    impl_->api->setCameraMatrix(fx, fy, cx, cy);
    return true;
}

bool Engine::processLeftNv12(const std::uint8_t* nv12, std::size_t bytes,
                             int width, int height, int stride_y,
                             std::uint64_t frame_id, std::int64_t timestamp_ns,
                             const CropHint* crop, Output& out, std::string& error) {
    std::lock_guard<std::mutex> lock(engine_mutex);
    error.clear();
    out = Output{};
    if (!impl_->initialized || initialized_engine != this) {
        error = "recharge Engine has not been initialized";
        return false;
    }
    if (!validateNv12(nv12, bytes, width, height, stride_y, error)) return false;
    try {
        // The replacement donor removed external YOLO crop hints.
        if (crop) {error="external crop is unsupported by charing-apriltag; configure roi in recharge_mark.json";return false;}
        const auto release = impl_->api->freeResult;
        std::unique_ptr<Aiper_Recharge::MarkPoseResult, decltype(release)> result(
            impl_->api->processLeftNv12(nv12, width, height, stride_y, frame_id), release);
        if (!result) {
            error = "upstream recharge returned a null result";
            return false;
        }
        result->post.time_stamp = frameStamp(timestamp_ns);
        out.pose = *result;
        out.pose.overlay_bgr = nullptr;
        if (result->overlay_bgr && result->overlay_step > 0 && result->overlay_h > 0) {
            const auto overlay_bytes = static_cast<std::size_t>(result->overlay_step) *
                                       static_cast<std::size_t>(result->overlay_h);
            out.overlay_bgr.assign(result->overlay_bgr, result->overlay_bgr + overlay_bytes);
        }
        return true;
    } catch (const std::exception& exception) {
        error = std::string("recharge processing exception: ") + exception.what();
        return false;
    } catch (...) {
        error = "unknown recharge processing exception";
        return false;
    }
}

}  // namespace r33::recharge
