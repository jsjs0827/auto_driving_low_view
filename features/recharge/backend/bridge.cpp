#include "bridge.hpp"

#include "v0_runtime.hpp"
#include <rapidjson/document.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <optional>

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
    Aiper_Recharge::RechargeMarkApi* api = nullptr;
    void* lease = nullptr;
    bool initialized = false;
};

Engine::Engine() : impl_(std::make_unique<Impl>()) {}

Engine::~Engine() {
    std::lock_guard<std::mutex> lock(engine_mutex);
    if (impl_->lease) r36_recharge_v0_close(impl_->lease);
    impl_->lease = nullptr;
    impl_->api = nullptr;
    impl_->initialized = false;
    if (initialized_engine == this) initialized_engine = nullptr;
}

bool Engine::initialize(const char* config_path, std::string& error) {
    std::lock_guard<std::mutex> lock(engine_mutex);
    error.clear();
    if (initialized_engine && initialized_engine != this) {
        error = "selected original V0 SDK uses process-global state: only one recharge Engine may be initialized";
        return false;
    }
    try {
        // Original init silently ignores non-.json paths, so fail explicitly.
        if (!config_path || std::filesystem::path(config_path).extension() != ".json")
            throw std::invalid_argument("V0 recharge requires an explicit lowercase .json configuration path");
        std::ifstream config_file(config_path, std::ios::binary);
        if (!config_file) throw std::invalid_argument("Cannot read V0 recharge configuration");
        const std::string text((std::istreambuf_iterator<char>(config_file)), {});
        rapidjson::Document config;
        config.Parse(text.data(), text.size());
        if (config.HasParseError() || !config.IsObject())
            throw std::invalid_argument("Invalid V0 recharge configuration JSON object");
        const rapidjson::Value* object = &config;
        if (config.HasMember("recharge_mark") && config["recharge_mark"].IsObject())
            object = &config["recharge_mark"];
        // V2 adapter I/O policy only. Absence preserves original SDK behavior;
        // true restores vendor result saving, false disables only that writer.
        std::optional<bool> save_result;
        if (object->HasMember("save_result")) {
            const auto& block = (*object)["save_result"];
            if (!block.IsObject()) throw std::invalid_argument("save_result must be an object");
            if (block.HasMember("enabled")) {
                if (!block["enabled"].IsBool())
                    throw std::invalid_argument("save_result.enabled must be boolean");
                save_result = block["enabled"].GetBool();
            }
        }
        // Reinitialization first destroys the previous API instance. Original
        // destroy(false) clears its camera override and joins save/worker jobs.
        if (impl_->lease) {
            r36_recharge_v0_close(impl_->lease);
            impl_->lease = nullptr;
            impl_->api = nullptr;
            impl_->initialized = false;
        }
        if (!impl_->api) {
            char why[1024]{};
            if (r36_recharge_v0_open(&impl_->api, &impl_->lease, why, sizeof(why)) != 0)
                throw std::runtime_error(why[0] ? why : "Cannot open selected original V0 SDK");
        }
        // Mirror the real V0 core initialization, including its own OpenCV
        // initialization and the COMPLETE original parser/options handling.
        if (impl_->api->init("", config_path) != 0)
            throw std::runtime_error(std::string("Original V0 SDK configuration initialization failed: ") + config_path);
        if (save_result.has_value()) impl_->api->setSaveResultEnable(*save_result);
        impl_->api->resetSession();  // Once per initialization/session, never per frame.
        impl_->initialized = true;
        initialized_engine = this;
        return true;
    } catch (const std::exception& exception) {
        if (impl_->lease) r36_recharge_v0_close(impl_->lease);
        impl_->lease = nullptr;
        impl_->api = nullptr;
        impl_->initialized = false;
        if (initialized_engine == this) initialized_engine = nullptr;
        error = std::string("recharge V0 initialization exception: ") + exception.what();
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
        // Preserve the V2 contract: external crop hints are not part of this integration.
        if (crop) {error="external crop is unsupported by the V0 integration; configure roi in recharge_mark.json";return false;}
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
