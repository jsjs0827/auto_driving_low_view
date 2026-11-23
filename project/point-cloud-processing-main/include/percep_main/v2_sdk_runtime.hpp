#pragma once

#include "perception_full_sdk.h"

#include <memory>
#include <string>

namespace perception {

struct V2ResultRelease {
    void operator()(psdk_full_result* result) const { psdk_full_result_release(result); }
};
using V2Result = std::unique_ptr<psdk_full_result, V2ResultRelease>;

class V2SdkRuntime {
public:
    V2SdkRuntime() = default;
    ~V2SdkRuntime();
    V2SdkRuntime(const V2SdkRuntime&) = delete;
    V2SdkRuntime& operator=(const V2SdkRuntime&) = delete;

    bool open(const std::string& path, std::string& error);
    bool command(int mode, std::string& error);
    V2Result process(const psdk_full_frame& frame, psdk_full_result_view& view, std::string& error);

private:
    psdk_full_engine* engine_ = nullptr;
};

inline psdk_frame v2Nv12(const uint8_t* data, uint64_t bytes, uint32_t width, uint32_t height,
                         uint32_t stride, uint64_t id, uint64_t timestamp)
{
    return {sizeof(psdk_frame), PSDK_ABI_VERSION, PSDK_FORMAT_NV12, width, height, stride,
            data, bytes, id, timestamp};
}

inline psdk_full_frame v2Frame(const psdk_frame& left, const psdk_camera_intrinsics& intrinsics)
{
    psdk_full_frame frame{};
    frame.struct_size = sizeof(frame);
    frame.abi_version = PSDK_ABI_VERSION;
    frame.left = left;
    frame.intrinsics = intrinsics;
    return frame;
}

}  // namespace perception
