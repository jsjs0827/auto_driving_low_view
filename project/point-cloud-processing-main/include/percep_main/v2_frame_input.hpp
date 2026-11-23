#pragma once

#include "percep_main/v2_sdk_runtime.hpp"
#include "shm_struct/camera_struct.h"

#include <cmath>

namespace perception {

inline bool makeV2Nv12(const PicData& meta, const uint8_t* data, size_t capacity,
                       psdk_frame& image)
{
    const uint32_t rows = uint32_t(meta.height) * 3 / 2;
    if (!data || meta.bits_per_pixel != 8 || !meta.width || !meta.height
        || (meta.width & 1) || (meta.height & 1) || !meta.time_ns
        || meta.bytes_used > capacity || meta.bytes_used % rows) return false;
    const uint32_t stride = meta.bytes_used / rows;
    if ((stride & 1) || stride < meta.width) return false;
    image = v2Nv12(data, meta.bytes_used, meta.width, meta.height, stride, meta.frame_id, meta.time_ns);
    return true;
}

inline bool makeV2MapInput(const CameraData& packet, const psdk_camera_intrinsics& k,
                           psdk_disparity_q8& disparity, psdk_full_frame& frame)
{
    psdk_frame left{};
    if (!makeV2Nv12(packet.left, packet.left_data, sizeof(packet.left_data), left)) return false;
    const auto& disp = packet.disp;
    if (disp.width != left.width || disp.height != left.height || disp.bits_per_pixel != 16
        || disp.bytes_used > sizeof(packet.disp_data) || disp.bytes_used % (2u * disp.height)
        || disp.bytes_used / disp.height < 2u * disp.width
        || disp.frame_id != left.frame_id || disp.time_ns != left.timestamp_ns
        || reinterpret_cast<uintptr_t>(packet.disp_data) % alignof(uint16_t)) return false;
    if (k.width != left.width || k.height != left.height || !std::isfinite(k.fx)
        || !std::isfinite(k.fy) || !std::isfinite(k.cx) || !std::isfinite(k.cy)
        || !std::isfinite(k.bxf) || k.fx <= 0 || k.fy <= 0 || k.bxf <= 0) return false;
    disparity = {sizeof(disparity), PSDK_ABI_VERSION, disp.width, disp.height,
                 disp.bytes_used / disp.height, 0, packet.disp_data, disp.bytes_used,
                 disp.frame_id, disp.time_ns};
    frame = v2Frame(left, k);
    frame.disparity = &disparity;
    return true;
}

inline bool makeV2RechargeInput(const CameraGdcData& packet, const psdk_camera_intrinsics& k,
                                psdk_frame& hires, psdk_full_frame& frame)
{
    if (packet.left.width != 1280 || packet.left.height != 1088
        || !makeV2Nv12(packet.left, packet.left_data, sizeof(packet.left_data), hires)) return false;
    frame = v2Frame(hires, k);
    frame.recharge_hires = &hires;
    return true;
}

struct V2DockValues { float x, y, yaw; };

inline V2DockValues v2DockValues(const psdk_dock_pose& pose)
{
    if (!pose.pose_ok) return {999.f, 999.f, 999.f};
    return {pose.x_m, pose.y_m, pose.yaw_rad};
}

}  // namespace perception
