#ifndef AI_NODE_PERCEP_DEMO_CHARGER_CROP_HPP_
#define AI_NODE_PERCEP_DEMO_CHARGER_CROP_HPP_

#include "percep_demo/pre_pointcloud_types.hpp"

#include <algorithm>
#include <cmath>

namespace ai_node_percep_demo {

inline constexpr float kMaxChargerCropPadRatio = 4.0F;

// Convert a detector-space box to an even-aligned crop on the recharge image.
// All floating-point values are checked and clamped in double precision before
// conversion to int, preventing NaN/Inf or oversized configuration values from
// reaching floor/ceil integer conversions.
inline bool buildChargerCropOnImage(float x1, float y1, float x2, float y2,
                                    int det_w, int det_h, int mark_w,
                                    int mark_h, float pad_ratio, int pad_px,
                                    int min_w, int min_h, int* ox, int* oy,
                                    int* ow, int* oh) noexcept {
    if (!ox || !oy || !ow || !oh || det_w <= 0 || det_h <= 0 ||
        mark_w <= 0 || mark_h <= 0 || det_w > kMaxCameraFrameDimension ||
        det_h > kMaxCameraFrameDimension ||
        mark_w > kMaxCameraFrameDimension ||
        mark_h > kMaxCameraFrameDimension || (mark_w & 1) != 0 ||
        (mark_h & 1) != 0 || !std::isfinite(x1) || !std::isfinite(y1) ||
        !std::isfinite(x2) || !std::isfinite(y2) ||
        !std::isfinite(pad_ratio) || pad_ratio < 0.0F ||
        pad_ratio > kMaxChargerCropPadRatio || pad_px < 0 ||
        pad_px > kMaxCameraFrameDimension || min_w <= 0 || min_h <= 0 ||
        min_w > kMaxCameraFrameDimension ||
        min_h > kMaxCameraFrameDimension) {
        return false;
    }

    const double sx = static_cast<double>(mark_w) / det_w;
    const double sy = static_cast<double>(mark_h) / det_h;
    double mx1 = static_cast<double>(x1) * sx;
    double my1 = static_cast<double>(y1) * sy;
    double mx2 = static_cast<double>(x2) * sx;
    double my2 = static_cast<double>(y2) * sy;
    if (!std::isfinite(mx1) || !std::isfinite(my1) ||
        !std::isfinite(mx2) || !std::isfinite(my2)) {
        return false;
    }
    if (mx2 < mx1) std::swap(mx1, mx2);
    if (my2 < my1) std::swap(my1, my2);
    const double bw = mx2 - mx1;
    const double bh = my2 - my1;
    if (!std::isfinite(bw) || !std::isfinite(bh) ||
        bw < static_cast<double>(min_w) ||
        bh < static_cast<double>(min_h)) {
        return false;
    }

    const double px =
        std::max(static_cast<double>(pad_px) * sx,
                 bw * static_cast<double>(pad_ratio));
    const double py =
        std::max(static_cast<double>(pad_px) * sy,
                 bh * static_cast<double>(pad_ratio));
    if (!std::isfinite(px) || !std::isfinite(py)) return false;

    const double left = std::max(0.0, std::min(static_cast<double>(mark_w),
                                               mx1 - px));
    const double top = std::max(0.0, std::min(static_cast<double>(mark_h),
                                              my1 - py));
    const double right = std::max(0.0, std::min(static_cast<double>(mark_w),
                                                mx2 + px));
    const double bottom = std::max(0.0, std::min(static_cast<double>(mark_h),
                                                 my2 + py));
    int crop_x0 = static_cast<int>(std::floor(left)) & ~1;
    int crop_y0 = static_cast<int>(std::floor(top)) & ~1;
    int crop_x1 = std::min(mark_w,
                           (static_cast<int>(std::ceil(right)) + 1) & ~1);
    int crop_y1 = std::min(mark_h,
                           (static_cast<int>(std::ceil(bottom)) + 1) & ~1);
    if (crop_x1 <= crop_x0 || crop_y1 <= crop_y0 ||
        crop_x1 - crop_x0 < min_w || crop_y1 - crop_y0 < min_h) {
        return false;
    }
    *ox = crop_x0;
    *oy = crop_y0;
    *ow = crop_x1 - crop_x0;
    *oh = crop_y1 - crop_y0;
    return true;
}

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_CHARGER_CROP_HPP_
