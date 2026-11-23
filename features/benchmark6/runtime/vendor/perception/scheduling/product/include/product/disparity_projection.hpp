#pragma once
#include <cmath>
#include <cstdint>

namespace r28::product {
struct ProjectedDisparity {float disparity=0,x=0,y=0,z=0;};
// Preserve the original product's float/double operation order exactly. The
// visualization clamp is deliberately absent from geometry calculations.
inline bool projectOriginalDisparity(std::uint16_t raw,int u,int v,
    float fx,float fy,float cx,float cy,double bxf,ProjectedDisparity& out) {
    out.disparity=static_cast<float>(raw)/256.0;
    out.z=static_cast<float>(bxf)/out.disparity;
    if (!std::isfinite(out.z)||out.z<0.001f) return false;
    out.x=(static_cast<float>(u)-cx)*out.z/fx;
    out.y=(static_cast<float>(v)-cy)*out.z/fy;
    return std::isfinite(out.x)&&std::isfinite(out.y);
}
}
