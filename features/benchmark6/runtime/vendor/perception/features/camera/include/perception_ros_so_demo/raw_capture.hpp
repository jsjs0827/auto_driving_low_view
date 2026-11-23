#pragma once
#include "perception_ros_so_demo/capture_types.hpp"
#include <MoTypes.h>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace ros_so_demo::raw {
inline Identity identity(const MoFrame& f) { return {f.FrameId, f.TimeStamp}; }
inline void checkStamp(std::uint64_t ns) {
    if (!ns || ns / 1000000000ULL > std::uint64_t(std::numeric_limits<std::int32_t>::max()))
        throw std::runtime_error("SDK timestamp is zero or outside ROS Time range; capture time is never replaced");
}
inline void sameCapture(Identity a, Identity b, const char* label) {
    checkStamp(a.ns); checkStamp(b.ns);
    if (a.id != b.id || a.ns != b.ns)
        throw std::runtime_error(std::string(label) + " capture mismatch: " +
            std::to_string(a.id) + "/" + std::to_string(a.ns) + " vs " +
            std::to_string(b.id) + "/" + std::to_string(b.ns));
}
inline void layout(const MoFrame& f, unsigned w, unsigned h, unsigned bytes, const char* label) {
    checkStamp(f.TimeStamp);
    if (!f.Data || f.ResolutionWidth != w || f.ResolutionHeight != h || f.BytesUsed != bytes)
        throw std::runtime_error(std::string(label) + " requires original tightly packed " +
            std::to_string(w) + "x" + std::to_string(h) + ", bytes=" + std::to_string(bytes) +
            "; got " + std::to_string(f.ResolutionWidth) + "x" + std::to_string(f.ResolutionHeight) +
            ", bytes=" + std::to_string(f.BytesUsed));
}
inline Plane copy(const MoFrame& f, unsigned w, unsigned h, bool q8, const char* label) {
    const unsigned bytes = q8 ? w*h*2 : w*h*3/2;
    layout(f,w,h,bytes,label);
    if (q8 && f.BitsPerPixel != 16)
        throw std::runtime_error("Disparity must contain original 16-bit Q8.8 values; mono8/depth is unsupported");
    Plane out; out.capture=identity(f); out.width=w; out.height=h;
    out.step=q8 ? w*2 : w; out.encoding=q8 ? "16UC1" : "nv12";
    const auto* first=static_cast<const std::uint8_t*>(f.Data);
    out.bytes.assign(first,first+bytes); return out;
}
inline Capture rgbd(const MoFrame& l,const MoFrame& r,const MoFrame& d,bool need_q8) {
    layout(r,640,544,640*544*3/2,"RGBD right");
    sameCapture(identity(l),identity(r),"RGBD left/right");
    Capture out; out.left=copy(l,640,544,false,"RGBD left");
    out.right=copy(r,640,544,false,"RGBD right");
    if (need_q8) {
        sameCapture(identity(l),identity(d),"RGBD left/Q8");
        out.disparity=copy(d,640,544,true,"Q8 disparity");
    }
    return out;
}
inline Plane gdc(const MoFrame& l,const MoFrame& r) {
    layout(r,1280,1088,1280*1088*3/2,"GDC right");
    sameCapture(identity(l),identity(r),"GDC left/right");
    return copy(l,1280,1088,false,"GDC left");
}
// Compare by capture time, not numeric ID ordering (IDs may wrap).
// Equal time with different IDs is a contradiction, never a valid pair.
inline int pairOrder(Identity rgbd,Identity high) {
    if (rgbd.ns == high.ns) { sameCapture(rgbd,high,"RGBD/GDC"); return 0; }
    if (rgbd.id == high.id)
        throw std::runtime_error("RGBD/GDC share an ID but have different capture timestamps");
    return rgbd.ns < high.ns ? -1 : 1;
}
}
