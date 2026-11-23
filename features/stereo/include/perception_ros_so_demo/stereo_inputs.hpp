#pragma once
#include "perception_sdk.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace ros_so_demo::stereo_inputs {
inline void validateFrame(const psdk_frame& frame) {
    if(frame.struct_size!=sizeof(frame)||frame.abi_version!=PSDK_ABI_VERSION)
        throw std::invalid_argument("Stereo image ABI mismatch");
    if(frame.format!=PSDK_FORMAT_NV12&&frame.format!=PSDK_FORMAT_I420)
        throw std::invalid_argument("Stereo image must be NV12 or I420");
    if(!frame.width||!frame.height||frame.width>4096||frame.height>4096||
       ((frame.width|frame.height|frame.stride)&1u)||frame.stride<frame.width)
        throw std::invalid_argument("Stereo YUV420 requires even dimensions and Y stride >= width");
    const auto required=std::uint64_t(frame.stride)*(std::uint64_t(frame.height)+frame.height/2);
    if(!frame.data||required>std::numeric_limits<std::size_t>::max()||frame.bytes<required)
        throw std::invalid_argument("Stereo image buffer is shorter than its padded YUV420 storage");
    if(!frame.timestamp_ns)
        throw std::invalid_argument("Stereo capture timestamp must be nonzero");
}
inline void validatePair(const psdk_frame& left,const psdk_frame& right) {
    validateFrame(left);validateFrame(right);
    if(left.frame_id!=right.frame_id||left.timestamp_ns!=right.timestamp_ns)
        throw std::invalid_argument("Stereo left/right capture identity differs; exact stamps are required");
    if(left.width!=right.width||left.height!=right.height||left.format!=right.format)
        throw std::invalid_argument("Stereo left/right must have the same dimensions and YUV420 format");
}
inline void validateHookFrame(const psdk_frame& expected,const psdk_frame& actual) {
    validateFrame(actual);
    if(expected.frame_id!=actual.frame_id||expected.timestamp_ns!=actual.timestamp_ns||
       expected.format!=actual.format||expected.width!=actual.width||expected.height!=actual.height||
       expected.stride!=actual.stride||expected.data!=actual.data||expected.bytes!=actual.bytes)
        throw std::invalid_argument("SDK stereo callback left frame differs from the registered pair");
}
inline cv::Mat toBgr(const psdk_frame& frame) {
    validateFrame(frame);
    const int width=int(frame.width),height=int(frame.height);
    cv::Mat tight(height+height/2,width,CV_8UC1);
    auto* destination=tight.ptr<std::uint8_t>();
    for(int row=0;row<height;++row)
        std::memcpy(destination+std::size_t(row)*width,
                    frame.data+std::size_t(row)*frame.stride,std::size_t(width));
    const auto* chroma=frame.data+std::size_t(frame.stride)*height;
    destination+=std::size_t(width)*height;
    if(frame.format==PSDK_FORMAT_NV12) {
        for(int row=0;row<height/2;++row)
            std::memcpy(destination+std::size_t(row)*width,
                        chroma+std::size_t(row)*frame.stride,std::size_t(width));
    } else {
        // I420 U and V rows use Y stride/2, including independent row padding.
        const auto chroma_stride=frame.stride/2;
        const auto plane_bytes=std::size_t(chroma_stride)*(height/2);
        const auto tight_plane_bytes=std::size_t(width/2)*(height/2);
        for(int plane=0;plane<2;++plane)for(int row=0;row<height/2;++row)
            std::memcpy(destination+plane*tight_plane_bytes+std::size_t(row)*(width/2),
                        chroma+plane*plane_bytes+std::size_t(row)*chroma_stride,std::size_t(width/2));
    }
    cv::Mat bgr;
    cv::cvtColor(tight,bgr,frame.format==PSDK_FORMAT_NV12?cv::COLOR_YUV2BGR_NV12:cv::COLOR_YUV2BGR_I420);
    return bgr;
}
} // namespace ros_so_demo::stereo_inputs
