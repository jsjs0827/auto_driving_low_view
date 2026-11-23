#pragma once
#include "perception/types.hpp"
#include <stdexcept>
#include <string>

namespace r26 {
// Original map task: latest_rgb.left + left_data, interpreted as NV12 by
// start_config.json. No content-based format guessing and no GDC fallback.
inline void validateSelection(const std::string& stream,const std::string& format,int mode,int every) {
    if(stream!="rgbd" || format!="nv12" || mode!=0 || every!=0)
        throw std::invalid_argument("ORIGINAL_INPUT_CONTRACT: use RGBD left, NV12, SDK mode unchanged (0), no GDC/demand substitution");
}
inline void validateInput(const perception::ImageView& image) {
    if(image.width!=640 || image.height!=544 || image.format!=perception::PixelFormat::NV12 ||
       image.stride<640 || image.stride>65536 || (image.stride&1) || !image.data ||
       image.bytes!=std::size_t(image.stride)*544*3/2)
        throw std::invalid_argument("ORIGINAL_INPUT_CONTRACT: expected untouched 640x544 NV12 with common even row stride and stride*544*3/2 bytes; do not resize GDC to replace RGBD");
}
}
