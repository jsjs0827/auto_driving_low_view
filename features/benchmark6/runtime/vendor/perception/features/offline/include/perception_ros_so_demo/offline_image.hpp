#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace ros_so_demo::offline {
struct Converted {
    std::vector<std::uint8_t> bytes;
    unsigned original_width=0, original_height=0;
    std::string resize;
};
// Decode JPEG/PNG/BMP/PPM to RGB, resize and convert to tight 640x544 YUV420.
// resize: letterbox (black RGB padding) or stretch. format: nv12 or i420.
// Raw .nv12/.i420 require the matching format; .yuv uses the explicit format.
// Raw files must already be tight 640x544 and are returned with resize="none".
// Encoded-image EXIF orientation/ICC profiles are not applied; alpha is dropped.
Converted load(const std::string& path,const std::string& resize,const std::string& format);
}
