#pragma once
#include <cstdint>
#include <cstring>
#include <stdexcept>
namespace vse_layout {
inline int inputStride(int width) { return (width + 15) & ~15; }
inline void copyPlane(std::uint8_t* dst, int stride, const std::uint8_t* src,
                      int width, int height) {
    if (!dst || !src || stride < width || width <= 0 || height <= 0)
        throw std::runtime_error("invalid VSE plane copy");
    if (stride == width) std::memcpy(dst, src, std::size_t(width) * height);
    else for (int y = 0; y < height; ++y)
        std::memcpy(dst + std::size_t(y)*stride, src + std::size_t(y)*width, width);
}
}
