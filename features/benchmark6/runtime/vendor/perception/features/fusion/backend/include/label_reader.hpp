#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace label_reader {
struct Layout { std::size_t y, x; };
// Batch validation instead of an exception check + float conversion per pixel.
// An invalid frame is rejected before it can be written or fingerprinted.
template<class T> void integers(const T* data, Layout stride, int h, int w,
                                std::vector<std::uint8_t>& mask) {
    static_assert(std::is_integral_v<T>);
    mask.resize(std::size_t(h) * w);
    bool invalid = false;
    for (int y = 0; y < h; ++y) {
        const T* row = data + std::size_t(y) * stride.y;
        auto* dst = mask.data() + std::size_t(y) * w;
        for (int x = 0; x < w; ++x) {
            const auto value = static_cast<std::uint64_t>(row[std::size_t(x) * stride.x]);
            invalid |= value > 2;
            dst[x] = static_cast<std::uint8_t>(value);
        }
    }
    if (invalid) throw std::runtime_error("compiled class index outside 0..2");
}
template<class T> void lookup(const T* data, Layout stride, int h, int w,
                             const std::vector<std::uint8_t>& table,
                             std::vector<std::uint8_t>& mask) {
    static_assert(std::is_integral_v<T> && sizeof(T) <= 2);
    using Unsigned = std::make_unsigned_t<T>;
    if (table.size() != std::size_t(1) << (sizeof(T)*8))
        throw std::runtime_error("invalid label lookup size");
    mask.resize(std::size_t(h) * w);
    bool invalid = false;
    for (int y = 0; y < h; ++y) {
        const auto* row = data + std::size_t(y) * stride.y;
        auto* dst = mask.data() + std::size_t(y) * w;
        for (int x = 0; x < w; ++x) {
            const auto value = table[static_cast<Unsigned>(row[std::size_t(x) * stride.x])];
            invalid |= value > 2; dst[x] = value;
        }
    }
    if (invalid) throw std::runtime_error("invalid quantized class index");
}
} // namespace label_reader
