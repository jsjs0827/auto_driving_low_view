#pragma once

#include "reusable_backend.hpp"
#include "perception/types.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
#if defined(__aarch64__) && defined(__ARM_NEON) && defined(__BYTE_ORDER__) && \
    __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#include <arm_neon.h>
#endif

namespace perception::detail {

// Private, borrowed view of a validated BPU output. The backend owns the buffer
// until the next inference or close; callers decode while holding their model
// session lock. Public Tensor remains an owned, contiguous CHW float value.
enum class NativeScalar { F32, S32, S16, S8, U8 };
enum class NativeQuant { None, Scale, Shift };
struct NativeTensorView {
    const void *data = nullptr;
    TensorShape shape;
    int aligned_c = 0, aligned_h = 0, aligned_w = 0;
    Layout layout = Layout::NHWC;
    NativeScalar scalar = NativeScalar::F32;
    NativeQuant quant = NativeQuant::None;
    const float *scales = nullptr;
    const std::int8_t *zeros = nullptr;
    const std::uint8_t *shifts = nullptr;
    int quant_length = 0;

    template <class T> static T load(const void *base, std::size_t index) noexcept {
        T value;
        std::memcpy(&value, static_cast<const std::uint8_t *>(base) + index * sizeof(T),
                    sizeof(T));
        return value;
    }
    std::size_t index(int c, int y, int x) const noexcept {
        return layout == Layout::NHWC
                   ? (std::size_t(y) * aligned_w + x) * aligned_c + c
                   : (std::size_t(c) * aligned_h + y) * aligned_w + x;
    }
    float at(int c, int y, int x) const noexcept {
        const auto offset = index(c, y, x);
        if (scalar == NativeScalar::F32)
            return load<float>(data, offset);
        double raw = 0;
        switch (scalar) {
        case NativeScalar::S32: raw = double(load<std::int32_t>(data, offset)); break;
        case NativeScalar::S16: raw = double(load<std::int16_t>(data, offset)); break;
        case NativeScalar::S8: raw = double(load<std::int8_t>(data, offset)); break;
        case NativeScalar::U8: raw = double(load<std::uint8_t>(data, offset)); break;
        case NativeScalar::F32: break;
        }
        const int q = quant_length == 1 ? 0 : c;
        if (quant == NativeQuant::Scale)
            raw = (raw - (zeros ? double(zeros[q]) : 0.)) * double(scales[q]);
        else if (quant == NativeQuant::Shift)
            raw = std::ldexp(raw, -int(shifts[q]));
        return float(raw);
    }
    // Read the representation, rather than doing floating-point arithmetic: an
    // all-ones exponent identifies both infinities and NaNs. Byte loads keep the
    // BPU buffer's alignment (including odd test offsets) out of the C++ type
    // system; ARM NEON loads can consume those bytes directly.
    static bool finiteF32Span(const std::uint8_t *bytes, std::size_t count) noexcept {
        constexpr std::uint32_t exponent_mask = 0x7f800000U;
#if defined(__aarch64__) && defined(__ARM_NEON) && defined(__BYTE_ORDER__) && \
    __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        const auto exponent = vdupq_n_u32(exponent_mask);
        auto nonfinite = vdupq_n_u32(0);
        while (count >= 4) {
            const auto bits = vreinterpretq_u32_u8(vld1q_u8(bytes));
            nonfinite = vorrq_u32(nonfinite,
                                  vceqq_u32(vandq_u32(bits, exponent), exponent));
            bytes += 4 * sizeof(float);
            count -= 4;
        }
        if (vmaxvq_u32(nonfinite) != 0)
            return false;
#endif
        for (std::size_t i = 0; i < count; ++i) {
            std::uint32_t bits;
            std::memcpy(&bits, bytes + i * sizeof(bits), sizeof(bits));
            if ((bits & exponent_mask) == exponent_mask)
                return false;
        }
        return true;
    }
    // v3 rejected nonfinite values anywhere in a selected F32 output, including
    // regression channels that the sparse decoder will not otherwise read.
    // Scan only logical values: aligned channel/row/plane padding is ignored.
    bool allFinite() const noexcept {
        if (scalar != NativeScalar::F32)
            return true;
        const auto *bytes = static_cast<const std::uint8_t *>(data);
        const auto channels = std::size_t(shape.channels);
        const auto height = std::size_t(shape.height);
        const auto width = std::size_t(shape.width);
        const auto row_stride = std::size_t(aligned_w);
        const auto channel_stride = std::size_t(aligned_c);
        const auto plane_stride = std::size_t(aligned_h) * row_stride;
        if (layout == Layout::NHWC) {
            if (aligned_c == shape.channels && aligned_w == shape.width)
                return finiteF32Span(bytes, channels * width * height);
            if (aligned_c == shape.channels) {
                for (std::size_t y = 0; y < height; ++y)
                    if (!finiteF32Span(bytes + y * row_stride * channel_stride * sizeof(float),
                                       width * channels))
                        return false;
            } else {
                for (std::size_t y = 0; y < height; ++y)
                    for (std::size_t x = 0; x < width; ++x)
                        if (!finiteF32Span(bytes + (y * row_stride + x) * channel_stride *
                                                       sizeof(float),
                                           channels))
                            return false;
            }
        } else {
            if (aligned_h == shape.height && aligned_w == shape.width)
                return finiteF32Span(bytes, channels * height * width);
            if (aligned_w == shape.width) {
                for (std::size_t c = 0; c < channels; ++c)
                    if (!finiteF32Span(bytes + c * plane_stride * sizeof(float),
                                       height * width))
                        return false;
            } else {
                for (std::size_t c = 0; c < channels; ++c)
                    for (std::size_t y = 0; y < height; ++y)
                        if (!finiteF32Span(bytes + (c * plane_stride + y * row_stride) *
                                                       sizeof(float),
                                           width))
                            return false;
            }
        }
        return true;
    }
    // Integer outputs normally cannot become nonfinite. For SCALE, reject the
    // native path when an extreme valid integer could overflow float; the
    // existing full converter remains available for that model.
    bool finiteForEveryInteger() const noexcept {
        if (scalar == NativeScalar::F32 || quant != NativeQuant::Scale)
            return true;
        double magnitude = 0;
        switch (scalar) {
        case NativeScalar::S32: magnitude = 2147483648.0; break;
        case NativeScalar::S16: magnitude = 32768.0; break;
        case NativeScalar::S8: magnitude = 128.0; break;
        case NativeScalar::U8: magnitude = 255.0; break;
        case NativeScalar::F32: break;
        }
        for (int q = 0; q < quant_length; ++q) {
            const double bound = (magnitude + (zeros ? 128.0 : 0.0)) * double(scales[q]);
            if (!std::isfinite(bound) || bound > double(std::numeric_limits<float>::max()))
                return false;
        }
        return true;
    }
};

class NativeDetectionBackend {
  public:
    virtual ~NativeDetectionBackend() = default;
    virtual bool supportsNativeDetection(const OutputSelection &) const noexcept = 0;
    virtual const std::vector<NativeTensorView> &runNativeDetection(
        const std::uint8_t *, std::size_t, bool, const OutputSelection &) = 0;
};

} // namespace perception::detail
