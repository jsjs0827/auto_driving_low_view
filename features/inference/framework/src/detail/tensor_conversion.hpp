#pragma once
#include "perception/types.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace perception::detail {
struct ConversionView {
    const void *data;
    std::size_t bytes;
    TensorShape valid;
    int aligned_c, aligned_h, aligned_w;
    template <class Scalar> void validate() const {
        valid.validate();
        if (!data || aligned_c < valid.channels || aligned_h < valid.height ||
            aligned_w < valid.width || aligned_c > 8192 || aligned_h > 8192 || aligned_w > 8192)
            throw std::invalid_argument("invalid conversion view");
        std::size_t count = 1;
        for (int n : {aligned_c, aligned_h, aligned_w}) {
            if (n <= 0 || count > 64U * 1024U * 1024U / std::size_t(n))
                throw std::invalid_argument("aligned conversion size exceeds limit");
            count *= n;
        }
        if (count > bytes / sizeof(Scalar))
            throw std::invalid_argument("short conversion storage");
    }
};
template <class T> T readScalar(const void *bytes, std::size_t index) noexcept {
    static_assert(std::is_arithmetic_v<T> && std::is_trivially_copyable_v<T>,
                  "numeric scalar required");
    T result;
    std::memcpy(&result, static_cast<const std::uint8_t *>(bytes) + index * sizeof(T), sizeof(T));
    return result;
}
struct IdentityQuant {
    template <class T> double operator()(T value, int) const noexcept { return double(value); }
};
struct ScaleQuant {
    const float *scales;
    const std::int8_t *zeros;
    int length;
    template <class T> double operator()(T value, int channel) const noexcept {
        const int q = length == 1 ? 0 : channel;
        // Double represents the exact difference of all supported integer scalar values.
        return (double(value) - (zeros ? double(zeros[q]) : 0.)) * double(scales[q]);
    }
};
struct ShiftQuant {
    const std::uint8_t *shifts;
    int length;
    template <class T> double operator()(T value, int channel) const noexcept {
        return std::ldexp(double(value), -int(shifts[length == 1 ? 0 : channel]));
    }
};
// The SDK adapter validates quantization metadata before constructing policy objects.
// Shapes/strides/scale values remain runtime data. No SDK types enter this header.
template <class Scalar, Layout L, class QuantPolicy>
bool convertTensorTrustedInto(const ConversionView &source, const QuantPolicy &quant, Tensor &out) {
    static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, std::int32_t> ||
                      std::is_same_v<Scalar, std::int16_t> || std::is_same_v<Scalar, std::int8_t> ||
                      std::is_same_v<Scalar, std::uint8_t>,
                  "unsupported tensor scalar");
    out.channels = source.valid.channels;
    out.height = source.valid.height;
    out.width = source.valid.width;
    out.data.resize(std::size_t(out.channels) * out.height * out.width);
    bool finite = true;
    auto copy = [&](int c, int y, int x) {
        std::size_t index;
        if constexpr (L == Layout::NHWC)
            index = (std::size_t(y) * source.aligned_w + x) * source.aligned_c + c;
        else
            index = (std::size_t(c) * source.aligned_h + y) * source.aligned_w + x;
        const double value = quant(readScalar<Scalar>(source.data, index), c);
        const float converted =
            std::abs(value) > std::numeric_limits<float>::max()
                ? std::numeric_limits<float>::quiet_NaN()
                : float(value);
        finite = finite && std::isfinite(converted);
        out.data[(std::size_t(c) * out.height + y) * out.width + x] = converted;
    };
    if constexpr (L == Layout::NHWC) {
        for (int y = 0; y < out.height; ++y)
            for (int x = 0; x < out.width; ++x)
                for (int c = 0; c < out.channels; ++c)
                    copy(c, y, x);
    } else {
        for (int c = 0; c < out.channels; ++c)
            for (int y = 0; y < out.height; ++y)
                for (int x = 0; x < out.width; ++x)
                    copy(c, y, x);
    }
    return finite;
}

template <class Scalar, Layout L, class QuantPolicy>
bool convertTensorInto(const ConversionView &source, const QuantPolicy &quant, Tensor &out) {
    source.validate<Scalar>();
    return convertTensorTrustedInto<Scalar, L>(source, quant, out);
}

template <class Scalar, Layout L, class QuantPolicy>
Tensor convertTensor(const ConversionView &source, const QuantPolicy &quant) {
    Tensor out;
    (void)convertTensorInto<Scalar, L>(source, quant, out);
    return out;
}
} // namespace perception::detail
