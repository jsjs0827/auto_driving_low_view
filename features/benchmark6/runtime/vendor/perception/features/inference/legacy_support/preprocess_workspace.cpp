#include "detail/preprocess_workspace.hpp"
#include "detail/math.hpp"
#include <algorithm>
#include <cfenv>
#include <cmath>

namespace perception::detail {
namespace {
ResizeKey key(const Geometry &g) {
    return {g.source_w, g.source_h, g.input_w, g.input_h,
            g.resized_w, g.resized_h, g.left, g.top, std::fegetround()};
}

Geometry rgbGeometry(const ImageView &im, const PreprocessConfig &cfg) {
    int rw = cfg.width, rh = cfg.height;
    if (cfg.letterbox) {
        const double scale =
            std::min(double(cfg.width) / im.width, double(cfg.height) / im.height);
        rw = std::clamp(int(std::lround(im.width * scale)), 1, cfg.width);
        rh = std::clamp(int(std::lround(im.height * scale)), 1, cfg.height);
    }
    return {im.width, im.height, cfg.width, cfg.height, rw, rh,
            (cfg.width - rw) / 2, (cfg.height - rh) / 2};
}

Geometry nv12Geometry(const ImageView &im, const PreprocessConfig &cfg) {
    int rw = cfg.width, rh = cfg.height;
    if (cfg.letterbox) {
        const double ratio =
            std::min(double(cfg.width) / im.width, double(cfg.height) / im.height);
        rw = std::clamp(int(std::lround(im.width * ratio / 2)) * 2, 2, cfg.width);
        rh = std::clamp(int(std::lround(im.height * ratio / 2)) * 2, 2, cfg.height);
    }
    return {im.width, im.height, cfg.width, cfg.height, rw, rh,
            ((cfg.width - rw) / 4) * 2, ((cfg.height - rh) / 4) * 2};
}

void validateImage(const ImageView &im) {
    pixels(im.width, im.height);
    const bool yuv = im.format == PixelFormat::NV12 || im.format == PixelFormat::I420;
    require(im.data != nullptr, "null image");
    const std::size_t row = std::size_t(im.width) * (yuv ? 1 : 3);
    require(im.stride >= row && im.stride <= 16U * 1024U * 1024U,
            "invalid image stride");
    const auto base = im.stride * im.height;
    require(!yuv || (im.width % 2 == 0 && im.height % 2 == 0 && im.stride % 2 == 0),
            "YUV420 needs even width/height/stride");
    const auto needed = yuv ? base + base / 2 : im.stride * (im.height - 1) + row;
    require(im.bytes >= needed, "short image buffer");
}

void validateNative(const ImageView &im, const PreprocessConfig &cfg) {
    require(im.format == PixelFormat::NV12 || im.format == PixelFormat::I420,
            "native input must be NV12/I420");
    require(im.data && im.width >= 2 && im.height >= 2 && im.width <= 4096 &&
                im.height <= 4096 && im.width % 2 == 0 && im.height % 2 == 0 &&
                cfg.width >= 2 && cfg.height >= 2 && cfg.width <= 4096 &&
                cfg.height <= 4096 && cfg.width % 2 == 0 && cfg.height % 2 == 0,
            "native YUV requires even dimensions in [2,4096]");
    require(im.stride >= std::size_t(im.width) && im.stride <= 16U * 1024U * 1024U &&
                im.stride % 2 == 0,
            "invalid YUV stride");
    const auto source_plane = im.stride * im.height;
    require(im.bytes >= source_plane + source_plane / 2, "short native YUV buffer");
}

void fillDoubleAxis(std::vector<LinearSampleD> &result, int source, int target) {
    result.resize(target);
    for (int i = 0; i < target; ++i) {
        const double position =
            std::clamp((i + .5) * source / target - .5, 0., double(source - 1));
        const int first = int(position);
        const double weight = position - first;
        result[i] = {first, std::min(first + 1, source - 1), weight, 1 - weight};
    }
}

std::uint8_t interpolate(const std::uint8_t *data, std::size_t stride, int pixel_stride,
                         int channel, const LinearSampleD &x, const LinearSampleD &y) {
    const auto at = [&](int xx, int yy) {
        return data[std::size_t(yy) * stride + std::size_t(xx) * pixel_stride + channel];
    };
    const double value =
        y.inverse_weight * (x.inverse_weight * at(x.first, y.first) +
                            x.weight * at(x.second, y.first)) +
        y.weight * (x.inverse_weight * at(x.first, y.second) +
                    x.weight * at(x.second, y.second));
    return static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
}

template <class Sample>
bool identityAxis(const std::vector<Sample> &samples) {
    for (std::size_t i = 0; i < samples.size(); ++i)
        if (samples[i].first != int(i) || samples[i].second < samples[i].first ||
            samples[i].weight != 0)
            return false;
    return true;
}
} // namespace

void PreprocessWorkspace::ensureRgbMap(const Geometry &g) {
    const auto next = key(g);
    if (rgb_key_valid_ && rgb_key_ == next)
        return;
    rgb_key_ = next;
    rgb_key_valid_ = true;
    rgb_columns_.resize(g.resized_w);
    rgb_rows_.resize(g.resized_h);
    direct_rgb_rows_ = g.resized_w == g.source_w;
    for (int x = 0; x < g.resized_w; ++x) {
        const float position =
            std::clamp(coordinate(x, g.source_w, g.resized_w, false), 0.f,
                       float(g.source_w - 1));
        const int first = int(position);
        const float weight = position - first;
        rgb_columns_[x] =
            {first, std::min(first + 1, g.source_w - 1), weight, 1 - weight};
        direct_rgb_rows_ = direct_rgb_rows_ && first == x && weight == 0.f;
    }
    for (int y = 0; y < g.resized_h; ++y) {
        const float position =
            std::clamp(coordinate(y, g.source_h, g.resized_h, false), 0.f,
                       float(g.source_h - 1));
        const int first = int(position);
        const float weight = position - first;
        rgb_rows_[y] =
            {first, std::min(first + 1, g.source_h - 1), weight, 1 - weight};
    }
}

void PreprocessWorkspace::ensureNv12Map(const Geometry &g) {
    const auto next = key(g);
    if (nv12_key_valid_ && nv12_key_ == next)
        return;
    nv12_key_ = next;
    nv12_key_valid_ = true;
    fillDoubleAxis(luma_columns_, g.source_w, g.resized_w);
    fillDoubleAxis(luma_rows_, g.source_h, g.resized_h);
    fillDoubleAxis(chroma_columns_, g.source_w / 2, g.resized_w / 2);
    fillDoubleAxis(chroma_rows_, g.source_h / 2, g.resized_h / 2);
}

PreparedInputView PreprocessWorkspace::prepareRgb(const ImageView &im,
                                                   const PreprocessConfig &cfg,
                                                   bool allow_borrowed) {
    validateImage(im);
    const auto g = rgbGeometry(im, cfg);
    const auto output_bytes = pixels(cfg.width, cfg.height) * 3;
    ensureRgbMap(g);
    if (allow_borrowed && im.format == PixelFormat::RGB && im.stride == std::size_t(im.width) * 3 &&
        g.resized_w == im.width && g.resized_h == im.height && g.left == 0 && g.top == 0 &&
        direct_rgb_rows_ && identityAxis(rgb_rows_)) {
        return {g, im.data, output_bytes, false, true};
    }

    const bool yuv = im.format == PixelFormat::NV12 || im.format == PixelFormat::I420;
    const std::uint8_t *source = im.data;
    std::size_t source_stride = im.stride;
    if (yuv) {
        converted_rgb_.resize(pixels(im.width, im.height) * 3);
        const auto base = im.stride * im.height;
        for (int y = 0; y < im.height; ++y)
            for (int x = 0; x < im.width; ++x) {
                auto *target = converted_rgb_.data() +
                               (std::size_t(y) * im.width + x) * 3;
                const float yy = float(im.data[std::size_t(y) * im.stride + x]) - 16;
                std::size_t ui =
                    base + std::size_t(y / 2) * im.stride + std::size_t(x / 2) * 2;
                std::size_t vi = ui + 1;
                if (im.format == PixelFormat::I420) {
                    ui = base + std::size_t(y / 2) * (im.stride / 2) + x / 2;
                    vi = ui + base / 4;
                }
                const float u = float(im.data[ui]) - 128;
                const float v = float(im.data[vi]) - 128;
                target[0] = byte(1.164383f * yy + 1.596027f * v);
                target[1] = byte(1.164383f * yy - .391762f * u - .812968f * v);
                target[2] = byte(1.164383f * yy + 2.017232f * u);
            }
        source = converted_rgb_.data();
        source_stride = std::size_t(im.width) * 3;
    }

    output_.resize(output_bytes);
    const auto output_key = key(g);
    const bool padded = g.left != 0 || g.top != 0 || g.resized_w != g.input_w ||
                        g.resized_h != g.input_h;
    if (padded &&
        (!output_key_valid_ || output_is_nv12_ || !(output_key_ == output_key) ||
         output_pad_ != cfg.pad))
        std::fill(output_.begin(), output_.end(), cfg.pad);
    const int channels[3] = {im.format == PixelFormat::BGR ? 2 : 0, 1,
                             im.format == PixelFormat::BGR ? 0 : 2};
    for (int y = 0; y < g.resized_h; ++y) {
        const auto &ys = rgb_rows_[y];
        const auto *row0 = source + std::size_t(ys.first) * source_stride;
        const auto *row1 = source + std::size_t(ys.second) * source_stride;
        auto *target = output_.data() +
                       (std::size_t(y + g.top) * cfg.width + g.left) * 3;
        if (direct_rgb_rows_ && ys.weight == 0.f) {
            if (im.format == PixelFormat::BGR) {
                for (int x = 0; x < g.resized_w; ++x) {
                    target[x * 3] = row0[x * 3 + 2];
                    target[x * 3 + 1] = row0[x * 3 + 1];
                    target[x * 3 + 2] = row0[x * 3];
                }
            } else {
                std::copy_n(row0, std::size_t(g.resized_w) * 3, target);
            }
            continue;
        }
        for (int x = 0; x < g.resized_w; ++x) {
            const auto &xs = rgb_columns_[x];
            for (int c = 0; c < 3; ++c) {
                const int channel = channels[c];
                const auto left = std::size_t(xs.first) * 3 + channel;
                const auto right = std::size_t(xs.second) * 3 + channel;
                target[x * 3 + c] = byte(
                    ys.inverse_weight *
                        (xs.inverse_weight * row0[left] + xs.weight * row0[right]) +
                    ys.weight *
                        (xs.inverse_weight * row1[left] + xs.weight * row1[right]));
            }
        }
    }
    output_key_ = output_key;
    output_key_valid_ = true;
    output_is_nv12_ = false;
    output_pad_ = cfg.pad;
    return {g, output_.data(), output_.size(), false, false};
}

PreparedInputView PreprocessWorkspace::prepareNative(const ImageView &im,
                                                      const PreprocessConfig &cfg,
                                                      bool allow_borrowed) {
    validateNative(im, cfg);
    const auto g = nv12Geometry(im, cfg);
    const auto plane = pixels(cfg.width, cfg.height);
    const auto output_bytes = plane + plane / 2;
    ensureNv12Map(g);
    const bool exact_identity = g.resized_w == im.width && g.resized_h == im.height &&
                                g.left == 0 && g.top == 0 && identityAxis(luma_columns_) &&
                                identityAxis(luma_rows_) && identityAxis(chroma_columns_) &&
                                identityAxis(chroma_rows_);
    if (allow_borrowed && im.format == PixelFormat::NV12 &&
        im.stride == std::size_t(im.width) && exact_identity) {
        return {g, im.data, output_bytes, true, true};
    }

    output_.resize(output_bytes);
    const auto pad = static_cast<std::uint8_t>(std::lround(16 + .858823 * cfg.pad));
    const auto output_key = key(g);
    const bool padded = g.left != 0 || g.top != 0 || g.resized_w != g.input_w ||
                        g.resized_h != g.input_h;
    if (padded &&
        (!output_key_valid_ || !output_is_nv12_ || !(output_key_ == output_key) ||
         output_pad_ != cfg.pad)) {
        std::fill(output_.begin(), output_.end(), std::uint8_t{128});
        std::fill(output_.begin(), output_.begin() + plane, pad);
    }

    const auto source_plane = im.stride * im.height;
    if (exact_identity) {
        for (int y = 0; y < im.height; ++y)
            std::copy_n(im.data + std::size_t(y) * im.stride, std::size_t(im.width),
                        output_.data() + std::size_t(y) * cfg.width);
        if (im.format == PixelFormat::NV12) {
            for (int y = 0; y < im.height / 2; ++y)
                std::copy_n(im.data + source_plane + std::size_t(y) * im.stride,
                            std::size_t(im.width),
                            output_.data() + plane + std::size_t(y) * cfg.width);
        } else {
            const auto *u = im.data + source_plane;
            const auto *v = u + source_plane / 4;
            for (int y = 0; y < im.height / 2; ++y)
                for (int x = 0; x < im.width / 2; ++x) {
                    const auto source = std::size_t(y) * (im.stride / 2) + x;
                    const auto target = plane + std::size_t(y) * cfg.width + x * 2;
                    output_[target] = u[source];
                    output_[target + 1] = v[source];
                }
        }
        output_key_ = output_key;
        output_key_valid_ = true;
        output_is_nv12_ = true;
        output_pad_ = cfg.pad;
        return {g, output_.data(), output_.size(), true, false};
    }

    for (int y = 0; y < g.resized_h; ++y)
        for (int x = 0; x < g.resized_w; ++x)
            output_[std::size_t(y + g.top) * cfg.width + x + g.left] =
                interpolate(im.data, im.stride, 1, 0, luma_columns_[x], luma_rows_[y]);

    for (int c = 0; c < 2; ++c) {
        const auto *source = im.data + source_plane +
                             (im.format == PixelFormat::I420 ? c * source_plane / 4 : 0);
        const auto source_stride =
            im.format == PixelFormat::I420 ? im.stride / 2 : im.stride;
        const int pixel_stride = im.format == PixelFormat::I420 ? 1 : 2;
        const int channel = im.format == PixelFormat::I420 ? 0 : c;
        for (int y = 0; y < g.resized_h / 2; ++y)
            for (int x = 0; x < g.resized_w / 2; ++x)
                output_[plane + std::size_t(y + g.top / 2) * cfg.width +
                        (x + g.left / 2) * 2 + c] =
                    interpolate(source, source_stride, pixel_stride, channel,
                                chroma_columns_[x], chroma_rows_[y]);
    }
    output_key_ = output_key;
    output_key_valid_ = true;
    output_is_nv12_ = true;
    output_pad_ = cfg.pad;
    return {g, output_.data(), output_.size(), true, false};
}

PreparedInputView PreprocessWorkspace::execute(const ImageView &image,
                                                const PreprocessConfig &config,
                                                bool native_nv12,
                                                bool allow_borrowed) {
    pixels(config.width, config.height);
    if (native_nv12 &&
        (image.format == PixelFormat::NV12 || image.format == PixelFormat::I420))
        return prepareNative(image, config, allow_borrowed);
    return prepareRgb(image, config, allow_borrowed);
}

void rgbToNv12Into(const std::uint8_t *rgb, std::size_t rgb_bytes, int width, int height,
                   std::uint8_t *target, std::size_t target_bytes) {
    const auto count = pixels(width, height);
    require(width % 2 == 0 && height % 2 == 0 && rgb && rgb_bytes == count * 3,
            "invalid RGB to NV12 input");
    require(target && target_bytes >= count + count / 2, "short RGB to NV12 output");
    const auto rounded = [](float value) {
        const int integral = static_cast<int>(value);
        return static_cast<std::uint8_t>(integral + (value - integral >= .5f));
    };
    for (std::size_t i = 0; i < count; ++i) {
        const auto *pixel = rgb + i * 3;
        target[i] =
            rounded(16 + .256788f * pixel[0] + .504129f * pixel[1] + .097906f * pixel[2]);
    }
    for (int y = 0; y < height; y += 2) {
        const auto *top = rgb + std::size_t(y) * width * 3;
        const auto *bottom = top + std::size_t(width) * 3;
        auto *uv = target + count + std::size_t(y / 2) * width;
        for (int x = 0; x < width; x += 2) {
            const auto index = std::size_t(x) * 3;
            float r = 0, g = 0, b = 0;
            for (const auto *pixel :
                 {top + index, top + index + 3, bottom + index, bottom + index + 3}) {
                r += pixel[0] * .25f;
                g += pixel[1] * .25f;
                b += pixel[2] * .25f;
            }
            uv[x] = rounded(128 - .148223f * r - .290993f * g + .439216f * b);
            uv[x + 1] = rounded(128 + .439216f * r - .367788f * g - .071427f * b);
        }
    }
}

} // namespace perception::detail
