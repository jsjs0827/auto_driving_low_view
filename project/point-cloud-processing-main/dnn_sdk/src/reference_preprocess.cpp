#include "legacy_preprocess.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
namespace r20 { namespace reference {
constexpr int kMaxImgSide=4096;
using LetterboxMeta=Geometry;
bool ValidOrigSize(int w, int h) {
    return w > 0 && h > 0 && w <= kMaxImgSide && h <= kMaxImgSide;
}
inline int ClampInt(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
}

float SampleYBilinear(const uint8_t* y_plane, int stride, int w, int h,
                      float sx, float sy) {
    sx = std::max(0.f, std::min(sx, static_cast<float>(w - 1)));
    sy = std::max(0.f, std::min(sy, static_cast<float>(h - 1)));
    const int x0 = static_cast<int>(std::floor(sx));
    const int y0 = static_cast<int>(std::floor(sy));
    const int x1 = std::min(x0 + 1, w - 1);
    const int y1 = std::min(y0 + 1, h - 1);
    const float wx = sx - static_cast<float>(x0);
    const float wy = sy - static_cast<float>(y0);
    const float v00 = y_plane[static_cast<size_t>(y0) * stride + x0];
    const float v01 = y_plane[static_cast<size_t>(y0) * stride + x1];
    const float v10 = y_plane[static_cast<size_t>(y1) * stride + x0];
    const float v11 = y_plane[static_cast<size_t>(y1) * stride + x1];
    const float v0 = v00 * (1.f - wx) + v01 * wx;
    const float v1 = v10 * (1.f - wx) + v11 * wx;
    return v0 * (1.f - wy) + v1 * wy;
}

void SampleUvBilinear(const uint8_t* uv_plane, int stride, int uv_w, int uv_h,
                      float sx, float sy, float& u, float& v) {
    sx = std::max(0.f, std::min(sx, static_cast<float>(uv_w - 1)));
    sy = std::max(0.f, std::min(sy, static_cast<float>(uv_h - 1)));
    const int x0 = static_cast<int>(std::floor(sx));
    const int y0 = static_cast<int>(std::floor(sy));
    const int x1 = std::min(x0 + 1, uv_w - 1);
    const int y1 = std::min(y0 + 1, uv_h - 1);
    const float wx = sx - static_cast<float>(x0);
    const float wy = sy - static_cast<float>(y0);

    auto sample_u = [&](int px, int py) -> float {
        return uv_plane[static_cast<size_t>(py) * stride + px * 2];
    };
    auto sample_v = [&](int px, int py) -> float {
        return uv_plane[static_cast<size_t>(py) * stride + px * 2 + 1];
    };

    const float u00 = sample_u(x0, y0);
    const float u01 = sample_u(x1, y0);
    const float u10 = sample_u(x0, y1);
    const float u11 = sample_u(x1, y1);
    const float v00 = sample_v(x0, y0);
    const float v01 = sample_v(x1, y0);
    const float v10 = sample_v(x0, y1);
    const float v11 = sample_v(x1, y1);

    const float u0 = u00 * (1.f - wx) + u01 * wx;
    const float u1 = u10 * (1.f - wx) + u11 * wx;
    const float v0 = v00 * (1.f - wx) + v01 * wx;
    const float v1 = v10 * (1.f - wx) + v11 * wx;
    u = u0 * (1.f - wy) + u1 * wy;
    v = v0 * (1.f - wy) + v1 * wy;
}

bool ValidNv12Buffer(int w, int h, int stride_y, size_t bytes_used) {
    if (!ValidOrigSize(w, h) || (w % 2) || (h % 2)) return false;
    if (stride_y < w) stride_y = w;
    const size_t need = static_cast<size_t>(stride_y) * h * 3 / 2;
    return bytes_used >= need;
}

bool ResizeNv12Stretch(const uint8_t* src, int src_w, int src_h, int src_stride,
                       std::vector<uint8_t>& dst, int dst_w, int dst_h) {
    if (!src || !ValidOrigSize(src_w, src_h) || !ValidOrigSize(dst_w, dst_h)) {
        return false;
    }
    if ((src_w % 2) || (src_h % 2) || (dst_w % 2) || (dst_h % 2)) {
        return false;
    }
    if (src_stride < src_w) src_stride = src_w;

    const size_t dst_y_size = static_cast<size_t>(dst_w) * dst_h;
    dst.assign(dst_y_size + dst_y_size / 2, 0);
    uint8_t* dst_y = dst.data();
    uint8_t* dst_uv = dst.data() + dst_y_size;
    const uint8_t* src_y = src;
    const uint8_t* src_uv = src + static_cast<size_t>(src_stride) * src_h;

    for (int y = 0; y < dst_h; ++y) {
        const float sy = (static_cast<float>(y) + 0.5f) * src_h / dst_h - 0.5f;
        for (int x = 0; x < dst_w; ++x) {
            const float sx = (static_cast<float>(x) + 0.5f) * src_w / dst_w - 0.5f;
            dst_y[static_cast<size_t>(y) * dst_w + x] = static_cast<uint8_t>(
                SampleYBilinear(src_y, src_stride, src_w, src_h, sx, sy) + 0.5f);
        }
    }

    const int src_uv_w = src_w / 2;
    const int src_uv_h = src_h / 2;
    const int dst_uv_w = dst_w / 2;
    const int dst_uv_h = dst_h / 2;
    for (int uy = 0; uy < dst_uv_h; ++uy) {
        const float sy =
            (static_cast<float>(uy) + 0.5f) * src_uv_h / dst_uv_h - 0.5f;
        for (int ux = 0; ux < dst_uv_w; ++ux) {
            const float sx =
                (static_cast<float>(ux) + 0.5f) * src_uv_w / dst_uv_w - 0.5f;
            float u = 128.f;
            float v = 128.f;
            SampleUvBilinear(src_uv, src_stride, src_uv_w, src_uv_h, sx, sy, u, v);
            const size_t off = static_cast<size_t>(uy) * dst_w + ux * 2;
            dst_uv[off] = static_cast<uint8_t>(std::max(0.f, std::min(255.f, u + 0.5f)));
            dst_uv[off + 1] = static_cast<uint8_t>(std::max(0.f, std::min(255.f, v + 0.5f)));
        }
    }
    return true;
}

bool LetterboxNv12(const uint8_t* src, int src_w, int src_h, int src_stride,
                   std::vector<uint8_t>& canvas, int dst_w, int dst_h,
                   LetterboxMeta* meta, uint8_t pad_y = 127) {
    if (!src || !meta || !ValidOrigSize(src_w, src_h) || !ValidOrigSize(dst_w, dst_h)) {
        return false;
    }
    if ((src_w % 2) || (src_h % 2) || (dst_w % 2) || (dst_h % 2)) {
        return false;
    }
    if (src_stride < src_w) src_stride = src_w;

    meta->x_scale = std::min(1.f * dst_h / src_h, 1.f * dst_w / src_w);
    meta->y_scale = meta->x_scale;
    int new_w = static_cast<int>(src_w * meta->x_scale);
    int new_h = static_cast<int>(src_h * meta->y_scale);
    new_w &= ~1;
    new_h &= ~1;
    if (new_w <= 0 || new_h <= 0) return false;
    meta->x_shift = (dst_w - new_w) / 2;
    meta->y_shift = (dst_h - new_h) / 2;
    meta->valid_w = new_w;
    meta->valid_h = new_h;

    const size_t y_size = static_cast<size_t>(dst_w) * dst_h;
    canvas.assign(y_size + y_size / 2, 0);
    uint8_t* dst_y = canvas.data();
    uint8_t* dst_uv = canvas.data() + y_size;
    std::fill(dst_y, dst_y + y_size, pad_y);
    std::memset(dst_uv, 128, y_size / 2);

    std::vector<uint8_t> resized;
    if (!ResizeNv12Stretch(src, src_w, src_h, src_stride, resized, new_w, new_h)) {
        return false;
    }

    const uint8_t* src_res_y = resized.data();
    const uint8_t* src_res_uv = resized.data() + static_cast<size_t>(new_w) * new_h;
    for (int y = 0; y < new_h; ++y) {
        std::memcpy(dst_y + static_cast<size_t>(meta->y_shift + y) * dst_w + meta->x_shift,
                    src_res_y + static_cast<size_t>(y) * new_w,
                    static_cast<size_t>(new_w));
    }
    const int uv_x_shift = meta->x_shift;
    const int uv_y_shift = meta->y_shift / 2;
    for (int uy = 0; uy < new_h / 2; ++uy) {
        std::memcpy(dst_uv + static_cast<size_t>(uv_y_shift + uy) * dst_w + uv_x_shift,
                    src_res_uv + static_cast<size_t>(uy) * new_w,
                    static_cast<size_t>(new_w));
    }
    return true;
}

bool PreprocessNv12(const uint8_t* nv12, int src_w, int src_h, int src_stride,
                    int dst_w, int dst_h, int resize_type, LetterboxMeta* meta,
                    std::vector<uint8_t>& out, uint8_t letterbox_pad_y = 127) {
    if (!nv12 || !meta) return false;
    if (resize_type == 1) {
        return LetterboxNv12(nv12, src_w, src_h, src_stride, out, dst_w, dst_h,
                             meta, letterbox_pad_y);
    }
    meta->x_scale = 1.f * dst_w / src_w;
    meta->y_scale = 1.f * dst_h / src_h;
    meta->x_shift = 0;
    meta->y_shift = 0;
    meta->valid_w = dst_w;
    meta->valid_h = dst_h;
    return ResizeNv12Stretch(nv12, src_w, src_h, src_stride, out, dst_w, dst_h);
}

bool ConvertI420ToNv12(const uint8_t* i420, int src_w, int src_h, int src_stride,
                       std::vector<uint8_t>& nv12) {
    if (!i420 || !ValidOrigSize(src_w, src_h) || (src_w % 2) || (src_h % 2)) {
        return false;
    }
    if (src_stride < src_w) src_stride = src_w;

    const size_t y_size = static_cast<size_t>(src_w) * src_h;
    nv12.resize(y_size + y_size / 2);
    const uint8_t* y_plane = i420;
    const uint8_t* u_plane = i420 + static_cast<size_t>(src_stride) * src_h;
    const uint8_t* v_plane =
        u_plane + static_cast<size_t>(src_stride / 2) * (src_h / 2);

    uint8_t* dst_y = nv12.data();
    for (int y = 0; y < src_h; ++y) {
        std::memcpy(dst_y + static_cast<size_t>(y) * src_w,
                    y_plane + static_cast<size_t>(y) * src_stride,
                    static_cast<size_t>(src_w));
    }

    uint8_t* dst_uv = nv12.data() + y_size;
    const int uv_w = src_w / 2;
    const int uv_h = src_h / 2;
    for (int uy = 0; uy < uv_h; ++uy) {
        for (int ux = 0; ux < uv_w; ++ux) {
            const size_t src_off =
                static_cast<size_t>(uy) * (src_stride / 2) + ux;
            const size_t dst_off =
                static_cast<size_t>(uy) * src_w + ux * 2;
            dst_uv[dst_off] = u_plane[src_off];
            dst_uv[dst_off + 1] = v_plane[src_off];
        }
    }
    return true;
}

bool PreprocessYuv420(const uint8_t* yuv, int src_w, int src_h, int src_stride,
                      size_t bytes_used, bool src_is_nv12, int dst_w, int dst_h,
                      int resize_type, LetterboxMeta* meta,
                      std::vector<uint8_t>& out, uint8_t letterbox_pad_y = 127) {
    if (!yuv || !meta) return false;
    if (src_stride < src_w) src_stride = src_w;
    const size_t need =
        static_cast<size_t>(src_stride) * src_h * 3 / 2;
    if (bytes_used < need) return false;

    if (src_is_nv12) {
        return PreprocessNv12(yuv, src_w, src_h, src_stride, dst_w, dst_h,
                              resize_type, meta, out, letterbox_pad_y);
    }

    std::vector<uint8_t> nv12;
    if (!ConvertI420ToNv12(yuv, src_w, src_h, src_stride, nv12)) {
        return false;
    }
    return PreprocessNv12(nv12.data(), src_w, src_h, src_w, dst_w, dst_h,
                          resize_type, meta, out, letterbox_pad_y);
}


} }
