#include "seg_post.hpp"
#include "exact_input.hpp"
#include "hardware.hpp"
#include <memory>
#include <exception>
/**
 * @file horizon_dnn_runner.cpp
 * @brief Horizon BPU YOLO detect + PIDNet seg (no OpenCV — avoids SDK ABI clash).
 */
#include "percep_demo/horizon_dnn_runner.hpp"
#include "percep_demo/detection_post.hpp"
#include "percep_demo/segmentation_post.hpp"
#include "legacy_common.hpp"
#include "legacy_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>
#include <chrono>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

namespace ai_node_percep_demo {
namespace {
using detail::RawBox;
using detail::RawDetection;
using detail::BoxIoU;
using detail::NmsPerClass;
using detail::ArgmaxFromChw;

constexpr int kMaxImgSide = 4096;
constexpr int kMaxLogitSide = 2048;
constexpr int kMaxClasses = 256;

#define HB_OK(expr, msg)                                           \
    do {                                                           \
        const int _ret = (expr);                                   \
        if (_ret != 0) {                                           \
            std::cerr << "[HorizonDnn] " << (msg) << " code="       \
                      << _ret << std::endl;                        \
            return false;                                          \
        }                                                          \
    } while (0)

struct LetterboxMeta {
    float x_scale = 1.f;
    float y_scale = 1.f;
    int x_shift = 0;
    int y_shift = 0;
    int valid_w = 0;
    int valid_h = 0;
};

std::vector<std::string> ParseCsv(const std::string& text) {
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = TrimCsvToken(item);
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

void SyncSegClassNamesToTensor(DnnSegConfig* cfg, int tensor_nc) {
    if (!cfg || tensor_nc <= 0) return;
    auto names = ParseSegClassNames(cfg->class_names);
    if (static_cast<int>(names.size()) != tensor_nc) {
        std::cerr << "[HorizonDnn] class_names count=" << names.size()
                  << " tensor_nc=" << tensor_nc
                  << ", pad/truncate to tensor (请核对 dnn.seg.class_names)"
                  << std::endl;
        if (static_cast<int>(names.size()) > tensor_nc) {
            names.resize(static_cast<size_t>(tensor_nc));
        } else {
            for (int i = static_cast<int>(names.size()); i < tensor_nc; ++i) {
                names.push_back("cls_" + std::to_string(i));
            }
        }
    }
    cfg->num_classes = tensor_nc;
    cfg->class_names = JoinCsv(names);
    std::cout << "[HorizonDnn] seg class map";
    for (int i = 0; i < tensor_nc; ++i) {
        std::cout << " " << i << "=" << names[static_cast<size_t>(i)];
    }
    std::cout << std::endl;
}

bool ValidOrigSize(int w, int h) {
    return w > 0 && h > 0 && w <= kMaxImgSide && h <= kMaxImgSide;
}

bool ValidImage(int w, int h, size_t buf_bytes) {
    if (!ValidOrigSize(w, h)) return false;
    return buf_bytes >= static_cast<size_t>(w) * h * 3;
}

bool ValidTensorHW(int h, int w, int c) {
    return h > 0 && w > 0 && c > 0 && h <= kMaxLogitSide && w <= kMaxLogitSide &&
           c <= kMaxClasses;
}

void GetInputHW(const hbDNNTensorProperties& props, int32_t* h, int32_t* w) {
    *h = props.validShape.dimensionSize[2];
    *w = props.validShape.dimensionSize[3];
}

float ReadOutputFloatDetect(const hbDNNTensor& tensor, int index) {
    const hbDNNTensorProperties& props = tensor.properties;
    void* base = tensor.sysMem[0].virAddr;
    if (props.quantiType == NONE &&
        props.tensorType == HB_DNN_TENSOR_TYPE_F32) {
        return reinterpret_cast<float*>(base)[index];
    }
    int32_t raw = 0;
    if (props.tensorType == HB_DNN_TENSOR_TYPE_S8) {
        raw = reinterpret_cast<int8_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_U8) {
        raw = reinterpret_cast<uint8_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_S16) {
        raw = reinterpret_cast<int16_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_F32) {
        return reinterpret_cast<float*>(base)[index];
    } else {
        raw = reinterpret_cast<int32_t*>(base)[index];
    }
    if (props.quantiType == SHIFT) {
        int32_t shift = props.shift.shiftLen > 0 ? props.shift.shiftData[0] : 0;
        if (props.shift.shiftLen > index) shift = props.shift.shiftData[index];
        return static_cast<float>(raw) / static_cast<float>(1 << shift);
    }
    if (props.quantiType == SCALE) {
        float scale = props.scale.scaleLen > 0 ? props.scale.scaleData[0] : 1.f;
        if (props.scale.scaleLen > index) scale = props.scale.scaleData[index];
        return static_cast<float>(raw) * scale;
    }
    return static_cast<float>(raw);
}

float ReadOutputFloatSeg(const hbDNNTensor& tensor, int index, int channel) {
    const hbDNNTensorProperties& props = tensor.properties;
    void* base = tensor.sysMem[0].virAddr;
    if (props.quantiType == NONE &&
        props.tensorType == HB_DNN_TENSOR_TYPE_F32) {
        return reinterpret_cast<float*>(base)[index];
    }
    int32_t raw = 0;
    if (props.tensorType == HB_DNN_TENSOR_TYPE_S8) {
        raw = reinterpret_cast<int8_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_U8) {
        raw = reinterpret_cast<uint8_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_S16) {
        raw = reinterpret_cast<int16_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_S32) {
        raw = reinterpret_cast<int32_t*>(base)[index];
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_F32) {
        return reinterpret_cast<float*>(base)[index];
    } else {
        raw = reinterpret_cast<int32_t*>(base)[index];
    }
    if (props.quantiType == SHIFT) {
        int32_t shift = props.shift.shiftLen > 0 ? props.shift.shiftData[0] : 0;
        if (props.shift.shiftLen > channel) shift = props.shift.shiftData[channel];
        return static_cast<float>(raw) / static_cast<float>(1 << shift);
    }
    if (props.quantiType == SCALE) {
        float scale = props.scale.scaleLen > 0 ? props.scale.scaleData[0] : 1.f;
        if (props.scale.scaleLen > channel) scale = props.scale.scaleData[channel];
        return static_cast<float>(raw) * scale;
    }
    return static_cast<float>(raw);
}

void RgbPixelToYuv(uint8_t r, uint8_t g, uint8_t b, uint8_t* y, uint8_t* u,
                   uint8_t* v) {
    const int yi = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
    const int ui = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
    const int vi = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
    *y = static_cast<uint8_t>(std::max(0, std::min(255, yi)));
    *u = static_cast<uint8_t>(std::max(0, std::min(255, ui)));
    *v = static_cast<uint8_t>(std::max(0, std::min(255, vi)));
}

bool Rgb24ToNv12(const uint8_t* rgb, int w, int h, std::vector<uint8_t>& nv12) {
    if (!rgb || w <= 0 || h <= 0 || (w % 2) || (h % 2)) return false;
    const size_t y_size = static_cast<size_t>(w) * h;
    nv12.assign(y_size + y_size / 2, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t off = (static_cast<size_t>(y) * w + x) * 3;
            uint8_t yy, uu, vv;
            RgbPixelToYuv(rgb[off], rgb[off + 1], rgb[off + 2], &yy, &uu, &vv);
            nv12[static_cast<size_t>(y) * w + x] = yy;
            if ((y % 2) == 0 && (x % 2) == 0) {
                const size_t uv_x = x / 2;
                const size_t uv_y = y / 2;
                const size_t uv_off = y_size + uv_y * w + uv_x * 2;
                nv12[uv_off] = uu;
                nv12[uv_off + 1] = vv;
            }
        }
    }
    return true;
}

bool ResizeRgbBilinear(const uint8_t* src, int src_w, int src_h,
                       std::vector<uint8_t>& dst, int dst_w, int dst_h) {
    if (!src || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
        return false;
    }
    if (src_w == dst_w && src_h == dst_h) {
        dst.assign(src, src + static_cast<size_t>(src_w) * src_h * 3);
        return true;
    }
    dst.assign(static_cast<size_t>(dst_w) * dst_h * 3, 0);
    for (int y = 0; y < dst_h; ++y) {
        const float sy = (static_cast<float>(y) + 0.5f) * src_h / dst_h - 0.5f;
        const int y0 = std::max(0, static_cast<int>(std::floor(sy)));
        const int y1 = std::min(src_h - 1, y0 + 1);
        const float wy = sy - static_cast<float>(y0);
        for (int x = 0; x < dst_w; ++x) {
            const float sx =
                (static_cast<float>(x) + 0.5f) * src_w / dst_w - 0.5f;
            const int x0 = std::max(0, static_cast<int>(std::floor(sx)));
            const int x1 = std::min(src_w - 1, x0 + 1);
            const float wx = sx - static_cast<float>(x0);
            for (int c = 0; c < 3; ++c) {
                const float v00 =
                    src[(static_cast<size_t>(y0) * src_w + x0) * 3 + c];
                const float v01 =
                    src[(static_cast<size_t>(y0) * src_w + x1) * 3 + c];
                const float v10 =
                    src[(static_cast<size_t>(y1) * src_w + x0) * 3 + c];
                const float v11 =
                    src[(static_cast<size_t>(y1) * src_w + x1) * 3 + c];
                const float v0 = v00 * (1.f - wx) + v01 * wx;
                const float v1 = v10 * (1.f - wx) + v11 * wx;
                dst[(static_cast<size_t>(y) * dst_w + x) * 3 + c] =
                    static_cast<uint8_t>(v0 * (1.f - wy) + v1 * wy + 0.5f);
            }
        }
    }
    return true;
}

bool LetterboxRgb(const uint8_t* src, int src_w, int src_h,
                  std::vector<uint8_t>& canvas, int dst_w, int dst_h,
                  LetterboxMeta* meta, uint8_t pad) {
    if (!src || !meta) return false;
    meta->x_scale = 1.f;
    meta->y_scale = 1.f;
    meta->x_shift = 0;
    meta->y_shift = 0;
    meta->valid_w = dst_w;
    meta->valid_h = dst_h;

    canvas.assign(static_cast<size_t>(dst_w) * dst_h * 3, pad);
    meta->x_scale =
        std::min(1.f * dst_h / src_h, 1.f * dst_w / src_w);
    meta->y_scale = meta->x_scale;
    const int new_w = static_cast<int>(src_w * meta->x_scale);
    const int new_h = static_cast<int>(src_h * meta->y_scale);
    if (new_w <= 0 || new_h <= 0) return false;
    meta->x_shift = (dst_w - new_w) / 2;
    meta->y_shift = (dst_h - new_h) / 2;
    meta->valid_w = new_w;
    meta->valid_h = new_h;

    std::vector<uint8_t> resized;
    if (!ResizeRgbBilinear(src, src_w, src_h, resized, new_w, new_h)) {
        return false;
    }
    for (int y = 0; y < new_h; ++y) {
        const size_t dst_off =
            (static_cast<size_t>(meta->y_shift + y) * dst_w + meta->x_shift) * 3;
        std::memcpy(canvas.data() + dst_off, resized.data() + static_cast<size_t>(y) * new_w * 3,
                    static_cast<size_t>(new_w) * 3);
    }
    return true;
}

bool PreprocessRgb(const uint8_t* rgb, int src_w, int src_h, int dst_w,
                   int dst_h, int resize_type, LetterboxMeta* meta,
                   std::vector<uint8_t>& out, uint8_t letterbox_pad) {
    if (resize_type == 1) {
        return LetterboxRgb(rgb, src_w, src_h, out, dst_w, dst_h, meta,
                            letterbox_pad);
    }
    meta->x_scale = 1.f * dst_w / src_w;
    meta->y_scale = 1.f * dst_h / src_h;
    meta->x_shift = 0;
    meta->y_shift = 0;
    meta->valid_w = dst_w;
    meta->valid_h = dst_h;
    return ResizeRgbBilinear(rgb, src_w, src_h, out, dst_w, dst_h);
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

bool CopyNv12ToTensor(hbDNNTensor& input, const std::vector<uint8_t>& nv12,
                      int w, int h) {
    const size_t need = static_cast<size_t>(w) * h * 3 / 2;
    if (nv12.size() < need || !input.sysMem[0].virAddr) return false;
    const size_t cap = input.properties.alignedByteSize;
    if(r20::optimizedPolicy() && cap>=need){
        std::memcpy(input.sysMem[0].virAddr,nv12.data(),need);
        if(cap>need)std::memset(static_cast<uint8_t*>(input.sysMem[0].virAddr)+need,0,cap-need);
    }else{
        std::memset(input.sysMem[0].virAddr, 0, cap);
        std::memcpy(input.sysMem[0].virAddr, nv12.data(),std::min(need, cap));
    }
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    return true;
}

void Yuv601ToRgb(uint8_t y, uint8_t u, uint8_t v, uint8_t* r, uint8_t* g,
                 uint8_t* b) {
    const int c = static_cast<int>(y) - 16;
    const int d = static_cast<int>(u) - 128;
    const int e = static_cast<int>(v) - 128;
    auto clip = [](int x) -> uint8_t {
        return static_cast<uint8_t>(std::max(0, std::min(255, x)));
    };
    *r = clip((298 * c + 409 * e + 128) >> 8);
    *g = clip((298 * c - 100 * d - 208 * e + 128) >> 8);
    *b = clip((298 * c + 516 * d + 128) >> 8);
}

void DecodeCenterRgbNv12(const uint8_t* data, int w, int h, int stride,
                         uint8_t* r, uint8_t* g, uint8_t* b) {
    const int cx = w / 2;
    const int cy = h / 2;
    const uint8_t* y_plane = data;
    const uint8_t* uv_plane = data + static_cast<size_t>(stride) * h;
    const int uv_off = (cy / 2) * stride + (cx / 2) * 2;
    Yuv601ToRgb(y_plane[static_cast<size_t>(cy) * stride + cx], uv_plane[uv_off],
                uv_plane[uv_off + 1], r, g, b);
}

void DecodeCenterRgbI420(const uint8_t* data, int w, int h, int stride,
                         uint8_t* r, uint8_t* g, uint8_t* b) {
    const int cx = w / 2;
    const int cy = h / 2;
    const uint8_t* y_plane = data;
    const uint8_t* u_plane = data + static_cast<size_t>(stride) * h;
    const uint8_t* v_plane =
        u_plane + static_cast<size_t>(stride / 2) * (h / 2);
    const size_t y_off = static_cast<size_t>(cy) * stride + cx;
    const size_t uv_off = static_cast<size_t>(cy / 2) * (stride / 2) + cx / 2;
    Yuv601ToRgb(y_plane[y_off], u_plane[uv_off], v_plane[uv_off], r, g, b);
}


void YuvToRgbFull(const uint8_t* data, int w, int h, int stride, bool is_nv12,
                  uint8_t* rgb) {
    if (!data || !rgb || w <= 0 || h <= 0) return;
    if (stride < w) stride = w;
    const uint8_t* y_plane = data;
    if (is_nv12) {
        const uint8_t* uv_plane = data + static_cast<size_t>(stride) * h;
        for (int y = 0; y < h; ++y) {
            const uint8_t* yrow = y_plane + static_cast<size_t>(y) * stride;
            const uint8_t* uvrow =
                uv_plane + static_cast<size_t>(y / 2) * stride;
            uint8_t* out = rgb + static_cast<size_t>(y) * w * 3;
            for (int x = 0; x < w; ++x) {
                const int uv_off = (x / 2) * 2;
                Yuv601ToRgb(yrow[x], uvrow[uv_off], uvrow[uv_off + 1],
                            &out[x * 3], &out[x * 3 + 1], &out[x * 3 + 2]);
            }
        }
    } else {
        const uint8_t* u_plane = data + static_cast<size_t>(stride) * h;
        const uint8_t* v_plane =
            u_plane + static_cast<size_t>(stride / 2) * (h / 2);
        const int uv_stride = stride / 2;
        for (int y = 0; y < h; ++y) {
            const uint8_t* yrow = y_plane + static_cast<size_t>(y) * stride;
            const uint8_t* urow =
                u_plane + static_cast<size_t>(y / 2) * uv_stride;
            const uint8_t* vrow =
                v_plane + static_cast<size_t>(y / 2) * uv_stride;
            uint8_t* out = rgb + static_cast<size_t>(y) * w * 3;
            for (int x = 0; x < w; ++x) {
                const int uxo = x / 2;
                Yuv601ToRgb(yrow[x], urow[uxo], vrow[uxo], &out[x * 3],
                            &out[x * 3 + 1], &out[x * 3 + 2]);
            }
        }
    }
}

int ColorNaturalScore(uint8_t r, uint8_t g, uint8_t b) {
    if (r < 5 && g < 5 && b < 5) return 0;
    if (r > 250 && g > 250 && b > 250) return 0;
    const int maxc = std::max({static_cast<int>(r), static_cast<int>(g),
                               static_cast<int>(b)});
    const int minc = std::min({static_cast<int>(r), static_cast<int>(g),
                               static_cast<int>(b)});
    return maxc - minc + 50;
}

int GetTensorLayout(const hbDNNTensorProperties& props) {
    if (props.tensorLayout == HB_DNN_LAYOUT_NCHW) return HB_DNN_LAYOUT_NCHW;
    if (props.tensorLayout == HB_DNN_LAYOUT_NHWC) return HB_DNN_LAYOUT_NHWC;
    return HB_DNN_LAYOUT_NCHW;
}

void GetOutputShape(const hbDNNTensorProperties& props, int* num_classes,
                    int* out_h, int* out_w) {
    const int layout = GetTensorLayout(props);
    if (layout == HB_DNN_LAYOUT_NCHW) {
        *num_classes = props.validShape.dimensionSize[1];
        *out_h = props.validShape.dimensionSize[2];
        *out_w = props.validShape.dimensionSize[3];
    } else {
        *out_h = props.validShape.dimensionSize[1];
        *out_w = props.validShape.dimensionSize[2];
        *num_classes = props.validShape.dimensionSize[3];
    }
}

int GetOutputIndex(const hbDNNTensorProperties& props, int c, int y, int x) {
    const hbDNNTensorShape& aligned = props.alignedShape;
    const int layout = GetTensorLayout(props);
    if (layout == HB_DNN_LAYOUT_NCHW) {
        const int ah = aligned.dimensionSize[2];
        const int aw = aligned.dimensionSize[3];
        return c * ah * aw + y * aw + x;
    }
    const int aw = aligned.dimensionSize[2];
    const int ac = aligned.dimensionSize[3];
    return y * aw * ac + x * ac + c;
}

void UpscaleMaskNearest(const uint8_t* src, int src_w, int src_h,
                        std::vector<uint8_t>& dst, int dst_w, int dst_h) {
    if (!src || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
        dst.clear();
        return;
    }
    if (src_w == dst_w && src_h == dst_h) {
        dst.assign(src, src + static_cast<size_t>(src_w) * src_h);
        return;
    }
    dst.resize(static_cast<size_t>(dst_w) * dst_h);
    for (int y = 0; y < dst_h; ++y) {
        const int sy = (y * src_h) / dst_h;
        const size_t src_row = static_cast<size_t>(sy) * src_w;
        const size_t dst_row = static_cast<size_t>(y) * dst_w;
        for (int x = 0; x < dst_w; ++x) {
            const int sx = (x * src_w) / dst_w;
            dst[dst_row + static_cast<size_t>(x)] = src[src_row + static_cast<size_t>(sx)];
        }
    }
}

bool DequantSegToChw(const hbDNNTensor& output, int num_classes, int out_h,
                     int out_w, std::vector<float>& chw) {
    if (!ValidTensorHW(out_h, out_w, num_classes)) return false;
    hbSysFlushMem(const_cast<hbSysMem*>(&output.sysMem[0]),
                  HB_SYS_MEM_CACHE_INVALIDATE);
    const hbDNNTensorProperties& props = output.properties;
    const size_t plane = static_cast<size_t>(out_h) * out_w;
    chw.resize(static_cast<size_t>(num_classes) * plane);
    // 通道在外：NCHW 连续读；每个输出格只反量化一次（与逐点 ReadOutputFloatSeg 数值相同）
    for (int c = 0; c < num_classes; ++c) {
        float* dst = chw.data() + static_cast<size_t>(c) * plane;
        for (int y = 0; y < out_h; ++y) {
            for (int x = 0; x < out_w; ++x) {
                dst[static_cast<size_t>(y) * out_w + static_cast<size_t>(x)] =
                    ReadOutputFloatSeg(output, GetOutputIndex(props, c, y, x), c);
            }
        }
    }
    return true;
}

bool ArgmaxFromTensor(const hbDNNTensor& output, int num_classes, int out_h,
                      int out_w, std::vector<uint8_t>& mask) {
    std::vector<float> chw;
    if (!DequantSegToChw(output, num_classes, out_h, out_w, chw)) return false;
    ArgmaxFromChw(chw, num_classes, out_h, out_w, mask);
    return true;
}

bool ArgmaxBilinearFromTensor(const hbDNNTensor& output, int num_classes,
                              int out_h,int out_w,int input_h,int input_w,
                              int orig_h,int orig_w,int resize_type,
                              const LetterboxMeta& meta,bool align_corners,
                              std::vector<uint8_t>& mask) {
    if (!ValidTensorHW(out_h,out_w,num_classes) || !ValidOrigSize(orig_w,orig_h)) return false;
    auto& ws=r20::segPostWorkspace();
    std::vector<float> baseline_chw;
    auto& chw=r20::optimizedPolicy()?ws.logits:baseline_chw;
    if (!DequantSegToChw(output,num_classes,out_h,out_w,chw)) return false;
    if(r20::capturePolicy()){
        std::ostringstream info;info<<"{\"kind\":\"logits\",\"channels\":"<<num_classes<<",\"height\":"<<out_h<<",\"width\":"<<out_w<<",\"tensor_type\":"<<int(output.properties.tensorType)<<",\"layout\":"<<int(output.properties.tensorLayout)<<",\"quantization_type\":"<<int(output.properties.quantiType)<<",\"native_bytes\":"<<output.properties.alignedByteSize<<",\"decoded_layout\":\"CHW float32\"}";
        r20::captureOutput("segmentation",output.sysMem[0].virAddr,output.properties.alignedByteSize,info.str(),&chw);
    }
    r20::Geometry g;g.x_shift=meta.x_shift;g.y_shift=meta.y_shift;g.valid_w=meta.valid_w;g.valid_h=meta.valid_h;
    if(r20::optimizedPolicy())ws.run(chw,num_classes,out_h,out_w,input_h,input_w,orig_h,orig_w,resize_type,g,align_corners,mask);
    else r20::referenceMask(chw,num_classes,out_h,out_w,input_h,input_w,orig_h,orig_w,resize_type,g,align_corners,mask);
    return true;
}

bool PostProcessSegmentation(const hbDNNTensor& output, int num_classes,
                             int out_h, int out_w, int input_h, int input_w,
                             int orig_h, int orig_w, int resize_type,
                             const LetterboxMeta& meta,
                             const std::string& argmax_mode,
                             bool align_corners,
                             std::vector<uint8_t>& seg_map) {
    if (!ValidTensorHW(out_h, out_w, num_classes) ||
        !ValidOrigSize(orig_w, orig_h)) {
        std::cerr << "[HorizonDnn] invalid seg tensor/orig size nc=" << num_classes
                  << " out=" << out_w << "x" << out_h
                  << " orig=" << orig_w << "x" << orig_h << std::endl;
        return false;
    }

    if (argmax_mode == "bilinear") {
        return ArgmaxBilinearFromTensor(
            output, num_classes, out_h, out_w, input_h, input_w, orig_h,
            orig_w, resize_type, meta, align_corners, seg_map);
    }

    std::vector<uint8_t> model_mask;
    if (!ArgmaxFromTensor(output, num_classes, out_h, out_w, model_mask)) {
        return false;
    }

    if (resize_type == 0) {
        UpscaleMaskNearest(model_mask.data(), out_w, out_h, seg_map, orig_w, orig_h);
        return true;
    }

    std::vector<uint8_t> input_mask;
    UpscaleMaskNearest(model_mask.data(), out_w, out_h, input_mask, input_w, input_h);

    const int crop_w = meta.valid_w;
    const int crop_h = meta.valid_h;
    if (meta.x_shift < 0 || meta.y_shift < 0 ||
        meta.x_shift + crop_w > input_w || meta.y_shift + crop_h > input_h ||
        crop_w <= 0 || crop_h <= 0) {
        return false;
    }

    std::vector<uint8_t> cropped(static_cast<size_t>(crop_w) * crop_h);
    for (int y = 0; y < crop_h; ++y) {
        const size_t src_off =
            static_cast<size_t>(meta.y_shift + y) * input_w + meta.x_shift;
        std::memcpy(cropped.data() + static_cast<size_t>(y) * crop_w,
                    input_mask.data() + src_off,
                    static_cast<size_t>(crop_w));
    }
    UpscaleMaskNearest(cropped.data(), crop_w, crop_h, seg_map, orig_w, orig_h);
    return true;
}

int InferClassesNum(hbDNNHandle_t dnn_handle, int reg) {
    int output_count = 0;
    hbDNNGetOutputCount(&output_count, dnn_handle);
    const int bbox_channels = reg * 4;
    int inferred = -1;
    for (int i = 0; i < output_count; ++i) {
        hbDNNTensorProperties props;
        if (hbDNNGetOutputTensorProperties(&props, dnn_handle, i) != 0) continue;
        const int32_t c = props.validShape.dimensionSize[3];
        if (c == bbox_channels) continue;
        if (inferred < 0) inferred = c;
    }
    return inferred > 0 ? inferred : 80;
}

int MapOutputOrder(hbDNNHandle_t dnn_handle, int input_h, int input_w,
                   int classes_num, int reg, int order[6]) {
    const int32_t expected[6][3] = {
        {input_h / 8, input_w / 8, classes_num},
        {input_h / 8, input_w / 8, reg * 4},
        {input_h / 16, input_w / 16, classes_num},
        {input_h / 16, input_w / 16, reg * 4},
        {input_h / 32, input_w / 32, classes_num},
        {input_h / 32, input_w / 32, reg * 4}};
    for (int i = 0; i < 6; ++i) {
        order[i] = -1;
        for (int j = 0; j < 6; ++j) {
            hbDNNTensorProperties props;
            if (hbDNNGetOutputTensorProperties(&props, dnn_handle, j) != 0) {
                continue;
            }
            const int32_t h = props.validShape.dimensionSize[1];
            const int32_t w = props.validShape.dimensionSize[2];
            const int32_t c = props.validShape.dimensionSize[3];
            if (h == expected[i][0] && w == expected[i][1] && c == expected[i][2]) {
                order[i] = j;
                break;
            }
        }
        if (order[i] < 0) return -1;
    }
    return 0;
}

int PostProcessDetections(hbDNNTensor* outputs, const int order[6], int input_h,
                          int input_w, int classes_num, int reg,
                          float score_threshold, float nms_threshold,
                          int nms_top_k, std::vector<RawDetection>* out) {
    const int strides[3] = {8, 16, 32};
    const int heights[3] = {input_h / 8, input_h / 16, input_h / 32};
    const int widths[3] = {input_w / 8, input_w / 16, input_w / 32};
    const float conf_thres_raw =
        -std::log(1.f / score_threshold - 1.f);

    std::vector<RawDetection> candidates;

    for (int scale_idx = 0; scale_idx < 3; ++scale_idx) {
        const int cls_output_idx = order[scale_idx * 2];
        const int bbox_output_idx = order[scale_idx * 2 + 1];
        const int stride = strides[scale_idx];
        const int h = heights[scale_idx];
        const int w = widths[scale_idx];

        hbSysFlushMem(&outputs[cls_output_idx].sysMem[0],
                      HB_SYS_MEM_CACHE_INVALIDATE);
        hbSysFlushMem(&outputs[bbox_output_idx].sysMem[0],
                      HB_SYS_MEM_CACHE_INVALIDATE);

        const int cls_stride = classes_num;
        const int bbox_stride = reg * 4;

        for (int gh = 0; gh < h; ++gh) {
            for (int gw = 0; gw < w; ++gw) {
                const int cell_index = gh * w + gw;
                int cls_id = 0;
                float max_logit = ReadOutputFloatDetect(
                    outputs[cls_output_idx], cell_index * cls_stride);
                for (int c = 1; c < classes_num; ++c) {
                    const float logit = ReadOutputFloatDetect(
                        outputs[cls_output_idx], cell_index * cls_stride + c);
                    if (logit > max_logit) {
                        max_logit = logit;
                        cls_id = c;
                    }
                }
                if (max_logit < conf_thres_raw) continue;

                const float score = 1.f / (1.f + std::exp(-max_logit));
                float ltrb[4] = {0.f, 0.f, 0.f, 0.f};
                for (int i = 0; i < 4; ++i) {
                    float sum = 0.f;
                    for (int j = 0; j < reg; ++j) {
                        const int idx = cell_index * bbox_stride + reg * i + j;
                        const float dfl = std::exp(
                            ReadOutputFloatDetect(outputs[bbox_output_idx], idx));
                        ltrb[i] += dfl * static_cast<float>(j);
                        sum += dfl;
                    }
                    ltrb[i] /= sum;
                }
                if (ltrb[2] + ltrb[0] <= 0.f || ltrb[3] + ltrb[1] <= 0.f) {
                    continue;
                }
                RawDetection det;
                det.class_id = cls_id;
                det.confidence = score;
                const float x1 = (gw + 0.5f - ltrb[0]) * stride;
                const float y1 = (gh + 0.5f - ltrb[1]) * stride;
                const float x2 = (gw + 0.5f + ltrb[2]) * stride;
                const float y2 = (gh + 0.5f + ltrb[3]) * stride;
                det.box.x = x1;
                det.box.y = y1;
                det.box.w = x2 - x1;
                det.box.h = y2 - y1;
                candidates.push_back(det);
            }
        }
    }

    NmsPerClass(candidates, nms_threshold, nms_top_k);
    *out = std::move(candidates);
    return 0;
}

RawDetection MapDetectionToOriginal(const RawDetection& det,
                                    const LetterboxMeta& meta, int orig_w,
                                    int orig_h) {
    RawDetection out = det;
    float x1 = (det.box.x - meta.x_shift) / meta.x_scale;
    float y1 = (det.box.y - meta.y_shift) / meta.y_scale;
    float x2 = x1 + det.box.w / meta.x_scale;
    float y2 = y1 + det.box.h / meta.y_scale;
    x1 = std::max(0.f, std::min(x1, static_cast<float>(orig_w)));
    y1 = std::max(0.f, std::min(y1, static_cast<float>(orig_h)));
    x2 = std::max(0.f, std::min(x2, static_cast<float>(orig_w)));
    y2 = std::max(0.f, std::min(y2, static_cast<float>(orig_h)));
    out.box.x = x1;
    out.box.y = y1;
    out.box.w = std::max(0.f, x2 - x1);
    out.box.h = std::max(0.f, y2 - y1);
    return out;
}

void DrawHLine(uint8_t* rgb, int w, int h, int x0, int x1, int y, uint8_t r,
               uint8_t g, uint8_t b) {
    if (y < 0 || y >= h) return;
    x0 = std::max(0, x0);
    x1 = std::min(w - 1, x1);
    for (int x = x0; x <= x1; ++x) {
        const size_t off = (static_cast<size_t>(y) * w + x) * 3;
        rgb[off] = r;
        rgb[off + 1] = g;
        rgb[off + 2] = b;
    }
}

void DrawVLine(uint8_t* rgb, int w, int h, int x, int y0, int y1, uint8_t r,
               uint8_t g, uint8_t b) {
    if (x < 0 || x >= w) return;
    y0 = std::max(0, y0);
    y1 = std::min(h - 1, y1);
    for (int y = y0; y <= y1; ++y) {
        const size_t off = (static_cast<size_t>(y) * w + x) * 3;
        rgb[off] = r;
        rgb[off + 1] = g;
        rgb[off + 2] = b;
    }
}

void DrawRect(uint8_t* rgb, int w, int h, int x1, int y1, int x2, int y2,
              uint8_t r, uint8_t g, uint8_t b, int thickness) {
    for (int t = 0; t < thickness; ++t) {
        DrawHLine(rgb, w, h, x1, x2, y1 + t, r, g, b);
        DrawHLine(rgb, w, h, x1, x2, y2 - t, r, g, b);
        DrawVLine(rgb, w, h, x1 + t, y1, y2, r, g, b);
        DrawVLine(rgb, w, h, x2 - t, y1, y2, r, g, b);
    }
}

void DrawOverlay(uint8_t* rgb, int w, int h,
                 const std::vector<DnnDetection>& dets,
                 const std::vector<uint8_t>& seg_map, float seg_alpha,
                 const std::string& seg_class_names) {
    if (!seg_map.empty() && static_cast<int>(seg_map.size()) == w * h) {
        const auto names = ParseSegClassNames(seg_class_names);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const int cls = seg_map[static_cast<size_t>(y) * w + x];
                if (cls < 0) continue;
                const std::string name =
                    (cls < static_cast<int>(names.size()))
                        ? names[static_cast<size_t>(cls)]
                        : std::string();
                uint8_t cr = 0, cg = 0, cb = 0;
                SegClassVisRgb(cls, name, cr, cg, cb);
                const size_t off = (static_cast<size_t>(y) * w + x) * 3;
                const float a = seg_alpha;
                rgb[off] = static_cast<uint8_t>(rgb[off] * (1.f - a) + cr * a);
                rgb[off + 1] =
                    static_cast<uint8_t>(rgb[off + 1] * (1.f - a) + cg * a);
                rgb[off + 2] =
                    static_cast<uint8_t>(rgb[off + 2] * (1.f - a) + cb * a);
            }
        }
    }
    static const uint8_t kColors[8][3] = {
        {255, 56, 56}, {255, 157, 151}, {255, 112, 31}, {255, 178, 29},
        {207, 210, 49}, {72, 249, 10}, {0, 128, 255}, {128, 0, 255},
    };
    for (const auto& det : dets) {
        const auto& c = kColors[det.class_id % 8];
        DrawRect(rgb, w, h, static_cast<int>(det.x1), static_cast<int>(det.y1),
                 static_cast<int>(det.x2), static_cast<int>(det.y2), c[0], c[1],
                 c[2], 2);
    }
}

struct InferContext {
    r21::FrameGate* gate=nullptr;
    r21::BpuTiming* timing=nullptr;
    int lane=0,priority=0;
};
thread_local InferContext* current_context=nullptr;
struct ContextScope {
    InferContext* previous;
    explicit ContextScope(InferContext* context):previous(current_context){current_context=context;}
    ~ContextScope(){current_context=previous;}
};
bool RunInfer(hbDNNHandle_t handle, hbDNNTensor* input,
              std::vector<hbDNNTensor>* outputs) {
    r21::BpuTiming local;
    auto* context=current_context;
    auto& metrics=context&&context->timing?*context->timing:local;
    const auto run=[&]() {
        using Clock=std::chrono::steady_clock;
        const auto ms=[](Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();};
        hbDNNTaskHandle_t task=nullptr;
        hbDNNTensor* out_ptr=outputs->data();
        hbDNNInferCtrlParam ctrl;
        HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
        if(context)ctrl.priority=context->priority;
        auto start=Clock::now();
        const int submit=hbDNNInfer(&task,&out_ptr,input,handle,&ctrl);
        metrics.submit_ms=ms(start,Clock::now());
        int wait=0;
        if(submit==0 && task){start=Clock::now();wait=hbDNNWaitTaskDone(task,0);metrics.wait_ms=ms(start,Clock::now());}
        int release=0;
        // Release on every path, including failed submit with a non-null handle.
        if(task){start=Clock::now();release=hbDNNReleaseTask(task);metrics.release_ms=ms(start,Clock::now());}
        if(submit || wait || release || !task){std::cerr<<"[HorizonDnn] task failed submit="<<submit<<" wait="<<wait<<" release="<<release<<std::endl;return false;}
        return true;
    };
    if(context && context->gate)return context->gate->run(context->lane,run,metrics.gate_ms);
    return run();
}

#if defined(__GNUC__)
__attribute__((noinline))
#elif defined(_MSC_VER)
__declspec(noinline)
#endif
bool ActualScalarInputOracle(const uint8_t* yuv,int sw,int sh,int stride,size_t bytes,bool nv12,
    int dw,int dh,int resize,r20::Geometry* geometry,std::vector<uint8_t>& result,uint8_t pad) {
    // Calibration follows the ORIGINAL raw format/stride, before shared NV12 packing.
    // This conversion is startup-only; preserve the original scalar helper and its branch.
    std::vector<uint8_t> raw;const auto context=r22::originalSourceContext();
    if(context){
        const bool target_nv12=context&1;const int target_stride=int(context>>2);
        if(target_stride<sw||target_stride%2)return false;
        if(target_nv12!=nv12 || target_stride!=stride){
            r20::PackWorkspace pack;const auto input=pack.run(yuv,sw,sh,stride,bytes,nv12);
            raw.assign(std::size_t(target_stride)*sh*3/2,0);
            for(int y=0;y<sh;++y)std::memcpy(raw.data()+std::size_t(y)*target_stride,input.data+std::size_t(y)*sw,sw);
            const auto plane=std::size_t(target_stride)*sh;const auto* uv=input.data+std::size_t(sw)*sh;
            if(target_nv12){for(int y=0;y<sh/2;++y)std::memcpy(raw.data()+plane+std::size_t(y)*target_stride,uv+std::size_t(y)*sw,sw);}
            else for(int y=0;y<sh/2;++y)for(int x=0;x<sw/2;++x){
                raw[plane+std::size_t(y)*(target_stride/2)+x]=uv[std::size_t(y)*sw+2*x];
                raw[plane+plane/4+std::size_t(y)*(target_stride/2)+x]=uv[std::size_t(y)*sw+2*x+1];
            }
            yuv=raw.data();stride=target_stride;bytes=raw.size();nv12=target_nv12;
        }
    }
    LetterboxMeta meta;
    if(!PreprocessYuv420(yuv,sw,sh,stride,bytes,nv12,dw,dh,resize,&meta,result,pad))return false;
    geometry->x_scale=meta.x_scale;geometry->y_scale=meta.y_scale;
    geometry->x_shift=meta.x_shift;geometry->y_shift=meta.y_shift;
    geometry->valid_w=meta.valid_w;geometry->valid_h=meta.valid_h;
    return true;
}

bool PrepareExactInput(int lane,const uint8_t* yuv,int sw,int sh,int stride,size_t bytes,bool nv12,
                       int dw,int dh,int resize,LetterboxMeta& meta,hbDNNTensor& input) {
    using Clock=std::chrono::steady_clock;
    const auto start=Clock::now();const size_t need=size_t(dw)*dh*3/2;
    if(!input.sysMem[0].virAddr || size_t(input.properties.alignedByteSize)<need)return false;
    r20::Geometry g;r22::View view;auto& metrics=r22::stats(lane);metrics={};
    if(r22::variant()==r22::Variant::R21) {
        auto& old=lane==0?r20::detectorWorkspace():r20::segmentationWorkspace();
        old.setReference(ActualScalarInputOracle,r22::originalSourceContext());
        metrics.initialized=old.mathMode()<0;
        if(!old.prepare(yuv,sw,sh,stride,bytes,nv12,dw,dh,resize,g))return false;
        metrics.math_mode=old.mathMode();metrics.math_modes=old.mathModes();metrics.padding_bytes=need;metrics.copied_bytes=need;
        metrics.prepare_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        const auto transfer=Clock::now();
        if(!CopyNv12ToTensor(input,old.output,dw,dh))return false;
        metrics.transfer_ms=std::chrono::duration<double,std::milli>(Clock::now()-transfer).count();
        r22::invalidateDestination(lane);view={old.output.data(),need,false};
    } else {
        const bool direct=r22::directVariant(r22::variant());auto& ws=r22::workspace(lane);
        ws.setReference(ActualScalarInputOracle,r22::originalSourceContext());
        auto* destination=direct?static_cast<uint8_t*>(input.sysMem[0].virAddr):nullptr;
        if(!ws.prepare(yuv,sw,sh,stride,bytes,nv12,dw,dh,resize,g,destination,size_t(input.properties.alignedByteSize)))return false;
        view={direct?destination:ws.output.data(),need,direct};
        const auto transfer=Clock::now();
        const bool measured=r22::profiling();auto phase=transfer;
        if(!direct) {
            std::memcpy(input.sysMem[0].virAddr,view.data,need);
            const size_t cap=input.properties.alignedByteSize;
            if(cap>need)std::memset(static_cast<uint8_t*>(input.sysMem[0].virAddr)+need,0,cap-need);
            metrics.copied_bytes=need;
        }
        if(measured){metrics.copy_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();phase=Clock::now();}
        // Retain the mandatory CPU-write -> BPU-read cache clean on every frame.
        if(hbSysFlushMem(&input.sysMem[0],HB_SYS_MEM_CACHE_CLEAN)!=0)return false;
        if(measured)metrics.clean_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();
        metrics.transfer_ms=std::chrono::duration<double,std::milli>(Clock::now()-transfer).count();
    }
    meta.x_scale=g.x_scale;meta.y_scale=g.y_scale;meta.x_shift=g.x_shift;meta.y_shift=g.y_shift;
    meta.valid_w=g.valid_w;meta.valid_h=g.valid_h;
    const auto audit_start=Clock::now();
    r22::captureInput(lane==0?"detection":"segmentation",view,input.sysMem[0].virAddr,input.properties.alignedByteSize,dw,dh,g);
    if(r22::profiling())metrics.audit_ms=std::chrono::duration<double,std::milli>(Clock::now()-audit_start).count();
    return true;
}

}  // namespace

struct HorizonDnnRunner::DetectEngine {
    hbPackedDNNHandle_t packed = nullptr;
    hbDNNHandle_t handle = nullptr;
    hbDNNTensor input{};
    std::vector<hbDNNTensor> outputs;
    int order[6]{};
    int input_h = 0;
    int input_w = 0;
    int classes_num = 12;
    std::vector<std::string> class_names;

    ~DetectEngine() { release(); }

    void release() {
        r22::invalidateDestination(0);
        if (input.sysMem[0].virAddr) {
            hbSysFreeMem(&input.sysMem[0]);
            input.sysMem[0].virAddr = nullptr;
        }
        for (auto& t : outputs) {
            if (t.sysMem[0].virAddr) {
                hbSysFreeMem(&t.sysMem[0]);
                t.sysMem[0].virAddr = nullptr;
            }
        }
        outputs.clear();
        if (packed) {
            hbDNNRelease(packed);
            packed = nullptr;
            handle = nullptr;
        }
    }

    bool init(const DnnDetectConfig& cfg) {
        release();
        class_names = ParseCsv(cfg.class_names);
        classes_num = cfg.classes_num;
        const char* model = cfg.model_path.c_str();
        HB_OK(hbDNNInitializeFromFiles(&packed, &model, 1),
              "detect hbDNNInitializeFromFiles");
        const char** names = nullptr;
        int count = 0;
        HB_OK(hbDNNGetModelNameList(&names, &count, packed),
              "detect hbDNNGetModelNameList");
        HB_OK(hbDNNGetModelHandle(&handle, packed, names[0]),
              "detect hbDNNGetModelHandle");

        int input_count = 0;
        int output_count = 0;
        HB_OK(hbDNNGetInputCount(&input_count, handle), "detect input count");
        HB_OK(hbDNNGetOutputCount(&output_count, handle), "detect output count");
        if (input_count != 1 || output_count != 6) {
            std::cerr << "[HorizonDnn] detect model IO mismatch in/out="
                      << input_count << "/" << output_count << std::endl;
            return false;
        }

        HB_OK(hbDNNGetInputTensorProperties(&input.properties, handle, 0),
              "detect input props");
        if (input.properties.tensorType != HB_DNN_IMG_TYPE_NV12) {
            std::cerr << "[HorizonDnn] detect input must be NV12" << std::endl;
            return false;
        }
        HB_OK(hbSysAllocCachedMem(&input.sysMem[0], input.properties.alignedByteSize),
              "detect input mem");
        input.properties.alignedShape = input.properties.validShape;

        outputs.resize(output_count);
        for (int i = 0; i < output_count; ++i) {
            HB_OK(hbDNNGetOutputTensorProperties(&outputs[i].properties, handle, i),
                  "detect output props");
            HB_OK(hbSysAllocCachedMem(
                      &outputs[i].sysMem[0],
                      outputs[i].properties.alignedByteSize),
                  "detect output mem");
        }

        GetInputHW(input.properties, &input_h, &input_w);
        if (classes_num <= 0) {
            classes_num = InferClassesNum(handle, cfg.reg);
        }
        if (MapOutputOrder(handle, input_h, input_w, classes_num, cfg.reg,
                           order) != 0) {
            std::cerr << "[HorizonDnn] detect output order map failed"
                      << std::endl;
            return false;
        }
        std::cout << "[HorizonDnn] detect model=" << cfg.model_path
                  << " input=" << input_w << "x" << input_h
                  << " classes=" << classes_num << std::endl;
        return true;
    }

    bool infer(const uint8_t* rgb, int src_w, int src_h,
               const DnnDetectConfig& cfg, std::vector<DnnDetection>* out) {
        LetterboxMeta meta;
        std::vector<uint8_t> preprocessed;
        if (!PreprocessRgb(rgb, src_w, src_h, input_w, input_h, cfg.resize_type,
                           &meta, preprocessed, 127)) {
            return false;
        }
        std::vector<uint8_t> nv12;
        if (!Rgb24ToNv12(preprocessed.data(), input_w, input_h, nv12)) {
            return false;
        }
        std::memcpy(input.sysMem[0].virAddr, nv12.data(), nv12.size());
        hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
        if (!RunInfer(handle, &input, &outputs)) return false;

        std::vector<RawDetection> raw;
        if (PostProcessDetections(outputs.data(), order, input_h, input_w,
                                  classes_num, cfg.reg, cfg.score_threshold,
                                  cfg.nms_threshold, cfg.nms_top_k,
                                  &raw) != 0) {
            return false;
        }
        out->clear();
        out->reserve(raw.size());
        for (const auto& det : raw) {
            const RawDetection mapped =
                MapDetectionToOriginal(det, meta, src_w, src_h);
            DnnDetection d;
            d.class_id = mapped.class_id;
            d.confidence = mapped.confidence;
            d.x1 = mapped.box.x;
            d.y1 = mapped.box.y;
            d.x2 = mapped.box.x + mapped.box.w;
            d.y2 = mapped.box.y + mapped.box.h;
            if (d.class_id >= 0 &&
                d.class_id < static_cast<int>(class_names.size())) {
                d.class_name = class_names[d.class_id];
            } else {
                d.class_name = "cls_" + std::to_string(d.class_id);
            }
            out->push_back(d);
        }
        return true;
    }

    bool inferYuv(const uint8_t* yuv, int src_w, int src_h, int src_stride,
                  size_t bytes_used, bool src_is_nv12,
                  const DnnDetectConfig& cfg, std::vector<DnnDetection>* out,
                  DnnInferTiming* timing = nullptr) {
        const auto t_all = std::chrono::steady_clock::now();
        LetterboxMeta meta;
        std::vector<uint8_t> baseline_storage;
        auto& preprocessed = r20::optimizedPolicy() ? r20::detectorWorkspace().output : baseline_storage;
        const auto t_pre = std::chrono::steady_clock::now();
        if(r20::optimizedPolicy()) {
            if(!PrepareExactInput(0,yuv,src_w,src_h,src_stride,bytes_used,src_is_nv12,input_w,input_h,cfg.resize_type,meta,input))return false;
        } else {
            r22::invalidateDestination(0);
            if(!PreprocessYuv420(yuv,src_w,src_h,src_stride,bytes_used,src_is_nv12,input_w,input_h,cfg.resize_type,&meta,preprocessed,127))return false;
            if(!CopyNv12ToTensor(input,preprocessed,input_w,input_h))return false;
            r20::captureTensor("detection",preprocessed,input.sysMem[0].virAddr,input.properties.alignedByteSize,input_w,input_h,meta.x_shift,meta.y_shift,meta.valid_w,meta.valid_h);
        }
        if (timing) {
            timing->detect_pre_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_pre).count();
        }

        const auto t_bpu = std::chrono::steady_clock::now();
        if (!RunInfer(handle, &input, &outputs)) return false;
        if (timing) {
            timing->detect_bpu_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_bpu).count();
        }

        std::vector<RawDetection> raw;
        if (PostProcessDetections(outputs.data(), order, input_h, input_w,
                                  classes_num, cfg.reg, cfg.score_threshold,
                                  cfg.nms_threshold, cfg.nms_top_k,
                                  &raw) != 0) {
            return false;
        }
        out->clear();
        out->reserve(raw.size());
        for (const auto& det : raw) {
            const RawDetection mapped =
                MapDetectionToOriginal(det, meta, src_w, src_h);
            DnnDetection d;
            d.class_id = mapped.class_id;
            d.confidence = mapped.confidence;
            d.x1 = mapped.box.x;
            d.y1 = mapped.box.y;
            d.x2 = mapped.box.x + mapped.box.w;
            d.y2 = mapped.box.y + mapped.box.h;
            if (d.class_id >= 0 &&
                d.class_id < static_cast<int>(class_names.size())) {
                d.class_name = class_names[d.class_id];
            } else {
                d.class_name = "cls_" + std::to_string(d.class_id);
            }
            out->push_back(d);
        }
        if (timing) {
            timing->detect_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_all).count();
        }
        return true;
    }
};

struct HorizonDnnRunner::SegEngine {
    hbPackedDNNHandle_t packed = nullptr;
    hbDNNHandle_t handle = nullptr;
    hbDNNTensor input{};
    std::vector<hbDNNTensor> outputs;
    int input_h = 0;
    int input_w = 0;
    int seg_output_idx = 0;
    int num_classes = 0;
    int out_h = 0;
    int out_w = 0;

    ~SegEngine() { release(); }

    void release() {
        r22::invalidateDestination(1);
        if (input.sysMem[0].virAddr) {
            hbSysFreeMem(&input.sysMem[0]);
            input.sysMem[0].virAddr = nullptr;
        }
        for (auto& t : outputs) {
            if (t.sysMem[0].virAddr) {
                hbSysFreeMem(&t.sysMem[0]);
                t.sysMem[0].virAddr = nullptr;
            }
        }
        outputs.clear();
        if (packed) {
            hbDNNRelease(packed);
            packed = nullptr;
            handle = nullptr;
        }
    }

    std::unique_ptr<FusedNv12Model> fused;
    std::unique_ptr<VseResize> vse;
    int vse_w=0,vse_h=0;
    bool init(const DnnSegConfig& cfg) {
        release();
        if(cfg.fused_only) {
            if(cfg.fused_model_path.empty()||cfg.mask_width!=864||cfg.mask_height!=480||cfg.resize_type!=0||!cfg.align_corners||cfg.argmax_mode!="bilinear")throw std::runtime_error("Invalid fused-only contract");
            input_w=832;input_h=480;num_classes=3;out_w=104;out_h=60;
            fused=std::make_unique<FusedNv12Model>(cfg.fused_model_path,input_w,input_h,864,480,false,1000,true);
            return true;
        }
        num_classes = (cfg.num_classes > 0) ? cfg.num_classes : 0;
        const char* model = cfg.model_path.c_str();
        HB_OK(hbDNNInitializeFromFiles(&packed, &model, 1),
              "seg hbDNNInitializeFromFiles");
        const char** names = nullptr;
        int count = 0;
        HB_OK(hbDNNGetModelNameList(&names, &count, packed),
              "seg hbDNNGetModelNameList");
        HB_OK(hbDNNGetModelHandle(&handle, packed, names[0]),
              "seg hbDNNGetModelHandle");

        int input_count = 0;
        int output_count = 0;
        HB_OK(hbDNNGetInputCount(&input_count, handle), "seg input count");
        HB_OK(hbDNNGetOutputCount(&output_count, handle), "seg output count");
        if (input_count != 1) return false;

        HB_OK(hbDNNGetInputTensorProperties(&input.properties, handle, 0),
              "seg input props");
        HB_OK(hbSysAllocCachedMem(&input.sysMem[0], input.properties.alignedByteSize),
              "seg input mem");
        input.properties.alignedShape = input.properties.validShape;

        outputs.resize(output_count);
        for (int i = 0; i < output_count; ++i) {
            HB_OK(hbDNNGetOutputTensorProperties(&outputs[i].properties, handle, i),
                  "seg output props");
            HB_OK(hbSysAllocCachedMem(
                      &outputs[i].sysMem[0],
                      outputs[i].properties.alignedByteSize),
                  "seg output mem");
        }

        GetInputHW(input.properties, &input_h, &input_w);
        seg_output_idx = 0;
        int best_score = -1;
        for (int i = 0; i < output_count; ++i) {
            int nc = 0, oh = 0, ow = 0;
            GetOutputShape(outputs[i].properties, &nc, &oh, &ow);
            int score = nc * oh * ow;
            if (cfg.num_classes > 0 && nc == cfg.num_classes) score += 100000;
            if (score > best_score) {
                best_score = score;
                seg_output_idx = i;
            }
        }
        GetOutputShape(outputs[seg_output_idx].properties, &num_classes, &out_h,
                       &out_w);
        if (!ValidTensorHW(out_h, out_w, num_classes)) {
            std::cerr << "[HorizonDnn] invalid seg output shape nc=" << num_classes
                      << " " << out_w << "x" << out_h << std::endl;
            return false;
        }
        if (cfg.num_classes > 0 && num_classes != cfg.num_classes) {
            std::cerr << "[HorizonDnn] seg tensor classes=" << num_classes
                      << " != class_names/num_classes=" << cfg.num_classes
                      << ", use tensor channel count" << std::endl;
        }
        std::cout << "[HorizonDnn] seg model=" << cfg.model_path
                  << " input=" << input_w << "x" << input_h
                  << " classes=" << num_classes
                  << " output=" << out_w << "x" << out_h
                  << " idx=" << seg_output_idx << std::endl;
        if(!cfg.fused_model_path.empty()) {
            if(num_classes!=3 || cfg.resize_type!=0 || !cfg.align_corners || cfg.argmax_mode!="bilinear" || cfg.mask_width!=864 || cfg.mask_height!=480) throw std::runtime_error("Existing fused-u8 route requires explicit 3-class/align_corners/direct-resize/864x480 contract");
            fused=std::make_unique<FusedNv12Model>(cfg.fused_model_path,input_w,input_h,cfg.mask_width,cfg.mask_height,false,1000,true);
        }
        return true;
    }

    bool infer(const uint8_t* rgb, int src_w, int src_h,
               const DnnSegConfig& cfg, std::vector<uint8_t>& seg_map,
               DnnInferTiming* timing = nullptr) {
        const auto t_all = std::chrono::steady_clock::now();
        LetterboxMeta meta;
        std::vector<uint8_t> preprocessed;
        const auto t_pre = std::chrono::steady_clock::now();
        if (!PreprocessRgb(rgb, src_w, src_h, input_w, input_h, cfg.resize_type,
                           &meta, preprocessed, 0)) {
            return false;
        }
        std::vector<uint8_t> nv12;
        if (!Rgb24ToNv12(preprocessed.data(), input_w, input_h, nv12)) {
            return false;
        }
        std::memcpy(input.sysMem[0].virAddr, nv12.data(), nv12.size());
        hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
        if (timing) {
            timing->seg_pre_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_pre).count();
        }

        const auto t_bpu = std::chrono::steady_clock::now();
        if (!RunInfer(handle, &input, &outputs)) return false;
        if (timing) {
            timing->seg_bpu_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_bpu).count();
        }

        const auto t_post = std::chrono::steady_clock::now();
        const bool ok = PostProcessSegmentation(
            outputs[seg_output_idx], num_classes, out_h, out_w, input_h,
            input_w, src_h, src_w, cfg.resize_type, meta, cfg.argmax_mode,
            cfg.align_corners, seg_map);
        if (timing) {
            timing->seg_post_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_post).count();
            timing->seg_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_all).count();
        }
        return ok;
    }

    bool inferYuv(const uint8_t* yuv, int src_w, int src_h, int src_stride,
                  size_t bytes_used, bool src_is_nv12,
                  const DnnSegConfig& cfg, std::vector<uint8_t>& seg_map,
                  DnnInferTiming* timing = nullptr) {
        const auto t_all = std::chrono::steady_clock::now();
        LetterboxMeta meta;
        std::vector<uint8_t> baseline_storage;
        auto& preprocessed=r20::optimizedPolicy()?r20::segmentationWorkspace().output:baseline_storage;
        const auto t_pre = std::chrono::steady_clock::now();
        if(r20::optimizedPolicy() && !cfg.fused_vse && !fused) {
            if(!PrepareExactInput(1,yuv,src_w,src_h,src_stride,bytes_used,src_is_nv12,input_w,input_h,cfg.resize_type,meta,input))return false;
        } else {
        r22::invalidateDestination(1);
        r20::Geometry g;
        bool prepared=false;
        if(r20::optimizedPolicy() && cfg.fused_vse) {
            if(!vse || vse_w!=src_w || vse_h!=src_h) {vse=std::make_unique<VseResize>(src_w,src_h,input_w,input_h,1000);vse_w=src_w;vse_h=src_h;}
            if(!src_is_nv12) throw std::runtime_error("VSE requires compact NV12; use the optimized shared packer");
            preprocessed=vse->run(yuv,bytes_used,src_stride);g=r20::originalGeometry(src_w,src_h,input_w,input_h,0);prepared=true;
        } else if(r20::optimizedPolicy()) prepared=r20::segmentationWorkspace().prepare(yuv,src_w,src_h,src_stride,bytes_used,src_is_nv12,input_w,input_h,cfg.resize_type,g);
        else prepared=PreprocessYuv420(yuv,src_w,src_h,src_stride,bytes_used,src_is_nv12,input_w,input_h,cfg.resize_type,&meta,preprocessed,127);
        if(!prepared) return false;
        if(r20::optimizedPolicy()){meta.x_scale=g.x_scale;meta.y_scale=g.y_scale;meta.x_shift=g.x_shift;meta.y_shift=g.y_shift;meta.valid_w=g.valid_w;meta.valid_h=g.valid_h;}
        if(r20::optimizedPolicy() && fused) {
            const auto prepared_end=std::chrono::steady_clock::now();
            fused->run(preprocessed.data(),preprocessed.size(),seg_map,nullptr,r20::capturePolicy(),r20::capturePolicy());
            if(r20::capturePolicy()) {const auto& storage=fused->capturedInputStorage();r20::captureTensor("segmentation",preprocessed,storage.data(),storage.size(),input_w,input_h,0,0,input_w,input_h);const auto& native=fused->capturedMaskStorage();r20::captureOutput("segmentation",native.data(),native.size(),fused->infoJson());}
            if(timing){const auto& ft=fused->timings();timing->seg_pre_ms=std::chrono::duration<double,std::milli>(prepared_end-t_pre).count()+ft.input_copy_ms+ft.input_clean_ms;timing->seg_bpu_ms=ft.model_wait_ms;timing->seg_post_ms=ft.mask_invalidate_ms+ft.mask_extract_ms;timing->seg_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t_all).count();}
            return true;
        }
        if (!CopyNv12ToTensor(input, preprocessed, input_w, input_h)) {
            return false;
        }
        r20::captureTensor("segmentation",preprocessed,input.sysMem[0].virAddr,input.properties.alignedByteSize,input_w,input_h,meta.x_shift,meta.y_shift,meta.valid_w,meta.valid_h);
        }
        if (timing) {
            timing->seg_pre_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_pre).count();
        }

        const auto t_bpu = std::chrono::steady_clock::now();
        if (!RunInfer(handle, &input, &outputs)) return false;
        if (timing) {
            timing->seg_bpu_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_bpu).count();
        }

        const auto t_post = std::chrono::steady_clock::now();
        const bool ok = PostProcessSegmentation(
            outputs[seg_output_idx], num_classes, out_h, out_w, input_h,
            input_w, cfg.mask_height>0?cfg.mask_height:src_h, cfg.mask_width>0?cfg.mask_width:src_w, cfg.resize_type, meta, cfg.argmax_mode,
            cfg.align_corners, seg_map);
        if (timing) {
            timing->seg_post_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_post).count();
            timing->seg_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_all).count();
        }
        return ok;
    }
};

HorizonDnnRunner::HorizonDnnRunner() = default;

HorizonDnnRunner::~HorizonDnnRunner() {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    workers_.reset();
    delete detect_;
    detect_ = nullptr;
    delete seg_;
    seg_ = nullptr;
}

bool HorizonDnnRunner::init(const DnnConfig& cfg) {
    if(cfg.execution_schedule!="legacy" && cfg.execution_schedule!="free" && cfg.execution_schedule!="det-first" && cfg.execution_schedule!="seg-first")throw std::invalid_argument("Invalid execution schedule");
    if(cfg.bpu_priority<0 || cfg.bpu_priority>255)throw std::invalid_argument("Invalid BPU priority");
    if(cfg.execution_schedule!="legacy" && (!cfg.parallel_det_seg || !cfg.seg.fused_model_path.empty()))throw std::invalid_argument("Scheduled execution requires exact same-frame detection/segmentation parallel path");
    cfg_ = cfg;
    detect_ready_ = false;
    seg_ready_ = false;

    if (!cfg_.enabled) return true;

    if (cfg_.detect.enabled && !cfg_.detect.model_path.empty()) {
        detect_ = new DetectEngine();
        if (!detect_->init(cfg_.detect)) {
            std::cerr << "[HorizonDnn] detect init failed" << std::endl;
            return false;
        }
        detect_input_w_ = detect_->input_w;
        detect_input_h_ = detect_->input_h;
        detect_ready_ = true;
    }
    if (cfg_.seg.enabled && !cfg_.seg.model_path.empty()) {
        seg_ = new SegEngine();
        if (!seg_->init(cfg_.seg)) {
            std::cerr << "[HorizonDnn] seg init failed" << std::endl;
            return false;
        }
        seg_input_w_ = seg_->input_w;
        seg_input_h_ = seg_->input_h;
        seg_ready_ = true;
        SyncSegClassNamesToTensor(&cfg_.seg, seg_->num_classes);
    }
    if (!detect_ready_ && !seg_ready_) {
        std::cerr << "[HorizonDnn] no model enabled" << std::endl;
        return false;
    }
    std::cout << "[HorizonDnn] ready detect=" << detect_ready_
              << " seg=" << seg_ready_
              << " parallel_det_seg=" << cfg_.parallel_det_seg
              << " run_every_n=" << cfg_.run_every_n
              << " input=yuv420_i420_or_nv12 seg_post="
              << cfg_.seg.argmax_mode
              << " (no-opencv path)" << std::endl;
    if(cfg_.execution_schedule!="legacy" && detect_ready_ && seg_ready_)workers_=std::make_unique<r21::PersistentPair>();
    return true;
}

int HorizonDnnRunner::seg_num_classes() const {
    return (seg_ready_ && seg_) ? seg_->num_classes : 0;
}

void HorizonDnnRunner::setExecutionSchedule(const std::string& schedule,int priority) {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    if(schedule!="legacy" && schedule!="free" && schedule!="det-first" && schedule!="seg-first")throw std::invalid_argument("Invalid schedule");
    if(priority<0 || priority>255)throw std::invalid_argument("Invalid priority");
    if(schedule!="legacy" && (!cfg_.parallel_det_seg || !cfg_.seg.fused_model_path.empty()))throw std::invalid_argument("Only exact parallel path supports scheduled execution");
    if(schedule!="legacy" && !workers_)workers_=std::make_unique<r21::PersistentPair>();
    cfg_.execution_schedule=schedule;cfg_.bpu_priority=priority;
}

bool HorizonDnnRunner::infer(const uint8_t* rgb, int width, int height,
                             std::vector<DnnDetection>* detections,
                             std::vector<uint8_t>* seg_mask,
                             std::vector<uint8_t>* overlay_rgb,
                             DnnInferTiming* timing) {
    if (!cfg_.enabled || !rgb || width <= 0 || height <= 0) return false;
    if (!ValidImage(width, height, static_cast<size_t>(width) * height * 3)) {
        std::cerr << "[HorizonDnn] invalid image size " << width << "x" << height
                  << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> infer_lock(infer_mtx_);
    if (!detect_ && !seg_) return false;

    std::vector<DnnDetection> dets;
    std::vector<uint8_t> local_seg;

    if (detect_ready_) {
        const auto t0 = std::chrono::steady_clock::now();
        if (!detect_->infer(rgb, width, height, cfg_.detect, &dets)) {
            std::cerr << "[HorizonDnn] detect infer failed" << std::endl;
            return false;
        }
        if (timing) {
            timing->detect_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
        }
    }
    const bool want_seg = (seg_mask != nullptr) && seg_ready_;
    if (want_seg) {
        if (!seg_->infer(rgb, width, height, cfg_.seg, local_seg, timing)) {
            std::cerr << "[HorizonDnn] seg infer failed" << std::endl;
            return false;
        }
    }

    if (detections) *detections = dets;
    if (seg_mask) *seg_mask = local_seg;
    if (overlay_rgb &&
        (cfg_.publish_overlay || cfg_.save_overlay) &&
        (!dets.empty() || !local_seg.empty())) {
        overlay_rgb->assign(rgb, rgb + static_cast<size_t>(width) * height * 3);
        DrawOverlay(overlay_rgb->data(), width, height, dets, local_seg, 0.15f,
                    cfg_.seg.class_names);
    }
    return true;
}

bool HorizonDnnRunner::inferNv12(const uint8_t* nv12, int width, int height,
                                 int src_stride_y, size_t bytes_used,
                                 bool src_is_nv12,
                                 std::vector<DnnDetection>* detections,
                                 std::vector<uint8_t>* seg_mask,
                                 std::vector<uint8_t>* overlay_rgb,
                                 DnnInferTiming* timing) {
    if (!cfg_.enabled || !nv12 || width <= 0 || height <= 0) return false;
    if (src_stride_y < width) src_stride_y = width;
    if (bytes_used == 0) {
        bytes_used = static_cast<size_t>(src_stride_y) * height * 3 / 2;
    }
    if (bytes_used <
        static_cast<size_t>(src_stride_y) * height * 3 / 2) {
        std::cerr << "[HorizonDnn] invalid yuv buffer " << width << "x"
                  << height << " stride=" << src_stride_y
                  << " bytes=" << bytes_used << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> infer_lock(infer_mtx_);
    if (!detect_ && !seg_) return false;

    std::vector<DnnDetection> dets;
    std::vector<uint8_t> local_seg;

    const bool want_seg = (seg_mask != nullptr) && seg_ready_;
    const bool run_parallel =
        want_seg && cfg_.parallel_det_seg && detect_ready_ && seg_ready_;

    if (run_parallel) {
        DnnInferTiming det_timing{};
        DnnInferTiming seg_timing{};
        bool det_ok = false;
        bool seg_ok = false;

        const int first=cfg_.execution_schedule=="det-first"?0:cfg_.execution_schedule=="seg-first"?1:-1;
        r21::FrameGate gate(first);
        const auto det_job=[&]() {
            InferContext context{&gate,&det_timing.detect_api,0,cfg_.bpu_priority};ContextScope scope(&context);
            try {det_ok=detect_->inferYuv(nv12,width,height,src_stride_y,bytes_used,src_is_nv12,cfg_.detect,&dets,&det_timing);}
            catch(...){gate.branchFinished(0,false);throw;}
            gate.branchFinished(0,det_ok);
        };
        const auto seg_job=[&]() {
            InferContext context{&gate,&seg_timing.seg_api,1,cfg_.bpu_priority};ContextScope scope(&context);
            try {seg_ok=seg_->inferYuv(nv12,width,height,src_stride_y,bytes_used,src_is_nv12,cfg_.seg,local_seg,&seg_timing);}
            catch(...){gate.branchFinished(1,false);throw;}
            gate.branchFinished(1,seg_ok);
        };
        if(workers_ && cfg_.execution_schedule!="legacy") workers_->run(det_job,seg_job);
        else {
            std::exception_ptr det_error,seg_error;
            std::thread det_thread([&]{try{det_job();}catch(...){det_error=std::current_exception();}});
            std::thread seg_thread;
            try {seg_thread=std::thread([&]{try{seg_job();}catch(...){seg_error=std::current_exception();}});}
            catch(...){gate.branchFinished(1,false);det_thread.join();throw;}
            det_thread.join();seg_thread.join();
            if(det_error)std::rethrow_exception(det_error);
            if(seg_error)std::rethrow_exception(seg_error);
        }

        if (!det_ok) {
            std::cerr << "[HorizonDnn] detect infer failed" << std::endl;
            return false;
        }
        if (!seg_ok) {
            std::cerr << "[HorizonDnn] seg infer failed" << std::endl;
            return false;
        }
        if (timing) {
            timing->detect_api=det_timing.detect_api;
            timing->seg_api=seg_timing.seg_api;
            timing->detect_pre_ms = det_timing.detect_pre_ms;
            timing->detect_bpu_ms = det_timing.detect_bpu_ms;
            timing->detect_ms = det_timing.detect_ms;
            timing->seg_pre_ms = seg_timing.seg_pre_ms;
            timing->seg_bpu_ms = seg_timing.seg_bpu_ms;
            timing->seg_post_ms = seg_timing.seg_post_ms;
            timing->seg_ms = seg_timing.seg_ms;
        }
    } else if (detect_ready_) {
        if (!detect_->inferYuv(nv12, width, height, src_stride_y, bytes_used,
                               src_is_nv12, cfg_.detect, &dets, timing)) {
            std::cerr << "[HorizonDnn] detect infer failed" << std::endl;
            return false;
        }
    }
    if (!run_parallel && want_seg) {
        if (!seg_->inferYuv(nv12, width, height, src_stride_y, bytes_used,
                            src_is_nv12, cfg_.seg, local_seg, timing)) {
            std::cerr << "[HorizonDnn] seg infer failed" << std::endl;
            return false;
        }
    }

    if (overlay_rgb && (cfg_.publish_overlay || cfg_.save_overlay)) {
        if (overlay_rgb->size() <
            static_cast<size_t>(width) * height * 3) {
            std::cerr << "[HorizonDnn] overlay rgb buffer too small" << std::endl;
            return false;
        }
        // NV12/I420 → RGB 底图，再画检测框（否则只有黑底框）
        YuvToRgbFull(nv12, width, height, src_stride_y, src_is_nv12,
                     overlay_rgb->data());
        if (!dets.empty() || !local_seg.empty()) {
            DrawOverlay(overlay_rgb->data(), width, height, dets, local_seg,
                        0.15f, cfg_.seg.class_names);
        }
    }
    if(cfg_.execution_schedule=="legacy") {
        if(detections)*detections=dets;
        if(seg_mask)*seg_mask=local_seg;
    } else {
        if(detections)*detections=std::move(dets);
        if(seg_mask)*seg_mask=std::move(local_seg);
    }
    return true;
}

void ProbeYuv420Format(const uint8_t* data, int w, int h, size_t bytes_used,
                       YuvFormatProbe* probe) {
    if (!probe) return;
    probe->width = w;
    probe->height = h;
    probe->bytes_used = bytes_used;
    probe->likely_format = "unknown";
    if (!data || w <= 0 || h <= 0 || bytes_used == 0) return;

    int stride = static_cast<int>(bytes_used / (h * 1.5f));
    if (stride < w) stride = w;
    probe->stride_y = stride;

    DecodeCenterRgbNv12(data, w, h, stride, &probe->center_r_nv12,
                        &probe->center_g_nv12, &probe->center_b_nv12);
    DecodeCenterRgbI420(data, w, h, stride, &probe->center_r_i420,
                        &probe->center_g_i420, &probe->center_b_i420);
    probe->center_rgb_max_delta = std::max(
        {std::abs(probe->center_r_nv12 - probe->center_r_i420),
         std::abs(probe->center_g_nv12 - probe->center_g_i420),
         std::abs(probe->center_b_nv12 - probe->center_b_i420)});

    const int nv12_score =
        ColorNaturalScore(probe->center_r_nv12, probe->center_g_nv12,
                          probe->center_b_nv12);
    const int i420_score =
        ColorNaturalScore(probe->center_r_i420, probe->center_g_i420,
                          probe->center_b_i420);

    if (probe->center_rgb_max_delta < 12) {
        probe->likely_format = "nv12";
    } else if (nv12_score >= i420_score) {
        probe->likely_format = "nv12";
    } else {
        probe->likely_format = "i420";
    }
}

}  // namespace ai_node_percep_demo

namespace r22 {
r20::ReferencePreprocess runnerReferencePreprocessor(){return ai_node_percep_demo::ActualScalarInputOracle;}
}

// R25_STAGE_ADAPTER_BEGIN
#include "native_stages.hpp"
namespace r25 {
using namespace ai_node_percep_demo;
struct NativeStages::Impl {
    HorizonDnnRunner runner;
    struct Lane {LetterboxMeta meta;int phase=0,src_w=0,src_h=0;};
    Lane lanes[2];
};
NativeStages::NativeStages(const DnnConfig& source):impl_(std::make_unique<Impl>()) {
    if(!source.enabled || (!source.detect.enabled && !source.seg.enabled))
        throw std::invalid_argument("DAG exact Features require at least one enabled model");
    if((source.seg.enabled && (!source.seg.fused_model_path.empty() || source.seg.fused_vse || source.seg.resize_type!=0)) ||
       (source.detect.enabled && source.detect.resize_type!=1))
        throw std::invalid_argument("DAG exact Features require original BINs, letterbox and direct segmentation resize");
    auto cfg=source;cfg.execution_schedule="legacy"; // allocate models only; no PersistentPair
    if(!impl_->runner.init(cfg) || (cfg.detect.enabled && !impl_->runner.detect_) || (cfg.seg.enabled && !impl_->runner.seg_))
        throw std::runtime_error("Original model initialization failed");
}
NativeStages::~NativeStages()=default;
void NativeStages::close() {impl_.reset();r22::invalidateDestination(0);r22::invalidateDestination(1);}
int NativeStages::width(int lane) const {if(!impl_)throw std::logic_error("Native stages closed");if(lane<0||lane>1 || (lane?!impl_->runner.seg_:!impl_->runner.detect_))throw std::logic_error("Native model lane disabled or invalid");return lane?impl_->runner.input_w_seg():impl_->runner.input_w_detect();}
int NativeStages::height(int lane) const {if(!impl_)throw std::logic_error("Native stages closed");if(lane<0||lane>1 || (lane?!impl_->runner.seg_:!impl_->runner.detect_))throw std::logic_error("Native model lane disabled or invalid");return lane?impl_->runner.input_h_seg():impl_->runner.input_h_detect();}
void NativeStages::abortFrame() noexcept {if(impl_)for(auto& l:impl_->lanes)l.phase=0;}
void NativeStages::preprocess(int lane,const perception::ImageView& image) {
    if(!impl_)throw std::logic_error("Native stages closed");
    if(lane<0||lane>1 || (lane?!impl_->runner.seg_:!impl_->runner.detect_))throw std::logic_error("Native model lane disabled or invalid");
    auto& l=impl_->lanes[lane];if(l.phase!=0)throw std::logic_error("preprocess phase violation");
    auto& input=lane?impl_->runner.seg_->input:impl_->runner.detect_->input;
    const auto& cfg=impl_->runner.cfg_;const int resize=lane?cfg.seg.resize_type:cfg.detect.resize_type;
    const bool nv12=image.format==perception::PixelFormat::NV12;
    if(r20::optimizedPolicy()) {
        if(!PrepareExactInput(lane,image.data,image.width,image.height,int(image.stride),image.bytes,nv12,width(lane),height(lane),resize,l.meta,input))
            throw std::runtime_error("Exact preprocess/cache clean failed");
    } else {
        r22::invalidateDestination(lane);std::vector<uint8_t> preprocessed;
        if(!PreprocessYuv420(image.data,image.width,image.height,int(image.stride),image.bytes,nv12,width(lane),height(lane),resize,&l.meta,preprocessed,127) ||
           !CopyNv12ToTensor(input,preprocessed,width(lane),height(lane)))throw std::runtime_error("Original preprocess/cache clean failed");
        r20::captureTensor(lane?"segmentation":"detection",preprocessed,input.sysMem[0].virAddr,input.properties.alignedByteSize,width(lane),height(lane),l.meta.x_shift,l.meta.y_shift,l.meta.valid_w,l.meta.valid_h);
    }
    l.src_w=image.width;l.src_h=image.height;l.phase=1;
}
void NativeStages::infer(int lane,DnnInferTiming& timing) {
    if(!impl_)throw std::logic_error("Native stages closed");
    if(lane<0||lane>1 || (lane?!impl_->runner.seg_:!impl_->runner.detect_))throw std::logic_error("Native model lane disabled or invalid");
    auto& l=impl_->lanes[lane];if(l.phase!=1)throw std::logic_error("backend phase violation");
    InferContext context{nullptr,lane?&timing.seg_api:&timing.detect_api,lane,impl_->runner.cfg_.bpu_priority};ContextScope scope(&context);
    const bool ok=lane?RunInfer(impl_->runner.seg_->handle,&impl_->runner.seg_->input,&impl_->runner.seg_->outputs):
                       RunInfer(impl_->runner.detect_->handle,&impl_->runner.detect_->input,&impl_->runner.detect_->outputs);
    if(!ok)throw std::runtime_error("BPU submit/wait/release/invalidate failed");l.phase=2;
}
std::vector<DnnDetection> NativeStages::detection() {
    if(!impl_)throw std::logic_error("Native stages closed");
    if(!impl_->runner.detect_)throw std::logic_error("Native model lane disabled");
    auto& l=impl_->lanes[0];if(l.phase!=2)throw std::logic_error("detection post phase violation");
    auto& engine=*impl_->runner.detect_;const auto& cfg=impl_->runner.cfg_.detect;
    const auto& meta=l.meta;const int src_w=l.src_w,src_h=l.src_h;
    std::vector<DnnDetection> result;auto* out=&result;
        std::vector<RawDetection> raw;
        if (PostProcessDetections(engine.outputs.data(), engine.order, engine.input_h, engine.input_w,
                                  engine.classes_num, cfg.reg, cfg.score_threshold,
                                  cfg.nms_threshold, cfg.nms_top_k,
                                  &raw) != 0) {
            throw std::runtime_error("Detection post failed");
        }
        out->clear();
        out->reserve(raw.size());
        for (const auto& det : raw) {
            const RawDetection mapped =
                MapDetectionToOriginal(det, meta, src_w, src_h);
            DnnDetection d;
            d.class_id = mapped.class_id;
            d.confidence = mapped.confidence;
            d.x1 = mapped.box.x;
            d.y1 = mapped.box.y;
            d.x2 = mapped.box.x + mapped.box.w;
            d.y2 = mapped.box.y + mapped.box.h;
            if (d.class_id >= 0 &&
                d.class_id < static_cast<int>(engine.class_names.size())) {
                d.class_name = engine.class_names[d.class_id];
            } else {
                d.class_name = "cls_" + std::to_string(d.class_id);
            }
            out->push_back(d);
        }

    l.phase=0;return result;
}
std::vector<std::uint8_t> NativeStages::segmentation() {
    if(!impl_)throw std::logic_error("Native stages closed");
    if(!impl_->runner.seg_)throw std::logic_error("Native model lane disabled");
    auto& l=impl_->lanes[1];if(l.phase!=2)throw std::logic_error("segmentation post phase violation");
    auto& e=*impl_->runner.seg_;const auto& cfg=impl_->runner.cfg_.seg;
    std::vector<std::uint8_t> result;
    if(!PostProcessSegmentation(e.outputs[e.seg_output_idx],e.num_classes,e.out_h,e.out_w,e.input_h,e.input_w,
        l.src_h,l.src_w,cfg.resize_type,l.meta,cfg.argmax_mode,cfg.align_corners,result))
        throw std::runtime_error("Original-compatible segmentation post failed");
    l.phase=0;return result;
}
} // namespace r25
// R25_STAGE_ADAPTER_END
