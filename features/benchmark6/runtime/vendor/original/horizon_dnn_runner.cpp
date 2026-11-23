/**
 * @file horizon_dnn_runner.cpp
 * @brief Horizon BPU YOLO detect + PIDNet seg (no OpenCV — avoids SDK ABI clash).
 */
#include "percep_demo/horizon_dnn_runner.hpp"
#include "percep_demo/detection_candidate_budget.hpp"
#include "percep_demo/detection_class_filter.hpp"
#include "percep_demo/dnn_palette.hpp"
#include "percep_demo/dnn_numeric.hpp"
#include "percep_demo/rgb_resize.hpp"
#include "percep_demo/resize_geometry.hpp"
#include "percep_demo/seg_class_names.hpp"

#include <algorithm>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>
#include <chrono>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

namespace ai_node_percep_demo {
namespace {

constexpr int kMaxImgSide = 4096;
constexpr int kMaxLogitSide = 2048;
constexpr int kMaxClasses = 256;
constexpr std::int64_t kMaxLogitElements = 32LL * 1024LL * 1024LL;
constexpr int kMaxNmsTopK = 4096;
// R=G=B=129 maps to limited-range BT.601 Y=127 and neutral U/V=128 in
// RgbPixelToYuv(), matching the direct YUV letterbox padding.
constexpr std::uint8_t kRgbLetterboxPadForY127 = 129;

#define HB_OK(expr, msg)                                           \
    do {                                                           \
        const int _ret = (expr);                                   \
        if (_ret != 0) {                                           \
            std::cerr << "[HorizonDnn] " << (msg) << " code="       \
                      << _ret << std::endl;                        \
            return false;                                          \
        }                                                          \
    } while (0)

void ReleaseTensorMemory(hbDNNTensor* tensor, const char* label) noexcept {
    if (!tensor || !tensor->sysMem[0].virAddr) return;
    const int ret = hbSysFreeMem(&tensor->sysMem[0]);
    if (ret != 0) {
        std::cerr << "[HorizonDnn] " << label
                  << " hbSysFreeMem cleanup code=" << ret << std::endl;
    }
    tensor->sysMem[0].phyAddr = 0;
    tensor->sysMem[0].virAddr = nullptr;
    tensor->sysMem[0].memSize = 0;
}

bool AllocateCachedTensor(hbDNNTensor* tensor, const char* label) {
    if (!tensor || tensor->properties.alignedByteSize <= 0) return false;
    const auto requested =
        static_cast<std::uint32_t>(tensor->properties.alignedByteSize);
    const int ret = hbSysAllocCachedMem(&tensor->sysMem[0], requested);
    if (ret != 0) {
        std::cerr << "[HorizonDnn] " << label
                  << " hbSysAllocCachedMem code=" << ret << std::endl;
        return false;
    }
    if (!tensor->sysMem[0].virAddr || tensor->sysMem[0].memSize < requested) {
        std::cerr << "[HorizonDnn] " << label
                  << " allocator returned insufficient storage requested="
                  << requested << " actual=" << tensor->sysMem[0].memSize
                  << std::endl;
        ReleaseTensorMemory(tensor, label);
        return false;
    }
    return true;
}

void ReleasePackedModel(hbPackedDNNHandle_t* packed, const char* label) noexcept {
    if (!packed || !*packed) return;
    const int ret = hbDNNRelease(*packed);
    if (ret != 0) {
        std::cerr << "[HorizonDnn] " << label
                  << " hbDNNRelease cleanup code=" << ret << std::endl;
    }
    *packed = nullptr;
}

struct LetterboxMeta {
    float x_scale = 1.f;
    float y_scale = 1.f;
    int x_shift = 0;
    int y_shift = 0;
    int valid_w = 0;
    int valid_h = 0;
};

struct RawBox {
    float x = 0.f;
    float y = 0.f;
    float w = 0.f;
    float h = 0.f;
};

struct RawDetection {
    int class_id = -1;
    float confidence = 0.f;
    RawBox box;
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
    if (h <= 0 || w <= 0 || c <= 0 || h > kMaxLogitSide ||
        w > kMaxLogitSide || c > kMaxClasses) {
        return false;
    }
    return static_cast<std::int64_t>(h) * w * c <= kMaxLogitElements;
}

bool ValidateRunnerConfig(const DnnConfig& cfg) {
    if (!cfg.enabled) return true;
    if (cfg.run_every_n < 1 || !std::isfinite(cfg.overlay_seg_alpha) ||
        cfg.overlay_seg_alpha < 0.0F || cfg.overlay_seg_alpha > 1.0F) {
        std::cerr << "[HorizonDnn] invalid common/overlay configuration"
                  << std::endl;
        return false;
    }
    if (cfg.detect.enabled) {
        if (cfg.detect.model_path.empty() ||
            !(cfg.detect.score_threshold > 0.0F &&
              cfg.detect.score_threshold < 1.0F) ||
            !std::isfinite(cfg.detect.score_threshold) ||
            !(cfg.detect.nms_threshold >= 0.0F &&
              cfg.detect.nms_threshold <= 1.0F) ||
            !std::isfinite(cfg.detect.nms_threshold) || cfg.detect.reg <= 0 ||
            cfg.detect.reg > kMaxClasses / 4 || cfg.detect.classes_num < 0 ||
            cfg.detect.classes_num > kMaxClasses ||
            cfg.detect.nms_top_k <= 0 ||
            cfg.detect.nms_top_k > kMaxNmsTopK ||
            (cfg.detect.resize_type != 0 && cfg.detect.resize_type != 1)) {
            std::cerr << "[HorizonDnn] invalid detection configuration"
                      << std::endl;
            return false;
        }
    }
    if (cfg.seg.enabled) {
        if (cfg.seg.model_path.empty() || cfg.seg.output_index < -1 ||
            cfg.seg.num_classes < 0 || cfg.seg.num_classes > kMaxClasses ||
            (cfg.seg.resize_type != 0 && cfg.seg.resize_type != 1) ||
            (cfg.seg.argmax_mode != "direct" &&
             cfg.seg.argmax_mode != "bilinear")) {
            std::cerr << "[HorizonDnn] invalid segmentation configuration"
                      << std::endl;
            return false;
        }
    }
    if (!cfg.detect.enabled && !cfg.seg.enabled) {
        std::cerr << "[HorizonDnn] detect and segmentation are both disabled"
                  << std::endl;
        return false;
    }
    return true;
}

bool GetInputHW(const hbDNNTensorProperties& props, int32_t* h, int32_t* w) {
    if (!h || !w || props.validShape.numDimensions != 4 ||
        props.validShape.dimensionSize[0] != 1) {
        return false;
    }
    // Horizon image tensors used by these converted models expose H/W in
    // dimensions 2/3. Validate the contract before indexing it.
    *h = props.validShape.dimensionSize[2];
    *w = props.validShape.dimensionSize[3];
    return ValidOrigSize(*w, *h) && ((*w & 1) == 0) && ((*h & 1) == 0);
}

bool IsSupportedOutputType(int tensor_type) {
    return tensor_type == HB_DNN_TENSOR_TYPE_F32 ||
           tensor_type == HB_DNN_TENSOR_TYPE_S8 ||
           tensor_type == HB_DNN_TENSOR_TYPE_U8 ||
           tensor_type == HB_DNN_TENSOR_TYPE_S16 ||
           tensor_type == HB_DNN_TENSOR_TYPE_S32;
}

int OutputElementBytes(int tensor_type) {
    if (tensor_type == HB_DNN_TENSOR_TYPE_S8 ||
        tensor_type == HB_DNN_TENSOR_TYPE_U8) {
        return 1;
    }
    if (tensor_type == HB_DNN_TENSOR_TYPE_S16) return 2;
    if (tensor_type == HB_DNN_TENSOR_TYPE_F32 ||
        tensor_type == HB_DNN_TENSOR_TYPE_S32) {
        return 4;
    }
    return 0;
}

bool TensorStorageCoversAlignedShape(const hbDNNTensorProperties& props) {
    const int element_bytes = OutputElementBytes(props.tensorType);
    if (element_bytes <= 0 || props.alignedByteSize <= 0 ||
        props.validShape.numDimensions != 4 ||
        props.alignedShape.numDimensions != 4 ||
        props.validShape.dimensionSize[0] != 1) {
        return false;
    }
    std::int64_t elements = 1;
    for (int i = 0; i < 4; ++i) {
        const int valid_dimension = props.validShape.dimensionSize[i];
        const int aligned_dimension = props.alignedShape.dimensionSize[i];
        if (valid_dimension <= 0 || aligned_dimension < valid_dimension ||
            elements > std::numeric_limits<std::int64_t>::max() /
                           aligned_dimension) {
            return false;
        }
        elements *= aligned_dimension;
    }
    return elements <= props.alignedByteSize / element_bytes;
}

bool TensorMemoryCoversProperties(const hbDNNTensor& tensor) {
    return tensor.sysMem[0].virAddr != nullptr &&
           tensor.properties.alignedByteSize > 0 &&
           tensor.sysMem[0].memSize >=
               static_cast<std::uint32_t>(tensor.properties.alignedByteSize);
}

bool IsSupportedTensorLayout(int layout) {
    return layout == HB_DNN_LAYOUT_NCHW || layout == HB_DNN_LAYOUT_NHWC;
}

bool IsSupportedQuantization(int quanti_type) {
    return quanti_type == NONE || quanti_type == SHIFT || quanti_type == SCALE;
}

bool ValidateOutputEncoding(const hbDNNTensorProperties& props, int channels) {
    if (channels <= 0 || !IsSupportedTensorLayout(props.tensorLayout) ||
        !IsSupportedOutputType(props.tensorType) ||
        !IsSupportedQuantization(props.quantiType) ||
        !TensorStorageCoversAlignedShape(props)) {
        return false;
    }
    // Floating-point output is already dequantized. Accepting SHIFT/SCALE here
    // would pass validation while the readers interpret the payload as float,
    // silently ignoring contradictory quantization metadata.
    if (props.tensorType == HB_DNN_TENSOR_TYPE_F32) {
        return props.quantiType == NONE;
    }
    const int channel_axis =
        props.tensorLayout == HB_DNN_LAYOUT_NCHW ? 1 : 3;
    if (props.quantiType == NONE) return true;
    if (props.quantiType == SHIFT) {
        if (props.shift.shiftLen <= 0 || !props.shift.shiftData ||
            (props.shift.shiftLen != 1 && props.shift.shiftLen != channels) ||
            (props.shift.shiftLen > 1 && props.quantizeAxis != channel_axis)) {
            return false;
        }
        for (int i = 0; i < props.shift.shiftLen; ++i) {
            const int shift = props.shift.shiftData[i];
            if (shift < -126 || shift > 126) return false;
        }
        return true;
    }
    if (props.scale.scaleLen <= 0 || !props.scale.scaleData ||
        (props.scale.scaleLen != 1 && props.scale.scaleLen != channels) ||
        (props.scale.scaleLen > 1 && props.quantizeAxis != channel_axis) ||
        (props.scale.zeroPointLen != 0 &&
         props.scale.zeroPointLen != props.scale.scaleLen) ||
        (props.scale.zeroPointLen > 0 && !props.scale.zeroPointData)) {
        return false;
    }
    for (int i = 0; i < props.scale.scaleLen; ++i) {
        if (!(props.scale.scaleData[i] > 0.0F) ||
            !std::isfinite(props.scale.scaleData[i])) {
            return false;
        }
    }
    return true;
}

int QuantParamIndex(const hbDNNTensorProperties& props, int parameter_count,
                    int channel, int fallback_channel_axis) {
    if (parameter_count <= 1) return 0;
    const int channel_axis = props.tensorLayout == HB_DNN_LAYOUT_NCHW
                                 ? 1
                                 : (props.tensorLayout == HB_DNN_LAYOUT_NHWC
                                        ? 3
                                        : fallback_channel_axis);
    if (props.quantizeAxis != channel_axis || channel < 0 ||
        channel >= parameter_count) {
        return -1;
    }
    return channel;
}

float ReadOutputFloatDetect(const hbDNNTensor& tensor, int index, int channel) {
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
    } else if (props.tensorType == HB_DNN_TENSOR_TYPE_S32) {
        raw = reinterpret_cast<int32_t*>(base)[index];
    } else {
        return std::numeric_limits<float>::quiet_NaN();
    }
    if (props.quantiType == SHIFT) {
        if (props.shift.shiftLen > 0 && !props.shift.shiftData) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        const int parameter_index =
            QuantParamIndex(props, props.shift.shiftLen, channel,
                            /*legacy detection channel axis=*/3);
        if (parameter_index < 0) return std::numeric_limits<float>::quiet_NaN();
        const int32_t shift = props.shift.shiftLen > 0
                                  ? props.shift.shiftData[parameter_index]
                                  : 0;
        if (shift < -126 || shift > 126) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return std::ldexp(static_cast<float>(raw), -shift);
    }
    if (props.quantiType == SCALE) {
        if (props.scale.scaleLen > 0 && !props.scale.scaleData) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        const int parameter_index =
            QuantParamIndex(props, props.scale.scaleLen, channel,
                            /*legacy detection channel axis=*/3);
        if (parameter_index < 0) return std::numeric_limits<float>::quiet_NaN();
        const float scale = props.scale.scaleLen > 0
                                ? props.scale.scaleData[parameter_index]
                                : 1.f;
        int32_t zero_point = 0;
        if (props.scale.zeroPointLen > 0) {
            if (!props.scale.zeroPointData ||
                props.scale.zeroPointLen != props.scale.scaleLen) {
                return std::numeric_limits<float>::quiet_NaN();
            }
            zero_point = props.scale.zeroPointData[parameter_index];
        }
        return dequantizeScale(raw, zero_point, scale);
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
        return std::numeric_limits<float>::quiet_NaN();
    }
    if (props.quantiType == SHIFT) {
        if (props.shift.shiftLen > 0 && !props.shift.shiftData) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        const int parameter_index =
            QuantParamIndex(props, props.shift.shiftLen, channel,
                            /*legacy segmentation channel axis=*/1);
        if (parameter_index < 0) return std::numeric_limits<float>::quiet_NaN();
        const int32_t shift = props.shift.shiftLen > 0
                                  ? props.shift.shiftData[parameter_index]
                                  : 0;
        if (shift < -126 || shift > 126) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return std::ldexp(static_cast<float>(raw), -shift);
    }
    if (props.quantiType == SCALE) {
        if (props.scale.scaleLen > 0 && !props.scale.scaleData) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        const int parameter_index =
            QuantParamIndex(props, props.scale.scaleLen, channel,
                            /*legacy segmentation channel axis=*/1);
        if (parameter_index < 0) return std::numeric_limits<float>::quiet_NaN();
        const float scale = props.scale.scaleLen > 0
                                ? props.scale.scaleData[parameter_index]
                                : 1.f;
        int32_t zero_point = 0;
        if (props.scale.zeroPointLen > 0) {
            if (!props.scale.zeroPointData ||
                props.scale.zeroPointLen != props.scale.scaleLen) {
                return std::numeric_limits<float>::quiet_NaN();
            }
            zero_point = props.scale.zeroPointData[parameter_index];
        }
        return dequantizeScale(raw, zero_point, scale);
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
    if (!src || !ValidOrigSize(src_w, src_h) ||
        !ValidOrigSize(dst_w, dst_h)) {
        return false;
    }
    return resizeRgb24Bilinear(src, src_w, src_h, &dst, dst_w, dst_h);
}

bool LetterboxRgb(const uint8_t* src, int src_w, int src_h,
                   std::vector<uint8_t>& canvas, int dst_w, int dst_h,
                   LetterboxMeta* meta, uint8_t pad) {
    if (!src || !meta || !ValidOrigSize(src_w, src_h) ||
        !ValidOrigSize(dst_w, dst_h)) {
        return false;
    }
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
    meta->x_scale = static_cast<float>(new_w) / src_w;
    meta->y_scale = static_cast<float>(new_h) / src_h;
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
    if (resize_type != 0 && resize_type != 1) return false;
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
    // The resize actually uses the even dimensions above. Record those exact
    // per-axis scales so inverse box/mask mapping does not use the pre-rounding
    // floating-point scale.
    meta->x_scale = static_cast<float>(new_w) / src_w;
    meta->y_scale = static_cast<float>(new_h) / src_h;
    // NV12 chroma samples cover 2x2 luma pixels. Keep both offsets even so
    // the interleaved UV plane starts on the same chroma grid as the Y plane.
    meta->x_shift = ((dst_w - new_w) / 2) & ~1;
    meta->y_shift = ((dst_h - new_h) / 2) & ~1;
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
    if (resize_type != 0 && resize_type != 1) return false;
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
    if (!ValidOrigSize(w, h) || (w % 2) != 0 || (h % 2) != 0 ||
        input.properties.alignedByteSize <= 0) {
        return false;
    }
    const size_t need = static_cast<size_t>(w) * h * 3 / 2;
    if (nv12.size() < need || !TensorMemoryCoversProperties(input)) {
        return false;
    }
    const size_t cap = input.properties.alignedByteSize;
    if (cap < need) {
        std::cerr << "[HorizonDnn] input tensor too small need=" << need
                  << " capacity=" << cap << std::endl;
        return false;
    }
    std::memset(input.sysMem[0].virAddr, 0, cap);
    std::memcpy(input.sysMem[0].virAddr, nv12.data(), need);
    const int flush_ret =
        hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    if (flush_ret != 0) {
        std::cerr << "[HorizonDnn] input cache clean failed code="
                  << flush_ret << std::endl;
        return false;
    }
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
    const size_t uv_off = static_cast<size_t>(cy / 2) *
                              static_cast<size_t>(stride) +
                          static_cast<size_t>(cx / 2) * 2U;
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
    return -1;
}

void GetOutputShape(const hbDNNTensorProperties& props, int* num_classes,
                    int* out_h, int* out_w) {
    const int layout = GetTensorLayout(props);
    if (layout == HB_DNN_LAYOUT_NCHW) {
        *num_classes = props.validShape.dimensionSize[1];
        *out_h = props.validShape.dimensionSize[2];
        *out_w = props.validShape.dimensionSize[3];
    } else if (layout == HB_DNN_LAYOUT_NHWC) {
        *out_h = props.validShape.dimensionSize[1];
        *out_w = props.validShape.dimensionSize[2];
        *num_classes = props.validShape.dimensionSize[3];
    } else {
        *num_classes = 0;
        *out_h = 0;
        *out_w = 0;
    }
}

void GetDetectionOutputShape(const hbDNNTensorProperties& props, int* channels,
                             int* out_h, int* out_w) {
    if (props.tensorLayout == HB_DNN_LAYOUT_NCHW) {
        *channels = props.validShape.dimensionSize[1];
        *out_h = props.validShape.dimensionSize[2];
        *out_w = props.validShape.dimensionSize[3];
        return;
    }
    if (props.tensorLayout == HB_DNN_LAYOUT_NHWC) {
        *out_h = props.validShape.dimensionSize[1];
        *out_w = props.validShape.dimensionSize[2];
        *channels = props.validShape.dimensionSize[3];
        return;
    }
    *channels = 0;
    *out_h = 0;
    *out_w = 0;
}

int GetDetectionOutputIndex(const hbDNNTensorProperties& props, int y, int x,
                            int channel) {
    if (props.alignedShape.numDimensions != 4 || y < 0 || x < 0 ||
        channel < 0) {
        return -1;
    }
    std::int64_t index = -1;
    if (props.tensorLayout == HB_DNN_LAYOUT_NCHW) {
        const int aligned_c = props.alignedShape.dimensionSize[1];
        const int aligned_h = props.alignedShape.dimensionSize[2];
        const int aligned_w = props.alignedShape.dimensionSize[3];
        if (channel >= aligned_c || y >= aligned_h || x >= aligned_w) return -1;
        index = (static_cast<std::int64_t>(channel) * aligned_h + y) *
                    aligned_w +
                x;
    } else if (props.tensorLayout == HB_DNN_LAYOUT_NHWC) {
        const int aligned_h = props.alignedShape.dimensionSize[1];
        const int aligned_w = props.alignedShape.dimensionSize[2];
        const int aligned_c = props.alignedShape.dimensionSize[3];
        if (y >= aligned_h || x >= aligned_w || channel >= aligned_c) return -1;
        index = (static_cast<std::int64_t>(y) * aligned_w + x) * aligned_c +
                channel;
    } else {
        return -1;
    }
    return index >= 0 && index <= std::numeric_limits<int>::max()
               ? static_cast<int>(index)
               : -1;
}

int GetOutputIndex(const hbDNNTensorProperties& props, int c, int y, int x) {
    const hbDNNTensorShape& aligned = props.alignedShape;
    if (aligned.numDimensions != 4 || c < 0 || y < 0 || x < 0) return -1;
    const int layout = GetTensorLayout(props);
    if (layout < 0) return -1;
    std::int64_t index = -1;
    if (layout == HB_DNN_LAYOUT_NCHW) {
        const int ac = aligned.dimensionSize[1];
        const int ah = aligned.dimensionSize[2];
        const int aw = aligned.dimensionSize[3];
        if (c >= ac || y >= ah || x >= aw) return -1;
        index = (static_cast<std::int64_t>(c) * ah + y) * aw + x;
    } else {
        const int ah = aligned.dimensionSize[1];
        const int aw = aligned.dimensionSize[2];
        const int ac = aligned.dimensionSize[3];
        if (y >= ah || x >= aw || c >= ac) return -1;
        index = (static_cast<std::int64_t>(y) * aw + x) * ac + c;
    }
    return index >= 0 && index <= std::numeric_limits<int>::max()
               ? static_cast<int>(index)
               : -1;
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
    if (!TensorMemoryCoversProperties(output)) return false;
    if (!ValidateOutputEncoding(output.properties, num_classes)) {
        std::cerr << "[HorizonDnn] unsupported segmentation output encoding"
                  << std::endl;
        return false;
    }
    const int flush_ret = hbSysFlushMem(
        const_cast<hbSysMem*>(&output.sysMem[0]),
        HB_SYS_MEM_CACHE_INVALIDATE);
    if (flush_ret != 0) {
        std::cerr << "[HorizonDnn] seg output cache invalidate failed code="
                  << flush_ret << std::endl;
        return false;
    }
    const hbDNNTensorProperties& props = output.properties;
    const size_t plane = static_cast<size_t>(out_h) * out_w;
    chw.resize(static_cast<size_t>(num_classes) * plane);
    // 通道在外：NCHW 连续读；每个输出格只反量化一次（与逐点 ReadOutputFloatSeg 数值相同）
    for (int c = 0; c < num_classes; ++c) {
        float* dst = chw.data() + static_cast<size_t>(c) * plane;
        for (int y = 0; y < out_h; ++y) {
            for (int x = 0; x < out_w; ++x) {
                const int index = GetOutputIndex(props, c, y, x);
                if (index < 0) return false;
                const float value = ReadOutputFloatSeg(output, index, c);
                if (!std::isfinite(value)) {
                    std::cerr << "[HorizonDnn] non-finite segmentation output"
                              << std::endl;
                    return false;
                }
                dst[static_cast<size_t>(y) * out_w + static_cast<size_t>(x)] =
                    value;
            }
        }
    }
    return true;
}

void ArgmaxFromChw(const std::vector<float>& chw, int num_classes, int out_h,
                   int out_w, std::vector<uint8_t>& mask) {
    const size_t plane = static_cast<size_t>(out_h) * out_w;
    mask.resize(plane);
    for (int y = 0; y < out_h; ++y) {
        for (int x = 0; x < out_w; ++x) {
            const size_t pix = static_cast<size_t>(y) * out_w + x;
            int best = 0;
            float best_val = chw[pix];
            for (int c = 1; c < num_classes; ++c) {
                const float val = chw[static_cast<size_t>(c) * plane + pix];
                if (val > best_val) {
                    best_val = val;
                    best = c;
                }
            }
            mask[pix] = static_cast<uint8_t>(best);
        }
    }
}

bool ArgmaxFromTensor(const hbDNNTensor& output, int num_classes, int out_h,
                      int out_w, std::vector<uint8_t>& mask) {
    std::vector<float> chw;
    if (!DequantSegToChw(output, num_classes, out_h, out_w, chw)) return false;
    ArgmaxFromChw(chw, num_classes, out_h, out_w, mask);
    return true;
}

float ResizeCoord(int dst_index, int dst_size, int src_size,
                  bool align_corners) {
    return resizeDestinationToSource(dst_index, dst_size, src_size,
                                     align_corners);
}

bool ArgmaxBilinearFromTensor(const hbDNNTensor& output, int num_classes,
                              int out_h, int out_w, int input_h, int input_w,
                              int orig_h, int orig_w, int resize_type,
                              const LetterboxMeta& meta, bool align_corners,
                              std::vector<uint8_t>& mask) {
    if (!ValidTensorHW(out_h, out_w, num_classes) ||
        !ValidOrigSize(orig_w, orig_h)) {
        return false;
    }
    // 仍按原图双线性 argmax（解决类别边缘锯齿），数值与逐点反量化+插值相同：
    // 1) 特征图只反量化一次到 CHW float
    // 2) 四角同类 / 权重退化为单点时，凸组合不会改 argmax，可直接写类别
    // 3) 仅真实类别边界对 4×C 个 float 做双线性
    std::vector<float> chw;
    if (!DequantSegToChw(output, num_classes, out_h, out_w, chw)) {
        return false;
    }
    std::vector<uint8_t> coarse_mask;
    ArgmaxFromChw(chw, num_classes, out_h, out_w, coarse_mask);
    const size_t plane = static_cast<size_t>(out_h) * out_w;
    mask.resize(static_cast<size_t>(orig_w) * orig_h);

    std::vector<int> x0_map(orig_w), x1_map(orig_w);
    std::vector<float> wx_map(orig_w);
    for (int x = 0; x < orig_w; ++x) {
        float ox = ResizeCoord(x, orig_w, out_w, align_corners);
        if (resize_type != 0) {
            const float valid_x =
                ResizeCoord(x, orig_w, meta.valid_w, align_corners);
            const float input_x = meta.x_shift + valid_x;
            ox = resizeDestinationToSource(input_x, input_w, out_w,
                                           align_corners);
        }
        ox = std::max(0.f, std::min(static_cast<float>(out_w - 1), ox));
        x0_map[x] = static_cast<int>(std::floor(ox));
        x1_map[x] = std::min(out_w - 1, x0_map[x] + 1);
        wx_map[x] = ox - x0_map[x];
    }

    auto logit = [&](int c, int y, int x) -> float {
        return chw[static_cast<size_t>(c) * plane +
                   static_cast<size_t>(y) * out_w + static_cast<size_t>(x)];
    };

    for (int y = 0; y < orig_h; ++y) {
        float oy = ResizeCoord(y, orig_h, out_h, align_corners);
        if (resize_type != 0) {
            const float valid_y =
                ResizeCoord(y, orig_h, meta.valid_h, align_corners);
            const float input_y = meta.y_shift + valid_y;
            oy = resizeDestinationToSource(input_y, input_h, out_h,
                                           align_corners);
        }
        oy = std::max(0.f, std::min(static_cast<float>(out_h - 1), oy));
        const int y0 = static_cast<int>(std::floor(oy));
        const int y1 = std::min(out_h - 1, y0 + 1);
        const float wy = oy - y0;
        const size_t row0 = static_cast<size_t>(y0) * out_w;
        const size_t row1 = static_cast<size_t>(y1) * out_w;
        for (int x = 0; x < orig_w; ++x) {
            const int x0 = x0_map[x];
            const int x1 = x1_map[x];
            const uint8_t c00 = coarse_mask[row0 + x0];
            const uint8_t c01 = coarse_mask[row0 + x1];
            const uint8_t c10 = coarse_mask[row1 + x0];
            const uint8_t c11 = coarse_mask[row1 + x1];
            const float wx = wx_map[x];
            // 凸组合保持 argmax：单点、水平边、垂直边、四角同类
            if ((wx == 0.f && wy == 0.f) ||
                (c00 == c01 && c00 == c10 && c00 == c11) ||
                (wx == 0.f && c00 == c10) || (wy == 0.f && c00 == c01)) {
                mask[static_cast<size_t>(y) * orig_w + x] = c00;
                continue;
            }
            int best = 0;
            float best_val = -std::numeric_limits<float>::infinity();
            for (int c = 0; c < num_classes; ++c) {
                const float v00 = logit(c, y0, x0);
                const float v01 = logit(c, y0, x1);
                const float v10 = logit(c, y1, x0);
                const float v11 = logit(c, y1, x1);
                const float top = v00 + (v01 - v00) * wx;
                const float bottom = v10 + (v11 - v10) * wx;
                const float val = top + (bottom - top) * wy;
                if (val > best_val) {
                    best_val = val;
                    best = c;
                }
            }
            mask[static_cast<size_t>(y) * orig_w + x] =
                static_cast<uint8_t>(best);
        }
    }
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
    if (!dnn_handle || reg <= 0 || reg > kMaxClasses) return -1;
    int output_count = 0;
    hbDNNGetOutputCount(&output_count, dnn_handle);
    const int bbox_channels = reg * 4;
    int inferred = -1;
    for (int i = 0; i < output_count; ++i) {
        hbDNNTensorProperties props;
        if (hbDNNGetOutputTensorProperties(&props, dnn_handle, i) != 0) continue;
        if (props.validShape.numDimensions != 4 ||
            props.validShape.dimensionSize[0] != 1 ||
            !IsSupportedTensorLayout(props.tensorLayout)) {
            continue;
        }
        int c = 0;
        int h = 0;
        int w = 0;
        GetDetectionOutputShape(props, &c, &h, &w);
        if (!ValidTensorHW(h, w, c)) continue;
        if (c == bbox_channels) continue;
        if (inferred < 0) inferred = c;
    }
    return inferred > 0 ? inferred : 80;
}

int MapOutputOrder(hbDNNHandle_t dnn_handle, int input_h, int input_w,
                   int classes_num, int reg, int order[6]) {
    if (!dnn_handle || !order || !ValidOrigSize(input_w, input_h) ||
        classes_num <= 0 || classes_num > kMaxClasses || reg <= 0 ||
        reg > kMaxClasses) {
        return -1;
    }
    const int32_t expected[6][3] = {
        {input_h / 8, input_w / 8, classes_num},
        {input_h / 8, input_w / 8, reg * 4},
        {input_h / 16, input_w / 16, classes_num},
        {input_h / 16, input_w / 16, reg * 4},
        {input_h / 32, input_w / 32, classes_num},
        {input_h / 32, input_w / 32, reg * 4}};
    bool used[6] = {false, false, false, false, false, false};
    for (int i = 0; i < 6; ++i) {
        order[i] = -1;
        for (int j = 0; j < 6; ++j) {
            if (used[j]) continue;
            hbDNNTensorProperties props;
            if (hbDNNGetOutputTensorProperties(&props, dnn_handle, j) != 0) {
                continue;
            }
            if (props.validShape.numDimensions != 4 ||
                props.validShape.dimensionSize[0] != 1 ||
                !IsSupportedTensorLayout(props.tensorLayout)) {
                continue;
            }
            int c = 0;
            int h = 0;
            int w = 0;
            GetDetectionOutputShape(props, &c, &h, &w);
            if (h == expected[i][0] && w == expected[i][1] && c == expected[i][2]) {
                order[i] = j;
                used[j] = true;
                break;
            }
        }
        if (order[i] < 0) return -1;
    }
    return 0;
}

float BoxIoU(const RawBox& a, const RawBox& b) {
    const float ax2 = a.x + a.w;
    const float ay2 = a.y + a.h;
    const float bx2 = b.x + b.w;
    const float by2 = b.y + b.h;
    const float ix1 = std::max(a.x, b.x);
    const float iy1 = std::max(a.y, b.y);
    const float ix2 = std::min(ax2, bx2);
    const float iy2 = std::min(ay2, by2);
    const float iw = std::max(0.f, ix2 - ix1);
    const float ih = std::max(0.f, iy2 - iy1);
    const float inter = iw * ih;
    const float uni = a.w * a.h + b.w * b.h - inter;
    return uni > 0.f ? inter / uni : 0.f;
}

void NmsPerClass(std::vector<RawDetection>& dets, float nms_threshold,
                 int nms_top_k) {
    std::sort(dets.begin(), dets.end(),
              [](const RawDetection& a, const RawDetection& b) {
                  return a.confidence > b.confidence;
              });
    std::vector<RawDetection> kept;
    std::vector<bool> removed(dets.size(), false);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (removed[i]) continue;
        kept.push_back(dets[i]);
        if (static_cast<int>(kept.size()) >= nms_top_k) break;
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (removed[j] || dets[i].class_id != dets[j].class_id) continue;
            if (BoxIoU(dets[i].box, dets[j].box) > nms_threshold) {
                removed[j] = true;
            }
        }
    }
    dets.swap(kept);
}

int PostProcessDetections(hbDNNTensor* outputs, const int order[6], int input_h,
                          int input_w, int classes_num,
                          const std::vector<int>& active_class_ids, int reg,
                          float score_threshold, float nms_threshold,
                          int nms_top_k, std::vector<RawDetection>* out) {
    if (!outputs || !order || !out || !ValidTensorHW(input_h, input_w, 1) ||
        classes_num <= 0 || classes_num > kMaxClasses || reg <= 0 ||
        reg > kMaxClasses || nms_top_k <= 0 || nms_top_k > kMaxNmsTopK ||
        active_class_ids.empty() ||
        !(score_threshold > 0.f && score_threshold < 1.f) ||
        !std::isfinite(score_threshold) ||
        !(nms_threshold >= 0.f && nms_threshold <= 1.f) ||
        !std::isfinite(nms_threshold)) {
        std::cerr << "[HorizonDnn] invalid detection postprocess arguments"
                  << std::endl;
        return -1;
    }
    int previous_class_id = -1;
    for (const int class_id : active_class_ids) {
        if (class_id < 0 || class_id >= classes_num ||
            class_id <= previous_class_id) {
            std::cerr << "[HorizonDnn] invalid active detection class IDs"
                      << std::endl;
            return -1;
        }
        previous_class_id = class_id;
    }
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

        if (!TensorMemoryCoversProperties(outputs[cls_output_idx]) ||
            !TensorMemoryCoversProperties(outputs[bbox_output_idx])) {
            std::cerr << "[HorizonDnn] detect output memory is null"
                      << std::endl;
            return -1;
        }
        if (!ValidateOutputEncoding(outputs[cls_output_idx].properties,
                                    classes_num) ||
            !ValidateOutputEncoding(outputs[bbox_output_idx].properties,
                                    reg * 4)) {
            std::cerr << "[HorizonDnn] unsupported detection output encoding"
                      << std::endl;
            return -1;
        }
        int flush_ret = hbSysFlushMem(
            &outputs[cls_output_idx].sysMem[0],
            HB_SYS_MEM_CACHE_INVALIDATE);
        if (flush_ret != 0) {
            std::cerr << "[HorizonDnn] detect class cache invalidate failed code="
                      << flush_ret << std::endl;
            return -1;
        }
        flush_ret = hbSysFlushMem(
            &outputs[bbox_output_idx].sysMem[0],
            HB_SYS_MEM_CACHE_INVALIDATE);
        if (flush_ret != 0) {
            std::cerr << "[HorizonDnn] detect box cache invalidate failed code="
                      << flush_ret << std::endl;
            return -1;
        }

        for (int gh = 0; gh < h; ++gh) {
            for (int gw = 0; gw < w; ++gw) {
                int cls_id = -1;
                float max_logit = 0.f;
                const bool selected = detail::SelectBestActiveDetectionClass(
                    active_class_ids,
                    [&](int class_id, float* logit) {
                        const int cls_index = GetDetectionOutputIndex(
                            outputs[cls_output_idx].properties, gh, gw,
                            class_id);
                        if (cls_index < 0 || !logit) return false;
                        *logit = ReadOutputFloatDetect(
                            outputs[cls_output_idx], cls_index, class_id);
                        return true;
                    },
                    &cls_id, &max_logit);
                if (!selected) return -1;
                if (!std::isfinite(max_logit) || max_logit < conf_thres_raw) {
                    continue;
                }

                const float score = 1.f / (1.f + std::exp(-max_logit));
                float ltrb[4] = {0.f, 0.f, 0.f, 0.f};
                bool dfl_valid = true;
                for (int i = 0; i < 4; ++i) {
                    float max_dfl_logit = -std::numeric_limits<float>::infinity();
                    for (int j = 0; j < reg; ++j) {
                        const int channel = reg * i + j;
                        const int idx = GetDetectionOutputIndex(
                            outputs[bbox_output_idx].properties, gh, gw,
                            channel);
                        if (idx < 0) {
                            dfl_valid = false;
                            break;
                        }
                        max_dfl_logit = std::max(
                            max_dfl_logit,
                            ReadOutputFloatDetect(outputs[bbox_output_idx], idx,
                                                  channel));
                    }
                    if (!dfl_valid || !std::isfinite(max_dfl_logit)) break;
                    float sum = 0.f;
                    for (int j = 0; j < reg; ++j) {
                        const int channel = reg * i + j;
                        const int idx = GetDetectionOutputIndex(
                            outputs[bbox_output_idx].properties, gh, gw,
                            channel);
                        if (idx < 0) {
                            dfl_valid = false;
                            break;
                        }
                        const float dfl = std::exp(ReadOutputFloatDetect(
                                                       outputs[bbox_output_idx],
                                                       idx, channel) -
                                                   max_dfl_logit);
                        ltrb[i] += dfl * static_cast<float>(j);
                        sum += dfl;
                    }
                    if (!dfl_valid || !(sum > 0.f) || !std::isfinite(sum)) {
                        dfl_valid = false;
                        break;
                    }
                    ltrb[i] /= sum;
                }
                if (!dfl_valid) continue;
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

    // Bound the quadratic NMS input while reserving capacity for every active
    // class. A global confidence-only cut lets many overlapping candidates of
    // one frequent class evict all candidates of a less frequent class before
    // class-aware NMS gets a chance to suppress the duplicates.
    const std::size_t pre_nms_limit = static_cast<std::size_t>(
        std::max(1024, nms_top_k * 4));
    if (candidates.size() > pre_nms_limit) {
        std::vector<detail::DetectionCandidateRank> ranks;
        ranks.reserve(candidates.size());
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            ranks.push_back({candidates[index].class_id,
                             candidates[index].confidence, index});
        }
        const std::vector<std::size_t> selected =
            detail::SelectClassAwareCandidateIndices(ranks, pre_nms_limit);
        std::vector<RawDetection> bounded;
        bounded.reserve(selected.size());
        for (const std::size_t index : selected) {
            if (index < candidates.size()) bounded.push_back(candidates[index]);
        }
        candidates.swap(bounded);
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
        const int color_index = det.class_id >= 0 ? det.class_id % 8 : 0;
        const auto& c = kColors[color_index];
        DrawRect(rgb, w, h, static_cast<int>(det.x1), static_cast<int>(det.y1),
                 static_cast<int>(det.x2), static_cast<int>(det.y2), c[0], c[1],
                 c[2], 2);
    }
}

// A single-slot worker is sufficient because HorizonDnnRunner serializes
// infer calls with infer_mtx_. The thread lives for the whole committed DNN
// session; only the small packaged job changes from frame to frame.
class PersistentBoolWorker final {
public:
    PersistentBoolWorker()
        : worker_(&PersistentBoolWorker::run, this) {}

    ~PersistentBoolWorker() { shutdown(); }

    PersistentBoolWorker(const PersistentBoolWorker&) = delete;
    PersistentBoolWorker& operator=(const PersistentBoolWorker&) = delete;

    std::future<bool> submit(std::function<bool()> work) {
        if (!work) {
            throw std::invalid_argument("cannot submit an empty DNN job");
        }

        auto task =
            std::make_shared<std::packaged_task<bool()>>(std::move(work));
        std::future<bool> result = task->get_future();
        std::function<void()> prepared = [task]() { (*task)(); };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_) {
                throw std::runtime_error("DNN worker is stopping");
            }
            if (pending_) {
                // infer_mtx_ makes this an invariant violation rather than
                // normal back-pressure.
                throw std::logic_error("DNN worker already has a pending job");
            }
            pending_.emplace(std::move(prepared));
        }
        condition_.notify_one();
        return result;
    }

private:
    void shutdown() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void run() noexcept {
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() {
                    return stopping_ || pending_.has_value();
                });
                if (!pending_) {
                    return;
                }
                job = std::move(*pending_);
                pending_.reset();
            }

            try {
                job();
            } catch (const std::exception& error) {
                // packaged_task normally captures worker exceptions in its
                // future. Keep this boundary so an executor defect cannot
                // silently kill the persistent thread.
                std::cerr << "[HorizonDnn] worker boundary exception: "
                          << error.what() << std::endl;
            } catch (...) {
                std::cerr << "[HorizonDnn] worker boundary unknown exception"
                          << std::endl;
            }
        }
    }

    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_{false};
    std::optional<std::function<void()>> pending_;
    std::thread worker_;
};

class ScopedDnnTask final {
public:
    ScopedDnnTask() = default;

    ~ScopedDnnTask() {
        const int release_ret = release();
        if (release_ret != 0) {
            std::cerr << "[HorizonDnn] hbDNNReleaseTask during cleanup code="
                      << release_ret << std::endl;
        }
    }

    ScopedDnnTask(const ScopedDnnTask&) = delete;
    ScopedDnnTask& operator=(const ScopedDnnTask&) = delete;

    hbDNNTaskHandle_t* outParam() noexcept { return &task_; }
    hbDNNTaskHandle_t get() const noexcept { return task_; }

    int release() noexcept {
        if (!task_) return 0;
        const hbDNNTaskHandle_t task = task_;
        task_ = nullptr;
        return hbDNNReleaseTask(task);
    }

private:
    hbDNNTaskHandle_t task_{nullptr};
};

bool RunInfer(hbDNNHandle_t handle, hbDNNTensor* input,
              std::vector<hbDNNTensor>* outputs) {
    if (!handle || !input || !outputs || outputs->empty()) {
        std::cerr << "[HorizonDnn] invalid inference handles" << std::endl;
        return false;
    }
    ScopedDnnTask task;
    hbDNNTensor* out_ptr = outputs->data();
    hbDNNInferCtrlParam ctrl;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
    HB_OK(hbDNNInfer(task.outParam(), &out_ptr, input, handle, &ctrl),
          "hbDNNInfer");
    if (!task.get()) {
        std::cerr << "[HorizonDnn] hbDNNInfer returned a null task"
                  << std::endl;
        return false;
    }
    HB_OK(hbDNNWaitTaskDone(task.get(), 0), "hbDNNWaitTaskDone");
    HB_OK(task.release(), "hbDNNReleaseTask");
    return true;
}

}  // namespace

struct HorizonDnnRunner::ParallelExecutor {
    PersistentBoolWorker detect_worker;
    PersistentBoolWorker seg_worker;
};

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
    std::vector<int> active_class_ids;

    ~DetectEngine() { release(); }

    void release() {
        ReleaseTensorMemory(&input, "detect input");
        for (auto& t : outputs) {
            ReleaseTensorMemory(&t, "detect output");
        }
        outputs.clear();
        active_class_ids.clear();
        if (packed) {
            ReleasePackedModel(&packed, "detect model");
            handle = nullptr;
        }
    }

    bool init(const DnnDetectConfig& cfg) {
        release();
        if (cfg.reg <= 0 || cfg.reg > kMaxClasses || cfg.classes_num < 0 ||
            cfg.classes_num > kMaxClasses) {
            std::cerr << "[HorizonDnn] invalid detect class/reg configuration"
                      << std::endl;
            return false;
        }
        class_names = ParseCsv(cfg.class_names);
        classes_num = cfg.classes_num;
        const char* model = cfg.model_path.c_str();
        HB_OK(hbDNNInitializeFromFiles(&packed, &model, 1),
              "detect hbDNNInitializeFromFiles");
        const char** names = nullptr;
        int count = 0;
        HB_OK(hbDNNGetModelNameList(&names, &count, packed),
              "detect hbDNNGetModelNameList");
        if (!names || count <= 0 || !names[0]) {
            std::cerr << "[HorizonDnn] detect model package is empty"
                      << std::endl;
            return false;
        }
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
        if (input.properties.alignedByteSize <= 0) {
            std::cerr << "[HorizonDnn] detect input has invalid storage size"
                      << std::endl;
            return false;
        }
        if (!AllocateCachedTensor(&input, "detect input")) return false;
        input.properties.alignedShape = input.properties.validShape;

        outputs.resize(output_count);
        for (int i = 0; i < output_count; ++i) {
            HB_OK(hbDNNGetOutputTensorProperties(&outputs[i].properties, handle, i),
                  "detect output props");
            if (outputs[i].properties.alignedByteSize <= 0) {
                std::cerr << "[HorizonDnn] detect output has invalid storage size"
                          << std::endl;
                return false;
            }
            if (!AllocateCachedTensor(&outputs[i], "detect output")) {
                return false;
            }
        }

        if (!GetInputHW(input.properties, &input_h, &input_w)) {
            std::cerr << "[HorizonDnn] invalid detect input shape" << std::endl;
            return false;
        }
        if (classes_num <= 0) {
            classes_num = InferClassesNum(handle, cfg.reg);
        }
        if (classes_num <= 0 || classes_num > kMaxClasses) {
            std::cerr << "[HorizonDnn] invalid detect class count="
                      << classes_num << std::endl;
            return false;
        }
        if (static_cast<int>(class_names.size()) != classes_num) {
            std::cerr << "[HorizonDnn] detect class_names count="
                      << class_names.size() << " tensor classes=" << classes_num
                      << "; pad/truncate names to tensor" << std::endl;
            class_names.resize(static_cast<size_t>(classes_num));
            for (int i = 0; i < classes_num; ++i) {
                if (class_names[static_cast<size_t>(i)].empty()) {
                    class_names[static_cast<size_t>(i)] =
                        "cls_" + std::to_string(i);
                }
            }
        }
        const detail::DetectionClassFilter class_filter =
            detail::ResolveDetectionClassFilter(class_names, cfg.keep_classes);
        if (!class_filter) {
            std::cerr << "[HorizonDnn] " << class_filter.error << std::endl;
            return false;
        }
        active_class_ids = class_filter.active_class_ids;
        if (MapOutputOrder(handle, input_h, input_w, classes_num, cfg.reg,
                           order) != 0) {
            std::cerr << "[HorizonDnn] detect output order map failed"
                      << std::endl;
            return false;
        }
        for (int i = 0; i < 6; ++i) {
            const int channels = (i % 2) == 0 ? classes_num : cfg.reg * 4;
            if (!ValidateOutputEncoding(outputs[order[i]].properties,
                                        channels) ||
                !TensorMemoryCoversProperties(outputs[order[i]])) {
                std::cerr << "[HorizonDnn] detect output " << order[i]
                          << " has unsupported layout/quantization/storage"
                          << std::endl;
                return false;
            }
        }
        std::cout << "[HorizonDnn] detect model=" << cfg.model_path
                  << " input=" << input_w << "x" << input_h
                  << " classes=" << classes_num << std::endl;
        std::cout << "[HorizonDnn] detect active classes";
        for (const int class_id : active_class_ids) {
            std::cout << " " << class_id << "="
                      << class_names[static_cast<std::size_t>(class_id)];
        }
        std::cout << (class_filter.enabled ? " (filtered)" : " (all)")
                  << std::endl;
        return true;
    }

    bool infer(const uint8_t* rgb, int src_w, int src_h,
               const DnnDetectConfig& cfg, std::vector<DnnDetection>* out) {
        LetterboxMeta meta;
        std::vector<uint8_t> preprocessed;
        if (!PreprocessRgb(rgb, src_w, src_h, input_w, input_h, cfg.resize_type,
                           &meta, preprocessed, kRgbLetterboxPadForY127)) {
            return false;
        }
        std::vector<uint8_t> nv12;
        if (!Rgb24ToNv12(preprocessed.data(), input_w, input_h, nv12)) {
            return false;
        }
        if (!CopyNv12ToTensor(input, nv12, input_w, input_h)) {
            return false;
        }
        if (!RunInfer(handle, &input, &outputs)) return false;

        std::vector<RawDetection> raw;
        if (PostProcessDetections(outputs.data(), order, input_h, input_w,
                                  classes_num, active_class_ids, cfg.reg,
                                  cfg.score_threshold, cfg.nms_threshold,
                                  cfg.nms_top_k,
                                  &raw) != 0) {
            return false;
        }
        out->clear();
        out->reserve(raw.size());
        for (const auto& det : raw) {
            const RawDetection mapped =
                MapDetectionToOriginal(det, meta, src_w, src_h);
            if (!(mapped.box.w > 0.f && mapped.box.h > 0.f) ||
                !std::isfinite(mapped.box.x) || !std::isfinite(mapped.box.y) ||
                !std::isfinite(mapped.box.w) || !std::isfinite(mapped.box.h)) {
                continue;
            }
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
        std::vector<uint8_t> preprocessed;
        const auto t_pre = std::chrono::steady_clock::now();
        if (!PreprocessYuv420(yuv, src_w, src_h, src_stride, bytes_used,
                            src_is_nv12, input_w, input_h, cfg.resize_type,
                            &meta, preprocessed, 127)) {
            return false;
        }
        if (!CopyNv12ToTensor(input, preprocessed, input_w, input_h)) {
            return false;
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
                                  classes_num, active_class_ids, cfg.reg,
                                  cfg.score_threshold, cfg.nms_threshold,
                                  cfg.nms_top_k,
                                  &raw) != 0) {
            return false;
        }
        out->clear();
        out->reserve(raw.size());
        for (const auto& det : raw) {
            const RawDetection mapped =
                MapDetectionToOriginal(det, meta, src_w, src_h);
            if (!(mapped.box.w > 0.f && mapped.box.h > 0.f) ||
                !std::isfinite(mapped.box.x) || !std::isfinite(mapped.box.y) ||
                !std::isfinite(mapped.box.w) || !std::isfinite(mapped.box.h)) {
                continue;
            }
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
        ReleaseTensorMemory(&input, "seg input");
        for (auto& t : outputs) {
            ReleaseTensorMemory(&t, "seg output");
        }
        outputs.clear();
        if (packed) {
            ReleasePackedModel(&packed, "seg model");
            handle = nullptr;
        }
    }

    bool init(const DnnSegConfig& cfg) {
        release();
        num_classes = (cfg.num_classes > 0) ? cfg.num_classes : 0;
        const char* model = cfg.model_path.c_str();
        HB_OK(hbDNNInitializeFromFiles(&packed, &model, 1),
              "seg hbDNNInitializeFromFiles");
        const char** names = nullptr;
        int count = 0;
        HB_OK(hbDNNGetModelNameList(&names, &count, packed),
              "seg hbDNNGetModelNameList");
        if (!names || count <= 0 || !names[0]) {
            std::cerr << "[HorizonDnn] seg model package is empty" << std::endl;
            return false;
        }
        HB_OK(hbDNNGetModelHandle(&handle, packed, names[0]),
              "seg hbDNNGetModelHandle");

        int input_count = 0;
        int output_count = 0;
        HB_OK(hbDNNGetInputCount(&input_count, handle), "seg input count");
        HB_OK(hbDNNGetOutputCount(&output_count, handle), "seg output count");
        if (input_count != 1 || output_count <= 0) {
            std::cerr << "[HorizonDnn] seg model IO mismatch in/out="
                      << input_count << "/" << output_count << std::endl;
            return false;
        }

        HB_OK(hbDNNGetInputTensorProperties(&input.properties, handle, 0),
              "seg input props");
        if (input.properties.tensorType != HB_DNN_IMG_TYPE_NV12) {
            std::cerr << "[HorizonDnn] seg input must be NV12" << std::endl;
            return false;
        }
        if (input.properties.alignedByteSize <= 0) {
            std::cerr << "[HorizonDnn] seg input has invalid storage size"
                      << std::endl;
            return false;
        }
        if (!AllocateCachedTensor(&input, "seg input")) return false;
        input.properties.alignedShape = input.properties.validShape;

        outputs.resize(output_count);
        for (int i = 0; i < output_count; ++i) {
            HB_OK(hbDNNGetOutputTensorProperties(&outputs[i].properties, handle, i),
                  "seg output props");
            if (outputs[i].properties.alignedByteSize <= 0) {
                std::cerr << "[HorizonDnn] seg output has invalid storage size"
                          << std::endl;
                return false;
            }
            if (!AllocateCachedTensor(&outputs[i], "seg output")) {
                return false;
            }
        }

        if (!GetInputHW(input.properties, &input_h, &input_w)) {
            std::cerr << "[HorizonDnn] invalid seg input shape" << std::endl;
            return false;
        }
        if (cfg.output_index >= output_count) {
            std::cerr << "[HorizonDnn] seg output_index=" << cfg.output_index
                      << " is outside output_count=" << output_count << std::endl;
            return false;
        }
        if (cfg.output_index >= 0) {
            seg_output_idx = cfg.output_index;
        } else {
            seg_output_idx = 0;
            std::int64_t best_score = -1;
            for (int i = 0; i < output_count; ++i) {
                if (outputs[i].properties.validShape.numDimensions != 4 ||
                    outputs[i].properties.validShape.dimensionSize[0] != 1 ||
                    !IsSupportedTensorLayout(
                        outputs[i].properties.tensorLayout)) {
                    continue;
                }
                int nc = 0, oh = 0, ow = 0;
                GetOutputShape(outputs[i].properties, &nc, &oh, &ow);
                if (!ValidTensorHW(oh, ow, nc)) continue;
                std::int64_t score = static_cast<std::int64_t>(nc) * oh * ow;
                if (cfg.num_classes > 0 && nc == cfg.num_classes) {
                    score += 100000;
                }
                if (score > best_score) {
                    best_score = score;
                    seg_output_idx = i;
                }
            }
            if (best_score < 0) {
                std::cerr << "[HorizonDnn] no valid 4D segmentation output"
                          << std::endl;
                return false;
            }
            if (output_count > 1) {
                std::cerr << "[HorizonDnn] seg output selected heuristically idx="
                          << seg_output_idx
                          << "; configure seg.output_index after model inspection"
                          << std::endl;
            }
        }
        GetOutputShape(outputs[seg_output_idx].properties, &num_classes, &out_h,
                       &out_w);
        if (!ValidTensorHW(out_h, out_w, num_classes)) {
            std::cerr << "[HorizonDnn] invalid seg output shape nc=" << num_classes
                      << " " << out_w << "x" << out_h << std::endl;
            return false;
        }
        if (!ValidateOutputEncoding(outputs[seg_output_idx].properties,
                                    num_classes) ||
            !TensorMemoryCoversProperties(outputs[seg_output_idx])) {
            std::cerr << "[HorizonDnn] segmentation output has unsupported "
                         "layout/quantization/storage"
                      << std::endl;
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
                           &meta, preprocessed, kRgbLetterboxPadForY127)) {
            return false;
        }
        std::vector<uint8_t> nv12;
        if (!Rgb24ToNv12(preprocessed.data(), input_w, input_h, nv12)) {
            return false;
        }
        if (!CopyNv12ToTensor(input, nv12, input_w, input_h)) {
            return false;
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
        std::vector<uint8_t> preprocessed;
        const auto t_pre = std::chrono::steady_clock::now();
        if (!PreprocessYuv420(yuv, src_w, src_h, src_stride, bytes_used,
                            src_is_nv12, input_w, input_h, cfg.resize_type,
                            &meta, preprocessed, 127)) {
            return false;
        }
        if (!CopyNv12ToTensor(input, preprocessed, input_w, input_h)) {
            return false;
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
};

HorizonDnnRunner::HorizonDnnRunner() = default;

HorizonDnnRunner::~HorizonDnnRunner() {
    std::lock_guard<std::mutex> init_lock(init_mtx_);
    std::lock_guard<std::mutex> infer_lock(infer_mtx_);
    // Stop and join workers while the engines they may reference are alive.
    parallel_executor_.reset();
    detect_.reset();
    seg_.reset();
}

bool HorizonDnnRunner::init(const DnnConfig& cfg) {
    std::lock_guard<std::mutex> init_lock(init_mtx_);
    if (!ValidateRunnerConfig(cfg)) return false;
    DnnConfig next_cfg = cfg;
    std::unique_ptr<DetectEngine> next_detect;
    std::unique_ptr<SegEngine> next_seg;
    std::unique_ptr<ParallelExecutor> next_parallel_executor;
    bool next_detect_ready = false;
    bool next_seg_ready = false;
    int next_detect_input_w = 0;
    int next_detect_input_h = 0;
    int next_seg_input_w = 0;
    int next_seg_input_h = 0;

    try {
        // Build a complete next session in local RAII owners. Nothing
        // observable changes until every requested model and worker has
        // initialized successfully.
        if (next_cfg.enabled && next_cfg.detect.enabled &&
            !next_cfg.detect.model_path.empty()) {
            next_detect = std::make_unique<DetectEngine>();
            if (!next_detect->init(next_cfg.detect)) {
                std::cerr << "[HorizonDnn] detect init failed" << std::endl;
                return false;
            }
            next_detect_input_w = next_detect->input_w;
            next_detect_input_h = next_detect->input_h;
            next_cfg.detect.classes_num = next_detect->classes_num;
            next_cfg.detect.class_names = JoinCsv(next_detect->class_names);
            next_detect_ready = true;
        }
        if (next_cfg.enabled && next_cfg.seg.enabled &&
            !next_cfg.seg.model_path.empty()) {
            next_seg = std::make_unique<SegEngine>();
            if (!next_seg->init(next_cfg.seg)) {
                std::cerr << "[HorizonDnn] seg init failed" << std::endl;
                return false;
            }
            next_seg_input_w = next_seg->input_w;
            next_seg_input_h = next_seg->input_h;
            next_seg_ready = true;
            SyncSegClassNamesToTensor(&next_cfg.seg, next_seg->num_classes);
        }
        if (next_cfg.enabled && !next_detect_ready && !next_seg_ready) {
            std::cerr << "[HorizonDnn] no model enabled" << std::endl;
            return false;
        }
        if (next_cfg.enabled && next_cfg.parallel_det_seg &&
            next_detect_ready && next_seg_ready) {
            next_parallel_executor = std::make_unique<ParallelExecutor>();
        }
    } catch (const std::exception& error) {
        std::cerr << "[HorizonDnn] session init exception: " << error.what()
                  << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[HorizonDnn] session init unknown exception"
                  << std::endl;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(infer_mtx_);
        cfg_ = std::move(next_cfg);
        // Replacing an old executor stops and joins it before replacing the
        // engine objects below. infer_mtx_ guarantees it has no pending jobs.
        parallel_executor_ = std::move(next_parallel_executor);
        detect_ = std::move(next_detect);
        seg_ = std::move(next_seg);
        detect_ready_ = next_detect_ready;
        seg_ready_ = next_seg_ready;
        detect_input_w_ = next_detect_input_w;
        detect_input_h_ = next_detect_input_h;
        seg_input_w_ = next_seg_input_w;
        seg_input_h_ = next_seg_input_h;
    }

    if (!cfg_.enabled) {
        return true;
    }
    std::cout << "[HorizonDnn] ready detect=" << detect_ready_
              << " seg=" << seg_ready_
              << " parallel_det_seg=" << cfg_.parallel_det_seg
              << " run_every_n=" << cfg_.run_every_n
              << " input=yuv420_i420_or_nv12 seg_post="
              << cfg_.seg.argmax_mode
              << " (no-opencv path)" << std::endl;
    return true;
}

DnnConfig HorizonDnnRunner::config() const {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    return cfg_;
}

int HorizonDnnRunner::input_w_detect() const {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    return detect_input_w_;
}

int HorizonDnnRunner::input_h_detect() const {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    return detect_input_h_;
}

int HorizonDnnRunner::input_w_seg() const {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    return seg_input_w_;
}

int HorizonDnnRunner::input_h_seg() const {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    return seg_input_h_;
}

int HorizonDnnRunner::seg_num_classes() const {
    std::lock_guard<std::mutex> lock(infer_mtx_);
    return (seg_ready_ && seg_) ? seg_->num_classes : 0;
}

bool HorizonDnnRunner::infer(const uint8_t* rgb, int width, int height,
                             std::vector<DnnDetection>* detections,
                             std::vector<uint8_t>* seg_mask,
                             std::vector<uint8_t>* overlay_rgb,
                             DnnInferTiming* timing,
                             DnnInferStatus* status) {
    try {
    if (detections) detections->clear();
    if (seg_mask) seg_mask->clear();
    if (overlay_rgb) overlay_rgb->clear();
    if (timing) *timing = DnnInferTiming{};
    if (status) *status = DnnInferStatus{};
    if (!rgb || width <= 0 || height <= 0) return false;
    if (!ValidImage(width, height, static_cast<size_t>(width) * height * 3)) {
        std::cerr << "[HorizonDnn] invalid image size " << width << "x" << height
                  << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> infer_lock(infer_mtx_);
    if (!cfg_.enabled || (!detect_ && !seg_)) return false;

    std::vector<DnnDetection> dets;
    std::vector<uint8_t> local_seg;

    const bool detect_requested = detect_ready_;
    const bool want_seg = (seg_mask != nullptr) && seg_ready_;
    bool det_ok = false;
    bool seg_ok = false;
    if (detect_requested) {
        try {
            const auto t0 = std::chrono::steady_clock::now();
            det_ok = detect_->infer(rgb, width, height, cfg_.detect, &dets);
            if (timing) {
                timing->detect_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
            }
        } catch (const std::exception& error) {
            dets.clear();
            std::cerr << "[HorizonDnn] detect infer exception: "
                      << error.what() << std::endl;
        } catch (...) {
            dets.clear();
            std::cerr << "[HorizonDnn] detect infer unknown exception"
                      << std::endl;
        }
        if (!det_ok) {
            dets.clear();
            std::cerr << "[HorizonDnn] detect infer failed" << std::endl;
        }
    }
    if (want_seg) {
        try {
            seg_ok = seg_->infer(rgb, width, height, cfg_.seg, local_seg,
                                 timing);
        } catch (const std::exception& error) {
            local_seg.clear();
            std::cerr << "[HorizonDnn] seg infer exception: " << error.what()
                      << std::endl;
        } catch (...) {
            local_seg.clear();
            std::cerr << "[HorizonDnn] seg infer unknown exception"
                      << std::endl;
        }
        if (!seg_ok) {
            local_seg.clear();
            std::cerr << "[HorizonDnn] seg infer failed" << std::endl;
        }
    }

    if (status) {
        status->detect_requested = detect_requested;
        status->detect_succeeded = det_ok;
        status->segmentation_requested = want_seg;
        status->segmentation_succeeded = seg_ok;
    }
    if (detections && det_ok) *detections = dets;
    if (seg_mask && seg_ok) *seg_mask = local_seg;
    if (!det_ok && !seg_ok) return false;
    if (overlay_rgb && (cfg_.publish_overlay || cfg_.save_overlay)) {
        try {
            // Overlay is best-effort. A valid inference may legitimately have
            // no detections/mask pixels, in which case return the RGB base.
            overlay_rgb->assign(
                rgb, rgb + static_cast<size_t>(width) * height * 3);
            const std::vector<uint8_t> empty_seg;
            const auto& overlay_seg =
                cfg_.overlay_blend_seg ? local_seg : empty_seg;
            if (!dets.empty() || !overlay_seg.empty()) {
                DrawOverlay(overlay_rgb->data(), width, height, dets,
                            overlay_seg, cfg_.overlay_seg_alpha,
                            cfg_.seg.class_names);
            }
        } catch (const std::exception& error) {
            overlay_rgb->clear();
            std::cerr << "[HorizonDnn] overlay generation exception: "
                      << error.what() << std::endl;
        } catch (...) {
            overlay_rgb->clear();
            std::cerr << "[HorizonDnn] overlay generation unknown exception"
                      << std::endl;
        }
    }
    return det_ok || seg_ok;
    } catch (const std::exception& error) {
        if (detections) detections->clear();
        if (seg_mask) seg_mask->clear();
        if (overlay_rgb) overlay_rgb->clear();
        if (timing) *timing = DnnInferTiming{};
        if (status) *status = DnnInferStatus{};
        std::cerr << "[HorizonDnn] RGB infer exception: " << error.what()
                  << std::endl;
        return false;
    } catch (...) {
        if (detections) detections->clear();
        if (seg_mask) seg_mask->clear();
        if (overlay_rgb) overlay_rgb->clear();
        if (timing) *timing = DnnInferTiming{};
        if (status) *status = DnnInferStatus{};
        std::cerr << "[HorizonDnn] RGB infer unknown exception" << std::endl;
        return false;
    }
}

bool HorizonDnnRunner::inferNv12(const uint8_t* nv12, int width, int height,
                                 int src_stride_y, size_t bytes_used,
                                 bool src_is_nv12,
                                 std::vector<DnnDetection>* detections,
                                 std::vector<uint8_t>* seg_mask,
                                 std::vector<uint8_t>* overlay_rgb,
                                 DnnInferTiming* timing,
                                 DnnInferStatus* status) {
    try {
    if (detections) detections->clear();
    if (seg_mask) seg_mask->clear();
    if (overlay_rgb) overlay_rgb->clear();
    if (timing) *timing = DnnInferTiming{};
    if (status) *status = DnnInferStatus{};
    if (!nv12 || !ValidOrigSize(width, height) || (width & 1) != 0 ||
        (height & 1) != 0 || src_stride_y < width ||
        (src_stride_y & 1) != 0) {
        std::cerr << "[HorizonDnn] invalid YUV geometry " << width << "x"
                  << height << " stride=" << src_stride_y << std::endl;
        return false;
    }
    if (bytes_used == 0 || bytes_used <
        static_cast<size_t>(src_stride_y) * height * 3 / 2) {
        std::cerr << "[HorizonDnn] invalid yuv buffer " << width << "x"
                  << height << " stride=" << src_stride_y
                  << " bytes=" << bytes_used << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> infer_lock(infer_mtx_);
    if (!cfg_.enabled || (!detect_ && !seg_)) return false;

    std::vector<DnnDetection> dets;
    std::vector<uint8_t> local_seg;

    const bool detect_requested = detect_ready_;
    const bool want_seg = (seg_mask != nullptr) && seg_ready_;
    bool run_parallel =
        want_seg && cfg_.parallel_det_seg && detect_ready_ && seg_ready_;
    if (run_parallel && !parallel_executor_) {
        std::cerr << "[HorizonDnn] parallel executor unavailable; "
                     "fall back to sequential inference"
                  << std::endl;
        run_parallel = false;
    }

    bool det_ok = false;
    bool seg_ok = false;
    if (run_parallel) {
        DnnInferTiming det_timing{};
        DnnInferTiming seg_timing{};
        std::future<bool> det_future;
        std::future<bool> seg_future;
        try {
            det_future = parallel_executor_->detect_worker.submit([&]() {
                return detect_->inferYuv(
                    nv12, width, height, src_stride_y, bytes_used,
                    src_is_nv12, cfg_.detect, &dets, &det_timing);
            });
        } catch (const std::exception& error) {
            std::cerr << "[HorizonDnn] detect submission exception: "
                      << error.what() << std::endl;
        } catch (...) {
            std::cerr << "[HorizonDnn] detect submission unknown exception"
                      << std::endl;
        }
        try {
            seg_future = parallel_executor_->seg_worker.submit([&]() {
                return seg_->inferYuv(nv12, width, height, src_stride_y,
                                      bytes_used, src_is_nv12, cfg_.seg,
                                      local_seg, &seg_timing);
            });
        } catch (const std::exception& error) {
            std::cerr << "[HorizonDnn] seg submission exception: "
                      << error.what() << std::endl;
        } catch (...) {
            std::cerr << "[HorizonDnn] seg submission unknown exception"
                      << std::endl;
        }
        if (det_future.valid()) {
            try {
                det_ok = det_future.get();
            } catch (const std::exception& error) {
                std::cerr << "[HorizonDnn] detect worker exception: "
                          << error.what() << std::endl;
            } catch (...) {
                std::cerr << "[HorizonDnn] detect worker unknown exception"
                          << std::endl;
            }
        }
        if (seg_future.valid()) {
            try {
                seg_ok = seg_future.get();
            } catch (const std::exception& error) {
                std::cerr << "[HorizonDnn] seg worker exception: "
                          << error.what() << std::endl;
            } catch (...) {
                std::cerr << "[HorizonDnn] seg worker unknown exception"
                          << std::endl;
            }
        }
        if (timing) {
            timing->detect_pre_ms = det_timing.detect_pre_ms;
            timing->detect_bpu_ms = det_timing.detect_bpu_ms;
            timing->detect_ms = det_timing.detect_ms;
            timing->seg_pre_ms = seg_timing.seg_pre_ms;
            timing->seg_bpu_ms = seg_timing.seg_bpu_ms;
            timing->seg_post_ms = seg_timing.seg_post_ms;
            timing->seg_ms = seg_timing.seg_ms;
        }
    } else {
        if (detect_requested) {
            try {
                det_ok = detect_->inferYuv(
                    nv12, width, height, src_stride_y, bytes_used,
                    src_is_nv12, cfg_.detect, &dets, timing);
            } catch (const std::exception& error) {
                std::cerr << "[HorizonDnn] detect infer exception: "
                          << error.what() << std::endl;
            } catch (...) {
                std::cerr << "[HorizonDnn] detect infer unknown exception"
                          << std::endl;
            }
        }
        if (want_seg) {
            try {
                seg_ok = seg_->inferYuv(nv12, width, height, src_stride_y,
                                        bytes_used, src_is_nv12, cfg_.seg,
                                        local_seg, timing);
            } catch (const std::exception& error) {
                std::cerr << "[HorizonDnn] seg infer exception: "
                          << error.what() << std::endl;
            } catch (...) {
                std::cerr << "[HorizonDnn] seg infer unknown exception"
                          << std::endl;
            }
        }
    }

    if (detect_requested && !det_ok) {
        dets.clear();
        std::cerr << "[HorizonDnn] detect infer failed" << std::endl;
    }
    if (want_seg && !seg_ok) {
        local_seg.clear();
        std::cerr << "[HorizonDnn] seg infer failed" << std::endl;
    }
    if (status) {
        status->detect_requested = detect_requested;
        status->detect_succeeded = det_ok;
        status->segmentation_requested = want_seg;
        status->segmentation_succeeded = seg_ok;
    }
    if (detections && det_ok) *detections = dets;
    if (seg_mask && seg_ok) *seg_mask = local_seg;
    if (!det_ok && !seg_ok) return false;
    if (overlay_rgb && (cfg_.publish_overlay || cfg_.save_overlay)) {
        try {
            overlay_rgb->resize(static_cast<size_t>(width) * height * 3);
            // NV12/I420 -> RGB base followed by the available model results.
            YuvToRgbFull(nv12, width, height, src_stride_y, src_is_nv12,
                         overlay_rgb->data());
            const std::vector<uint8_t> empty_seg;
            const auto& overlay_seg =
                cfg_.overlay_blend_seg ? local_seg : empty_seg;
            if (!dets.empty() || !overlay_seg.empty()) {
                DrawOverlay(overlay_rgb->data(), width, height, dets,
                            overlay_seg, cfg_.overlay_seg_alpha,
                            cfg_.seg.class_names);
            }
        } catch (const std::exception& error) {
            overlay_rgb->clear();
            std::cerr << "[HorizonDnn] overlay generation exception: "
                      << error.what() << std::endl;
        } catch (...) {
            overlay_rgb->clear();
            std::cerr << "[HorizonDnn] overlay generation unknown exception"
                      << std::endl;
        }
    }
    return true;
    } catch (const std::exception& error) {
        if (detections) detections->clear();
        if (seg_mask) seg_mask->clear();
        if (overlay_rgb) overlay_rgb->clear();
        if (timing) *timing = DnnInferTiming{};
        if (status) *status = DnnInferStatus{};
        std::cerr << "[HorizonDnn] YUV infer exception: " << error.what()
                  << std::endl;
        return false;
    } catch (...) {
        if (detections) detections->clear();
        if (seg_mask) seg_mask->clear();
        if (overlay_rgb) overlay_rgb->clear();
        if (timing) *timing = DnnInferTiming{};
        if (status) *status = DnnInferStatus{};
        std::cerr << "[HorizonDnn] YUV infer unknown exception" << std::endl;
        return false;
    }
}

void ProbeYuv420Format(const uint8_t* data, int w, int h, int stride_y,
                       size_t bytes_used, YuvFormatProbe* probe) {
    if (!probe) return;
    probe->width = w;
    probe->height = h;
    probe->bytes_used = bytes_used;
    probe->likely_format = "unknown";
    if (!data || !ValidOrigSize(w, h) || (w & 1) != 0 || (h & 1) != 0 ||
        stride_y < w || (stride_y & 1) != 0) {
        return;
    }
    const size_t row_bytes = static_cast<size_t>(stride_y);
    const size_t row_count = static_cast<size_t>(h) +
                             static_cast<size_t>(h / 2);
    if (row_count == 0U ||
        row_bytes > std::numeric_limits<size_t>::max() / row_count ||
        bytes_used < row_bytes * row_count) {
        return;
    }
    const int stride = stride_y;
    probe->stride_y = stride_y;

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
