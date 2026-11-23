#include "r36_stereo/tensor_codec.hpp"
#include "r36_stereo/nv12.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <opencv2/imgproc.hpp>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace r36stereo::codec {
namespace {
constexpr std::size_t kMaxBytes = 512u * 1024u * 1024u;
template<class T, class = void> struct HasStride : std::false_type {};
template<class T> struct HasStride<T, std::void_t<decltype(std::declval<T>().stride)>> : std::true_type {};
template<class T, class = void> struct HasAxis : std::false_type {};
template<class T> struct HasAxis<T, std::void_t<decltype(std::declval<T>().quantizeAxis)>> : std::true_type {};

[[noreturn]] void reject(const std::string& why) {
  throw std::runtime_error("Horizon tensor contract: " + why);
}
std::size_t mul(std::size_t a, std::size_t b) {
  if (b && a > kMaxBytes / b) reject("storage exceeds 512 MiB safety limit");
  return a * b;
}
template<class T> std::array<std::int64_t, 4> strides(const T& p) {
  std::array<std::int64_t, 4> result{};
  if constexpr (HasStride<T>::value) {
    for (int i = 0; i < 4; ++i) result[i] = p.stride[i];
  }
  return result;
}
template<class T> int axis(const T& p) {
  if constexpr (HasAxis<T>::value) return p.quantizeAxis;
  return -1;
}
bool nv12(int type) {
  return type == HB_DNN_IMG_TYPE_NV12 || type == HB_DNN_IMG_TYPE_NV12_SEPARATE;
}
std::size_t element_bytes(int type) {
  switch (type) {
    case HB_DNN_IMG_TYPE_RGB: case HB_DNN_IMG_TYPE_BGR:
    case HB_DNN_TENSOR_TYPE_S8: case HB_DNN_TENSOR_TYPE_U8: return 1;
    case HB_DNN_TENSOR_TYPE_S16: case HB_DNN_TENSOR_TYPE_U16: return 2;
    case HB_DNN_TENSOR_TYPE_F32: case HB_DNN_TENSOR_TYPE_S32: return 4;
    default: reject("unsupported tensorType=" + std::to_string(type) +
                    "; no reinterpret-cast fallback is permitted");
  }
}
Shape shape(const hbDNNTensorProperties& p, std::size_t item_bytes, bool is_nv12) {
  if (p.validShape.numDimensions != 4 || p.alignedShape.numDimensions != 4)
    reject("validShape and alignedShape must both have rank 4");
  Shape s;
  if (p.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    s.channel_axis = 1; s.height_axis = 2; s.width_axis = 3;
  } else if (p.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    s.height_axis = 1; s.width_axis = 2; s.channel_axis = 3;
  } else reject("only explicit NCHW or NHWC layout is supported");
  for (int i = 0; i < 4; ++i) {
    const int v = p.validShape.dimensionSize[i], a = p.alignedShape.dimensionSize[i];
    if (v <= 0 || v > 8192 || a < v || a > 8192)
      reject("invalid, smaller-than-valid or excessive aligned dimension");
  }
  if (p.validShape.dimensionSize[0] != 1 || p.alignedShape.dimensionSize[0] != 1)
    reject("only a single batch (valid and aligned N=1) is supported");
  s.width = p.validShape.dimensionSize[s.width_axis];
  s.height = p.validShape.dimensionSize[s.height_axis];
  s.channels = p.validShape.dimensionSize[s.channel_axis];
  const auto supplied = strides(p);
  const bool all_zero = std::all_of(supplied.begin(), supplied.end(), [](auto n) { return n == 0; });
  if (is_nv12) {
    if (!all_zero) reject("NV12 requires zero/absent stride metadata in this implementation");
    if ((s.width | s.height) & 1) reject("NV12 valid width and height must be even");
    if (s.channels != 1 && s.channels != 3) reject("NV12 channel metadata must be 1 or 3");
    if (p.alignedShape.dimensionSize[s.channel_axis] != s.channels)
      reject("NV12 aligned channel padding is unsupported");
    const auto aw = static_cast<std::size_t>(p.alignedShape.dimensionSize[s.width_axis]);
    const auto ah = static_cast<std::size_t>(p.alignedShape.dimensionSize[s.height_axis]);
    if ((aw % 16) || (ah % 2)) reject("NV12 aligned width must be a multiple of 16 and height even");
    s.byte_stride[s.width_axis] = 1;
    s.byte_stride[s.height_axis] = aw;
    const auto y_bytes = mul(aw, ah);
    s.storage_bytes = y_bytes + y_bytes / 2;
  } else {
    s.byte_stride[3] = item_bytes;
    for (int i = 2; i >= 0; --i)
      s.byte_stride[i] = mul(s.byte_stride[i + 1], p.alignedShape.dimensionSize[i + 1]);
    s.storage_bytes = mul(s.byte_stride[0], p.alignedShape.dimensionSize[0]);
    if (!all_zero) {
      for (int i = 0; i < 4; ++i) {
        if (supplied[i] <= 0 || static_cast<std::uint64_t>(supplied[i]) != s.byte_stride[i])
          reject("nonstandard stride metadata: only dense aligned byte strides are supported");
      }
    }
  }
  if (p.alignedByteSize <= 0 || static_cast<std::size_t>(p.alignedByteSize) > kMaxBytes ||
      static_cast<std::size_t>(p.alignedByteSize) < s.storage_bytes)
    reject("alignedByteSize is invalid or smaller than required aligned storage");
  return s;
}
template<class T> double load(const std::uint8_t* p) {
  T value; std::memcpy(&value, p, sizeof(value)); return static_cast<double>(value);
}
std::size_t offset(const Shape& s, int y, int x, int c = 0) {
  return static_cast<std::size_t>(y) * s.byte_stride[s.height_axis] +
         static_cast<std::size_t>(x) * s.byte_stride[s.width_axis] +
         static_cast<std::size_t>(c) * s.byte_stride[s.channel_axis];
}

// The generic F32 decoder promotes to double and then narrows back to float.
// Ordinary finite values, signed zero and infinities are unchanged by that
// round trip. NaNs and subnormals can depend on the floating-point environment
// (sNaN quieting/default-NaN/flush-to-zero), so retain the actual old conversion
// for those exceptional values rather than promising that blind memcpy is equal.
void canonicalize_exceptional_f32(std::uint8_t* dst, const std::uint8_t* src) {
  std::uint32_t bits;
  std::memcpy(&bits, src, sizeof(bits));
  const auto exponent = bits & 0x7f800000u;
  if ((exponent == 0 || exponent == 0x7f800000u) && (bits & 0x007fffffu)) {
    // Volatile keeps both conversions observable even when the compiler would
    // otherwise remove float -> double -> float under its default NaN model.
    volatile double promoted = load<float>(src);
    const float value = static_cast<float>(promoted);
    std::memcpy(dst, &value, sizeof(value));
  }
}
void copy_f32_legacy_compatible(std::uint8_t* dst, const std::uint8_t* src,
                                std::size_t values) {
  std::size_t x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
  const auto exponent_mask = vdupq_n_u32(0x7f800000u);
  const auto mantissa_mask = vdupq_n_u32(0x007fffffu);
  const auto zero = vdupq_n_u32(0);
  for (; x + 4 <= values; x += 4) {
    // Byte loads/stores are legal for nonaligned input buffers and avoid
    // violating strict aliasing by interpreting float storage as uint32_t*.
    const auto packed = vld1q_u8(src + x * sizeof(float));
    vst1q_u8(dst + x * sizeof(float), packed);
    const auto bits = vreinterpretq_u32_u8(packed);
    const auto exponent = vandq_u32(bits, exponent_mask);
    const auto extreme = vorrq_u32(vceqq_u32(exponent, zero),
                                  vceqq_u32(exponent, exponent_mask));
    const auto nonzero_mantissa = vmvnq_u32(vceqq_u32(vandq_u32(bits, mantissa_mask), zero));
    if (vmaxvq_u32(vandq_u32(extreme, nonzero_mantissa))) {
      for (std::size_t lane = 0; lane < 4; ++lane)
        canonicalize_exceptional_f32(dst + (x + lane) * sizeof(float),
                                     src + (x + lane) * sizeof(float));
    }
  }
#endif
  for (; x < values; ++x) {
    std::memcpy(dst + x * sizeof(float), src + x * sizeof(float), sizeof(float));
    canonicalize_exceptional_f32(dst + x * sizeof(float), src + x * sizeof(float));
  }
}
}  // namespace

bool header_has_stride() { return HasStride<hbDNNTensorProperties>::value; }
bool header_has_quantize_axis() { return HasAxis<hbDNNTensorProperties>::value; }

InputSpec inspect_input(const hbDNNTensorProperties& p) {
  const bool yuv = nv12(p.tensorType);
  if (!yuv && p.tensorType != HB_DNN_IMG_TYPE_RGB && p.tensorType != HB_DNN_IMG_TYPE_BGR &&
      p.tensorType != HB_DNN_TENSOR_TYPE_F32)
    reject("input supports NV12, NV12_SEPARATE, RGB/BGR uint8 or RGB F32 0..255 only");
  if (p.quantiType != NONE) reject("quantized input preprocessing is unsupported; provide an explicit validated model contract");
  InputSpec s; s.properties = p;
  s.shape = shape(p, yuv ? 1 : element_bytes(p.tensorType), yuv);
  if (!yuv && s.shape.channels != 3) reject("RGB/BGR/F32 input requires three channels");
  if (p.tensorType == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
    const auto y_bytes = mul(p.alignedShape.dimensionSize[s.shape.width_axis],
                             p.alignedShape.dimensionSize[s.shape.height_axis]);
    s.plane_bytes = {y_bytes, y_bytes / 2};
  } else s.plane_bytes = {static_cast<std::size_t>(p.alignedByteSize)};
  return s;
}

OutputSpec inspect_output(const hbDNNTensorProperties& p, int expected_width) {
  OutputSpec s; s.properties = p; s.element_bytes = element_bytes(p.tensorType);
  if (p.tensorType == HB_DNN_IMG_TYPE_RGB || p.tensorType == HB_DNN_IMG_TYPE_BGR)
    reject("disparity output must be a numeric tensor, not an image type");
  s.shape = shape(p, s.element_bytes, false);
  if (s.shape.channels != 1) reject("disparity output requires one valid channel");
  if (expected_width <= 0 || s.shape.width != expected_width)
    reject("output width must equal input width; horizontal disparity units cannot be guessed");
  if (p.tensorType == HB_DNN_TENSOR_TYPE_F32 && p.quantiType != NONE)
    reject("F32 output with quantization metadata is inconsistent/unsupported");
  switch (p.quantiType) {
    case NONE: break;
    case SHIFT:
      if (p.shift.shiftLen != 1 || !p.shift.shiftData)
        reject("SHIFT output supports exactly one per-tensor shift; per-axis metadata is rejected");
      s.shift = p.shift.shiftData[0];
      break;
    case SCALE:
      if (p.scale.scaleLen != 1 || !p.scale.scaleData)
        reject("SCALE output supports exactly one per-tensor scale; per-axis metadata is rejected");
      s.scale = p.scale.scaleData[0];
      if (!std::isfinite(s.scale) || s.scale <= 0) reject("SCALE must be finite and positive");
      if (p.scale.zeroPointLen == 1 && p.scale.zeroPointData)
        s.zero_point = p.scale.zeroPointData[0];
      else if (p.scale.zeroPointLen != 0)
        reject("SCALE zero point must be absent or exactly one valid value");
      break;
    default: reject("unknown quantiType");
  }
  return s;
}

PackedInput pack_nv12(const cv::Mat& image, const InputSpec& spec) {
  if (!nv12(spec.properties.tensorType))
    reject("direct NV12 input requires an NV12 or NV12_SEPARATE model tensorType");
  const auto size = nv12_size(image);
  if (size.width != spec.shape.width || size.height != spec.shape.height)
    reject("NV12 image must exactly match model width/height; resize explicitly upstream");
  PackedInput packed;
  for (auto bytes : spec.plane_bytes) packed.planes.emplace_back(bytes, 0);
  const auto& p = spec.properties;
  const auto& s = spec.shape;
  const auto pitch = s.byte_stride[s.height_axis];
  const auto aligned_y = pitch * p.alignedShape.dimensionSize[s.height_axis];
  auto* y_dst = packed.planes[0].data();
  auto* uv_dst = p.tensorType == HB_DNN_IMG_TYPE_NV12_SEPARATE ? packed.planes[1].data() : y_dst + aligned_y;
  std::fill(y_dst, y_dst + aligned_y, std::uint8_t(16));
  std::fill(uv_dst, uv_dst + aligned_y / 2, std::uint8_t(128));
  for (int y = 0; y < s.height; ++y)
    std::memcpy(y_dst + static_cast<std::size_t>(y) * pitch, image.ptr(y), s.width);
  for (int y = 0; y < s.height / 2; ++y)
    std::memcpy(uv_dst + static_cast<std::size_t>(y) * pitch, image.ptr(s.height + y), s.width);
  return packed;
}

PackedInput pack_bgr(const cv::Mat& image, const InputSpec& spec) {
  if (image.empty() || image.type() != CV_8UC3 || image.cols != spec.shape.width || image.rows != spec.shape.height)
    reject("input image must be CV_8UC3 BGR and exactly match the model width/height; resize explicitly upstream");
  if (nv12(spec.properties.tensorType)) return pack_nv12(bgr_to_nv12(image), spec);
  PackedInput packed;
  for (auto bytes : spec.plane_bytes) packed.planes.emplace_back(bytes, 0);
  const auto& p = spec.properties;
  const auto& s = spec.shape;
  {
    auto* dst = packed.planes[0].data();
    for (int y = 0; y < s.height; ++y) {
      const auto* row = image.ptr<cv::Vec3b>(y);
      for (int x = 0; x < s.width; ++x) {
        for (int c = 0; c < 3; ++c) {
          const int src_c = p.tensorType == HB_DNN_IMG_TYPE_BGR ? c : 2 - c;
          const auto pos = offset(s, y, x, c);
          if (p.tensorType == HB_DNN_TENSOR_TYPE_F32) {
            const float value = row[x][src_c]; // RGB float32, 0..255, no /255.
            std::memcpy(dst + pos, &value, sizeof(value));
          } else dst[pos] = row[x][src_c];
        }
      }
    }
  }
  return packed;
}

cv::Mat decode_disparity(const OutputSpec& spec, const void* data, std::size_t available_bytes) {
  if (!data || available_bytes < static_cast<std::size_t>(spec.properties.alignedByteSize))
    reject("output memory is null or smaller than alignedByteSize");
  const auto* base = static_cast<const std::uint8_t*>(data);
  cv::Mat result(spec.shape.height, spec.shape.width, CV_32FC1);
  const auto& shape = spec.shape;
  if (spec.properties.tensorType == HB_DNN_TENSOR_TYPE_F32 &&
      spec.properties.quantiType == NONE && spec.element_bytes == sizeof(float) &&
      shape.width > 0 && shape.height > 0 && shape.channels == 1 &&
      ((spec.properties.tensorLayout == HB_DNN_LAYOUT_NCHW &&
        shape.width_axis == 3 && shape.height_axis == 2 && shape.channel_axis == 1) ||
       (spec.properties.tensorLayout == HB_DNN_LAYOUT_NHWC &&
        shape.width_axis == 2 && shape.height_axis == 1 && shape.channel_axis == 3)) &&
      shape.byte_stride[shape.width_axis] == sizeof(float)) {
    const auto row_bytes = static_cast<std::size_t>(shape.width) * sizeof(float);
    const auto pitch = shape.byte_stride[shape.height_axis];
    if (pitch >= row_bytes) {
      // Validate the last row once with division, avoiding offset overflow.
      // Every copied byte then belongs to a valid logical pixel, not padding.
      if (row_bytes > available_bytes ||
          static_cast<std::size_t>(shape.height - 1) > (available_bytes - row_bytes) / pitch)
        reject("output pixel address exceeds allocation");
      if (pitch == row_bytes && result.isContinuous()) {
        copy_f32_legacy_compatible(result.data, base,
                                  static_cast<std::size_t>(shape.width) * shape.height);
      } else {
        for (int y = 0; y < shape.height; ++y)
          copy_f32_legacy_compatible(result.ptr<std::uint8_t>(y),
                                    base + static_cast<std::size_t>(y) * pitch,
                                    static_cast<std::size_t>(shape.width));
      }
      return result;
    }
  }
  for (int y = 0; y < result.rows; ++y) {
    auto* row = result.ptr<float>(y);
    for (int x = 0; x < result.cols; ++x) {
      const auto pos = offset(spec.shape, y, x);
      if (pos > available_bytes || spec.element_bytes > available_bytes - pos)
        reject("output pixel address exceeds allocation");
      double raw = 0;
      switch (spec.properties.tensorType) {
        case HB_DNN_TENSOR_TYPE_F32: raw = load<float>(base + pos); break;
        case HB_DNN_TENSOR_TYPE_S8: raw = load<std::int8_t>(base + pos); break;
        case HB_DNN_TENSOR_TYPE_U8: raw = load<std::uint8_t>(base + pos); break;
        case HB_DNN_TENSOR_TYPE_S16: raw = load<std::int16_t>(base + pos); break;
        case HB_DNN_TENSOR_TYPE_U16: raw = load<std::uint16_t>(base + pos); break;
        case HB_DNN_TENSOR_TYPE_S32: raw = load<std::int32_t>(base + pos); break;
        default: reject("unsupported output type");
      }
      if (spec.properties.quantiType == SHIFT) raw = std::ldexp(raw, -spec.shift);
      else if (spec.properties.quantiType == SCALE) raw = (raw - spec.zero_point) * spec.scale;
      if (std::isfinite(raw) && std::abs(raw) > std::numeric_limits<float>::max())
        reject("dequantized disparity exceeds float32 range");
      row[x] = static_cast<float>(raw);
    }
  }
  return result;
}

std::string json_string(const std::string& text) {
  std::ostringstream out; out << '"';
  for (std::size_t i = 0; i < text.size(); ++i) {
    const auto c = static_cast<unsigned char>(text[i]);
    if (c >= 128) {
      // Keep valid UTF-8 intact. Escape an invalid OS filename byte rather
      // than emitting invalid JSON or corrupting valid Unicode paths.
      const int count = c >= 0xc2 && c <= 0xdf ? 2 : c >= 0xe0 && c <= 0xef ? 3 : c >= 0xf0 && c <= 0xf4 ? 4 : 0;
      bool valid = count && i + count <= text.size();
      for (int j = 1; valid && j < count; ++j) {
        const auto next = static_cast<unsigned char>(text[i + j]);
        valid = next >= 0x80 && next <= 0xbf;
      }
      if (valid) {
        const auto next = static_cast<unsigned char>(text[i + 1]);
        if ((c == 0xe0 && next < 0xa0) || (c == 0xed && next >= 0xa0) ||
            (c == 0xf0 && next < 0x90) || (c == 0xf4 && next >= 0x90)) valid = false;
      }
      if (valid) { out.write(text.data() + i, count); i += count - 1; continue; }
    }
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 32 || c >= 127) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << unsigned(c) << std::dec;
        else out << char(c);
    }
  }
  out << '"'; return out.str();
}
std::string tensor_contract_json(const hbDNNTensorProperties& p) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<float>::max_digits10);
  out << "{\"tensor_type\":" << p.tensorType << ",\"layout\":"
      << json_string(p.tensorLayout == HB_DNN_LAYOUT_NCHW ? "NCHW" : p.tensorLayout == HB_DNN_LAYOUT_NHWC ? "NHWC" : "NONE")
      << ",\"valid_shape\":[";
  for (int i = 0; i < std::clamp(p.validShape.numDimensions, 0, HB_DNN_TENSOR_MAX_DIMENSIONS); ++i) {
    if (i) out << ',';
    out << p.validShape.dimensionSize[i];
  }
  out << "],\"aligned_shape\":[";
  for (int i = 0; i < std::clamp(p.alignedShape.numDimensions, 0, HB_DNN_TENSOR_MAX_DIMENSIONS); ++i) {
    if (i) out << ',';
    out << p.alignedShape.dimensionSize[i];
  }
  out << "],\"aligned_bytes\":" << p.alignedByteSize << ",\"quanti_type\":" << int(p.quantiType)
      << ",\"quantize_axis\":";
  if (header_has_quantize_axis()) out << axis(p); else out << "null";
  out << ",\"stride\":";
  if (header_has_stride()) {
    const auto st = strides(p); out << '[';
    for (int i = 0; i < 4; ++i) { if (i) out << ','; out << st[i]; }
    out << ']';
  } else out << "null";
  out << ",\"scale_len\":" << p.scale.scaleLen << ",\"zero_point_len\":" << p.scale.zeroPointLen
      << ",\"shift_len\":" << p.shift.shiftLen << ",\"scalar_scale\":";
  if (p.quantiType == SCALE && p.scale.scaleLen == 1 && p.scale.scaleData && std::isfinite(p.scale.scaleData[0]))
    out << p.scale.scaleData[0];
  else out << "null";
  out << ",\"scalar_zero_point\":";
  if (p.quantiType == SCALE && p.scale.zeroPointLen == 1 && p.scale.zeroPointData) out << int(p.scale.zeroPointData[0]);
  else out << "null";
  out << ",\"scalar_shift\":";
  if (p.quantiType == SHIFT && p.shift.shiftLen == 1 && p.shift.shiftData) out << unsigned(p.shift.shiftData[0]);
  else out << "null";
  out << '}';
  return out.str();
}
}  // namespace r36stereo::codec
