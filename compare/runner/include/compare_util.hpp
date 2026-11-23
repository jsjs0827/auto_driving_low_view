#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "r36_stereo/nv12.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace r36compare {
inline std::string quote(const std::string& value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << char(c);
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
    else out << char(c);
  }
  out << '"'; return out.str();
}
inline std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out; out << std::setprecision(15) << value; return out.str();
}
inline std::vector<std::string> tsv(const std::string& line) {
  std::vector<std::string> fields; std::size_t start = 0;
  for (;;) { const auto end = line.find('\t', start); fields.push_back(line.substr(start, end - start));
    if (end == std::string::npos) break;
    start = end + 1;
  }
  if (fields.size() != 5 && fields.size() != 6) throw std::invalid_argument("Manifest needs five legacy or six NV12 tab-separated fields per line (no header)");
  for (const auto& field : fields) if (field.empty() || field.find('\r') != std::string::npos || field.find('\0') != std::string::npos)
    throw std::invalid_argument("Manifest fields must be nonempty UTF-8/LF text without NUL/CR");
  const auto& stem = fields.front();
  if (stem == "." || stem == ".." || stem.find_first_of("/\\") != std::string::npos)
    throw std::invalid_argument("Manifest stem cannot contain a path separator");
  for (unsigned char c : stem) if (c < 32 || c == 127) throw std::invalid_argument("Manifest stem contains a control character");
  return fields;
}

struct StereoInputAdaptation {
  std::string mode;
  cv::Size source_size, model_input_size, asserted_model_input_size;
  cv::Rect source_roi;
  double scale_x = 1, scale_y = 1, disparity_x_scale = 1;
};
inline void validateStereoInputRequest(const std::string& mode, cv::Size asserted) {
  if (mode != "strict" && mode != "resize" && mode != "crop")
    throw std::invalid_argument("--stereo-input-mode must be crop, strict or resize");
  if ((asserted.width == 0) != (asserted.height == 0) || asserted.width < 0 || asserted.height < 0 ||
      asserted.width > 4096 || asserted.height > 4096 ||
      (asserted.width && (asserted.width < 2 || asserted.height < 2 || ((asserted.width | asserted.height) & 1))))
    throw std::invalid_argument("--stereo-input-width and --stereo-input-height must both be even model metadata assertions in [2,4096] or both omitted");
}
inline StereoInputAdaptation stereoInputAdaptation(const std::string& mode, cv::Size source,
                                                   cv::Size model, cv::Size asserted = {}) {
  validateStereoInputRequest(mode, asserted);
  if (source.width < 1 || source.height < 1 || model.width < 1 || model.height < 1)
    throw std::invalid_argument("Source and actual model dimensions must be positive");
  if (asserted.width && asserted != model)
    throw std::invalid_argument("Asserted stereo model input " + std::to_string(asserted.width) + "x" + std::to_string(asserted.height) +
      " differs from actual Horizon metadata " + std::to_string(model.width) + "x" + std::to_string(model.height));
  if (mode == "strict" && source != model)
    throw std::invalid_argument("Stereo model input " + std::to_string(model.width) + "x" + std::to_string(model.height) +
      " differs from source " + std::to_string(source.width) + "x" + std::to_string(source.height) +
      "; strict mode forbids geometry changes; choose crop or resize explicitly");
  StereoInputAdaptation result;
  result.mode=mode;result.source_size=source;result.model_input_size=model;
  result.asserted_model_input_size=asserted;result.source_roi={0,0,source.width,source.height};
  if(mode=="crop") {
    if(source.width!=model.width || source.height<model.height ||
       source.width<2 || model.height<2 || source.width>8192 || source.height>8192 ||
       ((source.width|source.height|model.width|model.height)&1))
      throw std::invalid_argument("crop requires equal source/model width, source height >= model height, and even NV12 dimensions");
    result.source_roi={0,source.height-model.height,model.width,model.height};
    return result;
  }
  result.scale_x=double(model.width)/source.width;result.scale_y=double(model.height)/source.height;
  result.disparity_x_scale=double(source.width)/model.width;
  return result;
}
inline void validateStereoCropAdaptation(const StereoInputAdaptation& adaptation) {
  const auto expected=stereoInputAdaptation("crop",adaptation.source_size,adaptation.model_input_size,
                                          adaptation.asserted_model_input_size);
  if(adaptation.source_roi!=expected.source_roi || adaptation.scale_x!=1 || adaptation.scale_y!=1 ||
     adaptation.disparity_x_scale!=1)
    throw std::invalid_argument("Crop adaptation must be the native bottom full-width ROI without scaling");
}
struct StereoBgrPair { cv::Mat left, right; };
inline StereoBgrPair prepareStereoBgr(const cv::Mat& left, const cv::Mat& right,
                                     const StereoInputAdaptation& adaptation) {
  // Validate both source views before doing any resize or submitting inference.
  if (left.empty() || right.empty() || left.type() != CV_8UC3 || right.type() != CV_8UC3 ||
      left.size() != adaptation.source_size || right.size() != adaptation.source_size)
    throw std::invalid_argument("Stereo source images must both be BGR8 at the unchanged recording dimensions");
  if (adaptation.mode == "crop") {
    validateStereoCropAdaptation(adaptation);
    return {left(adaptation.source_roi).clone(),right(adaptation.source_roi).clone()};
  }
  if (adaptation.source_size == adaptation.model_input_size) return {left, right};
  if (adaptation.mode != "resize") throw std::invalid_argument("Only explicit resize mode may change stereo image dimensions");
  StereoBgrPair out;
  cv::resize(left, out.left, adaptation.model_input_size, 0, 0, cv::INTER_LINEAR);
  cv::resize(right, out.right, adaptation.model_input_size, 0, 0, cv::INTER_LINEAR);
  return out;
}
struct StereoNv12Pair { cv::Mat left, right; };
inline StereoNv12Pair prepareStereoNv12(const cv::Mat& left,const cv::Mat& right,
                                       const StereoInputAdaptation& adaptation) {
  if(r36stereo::nv12_size(left)!=adaptation.source_size || r36stereo::nv12_size(right)!=adaptation.source_size)
    throw std::invalid_argument("Stereo NV12 pair must match the unchanged source dimensions");
  if(adaptation.mode=="crop") {
    validateStereoCropAdaptation(adaptation);
    return {r36stereo::crop_nv12(left,adaptation.source_roi),r36stereo::crop_nv12(right,adaptation.source_roi)};
  }
  if(adaptation.mode!="strict" && adaptation.mode!="resize")
    throw std::invalid_argument("Unknown stereo NV12 adaptation mode");
  if(adaptation.mode=="strict" && adaptation.source_size!=adaptation.model_input_size)
    throw std::invalid_argument("Strict NV12 preparation cannot change dimensions");
  return {r36stereo::resize_nv12(left,adaptation.model_input_size),r36stereo::resize_nv12(right,adaptation.model_input_size)};
}
inline std::string sizeJson(cv::Size size) {
  return "{\"width\":" + std::to_string(size.width) + ",\"height\":" + std::to_string(size.height) + "}";
}
inline std::string stereoInputAdaptationJson(const StereoInputAdaptation& a, cv::Size output = {}) {
  return "{\"mode\":" + quote(a.mode) + ",\"source_size\":" + sizeJson(a.source_size) +
    ",\"model_input_size\":" + sizeJson(a.model_input_size) +
    ",\"output_size\":" + (output.width > 0 && output.height > 0 ? sizeJson(output) : "null") +
    ",\"restored_q8_size\":" + sizeJson(a.source_size) +
    ",\"source_roi\":{\"x\":"+std::to_string(a.source_roi.x)+",\"y\":"+std::to_string(a.source_roi.y)+
      ",\"width\":"+std::to_string(a.source_roi.width)+",\"height\":"+std::to_string(a.source_roi.height)+"}"+
    ",\"scale_x\":" + number(a.scale_x) + ",\"scale_y\":" + number(a.scale_y) +
    ",\"disparity_x_scale\":" + number(a.disparity_x_scale) +
    ",\"scale_direction\":\"scale_x/y=source_to_model;disparity_x_scale=model_to_source_horizontal\""
    ",\"resize_interpolation\":" + (a.mode=="resize"?quote("INTER_LINEAR"):std::string("null")) +
    ",\"resize_applied\":" + (a.mode=="resize" && a.source_size != a.model_input_size ? "true" : "false") +
    ",\"crop_applied\":" + (a.mode=="crop"?"true":"false") +
    ",\"asserted_model_input_size\":" + (a.asserted_model_input_size.width ? sizeJson(a.asserted_model_input_size) : "null") +
    ",\"q8_restore_policy\":" + quote(a.mode=="crop"?"zero_full_source_then_write_native_ROI_no_resize_or_disparity_scale":"validity_aware_resize_and_horizontal_disparity_scale") +
    ",\"compute_scope\":\"NV12_Y_UV_explicit_transform_model_pack_infer_decode_disparity_restore_and_Q8_quantization_inside_SDK_disparity_task\""
    ",\"source_nv12_and_intrinsics_unchanged\":true}";
}

// Preserve floor(double(d)*256+0.5), with invalid/underflow/overflow mapped to zero.
// This row kernel deliberately does not use float(d*256+0.5): that addition can
// round the float immediately below a half-integer up to the integer boundary.
inline void quantizeQ8Row(const float* source, std::uint16_t* target, int width) {
  int x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
  const float32x4_t minimum = vdupq_n_f32(0.5f / 256.0f);
  const float32x4_t maximum = vdupq_n_f32(65535.5f / 256.0f);
  const float32x4_t half = vdupq_n_f32(0.5f);
  const float32x4_t zero = vdupq_n_f32(0.0f);
  const uint32x4_t one = vdupq_n_u32(1);
  for (; width - x >= 4; x += 4) {
    const float32x4_t d = vld1q_f32(source + x);
    const uint32x4_t valid = vandq_u32(vcgeq_f32(d, minimum), vcltq_f32(d, maximum));
    // Only finite, positive values that can round into [1,65535] reach vcvt.
    const float32x4_t scaled = vmulq_n_f32(vbslq_f32(valid, d, zero), 256.0f);
    const uint32x4_t base = vcvtq_u32_f32(scaled); // truncation, independent of FPCR rounding mode
    // Multiplication by 256 is exact here, as is base+0.5 for base<=65535.
    // Testing the fractional part avoids any rounded float +0.5 operation.
    const float32x4_t threshold = vaddq_f32(vcvtq_f32_u32(base), half);
    const uint32x4_t increment = vandq_u32(vcgeq_f32(scaled, threshold), one);
    const uint32x4_t rounded = vandq_u32(vaddq_u32(base, increment), valid);
    vst1_u16(target + x, vmovn_u32(rounded));
  }
#endif
  for (; x < width; ++x) {
    const double d = source[x];
    std::uint16_t value = 0;
    if (std::isfinite(d) && d > 0) {
      const double q = std::floor(d * 256.0 + 0.5);
      if (q >= 1 && q <= 65535) value = static_cast<std::uint16_t>(q);
    }
    target[x] = value;
  }
}

// Same contract as v8p1 stereo_core.normalize_disparity/disparity_to_q8:
// normalized linear interpolation, nearest validity mask, horizontal scaling,
// half-up Q8 rounding; invalid, underflow and overflow remain zero.
inline std::vector<std::uint16_t> normalizeQ8(const cv::Mat& disparity, cv::Size target) {
  if (disparity.empty() || disparity.type() != CV_32FC1 || target.width < 1 || target.height < 1)
    throw std::invalid_argument("Stereo output must be a nonempty CV_32FC1 disparity matrix");
  if (disparity.size() == target) {
    // No interpolation or horizontal scaling occurs at equal dimensions.
    // Avoid allocating double numerator/weight maps and three same-size resizes.
    std::vector<std::uint16_t> out(std::size_t(target.width) * target.height);
    for (int y = 0; y < disparity.rows; ++y)
      quantizeQ8Row(disparity.ptr<float>(y), out.data() + std::size_t(y) * target.width, target.width);
    return out;
  }
  cv::Mat numerator(disparity.size(), CV_64FC1, cv::Scalar(0));
  cv::Mat weight(disparity.size(), CV_64FC1, cv::Scalar(0));
  cv::Mat valid(disparity.size(), CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < disparity.rows; ++y) for (int x = 0; x < disparity.cols; ++x) {
    const float d = disparity.at<float>(y, x);
    if (std::isfinite(d) && d > 0) { numerator.at<double>(y, x) = d; weight.at<double>(y, x) = 1; valid.at<std::uint8_t>(y, x) = 1; }
  }
  cv::Mat values, weights, nearest;
  cv::resize(numerator, values, target, 0, 0, cv::INTER_LINEAR);
  cv::resize(weight, weights, target, 0, 0, cv::INTER_LINEAR);
  cv::resize(valid, nearest, target, 0, 0, cv::INTER_NEAREST_EXACT);
  std::vector<std::uint16_t> out(std::size_t(target.width) * target.height, 0);
  const double scale = double(target.width) / disparity.cols;
  for (int y = 0; y < target.height; ++y) for (int x = 0; x < target.width; ++x) {
    const double w = weights.at<double>(y, x);
    if (!nearest.at<std::uint8_t>(y, x) || w <= 1e-12) continue;
    // v8p1 rounds its normalized matrix to float32 before Q8 conversion.
    const float d = float(values.at<double>(y, x) / w * scale);
    const double q = std::floor(double(d) * 256.0 + 0.5);
    if (std::isfinite(q) && q >= 1 && q <= 65535) out[std::size_t(y) * target.width + x] = std::uint16_t(q);
  }
  return out;
}
inline std::vector<std::uint16_t> restoreStereoQ8(const cv::Mat& disparity,
                                               const StereoInputAdaptation& adaptation) {
  if(adaptation.mode!="crop") {
    validateStereoInputRequest(adaptation.mode,adaptation.asserted_model_input_size);
    return normalizeQ8(disparity,adaptation.source_size);
  }
  // Revalidate the public value before using its ROI for vector indexing.
  validateStereoCropAdaptation(adaptation);
  if(disparity.empty() || disparity.type()!=CV_32FC1 ||
     disparity.size()!=adaptation.source_roi.size())
    throw std::invalid_argument("Crop disparity must match the native bottom ROI exactly; resizing/scaling is forbidden");
  std::vector<std::uint16_t> out(std::size_t(adaptation.source_size.width)*adaptation.source_size.height,0);
  for (int y = 0; y < disparity.rows; ++y)
    quantizeQ8Row(disparity.ptr<float>(y),
                  out.data() + std::size_t(y + adaptation.source_roi.y) * adaptation.source_size.width + adaptation.source_roi.x,
                  disparity.cols);
  return out;
}
inline std::vector<std::uint16_t> decodeQ8(const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() % 2) throw std::invalid_argument("Q8 little-endian input must have even byte count");
  std::vector<std::uint16_t> values(bytes.size() / 2);
  for (std::size_t i = 0; i < values.size(); ++i) values[i] = std::uint16_t(bytes[i * 2]) | (std::uint16_t(bytes[i * 2 + 1]) << 8);
  return values;
}
}  // namespace r36compare
