#include "r36_stereo/tensor_codec.hpp"
#include "r36_stereo/nv12.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
int assertions = 0;
void require(bool okay, const char* text) {
  ++assertions;
  if (!okay) throw std::runtime_error(text);
}
template<class F> void rejected(F function, const char* message) {
  ++assertions;
  try { function(); } catch (const std::runtime_error& e) {
    if (std::string(e.what()).find(message) != std::string::npos) return;
    throw std::runtime_error(std::string("wrong rejection: ") + e.what());
  }
  throw std::runtime_error(std::string("missing rejection: ") + message);
}
template<class T, class = void> struct HasStride : std::false_type {};
template<class T> struct HasStride<T, std::void_t<decltype(std::declval<T>().stride)>> : std::true_type {};
template<class T> void set_stride(T& p, int index, int value) {
  if constexpr (HasStride<T>::value) p.stride[index] = value;
}
hbDNNTensorProperties props(int type, int layout, int w, int h, int c, int aw = 0, int ah = 0, int ac = 0) {
  hbDNNTensorProperties p{};
  p.tensorType = type; p.tensorLayout = layout; p.quantiType = NONE;
  p.validShape.numDimensions = p.alignedShape.numDimensions = 4;
  const int ca = layout == HB_DNN_LAYOUT_NCHW ? 1 : 3;
  const int ha = layout == HB_DNN_LAYOUT_NCHW ? 2 : 1;
  const int wa = layout == HB_DNN_LAYOUT_NCHW ? 3 : 2;
  p.validShape.dimensionSize[0] = 1;
  p.validShape.dimensionSize[ca] = c;
  p.validShape.dimensionSize[ha] = h;
  p.validShape.dimensionSize[wa] = w;
  p.alignedShape = p.validShape;
  p.alignedShape.dimensionSize[ca] = ac ? ac : c;
  p.alignedShape.dimensionSize[ha] = ah ? ah : h;
  p.alignedShape.dimensionSize[wa] = aw ? aw : w;
  const auto pixels = p.alignedShape.dimensionSize[wa] * p.alignedShape.dimensionSize[ha];
  if (type == HB_DNN_IMG_TYPE_NV12 || type == HB_DNN_IMG_TYPE_NV12_SEPARATE)
    p.alignedByteSize = pixels * 3 / 2;
  else {
    int item = 1;
    if (type == HB_DNN_TENSOR_TYPE_F32 || type == HB_DNN_TENSOR_TYPE_S32) item = 4;
    if (type == HB_DNN_TENSOR_TYPE_S16 || type == HB_DNN_TENSOR_TYPE_U16) item = 2;
    p.alignedByteSize = pixels * p.alignedShape.dimensionSize[ca] * item;
  }
  return p;
}
float read_float(const std::vector<std::uint8_t>& bytes, std::size_t pos) {
  float value; std::memcpy(&value, bytes.data() + pos, sizeof(value)); return value;
}
template<class T> void store(std::vector<std::uint8_t>& bytes, std::size_t pos, T value) {
  std::memcpy(bytes.data() + pos, &value, sizeof(value));
}
}  // namespace

int main() {
  try {
    using namespace r36stereo::codec;
    // A non-contiguous OpenCV ROI catches assumptions that cv::Mat is packed.
    cv::Mat larger(3, 5, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat red = larger(cv::Rect(1, 0, 2, 2));
    require(!red.isContinuous(), "ROI should have row padding");
    for (int layout : {HB_DNN_LAYOUT_NCHW, HB_DNN_LAYOUT_NHWC}) {
      auto p = props(HB_DNN_IMG_TYPE_RGB, layout, 2, 2, 3, 3, 3, 4);
      const auto original = p;
      const auto spec = inspect_input(p);
      const auto data = pack_bgr(red, spec);
      const auto& b = data.planes[0];
      const auto r = spec.shape.byte_stride[spec.shape.channel_axis];
      require(b[0] == 255 && b[r] == 0 && b[2 * r] == 0, "RGB channel order must be RGB");
      require(b[2 * spec.shape.byte_stride[spec.shape.width_axis]] == 0, "RGB right padding must remain zero");
      require(std::memcmp(&p, &original, sizeof(p)) == 0, "input inspection must preserve properties");
      p.tensorType = HB_DNN_IMG_TYPE_BGR;
      auto bgr = pack_bgr(red, inspect_input(p));
      require(bgr.planes[0][0] == 0 && bgr.planes[0][2 * r] == 255, "BGR type must preserve BGR order");
      p = props(HB_DNN_TENSOR_TYPE_F32, layout, 2, 2, 3, 3, 3, 4);
      const auto f = inspect_input(p);
      const auto fdata = pack_bgr(red, f);
      require(read_float(fdata.planes[0], 0) == 255.0f, "F32 RGB must be 0..255, not /255");
      require(read_float(fdata.planes[0], f.shape.byte_stride[f.shape.channel_axis]) == 0, "F32 green channel");
    }
    for (int layout : {HB_DNN_LAYOUT_NCHW, HB_DNN_LAYOUT_NHWC}) {
      auto p = props(HB_DNN_IMG_TYPE_NV12, layout, 2, 2, 3, 16, 4);
      const auto spec = inspect_input(p);
      auto continuous = pack_bgr(red, spec);
      require(continuous.planes.size() == 1, "NV12 must have one allocation");
      require(continuous.planes[0][0] == 82 && continuous.planes[0][16] == 82, "NV12 red Y must equal 82");
      require(continuous.planes[0][64] == 90 && continuous.planes[0][65] == 240, "NV12 chroma must be UV, never VU");
      require(continuous.planes[0][2] == 16 && continuous.planes[0][32] == 16, "NV12 Y padding is limited-range black");
      require(continuous.planes[0][66] == 128, "NV12 UV padding is neutral");
      p.tensorType = HB_DNN_IMG_TYPE_NV12_SEPARATE;
      auto separate = pack_bgr(red, inspect_input(p));
      require(separate.planes.size() == 2 && separate.planes[0].size() == 64 && separate.planes[1].size() == 32,
              "NV12_SEPARATE needs separate Y and UV allocations");
      require(separate.planes[1][0] == 90 && separate.planes[1][1] == 240, "NV12_SEPARATE chroma remains UV");
    }
    // The 352 model advertises NCHW [1,3,352,640], but NV12 is still Y+UV,
    // not three planar RGB channels. Exercise the full-size boundary as well.
    {
      auto p = props(HB_DNN_IMG_TYPE_NV12, HB_DNN_LAYOUT_NCHW, 640, 352, 3);
      const auto original = p;
      const auto spec = inspect_input(p);
      require(spec.shape.width == 640 && spec.shape.height == 352 && spec.shape.channels == 3,
              "352 NCHW axes must be C,H,W");
      cv::Mat image(352, 640, CV_8UC3, cv::Scalar(0, 0, 255));
      const auto packed = pack_bgr(image, spec);
      require(packed.planes.size() == 1 && packed.planes[0].size() == 337920,
              "NCHW C=3 NV12 allocation must be 640*352*3/2 bytes");
      require(packed.planes[0][640 * 352 - 1] == 82,
              "352 NV12 final Y pixel");
      require(packed.planes[0][640 * 352] == 90 && packed.planes[0].back() == 240,
              "352 NV12 UV boundary and last chroma byte");
      require(std::memcmp(&p, &original, sizeof(p)) == 0,
              "352 input packing must preserve Horizon metadata");
    }
    // Direct packing preserves source YUV, including values outside RGB gamut,
    // across NCHW/NHWC metadata and contiguous/separate tensor storage.
    for (int layout : {HB_DNN_LAYOUT_NCHW, HB_DNN_LAYOUT_NHWC}) {
      std::vector<std::uint8_t> source(6*8, 0xee);
      for(int y=0;y<6;++y)for(int x=0;x<4;++x)
        source[y*8+x]=std::uint8_t((y*71+x*83)%256);
      cv::Mat raw(6,4,CV_8UC1,source.data(),8);
      for (int type : {HB_DNN_IMG_TYPE_NV12, HB_DNN_IMG_TYPE_NV12_SEPARATE}) {
        const auto p = props(type,layout,4,4,3,16,6);
        const auto original=p;
        const auto spec=inspect_input(p);
        const auto packed=pack_nv12(raw,spec);
        const auto* y_data=packed.planes[0].data();
        const auto* uv_data=type==HB_DNN_IMG_TYPE_NV12_SEPARATE?packed.planes[1].data():y_data+96;
        for(int y=0;y<4;++y)
          require(std::memcmp(y_data+y*16,raw.ptr(y),4)==0,"direct NV12 Y byte identity");
        for(int y=0;y<2;++y)
          require(std::memcmp(uv_data+y*16,raw.ptr(4+y),4)==0,"direct NV12 UV byte identity");
        require(y_data[4]==16 && y_data[4*16]==16,"direct NV12 tensor Y padding");
        require(uv_data[4]==128 && uv_data[2*16]==128,"direct NV12 tensor UV padding");
        require(std::memcmp(&p,&original,sizeof(p))==0,"direct pack must preserve model properties");
        rejected([&]{pack_nv12(cv::Mat(3,4,CV_8UC1),spec);},"exactly match");
      }
      auto rgb=inspect_input(props(HB_DNN_IMG_TYPE_RGB,layout,4,4,3));
      rejected([&]{pack_nv12(raw,rgb);},"NV12 or NV12_SEPARATE");
    }
    // NHWC aligned C=4 makes adjacent valid output pixels four elements apart;
    // NCHW W/H padding catches flat h*w reads that cross a padded row.
    for (int layout : {HB_DNN_LAYOUT_NCHW, HB_DNN_LAYOUT_NHWC}) {
      auto p = props(HB_DNN_TENSOR_TYPE_F32, layout, 2, 2, 1, 4, 3, 4);
      const auto spec = inspect_output(p, 2);
      std::vector<std::uint8_t> bytes(p.alignedByteSize, 0xff);
      for (int y = 0; y < 2; ++y) for (int x = 0; x < 2; ++x)
        store(bytes, y * spec.shape.byte_stride[spec.shape.height_axis] + x * spec.shape.byte_stride[spec.shape.width_axis], float(y * 10 + x + 1));
      const auto out = decode_disparity(spec, bytes.data(), bytes.size());
      require(out.type() == CV_32FC1 && out.at<float>(0, 1) == 2 && out.at<float>(1, 0) == 11, "padded output addressing");
      rejected([&] { decode_disparity(spec, bytes.data(), bytes.size() - 1); }, "smaller than alignedByteSize");
      rejected([&] { inspect_output(p, 3); }, "output width must equal input width");
      if (header_has_stride()) {
        for (int i = 0; i < 4; ++i) set_stride(p, i, static_cast<int>(spec.shape.byte_stride[i]));
        require(inspect_output(p, 2).shape.width == 2, "explicit dense byte strides must work");
        set_stride(p, 3, 3);
        rejected([&] { inspect_output(p, 2); }, "nonstandard stride");
      }
    }
    auto p = props(HB_DNN_TENSOR_TYPE_S16, HB_DNN_LAYOUT_NCHW, 2, 1, 1);
    float scale = 0.25f; std::int8_t zero = 3;
    p.quantiType = SCALE; p.scale.scaleLen = 1; p.scale.scaleData = &scale;
    p.scale.zeroPointLen = 1; p.scale.zeroPointData = &zero;
    std::vector<std::uint8_t> integer(p.alignedByteSize);
    store(integer, 0, std::int16_t(11)); store(integer, 2, std::int16_t(-5));
    auto decoded = decode_disparity(inspect_output(p, 2), integer.data(), integer.size());
    require(decoded.at<float>(0, 0) == 2 && decoded.at<float>(0, 1) == -2, "SCALE zero point must be subtracted");
    p.scale.zeroPointLen = 0;
    require(decode_disparity(inspect_output(p, 2), integer.data(), integer.size()).at<float>(0, 0) == 2.75f, "absent zero point means zero");
    p.scale.scaleLen = 2;
    rejected([&] { inspect_output(p, 2); }, "per-axis metadata is rejected");
    p.scale.scaleLen = 1; p.scale.zeroPointLen = 2;
    rejected([&] { inspect_output(p, 2); }, "zero point");
    p.scale.zeroPointLen = 0; scale = std::numeric_limits<float>::quiet_NaN();
    rejected([&] { inspect_output(p, 2); }, "finite and positive");
    p.quantiType = SHIFT; std::uint8_t shift = 40; p.shift.shiftLen = 1; p.shift.shiftData = &shift;
    require(decode_disparity(inspect_output(p, 2), integer.data(), integer.size()).at<float>(0, 0) == std::ldexp(11.0f, -40),
            "SHIFT>=32 must use ldexp without signed bit-shift overflow");
    shift = 255;
    require(decode_disparity(inspect_output(p, 2), integer.data(), integer.size()).at<float>(0, 0) == 0,
            "SHIFT255 safely underflows to float zero");
    p.shift.shiftLen = 2;
    rejected([&] { inspect_output(p, 2); }, "per-axis metadata is rejected");
    p = props(HB_DNN_TENSOR_TYPE_F32, HB_DNN_LAYOUT_NHWC, 2, 1, 1);
    std::vector<std::uint8_t> floats(p.alignedByteSize);
    store(floats, 0, std::numeric_limits<float>::quiet_NaN());
    store(floats, 4, std::numeric_limits<float>::infinity());
    decoded = decode_disparity(inspect_output(p, 2), floats.data(), floats.size());
    require(std::isnan(decoded.at<float>(0, 0)) && std::isinf(decoded.at<float>(0, 1)), "invalid predictions remain explicit for downstream masking");
    p.tensorType = HB_DNN_TENSOR_TYPE_F16;
    rejected([&] { inspect_output(p, 2); }, "unsupported tensorType");
    p = props(HB_DNN_TENSOR_TYPE_F32, HB_DNN_LAYOUT_NCHW, 2, 2, 1);
    p.quantiType = SCALE;
    rejected([&] { inspect_output(p, 2); }, "F32 output with quantization");
    p = props(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NCHW, 2, 2, 3);
    p.validShape.dimensionSize[0] = p.alignedShape.dimensionSize[0] = 2;
    rejected([&] { inspect_input(p); }, "single batch");
    p = props(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NCHW, 2, 2, 3);
    p.alignedByteSize = 1;
    rejected([&] { inspect_input(p); }, "alignedByteSize");
    p = props(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NCHW, 2, 2, 3);
    p.alignedShape.dimensionSize[3] = 1;
    rejected([&] { inspect_input(p); }, "aligned dimension");
    p = props(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NCHW, 2, 2, 3);
    p.tensorLayout = HB_DNN_LAYOUT_NONE;
    rejected([&] { inspect_input(p); }, "explicit NCHW or NHWC");
    p = props(HB_DNN_IMG_TYPE_NV12, HB_DNN_LAYOUT_NCHW, 3, 2, 3, 16);
    rejected([&] { inspect_input(p); }, "must be even");
    p = props(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NCHW, 2, 2, 3);
    auto spec = inspect_input(p);
    rejected([&] { pack_bgr(cv::Mat(2, 3, CV_8UC3), spec); }, "exactly match");
    rejected([&] { pack_bgr(cv::Mat(2, 2, CV_32FC3), spec); }, "CV_8UC3");
    require(json_string("a\"b\\c\n") == "\"a\\\"b\\\\c\\n\"", "JSON escaping");
    require(json_string(u8"模型.bin") == u8"\"模型.bin\"", "JSON must preserve valid UTF8 model paths");
    require(json_string(std::string(1, char(0xff))) == "\"\\u00ff\"", "JSON must escape invalid UTF8 bytes");
    require(tensor_contract_json(p).find("\"valid_shape\":[1,3,2,2]") != std::string::npos, "contract shape is JSON data");
    std::cout << "PASS tensor codec: " << assertions << " assertions; actual header "
              << HB_DNN_VERSION_MAJOR << '.' << HB_DNN_VERSION_MINOR << '.' << HB_DNN_VERSION_PATCH
              << "; stride=" << header_has_stride() << "; quantizeAxis=" << header_has_quantize_axis() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL tensor codec after " << assertions << " assertions: " << e.what() << '\n';
    return 1;
  }
}
