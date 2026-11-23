#pragma once

// Compile this header against the SAME Horizon headers as the linked libdnn.
// Member detection supports both the actual 1.12.3 x86 and 1.24.5 X5 headers;
// it does not make those different binary ABIs interchangeable.
#include <dnn/hb_dnn.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace r36stereo::codec {

struct Shape {
  int width = 0, height = 0, channels = 0;
  int channel_axis = 0, height_axis = 0, width_axis = 0;
  std::array<std::size_t, 4> byte_stride{};
  std::size_t storage_bytes = 0;
};

struct InputSpec {
  hbDNNTensorProperties properties{};
  Shape shape;
  std::vector<std::size_t> plane_bytes;
};

struct OutputSpec {
  hbDNNTensorProperties properties{};
  Shape shape;
  std::size_t element_bytes = 0;
  double scale = 1.0, zero_point = 0.0;
  int shift = 0;
};

struct PackedInput {
  std::vector<std::vector<std::uint8_t>> planes;
};

// All functions throw std::runtime_error on a rejected/unsafe contract.
// Four-dimensional NCHW/NHWC, batch 1 only. Aligned metadata is preserved.
// Standard dense aligned storage or all-zero/absent stride metadata only.
InputSpec inspect_input(const hbDNNTensorProperties& properties);
OutputSpec inspect_output(const hbDNNTensorProperties& properties,
                          int expected_input_width);
PackedInput pack_bgr(const cv::Mat& image, const InputSpec& spec);
// Direct Y+UV packing; NCHW C=3 image metadata still denotes NV12 storage.
// Requires NV12/NV12_SEPARATE model inputs, and never applies /255 or RGB conversion.
PackedInput pack_nv12(const cv::Mat& image, const InputSpec& spec);
cv::Mat decode_disparity(const OutputSpec& spec, const void* data,
                         std::size_t available_bytes);

bool header_has_stride();
bool header_has_quantize_axis();
std::string tensor_contract_json(const hbDNNTensorProperties& properties);
std::string json_string(const std::string& value);

}  // namespace r36stereo::codec
