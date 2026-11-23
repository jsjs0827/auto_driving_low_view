#include "r36_stereo/horizon_stereo.hpp"
#include "r36_stereo/tensor_codec.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace r36stereo {
namespace {
void check(int rc, const char* operation) {
  if (rc != 0) throw std::runtime_error(std::string(operation) + " failed, hb status=" + std::to_string(rc));
}
void cleanup_error(int rc, const char* operation) noexcept {
  if (rc != 0) std::fprintf(stderr, "HorizonStereoModel cleanup: %s failed, hb status=%d\n", operation, rc);
}
struct PackedHandle {
  hbPackedDNNHandle_t value = nullptr;
  ~PackedHandle() { if (value) cleanup_error(hbDNNRelease(value), "hbDNNRelease"); }
};
struct TensorMemory {
  hbDNNTensor tensor{};
  std::array<bool, 4> allocated{};
  ~TensorMemory() {
    for (int i = 0; i < 4; ++i)
      if (allocated[i]) cleanup_error(hbSysFreeMem(&tensor.sysMem[i]), "hbSysFreeMem");
  }
  void allocate(const hbDNNTensorProperties& properties, const std::vector<std::size_t>& planes) {
    tensor.properties = properties; // No alignment, stride or type overrides.
    if (planes.empty() || planes.size() > 4) throw std::runtime_error("invalid input plane count");
    for (std::size_t i = 0; i < planes.size(); ++i) {
      if (!planes[i] || planes[i] > 512u * 1024u * 1024u) throw std::runtime_error("invalid tensor allocation size");
      check(hbSysAllocCachedMem(&tensor.sysMem[i], static_cast<std::uint32_t>(planes[i])), "hbSysAllocCachedMem");
      allocated[i] = true;
      if (!tensor.sysMem[i].virAddr || tensor.sysMem[i].memSize < planes[i])
        throw std::runtime_error("hbSysAllocCachedMem returned insufficient/null memory");
      std::memset(tensor.sysMem[i].virAddr, 0, planes[i]);
    }
  }
  void write(const codec::PackedInput& packed) {
    for (std::size_t i = 0; i < packed.planes.size(); ++i) {
      if (i >= allocated.size() || !allocated[i] || packed.planes[i].size() > tensor.sysMem[i].memSize)
        throw std::runtime_error("packed input exceeds allocated plane");
      std::memcpy(tensor.sysMem[i].virAddr, packed.planes[i].data(), packed.planes[i].size());
      check(hbSysFlushMem(&tensor.sysMem[i], HB_SYS_MEM_CACHE_CLEAN), "hbSysFlushMem(input CLEAN)");
    }
  }
};
struct Task {
  hbDNNTaskHandle_t value = nullptr;
  bool waited = false;
  ~Task() {
    if (!value) return;
    // The documented timeout<=0 waits until completion. Do not release buffers
    // while a successful submission may still be using them.
    if (!waited) cleanup_error(hbDNNWaitTaskDone(value, 0), "hbDNNWaitTaskDone(cleanup)");
    cleanup_error(hbDNNReleaseTask(value), "hbDNNReleaseTask(cleanup)");
  }
  void wait() {
    const int rc = hbDNNWaitTaskDone(value, 0);
    waited = rc == 0;
    check(rc, "hbDNNWaitTaskDone");
  }
  void release() {
    check(hbDNNReleaseTask(value), "hbDNNReleaseTask");
    value = nullptr;
  }
};
int side(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const bool left = lower.find("left") != std::string::npos;
  const bool right = lower.find("right") != std::string::npos;
  if (left == right) return -1;
  return left ? 0 : 1;
}
}  // namespace

struct HorizonStereoModel::Impl {
  // Destruction runs outputs/inputs before the packed model. Local Task is
  // always destroyed before an inference can unwind past these owners.
  PackedHandle packed;
  hbDNNHandle_t dnn = nullptr;
  std::array<TensorMemory, 2> inputs;
  TensorMemory output;
  std::array<codec::InputSpec, 2> input_specs;
  codec::OutputSpec output_spec;
  std::array<std::string, 2> input_names;
  std::array<int, 2> side_slot{{-1, -1}};
  std::string path, model_name, output_name, runtime_version;
  bool require_nv12 = false;
  std::mutex mutex;
  bool poisoned = false;

  explicit Impl(const std::string& model, bool require) : path(model), require_nv12(require) {
    if (model.empty()) throw std::runtime_error("model path is empty");
    const char* file = model.c_str();
    check(hbDNNInitializeFromFiles(&packed.value, &file, 1), "hbDNNInitializeFromFiles");
    if (!packed.value) throw std::runtime_error("hbDNNInitializeFromFiles succeeded with null handle");
    const char** names = nullptr;
    int count = 0;
    check(hbDNNGetModelNameList(&names, &count, packed.value), "hbDNNGetModelNameList");
    if (count != 1 || !names || !names[0])
      throw std::runtime_error("stereo runner requires exactly one named model in the packed file");
    model_name = names[0];
    check(hbDNNGetModelHandle(&dnn, packed.value, names[0]), "hbDNNGetModelHandle");
    if (!dnn) throw std::runtime_error("hbDNNGetModelHandle returned null");
    int input_count = 0, output_count = 0;
    check(hbDNNGetInputCount(&input_count, dnn), "hbDNNGetInputCount");
    check(hbDNNGetOutputCount(&output_count, dnn), "hbDNNGetOutputCount");
    if (input_count != 2 || output_count != 1)
      throw std::runtime_error("stereo model must have exactly two inputs and one disparity output");
    for (int i = 0; i < 2; ++i) {
      const char* name = nullptr;
      check(hbDNNGetInputName(&name, dnn, i), "hbDNNGetInputName");
      if (!name || !*name) throw std::runtime_error("stereo input has an empty name");
      input_names[i] = name;
      const int which_side = side(input_names[i]);
      if (which_side < 0 || side_slot[which_side] >= 0)
        throw std::runtime_error("input names must identify one 'left' and one 'right' input; ambiguous order is rejected");
      side_slot[which_side] = i;
      hbDNNTensorProperties properties{};
      check(hbDNNGetInputTensorProperties(&properties, dnn, i), "hbDNNGetInputTensorProperties");
      input_specs[i] = codec::inspect_input(properties);
    }
    if (input_specs[0].shape.width != input_specs[1].shape.width ||
        input_specs[0].shape.height != input_specs[1].shape.height)
      throw std::runtime_error("left/right model inputs must have equal width and height");
    if (require_nv12 && !supports_nv12())
      throw std::runtime_error("direct NV12 stereo requires NV12/NV12_SEPARATE tensorType for both model inputs");
    const char* name = nullptr;
    check(hbDNNGetOutputName(&name, dnn, 0), "hbDNNGetOutputName");
    if (!name || !*name) throw std::runtime_error("disparity output has an empty name");
    output_name = name;
    hbDNNTensorProperties properties{};
    check(hbDNNGetOutputTensorProperties(&properties, dnn, 0), "hbDNNGetOutputTensorProperties");
    output_spec = codec::inspect_output(properties, input_specs[0].shape.width);
    // Validate every contract before any large tensor allocation.
    for (int i = 0; i < 2; ++i)
      inputs[i].allocate(input_specs[i].properties, input_specs[i].plane_bytes);
    output.allocate(output_spec.properties, {static_cast<std::size_t>(properties.alignedByteSize)});
    const char* version = hbDNNGetVersion();
    runtime_version = version ? version : "unknown";
  }

  bool supports_nv12() const {
    const auto yuv = [](int type) {
      return type == HB_DNN_IMG_TYPE_NV12 || type == HB_DNN_IMG_TYPE_NV12_SEPARATE;
    };
    return yuv(input_specs[0].properties.tensorType) && yuv(input_specs[1].properties.tensorType);
  }

  // The caller holds mutex. Both inputs are fully packed/validated before any
  // tensor memory is modified. User input errors do not poison a model session.
  cv::Mat run(const codec::PackedInput& left_data, const codec::PackedInput& right_data) {
    try {
      inputs[side_slot[0]].write(left_data);
      inputs[side_slot[1]].write(right_data);
      std::array<hbDNNTensor, 2> input_tensors{{inputs[0].tensor, inputs[1].tensor}};
      hbDNNTensor* output_ptr = &output.tensor;
      hbDNNInferCtrlParam control{};
      HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
      Task task;
      check(hbDNNInfer(&task.value, &output_ptr, input_tensors.data(), dnn, &control), "hbDNNInfer");
      if (!task.value) throw std::runtime_error("hbDNNInfer succeeded with null task");
      if (output_ptr != &output.tensor) throw std::runtime_error("hbDNNInfer unexpectedly replaced caller-owned output tensor");
      task.wait();
      check(hbSysFlushMem(&output.tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE), "hbSysFlushMem(output INVALIDATE)");
      cv::Mat result = codec::decode_disparity(output_spec, output.tensor.sysMem[0].virAddr,
                                              output.tensor.sysMem[0].memSize);
      task.release();
      return result;
    } catch (...) {
      poisoned = true; // Never submit another task after any hb/runtime error.
      throw;
    }
  }

  cv::Mat infer(const cv::Mat& left, const cv::Mat& right, bool direct_nv12) {
    std::lock_guard<std::mutex> lock(mutex);
    if (poisoned) throw std::runtime_error("Horizon model session is unusable after a previous runtime failure; recreate it");
    if (direct_nv12 && !supports_nv12())
      throw std::runtime_error("infer_nv12 requires both model inputs to have an NV12 image tensorType");
    const auto left_data = direct_nv12 ? codec::pack_nv12(left, input_specs[side_slot[0]])
                                       : codec::pack_bgr(left, input_specs[side_slot[0]]);
    const auto right_data = direct_nv12 ? codec::pack_nv12(right, input_specs[side_slot[1]])
                                        : codec::pack_bgr(right, input_specs[side_slot[1]]);
    return run(left_data, right_data);
  }

  std::string json() const {
    std::ostringstream out;
    out << "{\"model_path\":" << codec::json_string(path)
        << ",\"model_name\":" << codec::json_string(model_name)
        << ",\"runtime_version\":" << codec::json_string(runtime_version)
        << ",\"header_version\":\"" << HB_DNN_VERSION_MAJOR << '.' << HB_DNN_VERSION_MINOR << '.' << HB_DNN_VERSION_PATCH << '"'
        << ",\"header_has_stride\":" << (codec::header_has_stride() ? "true" : "false")
        << ",\"header_has_quantize_axis\":" << (codec::header_has_quantize_axis() ? "true" : "false")
        << ",\"input_storage\":\"unaltered alignedShape; dense aligned byte strides or zero/absent stride metadata\""
        << ",\"direct_nv12_supported\":" << (supports_nv12() ? "true" : "false")
        << ",\"require_nv12\":" << (require_nv12 ? "true" : "false")
        << ",\"input_image\":\"infer_nv12: original Y plus interleaved UV, explicit separate-plane resize, no RGB round trip or host /255; infer: legacy BGR CV_8UC3 packing\""
        << ",\"nv12_normalization\":\"compiled model preprocessing owns RGB conversion/data_scale; host copies NV12 bytes without additional normalization\""
        << ",\"output_units\":\"horizontal disparity pixels at input width; model semantics must be independently confirmed\""
        << ",\"output_invalid_policy\":\"preserve raw nonpositive/nonfinite values; downstream maps invalid disparity to Q8 zero\""
        << ",\"quantization_support\":\"NONE or per-tensor SCALE with optional zero point or SHIFT; per-axis rejected\""
        << ",\"inputs\":[";
    for (int i = 0; i < 2; ++i) {
      if (i) out << ',';
      out << "{\"index\":" << i << ",\"name\":" << codec::json_string(input_names[i])
          << ",\"side\":" << codec::json_string(side_slot[0] == i ? "left" : "right")
          << ",\"tensor\":" << codec::tensor_contract_json(input_specs[i].properties)
          << ",\"plane_bytes\":[";
      for (std::size_t j = 0; j < input_specs[i].plane_bytes.size(); ++j) { if (j) out << ','; out << input_specs[i].plane_bytes[j]; }
      out << "]}";
    }
    out << "],\"output\":{\"name\":" << codec::json_string(output_name)
        << ",\"tensor\":" << codec::tensor_contract_json(output_spec.properties) << "}}";
    return out.str();
  }
};

HorizonStereoModel::HorizonStereoModel(const std::string& model, bool require_nv12) : impl_(std::make_unique<Impl>(model, require_nv12)) {}
HorizonStereoModel::~HorizonStereoModel() = default;
cv::Size HorizonStereoModel::input_size() const { return {impl_->input_specs[0].shape.width, impl_->input_specs[0].shape.height}; }
bool HorizonStereoModel::supports_nv12() const { return impl_->supports_nv12(); }
cv::Mat HorizonStereoModel::infer_nv12(const cv::Mat& left, const cv::Mat& right) { return impl_->infer(left, right, true); }
cv::Mat HorizonStereoModel::infer(const cv::Mat& left, const cv::Mat& right) { return impl_->infer(left, right, false); }
std::string HorizonStereoModel::contract_json() const { return impl_->json(); }
}  // namespace r36stereo
