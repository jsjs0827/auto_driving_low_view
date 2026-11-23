#include "detail/diagnostic_boundary.hpp"
#include "perception/backend.hpp"
#include "perception/preprocess.hpp"
#include <algorithm>
#include <filesystem>
#include <onnxruntime_cxx_api.h>
#include <stdexcept>

namespace perception {
namespace {
// ORT diagnostics must not bypass the SDK log switch. API failures are captured
// with structured codes at the call boundary. No session-owned callback pointer.
void ORT_API_CALL quietOrt(void *, OrtLoggingLevel, const char *, const char *, const char *,
                           const char *) {}
class OrtBackend final : public Backend {
    ModelConfig cfg_;
    ModelInfo metadata_;
    bool closed_ = false;
    Ort::Env env_{ORT_LOGGING_LEVEL_FATAL, "perception", quietOrt, nullptr};
    Ort::Session session_{nullptr};
    std::string input_name_;
    std::vector<std::string> output_names_;
    int width_ = 0, height_ = 0;

  public:
    explicit OrtBackend(const ModelConfig &c) : cfg_(c) {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(c.backend.cpu_threads);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        if (c.backend.device == Device::CUDA) {
            const auto providers = Ort::GetAvailableProviders();
            if (std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") ==
                providers.end())
                throw std::runtime_error(
                    "CUDAExecutionProvider unavailable; refusing silent CPU fallback");
            OrtCUDAProviderOptions cuda{};
            cuda.device_id = c.backend.device_id;
            cuda.do_copy_in_default_stream = 1;
            options.AppendExecutionProvider_CUDA(cuda); // failure propagates; no CPU retry
        }
        const auto path = std::filesystem::u8path(c.source.path);
        session_ = detail::boundary(ErrorCode::ModelLoadFailed, "ort.model.load",
                                    [&] { return Ort::Session(env_, path.c_str(), options); });
        if (session_.GetInputCount() != 1)
            throw std::runtime_error("expected one float32 NCHW input");
        const auto input_type = session_.GetInputTypeInfo(0);
        const auto info = input_type.GetTensorTypeAndShapeInfo();
        const auto input_shape = info.GetShape();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            input_shape.size() != 4 || (input_shape[0] != 1 && input_shape[0] != -1) ||
            input_shape[1] != 3)
            throw std::runtime_error("ONNX input must be float32 [1/dynamic,3,H,W]");
        auto dim = [](std::int64_t model, int specified) {
            if (model > 4096 || model == 0 || model < -1)
                throw std::runtime_error("unsupported input dimension");
            if (model > 0 && specified > 0 && model != specified)
                throw std::runtime_error("ONNX static size differs from profile");
            const int result = model > 0 ? int(model) : specified;
            if (result <= 0 || result > 4096)
                throw std::runtime_error("dynamic ONNX input requires width/height profile");
            return result;
        };
        height_ = dim(input_shape[2], c.preprocess.height);
        width_ = dim(input_shape[3], c.preprocess.width);
        Ort::AllocatorWithDefaultOptions alloc;
        input_name_ = session_.GetInputNameAllocated(0, alloc).get();
        const auto count = session_.GetOutputCount();
        if (count == 0 || count > 64)
            throw std::runtime_error("unsupported output count");
        metadata_.width = width_;
        metadata_.height = height_;
        for (std::size_t i = 0; i < count; ++i) {
            output_names_.emplace_back(session_.GetOutputNameAllocated(i, alloc).get());
            const auto type = session_.GetOutputTypeInfo(i);
            const auto tensor = type.GetTensorTypeAndShapeInfo();
            const auto shape = tensor.GetShape();
            if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                (shape.size() != 4 && shape.size() != 2) || (shape[0] != 1 && shape[0] != -1))
                throw std::runtime_error("expected float32 rank-2 or rank-4 output logits");
            for (auto d : shape)
                if (d == 0 || d < -1 || d > 4096)
                    throw std::runtime_error("unsupported ONNX output metadata");
            const bool nhwc = cfg_.output_layout == Layout::NHWC;
            TensorShape desc=shape.size()==2?TensorShape{int(shape[1]),1,1}:
                TensorShape{int(shape[nhwc ? 3 : 1]), int(shape[nhwc ? 1 : 2]),int(shape[nhwc ? 2 : 3])};
            desc.validate(true);
            metadata_.outputs.push_back(desc);
        }
        metadata_.output_names = output_names_;
    }
    std::array<int, 2> inputSize() const override { return {width_, height_}; }
    ModelInfo modelInfo() const override { return metadata_; }
    BackendState state() const noexcept override {
        return closed_ ? BackendState::Closed : BackendState::Ready;
    }
    void close() override {
        session_ = Ort::Session{nullptr};
        closed_ = true;
    }
    std::vector<Tensor> run(const std::vector<std::uint8_t> &rgb) override {
        if (closed_)
            throw std::runtime_error("ORT backend is closed");
        if (rgb.size() != std::size_t(width_) * height_ * 3)
            throw std::invalid_argument("ORT input size mismatch");
        auto values = normalize(rgb, cfg_.preprocess);
        const std::array<std::int64_t, 4> shape{1, 3, height_, width_};
        const auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto input = Ort::Value::CreateTensor<float>(mem, values.data(), values.size(),
                                                     shape.data(), shape.size());
        const char *input_name = input_name_.c_str();
        std::vector<const char *> names;
        for (const auto &name : output_names_)
            names.push_back(name.c_str());
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, &input_name, &input, 1, names.data(),
                                    names.size());
        return detail::boundary(ErrorCode::InferenceResultInvalid, "ort.outputs", [&] {
            std::vector<Tensor> result;
            for (auto &out : outputs) {
                if (!out.IsTensor())
                    throw std::runtime_error("non-tensor ONNX output");
                const auto info = out.GetTensorTypeAndShapeInfo();
                const auto s = info.GetShape();
                if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || (s.size() != 4 && s.size() != 2) ||
                    s[0] != 1)
                    throw std::runtime_error("expected float32, rank 2/4, batch 1 output logits");
                for (auto d : s)
                    if (d <= 0 || d > 4096)
                        throw std::runtime_error("invalid ONNX output shape");
                const bool nhwc = cfg_.output_layout == Layout::NHWC;
                Tensor t;
                t.channels = int(s.size()==2?s[1]:s[nhwc ? 3 : 1]);
                t.height = s.size()==2?1:int(s[nhwc ? 1 : 2]);
                t.width = s.size()==2?1:int(s[nhwc ? 2 : 3]);
                const auto count = info.GetElementCount();
                if (t.channels > 1024 || count > 64U * 1024U * 1024U)
                    throw std::runtime_error("ONNX output exceeds limit");
                t.data.resize(count);
                const float *data = out.GetTensorData<float>();
                if (!nhwc)
                    std::copy(data, data + count, t.data.begin());
                else
                    for (int y = 0; y < t.height; ++y)
                        for (int x = 0; x < t.width; ++x)
                            for (int c = 0; c < t.channels; ++c)
                                t.data[(std::size_t(c) * t.height + y) * t.width + x] =
                                    data[(std::size_t(y) * t.width + x) * t.channels + c];
                t.validate();
                result.push_back(std::move(t));
            }
            return result;
        });
    }
};
} // namespace
std::unique_ptr<Backend> makeOrt(const ModelConfig &c) {
    return std::make_unique<OrtBackend>(c);
}
} // namespace perception
