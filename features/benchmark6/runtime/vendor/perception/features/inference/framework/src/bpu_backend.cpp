#include "detail/diagnostic_boundary.hpp"
#include "detail/bpu_trace.hpp"
#include "detail/native_detection.hpp"
#include "detail/preprocess_workspace.hpp"
#include "detail/reusable_backend.hpp"
#include "detail/tensor_conversion.hpp"
#include "perception/backend.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#if defined(__linux__)
#include <ctime>
#endif
#include <dnn/hb_dnn.h>
#include <dnn/hb_sys.h>
#include <limits>
#include <stdexcept>

namespace perception {
namespace detail {
static thread_local BpuTrace g_last_bpu_trace{};

BpuTrace lastBpuTrace() noexcept { return g_last_bpu_trace; }
} // namespace detail

namespace {
std::uint64_t traceWallNs() noexcept {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint64_t traceThreadCpuNs() noexcept {
#if defined(__linux__)
    timespec value{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0)
        return 0;
    return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
           static_cast<std::uint64_t>(value.tv_nsec);
#else
    return 0;
#endif
}

class TraceScope {
    detail::BpuTraceStage &stage_;
    std::uint64_t wall_begin_, cpu_begin_;

  public:
    explicit TraceScope(detail::BpuTraceStage &stage) noexcept
        : stage_(stage), wall_begin_(traceWallNs()),
          cpu_begin_(traceThreadCpuNs()) {}
    TraceScope(const TraceScope &) = delete;
    TraceScope &operator=(const TraceScope &) = delete;
    ~TraceScope() noexcept {
        const auto cpu_end = traceThreadCpuNs();
        const auto wall_end = traceWallNs();
        if (wall_begin_ && wall_end >= wall_begin_)
            stage_.wall_ns += wall_end - wall_begin_;
        if (cpu_begin_ && cpu_end >= cpu_begin_)
            stage_.thread_cpu_ns += cpu_end - cpu_begin_;
    }
};

void check(int code, const char *operation) {
    if (code != 0)
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(code));
}
void require(bool b, const char *s) {
    if (!b)
        throw std::runtime_error(s);
}
struct SessionResources {
    SessionResources() = default;
    SessionResources(const SessionResources &) = delete;
    SessionResources &operator=(const SessionResources &) = delete;
    hbPackedDNNHandle_t packed = nullptr;
    hbDNNHandle_t model = nullptr;
    hbDNNTaskHandle_t task = nullptr;
    hbDNNTensor input{};
    std::vector<hbDNNTensor> outputs;
    BackendState phase = BackendState::Ready;
    std::shared_ptr<Logger> logger;
    void cleanupError(const char *detail) noexcept {
        if (!logger || !logger->enabled(LogLevel::Error))
            return;
        try {
            reportError(logger, ErrorCode::PerceptionStateInvalid, detail, "bpu.cleanup");
        } catch (...) {
        } // Destructors and task guards cannot throw.
    }
    ~SessionResources() {
        // Only destroyed when there is no unconfirmed task using these buffers.
        for (auto &t : outputs)
            if (t.sysMem[0].virAddr && hbSysFreeMem(&t.sysMem[0]) != 0)
                cleanupError("BPU output free failed");
        if (input.sysMem[0].virAddr && hbSysFreeMem(&input.sysMem[0]) != 0)
            cleanupError("BPU input free failed");
        if (packed && hbDNNRelease(packed) != 0)
            cleanupError("BPU model release failed");
    }
    int releaseTask() {
        if (!task) {
            phase = BackendState::Ready;
            return 0;
        }
        const int code = hbDNNReleaseTask(task);
        if (code == 0) {
            task = nullptr;
            phase = BackendState::Ready;
        } else
            phase = BackendState::Quarantined;
        return code;
    }
};
void allocate(hbDNNTensor &t) {
    require(t.properties.alignedByteSize > 0 && t.properties.alignedByteSize <= 256 * 1024 * 1024,
            "invalid BPU allocation size");
    const int code =
        hbSysAllocCachedMem(&t.sysMem[0], static_cast<std::uint32_t>(t.properties.alignedByteSize));
    if (code != 0)
        throw PerceptionException(ErrorCode::AiInitFailed,
                                  "hbSysAllocCachedMem failed: " + std::to_string(code),
                                  "bpu.memory.allocate");
    if (!t.sysMem[0].virAddr ||
        t.sysMem[0].memSize < std::uint32_t(t.properties.alignedByteSize))
        throw PerceptionException(ErrorCode::AiInitFailed,
                                  "BPU allocator returned short/null storage",
                                  "bpu.memory.allocate");
}
// Every supported output becomes owned CHW float once. Both tasks use this converter.
Tensor validateOutput(const hbDNNTensor &t) {
    const auto &p = t.properties;
    require(p.validShape.numDimensions == 4 && p.alignedShape.numDimensions == 4 &&
                p.validShape.dimensionSize[0] == 1,
            "BPU output must be rank 4, batch 1");
    require(p.tensorLayout == HB_DNN_LAYOUT_NCHW || p.tensorLayout == HB_DNN_LAYOUT_NHWC,
            "unsupported BPU output layout");
    const bool nhwc = p.tensorLayout == HB_DNN_LAYOUT_NHWC;
    const int ca = nhwc ? 3 : 1, ha = nhwc ? 1 : 2, wa = nhwc ? 2 : 3;
    std::size_t aligned = 1;
    for (int i = 0; i < 4; ++i) {
        const int v = p.validShape.dimensionSize[i], a = p.alignedShape.dimensionSize[i];
        require(v > 0 && a >= v && a <= 8192, "invalid BPU valid/aligned shape");
        require(aligned <= 64U * 1024U * 1024U / std::size_t(a),
                "BPU aligned tensor exceeds limit");
        aligned *= a;
    }
    int element = 0;
    switch (p.tensorType) {
    case HB_DNN_TENSOR_TYPE_F32:
    case HB_DNN_TENSOR_TYPE_S32:
        element = 4;
        break;
    case HB_DNN_TENSOR_TYPE_S16:
        element = 2;
        break;
    case HB_DNN_TENSOR_TYPE_S8:
    case HB_DNN_TENSOR_TYPE_U8:
        element = 1;
        break;
    default:
        throw std::runtime_error("unsupported BPU type (need F32/S32/S16/S8/U8)");
    }
    require(p.alignedByteSize > 0 && aligned * element <= std::size_t(p.alignedByteSize) &&
                t.sysMem[0].virAddr && std::size_t(p.alignedByteSize) <= t.sysMem[0].memSize,
            "BPU output storage too small");
    Tensor out;
    out.channels = p.validShape.dimensionSize[ca];
    out.height = p.validShape.dimensionSize[ha];
    out.width = p.validShape.dimensionSize[wa];
    require(out.channels <= 1024 && out.width <= 4096 && out.height <= 4096,
            "BPU valid tensor exceeds limit");
    auto validQuantLength = [&](int len) {
        return len == 1 || (len == out.channels && p.quantizeAxis == ca);
    };
    if (p.tensorType == HB_DNN_TENSOR_TYPE_F32)
        require(p.quantiType == NONE, "float output unexpectedly quantized");
    if (p.quantiType == SCALE) {
        require(p.scale.scaleData && validQuantLength(p.scale.scaleLen), "invalid BPU scales/axis");
        require(p.scale.zeroPointLen == 0 ||
                    (p.scale.zeroPointLen == p.scale.scaleLen && p.scale.zeroPointData),
                "invalid BPU zero points");
        for (int i = 0; i < p.scale.scaleLen; ++i)
            require(std::isfinite(p.scale.scaleData[i]) && p.scale.scaleData[i] > 0,
                    "invalid BPU scale value");
    } else if (p.quantiType == SHIFT) {
        require(p.shift.shiftData && validQuantLength(p.shift.shiftLen), "invalid BPU shift/axis");
        for (int i = 0; i < p.shift.shiftLen; ++i)
            require(p.shift.shiftData[i] <= 126, "invalid BPU shift value");
    } else
        require(p.quantiType == NONE, "unknown BPU quantization");
    return out;
}
template <class Scalar, Layout L>
bool convertQuantizedInto(const hbDNNTensor &t, const detail::ConversionView &view, Tensor &out) {
    const auto &p = t.properties;
    if constexpr (std::is_same_v<Scalar, float>)
        return detail::convertTensorTrustedInto<Scalar, L>(view, detail::IdentityQuant{}, out);
    else {
        switch (p.quantiType) {
        case SCALE:
            return detail::convertTensorTrustedInto<Scalar, L>(
                view, detail::ScaleQuant{p.scale.scaleData,
                                         p.scale.zeroPointLen ? p.scale.zeroPointData : nullptr,
                                         p.scale.scaleLen}, out);
        case SHIFT:
            return detail::convertTensorTrustedInto<Scalar, L>(
                view, detail::ShiftQuant{p.shift.shiftData, p.shift.shiftLen}, out);
        default:
            return detail::convertTensorTrustedInto<Scalar, L>(view, detail::IdentityQuant{}, out);
        }
    }
}
template <class Scalar>
bool convertLayoutInto(const hbDNNTensor &t, const detail::ConversionView &view, Tensor &out) {
    return t.properties.tensorLayout == HB_DNN_LAYOUT_NHWC
               ? convertQuantizedInto<Scalar, Layout::NHWC>(t, view, out)
               : convertQuantizedInto<Scalar, Layout::NCHW>(t, view, out);
}
bool convertInto(const hbDNNTensor &t, const TensorShape &shape, Tensor &out) {
    const auto &p = t.properties;
    const bool nhwc = p.tensorLayout == HB_DNN_LAYOUT_NHWC;
    detail::ConversionView view{t.sysMem[0].virAddr,
                                t.sysMem[0].memSize,
                                shape,
                                p.alignedShape.dimensionSize[nhwc ? 3 : 1],
                                p.alignedShape.dimensionSize[nhwc ? 1 : 2],
                                p.alignedShape.dimensionSize[nhwc ? 2 : 3]};
    switch (p.tensorType) {
    case HB_DNN_TENSOR_TYPE_F32:
        return convertLayoutInto<float>(t, view, out);
    case HB_DNN_TENSOR_TYPE_S32:
        return convertLayoutInto<std::int32_t>(t, view, out);
    case HB_DNN_TENSOR_TYPE_S16:
        return convertLayoutInto<std::int16_t>(t, view, out);
    case HB_DNN_TENSOR_TYPE_S8:
        return convertLayoutInto<std::int8_t>(t, view, out);
    case HB_DNN_TENSOR_TYPE_U8:
        return convertLayoutInto<std::uint8_t>(t, view, out);
    default:
        throw std::runtime_error("unsupported BPU tensor type");
    }
}
detail::NativeTensorView nativeView(const hbDNNTensor &t, const TensorShape &shape) {
    const auto &p = t.properties;
    const bool nhwc = p.tensorLayout == HB_DNN_LAYOUT_NHWC;
    detail::NativeTensorView view;
    view.data = t.sysMem[0].virAddr;
    view.shape = shape;
    view.aligned_c = p.alignedShape.dimensionSize[nhwc ? 3 : 1];
    view.aligned_h = p.alignedShape.dimensionSize[nhwc ? 1 : 2];
    view.aligned_w = p.alignedShape.dimensionSize[nhwc ? 2 : 3];
    view.layout = nhwc ? Layout::NHWC : Layout::NCHW;
    switch (p.tensorType) {
    case HB_DNN_TENSOR_TYPE_F32: view.scalar = detail::NativeScalar::F32; break;
    case HB_DNN_TENSOR_TYPE_S32: view.scalar = detail::NativeScalar::S32; break;
    case HB_DNN_TENSOR_TYPE_S16: view.scalar = detail::NativeScalar::S16; break;
    case HB_DNN_TENSOR_TYPE_S8: view.scalar = detail::NativeScalar::S8; break;
    case HB_DNN_TENSOR_TYPE_U8: view.scalar = detail::NativeScalar::U8; break;
    default: throw std::runtime_error("unsupported native BPU tensor type");
    }
    if (p.quantiType == SCALE) {
        view.quant = detail::NativeQuant::Scale;
        view.scales = p.scale.scaleData;
        view.zeros = p.scale.zeroPointLen ? p.scale.zeroPointData : nullptr;
        view.quant_length = p.scale.scaleLen;
    } else if (p.quantiType == SHIFT) {
        view.quant = detail::NativeQuant::Shift;
        view.shifts = p.shift.shiftData;
        view.quant_length = p.shift.shiftLen;
    }
    return view;
}
// Scope guard handles exceptions between submit, wait and release. Resources stay
// attached to the backend if task release cannot be confirmed.
class InFlightTask {
    SessionResources &resources_;

  public:
    explicit InFlightTask(SessionResources &r) : resources_(r) {
        resources_.phase = BackendState::Running;
    }
    ~InFlightTask() {
        if (resources_.phase == BackendState::Running && resources_.releaseTask() != 0)
            resources_.cleanupError("BPU task scope cleanup failed; session quarantined");
    }
    int finish() { return resources_.releaseTask(); }
};
class BpuBackend final : public Backend, public detail::ReusableBackend,
                         public detail::NativeDetectionBackend {
    std::unique_ptr<SessionResources> s_ = std::make_unique<SessionResources>();
    ModelInfo metadata_;
    std::vector<TensorShape> output_shapes_;
    std::vector<Tensor> converted_;
    std::vector<detail::NativeTensorView> native_views_;
    int width_ = 0, height_ = 0, timeout_ = 0;

    const std::vector<Tensor> &execute(const std::uint8_t *input, std::size_t input_bytes,
                                       bool nv12, const detail::OutputSelection &selection,
                                       bool materialize = true) {
        auto &trace = detail::g_last_bpu_trace;
        trace = {};
        trace.mode = materialize ? detail::BpuTraceMode::Materialized
                                 : detail::BpuTraceMode::NativeDetection;
        trace.first_tensor_type = -1;
        require(s_ && s_->phase == BackendState::Ready,
                "BPU session is not ready (closed, running or quarantined)");
        const auto expected = std::size_t(width_) * height_ * 3 / 2;
        require(input, "null BPU input");
        require(selection.all || (selection.count > 0 && selection.count <= selection.indices.size()),
                "invalid BPU output selection");
        std::array<bool, 64> selected{};
        if (selection.all) {
            std::fill_n(selected.begin(), s_->outputs.size(), true);
        } else {
            for (std::size_t i = 0; i < selection.count; ++i) {
                const int index = selection.indices[i];
                require(index >= 0 && std::size_t(index) < s_->outputs.size(),
                        "BPU output selection out of range");
                require(!selected[std::size_t(index)], "duplicate BPU output selection");
                selected[std::size_t(index)] = true;
            }
        }

        {
            TraceScope measure(trace.input_copy_flush);
            auto *target = static_cast<std::uint8_t *>(s_->input.sysMem[0].virAddr);
            if (nv12) {
                require(input_bytes == expected, "BPU NV12 size mismatch");
                std::memcpy(target, input, expected);
            } else {
                detail::rgbToNv12Into(input, input_bytes, width_, height_, target,
                                       s_->input.sysMem[0].memSize);
            }
            if (s_->input.sysMem[0].memSize > expected)
                std::memset(target + expected, 0, s_->input.sysMem[0].memSize - expected);
            check(hbSysFlushMem(&s_->input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
                  "BPU input cache clean");
        }

        hbDNNInferCtrlParam ctrl;
        HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
        hbDNNTensor *output = s_->outputs.data();
        InFlightTask task_scope(*s_);
        int infer = 0;
        {
            TraceScope measure(trace.submit);
            infer = hbDNNInfer(&s_->task, &output, &s_->input, s_->model, &ctrl);
        }
        if (infer != 0) {
            const int release = task_scope.finish();
            check(release, "release after infer failure");
            check(infer, "hbDNNInfer");
        }
        if (!s_->task) {
            s_->phase = BackendState::Quarantined;
            throw std::runtime_error("hbDNNInfer returned null task; session quarantined");
        }
        int wait = 0;
        {
            TraceScope measure(trace.wait);
            wait = hbDNNWaitTaskDone(s_->task, timeout_);
        }
        int release = 0;
        {
            TraceScope measure(trace.release);
            release = task_scope.finish();
        }
        check(release, "hbDNNReleaseTask (session quarantined)");
        check(wait, "hbDNNWaitTaskDone");

        for (std::size_t i = 0; i < s_->outputs.size(); ++i) {
            if (!selected[i])
                continue;
            auto &tensor = s_->outputs[i];
            const auto &shape = output_shapes_[i];
            const auto tensor_type = static_cast<std::int32_t>(tensor.properties.tensorType);
            if (trace.output_count == 0)
                trace.first_tensor_type = tensor_type;
            else if (trace.first_tensor_type != tensor_type)
                trace.mixed_tensor_types = 1;
            ++trace.output_count;
            const auto elements = static_cast<std::uint64_t>(shape.channels) *
                                  static_cast<std::uint64_t>(shape.height) *
                                  static_cast<std::uint64_t>(shape.width);
            trace.output_elements += elements;
            if (tensor.properties.tensorType == HB_DNN_TENSOR_TYPE_F32)
                trace.f32_output_elements += elements;
            {
                TraceScope measure(trace.invalidate);
                check(hbSysFlushMem(&tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE),
                      "BPU output cache invalidate");
            }
            {
                TraceScope measure(trace.output_convert_or_validate);
                detail::boundary(ErrorCode::InferenceResultInvalid, "bpu.outputs", [&] {
                    if (materialize) {
                        if (!convertInto(tensor, output_shapes_[i], converted_[i]))
                            throw std::runtime_error("nonfinite model output");
                    } else if (!native_views_[i].allFinite()) {
                        throw std::runtime_error("nonfinite model output");
                    }
                });
            }
        }
        trace.valid = 1;
        return converted_;
    }

  public:
    explicit BpuBackend(const ModelConfig &c) : timeout_(c.backend.bpu_timeout_ms) {
        s_->logger = c.logger;
        const char *file = c.source.path.c_str();
        detail::boundary(ErrorCode::ModelLoadFailed, "bpu.model.load", [&] {
            check(hbDNNInitializeFromFiles(&s_->packed, &file, 1), "hbDNNInitializeFromFiles");
        });
        const char **names = nullptr;
        int count = 0;
        check(hbDNNGetModelNameList(&names, &count, s_->packed), "hbDNNGetModelNameList");
        require(names && count > 0, "empty model package");
        require(!c.source.model_name.empty() || count == 1, "multiple BPU models: set model_name");
        const char *selected = nullptr;
        for (int i = 0; i < count; ++i)
            if (names[i] && (c.source.model_name.empty() || c.source.model_name == names[i]))
                selected = names[i];
        require(selected, "model_name not in package");
        check(hbDNNGetModelHandle(&s_->model, s_->packed, selected), "hbDNNGetModelHandle");
        check(hbDNNGetInputCount(&count, s_->model), "hbDNNGetInputCount");
        require(count == 1, "BPU expects one NV12 input");
        check(hbDNNGetInputTensorProperties(&s_->input.properties, s_->model, 0),
              "hbDNNGetInputTensorProperties");
        auto &p = s_->input.properties;
        require(p.tensorType == HB_DNN_IMG_TYPE_NV12 && p.validShape.numDimensions == 4 &&
                    p.validShape.dimensionSize[0] == 1,
                "BPU input must be single-plane NV12, rank 4, batch 1");
        require(p.tensorLayout == HB_DNN_LAYOUT_NCHW || p.tensorLayout == HB_DNN_LAYOUT_NHWC,
                "unknown BPU input layout");
        const bool nhwc = p.tensorLayout == HB_DNN_LAYOUT_NHWC;
        width_ = p.validShape.dimensionSize[nhwc ? 2 : 3];
        height_ = p.validShape.dimensionSize[nhwc ? 1 : 2];
        require(width_ > 0 && height_ > 0 && width_ <= 4096 && height_ <= 4096 && width_ % 2 == 0 &&
                    height_ % 2 == 0,
                "invalid NV12 input size");
        require(c.preprocess.width == 0 ||
                    (c.preprocess.width == width_ && c.preprocess.height == height_),
                "BPU input size differs from profile");
        require(std::size_t(width_) * height_ * 3 / 2 <=
                    std::size_t(std::max(0, p.alignedByteSize)),
                "BPU input storage too small");
        allocate(s_->input);
        // SDK contract: tight NV12 is allowed; SDK applies alignment padding itself.
        p.alignedShape = p.validShape;
        check(hbDNNGetOutputCount(&count, s_->model), "hbDNNGetOutputCount");
        require(count > 0 && count <= 64, "invalid BPU output count");
        metadata_.width = width_;
        metadata_.height = height_;
        metadata_.native_nv12 = true;
        s_->outputs.resize(count);
        output_shapes_.reserve(count);
        converted_.resize(count);
        native_views_.resize(count);
        std::size_t total = 0;
        for (int i = 0; i < count; ++i) {
            auto &t = s_->outputs[i];
            check(hbDNNGetOutputTensorProperties(&t.properties, s_->model, i),
                  "hbDNNGetOutputTensorProperties");
            require(t.properties.alignedByteSize > 0, "invalid BPU output bytes");
            total += std::size_t(t.properties.alignedByteSize);
            require(total <= 512U * 1024U * 1024U, "total BPU outputs exceed 512 MiB");
            allocate(t);
            const auto shape = validateOutput(t).shape();
            metadata_.outputs.push_back(shape);
            output_shapes_.push_back(shape);
            native_views_[i] = nativeView(t, shape);
            converted_[i].channels = shape.channels;
            converted_[i].height = shape.height;
            converted_[i].width = shape.width;
        }
    }
    ~BpuBackend() override {
        if (s_ &&
            (s_->phase == BackendState::Quarantined || (s_->task && s_->releaseTask() != 0))) {
            s_->cleanupError("BPU task release failed: session quarantined; restart process to "
                             "reclaim model/tensors");
            s_->logger.reset(); // Quarantine must not retain the application's log file handle.
            (void)s_.release(); // intentional fail-closed retention, including model & tensors
        }
    }
    std::array<int, 2> inputSize() const override { return {width_, height_}; }
    ModelInfo modelInfo() const override { return metadata_; }
    BackendState state() const noexcept override { return s_ ? s_->phase : BackendState::Closed; }
    void close() override {
        if (!s_)
            return;
        require(s_->phase != BackendState::Quarantined,
                "BPU session quarantined; cannot reclaim resources before process exit");
        require(s_->phase == BackendState::Ready, "BPU session is running");
        native_views_.clear();
        s_.reset();
    }
    bool acceptsNv12() const noexcept override { return true; }
    std::vector<Tensor> run(const std::vector<std::uint8_t> &rgb) override {
        detail::OutputSelection selection;
        selection.all = true;
        return execute(rgb.data(), rgb.size(), false, selection);
    }
    std::vector<Tensor> runNv12(const std::vector<std::uint8_t> &nv12) override {
        detail::OutputSelection selection;
        selection.all = true;
        return execute(nv12.data(), nv12.size(), true, selection);
    }
    const std::vector<Tensor> &runReusable(const std::uint8_t *input,
                                           std::size_t input_bytes, bool nv12,
                                           const detail::OutputSelection &selection) override {
        return execute(input, input_bytes, nv12, selection);
    }
    bool supportsNativeDetection(const detail::OutputSelection &selection) const noexcept override {
        if (!s_ || s_->phase != BackendState::Ready || selection.all ||
            selection.count != 6 || s_->outputs.size() != 6)
            return false;
        std::array<bool, 6> seen{};
        for (std::size_t i = 0; i < selection.count; ++i) {
            const int index = selection.indices[i];
            if (index < 0 || index >= 6 || seen[std::size_t(index)])
                return false;
            seen[std::size_t(index)] = true;
            const auto &view = native_views_[std::size_t(index)];
            if (view.layout != Layout::NHWC || !view.finiteForEveryInteger())
                return false;
        }
        return true;
    }
    const std::vector<detail::NativeTensorView> &runNativeDetection(
        const std::uint8_t *input, std::size_t input_bytes, bool nv12,
        const detail::OutputSelection &selection) override {
        require(supportsNativeDetection(selection), "native detection output is unsupported");
        execute(input, input_bytes, nv12, selection, false);
        return native_views_;
    }
};
} // namespace
std::unique_ptr<Backend> makeBpu(const ModelConfig &c) {
    return std::make_unique<BpuBackend>(c);
}
} // namespace perception
