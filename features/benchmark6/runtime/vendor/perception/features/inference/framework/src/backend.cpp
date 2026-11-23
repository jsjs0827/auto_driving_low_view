#include "perception/backend.hpp"
#include "detail/diagnostic_boundary.hpp"
#include <filesystem>
#include <fstream>
#include <stdexcept>
namespace perception {
#ifdef PERCEPTION_WITH_ORT
std::unique_ptr<Backend> makeOrt(const ModelConfig &);
#endif
#ifdef PERCEPTION_WITH_BPU
std::unique_ptr<Backend> makeBpu(const ModelConfig &);
#endif
const char *stateName(BackendState s) noexcept {
    switch (s) {
    case BackendState::Ready:
        return "Ready";
    case BackendState::Running:
        return "Running";
    case BackendState::Quarantined:
        return "Quarantined";
    case BackendState::Closed:
        return "Closed";
    }
    return "Unknown";
}
ModelInfo Backend::modelInfo() const {
    const auto size = inputSize();
    return {size[0], size[1], {}, {}, acceptsNv12()};
}
std::vector<Tensor> Backend::runNv12(const std::vector<std::uint8_t> &) {
    throw std::runtime_error("backend does not accept native NV12");
}
std::unique_ptr<Backend> makeBackend(const ModelConfig &c) {
    detail::boundary(ErrorCode::AiInitFailed, "backend.config", [&] { c.validate(); });
    if (c.source.path.empty())
        throw PerceptionException(ErrorCode::ModelLoadFailed, "model path is empty", "model.load");
    // Verify readability without consuming the model. SDK-specific load errors are
    // tagged at the actual ORT/hbDNN load call below, not guessed from error text.
    std::ifstream model(std::filesystem::u8path(c.source.path), std::ios::binary);
    if (!model || model.peek() == std::char_traits<char>::eof())
        throw PerceptionException(ErrorCode::ModelLoadFailed,
                                  "model is missing, unreadable or empty: " + c.source.path,
                                  "model.load");
    if (c.backend.device == Device::BPU) {
#ifdef PERCEPTION_WITH_BPU
        return detail::boundary(ErrorCode::AiInitFailed, "bpu.init", [&] { return makeBpu(c); });
#else
        throw PerceptionException(ErrorCode::AiInitFailed,
                                  "BPU backend not built: enable PERCEPTION_WITH_BPU", "bpu.init");
#endif
    }
#ifdef PERCEPTION_WITH_ORT
    return detail::boundary(ErrorCode::AiInitFailed, "ort.init", [&] { return makeOrt(c); });
#else
    throw PerceptionException(ErrorCode::AiInitFailed,
                              "CPU/CUDA backend not built: enable PERCEPTION_WITH_ORT", "ort.init");
#endif
}
} // namespace perception
