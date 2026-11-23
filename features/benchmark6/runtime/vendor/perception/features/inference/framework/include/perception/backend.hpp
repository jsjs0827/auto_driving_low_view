#pragma once
#include "perception/config.hpp"
#include <memory>
namespace perception {
// Implementations are serialized by ModelSession. The owner must join users before destruction.
class Backend {
  public:
    virtual ~Backend() = default;
    virtual std::array<int, 2> inputSize() const = 0;
    virtual bool acceptsNv12() const noexcept { return false; }
    // Default supports old injected test backends: bind output metadata on first successful run.
    virtual ModelInfo modelInfo() const;
    virtual std::vector<Tensor> run(const std::vector<std::uint8_t> &rgb) = 0;
    virtual std::vector<Tensor> runNv12(const std::vector<std::uint8_t> &);
    virtual BackendState state() const noexcept { return BackendState::Ready; }
    virtual void close() {} // Explicit close may report failure; destructors never throw.
};
std::unique_ptr<Backend> makeBackend(const ModelConfig &);
} // namespace perception
