#pragma once

#include "perception/types.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace perception::inference {

enum class Stage : std::size_t { Preprocess = 0, Backend = 1, Postprocess = 2 };
struct StageTiming { double elapsed_ms = 0; bool succeeded = false; };

// Common execution contract, independent of tensors or image count. Algorithm
// adapters keep their existing numeric implementations and model ownership.
// A session is frame-local: tasks execute its three stages in this order.
class StagedInvocation {
public:
    void begin(std::string algorithm, FrameInfo frame,
               const std::vector<FrameInfo>& inputs, std::size_t required_inputs) {
        if (algorithm.empty() || !required_inputs || inputs.size() != required_inputs)
            throw std::invalid_argument("Inference input count does not satisfy its algorithm contract");
        for (const auto& input : inputs)
            if (input.id != frame.id || input.timestamp_ns != frame.timestamp_ns)
                throw std::invalid_argument("Inference inputs must belong to the same frame");
        algorithm_ = std::move(algorithm); frame_ = frame; timings_ = {};
        next_ = 0; failed_ = false; begun_ = true;
    }
    template<class Function> void execute(Stage stage, Function&& function) {
        const auto index = static_cast<std::size_t>(stage);
        if (!begun_ || failed_ || index >= timings_.size() || index != next_)
            throw std::logic_error("Inference stages must run preprocess, backend, postprocess exactly once");
        const auto start = std::chrono::steady_clock::now();
        auto& timing = timings_[index];
        const auto finish = [&] {
            timing.elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        };
        try { std::forward<Function>(function)(); finish(); timing.succeeded = true; ++next_; }
        catch (...) { finish(); failed_ = true; throw; }
    }
    bool complete() const noexcept { return begun_ && !failed_ && next_ == 3; }
    const std::array<StageTiming, 3>& timings() const noexcept { return timings_; }
    const std::string& algorithm() const noexcept { return algorithm_; }
    FrameInfo frame() const noexcept { return frame_; }
private:
    std::string algorithm_;
    FrameInfo frame_{};
    std::array<StageTiming, 3> timings_{};
    std::size_t next_ = 0;
    bool begun_ = false, failed_ = false;
};

} // namespace perception::inference
