#pragma once
#include "motion_types.hpp"
#include <deque>
namespace perception::motion {
// Caller serializes push/integrate. Samples use the same clock domain as the camera.
class MotionBuffer {
public:
    explicit MotionBuffer(ReferenceSettings settings={});
    bool push(MotionSample,std::string* reason=nullptr);
    ReferenceResult integrate(std::uint64_t t0_ns,std::uint64_t t1_ns) const;
    void clear() noexcept { samples_.clear(); }
    std::size_t size() const noexcept { return samples_.size(); }
private:
    ReferenceSettings settings_;
    std::deque<MotionSample> samples_;
};
}
