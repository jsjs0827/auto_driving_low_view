#pragma once
#include "motion_types.hpp"
namespace perception::motion {
// Copy before update; commit the copy only after the full frame DAG succeeds.
class SlipDetector {
public:
    explicit SlipDetector(SlipSettings settings={});
    SlipResult update(const VisualResult&,const ReferenceResult&);
    void reset() noexcept;
private:
    SlipSettings settings_;
    bool slipping_=false;
    std::uint64_t last_end_ns_=0;
    double enter_ms_=0,exit_ms_=0;
};
}
