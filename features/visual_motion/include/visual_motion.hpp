#pragma once
#include "motion_types.hpp"
#include "perception_temporal.hpp"
namespace perception::motion {
class VisualMotion {
public:
    explicit VisualMotion(VisualSettings settings={});
    VisualResult estimate(const r36::temporal::Window&) const;
private:
    VisualSettings settings_;
};
}
