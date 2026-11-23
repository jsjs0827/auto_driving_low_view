#pragma once
#include "algorithm_runtime.hpp"
namespace perception::raw_lux_feature {
std::unique_ptr<runtime::Feature> create(const algorithms::Invocation& invocation);
}
