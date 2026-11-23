#pragma once
#include "algorithm_runtime.hpp"
namespace perception::scene_cls {
std::unique_ptr<runtime::Feature> create(const algorithms::Invocation&);
}
