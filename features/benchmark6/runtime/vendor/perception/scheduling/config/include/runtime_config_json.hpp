#pragma once

#include "perception/task_graph.hpp"

#include <string>

namespace perception::runtime {

// Load the r12-v3 strict JSON format. Task definitions live in the top-level
// `tasks` object while `dag` composes references with nested sequence/parallel
// nodes. This is intentionally independent of Python/MMEngine on the target.
TaskGraphConfig loadNestedJsonTaskGraphConfig(const std::string &path);

} // namespace perception::runtime
