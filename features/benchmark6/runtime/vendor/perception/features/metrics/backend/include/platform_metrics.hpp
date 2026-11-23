#pragma once
#include <map>
#include <string>

namespace r29 {
// Process CPU time includes every inference/scheduler thread. CPU percentage
// uses one logical CPU = 100%, so parallel work may legitimately exceed 100%.
double processCpuMs();
// Values are KiB. Linux /proc keys are retained for existing CSV consumers;
// unavailable fields (including CMA on Windows) are absent, never invented.
std::map<std::string,long> processResources();
}
