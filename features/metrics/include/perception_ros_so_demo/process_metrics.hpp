#pragma once
#include <chrono>

namespace ros_so_demo {
using Steady=std::chrono::steady_clock;
double milliseconds(Steady::time_point begin);
double cpuMs();
long rssKib();
} // namespace ros_so_demo
