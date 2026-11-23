#pragma once
#include "perception_ros_so_demo/sdk_caller.hpp"

// Source-neutral names for the existing shared execution boundary. These are
// aliases, not another implementation: offline/ROS and V0 host call precisely
// the same Caller -> psdk_full_process -> product DAG/feature implementation.
namespace perception::entry {
using Result = ros_so_demo::Result;
using Session = ros_so_demo::Caller;
inline psdk_status newStatus() { return ros_so_demo::newStatus(); }
}
