#pragma once
// Test-only injection. Never installed/exported or enabled in the production DLL/SO.
#include "perception_sdk.h"
#include "native_stages.hpp"
#include <memory>
#include <string>
#include <utility>
namespace psdk_test {
std::pair<std::string,std::string> originalModelPaths(const char*);
uint32_t create(const psdk_config*,std::shared_ptr<r25::Stages>,psdk_engine**,psdk_status*) noexcept;
}
