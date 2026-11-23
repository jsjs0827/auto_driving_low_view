#pragma once
#include "perception_sdk.h"
#include "visibility.hpp"
#include <cstdint>

// Private implementation declarations. Opaque handle layouts stay in scheduling.
namespace perception::sdk_internal {
PSDK_INTERNAL uint32_t impl_psdk_abi_version(void);
PSDK_INTERNAL uint32_t impl_psdk_build_capabilities(void);
PSDK_INTERNAL uint32_t impl_psdk_create(const psdk_config* config,psdk_engine** engine,psdk_status* status);
PSDK_INTERNAL uint32_t impl_psdk_process(psdk_engine* engine,const psdk_frame* frame,psdk_result** result,psdk_status* status);
PSDK_INTERNAL uint32_t impl_psdk_result_get_view(const psdk_result* result,psdk_result_view* view,psdk_status* status);
PSDK_INTERNAL uint32_t impl_psdk_save_png(const psdk_result* result,const psdk_frame* frame,const char* path,psdk_status* status);
PSDK_INTERNAL void impl_psdk_result_release(psdk_result* result);
PSDK_INTERNAL uint32_t impl_psdk_destroy(psdk_engine** engine,psdk_status* status);
}
