#pragma once
#include "perception_temporal_sdk.h"
#include "perception_slip_sdk.h"
#include "visibility.hpp"
#include <cstdint>

// Private implementation declarations. Opaque handle layouts stay in scheduling.
namespace perception::sdk_internal {
PSDK_INTERNAL std::uint32_t impl_psdk_full_result_get_algorithms_json(const psdk_full_result*,const char**,psdk_status*);
PSDK_INTERNAL std::uint32_t impl_psdk_full_capabilities(void);
PSDK_INTERNAL std::uint32_t impl_psdk_full_create(const psdk_full_config* config,psdk_full_engine** engine,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_process(psdk_full_engine* e,const psdk_full_frame* frame,psdk_full_result** result,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_result_get_view(const psdk_full_result* result,psdk_full_result_view* view,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_command(psdk_full_engine* e,std::int32_t command,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_save_result(const psdk_full_result* result,const char* directory,psdk_status* error);
PSDK_INTERNAL void impl_psdk_full_result_release(psdk_full_result* result);
PSDK_INTERNAL std::uint32_t impl_psdk_full_result_get_temporal_view(const psdk_full_result* result,psdk_temporal_view* view,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_set_temporal_callback(psdk_full_engine* e,psdk_temporal_callback callback,void* userdata,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_reset_history(psdk_full_engine* e,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_destroy(psdk_full_engine** engine,psdk_status* error);
PSDK_INTERNAL std::uint32_t impl_psdk_full_push_motion_samples(psdk_full_engine*,const psdk_motion_sample*,std::uint32_t,psdk_status*);
PSDK_INTERNAL std::uint32_t impl_psdk_full_result_get_slip_view(const psdk_full_result*,psdk_slip_view*,psdk_status*);
}
