#include "full_pipeline_api.hpp"

// Stable public C ABI; scheduling owns configuration, state and execution.
extern "C" {
std::uint32_t psdk_full_result_get_algorithms_json(const psdk_full_result* result,const char** json,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_result_get_algorithms_json(result,json,error); }
std::uint32_t psdk_full_capabilities(void) { return perception::sdk_internal::impl_psdk_full_capabilities(); }
std::uint32_t psdk_full_create(const psdk_full_config* config,psdk_full_engine** engine,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_create(config, engine, error); }
std::uint32_t psdk_full_process(psdk_full_engine* e,const psdk_full_frame* frame,psdk_full_result** result,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_process(e, frame, result, error); }
std::uint32_t psdk_full_result_get_view(const psdk_full_result* result,psdk_full_result_view* view,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_result_get_view(result, view, error); }
std::uint32_t psdk_full_command(psdk_full_engine* e,std::int32_t command,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_command(e, command, error); }
std::uint32_t psdk_full_save_result(const psdk_full_result* result,const char* directory,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_save_result(result, directory, error); }
void psdk_full_result_release(psdk_full_result* result) { perception::sdk_internal::impl_psdk_full_result_release(result); }
std::uint32_t psdk_full_result_get_temporal_view(const psdk_full_result* result,psdk_temporal_view* view,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_result_get_temporal_view(result, view, error); }
std::uint32_t psdk_full_set_temporal_callback(psdk_full_engine* e,psdk_temporal_callback callback,void* userdata,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_set_temporal_callback(e, callback, userdata, error); }
std::uint32_t psdk_full_reset_history(psdk_full_engine* e,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_reset_history(e, error); }
std::uint32_t psdk_full_destroy(psdk_full_engine** engine,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_destroy(engine, error); }
std::uint32_t psdk_full_push_motion_samples(psdk_full_engine* engine,const psdk_motion_sample* samples,std::uint32_t count,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_push_motion_samples(engine,samples,count,error); }
std::uint32_t psdk_full_result_get_slip_view(const psdk_full_result* result,psdk_slip_view* view,psdk_status* error) { return perception::sdk_internal::impl_psdk_full_result_get_slip_view(result,view,error); }
}
