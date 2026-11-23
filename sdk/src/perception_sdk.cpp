#include "inference_session_api.hpp"

// Stable public C ABI; scheduling owns configuration, state and execution.
extern "C" {
uint32_t psdk_abi_version(void) { return perception::sdk_internal::impl_psdk_abi_version(); }
uint32_t psdk_build_capabilities(void) { return perception::sdk_internal::impl_psdk_build_capabilities(); }
uint32_t psdk_create(const psdk_config* config,psdk_engine** engine,psdk_status* status) { return perception::sdk_internal::impl_psdk_create(config, engine, status); }
uint32_t psdk_process(psdk_engine* engine,const psdk_frame* frame,psdk_result** result,psdk_status* status) { return perception::sdk_internal::impl_psdk_process(engine, frame, result, status); }
uint32_t psdk_result_get_view(const psdk_result* result,psdk_result_view* view,psdk_status* status) { return perception::sdk_internal::impl_psdk_result_get_view(result, view, status); }
uint32_t psdk_save_png(const psdk_result* result,const psdk_frame* frame,const char* path,psdk_status* status) { return perception::sdk_internal::impl_psdk_save_png(result, frame, path, status); }
void psdk_result_release(psdk_result* result) { perception::sdk_internal::impl_psdk_result_release(result); }
uint32_t psdk_destroy(psdk_engine** engine,psdk_status* status) { return perception::sdk_internal::impl_psdk_destroy(engine, status); }
}
