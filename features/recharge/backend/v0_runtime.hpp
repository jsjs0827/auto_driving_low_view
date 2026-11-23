#pragma once
#include "v0-sdk/include/percep_demo/api_recharge_mark.h"
#include <cstddef>

// Private shim ABI only. This is not part of the public perception C ABI.
// All API table offsets/types come from the selected, unchanged V0 header.
extern "C" {
#if defined(__GNUC__)
#define R36_V0_EXPORT __attribute__((visibility("default")))
#else
#define R36_V0_EXPORT
#endif
R36_V0_EXPORT int r36_recharge_v0_open(Aiper_Recharge::RechargeMarkApi** api, void** lease,
                         char* error, std::size_t error_size);
R36_V0_EXPORT void r36_recharge_v0_close(void* lease);
R36_V0_EXPORT const char* r36_recharge_v0_identity();
#undef R36_V0_EXPORT
}
