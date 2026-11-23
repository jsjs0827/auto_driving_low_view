#pragma once
// These implementation entries are not part of the installed C ABI.
#if defined(__GNUC__) || defined(__clang__)
#define PSDK_INTERNAL __attribute__((visibility("hidden")))
#else
#define PSDK_INTERNAL
#endif
