#pragma once
// Host-only MSVC test of the real Linux camera adapter body and vendor ABI.
#if defined(_MSC_VER) && !defined(__clang__)
#define __attribute__(value)
#endif
