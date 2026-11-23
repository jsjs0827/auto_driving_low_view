#pragma once
#include "runtime_status.hpp"
#include "perception/product_status.h"
#include <cstddef>
namespace r28::product { void setLastStatus(const r28::Status&); }
// Returns the full stable business code. required includes the NUL terminator.
// A too-small/null buffer is untouched; callers can first query its size.
