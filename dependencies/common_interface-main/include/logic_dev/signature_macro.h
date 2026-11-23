/**
 * @file signature_macro.h
 * @author Harvey (Harvey.Wang@dayepower.com)
 * @brief signature macro definitions
 * @version 0.2
 * @date 2026-06-17
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved
 *
 */

#pragma once

#include <string>
#include <cstdint>

struct TwoUInt8 {
  uint8_t one {0};
  uint8_t two {0};
};

constexpr const char* AUTOMATION_TASK_EVENT = "automation_task_event";

using VoidSignature = void();
using SignatureInt  = void(int);
using SignatureStr = std::string(void);
using Signature1Str = void(std::string);
using Signature1TwoUInt8 = void(TwoUInt8);
