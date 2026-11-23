#include "backend_lifecycle.hpp"
#include "recharge_mark_logger.h"
extern "C" void r35_recharge_log_init() { recharge_log::Init(); }
extern "C" void r35_recharge_log_shutdown() { recharge_log::Shutdown(); }
