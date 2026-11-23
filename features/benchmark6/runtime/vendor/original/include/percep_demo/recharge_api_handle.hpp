#ifndef AI_NODE_PERCEP_DEMO_RECHARGE_API_HANDLE_HPP_
#define AI_NODE_PERCEP_DEMO_RECHARGE_API_HANDLE_HPP_

#include <memory>

#include "percep_demo/api_recharge_mark.h"

namespace ai_node_percep_demo {

using RechargeHostConfig = Aiper_Recharge::RechargeHostConfig;

struct RechargeApiDeleter {
    void operator()(Aiper_Recharge::RechargeMarkApi* api) const noexcept;
};

using RechargeApiPtr =
    std::unique_ptr<Aiper_Recharge::RechargeMarkApi, RechargeApiDeleter>;

}  // namespace ai_node_percep_demo

#endif  // AI_NODE_PERCEP_DEMO_RECHARGE_API_HANDLE_HPP_
