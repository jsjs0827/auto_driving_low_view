#pragma once
#include "runtime_status.hpp"
#include <cstdint>
#include <exception>

namespace r30::product {
struct CommandPlan {
    bool map = false, station = false;
    std::uint8_t camera = 0;
    int app = 0;
};
// Resolve and validate the original wire command before touching mode or camera.
inline CommandPlan commandPlan(int command, bool recharge_ready) {
    CommandPlan plan;
    switch(command) {
    case 1: plan = {true, false, 1, 2001}; break;
    case 2: plan = {false, true, 2, 2002}; break;
    case 3: plan = {true, true, 3, 2002}; break;
    case 21: plan = {false, false, 4, 2021}; break;
    default: {
        r28::Status status; status.code=2001;
        status.detail="unknown perception command: "+std::to_string(command);
        throw r28::Failure(status);
    }
    }
    if(plan.station&&!recharge_ready) {
        r28::Status status; status.code=2004;
        status.detail="recharge command requires enabled, initialized SDK and publication";
        throw r28::Failure(status);
    }
    return plan;
}
// Cleanup is best effort: a cleanup exception must not replace the initiating
// transport/SDK failure and its native return value.
template<class Apply,class Rollback>
void commandTransaction(Apply&& apply,Rollback&& rollback) {
    try {apply();}
    catch(...) {
        const auto failure=std::current_exception();
        try {rollback();} catch(...) {}
        std::rethrow_exception(failure);
    }
}
}
