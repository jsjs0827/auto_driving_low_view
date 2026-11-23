#include "product/status_api.hpp"
#include <cstring>
#include <mutex>
namespace {
std::mutex status_mutex;
r28::Status last_status;
}
namespace r28::product {
void setLastStatus(const r28::Status& status) {
    std::lock_guard<std::mutex> lock(status_mutex); last_status=status;
}
}
extern "C" std::uint32_t perception_product_last_status(char* json,std::size_t capacity,std::size_t* required) {
    try {
        std::lock_guard<std::mutex> lock(status_mutex);
        const auto value=r28::statusJson(last_status);
        if (required) *required=value.size()+1;
        if (json&&capacity>=value.size()+1) std::memcpy(json,value.c_str(),value.size()+1);
        return last_status.code;
    } catch(...) {
        if(required) *required=0;
        return 2002;
    }
}
