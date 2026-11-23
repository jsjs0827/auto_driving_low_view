#include "runtime_status.hpp"
#include <iomanip>
#include <sstream>
#include <utility>
#include <system_error>

namespace r28 {
namespace {
std::string quote(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 32) out << "\\u" << std::hex << std::setw(4)
                            << std::setfill('0') << int(c) << std::dec;
        else out << char(c);
    }
    out << '"'; return out.str();
}
}
const char* stableName(std::uint32_t code) noexcept {
    switch (code) {
    case ConfigInvalid: return "CONFIG_INVALID";
    case OutputFailed: return "OUTPUT_FAILED";
    case TransportFailed: return "TRANSPORT_FAILED";
    case RechargeFailed: return "RECHARGE_FAILED";
    case CalibrationFailed: return "CALIBRATION_FAILED";
    case ResourceCleanupFailed: return "RESOURCE_CLEANUP_FAILED";
    default: return perception::errorCodeName(static_cast<perception::ErrorCode>(code));
    }
}
int processExit(std::uint32_t code) noexcept { return code ? 1 : 0; }
std::string statusJson(const Status& status) {
    std::ostringstream out;
    out << "{\"code\":" << status.code << ",\"name\":" << quote(stableName(status.code))
        << ",\"native_code\":" << status.native_code
        << ",\"feature\":" << quote(status.feature) << ",\"task\":" << quote(status.task)
        << ",\"frame_id\":" << status.frame.id << ",\"timestamp_ns\":" << status.frame.timestamp_ns
        << ",\"detail\":" << quote(status.detail) << '}';
    return out.str();
}
Status statusFromException(const std::exception& error, std::uint32_t fallback,
                           const std::string& feature, const std::string& task,
                           perception::FrameInfo frame) {
    Status result;
    if (const auto* failure = dynamic_cast<const Failure*>(&error)) result = failure->status();
    else if (const auto* typed = dynamic_cast<const perception::PerceptionException*>(&error)) {
        result.code = static_cast<std::uint32_t>(typed->info().code);
        result.feature = typed->info().component;
        result.detail = typed->info().detail;
        result.frame = typed->info().frame;
    } else {
        result.code = fallback; result.detail = error.what();
        if(const auto* system=dynamic_cast<const std::system_error*>(&error))
            result.native_code=system->code().value();
    }
    if (result.code == 0) result.code = fallback;
    if (result.feature.empty()) result.feature = feature;
    if (result.task.empty()) result.task = task;
    if (result.frame.id == 0 && result.frame.timestamp_ns == 0) result.frame = frame;
    return result;
}
Failure::Failure(Status status)
    : std::runtime_error(std::to_string(status.code) + " " + stableName(status.code) + ": " + status.detail),
      status_(std::move(status)) {}
}
