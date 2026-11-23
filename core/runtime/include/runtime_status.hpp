#pragma once
#include "perception/error.hpp"
#include <cstdint>
#include <stdexcept>
#include <string>

namespace r28 {
// Product extensions. The framework's existing 1001..1015 wire values stay intact.
constexpr std::uint32_t ConfigInvalid = 2001;
constexpr std::uint32_t OutputFailed = 2002;
constexpr std::uint32_t TransportFailed = 2003;
constexpr std::uint32_t RechargeFailed = 2004;
constexpr std::uint32_t CalibrationFailed = 2005;
constexpr std::uint32_t ResourceCleanupFailed = 2006;

struct Status {
    std::uint32_t code = 0;
    int native_code = 0; // SDK/library return value when one was actually returned.
    std::string feature, task, detail;
    perception::FrameInfo frame;
    bool ok() const noexcept { return code == 0; }
};
const char* stableName(std::uint32_t) noexcept;
// POSIX process exits are only eight bits. The full business code is returned
// by tryRun/lastStatus and serialized into JSON, never truncated into exit(1001).
int processExit(std::uint32_t) noexcept;
std::string statusJson(const Status&);
Status statusFromException(const std::exception&, std::uint32_t fallback,
                           const std::string& feature = {},
                           const std::string& task = {},
                           perception::FrameInfo frame = {});
class Failure : public std::runtime_error {
    Status status_;
public:
    explicit Failure(Status);
    const Status& status() const noexcept { return status_; }
};
}
