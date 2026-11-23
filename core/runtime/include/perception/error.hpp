#pragma once
#include "perception/types.hpp"
#include <stdexcept>

namespace perception {
// Stable wire values. Never reorder/renumber; 0 means no fault.
// Removed module values 1006, 1008, 1013, 1014 and 1016 remain reserved.
enum class ErrorCode : std::uint32_t {
    None = 0,
    CameraAcquisitionFailed = 1001,
    CameraDataInvalid = 1002,
    LensObstructed = 1003,
    PerceptionInitFailed = 1004,
    AiInitFailed = 1005,
    MapDataInvalid = 1007,
    PerceptionStateInvalid = 1009,
    ModelLoadFailed = 1010,
    InferenceFailed = 1011,
    InferenceResultInvalid = 1012,
    MapProcessingTooSlow = 1015
};
const char *errorCodeName(ErrorCode) noexcept;
const char *errorCodeMessage(ErrorCode) noexcept; // UTF-8 Chinese description.
bool isKnownErrorCode(ErrorCode) noexcept;
struct ErrorInfo {
    ErrorCode code = ErrorCode::None;
    std::vector<ErrorCode>
        context_codes; // Inner-to-outer initialization context; root code retained.
    std::string component, detail;
    FrameInfo frame;
    double elapsed_ms = 0;
    bool ok() const noexcept { return code == ErrorCode::None; }
    bool contains(ErrorCode) const noexcept;
};
class PerceptionException : public std::runtime_error {
  public:
    explicit PerceptionException(ErrorInfo);
    PerceptionException(ErrorCode, std::string detail, std::string component = {});
    const ErrorInfo &info() const noexcept { return info_; }

  private:
    ErrorInfo info_;
};
ErrorInfo errorFromException(ErrorCode fallback, const std::exception &, const char *component);
ErrorInfo withErrorContext(ErrorInfo, ErrorCode);
// Structural buffer validation only; never guesses camera acquisition/occlusion from pixels.
void validateCameraImage(const ImageView &);
} // namespace perception
