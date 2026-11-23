#include "perception/error.hpp"
#include <algorithm>

namespace perception {
namespace {
struct Entry {
    ErrorCode code;
    const char *name;
    const char *message;
};
constexpr Entry entries[] = {
    {ErrorCode::None, "NONE", u8"无错误"},
    {ErrorCode::CameraAcquisitionFailed, "CAMERA_ACQUISITION_FAILED", u8"相机数据获取失败"},
    {ErrorCode::CameraDataInvalid, "CAMERA_DATA_INVALID", u8"相机数据异常"},
    {ErrorCode::LensObstructed, "LENS_OBSTRUCTED", u8"镜头遮挡"},
    {ErrorCode::PerceptionInitFailed, "PERCEPTION_INIT_FAILED", u8"感知模块初始化失败"},
    {ErrorCode::AiInitFailed, "AI_INIT_FAILED", u8"AI推理初始化失败"},
    {ErrorCode::MapDataInvalid, "MAP_DATA_INVALID", u8"感知地图数据异常"},
    {ErrorCode::PerceptionStateInvalid, "PERCEPTION_STATE_INVALID", u8"感知状态异常"},
    {ErrorCode::ModelLoadFailed, "MODEL_LOAD_FAILED", u8"AI模型加载失败"},
    {ErrorCode::InferenceFailed, "INFERENCE_FAILED", u8"AI推理失败"},
    {ErrorCode::InferenceResultInvalid, "INFERENCE_RESULT_INVALID", u8"AI推理结果异常"},
    {ErrorCode::MapProcessingTooSlow, "MAP_PROCESSING_TOO_SLOW", u8"感知地图数据耗时异常"},
};
const Entry *find(ErrorCode code) noexcept {
    for (const auto &e : entries)
        if (e.code == code)
            return &e;
    return nullptr;
}
std::string description(const ErrorInfo &error) {
    return std::to_string(static_cast<std::uint32_t>(error.code)) + " " +
           errorCodeName(error.code) + ": " + error.detail;
}
} // namespace
const char *errorCodeName(ErrorCode code) noexcept {
    auto e = find(code);
    return e ? e->name : "UNKNOWN";
}
const char *errorCodeMessage(ErrorCode code) noexcept {
    auto e = find(code);
    return e ? e->message : u8"未知错误码";
}
bool isKnownErrorCode(ErrorCode code) noexcept {
    return find(code) != nullptr;
}
bool ErrorInfo::contains(ErrorCode value) const noexcept {
    return code == value ||
           std::find(context_codes.begin(), context_codes.end(), value) != context_codes.end();
}
PerceptionException::PerceptionException(ErrorInfo info)
    : std::runtime_error(description(info)), info_(std::move(info)) {}
PerceptionException::PerceptionException(ErrorCode code, std::string detail, std::string component)
    : PerceptionException(ErrorInfo{code, {}, std::move(component), std::move(detail), {}, 0}) {}
ErrorInfo errorFromException(ErrorCode fallback, const std::exception &e, const char *component) {
    if (auto typed = dynamic_cast<const PerceptionException *>(&e))
        return typed->info();
    return {fallback, {}, component, e.what(), {}, 0};
}
ErrorInfo withErrorContext(ErrorInfo info, ErrorCode context) {
    if (!info.contains(context))
        info.context_codes.push_back(context);
    return info;
}
void validateCameraImage(const ImageView &im) {
    auto require = [](bool ok, const char *why) {
        if (!ok)
            throw PerceptionException(ErrorCode::CameraDataInvalid, why, "camera.input");
    };
    require(im.data && im.width > 0 && im.height > 0 && im.width <= 8192 && im.height <= 8192,
            "null image or invalid dimensions (maximum 8192)");
    require(im.format == PixelFormat::RGB || im.format == PixelFormat::BGR ||
                im.format == PixelFormat::NV12 || im.format == PixelFormat::I420,
            "unknown pixel format");
    const bool yuv = im.format == PixelFormat::NV12 || im.format == PixelFormat::I420;
    const auto row = std::size_t(im.width) * (yuv ? 1 : 3);
    require(im.stride >= row && im.stride <= 16U * 1024U * 1024U, "invalid image stride");
    require(!yuv || (im.width % 2 == 0 && im.height % 2 == 0 && im.stride % 2 == 0),
            "YUV420 requires even width/height/stride");
    const auto base = im.stride * im.height;
    const auto needed = yuv ? base + base / 2 : im.stride * (im.height - 1) + row;
    require(im.bytes >= needed, "short image buffer");
}
} // namespace perception
