#include "camera_source.hpp"
#include "camera_close_policy.hpp"

#include "perception/error.hpp"
#include "perception/preprocess.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <array>
#include <atomic>
#include <mutex>

#if defined(PERCEPTION_MO_VENDOR_TYPES_AVAILABLE)
#include <MoTypes.h>
#endif
#if defined(__linux__) && defined(PERCEPTION_MO_VENDOR_TYPES_AVAILABLE)

#include <MoLocalSdkInterface.h>
#include <dlfcn.h>
#endif

namespace demo_camera {
namespace {

using perception::ErrorCode;
using perception::PerceptionException;
using perception::PixelFormat;
using Clock = std::chrono::steady_clock;

constexpr int kOk = 0;
constexpr std::size_t kMaxImageBytes = 256U * 1024U * 1024U;

std::int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               Clock::now().time_since_epoch())
        .count();
}

[[noreturn]] void cameraError(ErrorCode code, const std::string &detail,
                              const char *component) {
    throw PerceptionException(code, detail, component);
}

PixelFormat pixelFormat(const std::string &format) {
    if (format == "i420")
        return PixelFormat::I420;
    if (format == "nv12")
        return PixelFormat::NV12;
    throw std::invalid_argument("camera format must be i420 or nv12");
}

#if defined(__linux__) && defined(PERCEPTION_MO_VENDOR_TYPES_AVAILABLE)

// The vendor callback has no user pointer. Storage lives until process exit,
// including when a failed close leaves vendor threads running. One source per
// process is supported for profiling; the callback never retains a Source.
struct StereoTraceStorage {
    std::mutex mutex;
    std::array<StereoEvent,65536> events{};
    std::size_t size=0;
    std::uint64_t starts=0,ends=0,dropped=0;
    std::int64_t start_ns=0,busy_ns=0;
    std::atomic<bool> enabled{false};
} stereo_storage;
void stereoCallback(int status) {
    if(!stereo_storage.enabled.load(std::memory_order_acquire))return;
    const auto ns=nowNs();
    std::lock_guard<std::mutex> lock(stereo_storage.mutex);
    if(stereo_storage.size<stereo_storage.events.size())stereo_storage.events[stereo_storage.size++]={ns,status};
    else ++stereo_storage.dropped;
    if(status==1){++stereo_storage.starts;stereo_storage.start_ns=ns;}
    else if(status==0){++stereo_storage.ends;if(stereo_storage.start_ns){stereo_storage.busy_ns+=ns-stereo_storage.start_ns;stereo_storage.start_ns=0;}}
}

// SDK 1.4.4.0's exported setter returns -2 for NULL despite the header's
// cancellation comment. Replace the recorder with a valid process-lifetime
// no-op before close. It touches neither Source nor trace storage and remains
// safe if moCloseCamera fails and the SDK has to stay loaded.
void stereoNoopCallback(int) noexcept {}

// Use the target SDK's real MoFrame definition.  A handwritten mirror can
// silently corrupt memory when the camera runtime and build headers differ.
std::shared_ptr<const example::Image> copyImage(const MoFrame &frame,
                                                const Options &options) {
    const int width = static_cast<int>(frame.ResolutionWidth);
    const int height = static_cast<int>(frame.ResolutionHeight);
    const std::size_t bytes = frame.BytesUsed;
    if (!frame.Data)
        cameraError(ErrorCode::CameraDataInvalid, "camera returned null left image",
                    "camera.mo.frame");
    if (width <= 0 || height <= 0 || width > 8192 || height > 8192 ||
        (width & 1) || (height & 1))
        cameraError(ErrorCode::CameraDataInvalid,
                    "camera returned invalid/odd image dimensions",
                    "camera.mo.frame");
    if (bytes == 0 || bytes > kMaxImageBytes)
        cameraError(ErrorCode::CameraDataInvalid,
                    "camera returned invalid image byte count", "camera.mo.frame");

    // ImageView's raw-YUV contract uses one common Y/chroma row stride and
    // total=stride*height*3/2.  Reject an unknown vendor layout rather than
    // guessing and silently changing model input pixels.
    const std::size_t denominator = 3U * static_cast<std::size_t>(height);
    if (bytes > std::numeric_limits<std::size_t>::max() / 2U ||
        (bytes * 2U) % denominator != 0U)
        cameraError(ErrorCode::CameraDataInvalid,
                    "camera YUV byte count cannot describe a common row stride",
                    "camera.mo.frame");
    const std::size_t stride = (bytes * 2U) / denominator;
    if (stride < static_cast<std::size_t>(width) || stride > 65536U || (stride & 1U))
        cameraError(ErrorCode::CameraDataInvalid, "camera returned invalid YUV stride",
                    "camera.mo.frame");

    auto image = std::make_shared<example::Image>();
    image->width = width;
    image->height = height;
    image->stride = stride;
    image->format = pixelFormat(options.format);
    image->decoded_format = "mo-" + options.stream + "-" + options.format;
    image->source_channels = 3;
    image->source_bit_depth = 8;
    const auto begin = Clock::now();
    const auto *first = static_cast<const std::uint8_t *>(frame.Data);
    image->bytes.assign(first, first + bytes);
    image->decode_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    perception::validateCameraImage(image->view());
    return image;
}

// RGBD acquisition owns all three vendor frames until release. Capture only
// their metadata here: this inspection never reads disparity pixels. The
// separate copy_disparity opt-in below owns pixels before SDK release.
// Matching metadata does not establish exposure/physical sensor synchronization
// or show that this frame's stereo inference overlaps its AI inference.
void inspectRgbdTriplet(const MoFrame &left, const MoFrame &right,
                        const MoFrame &disparity, const Options &options,
                        Frame &result) {
    result.sdk_rgbd_triplet_available=true;
    result.sdk_right_frame_id=right.FrameId;
    result.sdk_right_timestamp_ns=right.TimeStamp;
    result.sdk_right_width=right.ResolutionWidth;
    result.sdk_right_height=right.ResolutionHeight;
    result.sdk_right_bytes_used=right.BytesUsed;
    result.sdk_disparity_frame_id=disparity.FrameId;
    result.sdk_disparity_timestamp_ns=disparity.TimeStamp;
    result.sdk_disparity_width=disparity.ResolutionWidth;
    result.sdk_disparity_height=disparity.ResolutionHeight;
    result.sdk_disparity_bytes_used=disparity.BytesUsed;
    // The inspected 1.4.4.0 moGetOneRGBDFrame has a successful-return branch
    // with all disparity ID/time/geometry/byte metadata zeroed. Treat that as
    // unreported, not as a contradictory ID=0. A genuinely reported frame ID
    // of zero is still valid and is compared exactly, with no /3 inference.
    const bool right_reported=right.TimeStamp || right.ResolutionWidth ||
        right.ResolutionHeight || right.BytesUsed;
    const bool disparity_reported=disparity.TimeStamp || disparity.ResolutionWidth ||
        disparity.ResolutionHeight || disparity.BytesUsed;
    const auto known_different=[](std::uint64_t a,std::uint64_t b){return a && b && a!=b;};
    const bool color_id_mismatch=right_reported && left.FrameId!=right.FrameId;
    const bool color_timestamp_mismatch=known_different(left.TimeStamp,right.TimeStamp);
    const bool color_geometry_mismatch=known_different(left.ResolutionWidth,right.ResolutionWidth) ||
        known_different(left.ResolutionHeight,right.ResolutionHeight);
    const bool color_bytes_mismatch=known_different(left.BytesUsed,right.BytesUsed);
    const bool color_data_present=left.Data && right.Data;
    result.sdk_color_pair_valid=right_reported && !color_id_mismatch &&
        left.TimeStamp && right.TimeStamp && !color_timestamp_mismatch &&
        left.ResolutionWidth && left.ResolutionHeight &&
        right.ResolutionWidth && right.ResolutionHeight && !color_geometry_mismatch &&
        left.BytesUsed && right.BytesUsed && !color_bytes_mismatch && color_data_present;
    const auto color_issue=[&](const char* text){if(!result.sdk_color_pair_issue.empty())result.sdk_color_pair_issue+=';';result.sdk_color_pair_issue+=text;};
    if(!right_reported)color_issue("right_metadata_unreported");
    if(color_id_mismatch)color_issue("frame_id_mismatch");
    if(!left.TimeStamp || !right.TimeStamp)color_issue("timestamp_unreported");
    if(color_timestamp_mismatch)color_issue("timestamp_mismatch");
    if(!left.ResolutionWidth || !left.ResolutionHeight || !right.ResolutionWidth || !right.ResolutionHeight)color_issue("geometry_unreported");
    if(color_geometry_mismatch)color_issue("geometry_mismatch");
    if(!left.BytesUsed || !right.BytesUsed)color_issue("byte_count_unreported");
    if(color_bytes_mismatch)color_issue("byte_count_mismatch");
    if(!color_data_present)color_issue("data_missing");
    const bool ids=(!right_reported || left.FrameId==right.FrameId) &&
        (!disparity_reported || left.FrameId==disparity.FrameId);
    const bool timestamps_present=left.TimeStamp && right.TimeStamp && disparity.TimeStamp;
    const bool timestamp_mismatch=known_different(left.TimeStamp,right.TimeStamp) ||
        known_different(left.TimeStamp,disparity.TimeStamp) ||
        known_different(right.TimeStamp,disparity.TimeStamp);
    const bool geometry_present=left.ResolutionWidth && left.ResolutionHeight &&
        right.ResolutionWidth && right.ResolutionHeight &&
        disparity.ResolutionWidth && disparity.ResolutionHeight;
    const bool geometry_mismatch=known_different(left.ResolutionWidth,right.ResolutionWidth) ||
        known_different(left.ResolutionHeight,right.ResolutionHeight) ||
        known_different(left.ResolutionWidth,disparity.ResolutionWidth) ||
        known_different(left.ResolutionHeight,disparity.ResolutionHeight) ||
        known_different(right.ResolutionWidth,disparity.ResolutionWidth) ||
        known_different(right.ResolutionHeight,disparity.ResolutionHeight);
    const std::uint64_t depth_bytes=std::uint64_t(disparity.ResolutionWidth)*
        disparity.ResolutionHeight*sizeof(std::uint16_t);
    const bool data_present=left.Data && right.Data && disparity.Data;
    const bool bytes_present=left.BytesUsed && right.BytesUsed && disparity.BytesUsed;
    const bool bytes_mismatch=known_different(left.BytesUsed,right.BytesUsed) ||
        (depth_bytes && disparity.BytesUsed && disparity.BytesUsed!=depth_bytes) ||
        depth_bytes>kMaxImageBytes;
    const bool contradiction=!ids || timestamp_mismatch || geometry_mismatch ||
        !color_data_present || (disparity_reported && !disparity.Data) || bytes_mismatch;
    result.sdk_rgbd_triplet_valid=!contradiction && timestamps_present &&
        geometry_present && bytes_present && right_reported && disparity_reported && data_present;
    const auto issue=[&](const char* text){if(!result.sdk_rgbd_triplet_issue.empty())result.sdk_rgbd_triplet_issue+=';';result.sdk_rgbd_triplet_issue+=text;};
    if(!ids)issue("frame_id_mismatch");
    if(!right_reported)issue("right_metadata_unreported");
    if(!disparity_reported)issue("disparity_metadata_unreported");
    if(!timestamps_present)issue("timestamp_unreported");
    if(timestamp_mismatch)issue("timestamp_mismatch");
    if(!geometry_present)issue("geometry_unreported");
    if(geometry_mismatch)issue("geometry_mismatch");
    if(!color_data_present || (disparity_reported && !disparity.Data))issue("data_missing");
    if(!disparity_reported && !disparity.Data)issue("disparity_data_unreported");
    if(!bytes_present)issue("byte_count_unreported");
    if(bytes_mismatch)issue("byte_count_mismatch");
    if(options.audit_rgbd_triplet && contradiction)
        cameraError(ErrorCode::CameraDataInvalid,
            std::string("RGBD_TRIPLET_METADATA_INVALID: ")+result.sdk_rgbd_triplet_issue+
            "; left/right/disp ids="+std::to_string(left.FrameId)+"/"+
            std::to_string(right.FrameId)+"/"+std::to_string(disparity.FrameId)+
            " timestamps="+std::to_string(left.TimeStamp)+"/"+
            std::to_string(right.TimeStamp)+"/"+std::to_string(disparity.TimeStamp),
            "camera.mo.rgbd_triplet");
}

std::shared_ptr<const std::vector<std::uint16_t>> copyRgbdDisparity(
    const MoFrame& left, const MoFrame& right, const MoFrame& disparity,
    const Frame& inspected) {
    constexpr std::size_t pixels=640U*544U;
    if (!inspected.sdk_rgbd_triplet_valid ||
        left.ResolutionWidth!=640 || left.ResolutionHeight!=544 ||
        right.ResolutionWidth!=640 || right.ResolutionHeight!=544 ||
        disparity.ResolutionWidth!=640 || disparity.ResolutionHeight!=544 ||
        left.BytesUsed!=pixels*3/2 || right.BytesUsed!=pixels*3/2 ||
        disparity.BytesUsed!=pixels*sizeof(std::uint16_t) || disparity.BitsPerPixel!=16 ||
        !disparity.Data || !left.TimeStamp || left.TimeStamp!=right.TimeStamp ||
        left.TimeStamp!=disparity.TimeStamp || left.FrameId!=right.FrameId ||
        left.FrameId!=disparity.FrameId)
        cameraError(ErrorCode::CameraDataInvalid,
            "RGBD_DISPARITY_COPY_METADATA_INVALID: require matching IDs/timestamps and tight 640x544 uint16 disparity; "+
                inspected.sdk_rgbd_triplet_issue,"camera.mo.disparity_copy");
    auto owned=std::make_shared<std::vector<std::uint16_t>>(pixels);
    std::memcpy(owned->data(),disparity.Data,pixels*sizeof(std::uint16_t));
    return owned;
}

class MoSource final : public Source {
    // Derive every ABI signature from the vendor interface header.  The
    // functions are still resolved dynamically so libMoGeneralSDK is not a
    // link-time dependency of the AI-only package.
    using SetConfig = decltype(&moSetConfigPath);
    using OpenCamera = decltype(&moOpenCamera);
    using CloseCamera = decltype(&moCloseCamera);
    using GetRgbd = decltype(&moGetOneRGBDFrame);
    using ReleaseRgbd = decltype(&moReleaseRGBDFrame);
    using GetGdc = decltype(&moGetOneGdcFrame);
    using ReleaseGdc = decltype(&moReleaseGdcFrame);

    Options options_;
    void *library_ = nullptr;
    int handle_ = -1;
    SetConfig set_config_ = nullptr;
    OpenCamera open_camera_ = nullptr;
    CloseCamera close_camera_ = nullptr;
    GetRgbd get_rgbd_ = nullptr;
    ReleaseRgbd release_rgbd_ = nullptr;
    GetGdc get_gdc_ = nullptr;
    ReleaseGdc release_gdc_ = nullptr;
    decltype(&moSetDisparityCalculationMode) set_mode_=nullptr;
    decltype(&moGetDisparityCalculationMode) get_mode_=nullptr;
    decltype(&moGetBpuRatio) get_ratio_=nullptr;
    decltype(&moGetFrameRateOrg) get_org_fps_=nullptr;
    decltype(&moGetFrameRateDisp) get_disp_fps_=nullptr;
    decltype(&moGetStereoVisionStatusCallback) set_callback_=nullptr;
    decltype(&moCalculateDisparityByFrameId) calculate_=nullptr;
    std::unique_ptr<std::uint16_t[]> demand_buffer_;
    int demand_width_=0, demand_height_=0;
    static constexpr std::size_t demand_capacity_=4096U*4096U;
    Diagnostics diagnostic_;
    int original_mode_=-1;
    std::mutex api_mutex_;

    template <class Function> Function symbol(const char *name) {
        dlerror();
        void *address = dlsym(library_, name);
        const char *error = dlerror();
        if (error || !address)
            cameraError(ErrorCode::CameraAcquisitionFailed,
                        std::string("missing camera SDK symbol ") + name +
                            (error ? std::string(": ") + error : std::string()),
                        "camera.mo.load");
        Function function = nullptr;
        static_assert(sizeof(function) == sizeof(address),
                      "dlsym function pointer size differs from object pointer");
        std::memcpy(&function, &address, sizeof(function));
        return function;
    }

    template<class Function> Function optionalSymbol(const char* name) {
        dlerror();void* address=dlsym(library_,name);const auto* error=dlerror();
        Function fn=nullptr;if(!error&&address)std::memcpy(&fn,&address,sizeof(fn));return fn;
    }

    void trace(const char* name,int code,double ms=0) noexcept {
        try {if(options_.lifecycle)options_.lifecycle(name,code,ms);}catch(...){}
    }
    template<class F> int traced(const char* name,F call) {
        const auto begin=Clock::now();const int code=call();
        trace(name,code,std::chrono::duration<double,std::milli>(Clock::now()-begin).count());return code;
    }
    bool closeForUnload(const char *context) noexcept {
        if(handle_<0)return true;
        stereo_storage.enabled.store(false,std::memory_order_release);
        if(diagnostic_.callback_available && set_callback_)
            traced("callback_replace_noop",[&]{return set_callback_(handle_,stereoNoopCallback);});
        const bool restore=options_.restore_mode_before_close && options_.disparity_mode && original_mode_>0 && set_mode_;
        const bool safe=closeWithPolicy(restore,
            [&]{return traced("moSetDisparityCalculationMode_restore",[&]{return set_mode_(handle_,original_mode_);});},
            [&]{return close_camera_?traced("moCloseCamera",[&]{return close_camera_(handle_);}):-999;},
            [&](const char* name,int code){if(code!=kOk)std::cerr<<context<<": "<<name<<" failed: "<<code<<'\n';});
        handle_=-1;
        if(!safe) {trace("sdk_retained_after_failed_close",-1);std::cerr<<context<<": keeping SDK loaded\n";}
        return safe;
    }

    void load() {
        // The original working integration used GLOBAL; retain that behavior
        // because vendor plugins may resolve symbols through the main namespace.
        library_ = dlopen(options_.sdk_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
        if (!library_) {
            const char *detail = dlerror();
            cameraError(ErrorCode::CameraAcquisitionFailed,
                        "cannot load camera SDK " + options_.sdk_path +
                            (detail ? std::string(": ") + detail : std::string()),
                        "camera.mo.load");
        }
        try {
            open_camera_ = symbol<OpenCamera>("moOpenCamera");
            close_camera_ = symbol<CloseCamera>("moCloseCamera");
            if (!options_.config_path.empty())
                set_config_ = symbol<SetConfig>("moSetConfigPath");
            if (options_.stream == "rgbd") {
                get_rgbd_ = symbol<GetRgbd>("moGetOneRGBDFrame");
                release_rgbd_ = symbol<ReleaseRgbd>("moReleaseRGBDFrame");
            } else if (options_.stream == "gdc") {
                get_gdc_ = symbol<GetGdc>("moGetOneGdcFrame");
                release_gdc_ = symbol<ReleaseGdc>("moReleaseGdcFrame");
            } else {
                throw std::invalid_argument("camera stream must be rgbd or gdc");
            }
            if (set_config_) {
                const int code = set_config_(options_.config_path.c_str());
                if (code != kOk)
                    cameraError(ErrorCode::CameraAcquisitionFailed,
                                "moSetConfigPath failed: " + std::to_string(code),
                                "camera.mo.configure");
            }
            const int code = traced("moOpenCamera",[&]{return open_camera_(&handle_);});
            if (code != kOk || handle_ < 0)
                cameraError(ErrorCode::CameraAcquisitionFailed,
                            "moOpenCamera failed: " + std::to_string(code),
                            "camera.mo.open");
            // Optional APIs are only touched when explicitly requested by the
            // diagnostic worker, preserving the previous adapter's defaults.
            if(options_.disparity_mode || options_.stereo_trace || options_.demand_enabled) {
                set_mode_=optionalSymbol<decltype(set_mode_)>("moSetDisparityCalculationMode");
                get_mode_=optionalSymbol<decltype(get_mode_)>("moGetDisparityCalculationMode");
                get_ratio_=optionalSymbol<decltype(get_ratio_)>("moGetBpuRatio");
                get_org_fps_=optionalSymbol<decltype(get_org_fps_)>("moGetFrameRateOrg");
                get_disp_fps_=optionalSymbol<decltype(get_disp_fps_)>("moGetFrameRateDisp");
                set_callback_=optionalSymbol<decltype(set_callback_)>("moGetStereoVisionStatusCallback");
                diagnostic_.requested_mode=options_.disparity_mode;
                if(get_mode_ && traced("get_original_mode",[&]{return get_mode_(handle_,&original_mode_);})==kOk) {
                    diagnostic_.original_mode=diagnostic_.effective_mode=original_mode_;
                    diagnostic_.mode_available=true;
                }
                if(options_.disparity_mode) {
                    if(!set_mode_ || !get_mode_)throw std::runtime_error("UNSUPPORTED: SDK disparity mode set/get API unavailable");
                    if(diagnostic_.effective_mode<0)throw std::runtime_error("SDK original disparity mode query failed; mode was not changed");
                    if(traced("set_requested_mode",[&]{return set_mode_(handle_,options_.disparity_mode);})!=kOk)throw std::runtime_error("SDK disparity mode set failed");
                    if(traced("readback_mode",[&]{return get_mode_(handle_,&diagnostic_.effective_mode);})!=kOk || diagnostic_.effective_mode!=options_.disparity_mode)
                        throw std::runtime_error("SDK disparity mode readback mismatch");
                }
                if(options_.demand_enabled) {
                    if((options_.stream!="gdc" && !options_.allow_rgbd_mode1_probe) || diagnostic_.effective_mode!=1)
                        throw std::invalid_argument("on-demand disparity requires mode 1 and GDC or explicit RGBD capability probe");
                    calculate_=symbol<decltype(calculate_)>("moCalculateDisparityByFrameId");
                    auto params=symbol<decltype(&moGetCameraParams)>("moGetCameraParams");
                    MoMonoCameraParams mono{};double bxf=0,baseline=0;int maximum=0;
                    if(params(handle_,&mono,&bxf,&baseline,&demand_width_,&demand_height_,&maximum)!=kOk ||
                       demand_width_<2 || demand_height_<2 || demand_width_>4096 || demand_height_>4096)
                        throw std::runtime_error("SDK disparity resolution query failed/out of bounds");
                    // Vendor API has no capacity argument. Reserve its supported
                    // 4096x4096 bound without touching unused pages. Validate the
                    // returned size before copying; no stale output on failures.
                    demand_buffer_.reset(new std::uint16_t[demand_capacity_]);
                }
                if(options_.stereo_trace && set_callback_) {
                    {std::lock_guard<std::mutex> lock(stereo_storage.mutex);stereo_storage.size=0;
                     stereo_storage.starts=stereo_storage.ends=stereo_storage.dropped=0;stereo_storage.start_ns=stereo_storage.busy_ns=0;}
                    stereo_storage.enabled.store(true,std::memory_order_release);
                    diagnostic_.callback_available=traced("callback_register",[&]{return set_callback_(handle_,stereoCallback);})==kOk;
                    if(!diagnostic_.callback_available)stereo_storage.enabled.store(false,std::memory_order_release);
                }
            }
        } catch (...) {
            const bool safe_to_unload = closeForUnload("camera initialization cleanup");
            if (library_ && safe_to_unload) {
                traced("dlclose",[&]{return dlclose(library_);});
            }
            library_ = nullptr;
            throw;
        }
    }

    struct AcquireResult {
        int code = -1;
        Frame frame;
    };

    AcquireResult acquireOnce() {
        std::lock_guard<std::mutex> api_lock(api_mutex_);
        MoFrame left{}, right{}, disparity{};
        int acquire = -1;
        int release = kOk;
        Frame result;
        const auto get_begin=Clock::now();
        if (options_.stream == "rgbd") {
            acquire = get_rgbd_(handle_, &left, &right, &disparity);
            result.acquisition.sdk_get_ms=std::chrono::duration<double,std::milli>(Clock::now()-get_begin).count();
            if (acquire != kOk) {
                // The vendor integration releases even after a failed get:
                // an error may still return partially-owned buffers.
                if (left.Data || right.Data || disparity.Data) {
                    const auto release_begin=Clock::now();
                    const int partial_release =
                        release_rgbd_(handle_, &left, &right, &disparity);
                    result.acquisition.release_ms=std::chrono::duration<double,std::milli>(Clock::now()-release_begin).count();
                    if (partial_release != kOk)
                        cameraError(
                            ErrorCode::CameraAcquisitionFailed,
                            "partial RGBD release failed after acquisition error; "
                            "acquire=" + std::to_string(acquire) +
                                " release=" + std::to_string(partial_release),
                            "camera.mo.release");
                }
                return {acquire, std::move(result)};
            }
            try {
                inspectRgbdTriplet(left,right,disparity,options_,result);
                const auto copy_begin=Clock::now();
                result.sdk_stream_type=left.StreamType;result.sdk_bits_per_pixel=left.BitsPerPixel;
                result.sdk_bytes_used=left.BytesUsed;result.sdk_rgb_image_type=left.RGBImageType;
                result.image = copyImage(left, options_);
                if(options_.copy_disparity)result.disparity_pixels=copyRgbdDisparity(left,right,disparity,result);
                result.acquisition.copy_ms=std::chrono::duration<double,std::milli>(Clock::now()-copy_begin).count();
                result.acquisition.sdk_timestamp=left.TimeStamp!=0;
                result.info.id = left.FrameId;
                result.info.timestamp_ns =
                    left.TimeStamp ? left.TimeStamp
                                   : static_cast<std::uint64_t>(nowNs());
            } catch (...) {
                const int cleanup =
                    release_rgbd_(handle_, &left, &right, &disparity);
                if (cleanup != kOk)
                    std::cerr << "RGBD release also failed while rejecting frame: "
                              << cleanup << '\n';
                throw;
            }
            const auto release_begin=Clock::now();
            release = release_rgbd_(handle_, &left, &right, &disparity);
            result.acquisition.release_ms=std::chrono::duration<double,std::milli>(Clock::now()-release_begin).count();
        } else {
            acquire = get_gdc_(handle_, &left, &right);
            result.acquisition.sdk_get_ms=std::chrono::duration<double,std::milli>(Clock::now()-get_begin).count();
            if (acquire != kOk) {
                if (left.Data || right.Data) {
                    const auto release_begin=Clock::now();
                    const int partial_release =
                        release_gdc_(handle_, &left, &right);
                    result.acquisition.release_ms=std::chrono::duration<double,std::milli>(Clock::now()-release_begin).count();
                    if (partial_release != kOk)
                        cameraError(
                            ErrorCode::CameraAcquisitionFailed,
                            "partial GDC release failed after acquisition error; "
                            "acquire=" + std::to_string(acquire) +
                                " release=" + std::to_string(partial_release),
                            "camera.mo.release");
                }
                return {acquire, std::move(result)};
            }
            try {
                const auto copy_begin=Clock::now();
                result.sdk_stream_type=left.StreamType;result.sdk_bits_per_pixel=left.BitsPerPixel;
                result.sdk_bytes_used=left.BytesUsed;result.sdk_rgb_image_type=left.RGBImageType;
                result.image = copyImage(left, options_);
                result.acquisition.copy_ms=std::chrono::duration<double,std::milli>(Clock::now()-copy_begin).count();
                result.acquisition.sdk_timestamp=left.TimeStamp!=0;
                result.info.id = left.FrameId;
                result.info.timestamp_ns =
                    left.TimeStamp ? left.TimeStamp
                                   : static_cast<std::uint64_t>(nowNs());
            } catch (...) {
                const int cleanup = release_gdc_(handle_, &left, &right);
                if (cleanup != kOk)
                    std::cerr << "GDC release also failed while rejecting frame: "
                              << cleanup << '\n';
                throw;
            }
            const auto release_begin=Clock::now();
            release = release_gdc_(handle_, &left, &right);
            result.acquisition.release_ms=std::chrono::duration<double,std::milli>(Clock::now()-release_begin).count();
        }
        if (acquire == kOk && release != kOk)
            cameraError(ErrorCode::CameraAcquisitionFailed,
                        "camera frame release failed: " + std::to_string(release),
                        "camera.mo.release");
        return {acquire, std::move(result)};
    }

  public:
    explicit MoSource(Options options) : options_(std::move(options)) {
        if (options_.sdk_path.empty())
            throw std::invalid_argument("camera SDK path is empty");
        if (options_.acquire_timeout_ms <= 0 || options_.retry_interval_ms <= 0 ||
            options_.retry_interval_ms > options_.acquire_timeout_ms)
            throw std::invalid_argument("invalid camera timeout/retry interval");
        (void)pixelFormat(options_.format);
        if(options_.disparity_mode<0 || options_.disparity_mode>2 ||
           (options_.disparity_mode==1 && options_.stream!="gdc" && !options_.allow_rgbd_mode1_probe))
            throw std::invalid_argument("disparity mode 1 requires GDC stream or explicit RGBD capability probe; modes are 0,1,2");
        if(options_.allow_rgbd_mode1_probe && (options_.stream!="rgbd" || options_.disparity_mode!=1))
            throw std::invalid_argument("RGBD capability probe requires rgbd stream and disparity mode 1");
        if(options_.audit_rgbd_triplet && options_.stream!="rgbd")
            throw std::invalid_argument("RGBD triplet audit requires rgbd stream");
        if(options_.copy_disparity && options_.stream!="rgbd")
            throw std::invalid_argument("copy_disparity requires rgbd stream; it does not calculate GDC disparity");
        load();
    }

    ~MoSource() override {
        const bool safe_to_unload = closeForUnload("camera shutdown");
        if (library_ && safe_to_unload)
            traced("dlclose",[&]{return dlclose(library_);});
    }

    Frame next() override {
        const auto deadline =
            Clock::now() + std::chrono::milliseconds(options_.acquire_timeout_ms);
        int last_code = -1;
        AcquisitionTiming total;
        do {
            auto acquired = acquireOnce();
            last_code = acquired.code;
            ++total.attempts;total.sdk_get_ms+=acquired.frame.acquisition.sdk_get_ms;
            total.copy_ms+=acquired.frame.acquisition.copy_ms;total.release_ms+=acquired.frame.acquisition.release_ms;
            if(last_code!=kOk)total.last_error=last_code;
            if (last_code == kOk && acquired.frame.image) {
                total.sdk_timestamp=acquired.frame.acquisition.sdk_timestamp;acquired.frame.acquisition=total;
                acquired.frame.received_host_ns=nowNs();
                return std::move(acquired.frame);
            }
            if (Clock::now() < deadline) {
                const auto sleep_begin=Clock::now();
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(options_.retry_interval_ms));
                total.retry_sleep_ms+=std::chrono::duration<double,std::milli>(Clock::now()-sleep_begin).count();
            }
        } while (Clock::now() < deadline);
        cameraError(ErrorCode::CameraAcquisitionFailed,
                    "camera acquisition timed out; last SDK code=" +
                        std::to_string(last_code),
                    "camera.mo.acquire");
    }

    const char *name() const noexcept override { return "mo-camera"; }
    DisparityResult calculateDisparity(std::uint64_t frame_id) override {
        std::lock_guard<std::mutex> lock(api_mutex_);
        if(!calculate_ || !demand_buffer_)throw std::runtime_error("UNSUPPORTED: demand mode not enabled");
        // SDK 1.4.4.0 multiplies this PUBLIC ID by 3 internally. Never multiply
        // the caller ID. Limit it before the vendor signed-int multiplication.
        if(frame_id>std::uint64_t(std::numeric_limits<int>::max()/3))
            throw std::invalid_argument("public frame ID exceeds safe demand API range");
        DisparityResult r;r.frame_id=frame_id;
        const auto begin=Clock::now();
        r.call_begin_host_ns=nowNs();
        r.code=calculate_(handle_,int(frame_id),demand_buffer_.get(),&r.width,&r.height);
        r.call_end_host_ns=nowNs();
        r.call_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
        if(r.code==kOk) {
            if(r.width!=demand_width_ || r.height!=demand_height_ || r.width<2 || r.height<2 ||
               std::size_t(r.width)*r.height>demand_capacity_)
                throw std::runtime_error("SDK demand output differs from queried resolution");
            const auto copy_begin=Clock::now();
            r.pixels.assign(demand_buffer_.get(),demand_buffer_.get()+std::size_t(r.width)*r.height);
            r.copy_ms=std::chrono::duration<double,std::milli>(Clock::now()-copy_begin).count();
        }
        return r; // On SDK error the pixels vector is empty, not prior output.
    }

    Diagnostics diagnostics(bool sample_sdk=false) override {
        auto d=diagnostic_;
        if(sample_sdk) {
            std::lock_guard<std::mutex> api_lock(api_mutex_);
            d.mode_available=get_mode_ && get_mode_(handle_,&d.effective_mode)==kOk;
            if(!d.mode_available)d.effective_mode=-1;
            if(get_ratio_)d.ratio_available=get_ratio_(handle_,&d.bpu_ratio)==kOk;
            if(!d.ratio_available)d.bpu_ratio=-1;
            if(get_org_fps_)d.original_fps_available=get_org_fps_(handle_,&d.original_fps)==kOk;
            if(!d.original_fps_available)d.original_fps=-1;
            if(get_disp_fps_)d.disparity_fps_available=get_disp_fps_(handle_,&d.disparity_fps)==kOk;
            if(!d.disparity_fps_available)d.disparity_fps=-1;
        }
        if(!d.callback_available)return d;
        std::lock_guard<std::mutex> lock(stereo_storage.mutex);
        d.stereo_starts=stereo_storage.starts;d.stereo_ends=stereo_storage.ends;d.stereo_dropped_events=stereo_storage.dropped;
        d.stereo_busy_ms=stereo_storage.busy_ns/1e6;return d;
    }
    std::vector<StereoEvent> stereoEvents() override {
        if(!diagnostic_.callback_available)return {};
        std::lock_guard<std::mutex> lock(stereo_storage.mutex);
        return {stereo_storage.events.begin(),stereo_storage.events.begin()+stereo_storage.size};
    }
};

#endif

} // namespace

std::shared_ptr<Source> open(const Options &options) {
#if defined(__linux__) && defined(PERCEPTION_MO_VENDOR_TYPES_AVAILABLE)
    return std::make_shared<MoSource>(options);
#elif defined(__linux__)
    (void)options;
    cameraError(ErrorCode::CameraAcquisitionFailed,
                "direct Mo camera input was not enabled at build time; "
                "configure with the target Mo SDK headers",
                "camera.mo.build");
#else
    (void)options;
    cameraError(ErrorCode::CameraAcquisitionFailed,
                "direct Mo camera input is available only on Linux/X5",
                "camera.mo.platform");
#endif
}

} // namespace demo_camera
