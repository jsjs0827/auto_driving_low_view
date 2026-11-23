#include "stereo_inputs.hpp"
#include "runtime_status.hpp"
#include "perception/error.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace r33::stereo {
namespace {
[[noreturn]] void fail(std::uint32_t code,const char* detail,perception::FrameInfo frame={},int native=0,const char* feature="stereo.input.validate") {
    throw r28::Failure({code,native,feature,{},detail,frame});
}
void validateStorage(const Q8View& disparity) {
    if(!disparity.data||disparity.width<1||disparity.height<1||disparity.width>8192||disparity.height>8192)
        fail(1002,"Invalid Q8.8 disparity pointer/geometry",disparity.frame);
    const auto row=std::size_t(disparity.width)*sizeof(std::uint16_t);
    if(disparity.stride_bytes<row||disparity.stride_bytes%2||disparity.stride_bytes>16U*1024U*1024U)
        fail(1002,"Invalid Q8.8 row stride",disparity.frame);
    const auto rows=std::size_t(disparity.height-1);
    if(rows&&disparity.stride_bytes>(std::numeric_limits<std::size_t>::max()-row)/rows)
        fail(1002,"Q8.8 storage extent overflow",disparity.frame);
    if(disparity.bytes<disparity.stride_bytes*rows+row)
        fail(1002,"Short Q8.8 disparity storage",disparity.frame);
}
struct CloseEvidence {
    std::mutex mutex;
    int first_error=0;
    std::string first_call;
    void record(const char* call,int code) {
        if(code==0)return;
        const std::string name=call?call:"";
        if(name!="moCloseCamera"&&name!="moSetDisparityCalculationMode_restore"&&name!="callback_replace_noop"&&name!="dlclose")return;
        std::lock_guard<std::mutex> lock(mutex);if(!first_error){first_error=code;first_call=name;}
    }
};
}
void validateIntrinsics(const Intrinsics& k) {
    if(k.width<1||k.height<1||k.width>8192||k.height>8192||
       !std::isfinite(k.fx)||!std::isfinite(k.fy)||!std::isfinite(k.cx)||!std::isfinite(k.cy)||!std::isfinite(k.bxf)||
       k.fx<=0||k.fy<=0||k.bxf<=0||k.cx<0||k.cy<0||k.cx>=k.width||k.cy>=k.height)
        fail(2001,"Invalid original stereo intrinsics/bxf",{},0,"stereo.calibration");
}
void validateDisparity(const perception::ImageView& color,perception::FrameInfo frame,
                       const Q8View& disparity,const Intrinsics& k) {
    validateIntrinsics(k);
    try{perception::validateCameraImage(color);}catch(const perception::PerceptionException& e){
        throw r28::Failure({1002,0,"stereo.input.validate",{},e.what(),frame});}
    if(color.format!=perception::PixelFormat::NV12&&color.format!=perception::PixelFormat::I420)
        fail(1002,"Stereo color input requires explicit NV12 or I420",frame);
    validateStorage(disparity);
    if(!frame.timestamp_ns||!disparity.frame.timestamp_ns||frame.id!=disparity.frame.id||frame.timestamp_ns!=disparity.frame.timestamp_ns)
        fail(1002,"Color and disparity frame identity/timestamp differ or are unverified",frame);
    if(color.width!=disparity.width||color.height!=disparity.height||color.width!=k.width||color.height!=k.height)
        fail(1002,"Color, disparity and intrinsics dimensions differ",frame);
}
void validateHires(const perception::ImageView& image,perception::FrameInfo frame,perception::FrameInfo rgbd) {
    if(!frame.timestamp_ns||!rgbd.timestamp_ns||frame.id!=rgbd.id||frame.timestamp_ns!=rgbd.timestamp_ns)
        fail(1002,"High-resolution GDC input and RGBD frame identity differ or are unverified",rgbd);
    try{perception::validateCameraImage(image);}catch(const perception::PerceptionException& e){
        throw r28::Failure({1002,0,"stereo.hires.validate",{},e.what(),rgbd});}
    if(image.format!=perception::PixelFormat::NV12||image.width!=1280||image.height!=1088||image.stride!=1280||image.bytes!=1280U*1088U*3U/2U)
        fail(1002,"Original GDC input requires tight 1280x1088 NV12",rgbd,0,"stereo.hires.validate");
}
std::shared_ptr<const std::vector<std::uint16_t>> copyDisparity(
    const perception::ImageView& color,perception::FrameInfo frame,const Q8View& source,const Intrinsics& k) {
    validateDisparity(color,frame,source,k);
    auto owned=std::make_shared<std::vector<std::uint16_t>>(std::size_t(source.width)*source.height);
    const auto* data=static_cast<const std::uint8_t*>(source.data);const auto row=std::size_t(source.width)*sizeof(std::uint16_t);
    for(int y=0;y<source.height;++y)std::memcpy(owned->data()+std::size_t(y)*source.width,data+std::size_t(y)*source.stride_bytes,row);
    return owned;
}
perception::fusion::DisparityView frameworkView(const Q8View& source) {
    validateStorage(source);return {source.data,source.bytes,source.stride_bytes,source.width,source.height,perception::fusion::DisparityType::UInt16,256.0f};
}
Q8View Frame::disparityView() const {
    if(!left||!disparity)return {};
    return {disparity->data(),disparity->size()*sizeof(std::uint16_t),std::size_t(left->width)*sizeof(std::uint16_t),left->width,left->height,info};
}
Frame fromCameraFrame(const demo_camera::Frame& source,const Intrinsics& k) {
    if(k.width!=640||k.height!=544)fail(2001,"Original RGBD camera adapter requires 640x544 intrinsics",{},0,"stereo.camera.configure");
    if(!source.image||!source.disparity_pixels||!source.sdk_rgbd_triplet_available||!source.sdk_rgbd_triplet_valid||!source.sdk_color_pair_valid||!source.acquisition.sdk_timestamp)
        fail(1002,"Vendor RGBD frame lacks verified same-frame color pair/disparity metadata",source.info);
    if(source.image->format!=perception::PixelFormat::NV12||source.image->width!=640||source.image->height!=544||source.image->stride!=640||source.image->bytes.size()!=640U*544U*3U/2U)
        fail(1002,"Vendor left input violates original tight 640x544 NV12 contract",source.info);
    const auto matching=[&](std::uint64_t id,std::uint64_t time,int w,int h){return id==source.info.id&&time==source.info.timestamp_ns&&time!=0&&w==640&&h==544;};
    if(!matching(source.sdk_right_frame_id,source.sdk_right_timestamp_ns,source.sdk_right_width,source.sdk_right_height)||
       !matching(source.sdk_disparity_frame_id,source.sdk_disparity_timestamp_ns,source.sdk_disparity_width,source.sdk_disparity_height)||
       source.sdk_right_bytes_used!=640U*544U*3U/2U||source.sdk_disparity_bytes_used!=640U*544U*2U||source.disparity_pixels->size()!=640U*544U)
        fail(1002,"Vendor RGBD metadata conflicts with the owned pixel planes",source.info);
    Frame out;out.info=source.info;out.left=source.image;out.disparity=source.disparity_pixels;out.acquisition=source.acquisition;
    out.right={source.info,source.sdk_right_width,source.sdk_right_height,source.sdk_right_bytes_used,true,false};
    validateDisparity(out.left->view(),out.info,out.disparityView(),k);return out;
}
struct CameraAdapter::Impl {
    Intrinsics intrinsics;
    std::shared_ptr<CloseEvidence> evidence=std::make_shared<CloseEvidence>();
    std::shared_ptr<demo_camera::Source> source;
    std::mutex mutex;
};
CameraAdapter::CameraAdapter(const CameraOptions& options,const Intrinsics& intrinsics):impl_(std::make_unique<Impl>()) {
    validateIntrinsics(intrinsics);if(intrinsics.width!=640||intrinsics.height!=544)fail(2001,"Original RGBD mode requires 640x544 intrinsics",{},0,"stereo.camera.configure");
    impl_->intrinsics=intrinsics;demo_camera::Options original;
    original.sdk_path=options.sdk_path;original.config_path=options.config_path;original.stream="rgbd";original.format="nv12";
    original.acquire_timeout_ms=options.acquire_timeout_ms;original.retry_interval_ms=options.retry_interval_ms;
    original.disparity_mode=2;original.audit_rgbd_triplet=true;original.copy_disparity=true;
    const auto evidence=impl_->evidence;original.lifecycle=[evidence](const char* call,int code,double){evidence->record(call,code);};
    impl_->source=demo_camera::open(original);
}
CameraAdapter::CameraAdapter(std::shared_ptr<demo_camera::Source> source,const Intrinsics& intrinsics):impl_(std::make_unique<Impl>()) {
    validateIntrinsics(intrinsics);if(!source)fail(2001,"Camera Source is null",{},0,"stereo.camera.configure");
    impl_->intrinsics=intrinsics;impl_->source=std::move(source);
}
CameraAdapter::~CameraAdapter(){try{close();}catch(...){}}
Frame CameraAdapter::next() {
    std::lock_guard<std::mutex> lock(impl_->mutex);if(!impl_->source)fail(1001,"Camera adapter is closed",{},0,"stereo.camera.acquire");
    return fromCameraFrame(impl_->source->next(),impl_->intrinsics);
}
demo_camera::Diagnostics CameraAdapter::diagnostics() {
    std::lock_guard<std::mutex> lock(impl_->mutex);if(!impl_->source)fail(1001,"Camera adapter is closed",{},0,"stereo.camera.diagnostics");
    return impl_->source->diagnostics(true);
}
void CameraAdapter::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);if(!impl_->source)return;
    impl_->source.reset();std::lock_guard<std::mutex> evidence_lock(impl_->evidence->mutex);
    if(impl_->evidence->first_error)throw r28::Failure({2006,impl_->evidence->first_error,"stereo.camera.close",{},impl_->evidence->first_call+" failed",{}});
}
}
