#include "perception_v0_host/input.hpp"
#include <cmath>
#include <limits>
#include <utility>

namespace perception::v0_host {
namespace {
bool nv12Valid(const Nv12Image& image,std::string& error) {
    if (!image.width || !image.height || (image.width&1u) || (image.height&1u) ||
        image.stride<image.width || (image.stride&1u) || !image.timestamp_ns) {
        error="invalid NV12 geometry, stride or original timestamp";return false;
    }
    const uint64_t pixels=uint64_t(image.stride)*image.height;
    if (pixels>std::numeric_limits<uint64_t>::max()/3u) {
        error="NV12 producer geometry overflows its payload size";return false;
    }
    const uint64_t required=pixels*3u/2u;
    if (required!=image.bytes.size()) {
        error="NV12 payload size does not match its producer descriptor";return false;
    }
    return true;
}
psdk_frame imageView(const Nv12Image& image) {
    psdk_frame result{};result.struct_size=sizeof(result);result.abi_version=PSDK_ABI_VERSION;
    result.format=PSDK_FORMAT_NV12;result.width=image.width;result.height=image.height;
    result.stride=image.stride;result.data=image.bytes.data();result.bytes=image.bytes.size();
    result.frame_id=image.frame_id;result.timestamp_ns=image.timestamp_ns;return result;
}
} // namespace

psdk_full_frame InputFrame::view(psdk_frame& hires,psdk_disparity_q8& q8) const {
    if (supplied) {
        auto result=*supplied;
        result.recharge_hires=supplied_hires?&*supplied_hires:nullptr;
        result.right=supplied_right?&*supplied_right:nullptr;
        result.rear=supplied_rear?&*supplied_rear:nullptr;
        result.disparity=supplied_q8?&*supplied_q8:nullptr;
        result.raw_light=supplied_raw?&*supplied_raw:nullptr;
        result.imu=supplied_imu?&*supplied_imu:nullptr;
        result.wheel=supplied_wheel?&*supplied_wheel:nullptr;
        result.radar=supplied_radar?&*supplied_radar:nullptr;
        return result;
    }
    psdk_full_frame result{};result.struct_size=sizeof(result);result.abi_version=PSDK_FULL_ABI_VERSION;
    result.left=imageView(left);result.intrinsics=intrinsics;
    if (right) {right_descriptor=imageView(*right);result.right=&right_descriptor;}
    if (command==2) {
        // The real GDC image supplies BOTH descriptors. No upsampled RGBD image
        // and no fabricated cross-stream identity is accepted here.
        hires=result.left;result.recharge_hires=&hires;
    }
    if (disparity) {
        q8={};q8.struct_size=sizeof(q8);q8.abi_version=PSDK_FULL_ABI_VERSION;
        q8.width=disparity->width;q8.height=disparity->height;q8.stride_bytes=disparity->stride_bytes;
        q8.data=disparity->samples.data();q8.bytes=disparity->samples.size()*sizeof(uint16_t);
        q8.frame_id=disparity->frame_id;q8.timestamp_ns=disparity->timestamp_ns;
        result.disparity=&q8;
    }
    return result;
}

bool BoardInput::setIntrinsics(const CameraParameters& p,std::string& error) {
    error.clear();
    if (!p.width || !p.height || !std::isfinite(p.fx) || !std::isfinite(p.fy) ||
        !std::isfinite(p.cx) || !std::isfinite(p.cy) || !std::isfinite(p.baseline_mm) ||
        !std::isfinite(p.bxf) || p.fx<=1e-3 || p.fy<=1e-3 || p.fx>p.width*2.0 ||
        p.fy>p.height*2.0 || p.cx < -10 || p.cx>p.width+10.0 || p.cy < -10 ||
        p.cy>p.height+10.0 || p.baseline_mm<=1e-3 || p.baseline_mm>200.0 ||
        std::abs(p.bxf-p.fx*p.baseline_mm)>2.0) {
        error="CameraParam failed V0 focal, principal point, baseline or bxf validation";return false;
    }
    const bool changed=!calibration_ || p.width!=calibration_->width || p.height!=calibration_->height ||
        p.fx!=calibration_->fx || p.fy!=calibration_->fy || p.cx!=calibration_->cx ||
        p.cy!=calibration_->cy || p.bxf!=calibration_->bxf || p.baseline_mm!=calibration_->baseline_mm;
    calibration_=p;
    if (changed) clear(); // Never run queued pixels with a different calibration.
    return true;
}

bool BoardInput::setCommand(int command) {
    if (command!=1 && command!=2 && command!=21) return false;
    if (command==command_) return false;
    command_=command;++generation_;clear();return true;
}

bool BoardInput::makeIntrinsics(uint32_t width,uint32_t height,
                               psdk_camera_intrinsics& out,std::string& error) const {
    if (!calibration_) {error="CameraParam has not been received";return false;}
    const auto& p=*calibration_;
    // Explicit proportional calibration for the V0 RGBD/GDC resolution pair.
    // Cropped or differently shaped streams require their own CameraParam.
    if (uint64_t(width)*p.height!=uint64_t(height)*p.width) {
        error="frame geometry is incompatible with CameraParam aspect ratio";return false;
    }
    const double sx=double(width)/p.width,sy=double(height)/p.height;
    out={width,height,p.fx*sx,p.fy*sy,p.cx*sx,p.cy*sy,p.bxf*sx};return true;
}

bool BoardInput::onRgbd(Nv12Image left,Q8Disparity q8,std::string& error) {
    error.clear();if (command_!=1) return false;
    if (!nv12Valid(left,error)) return false;
    if (q8.width!=left.width || q8.height!=left.height || q8.stride_bytes<q8.width*2ull ||
        (q8.stride_bytes&1u) || uint64_t(q8.stride_bytes)*q8.height!=q8.samples.size()*2ull ||
        q8.frame_id!=left.frame_id || q8.timestamp_ns!=left.timestamp_ns) {
        error="SDK Q8 geometry or original identity differs from its RGBD left image";return false;
    }
    InputFrame frame;frame.command=1;frame.generation=generation_;
    if (!makeIntrinsics(left.width,left.height,frame.intrinsics,error)) return false;
    frame.left=std::move(left);frame.disparity=std::move(q8);
    if (pending_) ++overwritten_;
    pending_=std::move(frame);return true;
}

bool BoardInput::onGdc(Nv12Image left,std::string& error) {
    error.clear();if (command_!=2) return false;
    if (!nv12Valid(left,error)) return false;
    if (left.width!=1280 || left.height!=1088) {
        error="recharge requires the producer's real 1280x1088 GDC left image";return false;
    }
    InputFrame frame;frame.command=2;frame.generation=generation_;
    if (!makeIntrinsics(left.width,left.height,frame.intrinsics,error)) return false;
    frame.left=std::move(left);
    if (pending_) ++overwritten_;
    pending_=std::move(frame);return true;
}

bool BoardInput::onStereo(Nv12Image left,Nv12Image right,std::string& error) {
    error.clear();if (command_!=1) return false;
    if (!nv12Valid(left,error) || !nv12Valid(right,error)) return false;
    if (left.width!=right.width || left.height!=right.height || left.frame_id!=right.frame_id ||
        left.timestamp_ns!=right.timestamp_ns) {
        error="Stereo right image differs from the original left capture";return false;
    }
    InputFrame frame;frame.command=1;frame.generation=generation_;
    if (!makeIntrinsics(left.width,left.height,frame.intrinsics,error)) return false;
    frame.left=std::move(left);frame.right=std::move(right);
    if (pending_) ++overwritten_;
    pending_=std::move(frame);return true;
}

bool BoardInput::onFullFrame(psdk_full_frame frame,std::shared_ptr<const void> owner,std::string& error) {
    error.clear();if (command_!=1 && command_!=2) return false;
    if (!owner || frame.struct_size<sizeof(psdk_full_frame) || frame.abi_version!=PSDK_FULL_ABI_VERSION ||
        !frame.left.data || !frame.left.bytes || !frame.left.timestamp_ns) {
        error="rich board input requires the complete SDK frame ABI and a live storage owner";return false;
    }
    if (command_==2 && (frame.left.width!=1280 || frame.left.height!=1088 || !frame.recharge_hires ||
        frame.recharge_hires->data!=frame.left.data || frame.recharge_hires->frame_id!=frame.left.frame_id ||
        frame.recharge_hires->timestamp_ns!=frame.left.timestamp_ns ||
        frame.recharge_hires->width!=frame.left.width || frame.recharge_hires->height!=frame.left.height)) {
        error="rich recharge input must use the real 1280x1088 GDC as left and hires";return false;
    }
    InputFrame pending;pending.command=command_;pending.generation=generation_;
    pending.owner=std::move(owner);pending.supplied=frame;
    if (frame.recharge_hires) pending.supplied_hires=*frame.recharge_hires;
    if (frame.right) pending.supplied_right=*frame.right;
    if (frame.rear) pending.supplied_rear=*frame.rear;
    if (frame.disparity) pending.supplied_q8=*frame.disparity;
    if (frame.raw_light) pending.supplied_raw=*frame.raw_light;
    if (frame.imu) pending.supplied_imu=*frame.imu;
    if (frame.wheel) pending.supplied_wheel=*frame.wheel;
    if (frame.radar) pending.supplied_radar=*frame.radar;
    if (pending_) ++overwritten_;
    pending_=std::move(pending);return true;
}

std::optional<InputFrame> BoardInput::take() {
    auto result=std::move(pending_);pending_.reset();return result;
}
void BoardInput::clear() {pending_.reset();}
} // namespace perception::v0_host
