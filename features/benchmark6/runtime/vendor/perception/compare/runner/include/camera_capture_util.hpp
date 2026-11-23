#pragma once
#include "perception_ros_so_demo/mo_camera.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>

namespace r36compare::runner {
struct CameraIntrinsics {
  int width=640,height=544;
  double fx=0,fy=0,cx=0,cy=0,bxf=0;
};
inline CameraIntrinsics cameraIntrinsics(const ros_so_demo::raw::CameraParameters& p) {
  if(!((p.width==640&&p.height==544)||(p.width==1280&&p.height==1088)))
    throw std::invalid_argument("Camera rectified reference must be verified 640x544 or 1280x1088; no guessed calibration");
  const auto& k=p.rectified;
  const double sx=640.0/p.width,sy=544.0/p.height;
  CameraIntrinsics out{640,544,k.fx*sx,k.fy*sy,k.cx*sx,k.cy*sy,p.bxf*sx};
  if(!std::isfinite(out.fx)||!std::isfinite(out.fy)||!std::isfinite(out.cx)||!std::isfinite(out.cy)||
     !std::isfinite(out.bxf)||out.fx<=0||out.fy<=0||out.bxf<=0||out.cx<0||out.cy<0||
     out.cx>=out.width||out.cy>=out.height)
    throw std::invalid_argument("Camera rectified reference intrinsics are invalid");
  return out;
}
inline bool validateCameraCapture(const ros_so_demo::raw::Capture& capture,bool need_q8,
                                  const std::optional<ros_so_demo::raw::Identity>& previous) {
  namespace raw=ros_so_demo::raw;
  const auto image=[](const raw::Plane& p) {
    raw::checkStamp(p.capture.ns);
    if(p.width!=640||p.height!=544||p.step!=640||p.encoding!="nv12"||p.bytes.size()!=640*544*3/2)
      throw std::invalid_argument("Camera source must retain tight 640x544 NV12 geometry");
  };
  image(capture.left);image(capture.right);
  raw::sameCapture(capture.left.capture,capture.right.capture,"camera runner left/right");
  if(need_q8) {
    const auto& d=capture.disparity;
    raw::sameCapture(capture.left.capture,d.capture,"camera runner left/Q8");
    if(d.width!=640||d.height!=544||d.step!=1280||d.encoding!="16UC1"||d.bytes.size()!=640*544*2)
      throw std::invalid_argument("Original camera mode requires genuine tight uint16 Q8");
  } else if(!capture.disparity.bytes.empty()) {
    throw std::invalid_argument("Disabled legacy disparity must not be copied into camera runner input");
  }
  // Timestamps define temporal order. Numeric capture IDs may wrap.
  const auto id=capture.left.capture;
  if(previous && id.ns==previous->ns && id.id==previous->id)return false;
  if(previous && (id.ns<=previous->ns || id.id==previous->id))
    throw std::invalid_argument("Duplicate/backward camera capture timestamp or repeated SDK capture ID");
  return true;
}
inline int requiredCameraMode(const std::string& method) {
  if(method=="original")return 2;
  if(method=="detseg_only"||method=="stereo")return 1;
  throw std::invalid_argument("Unknown camera comparison method");
}
} // namespace r36compare::runner
