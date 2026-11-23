#include "camera_capture_util.hpp"
#include <iostream>
#include <limits>

namespace raw=ros_so_demo::raw;
namespace runner=r36compare::runner;
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
template<class F> void rejects(F&& f) {
  try{f();}catch(const std::exception&){return;}
  throw std::runtime_error("Invalid camera input was accepted");
}
raw::Capture fixture(bool q8,std::uint64_t id=0,std::uint64_t ns=1700000000000000123ULL) {
  raw::Capture c;
  c.left={{id,ns},640,544,640,"nv12",std::vector<std::uint8_t>(640*544*3/2,17)};
  c.right=c.left;c.right.bytes[0]=19;
  if(q8)c.disparity={{id,ns},640,544,1280,"16UC1",std::vector<std::uint8_t>(640*544*2,2)};
  return c;
}
int main() {
  try {
    raw::CameraParameters p;p.width=1280;p.height=1088;
    p.rectified.fx=600;p.rectified.fy=602;p.rectified.cx=640;p.rectified.cy=544;p.bxf=36000;
    const auto k=runner::cameraIntrinsics(p);
    check(k.width==640&&k.height==544&&k.fx==300&&k.fy==301&&k.cx==320&&k.cy==272&&k.bxf==18000,
          "Verified SDK reference calibration must scale focal/principal/bxf without half-pixel offset");
    p.width=640;p.height=544;p.rectified.fx=300;p.rectified.fy=301;p.rectified.cx=320;p.rectified.cy=272;p.bxf=18000;
    check(runner::cameraIntrinsics(p).bxf==18000,"640 source reference must remain unchanged");
    auto bad=p;bad.width=800;rejects([&]{runner::cameraIntrinsics(bad);});
    bad=p;bad.rectified.cx=640;rejects([&]{runner::cameraIntrinsics(bad);});
    bad=p;bad.rectified.fx=std::numeric_limits<float>::quiet_NaN();rejects([&]{runner::cameraIntrinsics(bad);});
    check(runner::requiredCameraMode("original")==2&&runner::requiredCameraMode("detseg_only")==1&&runner::requiredCameraMode("stereo")==1,
          "Each method must select the declared vendor disparity producer mode");
    rejects([]{runner::requiredCameraMode("auto");});
    auto c=fixture(true);
    check(runner::validateCameraCapture(c,true,std::nullopt),"SDK ID zero is valid when timestamp is genuine");
    const auto previous=std::optional<raw::Identity>(c.left.capture);
    check(!runner::validateCameraCapture(c,true,previous),"Cached identical capture must be skipped, not counted");
    auto broken=c;broken.left.capture.ns--;broken.right.capture=broken.disparity.capture=broken.left.capture;
    rejects([&]{runner::validateCameraCapture(broken,true,previous);});
    broken=c;broken.left.capture.id=1;broken.right.capture=broken.disparity.capture=broken.left.capture;
    rejects([&]{runner::validateCameraCapture(broken,true,previous);});
    broken=c;broken.left.capture.ns++;broken.right.capture=broken.disparity.capture=broken.left.capture;
    rejects([&]{runner::validateCameraCapture(broken,true,previous);});
    auto wrapped=fixture(false,0,1001);
    check(runner::validateCameraCapture(wrapped,false,raw::Identity{std::numeric_limits<std::uint64_t>::max(),1000}),
          "Capture ID wrap must be allowed when SDK time moves forward");
    broken=c;broken.right.capture.ns++;rejects([&]{runner::validateCameraCapture(broken,true,std::nullopt);});
    broken=c;broken.disparity.bytes.pop_back();rejects([&]{runner::validateCameraCapture(broken,true,std::nullopt);});
    broken=c;broken.left.width=320;rejects([&]{runner::validateCameraCapture(broken,true,std::nullopt);});
    rejects([&]{runner::validateCameraCapture(c,false,std::nullopt);});
    check(runner::validateCameraCapture(fixture(false),false,std::nullopt),"Mode1 camera input must accept owned pair without Q8 pixels");
    std::cout<<"Camera calibration, producer modes, cached duplicate handling, contradictions, ID wrap and owned source contracts passed\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
