#include "r36_stereo/nv12.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
int assertions = 0;
void require(bool value, const char* message) {
  ++assertions;
  if (!value) throw std::runtime_error(message);
}
template<class F> void rejects(F function) {
  ++assertions;
  try { function(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("invalid YUV descriptor unexpectedly accepted");
}
}  // namespace

int main() {
  try {
    using namespace r36stereo;
    // Extreme original samples would change in a YUV -> BGR -> YUV round trip.
    std::vector<std::uint8_t> tight = {0,1,254,255,16,235,4,252,2,250,128,129,9,10,11,12,
                                     0,255,1,254,127,129,2,253};
    std::vector<std::uint8_t> padded(6 * 8, 0xee);
    for (int y = 0; y < 6; ++y)
      std::memcpy(padded.data() + y * 8, tight.data() + y * 4, 4);
    auto nv12 = copy_yuv420_to_nv12(padded.data(),4,4,8,padded.size(),true);
    require(nv12.isContinuous() && nv12_size(nv12) == cv::Size(4,4), "NV12 geometry/tight copy");
    require(std::equal(tight.begin(),tight.end(),nv12.data), "NV12 must preserve every Y/UV byte");
    auto identity = resize_nv12(nv12,{4,4});
    require(identity.data == nv12.data, "same-size adaptation must not colour-convert or resize");
    require(std::equal(tight.begin(),tight.end(),identity.data), "same-size identity bytes");
    // Independent I420 plane padding, including unused bytes after each U/V row.
    std::vector<std::uint8_t> i420(48,0xee);
    for (int y=0;y<4;++y)std::memcpy(i420.data()+y*8,tight.data()+y*4,4);
    for (int y=0;y<2;++y)for(int x=0;x<2;++x) {
      i420[32+y*4+x]=tight[16+y*4+2*x];
      i420[40+y*4+x]=tight[16+y*4+2*x+1];
    }
    auto planar = copy_yuv420_to_nv12(i420.data(),4,4,8,i420.size(),false);
    require(std::equal(tight.begin(),tight.end(),planar.data), "I420 must only interleave U,V");
    // A padded Mat catches accidental flat memcpy and resize of all 1.5H rows.
    cv::Mat view(6,4,CV_8UC1,padded.data(),8);
    auto from_view = resize_nv12(view,{4,4});
    require(from_view.step == 8 && from_view.ptr(4)[0] == 0, "NV12 Mat may retain row padding");
    cv::Mat gradient(6,4,CV_8UC1);
    for(int y=0;y<4;++y)for(int x=0;x<4;++x)gradient.at<std::uint8_t>(y,x)=std::uint8_t(2*(y*4+x));
    const std::uint8_t uv[] = {10,101,20,111,30,121,40,131};
    std::memcpy(gradient.ptr(4),uv,sizeof(uv));
    auto reduced = resize_nv12(gradient,{2,2});
    require(reduced.at<std::uint8_t>(0,0)==5 && reduced.at<std::uint8_t>(0,1)==9 &&
            reduced.at<std::uint8_t>(1,0)==21 && reduced.at<std::uint8_t>(1,1)==25,
            "Y resize samples must only come from Y plane");
    require(reduced.at<std::uint8_t>(2,0)==25 && reduced.at<std::uint8_t>(2,1)==116,
            "UV resize must interpolate U and V independently, never mix them");
    cv::Mat camera(544*3/2,640,CV_8UC1,cv::Scalar(13));
    for(int y=544;y<camera.rows;++y)for(int x=0;x<640;x+=2) {
      camera.at<std::uint8_t>(y,x)=33;camera.at<std::uint8_t>(y,x+1)=231;
    }
    auto model = resize_nv12(camera,{640,352});
    require(model.total()==337920 && nv12_size(model)==cv::Size(640,352), "352 model byte count");
    require(model.ptr(351)[639]==13 && model.ptr(352)[0]==33 && model.ptr(527)[639]==231,
            "352 input Y/UV boundary must not leak source rows");
    // Every visible source byte is distinct by row/column, with extra row
    // padding. The crop must start Y at 192 and UV at 544+192/2, preserving U,V.
    constexpr int camera_stride=648;
    std::vector<std::uint8_t> padded_camera(816*camera_stride,0xee);
    cv::Mat camera_view(816,640,CV_8UC1,padded_camera.data(),camera_stride);
    for(int y=0;y<544;++y)for(int x=0;x<640;++x)
      camera_view.ptr(y)[x]=std::uint8_t((y*17+x*3)&255);
    for(int y=0;y<272;++y)for(int x=0;x<640;x+=2) {
      camera_view.ptr(544+y)[x]=std::uint8_t((y*7+x/2)&255);
      camera_view.ptr(544+y)[x+1]=std::uint8_t((201+y*11+x/2)&255);
    }
    const auto camera_before=padded_camera;
    const auto cropped=crop_nv12(camera_view,{0,192,640,352});
    require(cropped.isContinuous() && cropped.step==640 && cropped.total()==337920 &&
            nv12_size(cropped)==cv::Size(640,352), "crop produces owned tight native model geometry");
    bool exact_crop=true;
    for(int y=0;y<352;++y)
      exact_crop=exact_crop && std::memcmp(cropped.ptr(y),camera_view.ptr(192+y),640)==0;
    for(int y=0;y<176;++y)
      exact_crop=exact_crop && std::memcmp(cropped.ptr(352+y),camera_view.ptr(544+96+y),640)==0;
    require(exact_crop,"crop must copy every Y/UV byte exactly, without interpolation or UV swap");
    require(padded_camera==camera_before,"crop cannot modify source samples or padding");
    require(cropped.data!=camera_view.data,"crop must own storage independent from borrowed source lifetime");
    const auto whole=crop_nv12(camera_view,{0,0,640,544});
    require(whole.total()==522240 && whole.ptr(815)[639]==camera_view.ptr(815)[639],
            "same-size crop copies full visible source without row padding");
    rejects([&]{crop_nv12(camera_view,{2,192,638,352});});
    rejects([&]{crop_nv12(camera_view,{0,192,638,352});});
    rejects([&]{crop_nv12(camera_view,{0,190,640,352});});
    rejects([&]{crop_nv12(camera_view,{0,193,640,352});});
    rejects([&]{crop_nv12(camera_view,{0,191,640,353});});
    rejects([&]{crop_nv12(camera_view,{0,-2,640,546});});
    rejects([&]{crop_nv12(camera_view,{0,192,640,0});});
    rejects([&]{crop_nv12(camera_view,{0,192,640,354});});
    cv::Mat red(2,2,CV_8UC3,cv::Scalar(0,0,255));
    auto converted=bgr_to_nv12(red);
    require(converted.ptr(0)[0]==82 && converted.ptr(2)[0]==90 && converted.ptr(2)[1]==240,
            "decoded BGR uses limited-range Y and UV order");
    rejects([&]{copy_yuv420_to_nv12(padded.data(),4,4,7,padded.size(),true);});
    rejects([&]{copy_yuv420_to_nv12(padded.data(),4,4,8,47,true);});
    rejects([&]{copy_yuv420_to_nv12(nullptr,4,4,8,48,true);});
    rejects([&]{copy_yuv420_to_nv12(padded.data(),3,4,8,48,true);});
    rejects([&]{resize_nv12(view,{3,2});});
    rejects([&]{nv12_size(cv::Mat(4,4,CV_8UC1));});
    rejects([&]{nv12_size(red);});
    std::cout << "PASS direct NV12: " << assertions << " assertions\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "FAIL direct NV12 after " << assertions << " assertions: " << e.what() << '\n';
    return 1;
  }
}
