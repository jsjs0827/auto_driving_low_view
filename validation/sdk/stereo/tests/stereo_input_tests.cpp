#include "stereo_inputs.hpp"
#include "runtime_status.hpp"
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value,const char* detail){if(!value)throw std::runtime_error(detail);}
void rejects(std::uint32_t code,const std::function<void()>& operation){
    bool rejected=false;try{operation();}catch(const r28::Failure& error){rejected=error.status().code==code;}
    require(rejected,"Invalid input accepted or wrong error code");
}
r33::stereo::Intrinsics cameraK(){return {640,544,298.404816f,298.404816f,313.862183f,274.359955f,17935.5};}
demo_camera::Frame cameraFrame(){
    demo_camera::Frame out;out.info={0,123456};
    auto image=std::make_shared<example::Image>();image->width=640;image->height=544;image->stride=640;
    image->format=perception::PixelFormat::NV12;image->bytes.resize(640*544*3/2,128);out.image=image;
    out.disparity_pixels=std::make_shared<std::vector<std::uint16_t>>(640*544,256*8);
    out.acquisition.sdk_timestamp=true;out.sdk_rgbd_triplet_available=true;out.sdk_rgbd_triplet_valid=true;out.sdk_color_pair_valid=true;
    out.sdk_right_frame_id=out.sdk_disparity_frame_id=0;out.sdk_right_timestamp_ns=out.sdk_disparity_timestamp_ns=out.info.timestamp_ns;
    out.sdk_right_width=out.sdk_disparity_width=640;out.sdk_right_height=out.sdk_disparity_height=544;
    out.sdk_right_bytes_used=640*544*3/2;out.sdk_disparity_bytes_used=640*544*2;return out;
}
struct FakeSource:demo_camera::Source {
    demo_camera::Frame value=cameraFrame();
    demo_camera::Frame next()override{return value;}
    const char* name()const noexcept override{return "test-only-no-camera";}
};
}
int main(){try{
    using namespace r33::stereo;
    std::vector<std::uint8_t> yuv(4*4*3/2,128);perception::ImageView color{yuv.data(),yuv.size(),4,4,4,perception::PixelFormat::NV12};
    Intrinsics intrinsics{4,4,3,3,1,1,180};perception::FrameInfo info{0,99};
    // Padded rows and intentionally unaligned storage must copy only valid Q8
    // samples, never row padding or an adjacent byte.
    std::vector<std::uint8_t> storage(1+4*12,0xee);for(int y=0;y<4;++y)for(int x=0;x<4;++x){std::uint16_t value=std::uint16_t((y*4+x+1)*256);std::memcpy(storage.data()+1+y*12+x*2,&value,2);}
    Q8View disparity{storage.data()+1,storage.size()-1,12,4,4,info};
    const auto owned=copyDisparity(color,info,disparity,intrinsics);require(owned->size()==16&&(*owned)[0]==256&&(*owned)[15]==4096,"Padded/unaligned Q8 copy changed pixels");
    storage[1]=255;require((*owned)[0]==256,"Owned Q8 pixels borrowed caller storage");
    const auto framework=frameworkView(disparity);require(framework.scale==256&&framework.type==perception::fusion::DisparityType::UInt16&&framework.stride_bytes==12,"Q8.8 semantic scale changed");
    auto bad=disparity;bad.bytes=43;rejects(1002,[&]{validateDisparity(color,info,bad,intrinsics);});
    bad=disparity;bad.stride_bytes=7;rejects(1002,[&]{validateDisparity(color,info,bad,intrinsics);});
    bad=disparity;bad.frame.id=1;rejects(1002,[&]{validateDisparity(color,info,bad,intrinsics);});
    bad=disparity;bad.frame.timestamp_ns=0;rejects(1002,[&]{validateDisparity(color,info,bad,intrinsics);});
    bad=disparity;bad.width=6;rejects(1002,[&]{validateDisparity(color,info,bad,intrinsics);});
    auto invalid=intrinsics;invalid.fx=std::numeric_limits<float>::quiet_NaN();rejects(2001,[&]{validateIntrinsics(invalid);});
    invalid=intrinsics;invalid.bxf=0;rejects(2001,[&]{validateIntrinsics(invalid);});
    std::vector<std::uint8_t> hires(1280U*1088U*3U/2U,128);
    perception::ImageView gdc{hires.data(),hires.size(),1280,1280,1088,perception::PixelFormat::NV12};
    validateHires(gdc,info,info);auto other=info;other.id++;
    rejects(1002,[&]{validateHires(gdc,other,info);});
    auto invalid_gdc=gdc;invalid_gdc.width=640;rejects(1002,[&]{validateHires(invalid_gdc,info,info);});
    auto original=cameraFrame();auto frame=fromCameraFrame(original,cameraK());
    require(frame.info.id==0&&frame.right.audited&&!frame.right.pixels_available,"Zero frame ID lost or right pixels fabricated");
    original.image.reset();original.disparity_pixels.reset();require(frame.left->bytes.size()==640U*544U*3/2&&frame.disparity->front()==2048,"Camera output did not own data after release");
    auto broken=cameraFrame();broken.sdk_disparity_timestamp_ns++;rejects(1002,[&]{fromCameraFrame(broken,cameraK());});
    broken=cameraFrame();broken.sdk_color_pair_valid=false;rejects(1002,[&]{fromCameraFrame(broken,cameraK());});
    auto fake=std::make_shared<FakeSource>();CameraAdapter adapter(fake,cameraK());
    auto retained=adapter.next();adapter.close();adapter.close();rejects(1001,[&]{adapter.next();});
    require(retained.disparity->front()==2048,"Closing source invalidated retained result");
#if defined(_WIN32)
    rejects(1001,[&]{FullCameraAdapter unsupported(CameraOptions{},cameraK(),true);});
#endif
    std::cout<<"Stereo Q8.8 stride, identity, intrinsics and ownership passed; injected frames, not camera or BPU evidence.\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
