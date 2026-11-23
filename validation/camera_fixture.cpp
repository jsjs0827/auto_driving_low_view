// TEST ONLY: deterministic ABI fixture, never a camera runtime or board payload.
// Uses the real vendor headers. No camera, disparity or model accuracy is tested.
#include <MoLocalSdkInterface.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>
namespace {
int scenario=0, current_mode=1, rgbd_calls=0, rgbd_releases=0, gdc_releases=0, closes=0;
unsigned long long capture_id=0;
std::vector<unsigned char> left(640*544*3/2),right(left.size()),depth(640*544*2),high(1280*1088*3/2);
void fill(MoFrame* out,std::vector<unsigned char>& bytes,unsigned w,unsigned h,unsigned bpp) {
    *out={};out->FrameId=capture_id;out->TimeStamp=1700000000000000000ULL+capture_id*40000000ULL;
    out->ResolutionWidth=w;out->ResolutionHeight=h;out->BitsPerPixel=bpp;
    out->BytesUsed=static_cast<unsigned>(bytes.size());out->Data=bytes.data();
}
}
extern "C" void fixture_reset(int value) {
    scenario=value;current_mode=1;rgbd_calls=rgbd_releases=gdc_releases=closes=0;capture_id=0;
}
extern "C" int fixture_count(int what) {
    return what==0?rgbd_calls:what==1?rgbd_releases:what==2?gdc_releases:what==3?closes:current_mode;
}
extern "C" int moSetConfigPath(const char*) { return 0; }
extern "C" int moOpenCamera(int* h) { *h=7; return 0; }
extern "C" int moCloseCamera(int) { ++closes; return scenario==5?-5:0; }
extern "C" int moGetDisparityCalculationMode(int,int* mode) { *mode=current_mode; return 0; }
extern "C" int moSetDisparityCalculationMode(int,int mode) { current_mode=mode; return 0; }
extern "C" int moGetCameraParams(int,MoMonoCameraParams* k,double* bxf,double* baseline,int* w,int* h,int* maximum) {
    *k={};k->fx=600;k->fy=600;k->cx=640;k->cy=544;
    *bxf=36000;*baseline=60;*w=1280;*h=1088;*maximum=192;return 0;
}
extern "C" int moGetOneRGBDFrame(int,MoFrame* l,MoFrame* r,MoFrame* d) {
    ++rgbd_calls;++capture_id;std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::fill(left.begin(),left.end(),17);std::fill(right.begin(),right.end(),19);
    for (std::size_t i=0;i<depth.size();i+=2) { depth[i]=1;depth[i+1]=2; }
    fill(l,left,640,544,12);fill(r,right,640,544,12);fill(d,depth,640,544,16);
    if (scenario==1) ++d->TimeStamp;
    if (scenario==2 && rgbd_calls==1) return -11; // partial payload requires release
    if (scenario==3) return -11;
    return 0;
}
extern "C" int moReleaseRGBDFrame(int,MoFrame*,MoFrame*,MoFrame*) {
    ++rgbd_releases;std::fill(left.begin(),left.end(),255);std::fill(depth.begin(),depth.end(),0);
    return scenario==4?-5:0;
}
extern "C" int moGetOneGdcFrame(int,MoFrame* l,MoFrame* r) {
    std::fill(high.begin(),high.end(),23);fill(l,high,1280,1088,12);fill(r,high,1280,1088,12);return 0;
}
extern "C" int moReleaseGdcFrame(int,MoFrame*,MoFrame*) {
    ++gdc_releases;std::fill(high.begin(),high.end(),255);return 0;
}
