#include "perception_ros_so_demo/raw_capture.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>
namespace raw=ros_so_demo::raw;
void expect(bool ok,const char* label) { if (!ok) throw std::runtime_error(label); }
void rejects(const std::function<void()>& f,const char* label) {
    bool rejected=false; try { f(); } catch (const std::runtime_error&) { rejected=true; }
    expect(rejected,label);
}
MoFrame frame(std::vector<std::uint8_t>& bytes,unsigned w,unsigned h,unsigned bits=12) {
    MoFrame f{}; f.FrameId=0; f.TimeStamp=1700000000000000123ULL;
    f.ResolutionWidth=w; f.ResolutionHeight=h; f.BitsPerPixel=bits;
    f.BytesUsed=static_cast<unsigned>(bytes.size()); f.Data=bytes.data(); return f;
}
int main() {
    std::vector<std::uint8_t> pixels(640*544*3/2),depth(640*544*2),high(1280*1088*3/2,27);
    for (std::size_t i=0;i<pixels.size();++i) pixels[i]=static_cast<std::uint8_t>(i%251);
    for (std::size_t i=0;i<depth.size();++i) depth[i]=static_cast<std::uint8_t>((i*13)%256);
    auto l=frame(pixels,640,544),r=l,d=frame(depth,640,544,16),g=frame(high,1280,1088);
    auto out=raw::rgbd(l,r,d,true); auto gdc=raw::gdc(g,g);
    expect(out.left.bytes==pixels && out.disparity.bytes==depth,"Raw bytes changed");
    expect(out.left.encoding=="nv12" && out.left.height==544 && out.left.step==640,"NV12 layout");
    expect(out.disparity.encoding=="16UC1" && out.disparity.step==1280,"Q8 layout");
    expect(out.left.capture.id==0 && out.left.capture.ns==l.TimeStamp,"Capture identity lost");
    expect(gdc.bytes==high && raw::pairOrder(out.left.capture,gdc.capture)==0,"GDC copy/pair");
    pixels.assign(pixels.size(),255);depth.assign(depth.size(),0);
    expect(out.left.bytes[0]==0 && out.disparity.bytes[1]==13,"Owned data changed after vendor release");
    ++r.FrameId; rejects([&]{raw::rgbd(l,r,d,true);},"Right ID mismatch accepted");r=l;
    ++d.TimeStamp; rejects([&]{raw::rgbd(l,r,d,true);},"Q8 timestamp mismatch accepted");--d.TimeStamp;
    d.BitsPerPixel=8;rejects([&]{raw::rgbd(l,r,d,true);},"mono8 accepted as Q8");d.BitsPerPixel=16;
    --d.BytesUsed;rejects([&]{raw::rgbd(l,r,d,true);},"Truncated Q8 accepted");++d.BytesUsed;
    l.ResolutionWidth=320;rejects([&]{raw::rgbd(l,r,d,false);},"Resized RGB accepted");l.ResolutionWidth=640;
    l.TimeStamp=0;rejects([&]{raw::rgbd(l,r,d,false);},"Zero time accepted");l.TimeStamp=r.TimeStamp;
    MoFrame absent{};expect(raw::rgbd(l,r,absent,false).disparity.bytes.empty(),"Infer requires irrelevant Q8");
    rejects([&]{raw::checkStamp(0);},"Zero ROS time accepted");
    rejects([&]{raw::checkStamp(2147483648000000000ULL);},"Overflow ROS time accepted");
    expect(raw::pairOrder({0xffffffffULL,100},{0,200})<0,"ID wrap breaks pairing");
    expect(raw::pairOrder({0,200},{0xffffffffULL,100})>0,"Reverse ID wrap breaks pairing");
    rejects([&]{raw::pairOrder({1,100},{2,100});},"Different IDs at same time accepted");
    rejects([&]{raw::pairOrder({1,100},{1,200});},"Different times at same ID accepted");
    std::cout<<"raw_capture: exact bytes, ownership, identity, Q8 rejection, timestamps and wrap pairing passed\n";
}
