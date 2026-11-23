#include "visual_motion.hpp"
#include "motion_reference.hpp"
#include "slip_detection.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace perception::motion;
namespace {
void need(bool x,const char*message){if(!x)throw std::runtime_error(message);}
void near(double x,double wanted,double tolerance,const char*message){if(!std::isfinite(x)||std::abs(x-wanted)>tolerance){std::cerr<<message<<" actual="<<x<<" expected="<<wanted<<'\n';throw std::runtime_error(message);}}
using V=std::array<double,3>;using M=std::array<double,9>;
V mv(const M&r,const V&p){return {{r[0]*p[0]+r[1]*p[1]+r[2]*p[2],r[3]*p[0]+r[4]*p[1]+r[5]*p[2],r[6]*p[0]+r[7]*p[1]+r[8]*p[2]}};}
M trans(const M&r){return {{r[0],r[3],r[6],r[1],r[4],r[7],r[2],r[5],r[8]}};}
M mm(const M&a,const M&b){M c{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)c[i*3+j]+=a[i*3+k]*b[k*3+j];return c;}
M ry(double a){return {{std::cos(a),0,std::sin(a),0,1,0,-std::sin(a),0,std::cos(a)}};}
M rz(double a){return {{std::cos(a),-std::sin(a),0,std::sin(a),std::cos(a),0,0,0,1}};}
unsigned hash(int x,int y){std::uint32_t v=std::uint32_t(x)*0x45d9f3bu^std::uint32_t(y)*0x119de1f3u;v^=v>>16;v*=0x45d9f3bu;v^=v>>16;return v&255;}
double texture(double x,double y){x=x*35+1000;y=y*35+1000;const int ix=int(std::floor(x)),iy=int(std::floor(y));const double a=x-ix,b=y-iy;return (hash(ix,iy)*(1-a)+hash(ix+1,iy)*a)*(1-b)+(hash(ix,iy+1)*(1-a)+hash(ix+1,iy+1)*a)*b;}
// Render a physical textured plane at world Z=4 m through a moving calibrated
// camera. Both luminance and Q8 come from ray/plane geometry, not pixel shifts
// or precomputed result transforms. The algorithm must discover correspondences.
std::shared_ptr<const r36::temporal::Frame> render(std::uint64_t id,std::uint64_t stamp,V camera_center={{0,0,0}},M camera_to_world={{1,0,0,0,1,0,0,0,1}},bool weak=false,bool outliers=false){
    auto f=std::make_shared<r36::temporal::Frame>();constexpr int w=240,h=180;const double fx=180,fy=180,cx=119.5,cy=89.5,bxf=18000;
    f->sequence=id;f->intrinsics={w,h,fx,fy,cx,cy,bxf};
    f->left.metadata={sizeof(psdk_frame),PSDK_ABI_VERSION,PSDK_FORMAT_NV12,w,h,w,nullptr,w*h*3/2,id,stamp};
    auto pixels=std::make_shared<std::vector<std::uint8_t>>(w*h*3/2,std::uint8_t(128));auto q=std::make_shared<std::vector<std::uint16_t>>(w*h);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){const auto ray=mv(camera_to_world,{{(x-cx)/fx,(y-cy)/fy,1}});const double z=(4-camera_center[2])/ray[2];
        const double wx=camera_center[0]+ray[0]*z,wy=camera_center[1]+ray[1]*z;
        (*pixels)[std::size_t(y)*w+x]=weak?128:std::uint8_t(std::clamp(std::round(texture(wx,wy)),0.,255.));
        double disparity=bxf/(z*1000);if(outliers&&x>w*2/3)disparity*=1.5;
        (*q)[std::size_t(y)*w+x]=std::uint16_t(std::round(disparity*256));
    }
    f->left.storage=pixels;f->disparity=q;return f;
}
r36::temporal::Window window(std::shared_ptr<const r36::temporal::Frame>a,std::shared_ptr<const r36::temporal::Frame>b){r36::temporal::Window w;w.frames={a,b};w.sampled=true;w.capacity=2;return w;}
VisualSettings visualSettings(){VisualSettings s;s.extrinsics.confirmed=true;s.grid_step=8;s.max_features=220;s.min_texture=7;s.max_patch_error=28;s.match_ratio=.9;s.inlier_threshold_m=.05;s.max_residual_m=.035;s.min_inlier_ratio=.45;return s;}
void visualTests(){
    const auto settings=visualSettings();VisualMotion visual(settings);const auto first=render(1,1000000000);
    auto stationary=visual.estimate(window(first,render(2,1100000000)));need(stationary.available,"stationary textured scene available");near(stationary.forward_m,0,.002,"stationary translation");near(stationary.yaw_rad,0,.002,"stationary rotation");
    auto forward=visual.estimate(window(first,render(2,1100000000,{{0,0,.08}})));std::cout<<"forward available="<<forward.available<<" reason="<<forward.reason<<" estimate="<<forward.forward_m<<" match="<<forward.matches<<" inliers="<<forward.inliers<<" rmse="<<forward.residual_m<<'\n';
    need(forward.available,"physical forward motion available");near(forward.forward_m,.08,.025,"physical forward distance");near(forward.lateral_m,0,.02,"forward lateral drift");
    auto lateral=visual.estimate(window(first,render(2,1100000000,{{.08,0,0}})));need(lateral.available,"physical lateral translation available");near(lateral.lateral_m,-.08,.025,"camera-right is body-right");
    auto rotated=visual.estimate(window(first,render(2,1100000000,{{0,0,0}},ry(.02))));std::cout<<"rotation available="<<rotated.available<<" yaw="<<rotated.yaw_rad<<" forward="<<rotated.forward_m<<'\n';need(rotated.available,"physical rotation available");near(rotated.yaw_rad,-.02,.012,"camera/body rotation sign");
    auto robust=visual.estimate(window(first,render(2,1100000000,{{0,0,.08}},{{1,0,0,0,1,0,0,0,1}},false,true)));need(robust.available,"depth outliers handled by robust 3D fit");near(robust.forward_m,.08,.03,"robust forward estimate");need(robust.inliers<robust.matches,"depth outliers excluded");
    auto weak=visual.estimate(window(render(1,1000000000,{{0,0,0}},{{1,0,0,0,1,0,0,0,1}},true),render(2,1100000000,{{0,0,.08}},{{1,0,0,0,1,0,0,0,1}},true)));need(!weak.available&&weak.reason=="insufficient_textured_features","weak texture unavailable");
    auto gap=visual.estimate(window(first,render(2,1400000000)));need(!gap.available&&gap.reason=="capture_time_gap","large capture gap unavailable");
    auto invalid=std::make_shared<r36::temporal::Frame>(*render(2,1100000000));invalid->disparity=std::make_shared<const std::vector<std::uint16_t>>(240*180,std::uint16_t(0));need(!visual.estimate(window(first,invalid)).available,"zero disparity cannot become motion");
    auto invalidK=std::make_shared<r36::temporal::Frame>(*render(2,1100000000));invalidK->intrinsics.fx+=1;need(!visual.estimate(window(first,invalidK)).available,"changed calibration unavailable");
    auto line0=std::make_shared<r36::temporal::Frame>(*first),line1=std::make_shared<r36::temporal::Frame>(*render(2,1100000000));auto lineQ=std::make_shared<std::vector<std::uint16_t>>(240*180,std::uint16_t(0));for(int y=90;y<=91;++y)for(int x=0;x<240;++x)(*lineQ)[y*240+x]=1152;line0->disparity=lineQ;line1->disparity=lineQ;auto line=visual.estimate(window(line0,line1));need(!line.available,"collinear 3D support cannot produce a rigid pose");
    auto mask0=std::make_shared<r36::temporal::Frame>(*first),mask1=std::make_shared<r36::temporal::Frame>(*render(2,1100000000));auto maskedQ=std::make_shared<std::vector<std::uint16_t>>(*first->disparity);std::fill(maskedQ->begin(),maskedQ->begin()+240*90,std::uint16_t(0));mask0->disparity=maskedQ;mask1->disparity=maskedQ;auto bottom=settings;bottom.roi={0,90,240,90};need(VisualMotion(bottom).estimate(window(mask0,mask1)).available,"source ROI and external invalid Q8 honored");
    auto roiSettings=settings;roiSettings.roi={0,0,241,180};need(!VisualMotion(roiSettings).estimate(window(first,render(2,1100000000))).available,"ROI bounds checked");
    auto rod=settings;rod.extrinsics.translation_m={{.4,.1,0}};const double angle=.02;const auto rbody=rz(angle),rcamera=mm(mm(trans(rod.extrinsics.rotation),rbody),rod.extrinsics.rotation);
    auto rotated_p=mv(rbody,rod.extrinsics.translation_m);V delta{{rotated_p[0]-.4,rotated_p[1]-.1,rotated_p[2]}};const auto center=mv(trans(rod.extrinsics.rotation),delta);
    auto lever=VisualMotion(rod).estimate(window(first,render(2,1100000000,center,rcamera)));need(lever.available,"lever arm rotation available");near(lever.forward_m,0,.025,"lever arm compensated forward");near(lever.lateral_m,0,.025,"lever arm compensated lateral");near(lever.yaw_rad,angle,.012,"lever arm body yaw");
    auto notConfirmed=settings;notConfirmed.extrinsics.confirmed=false;auto diagnostic=VisualMotion(notConfirmed).estimate(window(first,render(2,1100000000)));need(diagnostic.available&&!diagnostic.body_confirmed,"unconfirmed extrinsics only diagnostic motion");
}
void referenceTests(){
    ReferenceSettings s;s.max_sample_gap_ms=110;MotionBuffer b(s);need(b.push({1000000000,1,.2}),"reference push");need(b.push({1100000000,1,.2}),"reference push2");need(b.push({1200000000,1,.2}),"reference push3");
    auto r=b.integrate(1025000000,1175000000);need(r.available,"bracketed interpolation");near(r.distance_m,.15,1e-10,"exact distance integral");near(r.yaw_rad,.03,1e-10,"exact yaw integral");near(r.forward_m,std::sin(.03)/.2,1e-9,"SE2 forward integral");near(r.lateral_m,(1-std::cos(.03))/.2,1e-9,"SE2 lateral integral");
    need(!b.integrate(999000000,1100000000).available,"no extrapolated boundary");need(!b.integrate(1000000000,1201000000).available,"no future extrapolation");need(!b.push({1200000000,1,0}),"duplicate reference rejected");need(!b.push({1300000000,std::numeric_limits<double>::quiet_NaN(),0}),"nonfinite sample rejected");need(b.size()==0&&!b.integrate(1100000000,1200000000).available,"invalid sample breaks interpolation continuity");
    MotionBuffer gap;gap.push({1000000000,1,0});gap.push({1300000000,1,0});need(!gap.integrate(1100000000,1200000000).available,"internal reference gap rejected");
    MotionBuffer linear(s);linear.push({1000000000,0,0});linear.push({1100000000,2,0});auto integrated=linear.integrate(1025000000,1075000000);need(integrated.available,"linear partial interval");near(integrated.distance_m,.05,1e-10,"linear interpolation integration");
    ReferenceSettings bounded=s;bounded.max_samples=2;MotionBuffer small(bounded);small.push({1000000000,1,0});small.push({1100000000,1,0});small.push({1200000000,1,0});need(small.size()==2&&!small.integrate(1000000000,1100000000).available,"bounded reference eviction");
}
void slipTests(){
    SlipSettings s;s.enter_duration_ms=200;s.exit_duration_ms=200;SlipDetector detector(s);
    VisualResult v;v.available=true;v.body_confirmed=true;v.quality=1;v.forward_m=.02;v.t0_ns=1000000000;v.t1_ns=1100000000;
    ReferenceResult r;r.available=true;r.forward_m=.1;r.t0_ns=v.t0_ns;r.t1_ns=v.t1_ns;
    auto first=detector.update(v,r);need(first.available&&first.state==SlipState::Suspected,"real-time slip enter pending");
    auto candidate=detector;v.t0_ns+=100000000;v.t1_ns+=100000000;r.t0_ns=v.t0_ns;r.t1_ns=v.t1_ns;need(candidate.update(v,r).state==SlipState::Slipping,"candidate enter after duration");
    need(detector.update(v,r).state==SlipState::Slipping,"copy updates are isolated until commit");
    v.forward_m=.09;v.t0_ns+=100000000;v.t1_ns+=100000000;r.t0_ns=v.t0_ns;r.t1_ns=v.t1_ns;need(detector.update(v,r).state==SlipState::Recovering,"exit pending");
    v.t0_ns+=100000000;v.t1_ns+=100000000;r.t0_ns=v.t0_ns;r.t1_ns=v.t1_ns;need(detector.update(v,r).state==SlipState::Normal,"exit after real duration");
    r.available=false;need(!detector.update(v,r).available,"missing reference not normal");r.available=true;v.body_confirmed=false;need(!detector.update(v,r).available,"unconfirmed extrinsics cannot decide");v.body_confirmed=true;v.yaw_rad=.2;need(!detector.update(v,r).available,"large turn cannot decide");v.yaw_rad=0;v.quality=.1;need(!detector.update(v,r).available,"low visual quality cannot decide");v.quality=1;r.forward_m=.001;need(!detector.update(v,r).available,"low speed cannot decide");
    r.forward_m=-.1;v.forward_m=-.02;need(detector.update(v,r).state==SlipState::Suspected,"reverse wheel spin supported");
    need(!detector.update(v,r).available,"repeated window cannot accumulate duration");
    SlipDetector gapDetector(s);v.t0_ns=1000000000;v.t1_ns=1100000000;r.t0_ns=v.t0_ns;r.t1_ns=v.t1_ns;need(gapDetector.update(v,r).state==SlipState::Suspected,"first independent interval");v.t0_ns=1200000000;v.t1_ns=1300000000;r.t0_ns=v.t0_ns;r.t1_ns=v.t1_ns;need(gapDetector.update(v,r).state==SlipState::Suspected,"discontinuous intervals do not accumulate enter duration");
}
}
int main(){try{visualTests();referenceTests();slipTests();std::cout<<"Motion algorithms: geometry/robustness/quality/reference/hysteresis PASS\n";return 0;}catch(const std::exception&e){std::cerr<<"Motion algorithms FAIL: "<<e.what()<<'\n';return 1;}}
