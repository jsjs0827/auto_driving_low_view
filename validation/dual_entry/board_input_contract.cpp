#include "perception_v0_host/input.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

// Synthetic owned descriptors only: no producer, IPC, camera or real SHM.
namespace {
using namespace perception::v0_host;
unsigned checks=0;
void need(bool valid,const char* detail){++checks;if(!valid)throw std::runtime_error(detail);}
Nv12Image image(std::uint32_t width=640,std::uint32_t height=544,std::uint64_t id=41) {
    Nv12Image i;i.width=width;i.height=height;i.stride=width;i.frame_id=id;i.timestamp_ns=id*1000;
    i.bytes.assign(std::size_t(width)*height*3/2,static_cast<std::uint8_t>(id));return i;
}
Q8Disparity disparity(const Nv12Image& image) {
    Q8Disparity q;q.width=image.width;q.height=image.height;q.stride_bytes=image.width*2;
    q.frame_id=image.frame_id;q.timestamp_ns=image.timestamp_ns;
    q.samples.assign(std::size_t(image.width)*image.height,256);return q;
}
CameraParameters calibration() {return {1280,1088,600,620,640,544,60,36000};}
}
int main() try {
    BoardInput input;std::string error;
    need(input.command()==21&&input.generation()==0&&!input.hasPending(),"initial stopped state");
    auto rgbd=image();auto q8=disparity(rgbd);
    need(!input.onRgbd(rgbd,q8,error)&&!input.hasPending(),"stopped mode queued RGBD");
    need(input.setCommand(1)&&input.generation()==1,"mapping boundary missing");
    need(!input.onRgbd(rgbd,q8,error)&&error.find("CameraParam")!=std::string::npos,"missing calibration was not explicit");
    auto bad_calibration=calibration();bad_calibration.bxf=1;
    need(!input.setIntrinsics(bad_calibration,error)&&!error.empty(),"inconsistent bxf accepted");
    bad_calibration=calibration();bad_calibration.fx=std::numeric_limits<double>::quiet_NaN();
    need(!input.setIntrinsics(bad_calibration,error)&&!error.empty(),"nonfinite calibration accepted");
    need(input.setIntrinsics(calibration(),error)&&error.empty(),"valid calibration rejected");
    need(input.onRgbd(rgbd,q8,error)&&input.hasPending(),"valid RGBD/Q8 rejected");
    const auto generation=input.generation();
    need(!input.setCommand(1)&&input.generation()==generation&&input.hasPending(),"same-mode heartbeat cleared pending capture");
    auto item=input.take();need(item&&item->command==1&&item->generation==generation&&!input.hasPending(),"mapping take state");
    need(item->intrinsics.width==640&&item->intrinsics.height==544&&item->intrinsics.fx==300&&
         item->intrinsics.fy==310&&item->intrinsics.cx==320&&item->intrinsics.cy==272&&item->intrinsics.bxf==18000,
         "RGBD calibration did not scale once");
    psdk_frame hires{};psdk_disparity_q8 q{};auto descriptor=item->view(hires,q);
    need(descriptor.left.data==item->left.bytes.data()&&descriptor.disparity&&descriptor.disparity->data==item->disparity->samples.data(),"view did not borrow owned frame bytes");
    need(!descriptor.recharge_hires&&!descriptor.right&&!descriptor.raw_light,"mapping fabricated optional input");
    need(descriptor.left.frame_id==41&&descriptor.left.timestamp_ns==41000&&q.frame_id==41&&q.timestamp_ns==41000,"producer identity was rewritten");
    rgbd.bytes[0]=255;q8.samples[0]=999;
    need(descriptor.left.data[0]==41&&q.data[0]==256,"queued input borrowed producer callback storage");
    auto next=image(640,544,42);need(input.onRgbd(next,disparity(next),error),"next RGBD failed");
    next=image(640,544,43);need(input.onRgbd(next,disparity(next),error)&&input.overwritten()==1,"latest-slot replacement accounting");
    need(descriptor.left.data[0]==41&&q.data[0]==256,"next capture invalidated active owned frame");
    auto newest=input.take();need(newest&&newest->left.frame_id==43&&!input.take(),"latest slot did not return only newest capture");

    auto malformed=image();auto malformed_q=disparity(malformed);malformed_q.frame_id++;
    need(!input.onRgbd(malformed,malformed_q,error)&&error.find("identity")!=std::string::npos,"Q8 frame mismatch accepted");
    malformed_q=disparity(malformed);malformed_q.timestamp_ns++;
    need(!input.onRgbd(malformed,malformed_q,error)&&!error.empty(),"Q8 timestamp mismatch accepted");
    malformed_q=disparity(malformed);malformed_q.samples.pop_back();
    need(!input.onRgbd(malformed,malformed_q,error)&&!error.empty(),"short Q8 accepted");
    malformed_q=disparity(malformed);malformed_q.stride_bytes++;
    need(!input.onRgbd(malformed,malformed_q,error)&&!error.empty(),"odd Q8 stride accepted");
    malformed=image();malformed.bytes.pop_back();
    need(!input.onRgbd(malformed,disparity(malformed),error)&&error.find("payload")!=std::string::npos,"short NV12 accepted");
    malformed=image();malformed.timestamp_ns=0;
    need(!input.onRgbd(malformed,disparity(malformed),error)&&!error.empty(),"missing capture timestamp accepted");
    malformed=image(640,480);need(!input.onRgbd(malformed,disparity(malformed),error)&&error.find("aspect ratio")!=std::string::npos,"incompatible calibration shape accepted");
    need(!input.hasPending(),"rejected input leaked into pending slot");

    rgbd=image();need(input.onRgbd(rgbd,disparity(rgbd),error),"boundary fixture enqueue failed");
    need(!input.setCommand(3)&&input.command()==1&&input.generation()==generation&&input.hasPending(),"unsupported combined command mutated core");
    need(input.setCommand(2)&&input.generation()==generation+1&&!input.hasPending(),"recharge boundary retained old RGBD queue");
    need(!input.onRgbd(rgbd,disparity(rgbd),error)&&!input.hasPending(),"recharge accepted RGBD/Q8 fallback");
    need(!input.onGdc(image(),error)&&error.find("1280x1088")!=std::string::npos,"recharge accepted low-resolution fake GDC");
    auto gdc=image(1280,1088,900);need(input.onGdc(gdc,error),"GDC frame rejected");
    need(!input.setCommand(2)&&input.hasPending(),"recharge heartbeat cleared pending GDC");
    auto charge=input.take();need(charge&&charge->command==2&&!charge->disparity,"GDC inherited RGBD/Q8 capture");
    descriptor=charge->view(hires,q);
    need(descriptor.left.width==1280&&descriptor.left.height==1088&&descriptor.recharge_hires&&
         descriptor.left.data==descriptor.recharge_hires->data&&descriptor.left.bytes==descriptor.recharge_hires->bytes,
         "recharge did not use genuine supplied GDC descriptor for both views");
    need(descriptor.left.frame_id==900&&descriptor.left.timestamp_ns==900000&&hires.frame_id==900&&hires.timestamp_ns==900000,
         "GDC identity was paired or rewritten to RGBD");
    need(descriptor.intrinsics.fx==600&&descriptor.intrinsics.bxf==36000,"GDC calibration scaled incorrectly");
    gdc.bytes[0]=7;need(descriptor.left.data[0]==static_cast<std::uint8_t>(900),"GDC view borrowed callback storage");
    need(input.onGdc(image(1280,1088,901),error),"second GDC enqueue failed");
    need(input.setIntrinsics(calibration(),error)&&input.hasPending(),"same calibration cleared current capture");
    auto changed=calibration();changed.cx+=1;
    need(input.setIntrinsics(changed,error)&&!input.hasPending(),"calibration change left queued old-calibration pixels");
    need(input.onGdc(image(1280,1088,902),error),"third GDC enqueue failed");
    need(input.setCommand(21)&&!input.hasPending()&&input.generation()==generation+2,"stop did not clear pending GDC");
    need(!input.onGdc(image(1280,1088,903),error)&&!input.hasPending(),"stopped mode queued GDC");
    need(input.setCommand(1)&&!input.hasPending(),"restart did not begin empty");
    need(input.onRgbd(image(),disparity(image()),error),"restart RGBD rejected");
    newest=input.take();need(newest&&newest->command==1&&!newest->view(hires,q).recharge_hires,"restart reused recharge mode");
    auto stereo_left=image(640,544,610),stereo_right=stereo_left;
    stereo_right.timestamp_ns++;
    need(!input.onStereo(stereo_left,stereo_right,error)&&!error.empty(),"stereo timestamp mismatch accepted");
    stereo_right=stereo_left;
    need(input.onStereo(stereo_left,stereo_right,error),"valid stereo pair rejected");
    auto stereo=input.take();descriptor=stereo->view(hires,q);
    need(descriptor.right&&descriptor.right->frame_id==610&&descriptor.right->data==stereo->right->bytes.data()&&!descriptor.disparity,
         "stereo descriptors did not retain exact-capture right owner");

    struct RichBuffers {
        std::vector<std::uint8_t> pixels=std::vector<std::uint8_t>(640*544*3/2,33),raw=std::vector<std::uint8_t>(8,9);
        std::vector<std::uint16_t> q8=std::vector<std::uint16_t>(640*544,512);
        psdk_imu_sample imu{};psdk_motion_sample wheel{};psdk_radar_point radar{};
    };
    auto rich_owner=std::make_shared<RichBuffers>();std::weak_ptr<RichBuffers> weak_owner=rich_owner;
    psdk_full_frame rich{};rich.struct_size=sizeof(rich);rich.abi_version=PSDK_FULL_ABI_VERSION;
    rich.left={sizeof(psdk_frame),PSDK_ABI_VERSION,PSDK_FORMAT_NV12,640,544,640,
        rich_owner->pixels.data(),rich_owner->pixels.size(),701,701000};
    rich.intrinsics={640,544,300,310,320,272,18000};
    psdk_frame optional_high=rich.left,optional_right=rich.left,optional_rear=rich.left;
    optional_rear.frame_id=9701;optional_rear.timestamp_ns=9701000;
    rich.recharge_hires=&optional_high;rich.right=&optional_right;rich.rear=&optional_rear;
    psdk_disparity_q8 optional_q{};optional_q.struct_size=sizeof(optional_q);optional_q.abi_version=PSDK_FULL_ABI_VERSION;
    optional_q.width=640;optional_q.height=544;optional_q.stride_bytes=1280;optional_q.data=rich_owner->q8.data();
    optional_q.bytes=rich_owner->q8.size()*2;optional_q.frame_id=701;optional_q.timestamp_ns=701000;rich.disparity=&optional_q;
    psdk_raw_stereo_input optional_raw{};optional_raw.struct_size=sizeof(optional_raw);optional_raw.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    optional_raw.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
    optional_raw.left.struct_size=sizeof(optional_raw.left);optional_raw.left.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    optional_raw.left.data=rich_owner->raw.data();optional_raw.left.bytes=rich_owner->raw.size();
    optional_raw.left.frame_id=7701;optional_raw.left.timestamp_ns=7701000;optional_raw.right=optional_raw.left;
    optional_raw.left_org.struct_size=sizeof(optional_raw.left_org);optional_raw.left_org.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    optional_raw.left_org.present=1;optional_raw.left_org.exposure_ms=5;optional_raw.left_org.analog_gain=2;
    optional_raw.left_org.frame_id=8701;optional_raw.left_org.timestamp_ns=8701000;optional_raw.right_org=optional_raw.left_org;
    rich.raw_light=&optional_raw;
    rich_owner->imu.timestamp_ns=701000;rich_owner->imu.linear_acceleration_mps2[0]=1.25;
    rich_owner->wheel.timestamp_ns=701000;rich_owner->wheel.linear_speed_mps=0.75;
    rich_owner->radar.x_m=2.5;
    psdk_imu_input optional_imu{sizeof(psdk_imu_input),PSDK_SENSOR_INPUT_ABI_VERSION,&rich_owner->imu,1,0};
    psdk_wheel_input optional_wheel{sizeof(psdk_wheel_input),PSDK_SENSOR_INPUT_ABI_VERSION,&rich_owner->wheel,1,0};
    psdk_radar_input optional_radar{};optional_radar.struct_size=sizeof(optional_radar);optional_radar.abi_version=PSDK_SENSOR_INPUT_ABI_VERSION;
    optional_radar.timestamp_ns=701000;optional_radar.points=&rich_owner->radar;optional_radar.point_count=1;
    rich.imu=&optional_imu;rich.wheel=&optional_wheel;rich.radar=&optional_radar;
    need(!input.onFullFrame(rich,{},error)&&error.find("owner")!=std::string::npos,"rich input accepted unowned pointers");
    need(input.onFullFrame(rich,rich_owner,error),"rich owned frame rejected");
    rich_owner.reset();need(!weak_owner.expired(),"queue did not retain rich buffer owner");
    optional_high.frame_id=0;optional_right.frame_id=0;optional_rear.frame_id=0;optional_q.frame_id=0;
    optional_raw.left_org.exposure_ms=0;optional_imu.count=0;optional_wheel.count=0;optional_radar.point_count=0;
    auto rich_pending=input.take();need(rich_pending&&rich_pending->owner&&!weak_owner.expired(),"take lost rich lease");
    auto moved_rich=std::move(*rich_pending);rich_pending.reset();descriptor=moved_rich.view(hires,q);
    need(descriptor.right&&descriptor.right->frame_id==701&&descriptor.rear&&descriptor.rear->frame_id==9701&&
         descriptor.recharge_hires&&descriptor.recharge_hires->frame_id==701&&descriptor.disparity&&descriptor.disparity->frame_id==701,
         "rich descriptor wrappers were not copied/rebound after queue move");
    need(descriptor.left.data[0]==33&&descriptor.disparity->data[0]==512&&descriptor.raw_light&&descriptor.raw_light->left.data[0]==9,
         "rich buffer values changed after producer owner reset");
    need(descriptor.raw_light->left.frame_id==7701&&descriptor.raw_light->left.timestamp_ns==7701000&&
         descriptor.raw_light->left_org.frame_id==8701&&descriptor.raw_light->left_org.timestamp_ns==8701000&&
         descriptor.raw_light->left_org.exposure_ms==5&&descriptor.raw_light->left_org.analog_gain==2,
         "rich RAW/ORG provenance or exposure units were rewritten");
    need(descriptor.imu&&descriptor.imu->count==1&&descriptor.imu->samples[0].linear_acceleration_mps2[0]==1.25&&
         descriptor.wheel&&descriptor.wheel->count==1&&descriptor.wheel->samples[0].linear_speed_mps==0.75&&
         descriptor.radar&&descriptor.radar->point_count==1&&descriptor.radar->points[0].x_m==2.5,
         "optional sensor wrappers did not survive queue move");
    moved_rich=InputFrame{};need(weak_owner.expired(),"rich frame leaked buffer lease after release");
    std::cout<<"PASS "<<checks<<" V0 input core checks; synthetic owned descriptors only, no real SHM, camera, algorithms or model execution\n";
    return 0;
} catch(const std::exception& exception){std::cerr<<exception.what()<<'\n';return 1;}
