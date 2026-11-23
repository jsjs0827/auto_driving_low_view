#include "perception/frame_input.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <vector>
extern "C" int raw_input_header_is_c(void);
namespace {
int checks=0;
void require(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
void rejects(const std::function<void()>& fn){++checks;try{fn();}catch(const std::invalid_argument&){return;}throw std::runtime_error("Invalid RAW descriptor was accepted");}
}
int main(){try{
    using perception::input::FrameBundle;
    require(raw_input_header_is_c(),"Public RAW input header is not C compatible");
    std::vector<std::uint8_t> yuv(12,128),left_raw(32,0),right_raw(32,0);
    psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    frame.left={};frame.left.struct_size=sizeof(frame.left);frame.left.abi_version=PSDK_ABI_VERSION;
    frame.left.data=yuv.data();frame.left.bytes=yuv.size();frame.left.width=4;frame.left.height=2;frame.left.stride=4;
    frame.left.format=PSDK_FORMAT_NV12;frame.left.frame_id=7;frame.left.timestamp_ns=1000;
    auto right=frame.left;frame.right=&right;
    psdk_raw_stereo_input input{};input.struct_size=sizeof(input);input.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    input.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
    auto& raw=input.left;raw.struct_size=sizeof(raw);raw.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    raw.width=4;raw.height=4;raw.stride_bytes=8;raw.format=PSDK_RAW_FORMAT_UINT16_LE;raw.valid_bits=10;raw.bayer=PSDK_RAW_BAYER_BGGR;
    raw.data=left_raw.data();raw.bytes=left_raw.size();raw.frame_id=7;raw.timestamp_ns=1000;input.right=raw;input.right.data=right_raw.data();
    auto& org=input.left_org;org.struct_size=sizeof(org);org.abi_version=PSDK_RAW_INPUT_ABI_VERSION;org.present=1;
    org.exposure_ms=10;org.analog_gain=8;org.frame_id=7;org.timestamp_ns=1000;input.right_org=org;
    frame.raw_light=&input;auto b=FrameBundle::borrowed(frame);perception::runtime::ArtifactStore artifacts;b.seed(artifacts);
    require(b.raw_light.has_value(),"RAW descriptor was lost");
    const auto copied=artifacts.get<psdk_raw_stereo_input>("input.raw_light");
    require(copied->left.data==left_raw.data()&&copied->right.data==right_raw.data(),"RAW image pixels were copied/replaced");
    require(copied->left_org.exposure_ms==10&&copied->right_org.analog_gain==8,"ORG source units changed");
    input.left_org.exposure_ms=12;
    require(copied->left_org.exposure_ms==10,"Artifact descriptor was not snapshotted");input.left_org.exposure_ms=10;
    frame.struct_size=PSDK_FULL_FRAME_V3_SIZE;frame.raw_light=reinterpret_cast<const psdk_raw_stereo_input*>(1);
    auto old_sensor=FrameBundle::borrowed(frame);require(old_sensor.right&&!old_sensor.raw_light,"Old sensor ABI inspected new RAW tail");
    frame.struct_size=PSDK_FULL_FRAME_V2_SIZE;frame.rear=reinterpret_cast<const psdk_frame*>(1);
    auto old_right=FrameBundle::borrowed(frame);require(old_right.right&&!old_right.rear&&!old_right.raw_light,"Old right ABI inspected optional tail");
    frame.struct_size=PSDK_FULL_FRAME_V1_SIZE;frame.right=reinterpret_cast<const psdk_frame*>(1);
    require(!FrameBundle::borrowed(frame).right,"Original ABI inspected right tail");
    frame.struct_size=sizeof(frame);frame.right=&right;frame.rear=nullptr;frame.raw_light=&input;
    input.struct_size--;rejects([&]{FrameBundle::borrowed(frame);});input.struct_size++;
    input.abi_version++;rejects([&]{FrameBundle::borrowed(frame);});input.abi_version--;
    input.reserved=1;rejects([&]{FrameBundle::borrowed(frame);});input.reserved=0;
    input.metadata_source=99;rejects([&]{FrameBundle::borrowed(frame);});input.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
    raw.bytes=1;rejects([&]{FrameBundle::borrowed(frame);});raw.bytes=left_raw.size();
    raw.data=nullptr;rejects([&]{FrameBundle::borrowed(frame);});raw.data=left_raw.data();
    raw.stride_bytes=7;rejects([&]{FrameBundle::borrowed(frame);});raw.stride_bytes=8;
    raw.stride_bytes=2;rejects([&]{FrameBundle::borrowed(frame);});raw.stride_bytes=8;
    raw.struct_size=0;rejects([&]{FrameBundle::borrowed(frame);});raw.struct_size=sizeof(raw);
    raw.format=99;rejects([&]{FrameBundle::borrowed(frame);});raw.format=PSDK_RAW_FORMAT_UINT16_LE;
    input.left_org.present=2;rejects([&]{FrameBundle::borrowed(frame);});input.left_org.present=1;
    input.left_org.reserved=1;rejects([&]{FrameBundle::borrowed(frame);});input.left_org.reserved=0;
    input.left_org.exposure_ms=std::numeric_limits<double>::quiet_NaN();
    require(FrameBundle::borrowed(frame).raw_light.has_value(),"Model-specific bad exposure aborted unrelated DAG inputs");
    input.left_org={};input.right_org={};input.left={};input.right={};
    require(FrameBundle::borrowed(frame).raw_light.has_value(),"Missing RAW/ORG cannot be represented");
    frame.raw_light=nullptr;auto absent=FrameBundle::borrowed(frame);perception::runtime::ArtifactStore empty;absent.seed(empty);
    require(!empty.contains("input.raw_light"),"Absent RAW input was fabricated");
    std::cout<<"PASS "<<checks<<" RAW input checks; C header, additive ABI, borrowed storage and isolation passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
