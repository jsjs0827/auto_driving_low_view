#include "compiler_compat.h"
#include "camera_source.hpp"
#include <MoLocalSdkInterface.h>
#include "perception/error.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>
namespace {
int mode=2,close_code=0,open_code=0,query_code=0,closes=0,unloads=0,releases=0;
int rgbd_fault=0;
bool readable_depth=false,mutate_depth_on_release=false;
std::vector<std::uint16_t> rgbd_depth(640*544);
std::vector<std::string> operations;
unsigned char image[24]{};
std::vector<unsigned char> rgbd_image(640*544*3/2);
void require(bool value){if(!value)throw std::runtime_error("camera adapter mock assertion failed");}
void reset(){mode=2;close_code=open_code=query_code=closes=unloads=releases=rgbd_fault=0;readable_depth=mutate_depth_on_release=false;operations.clear();}
}
extern "C" int moSetConfigPath(const char*){return 0;}
extern "C" int moOpenCamera(int* handle){*handle=open_code?-1:7;return open_code;}
extern "C" int moCloseCamera(int){++closes;operations.push_back("close");return close_code;}
extern "C" int moGetDisparityCalculationMode(int,int* result){*result=mode;return query_code;}
extern "C" int moSetDisparityCalculationMode(int,int value){mode=value;operations.push_back("mode"+std::to_string(value));return 0;}
extern "C" int moGetOneGdcFrame(int,MoFrame* left,MoFrame*){
    left->ResolutionWidth=4;left->ResolutionHeight=4;left->BytesUsed=24;left->FrameId=42;left->TimeStamp=12345;left->Data=image;return 0;
}
extern "C" int moReleaseGdcFrame(int,MoFrame*,MoFrame*){++releases;return 0;}
extern "C" int moGetOneRGBDFrame(int,MoFrame* left,MoFrame* right,MoFrame* disparity){
    left->ResolutionWidth=640;left->ResolutionHeight=544;left->BytesUsed=unsigned(rgbd_image.size());
    left->BitsPerPixel=12;left->RGBImageType=7;left->StreamType=16;left->FrameId=53;left->TimeStamp=56789;left->Data=rgbd_image.data();
    *right=*left;*disparity=*left;disparity->BitsPerPixel=16;
    disparity->BytesUsed=640*544*sizeof(std::uint16_t);
    // Deliberately not readable: the audit must never copy or scan disparity.
    disparity->Data=readable_depth?static_cast<void*>(rgbd_depth.data()):reinterpret_cast<void*>(1);
    if(rgbd_fault==1)++right->FrameId;
    if(rgbd_fault==2)++disparity->FrameId;
    if(rgbd_fault==3)++right->TimeStamp;
    if(rgbd_fault==4)++disparity->TimeStamp;
    if(rgbd_fault==5)left->TimeStamp=right->TimeStamp=disparity->TimeStamp=0;
    if(rgbd_fault==6)--right->ResolutionWidth;
    if(rgbd_fault==7)--disparity->ResolutionHeight;
    if(rgbd_fault==8)--right->BytesUsed;
    if(rgbd_fault==9)--disparity->BytesUsed;
    if(rgbd_fault==10)right->Data=nullptr;
    if(rgbd_fault==11)disparity->Data=nullptr;
    if(rgbd_fault==12)disparity->ResolutionWidth=disparity->ResolutionHeight=disparity->BytesUsed=0;
    if(rgbd_fault==13)left->FrameId=right->FrameId=disparity->FrameId=0;
    if(rgbd_fault==14)right->BytesUsed=0;
    if(rgbd_fault==15)disparity->BytesUsed=0;
    if(rgbd_fault==16)disparity->TimeStamp=0;
    if(rgbd_fault==17){disparity->FrameId=disparity->TimeStamp=0;disparity->ResolutionWidth=disparity->ResolutionHeight=0;disparity->BitsPerPixel=disparity->BytesUsed=0;}
    if(rgbd_fault==18)disparity->BitsPerPixel=8;
    return 0;
}
extern "C" int moReleaseRGBDFrame(int,MoFrame*,MoFrame*,MoFrame*){++releases;if(mutate_depth_on_release)std::fill(rgbd_depth.begin(),rgbd_depth.end(),std::uint16_t(65535));return 0;}
void* dlopen(const char*,int){return reinterpret_cast<void*>(1);}
char* dlerror(){return nullptr;}
int dlclose(void*){++unloads;operations.push_back("unload");return 0;}
void* dlsym(void*,const char* name){
    void* result=nullptr;
#define SYMBOL(symbol) if(std::strcmp(name,#symbol)==0){auto function=&symbol;std::memcpy(&result,&function,sizeof result);return result;}
    SYMBOL(moSetConfigPath) SYMBOL(moOpenCamera) SYMBOL(moCloseCamera)
    SYMBOL(moGetDisparityCalculationMode) SYMBOL(moSetDisparityCalculationMode)
    SYMBOL(moGetOneGdcFrame) SYMBOL(moReleaseGdcFrame)
    SYMBOL(moGetOneRGBDFrame) SYMBOL(moReleaseRGBDFrame)
#undef SYMBOL
    return result;
}
int main(){try{
    reset();
    for(std::size_t i=0;i<rgbd_image.size();++i)rgbd_image[i]=static_cast<unsigned char>(i%251);
    {
        demo_camera::Options o;o.stream="rgbd";o.format="nv12";o.disparity_mode=0;
        auto source=demo_camera::open(o);auto frame=source->next();
        require(frame.info.id==53&&frame.info.timestamp_ns==56789&&frame.image->width==640&&frame.image->height==544&&frame.image->stride==640);
        require(frame.image->format==perception::PixelFormat::NV12&&frame.image->bytes==rgbd_image);
        require(frame.sdk_stream_type==16&&frame.sdk_rgb_image_type==7&&frame.sdk_bits_per_pixel==12&&frame.sdk_bytes_used==rgbd_image.size());
        require(frame.sdk_rgbd_triplet_available&&frame.sdk_rgbd_triplet_valid);
        require(!frame.disparity_pixels); // pointer=1 is intentionally never read by default
        require(frame.sdk_color_pair_valid&&frame.sdk_color_pair_issue.empty());
        require(frame.sdk_right_frame_id==53&&frame.sdk_right_timestamp_ns==56789&&frame.sdk_right_width==640&&frame.sdk_right_height==544&&frame.sdk_right_bytes_used==rgbd_image.size());
        require(frame.sdk_disparity_frame_id==53&&frame.sdk_disparity_timestamp_ns==56789&&frame.sdk_disparity_width==640&&frame.sdk_disparity_height==544&&frame.sdk_disparity_bytes_used==640*544*sizeof(std::uint16_t));
        rgbd_image[0]^=255;require(frame.image->bytes[0]!=rgbd_image[0]);
    }
    require(releases==1&&closes==1&&unloads==1&&operations==std::vector<std::string>({"close","unload"}));
    reset();readable_depth=true;mutate_depth_on_release=true;
    for(std::size_t i=0;i<rgbd_depth.size();++i)rgbd_depth[i]=std::uint16_t(i%65535);
    const auto depth_expected=rgbd_depth;
    {demo_camera::Options o;o.stream="rgbd";o.copy_disparity=true;
        auto source=demo_camera::open(o);auto frame=source->next();
        require(frame.disparity_pixels&&*frame.disparity_pixels==depth_expected&&rgbd_depth!=depth_expected);
        require(frame.sdk_rgbd_triplet_valid&&frame.info.id==frame.sdk_disparity_frame_id);
    }
    require(releases==1&&closes==1&&unloads==1&&operations==std::vector<std::string>({"close","unload"}));
    for(int fault=1;fault<=18;++fault){
        reset();readable_depth=true;rgbd_fault=fault;
        demo_camera::Options o;o.stream="rgbd";o.copy_disparity=true;bool rejected=false;
        {auto source=demo_camera::open(o);try{auto frame=source->next();require(fault==13&&frame.info.id==0&&frame.disparity_pixels);}
            catch(const perception::PerceptionException& e){rejected=true;require(e.info().code==perception::ErrorCode::CameraDataInvalid&&std::string(e.what()).find("RGBD_DISPARITY_COPY_METADATA_INVALID")!=std::string::npos);}}
        require(rejected==(fault!=13)&&releases==1&&closes==1&&unloads==1);
    }
    reset();
    {demo_camera::Options o;o.stream="gdc";o.copy_disparity=true;bool rejected=false;try{demo_camera::open(o);}catch(const std::invalid_argument&){rejected=true;}require(rejected&&closes==0&&unloads==0);}
    for(int fault=0;fault<=17;++fault) {
        reset();rgbd_fault=fault;
        demo_camera::Options o;o.stream="rgbd";o.audit_rgbd_triplet=true;
        bool rejected=false;
        const bool unreported=fault==5||fault==12||fault==14||fault==15||fault==16||fault==17;
        {
            auto source=demo_camera::open(o);
            try {auto frame=source->next();require(frame.sdk_rgbd_triplet_valid==!unreported);if(unreported)require(frame.sdk_rgbd_triplet_issue.find("unreported")!=std::string::npos);if(fault==13)require(frame.info.id==0);}
            catch(const std::exception& e){rejected=true;require(std::string(e.what()).find("RGBD_TRIPLET_METADATA_INVALID")!=std::string::npos);}
        }
        require(rejected==(fault>0&&fault<13&&!unreported));
        require(releases==1&&closes==1&&unloads==1); // also release rejected triplets
    }
    reset();rgbd_fault=11;
    {demo_camera::Options o;o.stream="rgbd";auto source=demo_camera::open(o);auto frame=source->next();require(frame.sdk_rgbd_triplet_available&&!frame.sdk_rgbd_triplet_valid&&frame.sdk_color_pair_valid&&frame.image);}
    require(releases==1&&closes==1&&unloads==1); // opt-in preserves legacy acceptance
    reset();
    {demo_camera::Options o;o.stream="gdc";o.audit_rgbd_triplet=true;bool rejected=false;try{demo_camera::open(o);}catch(const std::invalid_argument&){rejected=true;}require(rejected&&closes==0&&unloads==0);}
    for(bool restore:{false,true}) {
        reset();std::vector<std::string> trace;
        demo_camera::Options o;o.stream="gdc";o.format="nv12";o.disparity_mode=1;o.restore_mode_before_close=restore;
        o.lifecycle=[&](const char* name,int,double){trace.emplace_back(name);};
        auto source=demo_camera::open(o);require(source->diagnostics().effective_mode==1);
        auto frame=source->next();require(frame.info.id==42&&frame.info.timestamp_ns==12345&&releases==1&&!frame.sdk_rgbd_triplet_available);
        image[0]=9;require(frame.image->bytes[0]!=9);image[0]=0;source.reset();
        require(closes==1&&unloads==1);
        require(operations==(restore?std::vector<std::string>{"mode1","mode2","close","unload"}:std::vector<std::string>{"mode1","close","unload"}));
        require(trace.back()=="dlclose");
    }
    reset();close_code=-3;
    {demo_camera::Options o;o.stream="gdc";o.disparity_mode=1;auto source=demo_camera::open(o);}
    require(closes==1&&unloads==0); // Preserve SO when close fails.
    reset();open_code=-3;
    bool threw=false;
    try{demo_camera::Options o;o.stream="gdc";demo_camera::open(o);}catch(const std::exception&){threw=true;}
    require(threw&&closes==0&&unloads==1);
    reset();query_code=-1;
    threw=false;
    try{demo_camera::Options o;o.stream="gdc";o.disparity_mode=1;demo_camera::open(o);}catch(const std::exception&){threw=true;}
    require(threw&&closes==1&&unloads==1&&mode==2);
    std::cout<<"Real adapter body with mock loader/vendor functions: RGBD triplet metadata/layout audit without disparity pixel reads, opt-in compatibility, restore, hold, frame ownership, open/query/close failures passed (NOT camera/BPU hardware)\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
