#include "compiler_compat.h"
#include "camera_source.hpp"
#include <MoLocalSdkInterface.h>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
int mode=2,close_code=0,open_code=0,query_code=0,restore_code=0;
int closes=0,unloads=0,releases=0,callback_calls=0,null_callbacks=0;
int register_code=0,replace_code=0,calculate_code=0,calculated_id=-1;
int metric_code=0,metric_calls=0;
bool callback_export=true;
StereoVisionStatusCallback callback=nullptr;
std::vector<std::string> operations;
struct Call {std::string name;int code;};
unsigned char image[24]{};
void require(bool value,const char* reason="camera callback mock assertion failed") {
    if(!value)throw std::runtime_error(reason);
}
void reset(){
    mode=2;close_code=open_code=query_code=restore_code=closes=unloads=releases=0;
    callback_calls=null_callbacks=register_code=replace_code=calculate_code=0;
    calculated_id=-1;callback=nullptr;callback_export=true;operations.clear();
    metric_code=metric_calls=0;
}
demo_camera::Options demandOptions(std::vector<Call>& trace,bool restore=true) {
    demo_camera::Options o;o.stream="gdc";o.format="nv12";o.disparity_mode=1;
    o.restore_mode_before_close=restore;o.demand_enabled=true;o.stereo_trace=true;
    o.lifecycle=[&trace](const char* name,int code,double){trace.push_back({name,code});};
    return o;
}
bool observed(const std::vector<Call>& trace,const char* name,int code) {
    for(const auto& call:trace)if(call.name==name&&call.code==code)return true;
    return false;
}
}
extern "C" int moSetConfigPath(const char*){return 0;}
extern "C" int moOpenCamera(int* handle){*handle=open_code?-1:7;return open_code;}
extern "C" int moCloseCamera(int){
    ++closes;operations.push_back("close");
    if(callback){callback(1);callback(0);} // SDK may finish work during close
    return close_code;
}
extern "C" int moGetDisparityCalculationMode(int,int* result){*result=mode;return query_code;}
extern "C" int moSetDisparityCalculationMode(int,int value){
    operations.push_back("mode"+std::to_string(value));
    if(value==2&&restore_code)return restore_code;
    mode=value;return 0;
}
extern "C" int moGetBpuRatio(int,std::int32_t* value){++metric_calls;*value=73;return metric_code;}
extern "C" int moGetFrameRateOrg(int,double* value){++metric_calls;*value=33.33;return metric_code;}
extern "C" int moGetFrameRateDisp(int,double* value){++metric_calls;*value=32.10;return metric_code;}
extern "C" int moGetStereoVisionStatusCallback(int,StereoVisionStatusCallback next){
    ++callback_calls;
    // Actual 1.4.4.0 binary behavior, contrary to the header's NULL comment.
    if(!next){++null_callbacks;operations.push_back("callback_null");return -2;}
    const bool registration=callback_calls==1;
    operations.push_back(registration?"callback_register":"callback_replace_noop");
    const int code=registration?register_code:replace_code;
    if(code)return code;
    if(callback){callback(1);callback(0);} // old callback can be in flight
    callback=next;callback(1);callback(0);return 0;
}
extern "C" int moGetCameraParams(int,MoMonoCameraParams*,double* bxf,double* baseline,int* w,int* h,int* maximum){
    *bxf=1;*baseline=1;*w=4;*h=4;*maximum=256;return 0;
}
extern "C" int moCalculateDisparityByFrameId(int,int id,std::uint16_t* output,int* w,int* h){
    calculated_id=id;*w=4;*h=4;
    // The inspected direct-demand API does not invoke the continuous callback.
    for(int i=0;i<16;++i)output[i]=std::uint16_t(id+i);
    return calculate_code;
}
extern "C" int moGetOneGdcFrame(int,MoFrame* left,MoFrame*){
    left->ResolutionWidth=4;left->ResolutionHeight=4;left->BytesUsed=24;
    left->FrameId=42;left->TimeStamp=12345;left->Data=image;return 0;
}
extern "C" int moReleaseGdcFrame(int,MoFrame*,MoFrame*){++releases;return 0;}
extern "C" int moGetOneRGBDFrame(int,MoFrame* left,MoFrame* right,MoFrame* disparity){
    left->ResolutionWidth=4;left->ResolutionHeight=4;left->BytesUsed=24;
    left->FrameId=42;left->TimeStamp=12345;left->Data=image;*right=*left;
    *disparity=MoFrame{};return 0; // mode 1: real color pair, no finished depth metadata
}
extern "C" int moReleaseRGBDFrame(int,MoFrame*,MoFrame*,MoFrame*){++releases;return 0;}
void* dlopen(const char*,int){return reinterpret_cast<void*>(1);}
char* dlerror(){return nullptr;}
int dlclose(void*){++unloads;operations.push_back("unload");return 0;}
void* dlsym(void*,const char* name){
    void* result=nullptr;
    if(!callback_export&&std::strcmp(name,"moGetStereoVisionStatusCallback")==0)return nullptr;
#define SYMBOL(symbol) if(std::strcmp(name,#symbol)==0){auto function=&symbol;std::memcpy(&result,&function,sizeof result);return result;}
    SYMBOL(moSetConfigPath) SYMBOL(moOpenCamera) SYMBOL(moCloseCamera)
    SYMBOL(moGetDisparityCalculationMode) SYMBOL(moSetDisparityCalculationMode)
    SYMBOL(moGetOneGdcFrame) SYMBOL(moReleaseGdcFrame)
    SYMBOL(moGetOneRGBDFrame) SYMBOL(moReleaseRGBDFrame)
    SYMBOL(moGetStereoVisionStatusCallback) SYMBOL(moGetCameraParams) SYMBOL(moCalculateDisparityByFrameId)
    SYMBOL(moGetBpuRatio) SYMBOL(moGetFrameRateOrg) SYMBOL(moGetFrameRateDisp)
#undef SYMBOL
    return result;
}
int main(){try{
    for(bool restore:{false,true})for(int cycle=0;cycle<3;++cycle) {
        reset();std::vector<Call> trace;auto source=demo_camera::open(demandOptions(trace,restore));
        require(source->diagnostics().callback_available);
        require(source->diagnostics().mode_available&&metric_calls==0);
        mode=2;auto sampled=source->diagnostics(true);
        require(sampled.mode_available&&sampled.effective_mode==2&&sampled.ratio_available&&sampled.bpu_ratio==73);
        require(sampled.original_fps_available&&sampled.disparity_fps_available&&sampled.original_fps==33.33&&sampled.disparity_fps==32.10&&metric_calls==3);
        query_code=metric_code=-7;sampled=source->diagnostics(true);
        require(!sampled.mode_available&&sampled.effective_mode==-1&&!sampled.ratio_available&&sampled.bpu_ratio==-1&&!sampled.original_fps_available&&sampled.original_fps==-1&&!sampled.disparity_fps_available&&sampled.disparity_fps==-1);
        query_code=metric_code=0;mode=1;
        auto recording=callback;require(recording!=nullptr);
        auto frame=source->next();auto depth=source->calculateDisparity(frame.info.id);
        require(depth.code==0&&depth.frame_id==42&&calculated_id==42,"public frame ID must not be multiplied");
        require(depth.width==4&&depth.height==4&&depth.pixels.size()==16&&depth.pixels[0]==42);
        require(depth.call_begin_host_ns>0&&depth.call_end_host_ns>=depth.call_begin_host_ns);
        require(source->stereoEvents().size()==2&&source->diagnostics().stereo_starts==1);
        calculate_code=-9;require(source->calculateDisparity(43).pixels.empty());
        bool threw=false;try{source->calculateDisparity(std::uint64_t(std::numeric_limits<int>::max())/3+1);}catch(const std::invalid_argument&){threw=true;}
        require(threw&&calculated_id==43);
        source.reset();
        require(null_callbacks==0,"regression: SDK rejects NULL cancellation with -2");
        require(callback_calls==2&&callback!=recording&&closes==1&&unloads==1);
        require(observed(trace,"callback_register",0)&&observed(trace,"callback_replace_noop",0));
        const std::vector<std::string> expected=restore?
            std::vector<std::string>{"mode1","callback_register","callback_replace_noop","mode2","close","unload"}:
            std::vector<std::string>{"mode1","callback_register","callback_replace_noop","close","unload"};
        require(operations==expected,"replacement must precede restoration and close");
        callback(1);callback(0);recording(1);recording(0); // no Source retained
    }
    for(bool demand:{false,true}) {
        reset();std::vector<Call> trace;auto o=demandOptions(trace);
        o.stream="rgbd";o.allow_rgbd_mode1_probe=true;o.audit_rgbd_triplet=true;o.demand_enabled=demand;
        auto source=demo_camera::open(o);auto frame=source->next();
        require(frame.sdk_color_pair_valid&&!frame.sdk_rgbd_triplet_valid&&frame.sdk_rgbd_triplet_issue.find("disparity_metadata_unreported")!=std::string::npos);
        require(frame.info.id==42&&frame.image->width==4&&frame.image->height==4&&source->diagnostics().effective_mode==1);
        if(demand){auto depth=source->calculateDisparity(frame.info.id);require(depth.code==0&&calculated_id==42&&depth.pixels.size()==16);}
        else {bool threw=false;try{source->calculateDisparity(frame.info.id);}catch(const std::runtime_error&){threw=true;}require(threw&&calculated_id==-1);}
        source.reset();require(releases==1&&closes==1&&unloads==1);
    }
    for(int invalid=0;invalid<3;++invalid) {
        reset();demo_camera::Options o;o.stream="rgbd";o.disparity_mode=1;
        if(invalid){o.allow_rgbd_mode1_probe=true;if(invalid==1)o.stream="gdc";else o.disparity_mode=2;}
        bool threw=false;try{demo_camera::open(o);}catch(const std::invalid_argument&){threw=true;}
        require(threw&&closes==0&&unloads==0);
    }
    for(int failure:{1,2,3}) {
        reset();std::vector<Call> trace;
        auto source=demo_camera::open(demandOptions(trace));
        if(failure==1)replace_code=-7;
        if(failure==2)restore_code=-8;
        if(failure==3)close_code=-3;
        source.reset();require(null_callbacks==0&&closes==1);
        require(unloads==(failure==3?0:1),"failed close must retain the SO");
        require(observed(trace,failure==1?"callback_replace_noop":failure==2?"moSetDisparityCalculationMode_restore":"moCloseCamera",failure==1?-7:failure==2?-8:-3));
        if(failure==3)require(observed(trace,"sdk_retained_after_failed_close",-1));
        callback(1);callback(0); // safe after close failure or replacement failure
    }
    reset();register_code=-4;std::vector<Call> trace;
    {auto source=demo_camera::open(demandOptions(trace));require(!source->diagnostics().callback_available);require(source->diagnostics().stereo_starts==0&&source->stereoEvents().empty());}
    require(observed(trace,"callback_register",-4)&&callback_calls==1&&closes==1&&unloads==1);
    reset();callback_export=false;trace.clear();
    {auto source=demo_camera::open(demandOptions(trace));require(!source->diagnostics().callback_available);require(source->diagnostics().stereo_starts==0&&source->stereoEvents().empty());require(source->calculateDisparity(42).code==0);}
    require(callback_calls==0&&closes==1&&unloads==1);
    std::cout<<"Real adapter with vendor/loader mocks: NULL->-2 regression, no-op replacement, demand ID/ownership, reopen, restore/hold and genuine failures passed (NOT camera/BPU hardware)\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
