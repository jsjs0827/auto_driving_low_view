#include "perception_ros_so_demo/stereo_inputs.hpp"
#include "perception_ros_so_demo/stereo_provider.hpp"
#include "compare_util.hpp"
#include <functional>
#include <iostream>
#include <vector>

namespace {
void require(bool value,const char* detail){if(!value)throw std::runtime_error(detail);}
void rejects(const std::function<void()>& operation) {
    try{operation();}catch(const std::invalid_argument&){return;}
    throw std::runtime_error("Expected malformed stereo source rejection");
}
psdk_frame view(const std::vector<std::uint8_t>& bytes,unsigned format,unsigned width,unsigned height,unsigned stride) {
    psdk_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_ABI_VERSION;
    frame.format=format;frame.width=width;frame.height=height;frame.stride=stride;
    frame.data=bytes.data();frame.bytes=bytes.size();frame.frame_id=42;frame.timestamp_ns=123456789;
    return frame;
}
}
int main() {
    namespace inputs=ros_so_demo::stereo_inputs;
    const unsigned width=6,height=4,stride=12;
    cv::Mat source(height,width,CV_8UC3);
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x)
        source.at<cv::Vec3b>(int(y),int(x))=cv::Vec3b(25+x*11,50+y*23,190-x*17);
    cv::Mat i420;cv::cvtColor(source,i420,cv::COLOR_BGR2YUV_I420);
    const auto* flat=i420.ptr<std::uint8_t>();
    const unsigned y_bytes=width*height,chroma_bytes=width*height/4;
    for(const auto format:{PSDK_FORMAT_NV12,PSDK_FORMAT_I420}) {
        std::vector<std::uint8_t> tight(flat,flat+y_bytes+chroma_bytes*2);
        if(format==PSDK_FORMAT_NV12)for(unsigned i=0;i<chroma_bytes;++i) {
            tight[y_bytes+i*2]=flat[y_bytes+i];tight[y_bytes+i*2+1]=flat[y_bytes+chroma_bytes+i];
        }
        std::vector<std::uint8_t> padded(stride*height*3/2,241);
        for(unsigned y=0;y<height;++y)
            std::memcpy(padded.data()+y*stride,tight.data()+y*width,width);
        if(format==PSDK_FORMAT_NV12) {
            for(unsigned y=0;y<height/2;++y)
                std::memcpy(padded.data()+stride*height+y*stride,tight.data()+y_bytes+y*width,width);
        } else {
            for(unsigned plane=0;plane<2;++plane)for(unsigned y=0;y<height/2;++y)
                std::memcpy(padded.data()+stride*height+plane*(stride/2)*(height/2)+y*(stride/2),
                    tight.data()+y_bytes+plane*chroma_bytes+y*(width/2),width/2);
        }
        const auto padded_before=padded;
        auto a=view(tight,format,width,height,width),b=view(padded,format,width,height,stride);
        const auto reference=inputs::toBgr(a),actual=inputs::toBgr(b);
        require(cv::norm(reference,actual,cv::NORM_INF)==0,"YUV row padding contaminated colors");
        require(padded==padded_before,"Source YUV bytes were modified");
        inputs::validatePair(a,b); // Strides may differ without changing pixel geometry.
        auto bad=b;bad.bytes--;rejects([&]{inputs::toBgr(bad);});
        bad=b;bad.stride=width-2;rejects([&]{inputs::validatePair(a,bad);});
        bad=b;bad.stride|=1;rejects([&]{inputs::validateFrame(bad);});
        bad=b;bad.frame_id++;rejects([&]{inputs::validatePair(a,bad);});
        bad=b;bad.timestamp_ns++;rejects([&]{inputs::validatePair(a,bad);});
        bad=b;bad.timestamp_ns=0;rejects([&]{inputs::validateFrame(bad);});
        bad=b;bad.format=format==PSDK_FORMAT_NV12?PSDK_FORMAT_I420:PSDK_FORMAT_NV12;
        rejects([&]{inputs::validatePair(a,bad);});
        bad=b;bad.width-=2;rejects([&]{inputs::validatePair(a,bad);});
        bad=b;bad.data=nullptr;rejects([&]{inputs::validateFrame(bad);});
        bad=b;bad.abi_version++;rejects([&]{inputs::validateFrame(bad);});
        inputs::validateHookFrame(a,a);
        rejects([&]{inputs::validateHookFrame(a,b);}); // Callback must use the exact registered left storage.
    }
    cv::Mat prediction(352,640,CV_32FC1,cv::Scalar(12.5f));
    const auto restored=r36compare::normalizeQ8(prediction,{640,544});
    require(restored.size()==640*544,"Restored Q8 geometry changed");
    for(auto q:restored)require(q==3200,"Height-only resize incorrectly changed disparity");
    cv::Mat half_width(352,320,CV_32FC1,cv::Scalar(12.5f));
    for(auto q:r36compare::normalizeQ8(half_width,{640,544}))
        require(q==6400,"Horizontal image scale was not applied to disparity");
    rejects([&]{r36compare::stereoInputAdaptation("strict",{640,544},{640,352});});
    rejects([&]{r36compare::stereoInputAdaptation("resize",{640,544},{640,352},{640,544});});
#ifdef R36_TEST_HORIZON_DISABLED
    bool unsupported=false;
    try{ros_so_demo::StereoProvider provider({"actual-model.bin","resize",640,352});}
    catch(const std::runtime_error& error){unsupported=std::string(error.what()).find("without Horizon Stereo support")!=std::string::npos;}
    require(unsupported,"Non-Horizon build did not explicitly reject model inference");
#endif
    std::cout<<"PASS: NV12/I420 padded rows, immutable sources, exact pair identities, malformed sources, source geometry and horizontal-only disparity scaling\n";
}
