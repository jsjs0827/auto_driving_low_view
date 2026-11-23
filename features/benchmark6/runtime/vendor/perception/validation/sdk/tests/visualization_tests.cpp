#include "visualization.hpp"
#include "image_input.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace fs=std::filesystem;
void expect(bool value,const char* message){if(!value)throw std::runtime_error(message);}

int main(){try{
    example::Image rgb;rgb.width=4;rgb.height=4;rgb.stride=16;
    rgb.format=perception::PixelFormat::RGB;rgb.bytes.assign(64,239);
    std::vector<std::uint8_t> expected;
    for(int y=0;y<4;++y)for(int x=0;x<4;++x){
        const std::uint8_t pixel[]={std::uint8_t(17*x),std::uint8_t(31*y),std::uint8_t(5*x+7*y)};
        std::copy_n(pixel,3,rgb.bytes.data()+y*rgb.stride+x*3);
        expected.insert(expected.end(),pixel,pixel+3);
    }
    const auto before_rgb=rgb.bytes;
    expect(demo_vis::renderRgb(rgb,nullptr,nullptr)==expected,"padded RGB conversion differs");
    auto bgr=rgb;bgr.format=perception::PixelFormat::BGR;
    for(int y=0;y<4;++y)for(int x=0;x<4;++x)std::swap(bgr.bytes[y*bgr.stride+x*3],bgr.bytes[y*bgr.stride+x*3+2]);
    expect(demo_vis::renderRgb(bgr,nullptr,nullptr)==expected,"BGR channel order differs");

    example::Image nv12;nv12.width=4;nv12.height=4;nv12.stride=6;
    nv12.format=perception::PixelFormat::NV12;nv12.bytes.assign(36,99);
    for(int y=0;y<4;++y)std::fill_n(nv12.bytes.data()+y*6,4,std::uint8_t(16));
    for(int y=0;y<2;++y)std::fill_n(nv12.bytes.data()+24+y*6,4,std::uint8_t(128));
    nv12.bytes[0]=235;
    std::vector<std::uint8_t> gray(48,0);std::fill_n(gray.begin(),3,std::uint8_t(255));
    expect(demo_vis::renderRgb(nv12,nullptr,nullptr)==gray,"NV12 black/white or stride differs");
    for(int y=0;y<4;++y)std::fill_n(nv12.bytes.data()+y*6,4,std::uint8_t(81));
    for(int y=0;y<2;++y)for(int x=0;x<2;++x){nv12.bytes[24+y*6+x*2]=90;nv12.bytes[25+y*6+x*2]=240;}
    std::vector<std::uint8_t> red(48,0);for(int p=0;p<16;++p)red[p*3]=254;
    expect(demo_vis::renderRgb(nv12,nullptr,nullptr)==red,"NV12 U/V order or color conversion differs");
    auto i420=nv12;i420.format=perception::PixelFormat::I420;
    for(int y=0;y<2;++y){std::fill_n(i420.bytes.data()+24+y*3,2,std::uint8_t(90));std::fill_n(i420.bytes.data()+30+y*3,2,std::uint8_t(240));}
    const auto before_i420=i420.bytes;
    expect(demo_vis::renderRgb(i420,nullptr,nullptr)==red,"I420 planes or stride differ");

    perception::DetectionResult det;det.width=4;det.height=4;det.detections.push_back({0,.8f,0,0,3,3});
    perception::SegmentationResult seg;seg.width=4;seg.height=4;seg.mask.resize(16);
    for(int p=0;p<16;++p)seg.mask[p]=std::uint8_t(p%3);
    const auto overlay=demo_vis::renderRgb(rgb,&det,&seg);
    expect(overlay.size()==expected.size()&&overlay!=expected,"overlay was not rendered");
    expect(rgb.bytes==before_rgb&&i420.bytes==before_i420,"renderer changed its source image");

    const auto dir=fs::temp_directory_path()/("r20-visualization-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    expect(fs::create_directory(dir),"temporary PNG directory was not created");
    const auto file=dir/"overlay.png";
    demo_vis::savePng(file,rgb,&det,&seg);
    const auto decoded=example::loadImage(file.string(),{});
    expect(decoded.width==4&&decoded.height==4&&decoded.bytes==overlay,"saved PNG round trip differs");
    bool rejected=false;try{demo_vis::savePng(file,rgb,&det,&seg);}catch(const std::exception&){rejected=true;}
    expect(rejected,"existing PNG was overwritten");
    expect(example::loadImage(file.string(),{}).bytes==overlay,"existing PNG contents changed");
    fs::remove(file);fs::remove(dir);
    std::cout<<"PASS visualization static link, RGB/BGR/NV12/I420 stride, overlays, PNG round trip and input immutability\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
