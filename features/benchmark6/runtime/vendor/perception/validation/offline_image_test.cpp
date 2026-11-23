#include "perception_ros_so_demo/offline_image.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs=std::filesystem;
namespace offline=ros_so_demo::offline;
constexpr std::size_t plane=640U*544U;
void require(bool value,const char* message) { if(!value)throw std::runtime_error(message); }
template<class F>void rejected(F action,const char* message) {
    bool failed=false;try{action();}catch(const std::exception&){failed=true;}
    require(failed,message);
}
void write(const fs::path& file,const std::vector<std::uint8_t>& bytes) {
    std::ofstream out(file,std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()));
    require(bool(out),"cannot write test fixture");
}
void ppm(const fs::path& file,unsigned w,unsigned h,const std::vector<std::uint8_t>& rgb) {
    require(rgb.size()==std::size_t(w)*h*3,"invalid test PPM size");
    std::ofstream out(file,std::ios::binary);out<<"P6\n"<<w<<' '<<h<<"\n255\n";
    out.write(reinterpret_cast<const char*>(rgb.data()),std::streamsize(rgb.size()));
    require(bool(out),"cannot write PPM fixture");
}
std::vector<std::uint8_t> red(unsigned w,unsigned h) {
    std::vector<std::uint8_t> bytes(std::size_t(w)*h*3);
    for(std::size_t i=0;i<bytes.size();i+=3)bytes[i]=255;
    return bytes;
}
void bmpRed(const fs::path& file) {
    std::vector<std::uint8_t> bytes(70,0);
    bytes[0]='B';bytes[1]='M';bytes[2]=70;bytes[10]=54;bytes[14]=40;
    bytes[18]=2;bytes[22]=2;bytes[26]=1;bytes[28]=24;bytes[34]=16;
    for(unsigned row=0;row<2;++row)for(unsigned x=0;x<2;++x)bytes[54+row*8+x*3+2]=255;
    write(file,bytes);
}
void solidRed(const offline::Converted& image) {
    require(image.bytes.size()==plane*3/2,"wrong YUV size");
    for(std::size_t i=0;i<plane;++i)require(image.bytes[i]==82,"red luma must be 82");
    for(std::size_t i=plane;i<image.bytes.size();i+=2) {
        require(image.bytes[i]==90,"red U must be 90");
        require(image.bytes[i+1]==240,"red V must be 240");
    }
}
int main(int argc,char** argv) {
    fs::path temporary;
    try {
        const auto base=argc>1?fs::u8path(argv[1]):fs::temp_directory_path();
        require(fs::is_directory(base),"test temporary parent must exist");
        temporary=base/("offline-image-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        require(fs::create_directory(temporary),"unique test directory must be newly created");
        const auto solid=temporary/"red.PPM";ppm(solid,4,2,red(4,2));
        const auto stretch=offline::load(solid.u8string(),"stretch","nv12");solidRed(stretch);
        require(stretch.original_width==4&&stretch.original_height==2&&stretch.resize=="stretch","original dimensions/resize lost");
        const auto bmp=temporary/"red.bmp";bmpRed(bmp);solidRed(offline::load(bmp.u8string(),"stretch","nv12"));

        const auto boxed=offline::load(solid.u8string(),"letterbox","nv12");
        require(boxed.resize=="letterbox","letterbox metadata lost");
        for(unsigned y=0;y<544;++y)for(unsigned x=0;x<640;++x)
            require(boxed.bytes[std::size_t(y)*640+x]==(y>=112&&y<432?82:16),"letterbox must preserve red region and limited-range black padding");
        require(boxed.bytes[plane]==128&&boxed.bytes[plane+1]==128,"black UV must be neutral");
        const auto center=plane+std::size_t(544/4)*640;
        require(boxed.bytes[center]==90&&boxed.bytes[center+1]==240,"letterbox center chroma incorrect");

        // Independent exact 2x2 colors test: red, green, blue, white.
        constexpr std::array<std::array<std::uint8_t,3>,4> colors{{{{255,0,0}},{{0,255,0}},{{0,0,255}},{{255,255,255}}}};
        constexpr std::array<std::uint8_t,4> expectedY{{82,144,41,235}};
        std::vector<std::uint8_t> mixed(plane*3);
        for(unsigned y=0;y<544;++y)for(unsigned x=0;x<640;++x)for(unsigned c=0;c<3;++c)
            mixed[(std::size_t(y)*640+x)*3+c]=colors[(y%2)*2+x%2][c];
        const auto mixedfile=temporary/"mixed.ppm";ppm(mixedfile,640,544,mixed);
        const auto nv12=offline::load(mixedfile.u8string(),"stretch","nv12");
        const auto i420=offline::load(mixedfile.u8string(),"stretch","i420");
        require(i420.bytes.size()==plane*3/2,"wrong I420 size");
        for(unsigned y=0;y<544;++y)for(unsigned x=0;x<640;++x) {
            const auto i=std::size_t(y)*640+x;
            require(nv12.bytes[i]==expectedY[(y%2)*2+x%2],"RGB primary luma conversion incorrect");
            require(nv12.bytes[i]==i420.bytes[i],"NV12/I420 luma differs");
        }
        for(std::size_t i=0;i<plane/4;++i) {
            require(nv12.bytes[plane+i*2]==128&&nv12.bytes[plane+i*2+1]==128,"2x2 average chroma incorrect");
            require(i420.bytes[plane+i]==nv12.bytes[plane+i*2],"I420 U plane incorrect");
            require(i420.bytes[plane+plane/4+i]==nv12.bytes[plane+i*2+1],"I420 V plane incorrect");
        }
        const auto red420=offline::load(solid.u8string(),"stretch","i420");
        for(std::size_t i=0;i<plane/4;++i)
            require(red420.bytes[plane+i]==90&&red420.bytes[plane+plane/4+i]==240,"I420 planes swapped/interleaved");

        const auto raw=temporary/"source.nv12";write(raw,nv12.bytes);
        const auto rawimage=offline::load(raw.u8string(),"letterbox","nv12");
        require(rawimage.bytes==nv12.bytes&&rawimage.resize=="none"&&rawimage.original_width==640&&rawimage.original_height==544,"raw bytes/metadata changed");
        const auto raw420=temporary/"source.i420";write(raw420,i420.bytes);
        require(offline::load(raw420.u8string(),"stretch","i420").bytes==i420.bytes,"raw I420 changed");
        const auto yuv=temporary/"source.yuv";write(yuv,nv12.bytes);
        require(offline::load(yuv.u8string(),"stretch","nv12").bytes==nv12.bytes,"explicit raw YUV interpretation changed");
        rejected([&]{offline::load(raw.u8string(),"stretch","i420");},"conflicting NV12 format accepted");
        rejected([&]{offline::load(raw420.u8string(),"stretch","nv12");},"conflicting I420 format accepted");
        rejected([&]{offline::load(solid.u8string(),"nearest","nv12");},"invalid resize accepted");
        rejected([&]{offline::load(solid.u8string(),"stretch","rgb8");},"invalid output format accepted");
        rejected([&]{offline::load((temporary/"missing.png").u8string(),"stretch","nv12");},"missing image accepted");
        const auto bad=temporary/"invalid.png";write(bad,{1,2,3,4});
        rejected([&]{offline::load(bad.u8string(),"stretch","nv12");},"invalid PNG accepted");
        const auto shortfile=temporary/"short.nv12";write(shortfile,{1,2,3,4});
        rejected([&]{offline::load(shortfile.u8string(),"stretch","nv12");},"short raw image accepted");
        const auto large=temporary/"huge.ppm";
        {std::ofstream out(large,std::ios::binary);out<<"P6\n8192 8192\n255\n";}
        rejected([&]{offline::load(large.u8string(),"stretch","nv12");},"unreasonable decoded dimensions accepted");
        fs::remove_all(temporary);temporary.clear();
        std::cout<<"Offline RGB decode/resize, BT.601 limited YUV, raw formats and invalid-input checks passed\n";
        return 0;
    }catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';
        if(!temporary.empty())std::cerr<<"Test fixtures retained at "<<temporary.u8string()<<'\n';
        return 1;
    }
}
