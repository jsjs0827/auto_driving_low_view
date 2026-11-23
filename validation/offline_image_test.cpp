#include "perception_ros_so_demo/offline_image.hpp"
#include <algorithm>
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
void big32(std::vector<std::uint8_t>& bytes,std::uint32_t value) {
    for(int shift=24;shift>=0;shift-=8)bytes.push_back(std::uint8_t(value>>shift));
}
void chunk(std::vector<std::uint8_t>& png,const char* type,const std::vector<std::uint8_t>& data) {
    big32(png,std::uint32_t(data.size()));const auto start=png.size();
    png.insert(png.end(),type,type+4);png.insert(png.end(),data.begin(),data.end());
    std::uint32_t crc=0xffffffffU;
    for(auto i=start;i<png.size();++i) {
        crc^=png[i];for(int bit=0;bit<8;++bit)crc=(crc>>1)^((crc&1)?0xedb88320U:0);
    }
    big32(png,crc^0xffffffffU);
}
// Independent test encoder: PNG grayscale/RGB, filter None, zlib stored blocks.
// This exercises the production decoder without any decoder-generated fixtures.
void q8png(const fs::path& file,unsigned w,unsigned h,unsigned bits=16,unsigned channels=1) {
    std::vector<std::uint8_t> png{137,80,78,71,13,10,26,10},ihdr;
    big32(ihdr,w);big32(ihdr,h);
    ihdr.insert(ihdr.end(),{std::uint8_t(bits),std::uint8_t(channels==1?0:2),0,0,0});
    chunk(png,"IHDR",ihdr);
    constexpr std::array<std::uint16_t,6> values{{0,1,255,256,4097,65535}};
    std::vector<std::uint8_t> scan;
    for(unsigned y=0;y<h;++y) {
        scan.push_back(0);
        for(unsigned x=0;x<w;++x)for(unsigned c=0;c<channels;++c) {
            const auto value=values[(std::size_t(y)*w+x)%values.size()];
            if(bits==16)scan.push_back(std::uint8_t(value>>8));
            scan.push_back(std::uint8_t(value));
        }
    }
    std::vector<std::uint8_t> zlib{0x78,0x01};
    for(std::size_t pos=0;pos<scan.size();) {
        const auto size=std::uint16_t(std::min(std::size_t(65535),scan.size()-pos));
        const auto inverse=std::uint16_t(~size);
        zlib.insert(zlib.end(),{std::uint8_t(pos+size==scan.size()?1:0),std::uint8_t(size),
            std::uint8_t(size>>8),std::uint8_t(inverse),std::uint8_t(inverse>>8)});
        zlib.insert(zlib.end(),scan.begin()+pos,scan.begin()+pos+size);pos+=size;
    }
    std::uint32_t a=1,b=0;for(const auto value:scan){a=(a+value)%65521;b=(b+a)%65521;}
    big32(zlib,(b<<16)|a);chunk(png,"IDAT",zlib);chunk(png,"IEND",{});write(file,png);
}
void q8metadata(const fs::path& file,const std::string& byte_order="little",unsigned w=640,
                const std::string& value_format="q8.8") {
    std::ofstream out(fs::u8path(file.u8string()+".json"),std::ios::binary);
    out<<"{\"width\":"<<w<<",\"height\":544,\"encoding\":\"16UC1\",\"byte_order\":\""
       <<byte_order<<"\",\"value_format\":\""<<value_format<<"\"}";
    require(bool(out),"cannot write Q8 metadata");
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

        const auto disparity=temporary/"disparity.png";q8png(disparity,640,544);
        const auto q8=offline::loadQ8(disparity.u8string());
        constexpr std::array<std::uint16_t,6> q8values{{0,1,255,256,4097,65535}};
        require(q8.size()==plane*2,"Q8 PNG byte count changed");
        for(std::size_t i=0;i<plane;++i)
            require((unsigned(q8[i*2])|(unsigned(q8[i*2+1])<<8))==q8values[i%q8values.size()],
                    "16-bit PNG Q8 values were rescaled, truncated or byte-swapped");
        const auto gray8=temporary/"visualization.png";q8png(gray8,640,544,8);
        rejected([&]{offline::loadQ8(gray8.u8string());},"8-bit PNG accepted as original Q8");
        const auto rgb16=temporary/"color.png";q8png(rgb16,640,544,16,3);
        rejected([&]{offline::loadQ8(rgb16.u8string());},"color PNG accepted as original Q8");
        const auto smallq8=temporary/"smallq8.png";q8png(smallq8,640,352);
        rejected([&]{offline::loadQ8(smallq8.u8string());},"wrong-size Q8 PNG resized instead of rejected");
        const auto rawq8=temporary/"disparity.raw";write(rawq8,q8);
        rejected([&]{offline::loadQ8(rawq8.u8string());},"raw Q8 without explicit metadata accepted");
        q8metadata(rawq8);
        require(offline::loadQ8(rawq8.u8string())==q8,"raw little-endian Q8 changed");
        const auto savedq8=temporary/"disparity_q8.u16";write(savedq8,q8);q8metadata(savedq8);
        require(offline::loadQ8(savedq8.u8string())==q8,"SDK-style u16 little-endian Q8 changed");
        q8metadata(rawq8,"big");
        rejected([&]{offline::loadQ8(rawq8.u8string());},"big-endian raw accepted as uint16LE");
        q8metadata(rawq8,"little",320);
        rejected([&]{offline::loadQ8(rawq8.u8string());},"wrong raw Q8 dimensions accepted");
        q8metadata(rawq8,"little",640,"depth_mm");
        rejected([&]{offline::loadQ8(rawq8.u8string());},"millimeter depth declared as disparity accepted");
        q8metadata(rawq8);write(rawq8,{0,0});
        rejected([&]{offline::loadQ8(rawq8.u8string());},"truncated raw Q8 accepted");
        write(rawq8,std::vector<std::uint8_t>(plane*2,0));
        require(offline::loadQ8(rawq8.u8string())==std::vector<std::uint8_t>(plane*2,0),
                "all-invalid zero Q8 must remain valid input data");
        fs::remove_all(temporary);temporary.clear();
        std::cout<<"Offline RGB/YUV and original Q8 PNG/raw fidelity and invalid-input checks passed\n";
        return 0;
    }catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';
        if(!temporary.empty())std::cerr<<"Test fixtures retained at "<<temporary.u8string()<<'\n';
        return 1;
    }
}
