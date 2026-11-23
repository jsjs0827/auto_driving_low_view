#include "perception_ros_so_demo/offline_image.hpp"
#include "../../../scheduling/config/include/json.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

// Existing project vendor dependency; keep its MIT/public-domain notice intact.
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_ONLY_BMP
#define STBI_ONLY_PNM
#define STBI_NO_STDIO
#define STBI_MAX_DIMENSIONS 8192
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "../vendor/stb/stb_image.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace ros_so_demo::offline {
namespace {
constexpr unsigned width=640, height=544;
constexpr std::size_t plane=std::size_t(width)*height;
constexpr std::size_t yuv_bytes=plane*3/2;
constexpr std::size_t max_encoded_bytes=128U*1024U*1024U;
constexpr std::size_t max_pixels=16U*1024U*1024U;

[[noreturn]] void fail(const std::string& path,const std::string& reason) {
    throw std::runtime_error("Offline image '"+path+"': "+reason);
}
std::vector<std::uint8_t> read(const std::string& path) {
    std::ifstream file(std::filesystem::u8path(path),std::ios::binary|std::ios::ate);
    if(!file)fail(path,"cannot open file");
    const auto length=file.tellg();
    if(length<=0||static_cast<std::uint64_t>(length)>max_encoded_bytes)
        fail(path,"file must be nonempty and no larger than 128 MiB");
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
    file.seekg(0);
    if(!file.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size())))
        fail(path,"cannot read the complete file");
    return bytes;
}
void checkDimensions(int w,int h,const std::string& path) {
    if(w<=0||h<=0||w>8192||h>8192||std::size_t(w)*std::size_t(h)>max_pixels)
        fail(path,"decoded dimensions must be within 1..8192 and at most 16M pixels");
}
struct Sample { int first,second; double weight; };
Sample sample(unsigned index,unsigned target,unsigned source) {
    const double value=std::clamp((double(index)+.5)*source/target-.5,0.,double(source-1));
    const auto first=static_cast<int>(value);
    return {first,std::min(first+1,int(source)-1),value-first};
}
std::vector<std::uint8_t> resizeRgb(const std::uint8_t* source,unsigned sw,unsigned sh,
                                   const std::string& resize) {
    unsigned rw=width,rh=height;
    if(resize=="letterbox") {
        const double scale=std::min(double(width)/sw,double(height)/sh);
        rw=std::clamp(unsigned(std::lround(sw*scale)),1U,width);
        rh=std::clamp(unsigned(std::lround(sh*scale)),1U,height);
    }
    const unsigned left=(width-rw)/2,top=(height-rh)/2;
    std::vector<std::uint8_t> rgb(plane*3,0); // RGB black -> limited Y=16, U/V=128.
    std::vector<Sample> columns;columns.reserve(rw);
    for(unsigned x=0;x<rw;++x)columns.push_back(sample(x,rw,sw));
    for(unsigned y=0;y<rh;++y) {
        const auto row=sample(y,rh,sh);
        for(unsigned x=0;x<rw;++x) {
            const auto col=columns[x];
            for(unsigned c=0;c<3;++c) {
                const auto at=[&](int sx,int sy){return double(source[(std::size_t(sy)*sw+unsigned(sx))*3+c]);};
                const double a=at(col.first,row.first)*(1-col.weight)+at(col.second,row.first)*col.weight;
                const double b=at(col.first,row.second)*(1-col.weight)+at(col.second,row.second)*col.weight;
                rgb[(std::size_t(y+top)*width+x+left)*3+c]=
                    static_cast<std::uint8_t>(std::clamp(std::lround(a*(1-row.weight)+b*row.weight),0L,255L));
            }
        }
    }
    return rgb;
}
// Portable arithmetic floor division, including negative chroma numerators.
int floorDivide(int value,int divisor) { return value>=0?value/divisor:-((-value+divisor-1)/divisor); }
std::uint8_t byte(int value) { return static_cast<std::uint8_t>(std::clamp(value,0,255)); }
std::vector<std::uint8_t> yuv420(const std::vector<std::uint8_t>& rgb,const std::string& format) {
    std::vector<std::uint8_t> out(yuv_bytes);
    for(std::size_t i=0;i<plane;++i) {
        const auto* p=rgb.data()+i*3;
        out[i]=byte((66*int(p[0])+129*int(p[1])+25*int(p[2])+128)/256+16);
    }
    for(unsigned y=0;y<height;y+=2)for(unsigned x=0;x<width;x+=2) {
        int r=0,g=0,b=0;
        for(unsigned dy=0;dy<2;++dy)for(unsigned dx=0;dx<2;++dx) {
            const auto* p=rgb.data()+(std::size_t(y+dy)*width+x+dx)*3;
            r+=p[0];g+=p[1];b+=p[2];
        }
        // 2x2 channel averages, with a single rounding step after conversion.
        const auto u=byte(floorDivide(-38*r-74*g+112*b+512,1024)+128);
        const auto v=byte(floorDivide(112*r-94*g-18*b+512,1024)+128);
        const auto cell=std::size_t(y/2)*(width/2)+x/2;
        if(format=="nv12") { out[plane+cell*2]=u;out[plane+cell*2+1]=v; }
        else { out[plane+cell]=u;out[plane+plane/4+cell]=v; }
    }
    return out;
}
}

Converted load(const std::string& path,const std::string& resize,const std::string& format) {
    if(resize!="letterbox"&&resize!="stretch")
        throw std::invalid_argument("offline resize must be letterbox or stretch");
    if(format!="nv12"&&format!="i420")
        throw std::invalid_argument("offline format must be nv12 or i420");
    if(path.empty())throw std::invalid_argument("offline image path must not be empty");
    auto extension=std::filesystem::u8path(path).extension().u8string();
    std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char c){return char(std::tolower(c));});
    const bool raw=extension==".nv12"||extension==".i420"||extension==".yuv";
    if(!raw&&extension!=".jpg"&&extension!=".jpeg"&&extension!=".png"&&extension!=".bmp"&&
       extension!=".ppm"&&extension!=".pnm"&&extension!=".pgm")
        fail(path,"supported extensions are JPG/JPEG/PNG/BMP/PPM/PNM/PGM and raw NV12/I420/YUV");
    if((extension==".nv12"&&format!="nv12")||(extension==".i420"&&format!="i420"))
        fail(path,"raw extension conflicts with the explicit output format; raw data is not converted");
    auto bytes=read(path);
    if(raw) {
        if(bytes.size()!=yuv_bytes)fail(path,"raw input must be tight 640x544 YUV420, exactly 522240 bytes");
        return {std::move(bytes),width,height,"none"};
    }
    int sw=0,sh=0,channels=0;
    if(!stbi_info_from_memory(bytes.data(),int(bytes.size()),&sw,&sh,&channels))
        fail(path,std::string("cannot inspect image: ")+(stbi_failure_reason()?stbi_failure_reason():"unknown format"));
    checkDimensions(sw,sh,path);
    using Pixels=std::unique_ptr<stbi_uc,decltype(&stbi_image_free)>;
    Pixels pixels(stbi_load_from_memory(bytes.data(),int(bytes.size()),&sw,&sh,&channels,3),stbi_image_free);
    if(!pixels)fail(path,std::string("cannot decode image: ")+(stbi_failure_reason()?stbi_failure_reason():"invalid encoded image"));
    checkDimensions(sw,sh,path);
    const auto rgb=resizeRgb(pixels.get(),unsigned(sw),unsigned(sh),resize);
    return {yuv420(rgb,format),unsigned(sw),unsigned(sh),resize};
}

std::vector<std::uint8_t> loadQ8(const std::string& path,unsigned expected_width,unsigned expected_height) {
    if(path.empty())throw std::invalid_argument("offline Q8 path must not be empty");
    checkDimensions(int(expected_width),int(expected_height),path);
    auto extension=std::filesystem::u8path(path).extension().u8string();
    std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char c){return char(std::tolower(c));});
    if(extension!=".png"&&extension!=".raw"&&extension!=".q8"&&extension!=".u16")
        fail(path,"Q8 supports only 16-bit single-channel PNG or uint16LE .raw/.q8/.u16 with JSON sidecar");
    auto bytes=read(path);
    const auto pixels=std::size_t(expected_width)*expected_height;
    if(extension!=".png") {
        using namespace r33::json;
        const auto metadata=r33::json::load(path+".json");
        fields(metadata,{"width","height","encoding","byte_order","value_format","_help"});
        if(integer(required(metadata,"width"),1,8192)!=int(expected_width)||
           integer(required(metadata,"height"),1,8192)!=int(expected_height)||
           string(required(metadata,"encoding"))!="16UC1"||
           string(required(metadata,"byte_order"))!="little"||
           string(required(metadata,"value_format"))!="q8.8")
            fail(path,"raw Q8 sidecar must declare matching dimensions, 16UC1, little byte order and q8.8 values");
        if(bytes.size()!=pixels*2)fail(path,"raw Q8 must contain exactly width*height*2 bytes with no padding");
        return bytes;
    }
    int sw=0,sh=0,channels=0;
    if(!stbi_info_from_memory(bytes.data(),int(bytes.size()),&sw,&sh,&channels)||
       !stbi_is_16_bit_from_memory(bytes.data(),int(bytes.size()))||channels!=1)
        fail(path,"Q8 PNG must be a genuine 16-bit single-channel image; 8-bit/color images are not disparity");
    if(sw!=int(expected_width)||sh!=int(expected_height))
        fail(path,"Q8 PNG dimensions must match the unresized left image and calibration");
    using Values=std::unique_ptr<stbi_us,decltype(&stbi_image_free)>;
    Values values(stbi_load_16_from_memory(bytes.data(),int(bytes.size()),&sw,&sh,&channels,1),stbi_image_free);
    if(!values)fail(path,"cannot decode 16-bit Q8 PNG");
    std::vector<std::uint8_t> output(pixels*2);
    for(std::size_t i=0;i<pixels;++i) {
        output[i*2]=std::uint8_t(values.get()[i]&255U);
        output[i*2+1]=std::uint8_t(values.get()[i]>>8);
    }
    return output;
}
}
