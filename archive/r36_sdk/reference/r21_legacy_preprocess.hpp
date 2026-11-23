#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <map>
#include <string>
#include <cfenv>
namespace r20 {
struct Geometry {
    float x_scale=1.f,y_scale=1.f;
    int x_shift=0,y_shift=0,valid_w=0,valid_h=0;
};
namespace reference {
bool PreprocessYuv420(const std::uint8_t*,int,int,int,std::size_t,bool,int,int,int,Geometry*,std::vector<std::uint8_t>&,std::uint8_t);
bool ConvertI420ToNv12(const std::uint8_t*,int,int,int,std::vector<std::uint8_t>&);
}
struct Packed {
    const std::uint8_t* data=nullptr;
    std::size_t bytes=0;
    int stride=0;
};
class PackWorkspace {
    std::vector<std::uint8_t> storage_;
public:
    Packed run(const std::uint8_t*,int,int,int,std::size_t,bool);
};
class DetectorWorkspace {
    struct Axis {int a,b;float w,iw;};
    std::vector<Axis> lx_,ly_,cx_,cy_;
    int sw_=0,sh_=0,rw_=0,rh_=0,rounding_=-1,math_=-1;
    PackWorkspace pack_;
    void maps(int,int,int,int);
    void render(const std::uint8_t*,int,int,int,int,int,const Geometry&,int);
public:
    std::vector<std::uint8_t> output;
    bool prepare(const std::uint8_t*,int,int,int,std::size_t,bool,int,int,int,Geometry&);
    int mathMode() const noexcept{return math_;}
};
Geometry originalGeometry(int,int,int,int,int);
struct TensorAudit {
    std::vector<std::uint8_t> prepared,storage;
    std::vector<std::uint8_t> output_storage;
    std::vector<float> decoded_logits;
    std::string output_metadata;
    int width=0,height=0,left=0,top=0,valid_width=0,valid_height=0;
};
void setPolicy(bool optimized,bool capture);
bool optimizedPolicy() noexcept;
bool capturePolicy() noexcept;
DetectorWorkspace& detectorWorkspace();
DetectorWorkspace& segmentationWorkspace();
void captureTensor(const char*,const std::vector<std::uint8_t>&,const void*,std::size_t,int,int,int,int,int,int);
void captureOutput(const char*,const void*,std::size_t,const std::string&,const std::vector<float>* =nullptr);
std::map<std::string,TensorAudit> takeAudit();
bool neonEnabled() noexcept;
}
