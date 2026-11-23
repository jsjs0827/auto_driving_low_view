#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <map>
#include <string>
#include <cfenv>
#include <array>
#include <cmath>
namespace r20 {
// Match the measured V0 AArch64 resize instructions, not an algebraically
// equivalent source expression. The reciprocal and product each round to FP32;
// coordinate multiplication/subtraction then uses exactly one explicit FMA.
// Volatile stores keep these two rounding points even if this internal helper
// is included by a caller with different contraction settings.
struct V0SamplingAxis {
    float ratio;
    V0SamplingAxis(int source,int target) noexcept {
        const volatile float reciprocal=1.f/static_cast<float>(target);
        const volatile float scaled=reciprocal*static_cast<float>(source);
        ratio=scaled;
    }
    float coordinate(int index) const noexcept {
        return std::fma(static_cast<float>(index)+.5f,ratio,-.5f);
    }
};
// The measured V0 NV12 sampler fuses the inverse-weight product with the
// already rounded right product. Keep the scalar calibration oracle explicit.
inline float v0Nv12Blend(float first,float second,float weight) noexcept {
    const volatile float right=weight*second;
    return std::fma(1.f-weight,first,right);
}
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
using ReferencePreprocess=bool (*)(const std::uint8_t*,int,int,int,std::size_t,bool,int,int,int,Geometry*,std::vector<std::uint8_t>&,std::uint8_t);
class PackWorkspace {
    std::vector<std::uint8_t> storage_;
public:
    Packed run(const std::uint8_t*,int,int,int,std::size_t,bool);
};
class DetectorWorkspace {
    struct Axis {int a,b;float w,iw;};
    std::vector<Axis> lx_,ly_,cx_,cy_;
    int sw_=0,sh_=0,rw_=0,rh_=0,rounding_=-1,math_=-1;
    std::array<int,3> math_modes_{{-1,-1,-1}};
    ReferencePreprocess reference_=reference::PreprocessYuv420;
    std::uint64_t reference_context_=0;
    PackWorkspace pack_;
    void maps(int,int,int,int);
    void render(const std::uint8_t*,int,int,int,int,int,const Geometry&,const std::array<int,3>&);
public:
    std::vector<std::uint8_t> output;
    bool prepare(const std::uint8_t*,int,int,int,std::size_t,bool,int,int,int,Geometry&);
    int mathMode() const noexcept{return math_;}
    std::array<int,3> mathModes() const noexcept{return math_modes_;}
    void setReference(ReferencePreprocess callback,std::uint64_t context=0) {
        if(!callback)callback=reference::PreprocessYuv420;
        if(reference_!=callback||reference_context_!=context){reference_=callback;reference_context_=context;math_=-1;math_modes_={{-1,-1,-1}};}
    }
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
