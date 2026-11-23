#include "legacy_preprocess.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#if defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#define R20_NEON 1
#endif
namespace r20 {
namespace {
std::atomic<bool> optimized{false},capturing{false};
std::mutex audit_mutex;
std::map<std::string,TensorAudit> audits;
float blend(float a,float b,float iw,float w,int mode) {
    // This translation unit disables implicit contraction. Explicit modes
    // reproduce the reference compiler's observed FP32 multiply/add order.
    if(mode==1)return std::fma(iw,a,w*b);
    if(mode==2)return std::fma(w,b,iw*a);
    return iw*a+w*b;
}
void validate(const std::uint8_t* s,int w,int h,int stride,std::size_t bytes) {
    if(!s||w<2||h<2||w>4096||h>4096||w%2||h%2||stride<w||stride%2||
       stride>16*1024*1024||bytes<std::size_t(stride)*h*3/2)
        throw std::invalid_argument("invalid source YUV420 buffer");
}
}
bool neonEnabled() noexcept {
#ifdef R20_NEON
    return true;
#else
    return false;
#endif
}
Packed PackWorkspace::run(const std::uint8_t* src,int w,int h,int stride,std::size_t bytes,bool nv12) {
    validate(src,w,h,stride,bytes);
    const auto plane=std::size_t(w)*h;
    if(nv12&&stride==w)return {src,plane*3/2,w};
    storage_.resize(plane*3/2);
    if(stride==w)std::memcpy(storage_.data(),src,plane);
    else for(int y=0;y<h;++y)std::memcpy(storage_.data()+std::size_t(y)*w,src+std::size_t(y)*stride,w);
    const auto* u=src+std::size_t(stride)*h;
    if(nv12)for(int y=0;y<h/2;++y)std::memcpy(storage_.data()+plane+std::size_t(y)*w,u+std::size_t(y)*stride,w);
    else {
        const auto* v=u+std::size_t(stride)*h/4;
        for(int y=0;y<h/2;++y){const auto* ur=u+std::size_t(y)*stride/2;const auto* vr=v+std::size_t(y)*stride/2;
            auto* out=storage_.data()+plane+std::size_t(y)*w;int x=0;
#ifdef R20_NEON
            for(;x+16<=w/2;x+=16){uint8x16x2_t uv;uv.val[0]=vld1q_u8(ur+x);uv.val[1]=vld1q_u8(vr+x);vst2q_u8(out+2*x,uv);}
#endif
            for(;x<w/2;++x){out[2*x]=ur[x];out[2*x+1]=vr[x];}
        }
    }
    return {storage_.data(),storage_.size(),w};
}
Geometry originalGeometry(int sw,int sh,int dw,int dh,int resize) {
    if(sw<2||sh<2||dw<2||dh<2||sw>4096||sh>4096||dw>4096||dh>4096||sw%2||sh%2||dw%2||dh%2)
        throw std::invalid_argument("invalid legacy resize dimensions");
    Geometry g;
    if(resize==1){g.x_scale=std::min(1.f*dh/sh,1.f*dw/sw);g.y_scale=g.x_scale;
        g.valid_w=int(sw*g.x_scale)&~1;g.valid_h=int(sh*g.y_scale)&~1;
        g.x_shift=(dw-g.valid_w)/2;g.y_shift=(dh-g.valid_h)/2;
    }else{g.x_scale=1.f*dw/sw;g.y_scale=1.f*dh/sh;g.valid_w=dw;g.valid_h=dh;}
    if(g.valid_w<2||g.valid_h<2)throw std::invalid_argument("legacy effective image too small");
    return g;
}
void DetectorWorkspace::maps(int sw,int sh,int rw,int rh) {
    const int rounding=std::fegetround();
    if(sw==sw_&&sh==sh_&&rw==rw_&&rh==rh_&&rounding==rounding_)return;
    const auto fill=[](std::vector<Axis>& axis,int source,int target){axis.resize(target);
        const V0SamplingAxis sampling(source,target);
        for(int i=0;i<target;++i){float p=sampling.coordinate(i);
            p=std::max(0.f,std::min(p,static_cast<float>(source-1)));
            const int a=int(std::floor(p));const float weight=p-static_cast<float>(a);
            axis[i]={a,std::min(a+1,source-1),weight,1.f-weight};}};
    fill(lx_,sw,rw);fill(ly_,sh,rh);fill(cx_,sw/2,rw/2);fill(cy_,sh/2,rh/2);
    sw_=sw;sh_=sh;rw_=rw;rh_=rh;rounding_=rounding;math_=-1;
}
void DetectorWorkspace::render(const std::uint8_t* source,int sw,int sh,int stride,int dw,int dh,const Geometry& g,const std::array<int,3>& math) {
    (void)sw;
    const auto plane=std::size_t(dw)*dh;
    output.resize(plane*3/2);
    std::fill(output.begin(),output.begin()+plane,std::uint8_t(127));
    std::fill(output.begin()+plane,output.end(),std::uint8_t(128));
    const auto run=[&](const std::uint8_t* data,int channels,const std::vector<Axis>& xs,const std::vector<Axis>& ys,std::uint8_t* target,int left,int top){
        const int first=channels==1?math[0]:math[1],second=channels==1?first:math[2];
        const int xmode=first%3,ymode=first/3;
        const int row_bytes=int(xs.size())*channels;
        for(int y=0;y<int(ys.size());++y){const auto& ay=ys[y];const auto* a=data+std::size_t(ay.a)*stride;const auto* b=data+std::size_t(ay.b)*stride;
            auto* out=target+std::size_t(y+top)*dw+left;int i=0;
#ifdef R20_NEON
            for(;i+4<=row_bytes;i+=4){float av[4],bv[4],cv[4],dv[4],wv[4],iv[4];
                for(int k=0;k<4;++k){const auto& x=xs[(i+k)/channels];const int ch=(i+k)%channels;
                    av[k]=a[x.a*channels+ch];bv[k]=a[x.b*channels+ch];cv[k]=b[x.a*channels+ch];dv[k]=b[x.b*channels+ch];wv[k]=x.w;iv[k]=x.iw;}
                const auto w=vld1q_f32(wv),iw=vld1q_f32(iv);
                const auto mix=[](float32x4_t first,float32x4_t second,float32x4_t inv,float32x4_t weight,int mode){
                    if(mode==1)return vfmaq_f32(vmulq_f32(weight,second),inv,first);
                    if(mode==2)return vfmaq_f32(vmulq_f32(inv,first),weight,second);
                    return vaddq_f32(vmulq_f32(inv,first),vmulq_f32(weight,second));};
                const auto t=mix(vld1q_f32(av),vld1q_f32(bv),iw,w,xmode),u=mix(vld1q_f32(cv),vld1q_f32(dv),iw,w,xmode);
                auto blended=mix(t,u,vdupq_n_f32(ay.iw),vdupq_n_f32(ay.w),ymode);
                if(channels==2 && first!=second){
                    const uint32_t bits[4]={0,0xffffffffu,0,0xffffffffu};
                    const auto tv=mix(vld1q_f32(av),vld1q_f32(bv),iw,w,second%3),uv=mix(vld1q_f32(cv),vld1q_f32(dv),iw,w,second%3);
                    blended=vbslq_f32(vld1q_u32(bits),mix(tv,uv,vdupq_n_f32(ay.iw),vdupq_n_f32(ay.w),second/3),blended);
                }
                auto v=vaddq_f32(blended,vdupq_n_f32(.5f));
                if(channels==2)v=vmaxq_f32(vdupq_n_f32(0.f),vminq_f32(vdupq_n_f32(255.f),v));
                std::uint32_t values[4];vst1q_u32(values,vcvtq_u32_f32(v));
                for(int k=0;k<4;++k)out[i+k]=static_cast<std::uint8_t>(values[k]);
            }
#endif
            for(;i<row_bytes;++i){const auto& x=xs[i/channels];const int ch=i%channels;
                const int m=channels==1?first:math[1+ch];
                const float t=blend(a[x.a*channels+ch],a[x.b*channels+ch],x.iw,x.w,m%3);
                const float u=blend(b[x.a*channels+ch],b[x.b*channels+ch],x.iw,x.w,m%3);
                float v=blend(t,u,ay.iw,ay.w,m/3)+.5f;
                if(channels==2)v=std::max(0.f,std::min(255.f,v));out[i]=static_cast<std::uint8_t>(v);
            }
        }
    };
    run(source,1,lx_,ly_,output.data(),g.x_shift,g.y_shift);
    run(source+std::size_t(stride)*sh,2,cx_,cy_,output.data()+plane,g.x_shift,g.y_shift/2);
}
bool DetectorWorkspace::prepare(const std::uint8_t* s,int sw,int sh,int stride,std::size_t bytes,bool nv12,int dw,int dh,int resize,Geometry& g) {
    const auto packed=pack_.run(s,sw,sh,stride,bytes,nv12);
    g=originalGeometry(sw,sh,dw,dh,resize);maps(sw,sh,g.valid_w,g.valid_h);
    if(math_<0){Geometry baseline_geometry;std::vector<std::uint8_t> baseline;
        std::array<std::vector<int>,3> candidates;
        for(auto& v:candidates)for(int m=0;m<9;++m)v.push_back(m);
        const auto match=[&](int channel){
            if(output.size()!=baseline.size())return false;
            const auto plane=std::size_t(dw)*dh;
            if(channel==0)return std::equal(output.begin(),output.begin()+plane,baseline.begin());
            for(std::size_t i=plane+channel-1;i<output.size();i+=2)if(output[i]!=baseline[i])return false;
            return true;
        };
        const auto intersect=[&](const Packed& image){
            for(int mode=0;mode<9;++mode){
                render(image.data,sw,sh,image.stride,dw,dh,g,{{mode,mode,mode}});
                for(int c=0;c<3;++c)if(!match(c)){
                    auto& v=candidates[c];v.erase(std::remove(v.begin(),v.end(),mode),v.end());
                }
            }
        };
        if(!reference_(s,sw,sh,stride,bytes,nv12,dw,dh,resize,&baseline_geometry,baseline,127))return false;
        intersect(packed);
        // A dark first frame alone cannot identify FMA order. Intersect with
        // deterministic full-range fixtures at the same geometry before use.
        std::vector<std::uint8_t> fixture(std::size_t(sw)*sh*3/2);
        std::uint32_t state=0x19ac420u;
        // Full-range samples alone can miss one-ULP errors on low-contrast half ties.
        // Include narrow ranges near 0, 128 and 255 before choosing a channel mode.
        for(int test=0;test<5;++test){for(std::size_t i=0;i<fixture.size();++i){state=state*1664525u+1013904223u;
            fixture[i]=test==0?std::uint8_t(state>>24):test==1?std::uint8_t((i*37+i/7)%256):
                test==2?std::uint8_t((state>>24)&7):test==3?std::uint8_t(120+((state>>24)&15)):std::uint8_t(240+((state>>24)&15));}
            if(!reference_(fixture.data(),sw,sh,sw,fixture.size(),true,dw,dh,resize,&baseline_geometry,baseline,127))return false;
            intersect({fixture.data(),fixture.size(),sw});
        }
        // Regression from FrameId 36: U(3,77) in the valid 282x240 chroma image.
        // The values discriminate 126.499992 (mode 3) from 126.5 (mode 4).
        if(sw==1280 && sh==1088 && g.valid_w==564 && g.valid_h==480){
            std::fill(fixture.begin(),fixture.end(),uint8_t(127));const auto plane=std::size_t(sw)*sh;
            for(int c=0;c<2;++c){fixture[plane+175*sw+7*2+c]=127;fixture[plane+175*sw+8*2+c]=126;
                fixture[plane+176*sw+7*2+c]=124;fixture[plane+176*sw+8*2+c]=129;}
            if(!reference_(fixture.data(),sw,sh,sw,fixture.size(),true,dw,dh,resize,&baseline_geometry,baseline,127))return false;
            intersect({fixture.data(),fixture.size(),sw});
        }
#include "calibration_640.inc"
        for(int c=0;c<3;++c){
            if(candidates[c].empty()){std::cerr<<"No exact FP32 mode matches actual legacy channel "<<c<<"; optimization rejected\n";return false;}
            math_modes_[c]=candidates[c].front();
        }
        math_=math_modes_[0];
        std::cerr<<"Exact input calibrated Y/U/V="<<math_modes_[0]<<'/'<<math_modes_[1]<<'/'<<math_modes_[2]<<" reference="<<(reference_==reference::PreprocessYuv420?"extracted-host":"actual-runner")<<'\n';
    }
    render(packed.data,sw,sh,packed.stride,dw,dh,g,math_modes_);return true;
}
void setPolicy(bool opt,bool capture){optimized.store(opt);capturing.store(capture);std::lock_guard<std::mutex> lock(audit_mutex);audits.clear();}
bool optimizedPolicy() noexcept{return optimized.load();}
DetectorWorkspace& detectorWorkspace(){static DetectorWorkspace workspace;return workspace;}
bool capturePolicy() noexcept{return capturing.load();}
DetectorWorkspace& segmentationWorkspace(){static DetectorWorkspace workspace;return workspace;}
void captureTensor(const char* tag,const std::vector<std::uint8_t>& prepared,const void* data,std::size_t bytes,int w,int h,int left,int top,int rw,int rh) {
    if(!capturing.load())return;
    TensorAudit t;t.prepared=prepared;const auto* start=static_cast<const std::uint8_t*>(data);t.storage.assign(start,start+bytes);
    t.width=w;t.height=h;t.left=left;t.top=top;t.valid_width=rw;t.valid_height=rh;
    std::lock_guard<std::mutex> lock(audit_mutex);audits[tag]=std::move(t);
}
std::map<std::string,TensorAudit> takeAudit(){std::lock_guard<std::mutex> lock(audit_mutex);return std::move(audits);}
void captureOutput(const char* tag,const void* data,std::size_t size,const std::string& metadata,const std::vector<float>* logits) {
    if(!capturing.load())return;
    std::lock_guard<std::mutex> lock(audit_mutex);auto& audit=audits[tag];
    if(data&&size){const auto* p=static_cast<const std::uint8_t*>(data);audit.output_storage.assign(p,p+size);}
    audit.output_metadata=metadata;if(logits)audit.decoded_logits=*logits;
}
}
