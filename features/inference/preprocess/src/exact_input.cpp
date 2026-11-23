#include "exact_input.hpp"
#include <algorithm>
#include <cfenv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#if defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#define R22_NEON 1
#endif
namespace r22 {
namespace {
std::atomic<Variant> selected{Variant::Direct};
std::atomic<bool> profile_enabled{false};
std::atomic<std::uint64_t> source_context{0};
using Clock=std::chrono::steady_clock;
template<int Mode>float mix(float a,float b,float iw,float w) {
    if constexpr(Mode==1)return std::fma(iw,a,w*b);
    if constexpr(Mode==2)return std::fma(w,b,iw*a);
    return iw*a+w*b;
}
float mixDynamic(float a,float b,float iw,float w,int mode) {
    if(mode==1)return mix<1>(a,b,iw,w);
    if(mode==2)return mix<2>(a,b,iw,w);
    return mix<0>(a,b,iw,w);
}
#ifdef R22_NEON
template<int Mode>float32x4_t mix4(float32x4_t a,float32x4_t b,float32x4_t iw,float32x4_t w) {
    if constexpr(Mode==1)return vfmaq_f32(vmulq_f32(w,b),iw,a);
    if constexpr(Mode==2)return vfmaq_f32(vmulq_f32(iw,a),w,b);
    return vaddq_f32(vmulq_f32(iw,a),vmulq_f32(w,b));
}
float32x4_t mixDynamic4(float32x4_t a,float32x4_t b,float32x4_t iw,float32x4_t w,int mode) {
    if(mode==1)return mix4<1>(a,b,iw,w);
    if(mode==2)return mix4<2>(a,b,iw,w);
    return mix4<0>(a,b,iw,w);
}
struct Floats8 {float32x4_t lo,hi;};
Floats8 widen(uint8x8_t bytes) {
    const auto words=vmovl_u8(bytes);
    return {vcvtq_f32_u32(vmovl_u16(vget_low_u16(words))),vcvtq_f32_u32(vmovl_u16(vget_high_u16(words)))};
}
#endif
}
Variant variant() noexcept{return selected.load();}
bool compactVariant(Variant v) noexcept{return v==Variant::CompactCached||v==Variant::CompactDirect;}
bool directVariant(Variant v) noexcept{return v==Variant::Direct||v==Variant::CompactDirect;}
void setProfiling(bool enabled) noexcept{profile_enabled.store(enabled);}
bool profiling() noexcept{return profile_enabled.load();}
std::uint64_t originalSourceContext() noexcept{return source_context.load();}
void setOriginalSourceContext(std::uint64_t context) noexcept{source_context.store(context);}
void setVariant(Variant v) noexcept{selected.store(v);}
Variant parseVariant(const std::string& s) {
    if(s=="r21")return Variant::R21;
    if(s=="cached")return Variant::Cached;
    if(s=="direct")return Variant::Direct;
    if(s=="compact_cached")return Variant::CompactCached;
    if(s=="compact_direct")return Variant::CompactDirect;
    throw std::invalid_argument("--preprocess must be r21, cached, direct, compact_cached or compact_direct");
}
const char* variantName(Variant v) noexcept{switch(v){case Variant::R21:return "r21";case Variant::Cached:return "cached";case Variant::Direct:return "direct";case Variant::CompactCached:return "compact_cached";default:return "compact_direct";}}
Workspace& workspace(int lane){static Workspace det,seg;return lane==0?det:seg;}
InputStats& stats(int lane){return workspace(lane).stats;}
void resetStats(){stats(0)={};stats(1)={};}
void invalidateDestination(int lane){workspace(lane).invalidateDestination();}

void Workspace::maps(int sw,int sh,int rw,int rh) {
    const auto fill=[](std::vector<Axis>& a,int source,int target) {
        a.resize(target);
        const r20::V0SamplingAxis sampling(source,target);
        for(int i=0;i<target;++i) {
            float p=sampling.coordinate(i);
            p=std::max(0.f,std::min(p,float(source-1)));
            const int lo=int(std::floor(p));const float w=p-float(lo);
            a[i]={lo,std::min(lo+1,source-1),w,1.f-w};
        }
    };
    const auto blocks=[&](const std::vector<Axis>& xs,int channels,std::vector<Block>& dst) {
        dst.clear();dst.reserve(xs.size()*channels/8);
        for(int i=0;i+8<=int(xs.size())*channels;i+=8) {
            Block b;
            for(int k=0;k<8;++k) {
                const auto& x=xs[(i+k)/channels];const int ch=(i+k)%channels;
                b.a[k]=x.a*channels+ch;b.b[k]=x.b*channels+ch;b.w[k]=x.w;b.iw[k]=x.iw;
            }
            b.base=b.a[0];
            // Both 16-byte and 32-byte tables are bounded by the compact source row.
            // 32 bytes covers the 1280 -> 564 detector reduction without scalar gathers.
            int max_offset=0;
            for(int k=0;k<8;++k) {
                max_offset=std::max(max_offset,std::max(b.a[k],b.b[k])-b.base);
                b.ia[k]=std::uint8_t(b.a[k]-b.base);b.ib[k]=std::uint8_t(b.b[k]-b.base);
            }
            b.table_bytes=max_offset<16&&b.base+16<=sw?16:max_offset<32&&b.base+32<=sw?32:0;
            dst.push_back(b);
        }
    };
    fill(lx_,sw,rw);fill(ly_,sh,rh);fill(cx_,sw/2,rw/2);fill(cy_,sh/2,rh/2);
    blocks(lx_,1,lb_);blocks(cx_,2,cb_);
    const auto compact=[](const std::vector<Block>& source,std::vector<CompactBlock>& target,std::vector<Gather>& gather) {
        target.clear();gather.clear();target.reserve(source.size());
        for(const auto& block:source){CompactBlock c;c.base=block.base;c.table_bytes=block.table_bytes;c.w=block.w;c.iw=block.iw;
            std::copy(block.ia.begin(),block.ia.end(),c.indices.begin());
            std::copy(block.ib.begin(),block.ib.end(),c.indices.begin()+8);
            if(!c.table_bytes){Gather g;for(int k=0;k<8;++k){g.positions[k]=std::uint16_t(block.a[k]);g.positions[k+8]=std::uint16_t(block.b[k]);}c.gather=int(gather.size());gather.push_back(g);}
            target.push_back(c);
        }
    };
    compact(lb_,clb_,lgb_);compact(cb_,ccb_,cgb_);
    ltable_=std::count_if(lb_.begin(),lb_.end(),[](const Block& b){return b.table_bytes!=0;});
    ctable_=std::count_if(cb_.begin(),cb_.end(),[](const Block& b){return b.table_bytes!=0;});
}

template<int XM,int YM,int Channels>
void Workspace::plane(const std::uint8_t* src,int stride,const std::vector<Axis>& xs,
                      const std::vector<Axis>& ys,const std::vector<Block>& blocks,
                      std::uint8_t* dst,int dw,int left,int top,int other_mode) {
    for(int y=0;y<int(ys.size());++y) {
        const auto ay=ys[y];const auto* a=src+std::size_t(ay.a)*stride;const auto* b=src+std::size_t(ay.b)*stride;
        auto* out=dst+std::size_t(y+top)*dw+left;int i=0;
        for(const auto& x:blocks) {
#ifdef R22_NEON
            uint8x8_t av,bv,cv,dv;
            if(x.table_bytes==16) {
                const auto ra=vld1q_u8(a+x.base),rb=vld1q_u8(b+x.base);
                const auto ia=vld1_u8(x.ia.data()),ib=vld1_u8(x.ib.data());
                av=vqtbl1_u8(ra,ia);bv=vqtbl1_u8(ra,ib);cv=vqtbl1_u8(rb,ia);dv=vqtbl1_u8(rb,ib);
            } else if(x.table_bytes==32) {
                uint8x16x2_t ra,rb;ra.val[0]=vld1q_u8(a+x.base);ra.val[1]=vld1q_u8(a+x.base+16);
                rb.val[0]=vld1q_u8(b+x.base);rb.val[1]=vld1q_u8(b+x.base+16);
                const auto ia=vld1_u8(x.ia.data()),ib=vld1_u8(x.ib.data());
                av=vqtbl2_u8(ra,ia);bv=vqtbl2_u8(ra,ib);cv=vqtbl2_u8(rb,ia);dv=vqtbl2_u8(rb,ib);
            } else {
                std::uint8_t aa[8],bb[8],cc[8],dd[8];
                for(int k=0;k<8;++k){aa[k]=a[x.a[k]];bb[k]=a[x.b[k]];cc[k]=b[x.a[k]];dd[k]=b[x.b[k]];}
                av=vld1_u8(aa);bv=vld1_u8(bb);cv=vld1_u8(cc);dv=vld1_u8(dd);
            }
            const auto af=widen(av),bf=widen(bv),cf=widen(cv),df=widen(dv);
            const auto wy=vdupq_n_f32(ay.w),iy=vdupq_n_f32(ay.iw);
            const auto lo_w=vld1q_f32(x.w.data()),lo_i=vld1q_f32(x.iw.data());
            const auto hi_w=vld1q_f32(x.w.data()+4),hi_i=vld1q_f32(x.iw.data()+4);
            auto lo=mix4<YM>(mix4<XM>(af.lo,bf.lo,lo_i,lo_w),mix4<XM>(cf.lo,df.lo,lo_i,lo_w),iy,wy);
            auto hi=mix4<YM>(mix4<XM>(af.hi,bf.hi,hi_i,hi_w),mix4<XM>(cf.hi,df.hi,hi_i,hi_w),iy,wy);
            if constexpr(Channels==2)if(other_mode>=0){
                const uint32_t bits[4]={0,0xffffffffu,0,0xffffffffu};const auto mask=vld1q_u32(bits);
                const int xm=other_mode%3,ym=other_mode/3;
                const auto vlo=mixDynamic4(mixDynamic4(af.lo,bf.lo,lo_i,lo_w,xm),mixDynamic4(cf.lo,df.lo,lo_i,lo_w,xm),iy,wy,ym);
                const auto vhi=mixDynamic4(mixDynamic4(af.hi,bf.hi,hi_i,hi_w,xm),mixDynamic4(cf.hi,df.hi,hi_i,hi_w,xm),iy,wy,ym);
                lo=vbslq_f32(mask,vlo,lo);hi=vbslq_f32(mask,vhi,hi);
            }
            lo=vaddq_f32(lo,vdupq_n_f32(.5f));hi=vaddq_f32(hi,vdupq_n_f32(.5f));
            if constexpr(Channels==2) {
                lo=vmaxq_f32(vdupq_n_f32(0),vminq_f32(vdupq_n_f32(255),lo));
                hi=vmaxq_f32(vdupq_n_f32(0),vminq_f32(vdupq_n_f32(255),hi));
            }
            const auto words=vcombine_u16(vmovn_u32(vcvtq_u32_f32(lo)),vmovn_u32(vcvtq_u32_f32(hi)));
            vst1_u8(out+i,vmovn_u16(words));
#else
            // Portable implementation exercises exactly the precomputed table/gather addressing.
            for(int k=0;k<8;++k) {
                const int ai=x.table_bytes?x.base+x.ia[k]:x.a[k],bi=x.table_bytes?x.base+x.ib[k]:x.b[k];
                float t=mix<XM>(a[ai],a[bi],x.iw[k],x.w[k]);
                float u=mix<XM>(b[ai],b[bi],x.iw[k],x.w[k]);
                float v=mix<YM>(t,u,ay.iw,ay.w);
                if constexpr(Channels==2)if(other_mode>=0 && (k%2)){
                    t=mixDynamic(a[ai],a[bi],x.iw[k],x.w[k],other_mode%3);
                    u=mixDynamic(b[ai],b[bi],x.iw[k],x.w[k],other_mode%3);
                    v=mixDynamic(t,u,ay.iw,ay.w,other_mode/3);
                }
                v+=.5f;
                if constexpr(Channels==2)v=std::max(0.f,std::min(255.f,v));
                out[i+k]=std::uint8_t(v);
            }
#endif
            i+=8;
        }
        for(;i<int(xs.size())*Channels;++i) {
            const auto x=xs[i/Channels];const int ch=i%Channels;
            float t=mix<XM>(a[x.a*Channels+ch],a[x.b*Channels+ch],x.iw,x.w);
            float u=mix<XM>(b[x.a*Channels+ch],b[x.b*Channels+ch],x.iw,x.w);
            float v=mix<YM>(t,u,ay.iw,ay.w);
            if constexpr(Channels==2)if(other_mode>=0 && ch){
                t=mixDynamic(a[x.a*Channels+ch],a[x.b*Channels+ch],x.iw,x.w,other_mode%3);
                u=mixDynamic(b[x.a*Channels+ch],b[x.b*Channels+ch],x.iw,x.w,other_mode%3);
                v=mixDynamic(t,u,ay.iw,ay.w,other_mode/3);
            }
            v+=.5f;
            if constexpr(Channels==2)v=std::max(0.f,std::min(255.f,v));
            out[i]=std::uint8_t(v);
        }
    }
}
template<int XM,int YM,int Channels>
void Workspace::compactPlane(const std::uint8_t* src,int stride,const std::vector<Axis>& xs,
                      const std::vector<Axis>& ys,const std::vector<CompactBlock>& blocks,const std::vector<Gather>& gathers,
                      std::uint8_t* dst,int dw,int left,int top,int other_mode) {
    for(int y=0;y<int(ys.size());++y) {
        const auto ay=ys[y];const auto* a=src+std::size_t(ay.a)*stride;const auto* b=src+std::size_t(ay.b)*stride;
        auto* out=dst+std::size_t(y+top)*dw+left;int i=0;
        for(const auto& x:blocks) {
#ifdef R22_NEON
            uint8x8_t av,bv,cv,dv;
            if(x.table_bytes==16) {
                const auto indices=vld1q_u8(x.indices.data());
                const auto ar=vqtbl1q_u8(vld1q_u8(a+x.base),indices);
                const auto br=vqtbl1q_u8(vld1q_u8(b+x.base),indices);
                av=vget_low_u8(ar);bv=vget_high_u8(ar);cv=vget_low_u8(br);dv=vget_high_u8(br);
            } else if(x.table_bytes==32) {
                uint8x16x2_t ra,rb;ra.val[0]=vld1q_u8(a+x.base);ra.val[1]=vld1q_u8(a+x.base+16);
                rb.val[0]=vld1q_u8(b+x.base);rb.val[1]=vld1q_u8(b+x.base+16);
                const auto indices=vld1q_u8(x.indices.data());
                const auto ar=vqtbl2q_u8(ra,indices),br=vqtbl2q_u8(rb,indices);
                av=vget_low_u8(ar);bv=vget_high_u8(ar);cv=vget_low_u8(br);dv=vget_high_u8(br);
            } else {
                const auto& positions=gathers[std::size_t(x.gather)].positions;
                std::uint8_t aa[8],bb[8],cc[8],dd[8];
                for(int k=0;k<8;++k){aa[k]=a[positions[k]];bb[k]=a[positions[k+8]];cc[k]=b[positions[k]];dd[k]=b[positions[k+8]];}
                av=vld1_u8(aa);bv=vld1_u8(bb);cv=vld1_u8(cc);dv=vld1_u8(dd);
            }
            const auto af=widen(av),bf=widen(bv),cf=widen(cv),df=widen(dv);
            const auto wy=vdupq_n_f32(ay.w),iy=vdupq_n_f32(ay.iw);
            const auto lo_w=vld1q_f32(x.w.data()),lo_i=vld1q_f32(x.iw.data());
            const auto hi_w=vld1q_f32(x.w.data()+4),hi_i=vld1q_f32(x.iw.data()+4);
            auto lo=mix4<YM>(mix4<XM>(af.lo,bf.lo,lo_i,lo_w),mix4<XM>(cf.lo,df.lo,lo_i,lo_w),iy,wy);
            auto hi=mix4<YM>(mix4<XM>(af.hi,bf.hi,hi_i,hi_w),mix4<XM>(cf.hi,df.hi,hi_i,hi_w),iy,wy);
            if constexpr(Channels==2)if(other_mode>=0){
                const uint32_t bits[4]={0,0xffffffffu,0,0xffffffffu};const auto mask=vld1q_u32(bits);
                const int xm=other_mode%3,ym=other_mode/3;
                const auto vlo=mixDynamic4(mixDynamic4(af.lo,bf.lo,lo_i,lo_w,xm),mixDynamic4(cf.lo,df.lo,lo_i,lo_w,xm),iy,wy,ym);
                const auto vhi=mixDynamic4(mixDynamic4(af.hi,bf.hi,hi_i,hi_w,xm),mixDynamic4(cf.hi,df.hi,hi_i,hi_w,xm),iy,wy,ym);
                lo=vbslq_f32(mask,vlo,lo);hi=vbslq_f32(mask,vhi,hi);
            }
            lo=vaddq_f32(lo,vdupq_n_f32(.5f));hi=vaddq_f32(hi,vdupq_n_f32(.5f));
            if constexpr(Channels==2) {
                lo=vmaxq_f32(vdupq_n_f32(0),vminq_f32(vdupq_n_f32(255),lo));
                hi=vmaxq_f32(vdupq_n_f32(0),vminq_f32(vdupq_n_f32(255),hi));
            }
            const auto words=vcombine_u16(vmovn_u32(vcvtq_u32_f32(lo)),vmovn_u32(vcvtq_u32_f32(hi)));
            vst1_u8(out+i,vmovn_u16(words));
#else
            // Portable implementation exercises exactly the precomputed table/gather addressing.
            for(int k=0;k<8;++k) {
                const int ai=x.table_bytes?x.base+x.indices[k]:gathers[std::size_t(x.gather)].positions[k],bi=x.table_bytes?x.base+x.indices[k+8]:gathers[std::size_t(x.gather)].positions[k+8];
                float t=mix<XM>(a[ai],a[bi],x.iw[k],x.w[k]);
                float u=mix<XM>(b[ai],b[bi],x.iw[k],x.w[k]);
                float v=mix<YM>(t,u,ay.iw,ay.w);
                if constexpr(Channels==2)if(other_mode>=0 && (k%2)){
                    t=mixDynamic(a[ai],a[bi],x.iw[k],x.w[k],other_mode%3);
                    u=mixDynamic(b[ai],b[bi],x.iw[k],x.w[k],other_mode%3);
                    v=mixDynamic(t,u,ay.iw,ay.w,other_mode/3);
                }
                v+=.5f;
                if constexpr(Channels==2)v=std::max(0.f,std::min(255.f,v));
                out[i+k]=std::uint8_t(v);
            }
#endif
            i+=8;
        }
        for(;i<int(xs.size())*Channels;++i) {
            const auto x=xs[i/Channels];const int ch=i%Channels;
            float t=mix<XM>(a[x.a*Channels+ch],a[x.b*Channels+ch],x.iw,x.w);
            float u=mix<XM>(b[x.a*Channels+ch],b[x.b*Channels+ch],x.iw,x.w);
            float v=mix<YM>(t,u,ay.iw,ay.w);
            if constexpr(Channels==2)if(other_mode>=0 && ch){
                t=mixDynamic(a[x.a*Channels+ch],a[x.b*Channels+ch],x.iw,x.w,other_mode%3);
                u=mixDynamic(b[x.a*Channels+ch],b[x.b*Channels+ch],x.iw,x.w,other_mode%3);
                v=mixDynamic(t,u,ay.iw,ay.w,other_mode/3);
            }
            v+=.5f;
            if constexpr(Channels==2)v=std::max(0.f,std::min(255.f,v));
            out[i]=std::uint8_t(v);
        }
    }
}
template<int Channels>
void Workspace::dispatchPlane(int mode,const std::uint8_t* src,int stride,const std::vector<Axis>& xs,
    const std::vector<Axis>& ys,const std::vector<Block>& blocks,std::uint8_t* dst,int dw,int left,int top,int other) {
#define R22_DISPATCH(M,X,Y) case M:if(compactVariant(variant()))compactPlane<X,Y,Channels>(src,stride,xs,ys,Channels==1?clb_:ccb_,Channels==1?lgb_:cgb_,dst,dw,left,top,other);else plane<X,Y,Channels>(src,stride,xs,ys,blocks,dst,dw,left,top,other);break
    switch(mode){
        R22_DISPATCH(0,0,0);R22_DISPATCH(1,1,0);R22_DISPATCH(2,2,0);
        R22_DISPATCH(3,0,1);R22_DISPATCH(4,1,1);R22_DISPATCH(5,2,1);
        R22_DISPATCH(6,0,2);R22_DISPATCH(7,1,2);R22_DISPATCH(8,2,2);
        default:throw std::runtime_error("Exact channel FMA mode not initialized");
    }
#undef R22_DISPATCH
}
void Workspace::render(const std::uint8_t* src,int sw,int sh,int dw,int dh,const r20::Geometry& g,std::uint8_t* dst) {
    const bool measured=profiling();auto begin=measured?Clock::now():Clock::time_point{};
    const std::array<int,6> key{{dw,dh,g.x_shift,g.y_shift,g.valid_w,g.valid_h}};
    const bool padded=g.x_shift||g.y_shift||g.valid_w!=dw||g.valid_h!=dh;
    if(padded && (!padding_valid_||dst!=last_destination_||key!=padding_key_)) {
        const auto plane_bytes=std::size_t(dw)*dh;
        std::memset(dst,127,plane_bytes);std::memset(dst+plane_bytes,128,plane_bytes/2);
        stats.padding_bytes=plane_bytes*3/2;
    }
    padding_valid_=true;last_destination_=dst;padding_key_=key;
    if(measured){stats.padding_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();begin=Clock::now();}
    dispatchPlane<1>(math_modes_[0],src,sw,lx_,ly_,lb_,dst,dw,g.x_shift,g.y_shift);
    if(measured){stats.y_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();begin=Clock::now();}
    dispatchPlane<2>(math_modes_[1],src+std::size_t(sw)*sh,sw,cx_,cy_,cb_,dst+std::size_t(dw)*dh,dw,g.x_shift,g.y_shift/2,
                     math_modes_[1]==math_modes_[2]?-1:math_modes_[2]);
    if(measured)stats.uv_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
}
bool Workspace::prepare(const std::uint8_t* src,int sw,int sh,int stride,std::size_t bytes,bool nv12,
                        int dw,int dh,int resize,r20::Geometry& g,std::uint8_t* destination,std::size_t capacity) {
    stats={};const auto start=Clock::now();
    const bool measured=profiling();auto phase=start;
    const auto packed=pack_.run(src,sw,sh,stride,bytes,nv12);
    if(measured){stats.pack_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();phase=Clock::now();}
    g=r20::originalGeometry(sw,sh,dw,dh,resize);
    const auto need=std::size_t(dw)*dh*3/2;
    if(destination && capacity<need)throw std::invalid_argument("BPU input capacity is smaller than exact NV12 image");
    const bool direct=destination!=nullptr;
    if(direct) {
        const auto src_address=reinterpret_cast<std::uintptr_t>(packed.data),dst_address=reinterpret_cast<std::uintptr_t>(destination);
        if(dst_address<src_address+packed.bytes && src_address<dst_address+need)throw std::invalid_argument("Exact resize requires separate source and destination storage");
    }
    const std::array<int,5> key{{sw,sh,g.valid_w,g.valid_h,std::fegetround()}};
    const bool changed=!mapped_||map_key_!=key;
    if(changed) {
        r20::Geometry old;
        if(!oracle_.prepare(src,sw,sh,stride,bytes,nv12,dw,dh,resize,old))return false;
        if(measured){stats.calibration_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();phase=Clock::now();}
        math_=oracle_.mathMode();math_modes_=oracle_.mathModes();maps(sw,sh,g.valid_w,g.valid_h);
        if(measured)stats.map_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();
        map_key_=key;mapped_=true;stats.initialized=true;
    }
    if(!destination){output.resize(need);destination=output.data();capacity=need;}
    render(packed.data,sw,sh,dw,dh,g,destination);
    phase=measured?Clock::now():Clock::time_point{};
    if(capacity>need)std::memset(destination+need,0,capacity-need);
    stats.tail_bytes=capacity-need;
    if(measured){stats.tail_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();phase=Clock::now();}
    if(changed && (oracle_.output.size()!=need||std::memcmp(destination,oracle_.output.data(),need))) {
        mapped_=false;padding_valid_=false;throw std::runtime_error("r22 first-frame exact input acceptance failed");
    }
    stats.table_blocks=ltable_*ly_.size()+ctable_*cy_.size();
    stats.gather_blocks=(lb_.size()-ltable_)*ly_.size()+(cb_.size()-ctable_)*cy_.size();
    stats.block_bytes=compactVariant(variant())?(clb_.size()+ccb_.size())*sizeof(CompactBlock)+(lgb_.size()+cgb_.size())*sizeof(Gather):(lb_.size()+cb_.size())*sizeof(Block);
    if(measured)stats.account_ms=std::chrono::duration<double,std::milli>(Clock::now()-phase).count();
    stats.direct=direct;stats.math_mode=math_;stats.math_modes=math_modes_;
    stats.prepare_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
    return true;
}
void captureInput(const char* tag,View view,const void* storage,std::size_t capacity,int w,int h,const r20::Geometry& g) {
    if(!r20::capturePolicy())return;
    const std::vector<std::uint8_t> prepared(view.data,view.data+view.bytes);
    r20::captureTensor(tag,prepared,storage,capacity,w,h,g.x_shift,g.y_shift,g.valid_w,g.valid_h);
}
}
