#include "seg_post.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace r20 {
namespace {
void coarse(const std::vector<float>& chw,int nc,int h,int w,std::vector<uint8_t>& mask) {
    const auto plane=std::size_t(h)*w;
    if(nc<1 || nc>256 || h<1 || w<1 || chw.size()!=std::size_t(nc)*plane) throw std::invalid_argument("Invalid segmentation logits");
    mask.resize(plane);
    for(std::size_t p=0;p<plane;++p){int best=0;float v=chw[p];for(int c=1;c<nc;++c){const float a=chw[std::size_t(c)*plane+p];if(a>v){v=a;best=c;}}mask[p]=uint8_t(best);}
}
}
#include "seg_post_reference.inc"
void SegPostWorkspace::run(const std::vector<float>& chw,int nc,int oh,int ow,int ih,int iw,int dh,int dw,int resize,const Geometry& g,bool align,std::vector<uint8_t>& mask) {
    if(dw<1 || dh<1 || dw>16384 || dh>16384) throw std::invalid_argument("Invalid mask size");
    coarse(chw,nc,oh,ow,coarse_);
    const std::array<int,12> key{{oh,ow,ih,iw,dh,dw,resize,g.x_shift,g.y_shift,g.valid_w,g.valid_h,int(align)}};
    if(!cached_ || key!=key_) {
        key_=key;cached_=true;x_.resize(dw);y_.resize(dh);runs_.clear();
        // Keep the original separate coordinate-loop expressions.  In the
        // shared library ResizeCoord must also have internal linkage, as in
        // the original runner, so fast-math can hoist the same reciprocals.
        // A generic out-of-line axis call changes one-ULP weights at ties.
        for(int x=0;x<dw;++x) {
            float ox=ResizeCoord(x,dw,ow,align);
            if(resize!=0) {
                const float valid_x=ResizeCoord(x,dw,g.valid_w,align);
                const float input_x=g.x_shift+valid_x;
                ox=(iw>1&&ow>1)?input_x*(ow-1)/static_cast<float>(iw-1):0.f;
            }
            ox=std::max(0.f,std::min(static_cast<float>(ow-1),ox));
            const int x0=static_cast<int>(std::floor(ox));
            x_[x]={x0,std::min(ow-1,x0+1),ox-x0};
        }
        for(int y=0;y<dh;++y) {
            float oy=ResizeCoord(y,dh,oh,align);
            if(resize!=0) {
                const float valid_y=ResizeCoord(y,dh,g.valid_h,align);
                const float input_y=g.y_shift+valid_y;
                oy=(ih>1&&oh>1)?input_y*(oh-1)/static_cast<float>(ih-1):0.f;
            }
            oy=std::max(0.f,std::min(static_cast<float>(oh-1),oy));
            const int y0=static_cast<int>(std::floor(oy));
            y_[y]={y0,std::min(oh-1,y0+1),oy-y0};
        }
        for(int x=0;x<dw;){const int start=x;const auto a=x_[x];while(x<dw&&x_[x].a==a.a&&x_[x].b==a.b)++x;runs_.push_back({start,x,a.a,a.b});}
    }
    stats={};mask.resize(std::size_t(dw)*dh);const auto plane=std::size_t(ow)*oh;
    for(int y=0;y<dh;++y){const auto ay=y_[y];const float wy=ay.w;const auto row0=std::size_t(ay.a)*ow,row1=std::size_t(ay.b)*ow;auto* dst=mask.data()+std::size_t(y)*dw;
        for(const auto& run:runs_){const int x0=run.a,x1=run.b;
            const uint8_t c00=coarse_[row0+x0],c01=coarse_[row0+x1],c10=coarse_[row1+x0],c11=coarse_[row1+x1];
            // Exactly the reference shortcuts, hoisted over pixels sharing the same corners.
            if((c00==c01&&c00==c10&&c00==c11)||(wy==0.f&&c00==c01)){
                std::memset(dst+run.begin,c00,std::size_t(run.end-run.begin));stats.filled_pixels+=run.end-run.begin;continue;
            }
            for(int x=run.begin;x<run.end;++x){const float wx=x_[x].w;
                if((wx==0.f&&wy==0.f)||(wx==0.f&&c00==c10)){dst[x]=c00;++stats.shortcut_pixels;continue;}
                int best=0;float best_val=-std::numeric_limits<float>::infinity();
                for(int c=0;c<nc;++c){const auto* p=chw.data()+std::size_t(c)*plane;
                    const float v00=p[row0+x0],v01=p[row0+x1],v10=p[row1+x0],v11=p[row1+x1];
                    // Preserve the original FP32 horizontal-then-vertical FMA expression and > tie rule.
                    const float top=v00+(v01-v00)*wx;
                    const float bottom=v10+(v11-v10)*wx;
                    const float val=top+(bottom-top)*wy;
                    if(val>best_val){best_val=val;best=c;}
                }
                dst[x]=uint8_t(best);++stats.interpolated_pixels;
            }
        }
    }
}
SegPostWorkspace& segPostWorkspace(){static SegPostWorkspace workspace;return workspace;}
}
