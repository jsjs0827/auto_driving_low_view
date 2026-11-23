#include "seg_post.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <limits>
#include <stdexcept>
int main(){try{
    std::mt19937 rng(20);r20::SegPostWorkspace ws;std::vector<uint8_t> oldmask,newmask;std::size_t cases=0,fallback=0;
    for(int nc:{1,3,9})for(bool align:{true,false})for(int resize:{0,1})for(int pattern=0;pattern<7;++pattern)for(bool large:{false,true}) {
        const int ow=large?104:7,oh=large?60:5,dw=large?1280:32,dh=large?1088:24;
        std::vector<float> chw(std::size_t(nc)*ow*oh);
        for(int c=0;c<nc;++c)for(int y=0;y<oh;++y)for(int x=0;x<ow;++x){float v=0;
            if(pattern==0)v=float(int(rng()%2001)-1000)/113.f;
            if(pattern==1)v=c==nc-1?7.f:-5.f;
            if(pattern==2)v=((x+y)%nc)==c?1.f:0.f;
            if(pattern==3)v=1.f;
            if(pattern==4)v=std::nextafter(1.f,c%2?2.f:0.f)+float((x+y)%3)*1.e-7f;
            if(pattern==5)v=((x*y+c)%17==0)?std::numeric_limits<float>::quiet_NaN():float(c);
            if(pattern==6)v=((x+y+c)%7==0)?std::numeric_limits<float>::infinity():float(c-x%3);
            chw[(std::size_t(c)*oh+y)*ow+x]=v;
        }
        const auto g=r20::originalGeometry(dw,dh,832,480,resize);
        r20::referenceMask(chw,nc,oh,ow,480,832,dh,dw,resize,g,align,oldmask);
        ws.run(chw,nc,oh,ow,480,832,dh,dw,resize,g,align,newmask);
        if(oldmask!=newmask)throw std::runtime_error("postprocess differs, case "+std::to_string(cases));
        if(ws.stats.filled_pixels+ws.stats.shortcut_pixels+ws.stats.interpolated_pixels!=newmask.size())throw std::runtime_error("statistics lost pixels");
        fallback+=ws.stats.interpolated_pixels;++cases;
    }
    if(!fallback)throw std::runtime_error("tests did not exercise complete interpolation");
    std::cout<<"PASS "<<cases<<" original postprocess cases, zero differing pixels; boundary pixels exercised="<<fallback<<"\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
