// Standalone regression: independent explicit FP32/FMA oracle; no BPU work.
// Compile with strict/no implicit contraction; link exact_input, legacy_exact,
// and legacy_reference from the project under test. Defaults to 9 uniform modes plus 3 mixed channel cases.
#include "exact_input.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

namespace {
std::array<int,3> requested{{4,4,4}};
float fpMultiply(float a,float b){volatile float v=a*b;return v;}
float fpAdd(float a,float b){volatile float v=a+b;return v;}
float fpDivide(float a,float b){volatile float v=a/b;return v;}
float blend(float a,float b,float w,int mode){
    const float iw=fpAdd(1.f,-w),pa=fpMultiply(iw,a),pb=fpMultiply(w,b);
    if(mode==1)return std::fma(iw,a,pb);
    if(mode==2)return std::fma(w,b,pa);
    return fpAdd(pa,pb);
}
float position(int i,int source,int target){
    const float p=fpAdd(fpDivide(fpMultiply(fpAdd(float(i),.5f),float(source)),float(target)),-.5f);
    return std::max(0.f,std::min(p,float(source-1)));
}
bool explicitOracle(const uint8_t* data,int sw,int sh,int stride,std::size_t bytes,bool nv12,
    int dw,int dh,int resize,r20::Geometry* g,std::vector<uint8_t>& out,uint8_t pad){
    if(!nv12||!data||stride<sw||bytes<std::size_t(stride)*sh*3/2)return false;
    *g=r20::originalGeometry(sw,sh,dw,dh,resize);
    const auto plane=std::size_t(dw)*dh;
    out.assign(plane*3/2,128);std::fill(out.begin(),out.begin()+plane,pad);
    for(int c=0;c<3;++c){
        const bool uv=c!=0;const int source_w=uv?sw/2:sw,source_h=uv?sh/2:sh;
        const int target_w=uv?g->valid_w/2:g->valid_w,target_h=uv?g->valid_h/2:g->valid_h;
        const auto* src=data+(uv?std::size_t(stride)*sh:0);const int channels=uv?2:1,ch=uv?c-1:0;
        for(int y=0;y<target_h;++y){
            const float sy=position(y,source_h,target_h);const int y0=int(std::floor(sy)),y1=std::min(y0+1,source_h-1);const float wy=fpAdd(sy,-float(y0));
            for(int x=0;x<target_w;++x){
                const float sx=position(x,source_w,target_w);const int x0=int(std::floor(sx)),x1=std::min(x0+1,source_w-1);const float wx=fpAdd(sx,-float(x0));
                const auto sample=[&](int xx,int yy){return float(src[std::size_t(yy)*stride+xx*channels+ch]);};
                const int m=requested[c];const float a=blend(sample(x0,y0),sample(x1,y0),wx,m%3),b=blend(sample(x0,y1),sample(x1,y1),wx,m%3);
                float v=fpAdd(blend(a,b,wy,m/3),.5f);if(uv)v=std::max(0.f,std::min(255.f,v));
                out[(uv?plane:0)+std::size_t(y+(uv?g->y_shift/2:g->y_shift))*dw+g->x_shift+x*channels+ch]=uint8_t(v);
            }
        }
    }
    return true;
}
void probe(std::vector<uint8_t>& source,int stride,int kind){
    std::fill(source.begin(),source.end(),uint8_t(127));const auto uv=std::size_t(stride)*544;
    // Detector Y and UV have the same scale ratios; each independent
    // half-tie fixture therefore also exercises the luma arithmetic.
    const auto set=[&](int x,int y,int value){
        source[std::size_t(y)*stride+x]=uint8_t(value);
        for(int c=0;c<2;++c)source[uv+std::size_t(y)*stride+2*x+c]=uint8_t(value);
    };
    if(kind==0){set(26,144,90);set(27,144,99);}
    else if(kind==1){set(26,171,121);set(27,171,131);set(26,172,122);set(27,172,121);}
    else {set(38,49,129);set(39,49,134);set(38,50,122);set(39,50,125);}
}
std::size_t differences(const std::vector<uint8_t>& expected,const uint8_t* actual,std::size_t& first){
    std::size_t count=0;first=0;for(std::size_t i=0;i<expected.size();++i)if(expected[i]!=actual[i]){if(!count)first=i;++count;}return count;
}
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
}
int main(int argc,char** argv){try{
    const bool only_four=argc>1&&std::string(argv[1])=="--mode4";
    std::size_t checks=0,failures=0;int contexts=0,mode_cases=0;
    const std::array<std::array<int,3>,12> mode_set{{
        {{0,0,0}},{{1,1,1}},{{2,2,2}},{{3,3,3}},{{4,4,4}},{{5,5,5}},
        {{6,6,6}},{{7,7,7}},{{8,8,8}},{{0,4,3}},{{4,3,4}},{{2,5,8}}
    }};
    r22::setVariant(r22::Variant::CompactDirect);
    for(const auto& modes:mode_set){if(only_four&&modes!=std::array<int,3>{{4,4,4}})continue;
        requested=modes;std::mt19937 random(26);++mode_cases;
        for(int stride:{640,656})for(int lane:{0,1}){
            const int dw=lane?832:864,dh=480,resize=lane?0:1;const auto need=std::size_t(dw)*dh*3/2;
            std::vector<uint8_t> source(std::size_t(stride)*544*3/2),expected,destination(need+64,0xa5);
            r22::Workspace cached,direct;cached.setReference(explicitOracle);direct.setReference(explicitOracle);++contexts;
            for(int frame=0;frame<13;++frame){
                if(frame<5){for(auto& v:source)v=frame==0?0:frame==1?255:uint8_t(random());}
                else if(frame<7)probe(source,stride,frame-5);
                else if(frame==7){std::uint32_t state=0x26d00du;for(auto& v:source){state=state*1664525u+1013904223u;v=uint8_t(120+((state>>24)&15));}}
                else if(frame==8)probe(source,stride,2);
                else {
                    // Held-out streams differ from every startup calibration fixture.
                    // Only logical visible bytes advance the PRNG; both strides carry
                    // identical images and padding is independently poisoned.
                    const std::uint32_t seeds[4]={0x26cafe01u,0x17bd9035u,0x8114a733u,0x3901eabdu};
                    const int test=frame-9;std::uint32_t state=seeds[test];std::fill(source.begin(),source.end(),uint8_t(0xa5));
                    for(int y=0;y<544*3/2;++y)for(int x=0;x<640;++x){state=state*1664525u+1013904223u;
                        const auto v=std::uint8_t(state>>24);source[std::size_t(y)*stride+x]=test==0?v:test==1?uint8_t(v&15):test==2?uint8_t(120+(v&15)):uint8_t(240+(v&15));}
                }
                r20::Geometry expected_geometry,g;
                require(explicitOracle(source.data(),640,544,stride,source.size(),true,dw,dh,resize,&expected_geometry,expected,127),"Independent oracle rejected fixture");
                require(cached.prepare(source.data(),640,544,stride,source.size(),true,dw,dh,resize,g),"Cached prepare rejected fixture");
                require(direct.prepare(source.data(),640,544,stride,source.size(),true,dw,dh,resize,g,destination.data()+16,need+32),"Direct prepare rejected fixture");
                for(int route=0;route<2;++route){
                    const auto* actual=route?destination.data()+16:cached.output.data();std::size_t first=0;
                    const auto different=differences(expected,actual,first);++checks;
                    if(different){++failures;std::cerr<<"MISMATCH oracle="<<requested[0]<<'/'<<requested[1]<<'/'<<requested[2]<<" stride="<<stride<<" lane="<<lane<<" frame="<<frame<<" route="<<(route?"direct":"cached")<<" count="<<different<<" first="<<first<<" expected="<<int(expected[first])<<" actual="<<int(actual[first])<<" selected=";for(auto m:direct.mathModes())std::cerr<<m<<'/';std::cerr<<'\n';}
                }
                require(std::all_of(destination.begin(),destination.begin()+16,[](uint8_t v){return v==0xa5;})&&std::all_of(destination.end()-16,destination.end(),[](uint8_t v){return v==0xa5;}),"Direct write overran guards");
                require(std::all_of(destination.begin()+16+need,destination.end()-16,[](uint8_t v){return v==0;}),"Aligned input tail not zero");
            }
        }
    }
    std::cout<<"EXPLICIT_FMA_REGRESSION contexts="<<contexts<<" checks="<<checks<<" failures="<<failures<<" modes="<<mode_cases<<" NEON="<<r20::neonEnabled()<<" (host synthetic oracle, NOT BPU)\n";
    return failures?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
