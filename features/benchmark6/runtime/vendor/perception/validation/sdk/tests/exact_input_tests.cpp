#include "exact_input.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <cmath>
#include <memory>
#include <iostream>
#include <random>
#include <stdexcept>
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void geometry(const r20::Geometry& a,const r20::Geometry& b){
    require(a.x_scale==b.x_scale&&a.y_scale==b.y_scale&&a.x_shift==b.x_shift&&a.y_shift==b.y_shift&&a.valid_w==b.valid_w&&a.valid_h==b.valid_h,"Original geometry changed");
}
float independentlyMixed(float a,float b,float w,int m){
    const float iw=1.f-w;
    volatile float pa=iw*a,pb=w*b;
    if(m==1)return std::fma(iw,a,pb);
    if(m==2)return std::fma(w,b,pa);
    return pa+pb;
}
bool independentChannelOracle(const uint8_t* data,int sw,int sh,int stride,std::size_t bytes,bool nv12,
    int dw,int dh,int resize,r20::Geometry* g,std::vector<uint8_t>& out,uint8_t pad){
    r20::PackWorkspace pack;const auto input=pack.run(data,sw,sh,stride,bytes,nv12);
    *g=r20::originalGeometry(sw,sh,dw,dh,resize);const auto size=std::size_t(dw)*dh;
    out.assign(size*3/2,128);std::fill(out.begin(),out.begin()+size,pad);
    for(int c=0;c<3;++c){const int half=c!=0,source_w=sw>>(half?1:0),source_h=sh>>(half?1:0);
        const int target_w=g->valid_w>>(half?1:0),target_h=g->valid_h>>(half?1:0),channels=half?2:1;
        const int m=c==0?0:c==1?4:3;const auto* src=input.data+(half?std::size_t(sw)*sh:0);
        for(int y=0;y<target_h;++y)for(int x=0;x<target_w;++x){
            volatile float xp=(float(x)+.5f)*source_w,yp=(float(y)+.5f)*source_h;
            float sx=xp/target_w-.5f,sy=yp/target_h-.5f;
            sx=std::max(0.f,std::min(sx,float(source_w-1)));sy=std::max(0.f,std::min(sy,float(source_h-1)));
            const int x0=int(std::floor(sx)),x1=std::min(source_w-1,x0+1),y0=int(std::floor(sy)),y1=std::min(source_h-1,y0+1),ch=half?c-1:0;
            const float wx=sx-x0,wy=sy-y0;
            const auto sample=[&](int xx,int yy){return src[std::size_t(yy)*sw+xx*channels+ch];};
            const float t=independentlyMixed(sample(x0,y0),sample(x1,y0),wx,m%3),b=independentlyMixed(sample(x0,y1),sample(x1,y1),wx,m%3);
            float v=independentlyMixed(t,b,wy,m/3)+.5f;if(half)v=std::max(0.f,std::min(255.f,v));
            out[(half?size:0)+std::size_t(y+(half?g->y_shift/2:g->y_shift))*dw+g->x_shift+x*channels+ch]=uint8_t(v);
        }
    }
    return true;
}
}
int main(int argc,char** argv){try{
    if(argc>1){require(std::string(argv[1])=="--compact","Unknown test option");r22::setVariant(r22::Variant::CompactDirect);}
    r22::setProfiling(true);
    const std::array<std::array<int,4>,12> shapes{{{{1280,1088,864,480}},{{1280,1088,832,480}},{{640,544,864,480}},{{864,480,864,480}},{{640,544,832,480}},{{34,22,32,32}},{{30,18,32,30}},{{62,46,64,64}},{{126,94,128,96}},{{128,96,62,46}},{{3000,32,64,16}},{{16,16,8,8}}}};
    std::mt19937 random(22);std::size_t checks=0,table=0,gather=0,steady_no_padding=0;
    for(const auto& shape:shapes)for(int resize:{0,1})for(bool nv12:{true,false}) {
        const int sw=shape[0],sh=shape[1],dw=shape[2],dh=shape[3],stride=sw+8;
        // The extremely wide fixture only supports stretch (letterbox height would become zero).
        if(resize==1&&sw==3000)continue;
        const auto need=std::size_t(dw)*dh*3/2,cap=need+37;
        std::vector<uint8_t> source(std::size_t(stride)*sh*3/2),baseline,destination(cap+64,0xa5);
        r22::Workspace cached,direct;
        for(int frame=0;frame<4;++frame) {
            for(std::size_t i=0;i<source.size();++i)source[i]=frame==0?uint8_t(random()):frame==1?0:frame==2?255:uint8_t(i%256);
            r20::Geometry old,g;
            require(r20::reference::PreprocessYuv420(source.data(),sw,sh,stride,source.size(),nv12,dw,dh,resize,&old,baseline,127),"Reference prepare failed");
            require(cached.prepare(source.data(),sw,sh,stride,source.size(),nv12,dw,dh,resize,g),"Cached prepare failed");
            geometry(old,g);require(cached.output==baseline,"Cached input bytes differ");++checks;
            require(direct.prepare(source.data(),sw,sh,stride,source.size(),nv12,dw,dh,resize,g,destination.data()+32,cap),"Direct prepare failed");
            geometry(old,g);require(std::memcmp(destination.data()+32,baseline.data(),need)==0,"Direct input bytes differ");++checks;
            require(std::all_of(destination.begin()+32+need,destination.begin()+32+cap,[](uint8_t b){return b==0;}),"Aligned tensor tail not zero");
            require(std::all_of(destination.begin(),destination.begin()+32,[](uint8_t b){return b==0xa5;})&&std::all_of(destination.end()-32,destination.end(),[](uint8_t b){return b==0xa5;}),"Direct write exceeded capacity");
            require(direct.stats.direct&&!cached.stats.direct,"Destination ownership incorrect");
            if(frame>0){require(cached.stats.padding_bytes==0&&direct.stats.padding_bytes==0,"Steady frame unexpectedly refilled padding");++steady_no_padding;}
            if(resize==0)require(cached.stats.padding_bytes==0&&direct.stats.padding_bytes==0,"Stretch unexpectedly filled canvas");
            table+=direct.stats.table_blocks;gather+=direct.stats.gather_blocks;
        }
    }
    // Reuse both destination addresses across geometry, resize policy and source-stride changes.
    r22::Workspace reused;std::vector<uint8_t> source(128*96*3/2),destination(128*96*3/2+32,0xa5),baseline;
    for(int round=0;round<3;++round)for(const auto& shape:std::array<std::array<int,3>,5>{{{{64,64,1}},{{64,64,0}},{{62,46,1}},{{128,96,0}},{{64,64,1}}}}) {
        for(auto& b:source)b=uint8_t(random());r20::Geometry old,g;const auto need=std::size_t(shape[0])*shape[1]*3/2;
        require(r20::reference::PreprocessYuv420(source.data(),128,96,128,source.size(),true,shape[0],shape[1],shape[2],&old,baseline,127),"Reuse reference failed");
        require(reused.prepare(source.data(),128,96,128,source.size(),true,shape[0],shape[1],shape[2],g,destination.data(),need+13),"Reuse direct failed");
        geometry(old,g);require(std::memcmp(destination.data(),baseline.data(),need)==0,"Changed geometry retained stale padding");++checks;
        reused.invalidateDestination();
    }
    r20::Geometry g;bool rejected=false;
    try{reused.prepare(source.data(),128,96,128,source.size(),true,64,64,1,g,destination.data(),1);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Too-small capacity accepted");rejected=false;
    try{reused.prepare(source.data(),128,96,128,source.size(),true,64,64,1,g,source.data(),source.size());}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Overlapping source and target accepted");require(table&&gather,"Table/gather paths were not both exercised");
    // Rounding mode participates in the mapping/calibration key.
    const int saved_round=std::fegetround();
    for(int rounding:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO}) {
        if(std::fesetround(rounding))continue;
        for(auto& b:source)b=uint8_t(random());r20::Geometry old;
        require(r20::reference::PreprocessYuv420(source.data(),128,96,128,source.size(),true,64,64,1,&old,baseline,127),"Rounding reference failed");
        require(reused.prepare(source.data(),128,96,128,source.size(),true,64,64,1,g,destination.data(),64*64*3/2+13),"Rounding prepare failed");
        geometry(old,g);require(std::memcmp(destination.data(),baseline.data(),baseline.size())==0,"Rounding-mode input bytes differ");++checks;
    }
    std::fesetround(saved_round);
    // Actual board failure: do not patch the pixel. Calibrate independent Y/U/V arithmetic.
    const auto regression=[&](r20::ReferencePreprocess callback,bool actual){
        // The observed original branch was I420; calibration fixtures are compact NV12.
        std::unique_ptr<r22::OriginalSourceScope> raw_context;
        if(actual)raw_context=std::make_unique<r22::OriginalSourceScope>(false,1280);
        std::vector<uint8_t> input(1280*1088*3/2,127),expected,dst(864*480*3/2+37,0xa5);
        const auto uv=std::size_t(1280)*1088;
        for(int c=0;c<2;++c){input[uv+175*1280+14+c]=127;input[uv+175*1280+16+c]=126;
            input[uv+176*1280+14+c]=124;input[uv+176*1280+16+c]=129;}
        r20::DetectorWorkspace corrected;r22::Workspace cache,write;
        corrected.setReference(callback);cache.setReference(callback);write.setReference(callback);
        for(int n=0;n<3;++n){if(n)for(auto& b:input)b=uint8_t(random());
            r20::Geometry old,current;
            require(callback(input.data(),1280,1088,1280,input.size(),true,864,480,1,&old,expected,127),"Channel oracle failed");
            if(n==0){require(expected[481404]==127,"Native regression: original U is no longer 127; baseline must not be rewritten to conceal the error");
                if(!actual)require(expected[481405]==126,"Independent V tie fixture incorrect");}
            require(corrected.prepare(input.data(),1280,1088,1280,input.size(),true,864,480,1,current),"Corrected r21 calibration failed");
            require(corrected.output==expected,"Corrected r21 differs from the actual oracle");++checks;
            require(cache.prepare(input.data(),1280,1088,1280,input.size(),true,864,480,1,current),"Channel cached prepare failed");
            require(cache.output==expected,"Channel cached bytes differ");++checks;
            require(write.prepare(input.data(),1280,1088,1280,input.size(),true,864,480,1,current,dst.data(),dst.size()),"Channel direct prepare failed");
            require(std::memcmp(dst.data(),expected.data(),expected.size())==0,"Channel direct bytes differ");++checks;
        }
        // Switching a callback invalidates calibration and tensor padding ownership.
        cache.setReference(r20::reference::PreprocessYuv420);r20::Geometry old,current;
        require(r20::reference::PreprocessYuv420(input.data(),1280,1088,1280,input.size(),true,864,480,1,&old,expected,127),"Reference switch failed");
        require(cache.prepare(input.data(),1280,1088,1280,input.size(),true,864,480,1,current),"Recalibration failed");
        require(cache.output==expected,"Reference identity change retained old channel math");++checks;
        std::cout<<"Regression 481404 U=127: "<<(actual?"actual-runner":"independent Y0/U4/V3")<<" passed; modes=";
        for(int m:corrected.mathModes())std::cout<<m<<'/';std::cout<<'\n';
    };
    regression(independentChannelOracle,false);
#ifdef R22_NATIVE_REFERENCE_TEST
    regression(r22::runnerReferencePreprocessor(),true);
#endif
    std::cout<<"PASS "<<checks<<" cached/direct exact cases; "<<steady_no_padding<<" steady frames without padding refill; table="<<table<<" gather="<<gather<<"; capacity/guards/tail/geometry transitions checked; NEON="<<r20::neonEnabled()<<"\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
