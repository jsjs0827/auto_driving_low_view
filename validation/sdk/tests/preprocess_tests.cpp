#include "legacy_preprocess.hpp"
#include <array>
#include <iostream>
#include <random>
#include <stdexcept>
int main(){try{
    // Non-model aspect ratios, odd half-padding, source padding and I420 tails.
    const std::array<std::array<int,4>,10> shapes{{{{1280,1088,864,480}},{{1280,1088,832,480}},{{640,544,864,480}},{{864,480,864,480}},{{640,544,832,480}},{{34,22,32,32}},{{30,18,32,30}},{{62,46,64,64}},{{126,94,128,96}},{{128,96,62,46}}}};
    std::mt19937 rng(19);std::size_t checks=0;
    for(const auto& shape:shapes)for(int resize:{0,1})for(bool nv12:{true,false}){
        const int w=shape[0],h=shape[1],stride=w+8,dw=shape[2],dh=shape[3];
        std::vector<std::uint8_t> image(std::size_t(stride)*h*3/2),baseline;
        r20::DetectorWorkspace workspace;r20::PackWorkspace pack;
        for(int iteration=0;iteration<4;++iteration){for(std::size_t i=0;i<image.size();++i)image[i]=iteration==0?std::uint8_t(rng()):iteration==1?0:iteration==2?255:std::uint8_t(i%256);
            const auto compact=pack.run(image.data(),w,h,stride,image.size(),nv12);
            if(!nv12){std::vector<std::uint8_t> original;r20::reference::ConvertI420ToNv12(image.data(),w,h,stride,original);
                if(original!=std::vector<std::uint8_t>(compact.data,compact.data+compact.bytes))throw std::runtime_error("I420 packing differs");}
            r20::Geometry oldg,newg;
            if(!r20::reference::PreprocessYuv420(image.data(),w,h,stride,image.size(),nv12,dw,dh,resize,&oldg,baseline,127)||
               !workspace.prepare(image.data(),w,h,stride,image.size(),nv12,dw,dh,resize,newg))throw std::runtime_error("prepare failed");
            if(workspace.output!=baseline)throw std::runtime_error("input bytes differ at shape "+std::to_string(w)+"x"+std::to_string(h));
            if(oldg.x_scale!=newg.x_scale||oldg.y_scale!=newg.y_scale||oldg.x_shift!=newg.x_shift||oldg.y_shift!=newg.y_shift||oldg.valid_w!=newg.valid_w||oldg.valid_h!=newg.valid_h)throw std::runtime_error("geometry differs");
            ++checks;
        }
    }
    const auto g=r20::originalGeometry(1280,1088,864,480,1);
    if(g.valid_w!=564||g.valid_h!=480||g.x_shift!=150||g.y_shift!=0)throw std::runtime_error("original letterbox changed");
    std::cout<<"PASS "<<checks<<" exact cases; original GDC letterbox=564x480 left=150; NEON="<<r20::neonEnabled()<<"\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
