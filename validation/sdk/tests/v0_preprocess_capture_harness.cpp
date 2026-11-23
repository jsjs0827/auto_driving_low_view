#include "exact_input.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

// Offline numerical acceptance utility. Expected bytes come from the real V0
// capture, not from this SDK's scalar calibration oracle.
int main(int argc,char** argv) {
    try {
        if(argc!=9)throw std::invalid_argument("input sw sh dw dh resize variant output");
        const int sw=std::stoi(argv[2]),sh=std::stoi(argv[3]);
        const int dw=std::stoi(argv[4]),dh=std::stoi(argv[5]),resize=std::stoi(argv[6]);
        std::ifstream input(argv[1],std::ios::binary);
        if(!input)throw std::runtime_error("Cannot read source image");
        std::vector<uint8_t> source((std::istreambuf_iterator<char>(input)),{});
        const auto v=r22::parseVariant(argv[7]);r22::setVariant(v);
        const auto size=std::size_t(dw)*dh*3/2;
        std::vector<uint8_t> destination(size+37+64,0xa5),result;
        r20::Geometry g;std::array<int,3> modes;
        if(v==r22::Variant::R21) {
            r20::DetectorWorkspace workspace;
            if(!workspace.prepare(source.data(),sw,sh,sw,source.size(),true,dw,dh,resize,g))
                throw std::runtime_error("Legacy compatible calibration rejected");
            result=workspace.output;modes=workspace.mathModes();
        } else {
            r22::Workspace workspace;
            const bool direct=r22::directVariant(v);
            if(!workspace.prepare(source.data(),sw,sh,sw,source.size(),true,dw,dh,resize,g,
                                  direct?destination.data()+32:nullptr,direct?size+37:0))
                throw std::runtime_error("Compatible calibration rejected");
            if(direct) {
                if(!std::all_of(destination.begin(),destination.begin()+32,[](uint8_t b){return b==0xa5;})||
                   !std::all_of(destination.end()-32,destination.end(),[](uint8_t b){return b==0xa5;}))
                    throw std::runtime_error("Destination guard changed");
                if(!std::all_of(destination.begin()+32+size,destination.begin()+32+size+37,[](uint8_t b){return b==0;}))
                    throw std::runtime_error("Aligned destination tail not zero");
                result.assign(destination.begin()+32,destination.begin()+32+size);
            } else result=workspace.output;
            modes=workspace.mathModes();
        }
        if(result.size()!=size)throw std::runtime_error("Wrong result size");
        std::ofstream out(argv[8],std::ios::binary);
        if(!out.write(reinterpret_cast<const char*>(result.data()),result.size()))
            throw std::runtime_error("Cannot write prepared image");
        std::cout<<"{\"bytes\":"<<result.size()<<",\"modes\":["<<modes[0]<<','<<modes[1]<<','<<modes[2]
                 <<"],\"valid_width\":"<<g.valid_w<<",\"valid_height\":"<<g.valid_h
                 <<",\"left\":"<<g.x_shift<<",\"top\":"<<g.y_shift
                 <<",\"neon\":"<<(r20::neonEnabled()?"true":"false")<<"}\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
