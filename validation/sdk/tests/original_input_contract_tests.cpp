#include "original_input_contract.hpp"
#include "exact_input.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

namespace {
void contractCheck(bool yes,const char* message){if(!yes)throw std::runtime_error(message);}
template<class F> void contractRejects(F f){bool threw=false;try{f();}catch(const std::invalid_argument&){threw=true;}contractCheck(threw,"Invalid original input was accepted");}

#ifndef R26_NATIVE_REFERENCE_TEST
// Host fallback has the same source-context conversion as the actual runner.
// Calibration supplies compact probes, but the original scalar helper must see
// the original raw stride/layout, including the 656-byte stride test case.
bool contractHostReference(const std::uint8_t* input,int sw,int sh,int stride,std::size_t bytes,bool nv12,
                          int dw,int dh,int resize,r20::Geometry* geometry,
                          std::vector<std::uint8_t>& result,std::uint8_t pad){
    std::vector<std::uint8_t> raw;const auto context=r22::originalSourceContext();
    if(context){
        const bool target_nv12=(context&1)!=0;const int target_stride=int(context>>2);
        if(target_stride<sw||target_stride%2)return false;
        if(target_nv12!=nv12||target_stride!=stride){
            r20::PackWorkspace pack;const auto packed=pack.run(input,sw,sh,stride,bytes,nv12);
            if(!packed.data)return false;
            raw.assign(std::size_t(target_stride)*sh*3/2,0);
            for(int y=0;y<sh;++y)std::memcpy(raw.data()+std::size_t(y)*target_stride,packed.data+std::size_t(y)*sw,sw);
            const auto plane=std::size_t(target_stride)*sh;const auto* uv=packed.data+std::size_t(sw)*sh;
            if(target_nv12){for(int y=0;y<sh/2;++y)std::memcpy(raw.data()+plane+std::size_t(y)*target_stride,uv+std::size_t(y)*sw,sw);}
            else for(int y=0;y<sh/2;++y)for(int x=0;x<sw/2;++x){
                raw[plane+std::size_t(y)*(target_stride/2)+x]=uv[std::size_t(y)*sw+2*x];
                raw[plane+plane/4+std::size_t(y)*(target_stride/2)+x]=uv[std::size_t(y)*sw+2*x+1];
            }
            input=raw.data();stride=target_stride;bytes=raw.size();nv12=target_nv12;
        }
    }
    return r20::reference::PreprocessYuv420(input,sw,sh,stride,bytes,nv12,dw,dh,resize,geometry,result,pad);
}
#endif

const char* contractReferenceName(){
#ifdef R26_NATIVE_REFERENCE_TEST
    return "actual-original-runner";
#else
    return "extracted-host-with-original-source-context";
#endif
}
r20::ReferencePreprocess contractReference(){
#ifdef R26_NATIVE_REFERENCE_TEST
    return r22::runnerReferencePreprocessor();
#else
    return contractHostReference;
#endif
}
std::string contractModes(const std::array<int,3>& modes){
    std::ostringstream out;out<<'['<<modes[0]<<','<<modes[1]<<','<<modes[2]<<']';return out.str();
}
struct ContractDifference {
    std::size_t total=0;
    std::array<std::size_t,3> planes{{0,0,0}};
    std::vector<std::size_t> first;
};
ContractDifference contractDifference(const std::vector<std::uint8_t>& expected,const std::uint8_t* actual,int dw,int dh){
    ContractDifference out;const auto y_size=std::size_t(dw)*dh;
    for(std::size_t i=0;i<expected.size();++i)if(expected[i]!=actual[i]){
        ++out.total;++out.planes[i<y_size?0:1+int((i-y_size)%2)];
        if(out.first.size()<20)out.first.push_back(i);
    }
    return out;
}
void contractWriteBytes(const std::filesystem::path& path,const std::uint8_t* bytes,std::size_t size){
    std::ofstream out(path,std::ios::binary);out.write(reinterpret_cast<const char*>(bytes),std::streamsize(size));
    if(!out)throw std::runtime_error("Cannot save input failure evidence: "+path.string());
}
void contractVerifyBytes(const std::filesystem::path& diagnostics,const char* path_name,int lane,int stride,int frame,
                         int dw,int dh,int resize,const std::vector<std::uint8_t>& source,
                         const std::vector<std::uint8_t>& expected,const std::uint8_t* actual,
                         const std::array<int,3>& modes,r20::ReferencePreprocess callback){
    const auto difference=contractDifference(expected,actual,dw,dh);if(!difference.total)return;
    std::cerr<<"Original-map "<<path_name<<" input bytes differ: lane="<<(lane?"segmentation":"detection")
             <<" stride="<<stride<<" frame="<<frame<<" reference="<<contractReferenceName()
             <<" context="<<r22::originalSourceContext()<<" math_modes="<<contractModes(modes)
             <<" different="<<difference.total<<" Y/U/V="<<difference.planes[0]<<'/'<<difference.planes[1]<<'/'<<difference.planes[2]<<'\n';
    for(auto i:difference.first)std::cerr<<"  offset="<<i<<" baseline="<<unsigned(expected[i])<<" candidate="<<unsigned(actual[i])<<'\n';

    // Diagnose the scalar-calibrated path without replacing the failed output.
    // An r21 mismatch points at calibration/reference, rather than compact input
    // writing alone. This result never changes the strict acceptance decision.
    std::size_t r21_difference=0;bool r21_ok=false;std::array<int,3> r21_modes{{-1,-1,-1}};
    try{
        r20::DetectorWorkspace r21;r21.setReference(callback,r22::originalSourceContext());r20::Geometry geometry;
        r21_ok=r21.prepare(source.data(),640,544,stride,source.size(),true,dw,dh,resize,geometry);
        r21_modes=r21.mathModes();
        if(r21_ok&&r21.output.size()==expected.size())r21_difference=contractDifference(expected,r21.output.data(),dw,dh).total;
        else r21_ok=false;
    }catch(const std::exception& error){std::cerr<<"  r21 diagnostic error: "<<error.what()<<'\n';}
    std::cerr<<"  r21_same_frame_ok="<<r21_ok<<" r21_different="<<r21_difference<<" r21_math_modes="<<contractModes(r21_modes)<<'\n';
    if(!diagnostics.empty()){
        const auto directory=diagnostics/(std::string(lane?"segmentation":"detection")+"_stride"+std::to_string(stride)+"_frame"+std::to_string(frame)+"_"+path_name);
        std::filesystem::create_directories(directory);
        contractWriteBytes(directory/"source.yuv",source.data(),source.size());
        contractWriteBytes(directory/"baseline_prepared.nv12",expected.data(),expected.size());
        contractWriteBytes(directory/"direct_prepared.nv12",actual,expected.size());
        std::ofstream out(directory/"failure.json",std::ios::binary);
        out<<"{\n  \"check\":\"r26-original-input-strict-bytes\",\n  \"accepted\":false,\n  \"lane\":\""<<(lane?"segmentation":"detection")
           <<"\",\n  \"path\":\""<<path_name<<"\",\n  \"stride\":"<<stride<<",\n  \"frame\":"<<frame
           <<",\n  \"source_width\":640,\n  \"source_height\":544,\n  \"destination_width\":"<<dw<<",\n  \"destination_height\":"<<dh
           <<",\n  \"reference\":\""<<contractReferenceName()<<"\",\n  \"source_context\":"<<r22::originalSourceContext()
           <<",\n  \"math_modes_yuv\":"<<contractModes(modes)<<",\n  \"different_bytes\":"<<difference.total
           <<",\n  \"different_yuv\":["<<difference.planes[0]<<','<<difference.planes[1]<<','<<difference.planes[2]
           <<"],\n  \"r21_same_frame_ok\":"<<(r21_ok?"true":"false")<<",\n  \"r21_different_bytes\":"<<r21_difference
           <<",\n  \"r21_math_modes_yuv\":"<<contractModes(r21_modes)<<",\n  \"first_differences\":[";
        for(std::size_t n=0;n<difference.first.size();++n){const auto i=difference.first[n];if(n)out<<',';
            out<<"{\"offset\":"<<i<<",\"baseline\":"<<unsigned(expected[i])<<",\"candidate\":"<<unsigned(actual[i])<<'}';}
        out<<"]\n}\n";if(!out)throw std::runtime_error("Cannot save failure.json");
        std::cerr<<"  evidence="<<directory.string()<<'\n';
    }
    throw std::runtime_error("EXACT_ACCEPTANCE_FAILED: original-map input bytes differ");
}
}

int main(int argc,char** argv){try{
    std::filesystem::path diagnostics;
    for(int i=1;i<argc;++i){
        if(std::string(argv[i])=="--diagnostics-dir"&&i+1<argc)diagnostics=std::filesystem::u8path(argv[++i]);
        else throw std::invalid_argument("Usage: original_input_contract_tests [--diagnostics-dir PATH]");
    }
    r26::validateSelection("rgbd","nv12",0,0);
    contractRejects([]{r26::validateSelection("gdc","nv12",0,0);});
    contractRejects([]{r26::validateSelection("rgbd","i420",0,0);});
    contractRejects([]{r26::validateSelection("rgbd","nv12",1,10);});
    contractRejects([]{r26::validateSelection("rgbd","nv12",2,0);});
    const auto callback=contractReference();std::mt19937 random(26);int cases=0;
    for(int stride:{640,656})for(int lane:{0,1}) {
        r22::OriginalSourceScope original_source(true,stride);
        std::vector<std::uint8_t> source(std::size_t(stride)*544*3/2),expected;
        const int dw=lane?832:864,dh=480,resize=lane?0:1;
        r22::Workspace direct,cached;
        direct.setReference(callback,r22::originalSourceContext());cached.setReference(callback,r22::originalSourceContext());
        r22::VariantScope variant(r22::Variant::CompactDirect);
        std::vector<std::uint8_t> destination(std::size_t(dw)*dh*3/2+48,0xa5);
        for(int frame=0;frame<5;++frame){
            // Retain black -> white -> random order. Calibration on an easy
            // first frame must still be exact on every later frame.
            for(auto& byte:source)byte=frame==0?0:frame==1?255:std::uint8_t(random());
            perception::ImageView view{source.data(),source.size(),std::size_t(stride),640,544,perception::PixelFormat::NV12};
            r26::validateInput(view);r20::Geometry a,b;
            std::cout<<"Input contract lane="<<(lane?"segmentation":"detection")<<" stride="<<stride<<" frame="<<frame
                     <<" reference="<<contractReferenceName()<<" context="<<r22::originalSourceContext()<<'\n';
            contractCheck(callback(source.data(),640,544,stride,source.size(),true,dw,dh,resize,&a,expected,127),"Original oracle failed");
            contractCheck(expected.size()==std::size_t(dw)*dh*3/2,"Original oracle input size changed");
            contractCheck(direct.prepare(source.data(),640,544,stride,source.size(),true,dw,dh,resize,b,destination.data(),destination.size()-16),"Direct input failed");
            contractVerifyBytes(diagnostics,"direct",lane,stride,frame,dw,dh,resize,source,expected,destination.data(),direct.mathModes(),callback);
            for(std::size_t i=expected.size();i<destination.size()-16;++i)contractCheck(destination[i]==0,"Aligned tensor tail not initialized");
            for(std::size_t i=destination.size()-16;i<destination.size();++i)contractCheck(destination[i]==0xa5,"Input overrun");
            contractCheck(cached.prepare(source.data(),640,544,stride,source.size(),true,dw,dh,resize,b),"Cached input failed");
            contractCheck(cached.output.size()==expected.size(),"Cached input size differs");
            contractVerifyBytes(diagnostics,"cached",lane,stride,frame,dw,dh,resize,source,expected,cached.output.data(),cached.mathModes(),callback);
            if(!lane)contractCheck(a.valid_w==564&&a.valid_h==480&&a.x_shift==150&&a.y_shift==0,"Original letterbox changed");
            else contractCheck(a.valid_w==832&&a.valid_h==480&&a.x_shift==0&&a.y_shift==0,"Original segmentation resize changed");
            ++cases;
        }
        auto bad=perception::ImageView{source.data(),source.size(),std::size_t(stride),1280,1088,perception::PixelFormat::NV12};contractRejects([&]{r26::validateInput(bad);});
        bad={source.data(),source.size(),std::size_t(stride),640,544,perception::PixelFormat::I420};contractRejects([&]{r26::validateInput(bad);});
        bad={source.data(),source.size()-1,std::size_t(stride),640,544,perception::PixelFormat::NV12};contractRejects([&]{r26::validateInput(bad);});
    }
    std::cout<<"PASS original RGBD-map contract, "<<cases<<" exact original-map fixtures, padding/tail reuse and rejection checks; reference="
             <<contractReferenceName()<<". CPU-only test, no BPU inference.\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
