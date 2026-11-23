#include "dag_pipeline.hpp"
#include <cfenv>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <atomic>
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct MockStages final:r25::Stages {
    std::atomic<int> phase[2]{};
    int fail=-1,round=FE_TOWARDZERO;std::atomic<int> posts{0};
    void preprocess(int lane,const perception::ImageView&) override {
        check(std::fegetround()==round,"DAG pool did not inherit frame FP environment");
        check(phase[lane]++==0,"Previous frame still active");
        if(fail==lane)throw std::runtime_error("injected preprocess failure");
    }
    void infer(int lane,ai_node_percep_demo::DnnInferTiming&) override {check(phase[lane]++==1,"Backend phase wrong");}
    std::vector<ai_node_percep_demo::DnnDetection> detection() override {check(phase[0].exchange(0)==2,"Det post phase wrong");++posts;return {{2,.75f,1,2,3,4,"test"}};}
    std::vector<std::uint8_t> segmentation() override {check(phase[1].exchange(0)==2,"Seg post phase wrong");++posts;return std::vector<std::uint8_t>(16*16,2);}
    int width(int) const override{return 16;}int height(int) const override{return 16;}
    void abortFrame() noexcept override{phase[0]=0;phase[1]=0;}
};
}
int main(int argc,char** argv){try{
    check(argc==2,"Profiles required");std::fesetround(FE_TOWARDZERO);
    std::vector<std::uint8_t> raw(16*16*3/2,127);perception::ImageView image{raw.data(),raw.size(),16,16,16,perception::PixelFormat::NV12};
    for(const char* name:{"dag_exact.json","dag_exact_serial.json"}){
        auto mock=std::make_shared<MockStages>();r25::Pipeline p(std::string(argv[1])+"/"+name,mock);
        r25::RunOptions o;o.optimized=false;int depth=0;o.depth=[&](std::uint64_t id){check(mock->posts==2,"Depth dependency failed");++depth;return r25::DepthResult{true,id,0,2,2,4};};
        auto a=p.run(image,{0,123},o);check(depth==1 && a.frame.frame.id==0,"Zero frame ID rejected");
        check(a.features[1].feature=="ai.exact.det.preprocess" && a.features[5].feature=="ai.exact.seg.backend","Feature factory name lifetime invalid");
        check(a.mask().size()==256 && a.frame.detection->detections.size()==1,"Final typed artifacts missing");
        check(a.frame.artifacts->keys().size()==4,"Transient/borrowed artifacts retained");
        auto saved=a.frame;mock->posts=0;auto b=p.run(image,{1,124},o);
        check(saved.segmentation->mask==a.mask() && saved.frame.id==0 && b.frame.frame.id==1,"Previous result overwritten");
        mock->fail=0;depth=0;bool rejected=false;try{p.run(image,{2,125},o);}catch(const std::exception& e){rejected=std::string(e.what()).find("DAG failed")!=std::string::npos;}
        check(rejected && depth==0 && mock->phase[0]==0 && mock->phase[1]==0,"Failure was not propagated/reset");
        mock->fail=-1;mock->posts=0;p.run(image,{3,126},o);check(depth==1,"Graph did not recover");
        check(std::fegetround()==FE_TOWARDZERO,"Caller FP environment changed");
    }
    r25::Pipeline camera(std::string(argv[1])+"/dag_capture.json");r25::RunOptions input;input.input_only=true;
    auto c=camera.run(image,{4,127},input);check(c.frame.schedule.tasks.size()==2 && !c.frame.detection && !c.frame.segmentation,"Capture graph did BPU work");
    std::fesetround(FE_TONEAREST);std::cout<<"Original Engine exact Feature contracts, pool FP context, immutable results, zero ID, errors, depth dependencies, capture route passed.\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
