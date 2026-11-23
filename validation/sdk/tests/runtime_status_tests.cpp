#include "dag_pipeline.hpp"
#include <iostream>
#include <stdexcept>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <chrono>

namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct Stages final:r25::Stages {
    std::atomic<int> starts[2]{};
    bool fail=false;
    bool fail_close=false;int closes=0;
    void preprocess(int lane,const perception::ImageView&) override {
        ++starts[lane];
        if(fail&&lane==0)throw r28::Failure({1011,-7,"bpu.submit",{},"vendor rejection",{}});
    }
    void infer(int,ai_node_percep_demo::DnnInferTiming&) override {}
    std::vector<ai_node_percep_demo::DnnDetection> detection() override{return {};}
    std::vector<std::uint8_t> segmentation() override{return std::vector<std::uint8_t>(16*16,0);}
    int width(int)const override{return 16;}
    int height(int)const override{return 16;}
    void abortFrame()noexcept override{}
    void close()override{++closes;if(fail_close)throw r28::Failure({2006,-19,"model.close",{},"close rejected",{}});}
};
}
int main(int argc,char** argv){try{
    require(argc==2,"profiles required");
    std::vector<std::uint8_t> raw(16*16*3/2,128);
    const perception::ImageView image{raw.data(),raw.size(),16,16,16,perception::PixelFormat::NV12};
    auto stages=std::make_shared<Stages>();
    r25::Pipeline graph(std::string(argv[1])+"/dag_exact.json",stages);
    r25::RunOptions options;options.optimized=false;r25::Output output;r28::Status status;
    require(graph.tryRun(image,{73,1000},options,output,status)==0&&status.ok(),"success did not return zero");
    stages->fail=true;
    require(graph.tryRun(image,{74,1001},options,output,status)==1011,"native failure code lost");
    require(status.native_code==-7&&status.frame.id==74&&status.task=="detection"&&status.feature=="bpu.submit","native fault identity lost");
    require(!output.frame.detection&&!output.frame.segmentation,"partial failed output exposed");
    require(r28::statusJson(status).find("\"native_code\":-7")!=std::string::npos,"wire code missing");
    auto invalid=image;invalid.bytes=1;
    require(graph.tryRun(invalid,{75,1002},options,output,status)==1002,"invalid image type lost");
    stages->fail=false;
    require(graph.tryRun(image,{76,1003},options,output,status)==0,"fault poisoned next frame");
    auto one=std::make_shared<Stages>();
    r25::Pipeline det(std::string(argv[1])+"/dag_exact_det_only.json",one);
    options.run_segmentation=false;
    require(det.tryRun(image,{77,1004},options,output,status)==0&&one->starts[0]==1&&one->starts[1]==0,"det-only executed disabled lane");
    require(output.frame.detection&&!output.frame.segmentation,"det-only artifacts wrong");
    require(r28::processExit(1001)==1&&r28::processExit(0)==0,"process code truncated");
    // Exercise the real output Feature through the exact Engine boundary. The
    // output task deliberately has a custom name: mapping by task-name strings
    // would lose the full output status and native filesystem return value.
    namespace fs=std::filesystem;
    const auto temp=fs::temp_directory_path()/("r28-status-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(temp);
    const auto blocked=temp/"blocked";{std::ofstream file(blocked);file<<"regular file";}
    const auto config=temp/"graph.json";
    {std::ofstream file(config);file<<"{\"config_version\":3,\"engine\":{\"worker_threads\":2},\"tasks\":{"
      "\"input\":{\"features\":[\"input.exact.pack\"]},"
      "\"detection\":{\"features\":[\"ai.exact.det.preprocess\",\"ai.exact.det.backend\",\"ai.exact.det.postprocess\"]},"
      "\"segmentation\":{\"features\":[\"ai.exact.seg.preprocess\",\"ai.exact.seg.backend\",\"ai.exact.seg.postprocess\"]},"
      "\"disparity\":{\"features\":[\"stereo.demand\"]},"
      "\"save_layers\":{\"features\":[{\"type\":\"output.visualize\",\"params\":{\"dir\":\""
      <<blocked.generic_u8string()<<"\",\"layers\":\"overlay\",\"require_identity\":true}}]}},"
      "\"dag\":{\"sequence\":[\"input\",{\"parallel\":[\"detection\",\"segmentation\"]},\"disparity\",\"save_layers\"]}}";}
    {r25::Pipeline saving(config.u8string(),std::make_shared<Stages>());
     options.run_segmentation=true;
     require(saving.tryRun(image,{78,1005},options,output,status)==2002,"output status lost at Engine boundary");
     require(status.feature=="output.visualize"&&status.task=="save_layers"&&status.frame.id==78,"output context lost");
     require(!output.frame.segmentation,"output failure exposed partial successful frame");}
    fs::remove_all(temp);
    auto broken=std::make_shared<Stages>();broken->fail_close=true;
    {r25::Pipeline closing(std::string(argv[1])+"/dag_exact_det_only.json",broken);
     require(closing.tryClose(status)==2006&&status.native_code==-19,"cleanup native failure hidden");
     require(closing.tryClose(status)==0&&broken->closes==1,"cleanup not idempotent");}
    std::cout<<"Full wire/native code, failed-frame isolation, recovery and det-only DAG passed; mock, not BPU.\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
