#include "sdk_test_support.hpp"
#include "runtime_status.hpp"
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
void require(bool test,const char* why){if(!test)throw std::runtime_error(why);}
void originalProfilePathContract(const std::string& source) {
    namespace fs=std::filesystem;
    const auto start=fs::current_path();
    const auto temp=fs::temp_directory_path()/("psdk-model-paths-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    struct Cleanup {
        fs::path cwd,directory;
        ~Cleanup(){std::error_code error;fs::current_path(cwd,error);fs::remove_all(directory,error);}
    } cleanup{start,temp};
    fs::create_directories(temp/"project"/"profiles");
    fs::create_directories(temp/"elsewhere");
    const auto absolute_seg=(temp/"absolute-seg.fixture.bin").lexically_normal();
    const auto profile=temp/"project"/"profiles"/"original.cfg";
    {
        std::ifstream input(fs::u8path(source),std::ios::binary);
        std::ofstream output(profile,std::ios::binary);
        require(bool(input)&&bool(output),"Cannot prepare relative model profile fixture");
        std::string row;
        while(std::getline(input,row)) {
            if(row.rfind("det_model=",0)==0)row="det_model=../models/det.fixture.bin";
            else if(row.rfind("seg_model=",0)==0)row="seg_model="+absolute_seg.generic_u8string();
            output<<row<<'\n';
        }
        require(!input.bad()&&bool(output),"Cannot write relative model profile fixture");
    }
    fs::current_path(temp/"elsewhere");
    const auto expected=(temp/"project"/"models"/"det.fixture.bin").lexically_normal().u8string();
    for(const auto& file:{profile.u8string(),std::string("../project/profiles/original.cfg")}) {
        const auto models=psdk_test::originalModelPaths(file.c_str());
        require(models.first==expected,"Original profile model was resolved against process cwd");
        require(models.second==absolute_seg.u8string(),"Absolute model path was changed");
    }
    fs::create_directories(temp/"relocated"/"profiles");
    const auto relocated=temp/"relocated"/"profiles"/"original.cfg";
    fs::copy_file(profile,relocated);
    const auto models=psdk_test::originalModelPaths(relocated.u8string().c_str());
    require(models.first==(temp/"relocated"/"models"/"det.fixture.bin").lexically_normal().u8string(),
        "Relative model path retained the old location after moving the profile tree");
    require(models.second==absolute_seg.u8string(),"Relocation changed an explicit absolute model path");
}
// Explicit fake model stages validate the ABI/DAG/lifetime, not real inference.
struct FakeStages final:r25::Stages {
    std::atomic<int> starts[2]{};
    std::atomic<int> failure{0};
    int widths[2]{},heights[2]{};
    uint8_t first[2]{};
    bool fail_close=false;
    int closes=0;
    void preprocess(int lane,const perception::ImageView& image) override {
        ++starts[lane];widths[lane]=image.width;heights[lane]=image.height;first[lane]=image.data[0];
        if(lane==0&&failure==1)throw r28::Failure({1011,-123,"fake.submit",{},"Injected library rejection",{}});
        if(lane==0&&failure==2)throw 7;
    }
    void infer(int,ai_node_percep_demo::DnnInferTiming&) override {}
    std::vector<ai_node_percep_demo::DnnDetection> detection() override {return {{8,.9f,1,2,8,10,"person"}};}
    std::vector<uint8_t> segmentation() override {return std::vector<uint8_t>(std::size_t(widths[1])*heights[1],first[1]%3);}
    int width(int lane)const override{return lane?832:864;}
    int height(int)const override{return 480;}
    void abortFrame()noexcept override{}
    void close()override{++closes;if(fail_close)throw r28::Failure({2006,-77,"fake.close",{},"Injected cleanup rejection",{}});}
};
psdk_config config(const std::string& original,const std::string& dag){
    psdk_config value{};value.struct_size=sizeof(value);value.abi_version=PSDK_ABI_VERSION;value.backend=PSDK_BACKEND_BPU;
    value.original_profile=original.c_str();value.dag_profile=dag.c_str();return value;
}
psdk_result_view view(psdk_result* result){
    psdk_result_view out{};out.struct_size=sizeof(out);out.abi_version=PSDK_ABI_VERSION;psdk_status status;
    require(psdk_result_get_view(result,&out,&status)==0&&status.code==0,"result view failed");return out;
}
}
int main(int argc,char** argv){try{
    require(argc==2,"Pass profiles directory");const auto original=std::string(argv[1])+"/original.cfg",dag=std::string(argv[1])+"/dag_exact.json";
    originalProfilePathContract(original);
    auto cfg=config(original,dag);psdk_status status{};psdk_engine* engine=nullptr;
    require(psdk_abi_version()==PSDK_ABI_VERSION,"ABI version mismatch");
    auto invalid=cfg;invalid.abi_version=99;
    require(psdk_create(&invalid,&engine,&status)==2001&&engine==nullptr&&status.code==2001,"invalid ABI accepted");
    invalid=cfg;invalid.reserved_flags=1;
    require(psdk_create(&invalid,&engine,&status)==2001&&engine==nullptr,"reserved flag accepted");
    auto stages=std::make_shared<FakeStages>();require(psdk_test::create(&cfg,stages,&engine,&status)==0&&engine,"injected test engine creation failed");
    std::vector<uint8_t> raw(16*16*3/2,128);psdk_frame frame{};
    frame.struct_size=sizeof(frame);frame.abi_version=PSDK_ABI_VERSION;frame.format=PSDK_FORMAT_NV12;
    frame.width=16;frame.height=16;frame.stride=16;frame.data=raw.data();frame.bytes=raw.size();frame.frame_id=91;frame.timestamp_ns=1001;
    psdk_result* first=nullptr;require(psdk_process(engine,&frame,&first,&status)==0&&first,"first process failed");
    const auto first_view=view(first);require(first_view.width==16&&first_view.height==16&&first_view.frame_id==91&&first_view.timestamp_ns==1001,"result identity lost");
    require(first_view.segmentation_bytes==256&&first_view.segmentation_mask[0]==2&&first_view.detection_count==1&&std::strcmp(first_view.detections[0].class_name,"person")==0,"result payload invalid");
    require(stages->starts[0]==1&&stages->starts[1]==1,"original DAG did not run both branches");
    raw[0]=127;frame.frame_id=92;frame.timestamp_ns=1002;psdk_result* second=nullptr;
    require(psdk_process(engine,&frame,&second,nullptr)==0&&second,"second process failed");
    require(view(second).segmentation_mask[0]==1&&first_view.segmentation_mask[0]==2,"retained result borrowed mutable model storage");
    psdk_result_view bad_view{};bad_view.struct_size=sizeof(bad_view);bad_view.abi_version=99;
    require(psdk_result_get_view(first,&bad_view,&status)==1012,"bad result view ABI accepted");
    const auto fail_image=[&](psdk_frame bad){psdk_result* result=nullptr;require(psdk_process(engine,&bad,&result,&status)==1002&&!result&&status.code==1002,"invalid input accepted or output retained");};
    auto bad=frame;bad.bytes=1;fail_image(bad);bad=frame;bad.stride=17;fail_image(bad);bad=frame;bad.height=15;fail_image(bad);
    bad=frame;bad.width=0xffffffffu;fail_image(bad);bad=frame;bad.data=nullptr;fail_image(bad);bad=frame;bad.abi_version=2;fail_image(bad);bad=frame;bad.format=17;fail_image(bad);
    stages->failure=1;psdk_result* failed=nullptr;frame.frame_id=93;
    require(psdk_process(engine,&frame,&failed,&status)==1011&&!failed,"failed frame escaped as a result");
    require(status.native_code==-123&&status.frame_id==93&&std::strcmp(status.feature,"fake.submit")==0&&std::strcmp(status.task,"detection")==0,"full native code/feature/task/frame lost");
    stages->failure=2;require(psdk_process(engine,&frame,&failed,&status)==1011&&!failed,"nonstandard exception crossed C ABI");
    stages->failure=0;frame.frame_id=94;frame.format=PSDK_FORMAT_I420;
    require(psdk_process(engine,&frame,&failed,&status)==0&&failed&&status.native_code==0&&status.detail[0]==0,"engine failed to recover or stale error remained");
    psdk_result_release(failed);
    std::atomic<int> concurrent_errors{0};
    const auto concurrent=[&](uint64_t base){
        for(uint64_t i=0;i<6;++i){auto input=frame;input.frame_id=base+i;input.timestamp_ns=base*100+i;psdk_result* result=nullptr;psdk_status error{};
            if(psdk_process(engine,&input,&result,&error)||!result){++concurrent_errors;continue;}
            psdk_result_view result_view{};result_view.struct_size=sizeof(result_view);result_view.abi_version=PSDK_ABI_VERSION;
            if(psdk_result_get_view(result,&result_view,&error)||result_view.frame_id!=input.frame_id||result_view.timestamp_ns!=input.timestamp_ns||result_view.segmentation_bytes!=256)++concurrent_errors;
            psdk_result_release(result);
        }
    };
    std::thread caller1(concurrent,200),caller2(concurrent,300);caller1.join();caller2.join();
    require(concurrent_errors==0,"concurrent synchronous callers mixed results");
    const auto dir=std::filesystem::temp_directory_path()/("r32-sdk-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(dir);const auto png=(dir/"overlay.png").u8string();
    // second belongs to frame 92 / NV12. Identity mismatch must fail before writing.
    require(psdk_save_png(second,&frame,png.c_str(),&status)==2002&&!std::filesystem::exists(std::filesystem::u8path(png)),"cross-frame PNG accepted");
    frame.frame_id=92;frame.timestamp_ns=1002;frame.format=PSDK_FORMAT_NV12;
    require(psdk_save_png(second,&frame,png.c_str(),&status)==0,"same-frame PNG failed");
    std::ifstream file(std::filesystem::u8path(png),std::ios::binary);char magic[8]{};file.read(magic,8);file.close();
    require(std::memcmp(magic,"\x89PNG\r\n\x1a\n",8)==0,"output is not PNG");
    const auto size=std::filesystem::file_size(std::filesystem::u8path(png));
    require(psdk_save_png(second,&frame,png.c_str(),&status)==2002&&std::filesystem::file_size(std::filesystem::u8path(png))==size,"PNG overwrote existing destination");
    std::filesystem::remove_all(dir);
    require(psdk_destroy(&engine,&status)==0&&!engine&&stages->closes==1,"destroy did not close backend exactly once");
    require(first_view.segmentation_mask[0]==2&&view(second).segmentation_mask[0]==1,"result did not outlive engine");
    require(psdk_destroy(&engine,&status)==0,"NULL repeated destroy failed");psdk_result_release(first);psdk_result_release(second);psdk_result_release(nullptr);
    auto broken=std::make_shared<FakeStages>();broken->fail_close=true;
    require(psdk_test::create(&cfg,broken,&engine,&status)==0,"cleanup test create failed");
    require(psdk_destroy(&engine,&status)==2006&&!engine&&status.native_code==-77,"cleanup fault hidden or stale engine retained");
    // The public ABI must honor a single selected model lane, not merely the
    // underlying scheduler test. Disabled lanes produce no stale payload.
    for(int lane=0;lane<2;++lane) {
        const auto selected=std::string(argv[1])+(lane?"/dag_exact_seg_only.json":"/dag_exact_det_only.json");
        auto single=config(original,selected);auto one=std::make_shared<FakeStages>();
        require(psdk_test::create(&single,one,&engine,&status)==0,"Single-lane SDK create failed");
        psdk_result* result=nullptr;
        require(psdk_process(engine,&frame,&result,&status)==0&&result,"Single-lane SDK process failed");
        const auto actual=view(result);
        require(one->starts[lane]==1&&one->starts[1-lane]==0,"Disabled model lane executed");
        require(lane?(actual.segmentation_bytes==256&&actual.detection_count==0):(actual.segmentation_bytes==0&&actual.detection_count==1),"Single-lane result payload was rejected or fabricated");
        psdk_result_release(result);require(psdk_destroy(&engine,&status)==0,"Single-lane cleanup failed");
    }
    // Two independent product instances must call two real lane pipelines.
    // The legacy global exact-workspace gate may serialize their execution,
    // but neither instance may secretly run or cache the other algorithm.
    {
        const auto det_dag=std::string(argv[1])+"/dag_exact_det_only.json";
        const auto seg_dag=std::string(argv[1])+"/dag_exact_seg_only.json";
        auto det_cfg=config(original,det_dag),seg_cfg=config(original,seg_dag);
        auto det=std::make_shared<FakeStages>(),seg=std::make_shared<FakeStages>();
        psdk_engine *det_engine=nullptr,*seg_engine=nullptr;psdk_result *det_result=nullptr,*seg_result=nullptr;
        require(!psdk_test::create(&det_cfg,det,&det_engine,&status)&&!psdk_test::create(&seg_cfg,seg,&seg_engine,&status),"Split model engines did not initialize independently");
        std::atomic<int> errors{0};
        std::thread a([&]{psdk_status s{};if(psdk_process(det_engine,&frame,&det_result,&s))++errors;});
        std::thread b([&]{psdk_status s{};if(psdk_process(seg_engine,&frame,&seg_result,&s))++errors;});
        a.join();b.join();
        require(errors==0&&det_result&&seg_result,"Independent lane dispatch failed");
        const auto d=view(det_result),s=view(seg_result);
        require(det->starts[0]==1&&det->starts[1]==0&&seg->starts[0]==0&&seg->starts[1]==1,"Split nodes ran a hidden combined det/seg pipeline");
        require(d.frame_id==s.frame_id&&d.timestamp_ns==s.timestamp_ns&&d.detection_count==1&&d.segmentation_bytes==0&&s.detection_count==0&&s.segmentation_bytes==256,"Independent results cannot form the same-frame combined output");
        det->failure=1;psdk_result* failed_det=nullptr;
        require(psdk_process(det_engine,&frame,&failed_det,&status)==1011&&!failed_det,"Detection failure was exposed as success");
        require(view(seg_result).segmentation_bytes==256&&seg->starts[1]==1,"Detection failure mutated the independent segmentation result");
        require(!psdk_destroy(&det_engine,&status)&&!psdk_destroy(&seg_engine,&status),"Split lane shutdown failed");
        require(d.detections[0].class_id==8&&s.segmentation_mask[0]==raw[0]%3,"Split result ownership did not survive engine destruction");
        psdk_result_release(det_result);psdk_result_release(seg_result);
    }
    std::cout<<"C ABI: validation, owned same-frame results, full errors, exception isolation, recovery and exclusive PNG passed; fake stages, not hardware inference.\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
