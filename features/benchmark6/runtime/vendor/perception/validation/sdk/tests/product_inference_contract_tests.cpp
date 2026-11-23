// Executes the real NativeStages and product adapter against the existing
// host API fixture compiled with real Horizon declarations. NOT hardware/BPU
// acceptance and NOT a deployable model/runtime implementation.
#define main r30_unused_donor_fixture_main
#include "runner_harness.cpp"
#undef main
#include "dag_pipeline.hpp"
#include "product/exact_dnn_runner.hpp"
#include <functional>

namespace {
void rejected(const std::function<void()>& action, const char* message) {
    bool failed=false;try {action();}catch(const std::exception&) {failed=true;}
    require(failed,message);
}
void imageFailure(const std::function<void()>& action) {
    bool failed=false;
    try {action();}catch(const r28::Failure& error) {
        failed=error.status().code==1002 && error.status().frame.id==901;
    }
    require(failed,"Invalid image was not rejected with structured1002/frame identity");
}
DnnConfig config(bool det,bool seg,bool parallel=true) {
    DnnConfig cfg;cfg.enabled=true;cfg.detect.enabled=det;cfg.seg.enabled=seg;
    cfg.detect.model_path="det.host.fixture";cfg.detect.classes_num=24;
    cfg.seg.model_path="seg.host.fixture";cfg.seg.class_names="a,b,c";
    cfg.parallel_det_seg=parallel;return cfg;
}
void compareSubset(const std::vector<DnnDetection>& expected,
                   const std::vector<DnnDetection>& actual) {
    require(expected.size()==actual.size(),"Subset detection count differs from original");
    for(std::size_t i=0;i<actual.size();++i) {
        const auto& x=expected[i];const auto& y=actual[i];
        require(x.class_id==y.class_id&&x.class_name==y.class_name,"Subset detection class differs");
        const float a[]={x.confidence,x.x1,x.y1,x.x2,x.y2};
        const float b[]={y.confidence,y.x1,y.y1,y.x2,y.y2};
        require(std::memcmp(a,b,sizeof a)==0,"Subset detection numerical output differs");
    }
}
}
int main(int argc,char** argv) {try {
    require(argc==2,"Profile directory required");const std::string profiles=argv[1];
    using namespace r28::product;
    require(std::string(selectExactLanes(config(true,true),true).profile)=="dag_exact.json","Parallel config ignored");
    require(std::string(selectExactLanes(config(true,true,false),true).profile)=="dag_exact_serial.json","Serial config ignored");
    require(std::string(selectExactLanes(config(true,true),false).profile)=="dag_exact_det_only.json","Recharge requested segmentation");
    require(std::string(selectExactLanes(config(false,true),true).profile)=="dag_exact_seg_only.json","Seg-only selection failed");
    rejected([&]{(void)selectExactLanes(config(false,true),false);},"Empty lane request accepted");
    rejected([&]{(void)selectExactLanes(config(false,false),true);},"No enabled model accepted");
    std::vector<uint8_t> raw(80*64*3/2);unsigned seed=30;
    for(auto& byte:raw) {seed=seed*1664525u+1013904223u;byte=uint8_t(seed>>24);}
    const perception::FrameInfo frame{901,456};
    require(productYuvStride(raw.data(),80,64,raw.size(),raw.size(),frame)==80,"Valid YUV stride changed");
    imageFailure([&]{(void)productYuvStride(nullptr,80,64,raw.size(),raw.size(),frame);});
    imageFailure([&]{(void)productYuvStride(raw.data(),81,64,raw.size(),raw.size(),frame);});
    imageFailure([&]{(void)productYuvStride(raw.data(),80,63,raw.size(),raw.size(),frame);});
    imageFailure([&]{(void)productYuvStride(raw.data(),80,64,raw.size()-1,raw.size(),frame);});
    imageFailure([&]{(void)productYuvStride(raw.data(),80,64,raw.size(),raw.size()-1,frame);});
    imageFailure([&]{validateProductYuv(raw.data(),80,64,78,raw.size(),frame);});
    imageFailure([&]{validateProductYuv(raw.data(),80,64,81,raw.size(),frame);});
    const perception::ImageView image{raw.data(),raw.size(),80,80,64,perception::PixelFormat::NV12};
    for(int lane=0;lane<2;++lane) {
        auto cfg=config(lane==0,lane==1);
        // Disabled lane configuration must not constrain the enabled model.
        if(lane==0) {cfg.seg.resize_type=9;cfg.seg.fused_model_path="disabled-unused.bin";}
        else cfg.detect.resize_type=9;
        {
            r25::NativeStages stages(cfg);
            require(models==1,"NativeStages loaded a disabled model");
            require(stages.width(lane)==64&&stages.height(lane)==64,"Enabled stage geometry unavailable");
            rejected([&]{(void)stages.width(1-lane);},"Disabled width lane accepted");
            rejected([&]{(void)stages.height(-1);},"Invalid height lane accepted");
            rejected([&]{stages.preprocess(1-lane,image);},"Disabled preprocess lane accepted");
            rejected([&]{stages.preprocess(2,image);},"Out-of-bounds lane accepted");
            DnnInferTiming timing;
            rejected([&]{stages.infer(1-lane,timing);},"Disabled backend lane accepted");
            if(lane==0) rejected([&]{(void)stages.segmentation();},"Disabled segmentation accepted");
            else rejected([&]{(void)stages.detection();},"Disabled detection accepted");
            r22::OriginalSourceScope source(true,80);r20::setPolicy(false,false);
            stages.preprocess(lane,image);stages.infer(lane,timing);
            if(lane==0) require(!stages.detection().empty(),"Enabled detection output empty");
            else require(stages.segmentation().size()==80*64,"Enabled segmentation mask invalid");
            require(active==0,"Single-lane inference leaked a task");
        }
        require(models==0&&allocated==0,"Single-lane NativeStages teardown leaked");
    }
    int runs=0;
    for(const auto& cfg:{config(true,false),config(false,true),config(true,true),config(true,true,false)}) {
        const int expected_models=int(cfg.detect.enabled)+int(cfg.seg.enabled);
        {
            ExactDnnRunner adapter(profiles);require(adapter.init(cfg),"Product subset initialization failed");
            require(models==expected_models,"Product adapter loaded unexpected models");
            adapter.setFrameInfo(frame);
            HorizonDnnRunner oracle;require(oracle.init(cfg),"Original subset oracle initialization failed");
            for(bool nv12:{true,false}) {
                r22::OriginalSourceScope source(nv12,80);r20::setPolicy(false,false);
                std::vector<DnnDetection> expected_det,det(2);
                std::vector<uint8_t> expected_mask,mask(12,42);DnnInferTiming timing;
                require(oracle.inferNv12(raw.data(),80,64,80,raw.size(),nv12,
                    &expected_det,&expected_mask,nullptr,nullptr),"Original subset inference failed");
                require(adapter.inferNv12(raw.data(),80,64,80,raw.size(),nv12,
                    cfg.detect.enabled?&det:nullptr,&mask,nullptr,&timing),"Product subset DAG inference failed");
                if(cfg.detect.enabled) compareSubset(expected_det,det);
                require(mask==expected_mask,"Subset mask differs from original");
                if(cfg.detect.enabled) {
                    require(adapter.inferNv12(raw.data(),80,64,80,raw.size(),nv12,
                        &det,nullptr,nullptr,&timing),"Product detection-only request failed");
                    compareSubset(expected_det,det);
                }
                det.resize(2);mask.assign(12,42);timing.detect_ms=123;
                require(!adapter.inferNv12(raw.data(),80,64,78,raw.size(),nv12,
                    &det,&mask,nullptr,&timing),"Product invalid image accepted");
                require(adapter.lastStatus().code==1002&&adapter.lastStatus().frame.id==901,
                    "Product invalid image status/frame lost");
                require(det.empty()&&mask.empty()&&timing.detect_ms==0,"Failure exposed stale outputs");
                require(adapter.inferNv12(raw.data(),80,64,80,raw.size(),nv12,
                    &det,&mask,nullptr,&timing),"Valid frame did not recover after failure");
                ++runs;
            }
        }
        require(active==0&&allocated==0&&models==0,"Product subset lifecycle leaked");
    }
    std::cout<<"Product enabled-lane/serial selection, NativeStages subset loading/guards, "
        <<runs<<" NV12/I420 product DAG oracle comparisons, structured invalid-image failure/recovery passed. NOT real X5 acceptance.\n";
    return 0;
} catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}}
