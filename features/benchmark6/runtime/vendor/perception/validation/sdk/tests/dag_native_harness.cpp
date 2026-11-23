// Runs real stage bodies against host-only API fixtures, never a BPU benchmark.
#define main r24_donor_harness_main
#include "runner_harness.cpp"
#undef main
#include "dag_pipeline.hpp"
#include <algorithm>
#include <cfenv>

int main(int argc,char** argv){try{
    require(argc==2,"Profile directory required");
    const std::string profiles=argv[1];
    require(r24_donor_harness_main()==0,"Preserved native donor checks failed");
    DnnConfig cfg;cfg.enabled=true;cfg.detect.model_path="det.host.fixture";cfg.detect.classes_num=24;
    cfg.seg.model_path="seg.host.fixture";cfg.seg.class_names="a,b,c";
    int cases=0;
    for(const char* graph:{"dag_exact.json","dag_exact_serial.json"}){
        auto stages=std::make_shared<r25::NativeStages>(cfg);
        r25::Pipeline pipeline(profiles+"/"+graph,stages);
        const auto names=pipeline.registeredFeatures();
        require(std::find(names.begin(),names.end(),"ai.preprocess")!=names.end(),"Original built-ins lost");
        require(std::find(names.begin(),names.end(),"ai.exact.det.preprocess")!=names.end(),"Exact Feature missing");
        // Original runner is only a host fixture oracle. Production worker never invokes it.
        HorizonDnnRunner oracle;require(oracle.init(cfg),"Oracle init failed");
        perception::runtime::FrameResult retained;std::vector<uint8_t> retained_mask;
        for(int format=0;format<2;++format)for(int frame=0;frame<4;++frame){
            const int stride=frame%2?96:80;std::vector<uint8_t> raw(stride*64*3/2);
            unsigned seed=frame+1;for(auto& p:raw){seed=seed*1664525u+1013904223u;p=seed>>24;}
            const bool nv12=format==0;
            perception::ImageView image{raw.data(),raw.size(),std::size_t(stride),80,64,nv12?perception::PixelFormat::NV12:perception::PixelFormat::I420};
            r22::OriginalSourceScope source(nv12,stride);r20::setPolicy(false,true);Result reference;DnnInferTiming t;
            require(oracle.inferNv12(raw.data(),80,64,stride,raw.size(),nv12,&reference.det,&reference.mask,nullptr,&t),"Original oracle failed");reference.audit=r20::takeAudit();
            for(bool optimized:{false,true})for(auto variant:{r22::Variant::R21,r22::Variant::CompactDirect}){
                r25::RunOptions options;options.optimized=optimized;options.capture=true;options.variant=variant;
                int depth_calls=0;options.depth=[&](std::uint64_t id){++depth_calls;return r25::DepthResult{true,id,0,2,2,4,1,0};};
                const auto id=std::uint64_t(frame);auto output=pipeline.run(image,{id,123},options);
                Result current{output.det,output.mask(),output.audit};equal(reference,current);++cases;
                require(output.frame.ok() && output.frame.frame.id==id && output.frame.frame.timestamp_ns==123,"Frame identity lost");
                require(output.frame.schedule.tasks.size()==4 && depth_calls==1 && output.depth.frame_id==id,"Depth/DAG route wrong");
                const auto& f=output.features;for(const auto& feature:f)require(feature.succeeded,"Feature skipped");
                require(f[0].end_ms<=f[1].start_ms && f[0].end_ms<=f[4].start_ms,"Input dependency violated");
                require(f[1].end_ms<=f[2].start_ms && f[2].end_ms<=f[3].start_ms,"Detection order wrong");
                require(f[4].end_ms<=f[5].start_ms && f[5].end_ms<=f[6].start_ms,"Segmentation order wrong");
                require(f[7].start_ms>=std::max(f[3].end_ms,f[6].end_ms),"Depth started before AI committed");
                require(f[1].thread==f[2].thread && f[2].thread==f[3].thread,"Detection nested workers found");
                require(f[4].thread==f[5].thread && f[5].thread==f[6].thread,"Segmentation nested workers found");
                require(&output.mask()==&output.frame.segmentation->mask,"Mask copied at framework export");
                require(!output.frame.artifacts->contains("exact.input") && !output.frame.artifacts->contains("exact.det.invocation"),"Borrowed/internal artifacts exported");
                if(retained.segmentation)require(retained.segmentation->mask==retained_mask,"Retained result overwritten by next frame");
                retained=output.frame;retained_mask=output.mask();
            }
        }
        // Original map stream shape and padded stride, through real DAG stage
        // bodies against the host API fixture. Still NOT real model/BPU output.
        for(int stride:{640,656}){
            std::vector<uint8_t> raw(std::size_t(stride)*544*3/2);unsigned seed=26;
            for(auto& p:raw){seed=seed*1664525u+1013904223u;p=seed>>24;}
            r22::OriginalSourceScope source(true,stride);r20::setPolicy(false,true);Result reference;DnnInferTiming t;
            require(oracle.inferNv12(raw.data(),640,544,stride,raw.size(),true,&reference.det,&reference.mask,nullptr,&t),"Original map oracle failed");reference.audit=r20::takeAudit();
            perception::ImageView map_image{raw.data(),raw.size(),std::size_t(stride),640,544,perception::PixelFormat::NV12};
            for(bool optimized:{false,true})for(auto variant:{r22::Variant::R21,r22::Variant::CompactDirect}){
                r25::RunOptions options;options.optimized=optimized;options.capture=true;options.variant=variant;
                auto output=pipeline.run(map_image,{640,544},options);Result current{output.det,output.mask(),output.audit};equal(reference,current);++cases;
                require(output.mask().size()==640*544&&output.frame.segmentation->width==640&&output.frame.segmentation->height==544,"Original mask dimensions changed");
            }
        }
        std::vector<uint8_t> raw(80*64*3/2,127);perception::ImageView image{raw.data(),raw.size(),80,80,64,perception::PixelFormat::NV12};
        for(int failure:{1,2,3}){
            fault=failure;int calls=0;r25::RunOptions options;options.depth=[&](std::uint64_t id){++calls;return r25::DepthResult{true,id,0};};
            bool rejected=false;try{(void)pipeline.run(image,{77,1},options);}catch(const std::exception& e){rejected=std::string(e.what()).find("DAG failed")!=std::string::npos;}
            require(rejected && active==0 && calls==0,"Failure did not stop dependent depth or release BPU task");fault=0;
            require(pipeline.run(image,{78,2},r25::RunOptions{}).frame.ok(),"Failed frame phase not reset");
        }
    }
    require(active==0 && allocated==0 && models==0,"DAG model/tensor/task leaked");
    std::cout<<"Original DAG native-stage host fixture: "<<cases<<" cases, zero input/storage/mask/box differences; graph order, frame IDs, retained ownership and failures passed. NOT real X5 acceptance.\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
