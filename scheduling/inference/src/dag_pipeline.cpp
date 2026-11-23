#include "dag_pipeline.hpp"
#include "seg_post.hpp"
#include "output_features.hpp"
#include "perception/inference_task.hpp"
#include <chrono>
#include <cfenv>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <algorithm>
namespace r25 {
namespace rt=perception::runtime;
using namespace ai_node_percep_demo;
using Clock=std::chrono::steady_clock;
namespace {
// The preserved numeric helpers use process-global per-lane workspaces and
// calibration policy. Serialize entire frames across Pipeline instances too.
std::mutex exact_frame_mutex;
double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
struct FpScope {
    std::fenv_t previous;
    explicit FpScope(const std::fenv_t& frame){std::fegetenv(&previous);std::fesetenv(&frame);}
    ~FpScope(){std::fesetenv(&previous);}
};
struct Packet {perception::ImageView image;perception::FrameInfo frame;};
struct Invocation {perception::FrameInfo frame;int lane=0;};
struct Session {
    std::shared_ptr<Stages> stages;
    RunOptions options;
    r20::PackWorkspace pack;
    std::array<r20::DetectorWorkspace,2> host;
    std::array<std::vector<uint8_t>,2> host_destination;
    std::array<FeatureTiming,8> trace;
    std::array<perception::inference::StagedInvocation,2> invocations;
    std::array<r28::Status,8> faults;
    r28::Status output_fault;
    std::mutex output_fault_mutex;
    perception::FrameInfo frame;
    DnnInferTiming timing;
    std::fenv_t fp;
    Clock::time_point start;
    std::uint64_t id=0;
};
class FunctionFeature final:public rt::Feature {
    std::shared_ptr<Session> session_;
    int index_;
    std::string name_;
    std::function<void(rt::FeatureContext&)> fn_;
public:
    FunctionFeature(std::shared_ptr<Session> s,int i,std::string n,std::function<void(rt::FeatureContext&)> fn)
        :session_(std::move(s)),index_(i),name_(std::move(n)),fn_(std::move(fn)){}
    void execute(rt::FeatureContext& c) override {
        FpScope fp(session_->fp);
        auto& t=session_->trace[index_];t.feature=name_;t.task=c.taskId();
        t.thread=std::hash<std::thread::id>{}(std::this_thread::get_id());
        t.start_ms=ms(session_->start,Clock::now());
        try{
            if(index_>=1&&index_<=6) {
                const int lane=(index_-1)/3;
                const auto stage=static_cast<perception::inference::Stage>((index_-1)%3);
                session_->invocations[lane].execute(stage,[&]{fn_(c);});
            } else fn_(c);
            t.succeeded=true;t.end_ms=ms(session_->start,Clock::now());
        }
        catch(const std::exception& e){
            t.end_ms=ms(session_->start,Clock::now());
            const auto fallback=index_==0?1002U:index_==7?1011U:1011U;
            session_->faults[index_]=r28::statusFromException(e,fallback,name_,c.taskId(),session_->frame);
            throw;
        }catch(...){
            t.end_ms=ms(session_->start,Clock::now());
            session_->faults[index_]={1011,0,name_,c.taskId(),"Unknown feature exception",session_->frame};
            throw;
        }
    }
};
std::string laneKey(int lane,const char* suffix){return std::string(lane?"exact.seg.":"exact.det.")+suffix;}
std::shared_ptr<const Invocation> invocation(rt::FeatureContext& c,int lane,const char* suffix,const Session& s){
    auto p=c.get<Invocation>(laneKey(lane,suffix));
    if(p->lane!=lane || p->frame.id!=s.id)throw std::logic_error("Cross-frame/lane invocation rejected");return p;
}
void hostPrepare(Session& s,int lane,const perception::ImageView& im){
    const int w=lane?832:864,h=480;const bool nv12=im.format==perception::PixelFormat::NV12;
    r20::Geometry g;std::vector<uint8_t> output;
    if(!s.options.optimized){
        if(!r20::reference::PreprocessYuv420(im.data,im.width,im.height,int(im.stride),im.bytes,nv12,w,h,lane?0:1,&g,output,127))throw std::runtime_error("Host scalar preprocess failed");
    }else if(r22::variant()==r22::Variant::R21){
        auto& ws=s.host[lane];if(!ws.prepare(im.data,im.width,im.height,int(im.stride),im.bytes,nv12,w,h,lane?0:1,g))throw std::runtime_error("Host r21 preprocess failed");output=ws.output;
    }else{
        auto& ws=r22::workspace(lane);const bool direct=r22::directVariant(r22::variant());auto& dst=s.host_destination[lane];
        if(direct)dst.resize(std::size_t(w)*h*3/2);
        if(!ws.prepare(im.data,im.width,im.height,int(im.stride),im.bytes,nv12,w,h,lane?0:1,g,direct?dst.data():nullptr,dst.size()))throw std::runtime_error("Host exact preprocess failed");output=direct?dst:ws.output;
    }
    r20::captureTensor(lane?"segmentation":"detection",output,output.data(),output.size(),w,h,g.x_shift,g.y_shift,g.valid_w,g.valid_h);
}
void registerExact(rt::FeatureRegistry& r,const std::shared_ptr<Session>& s){
    r.addCloser([s]{if(s->stages)s->stages->close();});
    auto add=[&](const char* name,int index,std::function<void(rt::FeatureContext&)> fn,rt::ExclusiveResourceResolver resources={}){
        const std::string registered_name(name);
        r.add(registered_name,[=](const rt::Parameters&){return std::make_unique<FunctionFeature>(s,index,registered_name,fn);},{},std::move(resources));
    };
    add("input.exact.pack",0,[s](rt::FeatureContext& c){
        auto image=*c.get<perception::ImageView>("input.image");const auto info=*c.get<perception::FrameInfo>("input.frame");
        if(info.id!=s->id)throw std::logic_error("Frame identity mismatch");
        if(s->options.optimized && !s->options.input_only){
            const auto b=s->pack.run(image.data,image.width,image.height,int(image.stride),image.bytes,image.format==perception::PixelFormat::NV12);
            image={b.data,b.bytes,std::size_t(b.stride),image.width,image.height,perception::PixelFormat::NV12};
        }
        c.set("exact.input",Packet{image,info});
    },[](const rt::Parameters&){return std::vector<std::string>{"exact.pack.workspace"};});
    for(int lane=0;lane<2;++lane){
        const std::string prefix=lane?"ai.exact.seg.":"ai.exact.det.";
        const auto resource=[lane](const rt::Parameters&){return std::vector<std::string>{lane?"exact.seg.model":"exact.det.model"};};
        add((prefix+"preprocess").c_str(),1+lane*3,[s,lane](rt::FeatureContext& c){
            if(!(lane?s->options.run_segmentation:s->options.run_detection))
                throw std::logic_error("Disabled AI lane scheduled; use a matching DAG profile");
            const auto packet=c.get<Packet>("exact.input");
            if(packet->frame.id!=s->id)throw std::logic_error("Stale input packet");
            if(s->options.preprocess_only)hostPrepare(*s,lane,packet->image);
            else {if(!s->stages)throw std::logic_error("No BPU stages");s->stages->preprocess(lane,packet->image);}
            c.set(laneKey(lane,"invocation"),Invocation{packet->frame,lane});
        },resource);
        add((prefix+"backend").c_str(),2+lane*3,[s,lane](rt::FeatureContext& c){
            auto p=invocation(c,lane,"invocation",*s);
            if(!s->options.preprocess_only)s->stages->infer(lane,s->timing);
            c.set(laneKey(lane,"inferred"),*p);
        },resource);
        add((prefix+"postprocess").c_str(),3+lane*3,[s,lane](rt::FeatureContext& c){
            (void)invocation(c,lane,"inferred",*s);const auto packet=c.get<Packet>("exact.input");
            const auto timing=[&]{perception::Timing t;
                const auto& pre=s->trace[1+lane*3];const auto& bpu=s->trace[2+lane*3];
                t.preprocess_ms=pre.end_ms-pre.start_ms;t.inference_ms=bpu.end_ms-bpu.start_ms;
                t.postprocess_ms=ms(s->start,Clock::now())-s->trace[3+lane*3].start_ms;return t;};
            if(lane){perception::SegmentationResult result;result.width=packet->image.width;result.height=packet->image.height;
                if(!s->options.preprocess_only)result.mask=s->stages->segmentation();
                result.timing=timing();
                c.set("ai.segmentation",std::move(result));
                c.set("ai.segmentation.frame",packet->frame);
            }else{std::vector<DnnDetection> boxes;if(!s->options.preprocess_only)boxes=s->stages->detection();
                perception::DetectionResult result;result.width=packet->image.width;result.height=packet->image.height;
                for(const auto& b:boxes)result.detections.push_back({b.class_id,b.confidence,b.x1,b.y1,b.x2,b.y2});
                result.timing=timing();
                c.set("ai.detection",std::move(result));c.set("exact.detections",std::move(boxes));
                c.set("ai.detection.frame",packet->frame);
            }
        },resource);
    }
    add("stereo.demand",7,[s](rt::FeatureContext& c){
        const auto packet=c.get<Packet>("exact.input");DepthResult d;d.frame_id=packet->frame.id;
        if(s->options.depth){d=s->options.depth(packet->frame.id);d.requested=true;
            if(d.frame_id!=packet->frame.id)throw std::runtime_error("Depth frame identity mismatch");
            if(d.code!=0)throw r28::Failure({1011,d.code,"stereo.demand",c.taskId(),"Requested disparity failed, code "+std::to_string(d.code),packet->frame});
            if(d.width<=0||d.height<=0||d.pixels!=std::size_t(d.width)*d.height)throw std::runtime_error("Requested disparity geometry invalid");}
        c.set("stereo.demand.result",d);
    },[](const rt::Parameters&){return std::vector<std::string>{"camera.disparity"};});
}
}
struct Pipeline::Impl {std::shared_ptr<Session> session=std::make_shared<Session>();std::unique_ptr<rt::Engine> engine;bool has_depth_feature=false;};
Pipeline::Pipeline(const std::string& graph,std::shared_ptr<Stages> stages):impl_(std::make_unique<Impl>()){
    impl_->session->stages=std::move(stages);rt::FeatureRegistry registry;
    rt::registerBuiltinFeatures(registry); // Preserve original built-ins and extension API.
    registerExact(registry,impl_->session);
    r28::registerOutputFeatures(registry,[session=impl_->session](const r28::Status& fault){
        std::lock_guard<std::mutex> lock(session->output_fault_mutex);
        if(session->output_fault.ok())session->output_fault=fault;
    });
    auto config=rt::loadPerceptionEngineConfig(graph);
    for(const auto& task:config.tasks)if(task.enabled)
        for(const auto& feature:task.features)if(feature=="stereo.demand")impl_->has_depth_feature=true;
    impl_->engine=std::make_unique<rt::Engine>(std::move(config),std::move(registry));
}
Pipeline::~Pipeline(){
    try{close();}catch(...){} // Call tryClose at a public boundary to retain errors.
}
void Pipeline::close(){
    std::lock_guard<std::mutex> lock(exact_frame_mutex);
    impl_->engine->close();
}
std::uint32_t Pipeline::tryClose(r28::Status& status)noexcept{
    status={};
    try{close();return 0;}
    catch(const std::exception& error){
        try{status=r28::statusFromException(error,r28::ResourceCleanupFailed,"dag.close");}
        catch(...){status.code=r28::ResourceCleanupFailed;}
    }catch(...){status.code=r28::ResourceCleanupFailed;status.feature="dag.close";}
    return status.code;
}
std::vector<std::string> Pipeline::registeredFeatures() const{return impl_->engine->registeredFeatures();}
Output Pipeline::run(const perception::ImageView& image,perception::FrameInfo frame,const RunOptions& options){
    std::lock_guard<std::mutex> lock(exact_frame_mutex);
    if(options.depth&&!impl_->has_depth_feature)throw std::invalid_argument("Requested depth provider has no enabled inference graph depth feature");
    auto& s=*impl_->session;s.options=options;s.id=frame.id;s.frame=frame;s.trace={};s.faults={};s.output_fault={};s.timing={};std::fegetenv(&s.fp);
    for(int lane=0;lane<2;++lane)
        s.invocations[lane].begin(lane?"seg":"det",frame,{frame},1);
    r22::OriginalSourceScope source(image.format==perception::PixelFormat::NV12,int(image.stride));
    r22::VariantScope variant(options.variant);r22::resetStats();r20::setPolicy(options.optimized,options.capture);
    s.start=Clock::now();perception::runtime::FrameInput input;input.image=image;input.frame=frame;input.disparity=options.disparity_view;
    Output out;
    try{out.frame=impl_->engine->run(input);}catch(...){if(s.stages)s.stages->abortFrame();throw;}
    if(!out.frame.ok()){
        if(s.stages)s.stages->abortFrame();
        for(auto fault:s.faults)if(!fault.ok()){
            fault.detail="DAG failed; "+fault.feature+": "+fault.detail;
            throw r28::Failure(std::move(fault));
        }
        if(!s.output_fault.ok())throw r28::Failure(s.output_fault);
        std::string error="DAG failed";for(const auto& t:out.frame.schedule.tasks)if(t.state==rt::TaskState::Failed)error+="; "+t.id+": "+t.error;
        // Extension tasks may fail outside the native exact feature group.
        for(const auto& t:out.frame.schedule.tasks)if(t.state==rt::TaskState::Failed)
            throw r28::Failure({t.id=="output"?r28::OutputFailed:1011U,0,{},t.id,error,frame});
        throw r28::Failure({1011,0,{}, {},error,frame});
    }
    out.graph_ms=ms(s.start,Clock::now());out.features=s.trace;out.timing=s.timing;
    const auto duration=[&](int i){return s.trace[i].end_ms-s.trace[i].start_ms;};
    // A concurrent depth interval overlaps AI. Subtracting its duration would
    // undercount AI and can become negative. AI completion and full graph are
    // separate measured scopes in the three-branch experiment.
    out.wall_ms=options.input_only?duration(0):std::max(options.run_detection?s.trace[3].end_ms:0.,options.run_segmentation?s.trace[6].end_ms:0.);
    if(!s.trace[0].succeeded||(impl_->has_depth_feature&&!s.trace[7].succeeded))throw std::runtime_error("Required input/depth Feature did not execute");
    if(!options.input_only){
        for(int lane=0;lane<2;++lane)if(lane?options.run_segmentation:options.run_detection)
            for(int i=1+lane*3;i<=3+lane*3;++i)if(!s.trace[i].succeeded)
                throw r28::Failure({1012,0,s.trace[i].feature,{},"Required AI Feature did not execute",frame});
        if((options.run_detection&&!out.frame.detection)||(options.run_segmentation&&!out.frame.segmentation))
            throw r28::Failure({1012,0,{}, {},"DAG final artifacts missing",frame});
    }
    out.pack_ms=duration(0);out.timing.detect_pre_ms=duration(1);out.timing.detect_bpu_ms=duration(2);
    out.det_infer_ms=duration(2);out.seg_infer_ms=duration(5);
    out.timing.detect_ms=duration(1)+duration(2)+duration(3); // execution time, not branch idle/scheduler wait
    out.timing.seg_pre_ms=duration(4);out.timing.seg_bpu_ms=duration(5);out.timing.seg_post_ms=duration(6);
    out.timing.seg_ms=duration(4)+duration(5)+duration(6);
    if(out.frame.detection)out.det=*out.frame.artifacts->get<std::vector<DnnDetection>>("exact.detections");
    if(out.frame.artifacts->contains("stereo.demand.result"))out.depth=*out.frame.artifacts->get<DepthResult>("stereo.demand.result");
    if(out.frame.artifacts->contains("output.visualization")){
        const auto v=out.frame.artifacts->get<r28::OutputVisualization>("output.visualization");
        out.output_ms=v->output_ms;out.render_ms=v->render_ms;out.encode_write_ms=v->encode_write_ms;
    }
    out.audit=r20::takeAudit();out.inputs={r22::stats(0),r22::stats(1)};
    return out;
}
std::uint32_t Pipeline::tryRun(const perception::ImageView& image,perception::FrameInfo frame,
                              const RunOptions& options,Output& output,r28::Status& status) noexcept {
    try { output=Output{};status=r28::Status{};output=run(image,frame,options);return 0; }
    catch(const std::exception& e){
        output=Output{};
        try{status=r28::statusFromException(e,1011,"dag.run",{},frame);}
        catch(...){status.code=1011;status.native_code=0;status.frame=frame;}
        return status.code;
    }catch(...){
        output=Output{};status.code=1011;status.native_code=0;status.frame=frame;
        return status.code;
    }
}
}
