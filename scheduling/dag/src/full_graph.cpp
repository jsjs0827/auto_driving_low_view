#include "full_graph.hpp"
#include "json.hpp"
#include "algorithm_runtime.hpp"
#include <mutex>
#include <set>
namespace r33::full {
namespace rt=perception::runtime;
namespace {
struct Step {std::string key,feature;std::vector<std::string> parents;std::uint32_t code;};
const std::vector<Step>& steps(){static const std::vector<Step> v=[] {std::vector<Step> result{
    {"dnn","product.ai.exact",{},1011},
    {"det","product.ai.detection",{},1011},
    {"seg","product.ai.segmentation",{},1011},
    {"recharge","product.recharge_mark",{"dnn"},2004},
    {"disparity","product.stereo.disparity",{},1002},
    {"rgb","product.image.rgb",{"dnn","recharge"},1002},
    // Only the unconfigured legacy fallback retains the old RGB ordering.
    // Configured v4 graphs use explicit dependencies and lazy RGB conversion.
    {"cloud","product.pointcloud.from_disparity",{"rgb","disparity"},1002},
    {"calibration","product.calibration.auto",{"cloud"},2005},
    {"labels","product.fusion.semantic_labels",{"cloud","dnn","det","seg","calibration"},1007},
    {"transform","product.pointcloud.transform_color",{"labels","cloud"},1007},
    {"ground","product.pointcloud.patchwork",{"transform"},1007},
    {"maps","product.fusion.ai_fusion",{"ground","labels"},1007},
    {"history","product.temporal.history",{"dnn","det","seg","disparity"},1002},
    {"visual_motion","product.motion.visual",{"history"},2011},
    {"motion_reference","product.motion.reference",{"history"},2011},
    {"slip_detection","product.motion.slip",{"visual_motion","motion_reference"},2011},
    {"temporal_difference","product.temporal.difference",{"history"},2010},
    {"temporal_match","product.temporal.match",{"history","temporal_difference"},2010},
    {"temporal_extension","product.temporal.extension",{"history","temporal_difference","temporal_match"},2010},
    {"publish","product.output.application",{"maps","recharge","dnn","det","seg","rgb","disparity","history","visual_motion","motion_reference","slip_detection","temporal_difference","temporal_match","temporal_extension"},2003},
    {"record","product.output.record",{"publish"},2002}
};for(const auto& entry:perception::algorithms::catalog())result.push_back({entry.first,entry.second.feature,{},1011});return result;}();return v;}
struct Session {const Graph::ContextBindings* work=nullptr;perception::FrameInfo frame;std::mutex mutex;r28::Status failure;};
class BoundFeature:public rt::Feature {
    std::shared_ptr<Session> session_;Step step_;
public:
    BoundFeature(std::shared_ptr<Session> s,Step step):session_(std::move(s)),step_(std::move(step)){}
    void execute(rt::FeatureContext& context)override {
        try {const auto binding=perception::algorithms::find(step_.key)?perception::algorithms::bindingKey(context.taskId()):step_.key;
            session_->work->at(binding)(context);context.set(std::string("full.completed.")+
                (perception::algorithms::find(step_.key)?"algorithm."+context.taskId():step_.key),session_->frame);}
        catch(const std::exception& e){std::lock_guard<std::mutex> lock(session_->mutex);if(!session_->failure.code)session_->failure=r28::statusFromException(e,step_.code,step_.feature,context.taskId(),session_->frame);throw;}
        catch(...){std::lock_guard<std::mutex> lock(session_->mutex);if(!session_->failure.code)session_->failure={step_.code,0,step_.feature,context.taskId(),"Unknown Feature exception",session_->frame};throw;}
    }
};
}
struct Graph::Impl {
    rt::FeatureRegistry registry;std::shared_ptr<Session> session=std::make_shared<Session>();
    ProductGraphPlan plan;
    std::map<std::string,std::unique_ptr<rt::TaskGraph>> graphs;
    Impl(){for(const auto& step:steps()){auto s=session;registry.add(step.feature,[s,step](const rt::Parameters&){return std::make_unique<BoundFeature>(s,step);});}}
};
Graph::Graph():p_(std::make_unique<Impl>()){}Graph::~Graph()=default;
void Graph::configure(const ProductGraphPlan& plan) {p_->graphs.clear();p_->plan=plan;}
std::uint32_t Graph::run(const Bindings& bindings,perception::FrameInfo frame,rt::GraphReport& report,r28::Status& status) {
    ContextBindings contextual;
    for(const auto& job:bindings)if(job.second)contextual[job.first]=[f=job.second](rt::FeatureContext&){f();};else contextual[job.first]={};
    return runContext(contextual,frame,report,status);
}
std::uint32_t Graph::runContext(const ContextBindings& bindings,perception::FrameInfo frame,rt::GraphReport& report,r28::Status& status,
    std::shared_ptr<const r36::temporal::Window> window, InputSeed seed) {
    // Invalid bindings may fail before TaskGraph::run. Clear both public
    // outputs and the previous invocation before validating the next frame.
    report={};status={};p_->session->failure={};p_->session->frame=frame;p_->session->work=nullptr;
    try {
        if(bindings.empty())throw std::invalid_argument("No enabled full SDK work");
        std::string signature;std::set<std::string> active;
        for(const auto& job:bindings){bool known=false;std::string selected=job.first;
            if(job.first.rfind("algorithm:",0)==0){const auto id=job.first.substr(10);const auto task=p_->plan.stage_for_task.find(id);
                if(task!=p_->plan.stage_for_task.end()&&perception::algorithms::find(task->second)){known=true;selected=task->second;}}
            else for(const auto& s:steps())if(job.first==s.key)known=true;
            if(!known||!job.second)throw std::invalid_argument("Unknown/empty full Feature: "+job.first);
            if(!p_->plan.configured||p_->plan.enabled(selected)){active.insert(job.first);signature+=job.first+";";}}
        if(active.empty())throw std::invalid_argument("No configured product graph features have executable work");
        auto& graph=p_->graphs[signature];
        if(!graph){rt::TaskGraphConfig cfg;cfg.worker_threads=2;
            if(p_->plan.configured) {
                cfg=p_->plan.config;
                for(auto& task:cfg.tasks){const auto& stage=p_->plan.stage_for_task.at(task.id);
                    task.enabled=task.enabled&&active.count(perception::algorithms::find(stage)?perception::algorithms::bindingKey(task.id):stage);}
                const auto tasks=cfg.tasks;std::map<std::string,rt::TaskSpec> by_id;
                for(const auto& task:tasks)by_id.emplace(task.id,task);
                for(auto& task:cfg.tasks)if(task.enabled) {
                    std::set<std::string> kept;
                    std::function<void(const std::string&)> expand=[&](const std::string& id){const auto& parent=by_id.at(id);
                        if(parent.enabled)kept.insert(id);else for(const auto& p:parent.depends_on)expand(p);};
                    for(const auto& parent:task.depends_on)expand(parent);
                    task.depends_on.assign(kept.begin(),kept.end());
                }
            } else {
            std::map<std::string,std::vector<std::string>> parents;for(const auto& step:steps())parents[step.key]=step.parents;
            for(const auto& step:steps())if(active.count(step.key)) {
                rt::TaskSpec task;task.id=step.key;task.features={step.feature};std::set<std::string> kept;
                std::function<void(const std::string&)> expand=[&](const std::string& id){if(active.count(id))kept.insert(id);else for(const auto& p:parents.at(id))expand(p);};
                for(const auto& id:step.parents)expand(id);task.depends_on.assign(kept.begin(),kept.end());cfg.tasks.push_back(std::move(task));
            }}graph=std::make_unique<rt::TaskGraph>(std::move(cfg),p_->registry);
        }
        p_->session->failure={};p_->session->frame=frame;p_->session->work=&bindings;
        rt::ArtifactStore artifacts;if(seed)seed(artifacts);if(window)artifacts.putShared(r36::temporal::input_artifact_key,window);report=graph->run(artifacts);p_->session->work=nullptr;
        status=p_->session->failure;
        if(!report.ok()&&!status.code){status={1007,0,"full.graph",{},"Full graph failed without a stage status",frame};}
        return status.code;
    }catch(const std::exception& e){p_->session->work=nullptr;status=r28::statusFromException(e,2001,"full.graph",{},frame);return status.code;}
}
std::string traceJson(const rt::GraphReport& report,std::uint64_t id) {
    std::ostringstream o;o<<"{\"run_id\":"<<id<<",\"graph_ms\":"<<report.elapsed_ms<<",\"tasks\":[";bool first=true;
    for(const auto& t:report.tasks){if(!first)o<<',';first=false;o<<"{\"task\":"<<json::quote(t.id)<<",\"state\":"<<json::quote(rt::taskStateName(t.state))<<",\"ms\":"<<t.elapsed_ms<<",\"error\":"<<json::quote(t.error)<<'}';}o<<"]}";return o.str();
}
}
