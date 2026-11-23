#include "product/product_events.hpp"
#include "platform_metrics.hpp"
#include "perception/runtime_engine.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <stdexcept>

namespace r30::product {
namespace rt=perception::runtime;
namespace {
struct Step {const char* event;const char* key;const char* feature;std::uint32_t code;};
const std::vector<Step>& steps(){
    static const std::vector<Step> value{
        {"command","command","product.control.command",2001},
        {"camera_params","params","product.input.camera_params",1002},
        {"camera_params","params_save","product.output.camera_params",2002},
        {"record_io","record_io","product.output.record_io",2002},
        {"recharge_result","recharge_publish","product.output.recharge",2003},
        {"offline_read","offline_read","product.input.offline",1002}};
    return value;
}
struct Invocation {
    EventContext context;
    ProductEvents::Bindings bindings;
    mutable r28::Status failure;
};
class EventFeature final:public rt::Feature {
    Step step_;
public:
    explicit EventFeature(Step step):step_(step){}
    void execute(rt::FeatureContext& context) override {
        const auto job=context.get<Invocation>("input.product.event");
        if(!job)throw std::logic_error("product event invocation absent");
        try {
            const auto action=job->bindings.find(step_.key);
            if(action==job->bindings.end()||!action->second)throw std::logic_error("product event binding absent");
            action->second();
            context.set(std::string("product.event.completed.")+step_.key,job->context);
        }catch(const std::exception& error){
            job->failure=r28::statusFromException(error,step_.code,step_.feature,context.taskId(),job->context.frame);
            throw;
        }catch(...){
            job->failure={};job->failure.code=step_.code;job->failure.feature=step_.feature;
            job->failure.task=context.taskId();job->failure.frame=job->context.frame;
            job->failure.detail="unknown product event exception";throw;
        }
    }
};
rt::TaskGraphConfig defaults(){
    rt::TaskGraphConfig config;config.worker_threads=1;
    for(const auto& step:steps()){
        rt::TaskSpec task;task.id=step.key;task.features={step.feature};
        if(task.id=="params_save")task.depends_on={"params"};
        config.tasks.push_back(std::move(task));
    }
    return config;
}
const Step& byFeature(const std::string& name){
    const auto found=std::find_if(steps().begin(),steps().end(),[&](const auto& step){return name==step.feature;});
    if(found==steps().end())throw std::invalid_argument("unknown product event Feature: "+name);
    return *found;
}
}
struct ProductEvents::Impl {
    rt::FeatureRegistry registry;
    // Never mutate graphs after construction; distinct event types can run
    // concurrently, while TaskGraph serializes repeated runs of the same type.
    std::map<std::string,std::unique_ptr<rt::TaskGraph>> graphs;
    std::mutex metrics_mutex;
    std::ofstream metrics,statuses;
    std::atomic<std::uint64_t> sequence{0};
    std::atomic<bool> closed{false};
};
ProductEvents::ProductEvents(const std::string& path,const std::string& metrics_dir):impl_(std::make_unique<Impl>()){
    for(const auto& step:steps())impl_->registry.add(step.feature,[step](const rt::Parameters&){return std::make_unique<EventFeature>(step);});
    auto config=path.empty()?defaults():rt::loadPerceptionEngineConfig(path);
    config.validate(impl_->registry);
    if(config.worker_threads!=1)throw std::invalid_argument("product event graphs require worker_threads=1");
    std::map<std::string,std::string> task_events,task_features;
    std::set<std::string> features;
    for(const auto& task:config.tasks){
        if(!task.enabled||task.features.size()!=1||!features.insert(task.features.front()).second)
            throw std::invalid_argument("product events require enabled, unique single-Feature tasks");
        const auto& step=byFeature(task.features.front());task_events[task.id]=step.event;task_features[task.id]=step.feature;
    }
    for(const auto& step:steps())if(!features.count(step.feature))throw std::invalid_argument(std::string("product event config missing ")+step.feature);
    bool save_ordered=false;
    for(const auto& task:config.tasks){
        for(const auto& dependency:task.depends_on){
            if(task_events.at(task.id)!=task_events.at(dependency))throw std::invalid_argument("product event graphs cannot depend on another callback type");
            if(task_features.at(task.id)=="product.output.camera_params"&&task_features.at(dependency)=="product.input.camera_params")save_ordered=true;
        }
    }
    if(!save_ordered)throw std::invalid_argument("camera parameter persistence must depend on parameter acceptance");
    for(const auto& event:std::vector<std::string>{"command","camera_params","record_io","recharge_result","offline_read"}){
        for(int save=0;save<(event=="camera_params"?2:1);++save){
            auto projected=config;projected.tasks.clear();
            for(const auto& task:config.tasks)if(task_events.at(task.id)==event && (event!="camera_params"||save||task.features.front()!="product.output.camera_params"))projected.tasks.push_back(task);
            const auto key=event+(event=="camera_params"?(save?"+save":"+accept"):"");
            impl_->graphs.emplace(key,std::make_unique<rt::TaskGraph>(std::move(projected),impl_->registry));
        }
    }
    if(!metrics_dir.empty()){
        std::filesystem::create_directories(std::filesystem::u8path(metrics_dir));
        const auto directory=std::filesystem::u8path(metrics_dir);
        const auto csv=directory/"product_event_metrics.csv";
        const bool header=!std::filesystem::exists(csv)||std::filesystem::file_size(csv)==0;
        impl_->metrics.open(csv,std::ios::binary|std::ios::app);
        impl_->statuses.open(directory/"product_event_status.jsonl",std::ios::binary|std::ios::app);
        if(!impl_->metrics||!impl_->statuses)throw std::runtime_error("cannot open product event metrics");
        if(header)impl_->metrics<<"event_id,parent_run_id,event,frame_id,timestamp_ns,total_wall_ms,total_cpu_ms,rss_kib,cma_free_kib,code,task,state,task_ms\n";
    }
}
ProductEvents::~ProductEvents()=default;
std::vector<std::string> ProductEvents::registeredFeatures()const{return impl_->registry.names();}
void ProductEvents::close(){
    if(impl_->closed.exchange(true))return;
    std::exception_ptr first;
    for(auto& graph:impl_->graphs)try{graph.second->close();}catch(...){if(!first)first=std::current_exception();}
    if(first)std::rethrow_exception(first);
}
std::uint32_t ProductEvents::run(const std::string& event,const Bindings& bindings,perception::FrameInfo frame,
    std::uint64_t parent,rt::GraphReport& report,r28::Status& status)noexcept{
    status={};report={};
    try{
        if(impl_->closed.load())throw std::invalid_argument("product event runner is closed");
        std::set<std::string> allowed;
        for(const auto& step:steps())if(event==step.event)allowed.insert(step.key);
        if(allowed.empty())throw std::invalid_argument("unknown product event: "+event);
        for(const auto& binding:bindings)if(!allowed.count(binding.first)||!binding.second)throw std::invalid_argument("unknown or empty product event binding: "+binding.first);
        for(const auto& key:allowed)if(key!="params_save"&&!bindings.count(key))throw std::invalid_argument("required product event binding missing: "+key);
        const auto name=event+(event=="camera_params"?(bindings.count("params_save")?"+save":"+accept"):"");
        Invocation job;job.context={event,++impl_->sequence,parent,frame};job.bindings=bindings;
        auto shared=std::make_shared<const Invocation>(std::move(job));
        rt::ArtifactStore store;store.putShared("input.product.event",shared);
        const auto start=std::chrono::steady_clock::now();const auto cpu=r29::processCpuMs();
        report=impl_->graphs.at(name)->run(store);status=shared->failure;status.frame=frame;
        if(!report.ok()&&!status.code){status.code=1007;status.feature="product.event";status.detail="product event graph failed";}
        const double wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        const double cpu_ms=r29::processCpuMs()-cpu;const auto resources=r29::processResources();
        const auto resource=[&](const char* key){const auto found=resources.find(key);return found==resources.end()?-1L:found->second;};
        std::lock_guard<std::mutex> lock(impl_->metrics_mutex);
        if(impl_->metrics.is_open()){
            for(const auto& task:report.tasks)impl_->metrics<<shared->context.event_id<<','<<parent<<','<<event<<','<<frame.id<<','<<frame.timestamp_ns<<','<<wall_ms<<','<<cpu_ms<<','<<resource("VmRSS:")<<','<<resource("CmaFree:")<<','<<status.code<<','<<task.id<<','<<rt::taskStateName(task.state)<<','<<task.elapsed_ms<<'\n';
            const auto json=r28::statusJson(status);
            impl_->statuses<<"{\"event_id\":"<<shared->context.event_id<<",\"parent_run_id\":"<<parent<<",\"event\":\""<<event<<"\","<<json.substr(1)<<'\n';
            impl_->metrics.flush();impl_->statuses.flush();
            if(!impl_->metrics||!impl_->statuses){status.code=2002;status.feature="product.events.metrics";status.detail="product event metrics write failed";}
        }
        return status.code;
    }catch(const std::exception& error){status=r28::statusFromException(error,2001,"product.events","",frame);return status.code;}
    catch(...){status.code=1007;status.feature="product.events";status.frame=frame;status.detail="unknown product event graph failure";return status.code;}
}
}
