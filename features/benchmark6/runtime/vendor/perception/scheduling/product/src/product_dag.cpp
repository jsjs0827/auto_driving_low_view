#include "product/product_dag.hpp"
#include "perception/runtime_engine.hpp"
#include "product/status_api.hpp"
#include "platform_metrics.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <mutex>
#include <set>
#include <stdexcept>

namespace r28::product {
namespace rt = perception::runtime;
namespace {
const std::vector<std::pair<std::string, std::string>>& steps() {
    static const std::vector<std::pair<std::string, std::string>> value{
        {"dnn", "product.ai.exact"},
        {"recharge", "product.recharge_mark"},
        {"rgb", "product.image.rgb"},
        {"cloud", "product.pointcloud.from_disparity"},
        {"calibration", "product.calibration.auto"},
        {"labels", "product.fusion.semantic_labels"},
        {"transform", "product.pointcloud.transform_color"},
        {"ground", "product.pointcloud.patchwork"},
        {"maps", "product.fusion.ai_fusion"},
        {"publish", "product.output.ros_maps"},
        {"record", "product.output.record_latest"}};
    return value;
}
std::uint32_t fallback(const std::string& step) {
    if (step == "dnn") return 1011;
    if (step == "recharge") return 2004;
    if (step == "calibration") return 2005;
    if (step == "rgb" || step == "cloud") return 1002;
    if (step == "publish") return 2003;
    if (step == "record") return 2002;
    return 1007;
}
struct Session {
    const ProductDag::Bindings* bindings = nullptr;
    perception::FrameInfo frame;
    std::mutex status_mutex;
    r28::Status first;
};
class ProductFeature final : public rt::Feature {
    std::shared_ptr<Session> session_;
    std::string step_, name_;
public:
    ProductFeature(std::shared_ptr<Session> session, std::string step, std::string name)
        : session_(std::move(session)), step_(std::move(step)), name_(std::move(name)) {}
    void execute(rt::FeatureContext& context) override {
        try {
            if (!session_->bindings) throw std::logic_error("product frame bindings absent");
            const auto found = session_->bindings->find(step_);
            if (found == session_->bindings->end() || !found->second)
                throw std::logic_error("enabled product Feature has no implementation: " + name_);
            found->second();
        } catch (const std::exception& e) {
            auto status = r28::statusFromException(e, fallback(step_), name_, context.taskId(), session_->frame);
            std::lock_guard<std::mutex> lock(session_->status_mutex);
            if (!session_->first.code) session_->first = std::move(status);
            throw;
        } catch (...) {
            std::lock_guard<std::mutex> lock(session_->status_mutex);
            if (!session_->first.code) {
                session_->first.code = fallback(step_);
                session_->first.feature = name_;
                session_->first.task = context.taskId();
                session_->first.frame = session_->frame;
                session_->first.detail = "unknown product Feature exception";
            }
            throw;
        }
    }
};
rt::TaskGraphConfig defaultConfig() {
    rt::TaskGraphConfig cfg;
    cfg.worker_threads = 1; // Original product clouds/maps are mutable; preserve their order.
    const std::map<std::string, std::vector<std::string>> deps{
        {"recharge", {"dnn"}}, {"rgb", {"dnn", "recharge"}},
        {"cloud", {"rgb"}}, {"calibration", {"cloud"}},
        {"labels", {"cloud", "dnn"}}, {"transform", {"labels", "cloud"}},
        {"ground", {"transform"}}, {"maps", {"ground", "labels"}},
        {"publish", {"maps"}}, {"record", {"publish"}}};
    for (const auto& step : steps()) {
        rt::TaskSpec task; task.id = step.first; task.features = {step.second};
        const auto found = deps.find(task.id);
        if (found != deps.end()) task.depends_on = found->second;
        cfg.tasks.push_back(std::move(task));
    }
    return cfg;
}
}
struct ProductDag::Impl {
    rt::FeatureRegistry registry;
    std::shared_ptr<Session> session = std::make_shared<Session>();
    rt::TaskGraphConfig config;
    std::map<std::string, std::unique_ptr<rt::TaskGraph>> graphs;
    std::mutex mutex;
    std::ofstream metrics, statuses;
    std::atomic<std::uint64_t> sequence{0};
};
ProductDag::ProductDag(const std::string& path,const std::string& metrics_dir) : impl_(std::make_unique<Impl>()) {
    for (const auto& step : steps()) {
        const auto session = impl_->session;
        impl_->registry.add(step.second, [session, step](const rt::Parameters&) {
            return std::make_unique<ProductFeature>(session, step.first, step.second);
        });
    }
    impl_->config = path.empty() ? defaultConfig() : rt::loadPerceptionEngineConfig(path);
    impl_->config.validate(impl_->registry);
    if (impl_->config.worker_threads != 1)
        throw std::invalid_argument("product mutable clouds/maps require worker_threads=1; AI lanes parallelize inside their exact DAG");
    // Product tasks may be reordered by DAG dependencies, but each implementation
    // must occur exactly once. Silently dropping publication is never accepted.
    std::set<std::string> names;
    for (const auto& task : impl_->config.tasks) {
        if (!task.enabled) throw std::invalid_argument("product tasks cannot be disabled in JSON; frame demand selects active tasks");
        if (task.features.size() != 1 || !names.insert(task.features.front()).second)
            throw std::invalid_argument("product DAG requires one unique Feature per task");
    }
    for (const auto& step : steps())
        if (!names.count(step.second)) throw std::invalid_argument("product DAG missing " + step.second);
    std::map<std::string,std::string> actual_ids;
    std::map<std::string,std::vector<std::string>> dependencies;
    for (const auto& task : impl_->config.tasks) {
        actual_ids[task.features.front()]=task.id; dependencies[task.id]=task.depends_on;
    }
    const auto expected=defaultConfig();
    std::map<std::string,std::string> expected_features;
    for (const auto& task : expected.tasks) expected_features[task.id]=task.features.front();
    const auto reaches=[&](const std::string& start,const std::string& target) {
        std::set<std::string> visited;
        std::function<bool(const std::string&)> visit=[&](const std::string& id) {
            if(id==target) return true;
            if(!visited.insert(id).second) return false;
            for(const auto& parent:dependencies[id]) if(visit(parent)) return true;
            return false;
        };
        return visit(start);
    };
    for(const auto& task:expected.tasks) for(const auto& parent:task.depends_on)
        if(!reaches(actual_ids.at(task.features.front()),actual_ids.at(expected_features.at(parent))))
            throw std::invalid_argument("product DAG weakens required dependency: "+parent+" -> "+task.id);
    if(!metrics_dir.empty()) {
        std::filesystem::create_directories(metrics_dir);
        const auto csv=std::filesystem::path(metrics_dir)/"product_metrics.csv";
        const bool header=!std::filesystem::exists(csv)||std::filesystem::file_size(csv)==0;
        impl_->metrics.open(csv,std::ios::app);
        impl_->statuses.open(std::filesystem::path(metrics_dir)/"product_status.jsonl",std::ios::app);
        if(!impl_->metrics||!impl_->statuses) throw std::runtime_error("cannot open product metrics output");
        if(header) impl_->metrics<<"run_id,frame_id,timestamp_ns,total_wall_ms,total_cpu_ms,rss_kib,cma_free_kib,code,task,state,task_ms\n";
    }
}
ProductDag::~ProductDag() = default;
std::vector<std::string> ProductDag::registeredFeatures() const { return impl_->registry.names(); }
std::uint64_t ProductDag::currentRunId() const noexcept {return impl_->sequence.load();}
void ProductDag::recordExternalStatus(const r28::Status& status,std::uint64_t run_id) noexcept {
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if(impl_->statuses.is_open()) {
            const auto json=r28::statusJson(status);
            impl_->statuses<<"{\"run_id\":"<<run_id<<','<<json.substr(1)<<'\n';impl_->statuses.flush();
            if(!impl_->statuses) {
                r28::Status failure;failure.code=2002;failure.feature="product.metrics";
                failure.frame=status.frame;failure.detail="product asynchronous status log write failed";
                setLastStatus(failure);
            }
        }
    } catch(...) {
        try {r28::Status failure;failure.code=2002;failure.feature="product.metrics";
            failure.detail="product asynchronous status log failure";setLastStatus(failure);} catch(...) {}
    }
}
std::uint32_t ProductDag::run(const Bindings& bindings, perception::FrameInfo frame,
                             rt::GraphReport& report, r28::Status& status) noexcept {
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const auto run_id=++impl_->sequence;
        std::set<std::string> active;
        std::string signature;
        if(bindings.empty()) {
            r28::Status failure;failure.code=2001;failure.detail="product frame has no active work";
            throw r28::Failure(failure);
        }
        for (const auto& binding : bindings) {
            const auto known=std::find_if(steps().begin(),steps().end(),[&](const auto& step){return step.first==binding.first;});
            if(known==steps().end()||!binding.second) {
                r28::Status failure;failure.code=2001;failure.detail="unknown or empty product binding: "+binding.first;
                throw r28::Failure(failure);
            }
            active.insert(binding.first); signature += binding.first + ";";
        }
        auto& graph = impl_->graphs[signature];
        if (!graph) {
            auto cfg = impl_->config;
            std::set<std::string> inactive;
            for (auto& task : cfg.tasks) {
                const auto found = std::find_if(steps().begin(), steps().end(), [&](const auto& item) {
                    return item.second == task.features.front();
                });
                if (found == steps().end()) throw std::invalid_argument("unknown product task implementation");
                task.enabled = task.enabled && active.count(found->first);
                if (!task.enabled) inactive.insert(task.id);
            }
            std::map<std::string,std::vector<std::string>> original_dependencies;
            for(const auto& task:cfg.tasks) original_dependencies[task.id]=task.depends_on;
            for(auto& task:cfg.tasks) {
                std::set<std::string> retained;
                std::function<void(const std::string&)> expand=[&](const std::string& id) {
                    if(!inactive.count(id)) {retained.insert(id);return;}
                    for(const auto& parent:original_dependencies[id]) expand(parent);
                };
                for(const auto& parent:task.depends_on) expand(parent);
                task.depends_on.assign(retained.begin(),retained.end());
            }
            graph = std::make_unique<rt::TaskGraph>(std::move(cfg), impl_->registry);
        }
        auto& session = *impl_->session;
        session.bindings = &bindings; session.frame = frame; session.first = {};
        struct ResetBinding {Session& session; ~ResetBinding(){session.bindings=nullptr;}} reset{session};
        rt::ArtifactStore artifacts;
        const auto wall_begin=std::chrono::steady_clock::now();
        const auto cpu_begin=r29::processCpuMs();
        report = graph->run(artifacts);
        status = session.first;
        status.frame=frame;
        if (!report.ok() && !status.code) {
            status.code = 1007; status.frame = frame; status.detail = "product DAG did not complete";
        }
        if(impl_->metrics.is_open()) {
            const double wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-wall_begin).count();
            const double cpu_ms=r29::processCpuMs()-cpu_begin;
            const auto resources=r29::processResources();
            const auto value=[&](const char* key){const auto found=resources.find(key);return found==resources.end()?-1L:found->second;};
            const auto rss=value("VmRSS:");const auto cma=value("CmaFree:");
            for(const auto& task:report.tasks) impl_->metrics<<run_id<<','<<frame.id<<','<<frame.timestamp_ns<<','<<wall_ms<<','<<cpu_ms<<','<<rss<<','<<cma<<','<<status.code<<','<<task.id<<','<<rt::taskStateName(task.state)<<','<<task.elapsed_ms<<'\n';
            const auto json=r28::statusJson(status);
            impl_->statuses<<"{\"run_id\":"<<run_id<<','<<json.substr(1)<<'\n';
            impl_->metrics.flush();impl_->statuses.flush();
            if(!impl_->metrics||!impl_->statuses) {
                status.code=2002;status.feature="product.metrics";status.detail="product metrics/status write failed";
            }
        }
        return status.code;
    } catch (const std::exception& e) {
        status = r28::statusFromException(e, 1007, "product.dag", "", frame);
        return status.code;
    } catch (...) {
        status.code = 1007; status.detail = "unknown product DAG failure"; status.frame = frame;
        return status.code;
    }
}
}
