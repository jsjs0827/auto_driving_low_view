#include "perception/task_graph.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <deque>
#include <exception>
#include <fstream>
#include <filesystem>
#include <future>
#include <limits>
#include <queue>
#include <set>
#include <shared_mutex>
#include <sstream>
#include <thread>

namespace perception::runtime {
namespace {
using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::string trim(const std::string &value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> split(const std::string &value) {
    std::vector<std::string> out;
    std::istringstream stream(value);
    std::string part;
    while (std::getline(stream, part, ',')) {
        part = trim(part);
        if (!part.empty())
            out.push_back(std::move(part));
    }
    return out;
}

bool parseBool(const std::string &value) {
    if (value == "true" || value == "1")
        return true;
    if (value == "false" || value == "0")
        return false;
    throw std::invalid_argument("boolean must be true/false");
}

void validateParameterValue(const std::string &name, const std::string &value,
                            FeatureParameterType type) {
    const auto invalid = [&]() {
        throw std::invalid_argument("invalid value for feature parameter: " + name);
    };
    try {
        std::size_t used = 0;
        switch (type) {
        case FeatureParameterType::String:
            return;
        case FeatureParameterType::Integer:
            (void)std::stoi(value, &used);
            if (used != value.size())
                invalid();
            return;
        case FeatureParameterType::Float: {
            const float parsed = std::stof(value, &used);
            if (used != value.size() || !std::isfinite(parsed))
                invalid();
            return;
        }
        case FeatureParameterType::Boolean:
            (void)parseBool(value);
            return;
        case FeatureParameterType::IntegerList:
            for (const auto &item : split(value)) {
                used = 0;
                (void)std::stoi(item, &used);
                if (used != item.size())
                    invalid();
            }
            return;
        case FeatureParameterType::FloatList:
            for (const auto &item : split(value)) {
                used = 0;
                const float parsed = std::stof(item, &used);
                if (used != item.size() || !std::isfinite(parsed))
                    invalid();
            }
            return;
        }
    } catch (const std::invalid_argument &) {
        invalid();
    } catch (const std::out_of_range &) {
        invalid();
    }
}

class ThreadPool {
  public:
    explicit ThreadPool(std::size_t count) {
        if (count == 0)
            throw std::invalid_argument("worker thread count must be positive");
        workers_.reserve(count);
        try {
            for (std::size_t i = 0; i < count; ++i)
                workers_.emplace_back([this] { worker(); });
        } catch (...) {
            // A vector containing joinable threads cannot be destroyed during
            // stack unwinding. Stop and join the workers that were created
            // before propagating the original construction failure.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stopping_ = true;
            }
            ready_.notify_all();
            for (auto &worker : workers_)
                if (worker.joinable())
                    worker.join();
            throw;
        }
    }
    ~ThreadPool() { close(); }

    void submit(std::function<void()> job) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_)
                throw std::runtime_error("task graph thread pool is closed");
            jobs_.push_back(std::move(job));
        }
        ready_.notify_one();
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_)
                return;
            stopping_ = true;
        }
        ready_.notify_all();
        for (auto &worker : workers_)
            if (worker.joinable())
                worker.join();
        workers_.clear();
    }

  private:
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::function<void()>> jobs_;
    std::vector<std::thread> workers_;
    bool stopping_ = false;

    void worker() {
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [&] { return stopping_ || !jobs_.empty(); });
                if (stopping_ && jobs_.empty())
                    return;
                job = std::move(jobs_.front());
                jobs_.pop_front();
            }
            try {
                job();
            } catch (...) {
                // Jobs contain their own exception boundary. Never terminate a worker.
            }
        }
    }
};
} // namespace

struct ArtifactStore::Impl {
    mutable std::shared_mutex mutex;
    std::unordered_map<std::string, Entry> entries;
};

ArtifactStore::ArtifactStore() : impl_(std::make_unique<Impl>()) {}
ArtifactStore::~ArtifactStore() = default;

void ArtifactStore::putErased(std::string key, std::type_index type,
                              std::shared_ptr<const void> value) {
    if (key.empty() || !value)
        throw std::invalid_argument("invalid artifact");
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    if (!impl_->entries.emplace(std::move(key), Entry{type, std::move(value)}).second)
        throw std::runtime_error("artifact already exists");
}

std::shared_ptr<const void> ArtifactStore::getErased(const std::string &key,
                                                     std::type_index type) const {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    const auto found = impl_->entries.find(key);
    if (found == impl_->entries.end())
        throw std::runtime_error("artifact not found: " + key);
    if (found->second.type != type)
        throw std::runtime_error("artifact type mismatch: " + key);
    return found->second.value;
}

bool ArtifactStore::contains(const std::string &key) const {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    return impl_->entries.count(key) != 0;
}

std::vector<std::string> ArtifactStore::keys() const {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    std::vector<std::string> result;
    result.reserve(impl_->entries.size());
    for (const auto &entry : impl_->entries)
        result.push_back(entry.first);
    std::sort(result.begin(), result.end());
    return result;
}

std::shared_ptr<const ArtifactStore> ArtifactStore::snapshot(
    const std::function<bool(const std::string &, std::type_index)> &keep) const {
    if (!keep)
        throw std::invalid_argument("artifact snapshot filter is empty");
    auto result = std::make_shared<ArtifactStore>();
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    result->impl_->entries.reserve(impl_->entries.size());
    for (const auto &entry : impl_->entries)
        if (keep(entry.first, entry.second.type))
            result->impl_->entries.emplace(entry.first, entry.second);
    return result;
}

void ArtifactStore::commit(std::unordered_map<std::string, Entry> &&writes) {
    std::unique_lock<std::shared_mutex> lock(impl_->mutex);
    for (const auto &write : writes)
        if (impl_->entries.count(write.first))
            throw std::runtime_error("artifact already exists: " + write.first);
    // Build the complete next state away from readers. If allocation or hashing
    // throws, the live store is untouched; the final swap is noexcept.
    auto merged = impl_->entries;
    for (auto &write : writes)
        merged.emplace(std::move(write.first), std::move(write.second));
    impl_->entries.swap(merged);
}

FeatureContext::FeatureContext(ArtifactStore &store, std::string task_id,
                               const Parameters &parameters)
    : store_(store), task_id_(std::move(task_id)), parameters_(&parameters) {}

std::string FeatureContext::parameter(const std::string &key,
                                      const std::string &fallback) const {
    const auto found = parameters_->find(key);
    return found == parameters_->end() ? fallback : found->second;
}

int FeatureContext::parameterInt(const std::string &key, int fallback) const {
    const auto found = parameters_->find(key);
    if (found == parameters_->end())
        return fallback;
    std::size_t used = 0;
    const int result = std::stoi(found->second, &used);
    if (used != found->second.size())
        throw std::invalid_argument("invalid integer parameter: " + key);
    return result;
}

float FeatureContext::parameterFloat(const std::string &key, float fallback) const {
    const auto found = parameters_->find(key);
    if (found == parameters_->end())
        return fallback;
    std::size_t used = 0;
    const float result = std::stof(found->second, &used);
    if (used != found->second.size())
        throw std::invalid_argument("invalid float parameter: " + key);
    return result;
}

bool FeatureContext::parameterBool(const std::string &key, bool fallback) const {
    const auto found = parameters_->find(key);
    return found == parameters_->end() ? fallback : parseBool(found->second);
}

void FeatureContext::commit() { store_.commit(std::move(writes_)); }

void FeatureRegistry::add(std::string name, FeatureFactory factory) {
    if (name.empty() || !factory)
        throw std::invalid_argument("invalid feature registration");
    Registration registration;
    registration.factory = std::move(factory);
    registration.strict_parameters = true;
    if (!factories_.emplace(std::move(name), std::move(registration)).second)
        throw std::invalid_argument("feature is already registered");
}

void FeatureRegistry::add(std::string name, FeatureFactory factory,
                          FeatureParameterSchema schema,
                          ExclusiveResourceResolver resources) {
    if (name.empty() || !factory)
        throw std::invalid_argument("invalid feature registration");
    for (const auto &entry : schema)
        if (entry.first.empty())
            throw std::invalid_argument("feature parameter schema contains an empty name");
    Registration registration;
    registration.factory = std::move(factory);
    registration.parameter_schema = std::move(schema);
    registration.exclusive_resources = std::move(resources);
    registration.strict_parameters = true;
    if (!factories_.emplace(std::move(name), std::move(registration)).second)
        throw std::invalid_argument("feature is already registered");
}

void FeatureRegistry::addCloser(std::function<void()> hook) {
    if (!hook)
        throw std::invalid_argument("invalid feature registry close hook");
    std::lock_guard<std::mutex> lock(close_state_->mutex);
    if (close_state_->closed)
        throw std::runtime_error("feature registry is closed");
    close_state_->hooks.push_back(std::move(hook));
}

void FeatureRegistry::close() {
    std::vector<std::function<void()>> hooks;
    {
        std::lock_guard<std::mutex> lock(close_state_->mutex);
        if (close_state_->closed)
            return;
        close_state_->closed = true;
        hooks.swap(close_state_->hooks);
    }
    std::exception_ptr first;
    for (auto it = hooks.rbegin(); it != hooks.rend(); ++it) {
        try {
            (*it)();
        } catch (...) {
            if (!first)
                first = std::current_exception();
        }
    }
    if (first)
        std::rethrow_exception(first);
}

bool FeatureRegistry::contains(const std::string &name) const {
    return factories_.count(name) != 0;
}

std::unique_ptr<Feature> FeatureRegistry::create(const std::string &name,
                                                 const Parameters &parameters) const {
    const auto found = factories_.find(name);
    if (found == factories_.end())
        throw std::invalid_argument("unknown feature: " + name);
    auto feature = found->second.factory(parameters);
    if (!feature)
        throw std::runtime_error("feature factory returned null: " + name);
    return feature;
}

const FeatureParameterSchema *
FeatureRegistry::parameterSchema(const std::string &name) const {
    const auto found = factories_.find(name);
    if (found == factories_.end())
        throw std::invalid_argument("unknown feature: " + name);
    return found->second.strict_parameters ? &found->second.parameter_schema : nullptr;
}

std::vector<std::string>
FeatureRegistry::exclusiveResources(const std::string &name,
                                    const Parameters &parameters) const {
    const auto found = factories_.find(name);
    if (found == factories_.end())
        throw std::invalid_argument("unknown feature: " + name);
    if (!found->second.exclusive_resources)
        return {};
    auto resources = found->second.exclusive_resources(parameters);
    std::set<std::string> unique;
    for (const auto &resource : resources)
        if (resource.empty() || !unique.insert(resource).second)
            throw std::invalid_argument("feature returned an invalid exclusive resource: " +
                                        name);
    return resources;
}

std::vector<std::string> FeatureRegistry::names() const {
    std::vector<std::string> result;
    result.reserve(factories_.size());
    for (const auto &entry : factories_)
        result.push_back(entry.first);
    return result;
}

void TaskGraphConfig::validate(const FeatureRegistry &registry) const {
    if (worker_threads == 0 || worker_threads > 256)
        throw std::invalid_argument("worker_threads must be 1..256");
    if (tasks.empty())
        throw std::invalid_argument("task graph is empty");
    std::set<std::string> unique_results;
    for (const auto &key : result_artifacts) {
        if (key.empty() || key.rfind("input.", 0) == 0)
            throw std::invalid_argument("invalid result artifact key: " + key);
        if (!unique_results.insert(key).second)
            throw std::invalid_argument("duplicate result artifact key: " + key);
    }
    std::map<std::string, std::size_t> indices;
    std::vector<std::set<std::string>> exclusive_resources(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        const auto &task = tasks[i];
        if (task.id.empty())
            throw std::invalid_argument("task id is empty");
        if (!indices.emplace(task.id, i).second)
            throw std::invalid_argument("duplicate task id: " + task.id);
        if (task.enabled && task.features.empty())
            throw std::invalid_argument("enabled task has no feature: " + task.id);
        if (!task.feature_parameters.empty() &&
            task.feature_parameters.size() != task.features.size())
            throw std::invalid_argument("feature parameter count mismatch: " + task.id);
        for (const auto &feature : task.features)
            if (!registry.contains(feature))
                throw std::invalid_argument("unknown feature in task " + task.id + ": " +
                                            feature);
        std::vector<const FeatureParameterSchema *> schemas;
        schemas.reserve(task.features.size());
        bool all_features_are_strict = true;
        for (const auto &feature : task.features) {
            const auto *schema = registry.parameterSchema(feature);
            schemas.push_back(schema);
            all_features_are_strict = all_features_are_strict && schema != nullptr;
        }
        if (all_features_are_strict) {
            for (const auto &entry : task.parameters) {
                bool accepted = false;
                for (const auto *schema : schemas) {
                    const auto rule = schema->find(entry.first);
                    if (rule == schema->end())
                        continue;
                    accepted = true;
                    validateParameterValue(entry.first, entry.second, rule->second);
                }
                if (!accepted)
                    throw std::invalid_argument("unknown parameter " + entry.first +
                                                " in task " + task.id);
            }
        }
        if (!task.feature_parameters.empty()) {
            for (std::size_t feature_index = 0; feature_index < task.features.size();
                 ++feature_index) {
                const auto *schema = schemas[feature_index];
                if (!schema)
                    continue;
                for (const auto &entry : task.feature_parameters[feature_index]) {
                    const auto rule = schema->find(entry.first);
                    if (rule == schema->end())
                        throw std::invalid_argument(
                            "unknown parameter " + entry.first + " for feature " +
                            task.features[feature_index] + " in task " + task.id);
                    validateParameterValue(entry.first, entry.second, rule->second);
                }
            }
        }
        if (task.enabled) {
            for (std::size_t feature_index = 0; feature_index < task.features.size();
                 ++feature_index) {
                auto effective = task.parameters;
                if (!task.feature_parameters.empty())
                    for (const auto &override_value :
                         task.feature_parameters[feature_index])
                        effective[override_value.first] = override_value.second;
                for (auto &resource : registry.exclusiveResources(
                         task.features[feature_index], effective))
                    exclusive_resources[i].insert(std::move(resource));
            }
        }
    }
    std::vector<int> indegree(tasks.size(), 0);
    std::vector<std::vector<std::size_t>> successors(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        std::set<std::string> unique;
        for (const auto &dependency : tasks[i].depends_on) {
            const auto found = indices.find(dependency);
            if (found == indices.end())
                throw std::invalid_argument("unknown dependency " + dependency + " for " +
                                            tasks[i].id);
            if (!unique.insert(dependency).second)
                throw std::invalid_argument("duplicate dependency " + dependency + " for " +
                                            tasks[i].id);
            if (dependency == tasks[i].id)
                throw std::invalid_argument("task depends on itself: " + tasks[i].id);
            if (tasks[i].enabled && !tasks[found->second].enabled)
                throw std::invalid_argument("enabled task depends on disabled task: " +
                                            tasks[i].id);
            ++indegree[i];
            successors[found->second].push_back(i);
        }
    }
    std::queue<std::size_t> ready;
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (indegree[i] == 0)
            ready.push(i);
    }
    std::size_t visited = 0;
    while (!ready.empty()) {
        const auto current = ready.front();
        ready.pop();
        ++visited;
        for (const auto next : successors[current])
            if (--indegree[next] == 0)
                ready.push(next);
    }
    if (visited != tasks.size())
        throw std::invalid_argument("task dependency graph contains a cycle");
    std::vector<std::vector<bool>> reachable(
        tasks.size(), std::vector<bool>(tasks.size(), false));
    for (std::size_t source = 0; source < tasks.size(); ++source) {
        if (!tasks[source].enabled)
            continue;
        std::vector<std::size_t> pending{source};
        while (!pending.empty()) {
            const auto current = pending.back();
            pending.pop_back();
            for (const auto next : successors[current]) {
                if (!tasks[next].enabled)
                    continue;
                if (reachable[source][next])
                    continue;
                reachable[source][next] = true;
                pending.push_back(next);
            }
        }
    }
    for (std::size_t left = 0; left < tasks.size(); ++left) {
        if (!tasks[left].enabled)
            continue;
        for (std::size_t right = left + 1; right < tasks.size(); ++right) {
            if (!tasks[right].enabled || reachable[left][right] ||
                reachable[right][left])
                continue;
            for (const auto &resource : exclusive_resources[left])
                if (exclusive_resources[right].count(resource))
                    throw std::invalid_argument(
                        "unordered tasks " + tasks[left].id + " and " +
                        tasks[right].id + " share exclusive resource " + resource +
                        "; add a dependency or configure distinct resource instances");
        }
    }
}

TaskGraphConfig loadTaskGraphConfig(const std::string &path) {
    std::ifstream stream(std::filesystem::u8path(path));
    if (!stream)
        throw std::runtime_error("cannot open task graph config: " + path);
    std::map<std::string, std::string> values;
    std::string line;
    int line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line_number == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0)
            line.erase(0, 3);
        line = trim(line);
        if (line.empty() || line.front() == '#')
            continue;
        const auto equal = line.find('=');
        if (equal == std::string::npos)
            throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                     ": expected key=value");
        const auto key = trim(line.substr(0, equal));
        if (key.empty() || !values.emplace(key, trim(line.substr(equal + 1))).second)
            throw std::runtime_error(path + ":" + std::to_string(line_number) +
                                     ": empty or duplicate key");
    }
    TaskGraphConfig config;
    const auto workers = values.find("worker_threads");
    if (workers != values.end()) {
        std::size_t used = 0;
        const auto parsed = std::stoull(workers->second, &used);
        if (used != workers->second.size() ||
            parsed > std::numeric_limits<std::size_t>::max())
            throw std::runtime_error("invalid worker_threads value");
        config.worker_threads = static_cast<std::size_t>(parsed);
        values.erase(workers);
    }
    const auto result_artifacts = values.find("result_artifacts");
    if (result_artifacts != values.end()) {
        config.result_artifacts = split(result_artifacts->second);
        values.erase(result_artifacts);
    }
    const auto task_list = values.find("tasks");
    if (task_list == values.end())
        throw std::runtime_error("task graph config requires tasks=");
    const auto ids = split(task_list->second);
    values.erase(task_list);
    for (const auto &id : ids) {
        TaskSpec task;
        task.id = id;
        const auto prefix = "task." + id + ".";
        const auto take = [&](const std::string &suffix, std::string fallback) {
            const auto key = prefix + suffix;
            const auto found = values.find(key);
            if (found == values.end())
                return fallback;
            auto result = found->second;
            values.erase(found);
            return result;
        };
        const auto feature_tokens = split(take("features", {}));
        std::vector<std::string> feature_aliases;
        std::set<std::string> unique_aliases;
        for (const auto &token : feature_tokens) {
            const auto colon = token.find(':');
            if (colon == std::string::npos) {
                task.features.push_back(token);
                feature_aliases.emplace_back();
                continue;
            }
            const auto alias = trim(token.substr(0, colon));
            const auto feature = trim(token.substr(colon + 1));
            if (alias.empty() || feature.empty() || feature.find(':') != std::string::npos)
                throw std::runtime_error("invalid aliased feature in task " + id + ": " +
                                         token);
            if (!unique_aliases.insert(alias).second)
                throw std::runtime_error("duplicate feature alias in task " + id + ": " +
                                         alias);
            task.features.push_back(feature);
            feature_aliases.push_back(alias);
        }
        task.depends_on = split(take("depends_on", {}));
        task.enabled = parseBool(take("enabled", "true"));
        for (auto it = values.begin(); it != values.end();) {
            const auto parameter_prefix = prefix + "param.";
            if (it->first.compare(0, parameter_prefix.size(), parameter_prefix) == 0) {
                const auto name = it->first.substr(parameter_prefix.size());
                if (name.empty())
                    throw std::runtime_error("empty task parameter for " + id);
                task.parameters.emplace(name, it->second);
                it = values.erase(it);
            } else {
                ++it;
            }
        }
        task.feature_parameters.resize(task.features.size());
        for (std::size_t feature_index = 0; feature_index < feature_aliases.size();
             ++feature_index) {
            const auto &alias = feature_aliases[feature_index];
            if (alias.empty())
                continue;
            const auto parameter_prefix =
                prefix + "feature." + alias + ".param.";
            for (auto it = values.begin(); it != values.end();) {
                if (it->first.compare(0, parameter_prefix.size(), parameter_prefix) == 0) {
                    const auto name = it->first.substr(parameter_prefix.size());
                    if (name.empty())
                        throw std::runtime_error("empty feature parameter for " + id + ":" +
                                                 alias);
                    task.feature_parameters[feature_index].emplace(name, it->second);
                    it = values.erase(it);
                } else {
                    ++it;
                }
            }
        }
        config.tasks.push_back(std::move(task));
    }
    if (!values.empty())
        throw std::runtime_error("unknown task graph field: " + values.begin()->first);
    return config;
}

const char *taskStateName(TaskState state) noexcept {
    switch (state) {
    case TaskState::Pending:
        return "pending";
    case TaskState::Running:
        return "running";
    case TaskState::Succeeded:
        return "succeeded";
    case TaskState::Failed:
        return "failed";
    case TaskState::Skipped:
        return "skipped";
    }
    return "failed";
}

bool GraphReport::ok() const noexcept {
    return std::all_of(tasks.begin(), tasks.end(), [](const TaskReport &task) {
        return task.state == TaskState::Succeeded ||
               (task.state == TaskState::Skipped && task.error == "disabled");
    });
}

struct TaskGraph::Impl {
    struct Node {
        TaskSpec spec;
        std::vector<std::unique_ptr<Feature>> features;
        std::vector<Parameters> effective_parameters;
        std::vector<std::size_t> predecessors;
        std::vector<std::size_t> successors;
        Node() = default;
        Node(Node &&) noexcept = default;
        Node &operator=(Node &&) noexcept = default;
        Node(const Node &) = delete;
        Node &operator=(const Node &) = delete;
    };
    struct RunState {
        ArtifactStore *store = nullptr;
        std::vector<TaskReport> reports;
        std::vector<int> remaining;
        std::size_t completed = 0;
        std::mutex mutex;
        std::condition_variable finished;
    };

    std::vector<Node> nodes;
    ThreadPool pool;
    std::mutex run_mutex;
    bool closed = false;

    static std::size_t validatedWorkerCount(const TaskGraphConfig &config,
                                            const FeatureRegistry &registry) {
        config.validate(registry);
        // A task executes on at most one worker. Disabled nodes never submit
        // work, so retaining a worker for each requested slot would only retain
        // idle threads in single-feature/mode-specific graphs. Keep the
        // dedicated persistent-worker contract, including all-disabled graphs.
        const auto enabled = static_cast<std::size_t>(std::count_if(
            config.tasks.begin(), config.tasks.end(),
            [](const TaskSpec &task) { return task.enabled; }));
        return std::max(std::size_t{1}, std::min(config.worker_threads, enabled));
    }

    Impl(TaskGraphConfig config, const FeatureRegistry &registry)
        : pool(validatedWorkerCount(config, registry)) {
        std::map<std::string, std::size_t> indices;
        nodes.reserve(config.tasks.size());
        for (std::size_t i = 0; i < config.tasks.size(); ++i) {
            indices.emplace(config.tasks[i].id, i);
            Node node;
            node.spec = config.tasks[i];
            if (node.spec.enabled) {
                node.features.reserve(node.spec.features.size());
                node.effective_parameters.reserve(node.spec.features.size());
                for (std::size_t feature_index = 0;
                     feature_index < node.spec.features.size(); ++feature_index) {
                    auto effective = node.spec.parameters;
                    if (!node.spec.feature_parameters.empty())
                        for (const auto &override_value :
                             node.spec.feature_parameters[feature_index])
                            effective[override_value.first] = override_value.second;
                    node.effective_parameters.push_back(std::move(effective));
                    node.features.push_back(registry.create(
                        node.spec.features[feature_index],
                        node.effective_parameters.back()));
                }
            }
            nodes.push_back(std::move(node));
        }
        for (std::size_t i = 0; i < nodes.size(); ++i)
            for (const auto &dependency : nodes[i].spec.depends_on) {
                const auto predecessor = indices.at(dependency);
                nodes[i].predecessors.push_back(predecessor);
                nodes[predecessor].successors.push_back(i);
            }
    }

    void execute(const std::shared_ptr<RunState> &state, std::size_t index) {
        const auto start = Clock::now();
        try {
            FeatureContext context(*state->store, nodes[index].spec.id,
                                   nodes[index].spec.parameters);
            for (std::size_t feature_index = 0;
                 feature_index < nodes[index].features.size(); ++feature_index) {
                context.selectParameters(nodes[index].effective_parameters[feature_index]);
                nodes[index].features[feature_index]->execute(context);
            }
            context.commit();
            std::lock_guard<std::mutex> lock(state->mutex);
            state->reports[index].state = TaskState::Succeeded;
        } catch (const std::exception &error) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->reports[index].state = TaskState::Failed;
            try {
                state->reports[index].error = error.what();
            } catch (...) {
                state->reports[index].error.clear();
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->reports[index].state = TaskState::Failed;
            try {
                state->reports[index].error = "unknown feature exception";
            } catch (...) {
                state->reports[index].error.clear();
            }
        }
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->reports[index].elapsed_ms = elapsedMs(start);
            ++state->completed;
            for (const auto successor : nodes[index].successors)
                if (nodes[successor].spec.enabled)
                    --state->remaining[successor];
            state->finished.notify_all();
        }
    }
};

TaskGraph::TaskGraph(TaskGraphConfig config, const FeatureRegistry &registry)
    : impl_(std::make_unique<Impl>(std::move(config), registry)) {}
TaskGraph::~TaskGraph() { close(); }

GraphReport TaskGraph::run(ArtifactStore &store) {
    std::lock_guard<std::mutex> serial(impl_->run_mutex);
    if (impl_->closed)
        throw std::runtime_error("task graph is closed");
    const auto start = Clock::now();
    auto state = std::make_shared<Impl::RunState>();
    state->store = &store;
    state->reports.resize(impl_->nodes.size());
    state->remaining.resize(impl_->nodes.size());
    for (std::size_t i = 0; i < impl_->nodes.size(); ++i) {
        state->reports[i].id = impl_->nodes[i].spec.id;
        if (!impl_->nodes[i].spec.enabled) {
            state->reports[i].state = TaskState::Skipped;
            state->reports[i].error = "disabled";
            ++state->completed;
        } else {
            state->remaining[i] = static_cast<int>(impl_->nodes[i].predecessors.size());
        }
    }
    std::vector<std::size_t> ready;
    ready.reserve(impl_->nodes.size());
    for (;;) {
        ready.clear();
        std::size_t observed_completed = 0;
        bool done = false;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            // Propagate dependency skips in the coordinator. Workers execute
            // features only; they never recursively allocate/schedule follow-ups.
            bool propagated = false;
            do {
                propagated = false;
                for (std::size_t i = 0; i < impl_->nodes.size(); ++i) {
                    auto &report = state->reports[i];
                    if (!impl_->nodes[i].spec.enabled ||
                        report.state != TaskState::Pending || state->remaining[i] != 0)
                        continue;
                    const bool blocked = std::any_of(
                        impl_->nodes[i].predecessors.begin(),
                        impl_->nodes[i].predecessors.end(), [&](std::size_t predecessor) {
                            return state->reports[predecessor].state != TaskState::Succeeded;
                        });
                    if (!blocked) {
                        report.state = TaskState::Running; // queued or executing
                        ready.push_back(i);
                        continue;
                    }
                    report.state = TaskState::Skipped;
                    try {
                        report.error = "dependency did not succeed";
                    } catch (...) {
                        report.error.clear();
                    }
                    ++state->completed;
                    for (const auto successor : impl_->nodes[i].successors)
                        if (impl_->nodes[successor].spec.enabled)
                            --state->remaining[successor];
                    propagated = true;
                }
            } while (propagated);
            observed_completed = state->completed;
            done = state->completed == impl_->nodes.size();
        }
        if (done)
            break;
        for (const auto index : ready) {
            try {
                impl_->pool.submit([impl = impl_.get(), state, index] {
                    impl->execute(state, index);
                });
            } catch (const std::exception &error) {
                std::lock_guard<std::mutex> lock(state->mutex);
                auto &report = state->reports[index];
                report.state = TaskState::Failed;
                try {
                    report.error = "task scheduling failed: ";
                    report.error += error.what();
                } catch (...) {
                    report.error.clear();
                }
                ++state->completed;
                for (const auto successor : impl_->nodes[index].successors)
                    if (impl_->nodes[successor].spec.enabled)
                        --state->remaining[successor];
                state->finished.notify_all();
            } catch (...) {
                std::lock_guard<std::mutex> lock(state->mutex);
                auto &report = state->reports[index];
                report.state = TaskState::Failed;
                try {
                    report.error = "unknown task scheduling failure";
                } catch (...) {
                    report.error.clear();
                }
                ++state->completed;
                for (const auto successor : impl_->nodes[index].successors)
                    if (impl_->nodes[successor].spec.enabled)
                        --state->remaining[successor];
                state->finished.notify_all();
            }
        }
        std::unique_lock<std::mutex> lock(state->mutex);
        if (state->completed != impl_->nodes.size() &&
            state->completed == observed_completed)
            state->finished.wait(lock, [&] {
                return state->completed != observed_completed;
            });
    }
    GraphReport report;
    report.tasks = std::move(state->reports);
    report.elapsed_ms = elapsedMs(start);
    return report;
}

void TaskGraph::close() {
    if (!impl_)
        return;
    std::lock_guard<std::mutex> serial(impl_->run_mutex);
    if (impl_->closed)
        return;
    impl_->closed = true;
    impl_->pool.close();
}

} // namespace perception::runtime
