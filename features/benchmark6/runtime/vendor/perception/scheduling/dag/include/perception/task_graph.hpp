#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace perception::runtime {

using Parameters = std::map<std::string, std::string>;

enum class FeatureParameterType {
    String,
    Integer,
    Float,
    Boolean,
    IntegerList,
    FloatList
};
using FeatureParameterSchema = std::map<std::string, FeatureParameterType>;
// Exclusive resources are scheduler-visible identities such as a reusable
// model workspace. Enabled tasks sharing one must be ordered by the DAG.
using ExclusiveResourceResolver =
    std::function<std::vector<std::string>(const Parameters &)>;

class ArtifactStore {
  public:
    ArtifactStore();
    ~ArtifactStore();
    ArtifactStore(const ArtifactStore &) = delete;
    ArtifactStore &operator=(const ArtifactStore &) = delete;

    template <class T> void put(std::string key, T value) {
        putErased(std::move(key), std::type_index(typeid(T)),
                  std::static_pointer_cast<const void>(std::make_shared<const T>(std::move(value))));
    }

    template <class T> void putShared(std::string key, std::shared_ptr<const T> value) {
        putErased(std::move(key), std::type_index(typeid(T)),
                  std::static_pointer_cast<const void>(std::move(value)));
    }

    template <class T> std::shared_ptr<const T> get(const std::string &key) const {
        auto value = getErased(key, std::type_index(typeid(T)));
        return std::static_pointer_cast<const T>(std::move(value));
    }

    bool contains(const std::string &key) const;
    std::vector<std::string> keys() const;
    // Creates a read-only snapshot by sharing immutable artifact payloads. The
    // filter is evaluated while this store is locked; it must not call back
    // into the same store. This is used to export extension results without
    // retaining borrowed frame inputs or internal inference invocations.
    std::shared_ptr<const ArtifactStore> snapshot(
        const std::function<bool(const std::string &, std::type_index)> &keep) const;

  private:
    struct Entry {
        std::type_index type{typeid(void)};
        std::shared_ptr<const void> value;
    };
    struct Impl;
    std::unique_ptr<Impl> impl_;

    void putErased(std::string, std::type_index, std::shared_ptr<const void>);
    std::shared_ptr<const void> getErased(const std::string &, std::type_index) const;
    void commit(std::unordered_map<std::string, Entry> &&);
    friend class FeatureContext;
};

// A feature writes into a task-local transaction. Its outputs become visible to
// dependent tasks only after every feature in the task succeeds.
class FeatureContext {
  public:
    FeatureContext(ArtifactStore &, std::string task_id, const Parameters &);
    const std::string &taskId() const noexcept { return task_id_; }
    const Parameters &parameters() const noexcept { return *parameters_; }
    bool contains(const std::string &key) const {
        return writes_.find(key) != writes_.end() || store_.contains(key);
    }

    std::string parameter(const std::string &key, const std::string &fallback = {}) const;
    int parameterInt(const std::string &key, int fallback) const;
    float parameterFloat(const std::string &key, float fallback) const;
    bool parameterBool(const std::string &key, bool fallback) const;

    template <class T> std::shared_ptr<const T> get(const std::string &key) const {
        const auto local = writes_.find(key);
        if (local != writes_.end()) {
            if (local->second.type != std::type_index(typeid(T)))
                throw std::runtime_error("artifact type mismatch: " + key);
            return std::static_pointer_cast<const T>(local->second.value);
        }
        return store_.get<T>(key);
    }

    template <class T> void set(std::string key, T value) {
        if (key.empty())
            throw std::invalid_argument("artifact key is empty");
        ArtifactStore::Entry entry{std::type_index(typeid(T)),
                                   std::static_pointer_cast<const void>(
                                       std::make_shared<const T>(std::move(value)))};
        if (!writes_.emplace(std::move(key), std::move(entry)).second)
            throw std::runtime_error("feature wrote an artifact twice");
    }

  private:
    ArtifactStore &store_;
    std::string task_id_;
    const Parameters *parameters_ = nullptr;
    std::unordered_map<std::string, ArtifactStore::Entry> writes_;
    void selectParameters(const Parameters &parameters) noexcept { parameters_ = &parameters; }
    void commit();
    friend class TaskGraph;
};

class Feature {
  public:
    virtual ~Feature() = default;
    virtual void execute(FeatureContext &) = 0;
};

using FeatureFactory = std::function<std::unique_ptr<Feature>(const Parameters &)>;

class FeatureRegistry {
  public:
    FeatureRegistry() = default;
    FeatureRegistry(const FeatureRegistry &) = delete;
    FeatureRegistry &operator=(const FeatureRegistry &) = delete;
    FeatureRegistry(FeatureRegistry &&) noexcept = default;
    FeatureRegistry &operator=(FeatureRegistry &&) noexcept = default;

    // The two-argument overload declares an empty schema (no parameters).
    // Parameterized features must use the schema overload so misspellings fail
    // during graph construction, including Tasks that mix built-ins/extensions.
    void add(std::string name, FeatureFactory);
    void add(std::string name, FeatureFactory, FeatureParameterSchema,
             ExclusiveResourceResolver = {});
    // Resource-owning feature groups can register shutdown hooks. The registry
    // is move-only, and the Engine closes transferred resources exactly once.
    void addCloser(std::function<void()>);
    void close();
    bool contains(const std::string &) const;
    std::unique_ptr<Feature> create(const std::string &, const Parameters &) const;
    std::vector<std::string> names() const;
    const FeatureParameterSchema *parameterSchema(const std::string &) const;
    std::vector<std::string> exclusiveResources(const std::string &,
                                                const Parameters &) const;

  private:
    struct CloseState {
        std::mutex mutex;
        bool closed = false;
        std::vector<std::function<void()>> hooks;
    };
    struct Registration {
        FeatureFactory factory;
        FeatureParameterSchema parameter_schema;
        ExclusiveResourceResolver exclusive_resources;
        bool strict_parameters = true;
    };
    std::map<std::string, Registration> factories_;
    std::shared_ptr<CloseState> close_state_ = std::make_shared<CloseState>();
};

struct TaskSpec {
    std::string id;
    std::vector<std::string> features;
    std::vector<std::string> depends_on;
    Parameters parameters;
    bool enabled = true;
    // Optional per-feature overrides, indexed like features. Empty entries use
    // only the task-level parameters. The config syntax fills these from
    // `alias:feature` plus `task.<id>.feature.<alias>.param.<key>` fields.
    std::vector<Parameters> feature_parameters;
};

struct TaskGraphConfig {
    std::size_t worker_threads = 1;
    std::vector<TaskSpec> tasks;
    // Empty exports every committed non-input artifact. Production profiles
    // should list final result keys so per-frame results do not retain large
    // transient DAG artifacts after scheduling has completed.
    std::vector<std::string> result_artifacts;
    void validate(const FeatureRegistry &) const;
};

TaskGraphConfig loadTaskGraphConfig(const std::string &path);

enum class TaskState { Pending, Running, Succeeded, Failed, Skipped };
const char *taskStateName(TaskState) noexcept;

struct TaskReport {
    std::string id;
    TaskState state = TaskState::Pending;
    std::string error;
    double elapsed_ms = 0;
};

struct GraphReport {
    std::vector<TaskReport> tasks; // Stable configuration order.
    double elapsed_ms = 0;
    bool ok() const noexcept;
};

class TaskGraph {
  public:
    TaskGraph(TaskGraphConfig, const FeatureRegistry &);
    ~TaskGraph();
    TaskGraph(const TaskGraph &) = delete;
    TaskGraph &operator=(const TaskGraph &) = delete;

    GraphReport run(ArtifactStore &);
    void close();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace perception::runtime
