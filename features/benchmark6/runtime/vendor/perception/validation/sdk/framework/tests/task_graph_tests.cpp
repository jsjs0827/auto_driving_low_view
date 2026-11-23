#include "perception/task_graph.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace perception::runtime;
namespace {
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
template <class F> void rejects(F action, const char *message) {
    try {
        action();
    } catch (const std::exception &) {
        return;
    }
    throw std::runtime_error(message);
}
class LambdaFeature final : public Feature {
  public:
    explicit LambdaFeature(std::function<void(FeatureContext &)> fn) : fn_(std::move(fn)) {}
    void execute(FeatureContext &context) override { fn_(context); }
  private:
    std::function<void(FeatureContext &)> fn_;
};
FeatureFactory factory(std::function<void(FeatureContext &)> fn) {
    return [fn = std::move(fn)](const Parameters &) {
        return std::make_unique<LambdaFeature>(fn);
    };
}

std::atomic<unsigned> next_thread_token{0};
unsigned currentThreadToken() {
    thread_local const unsigned token =
        next_thread_token.fetch_add(1, std::memory_order_relaxed) + 1;
    return token;
}

void registryAndValidation() {
    FeatureRegistry registry;
    registry.add("ok", factory([](FeatureContext &) {}));
    rejects([&] { registry.add("ok", factory([](FeatureContext &) {})); },
            "duplicate registration accepted");
    TaskGraphConfig unknown{1, {{"a", {"missing"}, {}, {}, true}}};
    rejects([&] { unknown.validate(registry); }, "unknown feature accepted");
    TaskGraphConfig cycle{2,
                          {{"a", {"ok"}, {"b"}, {}, true},
                           {"b", {"ok"}, {"a"}, {}, true}}};
    rejects([&] { cycle.validate(registry); }, "cycle accepted");
    TaskGraphConfig disabled_cycle{
        2,
        {{"a", {"ok"}, {"b"}, {}, false},
         {"b", {"ok"}, {"a"}, {}, false}}};
    rejects([&] { disabled_cycle.validate(registry); },
            "cycle consisting only of disabled tasks was accepted");
    TaskGraphConfig missing{1, {{"a", {"ok"}, {"nope"}, {}, true}}};
    rejects([&] { missing.validate(registry); }, "missing dependency accepted");
    TaskGraphConfig excessive{100000, {{"a", {"ok"}, {}, {}, true}}};
    rejects([&] { TaskGraph graph(excessive, registry); },
            "thread pool was created before worker count validation");

    registry.add("strict", factory([](FeatureContext &) {}),
                 FeatureParameterSchema{{"count", FeatureParameterType::Integer},
                                        {"enabled", FeatureParameterType::Boolean}});
    TaskGraphConfig unknown_parameter{
        1, {{"strict", {"strict"}, {}, {{"coutn", "3"}}, true}}};
    rejects([&] { unknown_parameter.validate(registry); },
            "strict feature accepted a misspelled parameter");
    registry.add("extension-without-parameters",
                 factory([](FeatureContext &) {}));
    TaskGraphConfig mixed_unknown_parameter{
        1, {{"mixed", {"strict", "extension-without-parameters"}, {},
             {{"coutn", "3"}}, true}}};
    rejects([&] { mixed_unknown_parameter.validate(registry); },
            "mixed built-in/extension task bypassed strict parameter validation");
    TaskGraphConfig invalid_parameter{
        1, {{"strict", {"strict"}, {}, {{"count", "3junk"}}, true}}};
    rejects([&] { invalid_parameter.validate(registry); },
            "strict feature accepted an invalid typed parameter");
}

void exclusiveResourceDependencies() {
    FeatureRegistry registry;
    registry.add(
        "exclusive", factory([](FeatureContext &) {}),
        FeatureParameterSchema{{"resource", FeatureParameterType::String}},
        [](const Parameters &parameters) {
            const auto found = parameters.find("resource");
            if (found == parameters.end() || found->second.empty())
                throw std::invalid_argument("exclusive test resource is missing");
            return std::vector<std::string>{found->second};
        });
    TaskGraphConfig parallel{
        2,
        {{"left", {"exclusive"}, {}, {{"resource", "model-a"}}, true},
         {"right", {"exclusive"}, {}, {{"resource", "model-a"}}, true}}};
    rejects([&] { parallel.validate(registry); },
            "unordered tasks sharing an exclusive resource were accepted");

    auto ordered = parallel;
    ordered.tasks[1].depends_on = {"left"};
    TaskGraph ordered_graph(std::move(ordered), registry);
    ArtifactStore ordered_store;
    check(ordered_graph.run(ordered_store).ok(),
          "ordered exclusive-resource tasks failed");

    auto independent = parallel;
    independent.tasks[1].parameters["resource"] = "model-b";
    TaskGraph independent_graph(std::move(independent), registry);
    ArtifactStore independent_store;
    check(independent_graph.run(independent_store).ok(),
          "distinct exclusive-resource tasks were rejected");
}

void localChainAndDependency() {
    FeatureRegistry registry;
    registry.add("produce", factory([](FeatureContext &context) { context.set<int>("x", 7); }));
    registry.add("within", factory([](FeatureContext &context) {
                     check(*context.get<int>("x") == 7, "local transaction invisible");
                     context.set<int>("y", 9);
                 }));
    registry.add("consume", factory([](FeatureContext &context) {
                     check(*context.get<int>("y") == 9, "dependency output invisible");
                     context.set<int>("z", 11);
                 }));
    TaskGraphConfig config{2,
                           {{"first", {"produce", "within"}, {}, {}, true},
                            {"second", {"consume"}, {"first"}, {}, true}}};
    TaskGraph graph(config, registry);
    ArtifactStore store;
    const auto report = graph.run(store);
    check(report.ok() && *store.get<int>("z") == 11, "serial feature/task chain failed");
}

void parallelAndJoin() {
    FeatureRegistry registry;
    std::atomic<int> active{0}, maximum{0}, completed{0};
    auto branch = [&](FeatureContext &) {
        const int now = ++active;
        int seen = maximum.load();
        while (seen < now && !maximum.compare_exchange_weak(seen, now)) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        --active;
        ++completed;
    };
    registry.add("branch", factory(branch));
    registry.add("join", factory([&](FeatureContext &) {
                     check(completed.load() == 2, "join ran before both predecessors committed");
                 }));
    TaskGraphConfig config{2,
                           {{"left", {"branch"}, {}, {}, true},
                            {"right", {"branch"}, {}, {}, true},
                            {"join", {"join"}, {"left", "right"}, {}, true}}};
    TaskGraph graph(config, registry);
    ArtifactStore store;
    const auto report = graph.run(store);
    check(report.ok() && maximum.load() == 2, "independent tasks did not execute in parallel");
}

void sharedWorkerPersistsAcrossFrames() {
    FeatureRegistry registry;
    std::mutex mutex;
    std::vector<unsigned> worker_tokens;
    registry.add("record-worker", factory([&](FeatureContext &) {
                     std::lock_guard<std::mutex> lock(mutex);
                     worker_tokens.push_back(currentThreadToken());
                 }));
    TaskGraphConfig config{
        1, {{"per-frame-task", {"record-worker"}, {}, {}, true}}};
    TaskGraph graph(std::move(config), registry);
    const auto caller_token = currentThreadToken();
    ArtifactStore first_frame;
    ArtifactStore second_frame;
    check(graph.run(first_frame).ok(), "first frame failed on the shared worker pool");
    check(graph.run(second_frame).ok(), "second frame failed on the shared worker pool");
    {
        std::lock_guard<std::mutex> lock(mutex);
        check(worker_tokens.size() == 2,
              "shared worker did not execute exactly one task per frame");
        check(worker_tokens[0] == worker_tokens[1],
              "TaskGraph recreated its worker thread between frames");
        check(worker_tokens[0] != caller_token,
              "TaskGraph executed per-frame work on the calling thread");
    }
    graph.close();
}

void failureIsolationAndRollback() {
    FeatureRegistry registry;
    registry.add("fail", factory([](FeatureContext &context) {
                     context.set<int>("uncommitted", 1);
                     throw std::runtime_error("expected failure");
                 }));
    registry.add("never", factory([](FeatureContext &context) { context.set<int>("bad", 1); }));
    registry.add("healthy", factory([](FeatureContext &context) { context.set<int>("good", 2); }));
    TaskGraphConfig config{3,
                           {{"failing", {"fail"}, {}, {}, true},
                            {"dependent", {"never"}, {"failing"}, {}, true},
                            {"independent", {"healthy"}, {}, {}, true}}};
    TaskGraph graph(config, registry);
    ArtifactStore store;
    const auto report = graph.run(store);
    check(!report.ok(), "failed graph reported success");
    check(report.tasks[0].state == TaskState::Failed &&
              report.tasks[1].state == TaskState::Skipped &&
              report.tasks[2].state == TaskState::Succeeded,
          "failure/skip/independent states are wrong");
    check(!store.contains("uncommitted") && !store.contains("bad") &&
              *store.get<int>("good") == 2,
          "transaction rollback or independent branch failed");
}

void configParsing() {
    const auto path = std::filesystem::temp_directory_path() /
                      std::filesystem::u8path(u8"感知-r12-graph-test.cfg");
    {
        std::ofstream file(path);
        file << "worker_threads=2\n"
                "result_artifacts=first.value,second.value\n"
                "tasks=a,b\n"
                "task.a.features=first:ok,second:ok\n"
                "task.a.depends_on=\n"
                "task.a.param.value=42\n"
                "task.a.feature.first.param.output_key=first.value\n"
                "task.a.feature.second.param.input_key=first.value\n"
                "task.a.feature.second.param.output_key=second.value\n"
                "task.b.features=ok\n"
                "task.b.depends_on=a\n";
    }
    const auto config = loadTaskGraphConfig(path.u8string());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    check(config.worker_threads == 2 && config.tasks.size() == 2 &&
              config.result_artifacts ==
                  std::vector<std::string>({"first.value", "second.value"}) &&
              config.tasks[0].parameters.at("value") == "42" &&
              config.tasks[0].features == std::vector<std::string>({"ok", "ok"}) &&
              config.tasks[0].feature_parameters[0].at("output_key") == "first.value" &&
              config.tasks[0].feature_parameters[1].at("input_key") == "first.value" &&
              config.tasks[1].depends_on == std::vector<std::string>{"a"},
          "task graph config parsing failed");
    FeatureRegistry registry;
    std::vector<Parameters> factory_parameters;
    registry.add("ok", [&](const Parameters &parameters) {
        factory_parameters.push_back(parameters);
        return std::make_unique<LambdaFeature>([](FeatureContext &context) {
                     const auto output = context.parameter("output_key");
                     if (output.empty())
                         return;
                     int value = context.parameterInt("value", 0);
                     const auto input = context.parameter("input_key");
                     if (!input.empty())
                         value = *context.get<int>(input) + 1;
                     context.set<int>(output, value);
                 });
    }, FeatureParameterSchema{{"value", FeatureParameterType::Integer},
                              {"input_key", FeatureParameterType::String},
                              {"output_key", FeatureParameterType::String}});
    TaskGraph graph(config, registry);
    check(factory_parameters.size() == 3 &&
              factory_parameters[0].at("value") == "42" &&
              factory_parameters[0].at("output_key") == "first.value" &&
              factory_parameters[1].at("input_key") == "first.value" &&
              factory_parameters[1].at("output_key") == "second.value",
          "feature factory did not receive merged alias-local parameters");
    ArtifactStore store;
    const auto report = graph.run(store);
    check(report.ok() && *store.get<int>("first.value") == 42 &&
              *store.get<int>("second.value") == 43,
          "aliased feature-local parameters did not compose within one task");

    const auto invalid = std::filesystem::temp_directory_path() /
                         "perception-r12-invalid-workers.cfg";
    {
        std::ofstream file(invalid);
        file << "worker_threads=4junk\ntasks=a\ntask.a.features=ok\n";
    }
    rejects([&] { (void)loadTaskGraphConfig(invalid.u8string()); },
            "worker_threads accepted trailing characters");
    std::filesystem::remove(invalid, ignored);
}

void artifactSnapshot() {
    ArtifactStore store;
    store.put<int>("input.borrowed", 1);
    store.put<std::vector<int>>("extension.output", {2, 3, 5});
    const auto snapshot = store.snapshot([](const std::string &key, std::type_index) {
        return key.rfind("input.", 0) != 0;
    });
    check(!snapshot->contains("input.borrowed") &&
              snapshot->get<std::vector<int>>("extension.output")->at(2) == 5,
          "filtered immutable artifact snapshot is wrong");
}
} // namespace

int main() {
    try {
        registryAndValidation();
        exclusiveResourceDependencies();
        localChainAndDependency();
        parallelAndJoin();
        sharedWorkerPersistsAcrossFrames();
        failureIsolationAndRollback();
        configParsing();
        artifactSnapshot();
        std::cout << "PASS r12-v3 feature registry, persistent pool, and DAG scheduling\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
