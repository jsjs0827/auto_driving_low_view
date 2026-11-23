#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <set>

namespace {
using namespace v3;
using namespace v3::detail;
namespace rt = perception::runtime;
int checks = 0;
void require(bool condition, const std::string &message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
Json parse(const std::string &text) { return r33::json::JsonParser(text, "DAG test").parse(); }
template <class Fn> void rejected(Fn fn, const std::string &expected) {
  try { fn(); }
  catch (const std::exception &e) {
    require(std::string(e.what()).find(expected) != std::string::npos,
            "Expected '" + expected + "', got: " + e.what());
    return;
  }
  throw std::runtime_error("Invalid DAG was accepted: " + expected);
}
r33::full::ProductGraphPlan plan(const Json &request) {
  return r33::full::parseProductGraph(get(normalizePipeline(request), "graph"));
}
std::map<std::string, std::set<std::string>> ancestors(const rt::TaskGraphConfig &config) {
  std::map<std::string, std::vector<std::string>> parents;
  for (const auto &task : config.tasks) parents[task.id] = task.depends_on;
  std::map<std::string, std::set<std::string>> result;
  std::function<void(const std::string &, const std::string &)> walk =
      [&](const std::string &target, const std::string &id) {
        for (const auto &parent : parents.at(id))
          if (result[target].insert(parent).second) walk(target, parent);
      };
  for (const auto &task : config.tasks) { result[task.id]; walk(task.id, task.id); }
  // v2 removes dormant edges from user files. Compare the effective execution
  // dependencies of enabled tasks; inactive definitions keep their identities.
  std::set<std::string> enabled;
  for (const auto &task : config.tasks) if (task.enabled) enabled.insert(task.id);
  for (auto it = result.begin(); it != result.end();) {
    if (!enabled.count(it->first)) it = result.erase(it);
    else {
      for (auto p = it->second.begin(); p != it->second.end();)
        if (!enabled.count(*p)) p = it->second.erase(p); else ++p;
      ++it;
    }
  }
  return result;
}
void migratedConfigurations(const Path &root) {
  const auto manifest = readJson(root / "validation/dag_legacy_configs/manifest.json");
  for (const auto &item : manifest.array) {
    const auto old = readJson(root / "validation/dag_legacy_configs" / item.text);
    const auto requested = readJson(root / item.text);
    require(has(requested, "dag") && has(requested, "tasks") && !has(requested, "graph"),
            item.text + " was not migrated to tasks + dag");
    require(!has(requested, "edges"), item.text + " still requires from/to edges");
    const auto expected = plan(old), actual = plan(requested);
    require(expected.config.worker_threads == actual.config.worker_threads,
            item.text + " changed worker count");
    require(expected.stage_for_task == actual.stage_for_task,
            item.text + " changed task identity or algorithm");
    require(expected.inputs_for_task == actual.inputs_for_task,
            item.text + " changed input bindings");
    for (const auto &stage : r33::full::productStages())
      require(expected.enabled(stage.key) == actual.enabled(stage.key),
              item.text + " changed enabled stages");
    require(ancestors(expected.config) == ancestors(actual.config),
            item.text + " changed enabled dependency reachability");
    require(jsonText(normalizePipeline(normalizePipeline(requested))) ==
                jsonText(normalizePipeline(requested)), "Normalization is not idempotent");
  }
  require(manifest.array.size() == 15, "Expected all 15 shipped pipeline configurations");
}
Json simple() {
  return parse(R"({"schema_version":5,"engine":{"worker_threads":3},
    "tasks":{"det":{"feature":"inference","algorithm":"det"},
             "seg":{"feature":"inference","algorithm":"seg"},"publish":"publish"},
    "dag":{"sequence":[{"parallel":["det","seg"]},"publish"]}})");
}
void validationCases() {
  const auto base = simple();
  auto serial = base;
  serial.object["dag"] = parse(R"({"sequence":["det",{"task":"seg"},"publish"]})");
  require(ancestors(plan(serial).config).at("seg").count("det"), "Sequence edge missing");
  const auto parallel = ancestors(plan(base).config);
  require(!parallel.at("det").count("seg") && !parallel.at("seg").count("det") &&
          parallel.at("publish").size() == 2, "Parallel/join semantics changed");
  auto disabled = serial;
  disabled.object["tasks"].object["seg"].object["enabled"] = boolean(false);
  const auto bypass = plan(disabled);
  require(!bypass.enabled("seg") && ancestors(bypass.config).at("publish").count("det") &&
          !ancestors(bypass.config).at("publish").count("seg"), "Disabled node was not bypassed");
  auto unused_disabled = base;
  unused_disabled.object["tasks"].object["extra"] = parse(R"({"feature":"history","enabled":false})");
  require(!plan(unused_disabled).enabled("history"), "Unused disabled task became enabled");
  auto features = parse(R"({"tasks":{"det":{"features":["product.ai.detection"]}},"dag":"det"})");
  require(plan(features).enabled("det"), "Legacy single-feature task definition failed");
  auto bad = base;
  bad.object["dag"] = parse(R"({"sequence":["det","missing"]})");
  rejected([&]{ plan(bad); }, "Unknown DAG task");
  bad = base; bad.object["dag"] = parse(R"({"parallel":["det","det"]})");
  rejected([&]{ plan(bad); }, "Repeated DAG task");
  bad = base; bad.object["dag"] = parse(R"({"parallel":["det"]})");
  rejected([&]{ plan(bad); }, "at least two");
  bad = base; bad.object["dag"] = parse(R"({"sequence":[]})");
  rejected([&]{ plan(bad); }, "nonempty");
  bad = base; bad.object["dag"] = str("det");
  rejected([&]{ plan(bad); }, "not referenced");
  bad = base; bad.object["graph"] = object();
  rejected([&]{ plan(bad); }, "never both");
  bad = base; bad.object["edges"] = parse(R"([{"from":"publish","to":"det"}])");
  rejected([&]{ plan(bad); }, "Cycle");
  bad = base; bad.object["edges"] = parse(R"([{"from":"det","to":"publish"}])");
  rejected([&]{ plan(bad); }, "Duplicate DAG edge");
  bad = base; bad.object["edges"] = parse(R"([{"from":"missing","to":"det"}])");
  rejected([&]{ plan(bad); }, "unknown task");
  bad = base; bad.object["edges"] = parse(R"([{"from":"det","to":"det"}])");
  rejected([&]{ plan(bad); }, "self edge");
  bad = base; bad.object["engine"].object["worker_threads"] = integer(33);
  rejected([&]{ plan(bad); }, "engine.worker_threads");
  bad = base; bad.object["tasks"].object["det"].object["enabled"] = str("false");
  rejected([&]{ plan(bad); }, "task.enabled");
  bad = base; bad.object["tasks"].object["det"].object["typo"] = boolean(true);
  rejected([&]{ plan(bad); }, "typo");
  bad = parse(R"({"tasks":{"stereo":"disparity","cloud":"cloud"},
                  "dag":{"parallel":["stereo","cloud"]}})");
  rejected([&]{ plan(bad); }, "requires enabled ancestor disparity");
  bad.object["dag"] = parse(R"({"sequence":["stereo","cloud"]})");
  require(plan(bad).enabled("cloud"), "Valid geometry-only sequence rejected");
  bad = parse(R"({"tasks":{"a":"det","b":"det"},"dag":{"parallel":["a","b"]}})");
  rejected([&]{ plan(bad); }, "only once");
  auto crossed = base;
  crossed.object["edges"] = parse(R"([{"from":"det","to":"seg"}])");
  require(ancestors(plan(crossed).config).at("seg").count("det"), "Explicit cross edge missing");
}
class Callback final : public rt::Feature {
public:
  explicit Callback(std::function<void(rt::FeatureContext &)> f) : fn_(std::move(f)) {}
  void execute(rt::FeatureContext &ctx) override { fn_(ctx); }
private:
  std::function<void(rt::FeatureContext &)> fn_;
};
void executionCases() {
  rt::FeatureRegistry registry;
  std::mutex mutex;
  std::condition_variable ready;
  int arrived = 0;
  std::atomic<int> completed{0};
  const auto add = [&](const std::string &feature, std::function<void(rt::FeatureContext &)> fn) {
    registry.add(feature, [fn](const rt::Parameters &) { return std::make_unique<Callback>(fn); });
  };
  const auto lane = [&](rt::FeatureContext &ctx) {
    std::unique_lock<std::mutex> lock(mutex);
    ++arrived; ready.notify_all();
    if (!ready.wait_for(lock, std::chrono::seconds(3), [&]{ return arrived == 2; }))
      throw std::runtime_error("Independent DAG lanes did not execute concurrently");
    lock.unlock();
    ctx.set(ctx.taskId() + ".done", true);
    ++completed;
  };
  add("product.ai.detection", lane);
  add("product.ai.segmentation", lane);
  add("product.output.application", [&](rt::FeatureContext &ctx) {
    if (completed != 2 || !ctx.get<bool>("det.done") || !ctx.get<bool>("seg.done"))
      throw std::runtime_error("Join ran before both committed results");
  });
  rt::TaskGraph graph(plan(simple()).config, registry);
  rt::ArtifactStore store;
  require(graph.run(store).ok() && completed == 2, "Real parallel task execution failed");

  rt::FeatureRegistry serial_registry;
  std::atomic<int> order{0};
  for (const auto &entry : std::map<std::string, int>{{"product.ai.detection", 0},
       {"product.ai.segmentation", 1}, {"product.output.application", 2}}) {
    serial_registry.add(entry.first, [&, expected = entry.second](const rt::Parameters &) {
      return std::make_unique<Callback>([&, expected](rt::FeatureContext &) {
        if (order.fetch_add(1) != expected) throw std::runtime_error("Sequence ran out of order");
      });
    });
  }
  auto request = simple();
  request.object["dag"] = parse(R"({"sequence":["det","seg","publish"]})");
  rt::TaskGraph serial(plan(request).config, serial_registry);
  rt::ArtifactStore serial_store;
  require(serial.run(serial_store).ok() && order == 3, "Real sequential execution failed");

  rt::FeatureRegistry failing_registry;
  for (const auto &feature : {"product.ai.detection", "product.ai.segmentation",
                              "product.output.application"})
    failing_registry.add(feature, [name = std::string(feature)](const rt::Parameters &) {
      return std::make_unique<Callback>([name](rt::FeatureContext &) {
        if (name == "product.ai.detection") throw std::runtime_error("Expected lane failure");
      });
    });
  rt::TaskGraph failing(plan(request).config, failing_registry);
  rt::ArtifactStore failed_store;
  const auto failed = failing.run(failed_store);
  require(!failed.ok(), "Failed task produced a successful run");
  require(failed.tasks.at(1).state == rt::TaskState::Skipped &&
          failed.tasks.at(2).state == rt::TaskState::Skipped,
          "Dependents were not skipped after failure");
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::invalid_argument("native_dag_test PROJECT_ROOT");
    validationCases();
    executionCases();
    migratedConfigurations(std::filesystem::absolute(argv[1]));
    std::cout << "Structured DAG checks passed: " << checks
              << "; all 15 configurations omit edges and preserve enabled dependencies; "
                 "real scheduler serial/parallel/failure propagation verified\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
