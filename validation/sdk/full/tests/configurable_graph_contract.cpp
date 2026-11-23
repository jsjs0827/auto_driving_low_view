#include "full_graph.hpp"
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
r33::full::ProductGraphPlan parse(const std::string &text) {
  return r33::full::parseProductGraph(r33::json::JsonParser(text, "test graph").parse());
}
void rejected(const std::string &text) {
  try {
    parse(text);
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Unsafe product graph was accepted");
}
} // namespace
int main() {
  try {
    // Deliberately reverse the legacy rgb-after-dnn order. The supplied graph
    // must really control the persistent scheduler, not merely label a preset.
    auto plan = parse(R"({"worker_threads":1,"nodes":[
        {"id":"decode_first","feature":"rgb"},
        {"id":"model_second","feature":"inference","depends_on":["decode_first"]},
        {"id":"unused_history","feature":"history","enabled":false}
    ]})");
    r33::full::Graph graph;
    graph.configure(plan);
    std::vector<std::string> order;
    r33::full::Graph::Bindings jobs;
    jobs["dnn"] = [&] {
      require(order.size() == 1 && order[0] == "rgb", "User dependency was ignored");
      order.push_back("dnn");
    };
    jobs["rgb"] = [&] {
      require(order.empty(), "Legacy dependency overrode user graph");
      order.push_back("rgb");
    };
    jobs["history"] = [] { throw std::runtime_error("Disabled feature executed"); };
    perception::runtime::GraphReport report;
    r28::Status status;
    require(graph.run(jobs, {1, 100}, report, status) == 0, "Configured graph failed");
    require(order.size() == 2 && report.tasks.size() == 3, "Configured tasks absent from report");
    require(report.tasks[0].id == "decode_first" && report.tasks[1].id == "model_second",
            "Custom node IDs lost");
    require(report.tasks[2].state == perception::runtime::TaskState::Skipped,
            "Disabled task did not remain skipped");

    // A graph that requests only inference must never run available but
    // unselected RGB/history work, including on the next cached invocation.
    graph.configure(parse(R"({"nodes":[{"id":"only_model","feature":"inference"}]})"));
    int calls = 0;
    jobs["dnn"] = [&] { ++calls; };
    jobs["rgb"] = [] { throw std::runtime_error("Unselected RGB executed"); };
    require(graph.run(jobs, {2, 101}, report, status) == 0, "Single feature graph failed");
    require(graph.run(jobs, {3, 102}, report, status) == 0 && calls == 2,
            "Cached graph lost current bindings");
    require(report.tasks.size() == 1 && report.tasks[0].id == "only_model",
            "Fixed graph leaked into custom graph");

    // Disabled optional control node bypass preserves its true ancestors.
    auto bypass = parse(R"({"nodes":[
      {"id":"ai","feature":"inference"},
      {"id":"optional","feature":"rgb","depends_on":["ai"],"enabled":false},
      {"id":"output","feature":"publish","depends_on":["optional"]}]})");
    require(bypass.config.tasks[2].depends_on == std::vector<std::string>{"ai"},
            "Disabled ancestor bypass lost dependency");
    // Shared inference type produces independent det/seg/stereo instances. RGB
    // is an internal conversion and cloud does not wait for the semantic lanes.
    auto split = parse(R"({"worker_threads":3,"nodes":[
      {"id":"detector","feature":"inference","algorithm":"det","inputs":{"image":"input.left"}},
      {"id":"segmenter","feature":"inference","algorithm":"seg"},
      {"id":"stereo","feature":"inference","algorithm":"stereo","inputs":{"left":"input.left","right":"input.right"}},
      {"id":"points","feature":"cloud","depends_on":["stereo"]},
      {"id":"semantics","feature":"labels","depends_on":["points","detector","segmenter"]},
      {"id":"out","feature":"publish","depends_on":["semantics"]}]})");
    require(split.enabled("det") && split.enabled("seg") && split.enabled("disparity") &&
                !split.enabled("dnn") && !split.enabled("rgb"), "Split inference plan incorrect");
    require(split.inputs_for_task.at("segmenter").at("image") == "input.left",
            "Default input binding absent");
    graph.configure(split);
    r33::full::Graph::ContextBindings split_jobs;
    for (const auto *stage : {"det", "seg", "disparity", "cloud", "labels", "publish"})
      split_jobs[stage] = [](perception::runtime::FeatureContext &ctx) {
        require(ctx.get<perception::FrameInfo>("input.frame")->id == 44,
                "Frame input was not seeded before DAG roots");
        require(*ctx.get<int>("input.left") == 6, "Typed image seed missing");
      };
    std::mutex split_mutex;
    std::condition_variable split_ready;
    bool points_ready = false;
    const auto verify_inputs = split_jobs.at("det");
    for (const auto *stage : {"det", "seg"})
      split_jobs[stage] = [&](perception::runtime::FeatureContext &ctx) {
        verify_inputs(ctx);
        std::unique_lock<std::mutex> lock(split_mutex);
        require(split_ready.wait_for(lock, std::chrono::seconds(3), [&] { return points_ready; }),
                "Geometry cloud remained blocked behind semantic inference");
      };
    split_jobs["cloud"] = [&](perception::runtime::FeatureContext &ctx) {
      verify_inputs(ctx);
      std::lock_guard<std::mutex> lock(split_mutex);
      points_ready = true;
      split_ready.notify_all();
    };
    require(graph.runContext(split_jobs, {44, 4400}, report, status, {},
                [](perception::runtime::ArtifactStore &store) {
                  store.put("input.frame", perception::FrameInfo{44, 4400});
                  store.put("input.left", 6);
                }) == 0 && report.tasks.size() == 6, "Split graph/typed input seed failed");
    rejected(R"({"nodes":[{"id":"a","feature":"inference","algorithm":"det"},{"id":"b","feature":"inference","algorithm":"det"}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference"},{"id":"b","feature":"inference","algorithm":"seg"}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference","algorithm":"classification"}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference","algorithm":"det","inputs":{"image":"input.right"}}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference","algorithm":"stereo","inputs":{"left":"input.left"}}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"rgb","algorithm":"det"}]})");
    // History samples source/model/Q8 products, not downstream cloud work.
    // Cloud deliberately waits for history: a hidden cloud->history edge would
    // stall this real scheduler invocation and fail the bounded wait.
    auto independent_history=parse(R"({"worker_threads":2,"nodes":[{"id":"q","feature":"disparity"},
      {"id":"c","feature":"cloud","depends_on":["q"]},
      {"id":"h","feature":"history","depends_on":["q"]}]})");
    graph.configure(independent_history);
    std::mutex history_mutex;std::condition_variable history_ready;bool sampled=false;
    jobs["disparity"]=[]{};
    jobs["cloud"]=[&]{
      std::unique_lock<std::mutex> lock(history_mutex);
      require(history_ready.wait_for(lock,std::chrono::seconds(3),[&]{return sampled;}),
              "History incorrectly waits for downstream cloud");
    };
    jobs["history"]=[&]{std::lock_guard<std::mutex> lock(history_mutex);sampled=true;history_ready.notify_all();};
    require(graph.run(jobs,{45,4500},report,status)==0&&sampled,
            "History independent from cloud did not execute");
    // Actual sampled producers must still be ancestors of history.
    rejected(R"({"nodes":[{"id":"d","feature":"inference"},{"id":"h","feature":"history"}]})");
    rejected(R"({"nodes":[{"id":"d","feature":"inference","algorithm":"det"},{"id":"h","feature":"history"}]})");
    rejected(R"({"nodes":[{"id":"s","feature":"inference","algorithm":"seg"},{"id":"h","feature":"history"}]})");
    rejected(R"({"nodes":[{"id":"q","feature":"disparity"},{"id":"h","feature":"history"}]})");
    // A consumer must wait for every enabled semantic producer, not one lane.
    rejected(R"({"nodes":[{"id":"d","feature":"inference","algorithm":"det"},
      {"id":"s","feature":"inference","algorithm":"seg"},
      {"id":"q","feature":"disparity"},{"id":"c","feature":"cloud","depends_on":["q"]},
      {"id":"l","feature":"labels","depends_on":["c","d"]}]})");
    rejected(R"({"nodes":[{"id":"c","feature":"cloud"}]})");
    rejected(R"({"nodes":[{"id":"m","feature":"maps"}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference","depends_on":["missing"]}]})");
    rejected(
        R"({"nodes":[{"id":"a","feature":"rgb","depends_on":["b"]},{"id":"b","feature":"inference","depends_on":["a"]}]})");
    rejected(
        R"({"nodes":[{"id":"a","feature":"inference"},{"id":"b","feature":"product.ai.exact"}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"uncompiled_feature"}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference","enabeld":false}]})");
    rejected(R"({"nodes":[{"id":"a","feature":"inference"},{"id":"p","feature":"publish"}]})");
    std::cout << "Configurable product graph execution and data dependencies passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
