#pragma once

#include "json.hpp"
#include "perception/task_graph.hpp"
#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace r33::full {

// One user-facing graph describes product work. Inference instances share a
// feature type but select a distinct algorithm and retain independent scheduling.
struct ProductStage {
  const char *key;
  const char *feature;
};
inline const std::vector<ProductStage> &productStages() {
  static const std::vector<ProductStage> stages{
      {"dnn", "product.ai.exact"},
      {"det", "product.ai.detection"},
      {"seg", "product.ai.segmentation"},
      {"recharge", "product.recharge_mark"},
      {"disparity", "product.stereo.disparity"},
      {"rgb", "product.image.rgb"},
      {"cloud", "product.pointcloud.from_disparity"},
      {"calibration", "product.calibration.auto"},
      {"labels", "product.fusion.semantic_labels"},
      {"transform", "product.pointcloud.transform_color"},
      {"ground", "product.pointcloud.patchwork"},
      {"maps", "product.fusion.ai_fusion"},
      {"history", "product.temporal.history"},
      {"visual_motion", "product.motion.visual"},
      {"motion_reference", "product.motion.reference"},
      {"slip_detection", "product.motion.slip"},
      {"temporal_difference", "product.temporal.difference"},
      {"temporal_match", "product.temporal.match"},
      {"temporal_extension", "product.temporal.extension"},
      {"publish", "product.output.application"},
      {"record", "product.output.record"}};
  return stages;
}
inline std::string productStageKey(const std::string &feature) {
  if (feature == "inference")
    return "dnn";
  if (feature == "pointcloud")
    return "cloud";
  if (feature == "fusion")
    return "maps";
  for (const auto &stage : productStages())
    if (feature == stage.key || feature == stage.feature)
      return stage.key;
  throw std::invalid_argument("Unregistered product feature: " + feature);
}
inline std::string productFeatureName(const std::string &key) {
  for (const auto &stage : productStages())
    if (key == stage.key)
      return stage.feature;
  throw std::invalid_argument("Unknown product stage: " + key);
}

struct ProductGraphPlan {
  bool configured = false;
  bool stereo_inference = false;
  perception::runtime::TaskGraphConfig config;
  std::map<std::string, std::string> stage_for_task;
  std::map<std::string, std::map<std::string, std::string>> inputs_for_task;
  bool enabled(const std::string &key) const {
    for (const auto &task : config.tasks)
      if (task.enabled && stage_for_task.at(task.id) == key)
        return true;
    return false;
  }
};

inline ProductGraphPlan parseProductGraph(const json::JsonValue &graph) {
  using namespace json;
  fields(graph, {"worker_threads", "nodes", "_help"});
  ProductGraphPlan plan;
  plan.configured = true;
  plan.config.worker_threads = 2;
  if (auto value = optional(graph, "worker_threads"))
    plan.config.worker_threads = integer(*value, 1, 32);
  const auto &nodes = required(graph, "nodes");
  if (nodes.kind != JsonValue::Kind::Array || nodes.array.empty() || nodes.array.size() > 64)
    throw std::invalid_argument("graph.nodes must contain 1..64 nodes");
  std::map<std::string, std::size_t> index;
  std::set<std::string> selected;
  for (const auto &node : nodes.array) {
    fields(node, {"id", "feature", "algorithm", "inputs", "depends_on", "enabled", "_help"});
    perception::runtime::TaskSpec task;
    task.id = string(required(node, "id"));
    if (task.id.size() > 128)
      throw std::invalid_argument("Graph node id exceeds 128 bytes");
    const auto feature = string(required(node, "feature"));
    auto key = productStageKey(feature);
    if (auto algorithm = optional(node, "algorithm")) {
      if (feature != "inference")
        throw std::invalid_argument("graph algorithm is only supported by feature inference");
      const auto name = string(*algorithm);
      if (name == "det") key = "det";
      else if (name == "seg") key = "seg";
      else if (name == "stereo") { key = "disparity"; plan.stereo_inference = true; }
      else throw std::invalid_argument("inference.algorithm must be det, seg or stereo");
    }
    std::map<std::string, std::string> inputs;
    if (key == "det" || key == "seg") inputs = {{"image", "input.left"}};
    else if (key == "disparity") inputs = {{"left", "input.left"}, {"right", "input.right"}};
    if (auto bindings = optional(node, "inputs")) {
      if (key == "det" || key == "seg") fields(*bindings, {"image"});
      else if (key == "disparity") fields(*bindings, {"left", "right"});
      else throw std::invalid_argument("graph inputs are supported only for inference algorithms");
      for (const auto &input : inputs) {
        const auto actual = string(required(*bindings, input.first.c_str()));
        if (actual != input.second)
          throw std::invalid_argument("Unsupported inference input binding " + input.first +
                                      ": expected " + input.second + ", got " + actual);
      }
    }
    task.features = {productFeatureName(key)};
    if (auto value = optional(node, "enabled"))
      task.enabled = boolean(*value);
    if (auto value = optional(node, "depends_on")) {
      if (value->kind != JsonValue::Kind::Array)
        throw std::invalid_argument("depends_on must be an array");
      std::set<std::string> parents;
      for (const auto &parent : value->array) {
        const auto id = string(parent);
        if (!parents.insert(id).second)
          throw std::invalid_argument("Duplicate dependency: " + id);
        task.depends_on.push_back(id);
      }
    }
    if (!index.emplace(task.id, plan.config.tasks.size()).second)
      throw std::invalid_argument("Duplicate graph node: " + task.id);
    if (!selected.insert(key).second)
      throw std::invalid_argument("Product features may occur only once per frame: " + key);
    plan.stage_for_task.emplace(task.id, key);
    plan.inputs_for_task.emplace(task.id, std::move(inputs));
    plan.config.tasks.push_back(std::move(task));
  }
  if (selected.count("dnn") && (selected.count("det") || selected.count("seg")))
    throw std::invalid_argument("Legacy inference cannot be mixed with det/seg inference instances");
  std::vector<int> state(plan.config.tasks.size());
  std::function<void(std::size_t)> visit = [&](std::size_t i) {
    if (state[i] == 1)
      throw std::invalid_argument("Cycle in product graph at " + plan.config.tasks[i].id);
    if (state[i] == 2)
      return;
    state[i] = 1;
    for (const auto &parent : plan.config.tasks[i].depends_on) {
      auto found = index.find(parent);
      if (found == index.end())
        throw std::invalid_argument("Unknown graph dependency: " + parent);
      visit(found->second);
    }
    state[i] = 2;
  };
  for (std::size_t i = 0; i < state.size(); ++i)
    visit(i);

  // Disabled control nodes may be bypassed, but the data contracts below
  // still require real producers. No missing image/cloud is fabricated.
  const auto original = plan.config.tasks;
  for (auto &task : plan.config.tasks)
    if (task.enabled) {
      std::set<std::string> kept;
      std::function<void(const std::string &)> expand = [&](const std::string &id) {
        const auto &parent = original.at(index.at(id));
        if (parent.enabled)
          kept.insert(id);
        else
          for (const auto &ancestor : parent.depends_on)
            expand(ancestor);
      };
      for (const auto &parent : task.depends_on)
        expand(parent);
      task.depends_on.assign(kept.begin(), kept.end());
    }
  const auto ancestor = [&](const std::string &child, const std::string &producer) {
    std::set<std::string> seen;
    std::function<bool(const std::string &)> walk = [&](const std::string &id) {
      if (!seen.insert(id).second)
        return false;
      const auto &task = plan.config.tasks.at(index.at(id));
      for (const auto &parent : task.depends_on)
        if (plan.stage_for_task.at(parent) == producer || walk(parent))
          return true;
      return false;
    };
    for (const auto &task : plan.config.tasks)
      if (task.enabled && plan.stage_for_task.at(task.id) == child)
        return walk(task.id);
    return false;
  };
  const auto
    requires
  = [&](const std::string &consumer, const std::string &producer) {
      if (plan.enabled(consumer) && (!plan.enabled(producer) || !ancestor(consumer, producer)))
        throw std::invalid_argument("graph feature " + consumer + " requires enabled ancestor " +
                                    producer);
    };
  requires("cloud", "disparity");
  requires("calibration", "cloud");
  requires("labels", "cloud");
  for (const auto *stage : {"dnn", "det", "seg"})
    if (plan.enabled(stage)) requires("labels", stage);
  if (plan.enabled("labels") && !plan.enabled("dnn") && !plan.enabled("det") && !plan.enabled("seg"))
    throw std::invalid_argument("graph labels requires an enabled det, seg or legacy inference producer");
  requires("transform", "cloud");
  requires("ground", "transform");
  requires("maps", "ground");
  // Mapping owns one reusable frame workspace; selected mapping operations
  // must remain ordered even when optional operations are disabled.
  std::string previous;
  for (const auto *stage : {"cloud", "calibration", "labels", "transform", "ground", "maps"})
    if (plan.enabled(stage)) {
      if (!previous.empty())
      requires(stage, previous);
      previous = stage;
    }
  requires("temporal_difference", "history");
  requires("temporal_match", "history");
  requires("temporal_extension", "history");
  requires("visual_motion", "history");
  requires("visual_motion", "disparity");
  requires("motion_reference", "history");
  requires("slip_detection", "visual_motion");
  requires("slip_detection", "motion_reference");
  // History captures input planes and the selected same-frame model/disparity
  // products. Independent downstream mapping must not delay this window.
  for (const auto *stage : {"dnn", "det", "seg", "disparity"})
    if (plan.enabled(stage))
  requires("history", stage);
  if (plan.enabled("temporal_difference"))
  requires("temporal_match", "temporal_difference");
  for (const auto *stage : {"temporal_difference", "temporal_match"})
    if (plan.enabled(stage))
  requires("temporal_extension", stage);
  // Hooks and recording read the whole result, and therefore join every
  // enabled producer. They may never race a late mapping/model write.
  for (const auto *sink : {"publish", "record"})
    for (const auto &task : plan.config.tasks)
      if (task.enabled) {
        const auto &key = plan.stage_for_task.at(task.id);
        if (key != sink && !(std::string(sink) == "publish" && key == "record"))
        requires(sink, key);
      }
  if (!plan.enabled("dnn") && !plan.enabled("det") && !plan.enabled("seg") && !plan.enabled("cloud") && !plan.enabled("recharge") &&
      !plan.enabled("rgb") && !plan.enabled("disparity"))
    throw std::invalid_argument("Graph has no enabled image/algorithm work");
  return plan;
}
} // namespace r33::full
