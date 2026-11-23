#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <algorithm>
#include <functional>
#include <map>
#include <set>

namespace v3 {
namespace {
using namespace detail;
struct Span {
  std::vector<std::string> entries, exits;
};
class DagCompiler {
public:
  explicit DagCompiler(const Json &requested) {
    const auto &tasks = get(requested, "tasks");
    if (tasks.kind != Json::Kind::Object || tasks.object.empty() || tasks.object.size() > 64)
      throw std::invalid_argument("tasks must define 1..64 product tasks");
    for (const auto &item : tasks.object) {
      if (item.first.empty() || item.first.size() > 128)
        throw std::invalid_argument("Task id must contain 1..128 bytes");
      Json node;
      if (item.second.kind == Json::Kind::String) {
        node = object();
        node.object["feature"] = item.second;
      } else {
        keys(item.second, {"feature", "features", "algorithm", "inputs", "parameters", "enabled"},
             "tasks." + item.first);
        node = item.second;
        if (has(node, "features")) {
          if (has(node, "feature"))
            throw std::invalid_argument("Task cannot contain both feature and features: " + item.first);
          const auto &features = get(node, "features");
          if (features.kind != Json::Kind::Array || features.array.size() != 1)
            throw std::invalid_argument("Each product task requires exactly one feature: " + item.first);
          node.object["feature"] = features.array.front();
          node.object.erase("features");
        }
      }
      text(get(node, "feature"), "tasks." + item.first + ".feature");
      node.object["id"] = str(item.first);
      node.object["enabled"] = boolean(flag(get(node, "enabled", boolean(true)), "task.enabled"));
      node.object["depends_on"] = array();
      nodes_.emplace(item.first, std::move(node));
    }
  }

  Json compile(const Json &requested) {
    expand(get(requested, "dag"), 0);
    for (const auto &item : nodes_) {
      if (!used_.count(item.first)) {
        if (get(item.second, "enabled").boolean)
          throw std::invalid_argument("Enabled task is not referenced in dag: " + item.first);
        order_.push_back(item.first);
      }
    }
    if (has(requested, "edges")) {
      const auto &edges = get(requested, "edges");
      if (edges.kind != Json::Kind::Array || edges.array.size() > 4096)
        throw std::invalid_argument("edges must be an array with at most 4096 entries");
      for (const auto &edge : edges.array) {
        keys(edge, {"from", "to"}, "edges item");
        connect(text(get(edge, "from"), "edge.from"), text(get(edge, "to"), "edge.to"));
      }
    }
    Json graph = object();
    const auto engine = get(requested, "engine", object());
    keys(engine, {"worker_threads"}, "engine");
    graph.object["worker_threads"] = integer(count(get(engine, "worker_threads", integer(3)),
                                                  "engine.worker_threads", 1, 32));
    graph.object["nodes"] = array();
    for (const auto &id : order_)
      graph.object["nodes"].array.push_back(nodes_.at(id));
    // This is the same data-dependency and cycle validator used by graph.nodes.
    // In particular, mapping stages cannot race their shared frame workspace.
    (void)r33::full::parseProductGraph(graph);
    return graph;
  }

private:
  void connect(const std::string &from, const std::string &to) {
    if (!nodes_.count(from) || !nodes_.count(to))
      throw std::invalid_argument("DAG edge references unknown task: " + from + " -> " + to);
    if (from == to)
      throw std::invalid_argument("DAG self edge: " + from);
    auto &parents = nodes_.at(to).object["depends_on"].array;
    if (std::any_of(parents.begin(), parents.end(), [&](const Json &p) { return p.text == from; }))
      throw std::invalid_argument("Duplicate DAG edge: " + from + " -> " + to);
    parents.push_back(str(from));
  }

  Span expand(const Json &value, std::size_t depth) {
    if (depth > 64)
      throw std::invalid_argument("DAG nesting exceeds 64 levels");
    if (value.kind == Json::Kind::String) {
      const auto id = text(value, "dag task reference");
      if (!nodes_.count(id))
        throw std::invalid_argument("Unknown DAG task: " + id);
      if (!used_.insert(id).second)
        throw std::invalid_argument("Repeated DAG task reference: " + id);
      order_.push_back(id);
      return {{id}, {id}};
    }
    if (value.kind != Json::Kind::Object || value.object.size() != 1)
      throw std::invalid_argument("Each dag node must contain exactly one of task, sequence, parallel");
    const auto &entry = *value.object.begin();
    if (entry.first == "task") {
      if (entry.second.kind != Json::Kind::String)
        throw std::invalid_argument("dag.task must be a task id string");
      return expand(entry.second, depth + 1);
    }
    if (entry.first != "sequence" && entry.first != "parallel")
      throw std::invalid_argument("Unknown DAG operator: " + entry.first);
    if (entry.second.kind != Json::Kind::Array || entry.second.array.empty() ||
        (entry.first == "parallel" && entry.second.array.size() < 2))
      throw std::invalid_argument("dag." + entry.first + " requires " +
                                  (entry.first == "parallel" ? "at least two branches" : "a nonempty array"));
    Span result;
    bool first = true;
    for (const auto &child : entry.second.array) {
      auto branch = expand(child, depth + 1);
      if (entry.first == "sequence") {
        if (first)
          result.entries = branch.entries;
        else
          for (const auto &from : result.exits)
            for (const auto &to : branch.entries)
              connect(from, to);
        result.exits = std::move(branch.exits);
      } else {
        result.entries.insert(result.entries.end(), branch.entries.begin(), branch.entries.end());
        result.exits.insert(result.exits.end(), branch.exits.begin(), branch.exits.end());
      }
      first = false;
    }
    return result;
  }

  std::map<std::string, Json> nodes_;
  std::set<std::string> used_;
  std::vector<std::string> order_;
};
} // namespace

Json normalizePipeline(const Json &requested) {
  if (requested.kind != Json::Kind::Object)
    throw std::invalid_argument("Pipeline configuration must be an object");
  const bool structured = has(requested, "dag");
  if (has(requested, "graph")) {
    if (structured || has(requested, "tasks") || has(requested, "engine") || has(requested, "edges"))
      throw std::invalid_argument("Use either tasks + dag or legacy graph, never both");
    return requested;
  }
  if (!structured) {
    if (has(requested, "tasks") || has(requested, "engine") || has(requested, "edges"))
      throw std::invalid_argument("tasks, engine and edges require dag");
    return requested;
  }
  auto result = requested;
  result.object["graph"] = DagCompiler(requested).compile(requested);
  for (const auto *key : {"dag", "tasks", "engine", "edges"})
    result.object.erase(key);
  return result;
}
} // namespace v3
