#pragma once
#include "algorithm_catalog.hpp"
#include "perception/task_graph.hpp"
#include <memory>

namespace perception::algorithms {
struct Invocation {
    std::string algorithm, task_id;
    Json parameters;
    std::map<std::string, std::string> inputs;
};
// A node publishes this immutable result in its FeatureContext transaction.
struct Output { std::string json; };
inline std::string resultKey(const std::string& task_id) {
    return "algorithm.result." + task_id;
}
inline std::string bindingKey(const std::string& task_id) {
    return "algorithm:" + task_id;
}
// Generated factories use exactly the same registration manifests as the catalog.
std::unique_ptr<runtime::Feature> create(const Invocation&);
}
