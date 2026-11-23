#pragma once
#include "runtime_status.hpp"
#include "perception/task_graph.hpp"
#include <functional>
#include <memory>
#include "perception_temporal.hpp"
#include "product_graph.hpp"
namespace r33::full {
class Graph {
    struct Impl;std::unique_ptr<Impl> p_;
public:
    using Bindings=std::map<std::string,std::function<void()>>;
    using ContextBindings=std::map<std::string,std::function<void(perception::runtime::FeatureContext&)>>;
    using InputSeed=std::function<void(perception::runtime::ArtifactStore&)>;
    Graph();~Graph();
    void configure(const ProductGraphPlan&);
    std::uint32_t run(const Bindings&,perception::FrameInfo,perception::runtime::GraphReport&,r28::Status&);
    std::uint32_t runContext(const ContextBindings&,perception::FrameInfo,perception::runtime::GraphReport&,r28::Status&,
        std::shared_ptr<const r36::temporal::Window> window={}, InputSeed seed={});
};
std::string traceJson(const perception::runtime::GraphReport&,std::uint64_t run_id);
}
