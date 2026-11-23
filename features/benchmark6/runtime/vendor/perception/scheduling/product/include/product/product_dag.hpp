#pragma once
#include "perception/task_graph.hpp"
#include "runtime_status.hpp"
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace r28::product {
// Bindings own no worker or frame queue. All jobs run in the original TaskGraph.
// A missing binding disables that task for this frame; enabled dependencies are
// retained, so an error skips dependent publication and recording.
class ProductDag {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    using Bindings = std::map<std::string, std::function<void()>>;
    explicit ProductDag(const std::string& config_path = {},const std::string& metrics_dir = {});
    ~ProductDag();
    ProductDag(const ProductDag&) = delete;
    ProductDag& operator=(const ProductDag&) = delete;
    std::uint32_t run(const Bindings&, perception::FrameInfo,
                      perception::runtime::GraphReport&, r28::Status&) noexcept;
    std::vector<std::string> registeredFeatures() const;
    std::uint64_t currentRunId() const noexcept;
    void recordExternalStatus(const r28::Status&,std::uint64_t run_id) noexcept;
};
}
