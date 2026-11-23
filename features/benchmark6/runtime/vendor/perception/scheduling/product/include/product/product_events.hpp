#pragma once
#include "perception/task_graph.hpp"
#include "runtime_status.hpp"
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace r30::product {
// Host callbacks retain their vendor/ROS signatures. Their real business work
// runs synchronously through the original TaskGraph. Each event type owns a
// separate graph, so a command waiting for an SDK callback never owns that
// callback's graph lock. This adds no cross-frame AI queue.
struct EventContext {
    std::string event;
    std::uint64_t event_id=0,parent_run_id=0;
    perception::FrameInfo frame;
};
class ProductEvents {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    using Bindings=std::map<std::string,std::function<void()>>;
    explicit ProductEvents(const std::string& config_path={},const std::string& metrics_dir={});
    ~ProductEvents();
    ProductEvents(const ProductEvents&)=delete;
    ProductEvents& operator=(const ProductEvents&)=delete;
    std::uint32_t run(const std::string& event,const Bindings&,perception::FrameInfo,
                      std::uint64_t parent_run_id,perception::runtime::GraphReport&,r28::Status&) noexcept;
    std::vector<std::string> registeredFeatures() const;
    void close();
};
}
