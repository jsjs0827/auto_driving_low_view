#pragma once
#include "native_stages.hpp"
#include "exact_input.hpp"
#include "perception/runtime_engine.hpp"
#include "runtime_status.hpp"
#include <array>
#include <functional>
#include <map>
#include <memory>

namespace r25 {
struct DepthResult {
    bool requested=false;
    std::uint64_t frame_id=0;
    int code=-999,width=0,height=0;
    std::size_t pixels=0;
    double call_ms=0,copy_ms=0;
    std::int64_t begin_host_ns=0,end_host_ns=0;
    std::uint64_t output_fnv=0;
};
struct RunOptions {
    bool optimized=true,capture=false,preprocess_only=false,input_only=false;
    bool run_detection=true,run_segmentation=true;
    bool depth_parallel=false;
    r22::Variant variant=r22::Variant::CompactDirect;
    std::function<DepthResult(std::uint64_t)> depth;
    std::optional<perception::fusion::DisparityView> disparity_view;
};
struct FeatureTiming {
    std::string feature,task;
    double start_ms=0,end_ms=0;
    std::size_t thread=0;
    bool succeeded=false;
};
struct Output {
    // The framework owns immutable final results; callers may retain this
    // snapshot across the next run without borrowing model or pack buffers.
    perception::runtime::FrameResult frame;
    std::vector<ai_node_percep_demo::DnnDetection> det;
    const std::vector<std::uint8_t>& mask() const {
        static const std::vector<std::uint8_t> empty;
        return frame.segmentation?frame.segmentation->mask:empty;
    }
    ai_node_percep_demo::DnnInferTiming timing;
    std::map<std::string,r20::TensorAudit> audit;
    std::array<r22::InputStats,2> inputs;
    std::array<FeatureTiming,8> features;
    DepthResult depth;
    double pack_ms=0,wall_ms=0,graph_ms=0;
    // Backend-neutral durations. DnnInferTiming remains the original legacy
    // carrier; its historically named bpu fields are not an execution-provider tag.
    double det_infer_ms=0,seg_infer_ms=0;
    double output_ms=0,render_ms=0,encode_write_ms=0;
    std::uint64_t dag_run_id=0;
};
// The original Engine and scheduler execute every Feature. The only injected
// adapter is model-stage execution, which has no scheduling or frame queue.
class Pipeline {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    Pipeline(const std::string& graph,std::shared_ptr<Stages> stages={});
    ~Pipeline();
    Pipeline(const Pipeline&)=delete;
    Pipeline& operator=(const Pipeline&)=delete;
    Output run(const perception::ImageView&,perception::FrameInfo,const RunOptions&);
    // Failure never exposes a partially completed frame as a successful result.
    // The full uint32_t value survives independently of shell exit limitations.
    std::uint32_t tryRun(const perception::ImageView&,perception::FrameInfo,
                         const RunOptions&,Output&,r28::Status&) noexcept;
    // Explicit shutdown exposes cleanup failures; destructors remain noexcept.
    void close();
    std::uint32_t tryClose(r28::Status&) noexcept;
    std::vector<std::string> registeredFeatures() const;
};
}
