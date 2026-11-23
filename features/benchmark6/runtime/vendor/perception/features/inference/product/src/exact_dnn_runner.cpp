#include "product/exact_dnn_runner.hpp"
#include "dag_pipeline.hpp"
#include "native_stages.hpp"
#include <algorithm>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>

#ifndef PERCEPTION_PRODUCT_EXACT_PROFILES
#define PERCEPTION_PRODUCT_EXACT_PROFILES "/app/daye/share/perception_x5/profiles"
#endif
namespace r28::product {
struct ExactDnnRunner::Impl {
    std::string profiles;
    ai_node_percep_demo::DnnConfig config;
    std::shared_ptr<r25::NativeStages> stages;
    std::map<std::string, std::unique_ptr<r25::Pipeline>> pipelines;
    r28::Status status;
    perception::FrameInfo frame;
};
ExactDnnRunner::ExactDnnRunner(std::string profiles) : impl_(std::make_unique<Impl>()) {
    impl_->profiles=profiles.empty()?PERCEPTION_PRODUCT_EXACT_PROFILES:std::move(profiles);
}
ExactDnnRunner::~ExactDnnRunner() = default;
bool ExactDnnRunner::init(const ai_node_percep_demo::DnnConfig& config) {
    try {
        impl_->pipelines.clear();impl_->stages.reset();
        if (!config.enabled || (!config.detect.enabled && !config.seg.enabled))
            throw std::invalid_argument("full product exact adapter requires at least one enabled model");
        std::set<std::string> profiles{selectExactLanes(config, config.seg.enabled).profile};
        if (config.detect.enabled) profiles.insert(selectExactLanes(config, false).profile);
        for (const auto& profile : profiles)
            if (!std::filesystem::is_regular_file(std::filesystem::path(impl_->profiles)/profile))
                throw std::invalid_argument("full product exact DAG profile is missing: "+profile);
        impl_->config=config;
        auto names=config.seg.class_names;
        if (impl_->config.seg.num_classes<=0)
            impl_->config.seg.num_classes=names.empty()?0:1+static_cast<int>(std::count(names.begin(),names.end(),','));
        impl_->stages=std::make_shared<r25::NativeStages>(config);
        for (const auto& profile : profiles)
            impl_->pipelines.emplace(profile, std::make_unique<r25::Pipeline>(
                (std::filesystem::path(impl_->profiles)/profile).string(), impl_->stages));
        impl_->status={};
        return true;
    } catch (const std::exception& e) {
        impl_->pipelines.clear();impl_->stages.reset();
        impl_->status=r28::statusFromException(e,1005,"product.ai.exact.init");
        return false;
    }
}
bool ExactDnnRunner::inferNv12(const std::uint8_t* data,int width,int height,int stride,
    std::size_t bytes,bool nv12,std::vector<ai_node_percep_demo::DnnDetection>* detections,
    std::vector<std::uint8_t>* mask,std::vector<std::uint8_t>* overlay,
    ai_node_percep_demo::DnnInferTiming* timing) {
    if (detections) detections->clear();
    if (mask) mask->clear();
    if (overlay) overlay->clear();
    if (timing) *timing={};
    try {
        if (impl_->pipelines.empty()) throw std::logic_error("exact product models not initialized");
        if (!detections && !mask) throw std::invalid_argument("exact product adapter requires a result output");
        if (overlay) throw std::invalid_argument("product overlay must use the original dnn_runtime renderer");
        validateProductYuv(data, width, height, stride, bytes, impl_->frame);
        const auto lanes=selectExactLanes(impl_->config, mask!=nullptr);
        perception::ImageView image{data,bytes,static_cast<std::size_t>(stride),width,height,
            nv12?perception::PixelFormat::NV12:perception::PixelFormat::I420};
        r25::RunOptions options; options.optimized=true;
        options.run_detection=lanes.detection; options.run_segmentation=lanes.segmentation;
        r25::Output output;
        const auto code=impl_->pipelines.at(lanes.profile)->tryRun(image,impl_->frame,options,output,impl_->status);
        if (code) return false;
        if (detections) *detections=std::move(output.det);
        if (mask) *mask=output.mask();
        if (timing) *timing=output.timing;
        return true;
    } catch (const std::exception& e) {
        impl_->status=r28::statusFromException(e,1011,"product.ai.exact", "dnn",impl_->frame);
        return false;
    }
}
const ai_node_percep_demo::DnnConfig& ExactDnnRunner::config() const { return impl_->config; }
const r28::Status& ExactDnnRunner::lastStatus() const { return impl_->status; }
void ExactDnnRunner::setFrameInfo(perception::FrameInfo frame) { impl_->frame=frame; }
}
