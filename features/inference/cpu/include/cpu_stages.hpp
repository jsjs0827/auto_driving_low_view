#pragma once
#include "native_stages.hpp"
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace r29 {
// CPU tensor conversion is declared independently of a quantized board BIN.
// RGB/BGR uint8 -> (channel * scale - mean[channel]) / stddev[channel].
struct CpuInputConfig {
    std::string name,color,yuv;
    float scale=0;
    std::array<float,3> mean{},stddev{};
};
struct CpuOutputConfig {
    std::string format,layout,name;
    std::vector<std::string> names;
    int reg=16;
};
struct CpuLaneConfig {
    int width=0,height=0,classes=0;
    std::vector<std::string> class_names;
    CpuInputConfig input;
    CpuOutputConfig output;
};
struct CpuConfig {
    ai_node_percep_demo::DnnConfig dnn;
    CpuLaneConfig detection,segmentation;
    int intra_threads=1,inter_threads=1;
};
CpuConfig loadCpuConfig(const std::string& path);
void validateCpuConfig(const CpuConfig&);

// Two real ONNX Runtime CPU sessions plug into the existing registered Features.
// The DAG owns concurrency; this adapter adds no frame queue or worker pipeline.
class CpuStages final: public r25::Stages {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    explicit CpuStages(const CpuConfig&);
    ~CpuStages() override;
    void preprocess(int,const perception::ImageView&) override;
    void infer(int,ai_node_percep_demo::DnnInferTiming&) override;
    std::vector<ai_node_percep_demo::DnnDetection> detection() override;
    std::vector<std::uint8_t> segmentation() override;
    int width(int) const override;
    int height(int) const override;
    void abortFrame() noexcept override;
    void close() override;
    std::string metadataJson() const;
    double lastRunMs(int lane) const;
};
}
