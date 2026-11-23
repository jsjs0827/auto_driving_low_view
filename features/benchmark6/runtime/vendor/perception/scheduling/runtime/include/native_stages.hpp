#pragma once
#include "perception/types.hpp"
#include "percep_demo/horizon_dnn_runner.hpp"
#include <memory>
namespace r25 {
using namespace ai_node_percep_demo;
// Stage boundary used by registered Features. No scheduler or worker threads.
class Stages {
public:
    virtual ~Stages()=default;
    virtual void preprocess(int lane,const perception::ImageView&)=0;
    virtual void infer(int lane,DnnInferTiming&)=0;
    virtual std::vector<DnnDetection> detection()=0;
    virtual std::vector<std::uint8_t> segmentation()=0;
    virtual int width(int lane) const=0;
    virtual int height(int lane) const=0;
    virtual void abortFrame() noexcept=0;
    virtual void close() {}
};
class NativeStages final: public Stages {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    explicit NativeStages(const DnnConfig&);
    ~NativeStages() override;
    void preprocess(int,const perception::ImageView&) override;
    void infer(int,DnnInferTiming&) override;
    std::vector<DnnDetection> detection() override;
    std::vector<std::uint8_t> segmentation() override;
    int width(int) const override;
    int height(int) const override;
    void abortFrame() noexcept override;
    void close() override;
};
}
