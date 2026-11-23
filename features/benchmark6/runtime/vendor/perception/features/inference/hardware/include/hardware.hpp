#pragma once
#include "perception/types.hpp"
#include <memory>
#include <string>
#include <vector>

class VseResize {
  public:
    VseResize(int source_w,int source_h,int target_w,int target_h,int timeout_ms);
    ~VseResize();
    VseResize(const VseResize&)=delete;
    VseResize& operator=(const VseResize&)=delete;
    const std::vector<std::uint8_t>& run(const std::vector<std::uint8_t>&);
    const std::vector<std::uint8_t>& run(const std::uint8_t*,std::size_t bytes,std::size_t stride);
    std::string infoJson() const;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// A separately compiled Resize->ArgMax graph with a float32 featuremap input.
// Its actual CPU/BPU placement must be checked in the compiler report.
class CompiledPostModel {
  public:
    CompiledPostModel(const std::string& model,const std::string& name,
                      int classes,int logits_h,int logits_w,int mask_h,int mask_w,int timeout_ms);
    ~CompiledPostModel();
    CompiledPostModel(const CompiledPostModel&)=delete;
    CompiledPostModel& operator=(const CompiledPostModel&)=delete;
    void run(const perception::Tensor&,std::vector<std::uint8_t>& mask);
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// One NV12 model call, with Resize->ArgMax fused after PIDNet.
// Diagnostic builds additionally expose low-resolution logits, outside performance runs.
struct FusedTimings {
    double input_copy_ms=0, input_clean_ms=0, model_wait_ms=0;
    double mask_invalidate_ms=0, mask_extract_ms=0, diagnostic_extract_ms=0;
};
class FusedNv12Model {
  public:
    FusedNv12Model(const std::string&,int width,int height,int mask_width,int mask_height,bool diagnostic,int timeout_ms,
                   bool optimized_labels=true);
    ~FusedNv12Model();
    FusedNv12Model(const FusedNv12Model&)=delete;
    FusedNv12Model& operator=(const FusedNv12Model&)=delete;
    void run(const std::uint8_t*,std::size_t,std::vector<std::uint8_t>&,perception::Tensor* diagnostic_logits=nullptr,
             bool verify_label_reader=false,bool capture_storage=false);
    const std::vector<std::uint8_t>& capturedInputStorage() const;
    const std::vector<std::uint8_t>& capturedMaskStorage() const;
    const FusedTimings& timings() const;
    std::string infoJson() const;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
