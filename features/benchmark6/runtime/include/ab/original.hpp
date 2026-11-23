#pragma once
#include <memory>
namespace ab {
struct Config;
struct Frame;
struct Output;
class OriginalInference {
    struct Impl;
    std::unique_ptr<Impl> p_;

  public:
    explicit OriginalInference(const Config &);
    ~OriginalInference();
    void process(const Frame &, Output &);
};
} // namespace ab
