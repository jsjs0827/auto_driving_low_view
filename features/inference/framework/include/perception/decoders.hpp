#pragma once
#include "perception/model_plan.hpp"
namespace perception {
class YoloDflDecoder {
  public:
    static std::vector<Detection> decode(const std::vector<Tensor> &, const ModelPlan &,
                                         const Geometry &, std::size_t &rejected);
};
class SemanticDecoder {
  public:
    static std::vector<std::uint8_t> decode(const std::vector<Tensor> &, const ModelPlan &,
                                            const Geometry &);
};
} // namespace perception
