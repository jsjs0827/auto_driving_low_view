#pragma once
#include "json_util.hpp"
#include "perception_compare/native_compare.hpp"
#include <cstdint>
#include <vector>

namespace perception_compare::detail {
struct ComparisonRoi {
  int x=0,y=0,width=0,height=0,source_width=0,source_height=0;
};
ComparisonRoi comparisonRoi(const std::string& mode,int model_width,int model_height,
                            const Intrinsics& source);
Json comparisonRoiJson(const ComparisonRoi& roi,const Intrinsics& source);
Json comparisonRoiIdentity(const Json& run);
// Pure same-position slicing/statistics. Zero Q8 is invalid; original is a
// reference, never ground truth. Pixel units are Q8/256 with no resize.
std::vector<std::uint16_t> sliceDisparity(const std::vector<std::uint16_t>& source,
                                        const ComparisonRoi& roi);
Json compareDisparity(const std::vector<std::uint16_t>& reference,
                      const std::vector<std::uint16_t>& candidate);
// Runs only after runner completion. Never contributes to CPU/BPU/FPS windows.
Json generateDisparityComparison(const fs::path& root,const Json& metadata);
} // namespace perception_compare::detail
