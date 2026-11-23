#ifndef PERCEP_DEMO_DETECTION_CANDIDATE_BUDGET_HPP_
#define PERCEP_DEMO_DETECTION_CANDIDATE_BUDGET_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <vector>

namespace ai_node_percep_demo {
namespace detail {

struct DetectionCandidateRank {
  int class_id{-1};
  float confidence{0.0F};
  std::size_t source_index{0U};
};

inline bool HigherCandidateRank(const DetectionCandidateRank& lhs,
                                const DetectionCandidateRank& rhs) noexcept {
  if (lhs.confidence != rhs.confidence) {
    return lhs.confidence > rhs.confidence;
  }
  return lhs.source_index < rhs.source_index;
}

// Select a bounded NMS input without allowing one frequent class to evict all
// candidates from another class. Half of the budget is reserved evenly across
// active classes; unused and remaining capacity is then filled globally by
// confidence. The final NMS still owns suppression and global top-k ordering.
inline std::vector<std::size_t> SelectClassAwareCandidateIndices(
    const std::vector<DetectionCandidateRank>& input,
    std::size_t limit) {
  std::map<int, std::vector<DetectionCandidateRank>> by_class;
  for (const auto& candidate : input) {
    if (candidate.class_id < 0 || !std::isfinite(candidate.confidence)) {
      continue;
    }
    by_class[candidate.class_id].push_back(candidate);
  }
  if (limit == 0U || by_class.empty()) return {};

  for (auto& entry : by_class) {
    std::sort(entry.second.begin(), entry.second.end(), HigherCandidateRank);
  }

  std::vector<std::size_t> selected;
  selected.reserve(std::min(limit, input.size()));
  if (by_class.size() > limit) {
    std::vector<DetectionCandidateRank> representatives;
    representatives.reserve(by_class.size());
    for (const auto& entry : by_class) representatives.push_back(entry.second[0]);
    std::sort(representatives.begin(), representatives.end(),
              HigherCandidateRank);
    for (std::size_t index = 0; index < limit; ++index) {
      selected.push_back(representatives[index].source_index);
    }
    return selected;
  }

  const std::size_t reserved_budget = std::max<std::size_t>(1U, limit / 2U);
  const std::size_t reserve_per_class =
      std::max<std::size_t>(1U, reserved_budget / by_class.size());
  std::vector<DetectionCandidateRank> remainder;
  remainder.reserve(input.size());
  for (const auto& entry : by_class) {
    const auto& group = entry.second;
    const std::size_t reserved = std::min(reserve_per_class, group.size());
    for (std::size_t index = 0; index < reserved; ++index) {
      selected.push_back(group[index].source_index);
    }
    remainder.insert(remainder.end(), group.begin() + reserved, group.end());
  }

  std::sort(remainder.begin(), remainder.end(), HigherCandidateRank);
  const std::size_t remaining_capacity = limit - selected.size();
  const std::size_t append_count =
      std::min(remaining_capacity, remainder.size());
  for (std::size_t index = 0; index < append_count; ++index) {
    selected.push_back(remainder[index].source_index);
  }
  return selected;
}

}  // namespace detail
}  // namespace ai_node_percep_demo

#endif  // PERCEP_DEMO_DETECTION_CANDIDATE_BUDGET_HPP_
