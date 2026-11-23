#ifndef PERCEP_DEMO_DETECTION_CLASS_FILTER_HPP_
#define PERCEP_DEMO_DETECTION_CLASS_FILTER_HPP_

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "percep_demo/seg_class_names.hpp"

namespace ai_node_percep_demo {
namespace detail {

// Resolved once while the detection model is initialized. Keeping the
// business filter as class IDs lets post-processing ignore unrelated logits
// before candidate creation, NMS, and the final top-k limit.
struct DetectionClassFilter {
  bool enabled{false};
  std::vector<int> active_class_ids;
  std::string error;

  explicit operator bool() const noexcept { return error.empty(); }
};

inline DetectionClassFilter ResolveDetectionClassFilter(
    const std::vector<std::string>& class_names,
    const std::string& keep_classes_csv) {
  DetectionClassFilter result;
  std::unordered_map<std::string, int> class_ids;
  class_ids.reserve(class_names.size());
  for (std::size_t index = 0; index < class_names.size(); ++index) {
    const std::string name = TrimCsvToken(class_names[index]);
    if (name.empty()) {
      result.error = "detection class_names contains an empty name at id " +
                     std::to_string(index);
      return result;
    }
    if (!class_ids.emplace(name, static_cast<int>(index)).second) {
      result.error = "detection class_names contains duplicate name: " + name;
      return result;
    }
  }
  if (class_ids.empty()) {
    result.error = "detection class_names is empty";
    return result;
  }

  if (TrimCsvToken(keep_classes_csv).empty()) {
    result.active_class_ids.resize(class_names.size());
    std::iota(result.active_class_ids.begin(),
              result.active_class_ids.end(), 0);
    return result;
  }

  result.enabled = true;
  std::unordered_set<int> seen_ids;
  std::size_t start = 0U;
  while (start <= keep_classes_csv.size()) {
    const std::size_t comma = keep_classes_csv.find(',', start);
    const std::string token = TrimCsvToken(
        comma == std::string::npos
            ? keep_classes_csv.substr(start)
            : keep_classes_csv.substr(start, comma - start));
    if (token.empty()) {
      result.error = "detection keep_classes contains an empty item";
      result.active_class_ids.clear();
      return result;
    }
    const auto found = class_ids.find(token);
    if (found == class_ids.end()) {
      result.error = "detection keep_classes contains unknown class: " + token;
      result.active_class_ids.clear();
      return result;
    }
    if (seen_ids.insert(found->second).second) {
      result.active_class_ids.push_back(found->second);
    }
    if (comma == std::string::npos) break;
    start = comma + 1U;
  }

  // Preserve the old argmax tie behavior: the lower tensor class ID wins.
  std::sort(result.active_class_ids.begin(), result.active_class_ids.end());
  return result;
}

// LogitReader signature: bool(int class_id, float* logit). Only active class
// IDs are requested, so an unrelated class can neither win the cell argmax
// nor consume candidate/NMS/top-k capacity later in the pipeline.
template <typename LogitReader>
inline bool SelectBestActiveDetectionClass(
    const std::vector<int>& active_class_ids, LogitReader&& read_logit,
    int* best_class_id, float* best_logit) {
  if (active_class_ids.empty() || best_class_id == nullptr ||
      best_logit == nullptr) {
    return false;
  }

  int selected_id = active_class_ids.front();
  float selected_logit = 0.0F;
  if (!read_logit(selected_id, &selected_logit)) return false;
  for (std::size_t index = 1U; index < active_class_ids.size(); ++index) {
    const int class_id = active_class_ids[index];
    float logit = 0.0F;
    if (!read_logit(class_id, &logit)) return false;
    if (logit > selected_logit) {
      selected_id = class_id;
      selected_logit = logit;
    }
  }
  *best_class_id = selected_id;
  *best_logit = selected_logit;
  return true;
}

}  // namespace detail
}  // namespace ai_node_percep_demo

#endif  // PERCEP_DEMO_DETECTION_CLASS_FILTER_HPP_
