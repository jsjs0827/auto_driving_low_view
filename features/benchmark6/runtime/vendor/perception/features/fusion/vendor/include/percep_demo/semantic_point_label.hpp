#ifndef PERCEP_DEMO_SEMANTIC_POINT_LABEL_HPP_
#define PERCEP_DEMO_SEMANTIC_POINT_LABEL_HPP_

// Point-cloud labels share one integer field, so their source domains must not
// overlap. Detection models are bounded to 256 classes by the runner. Reserve
// a separate 256-value range for segmentation and keep -1 for unknown.
inline constexpr int kAiClassIdLimit = 256;
inline constexpr int kAiSegLabelBase = 1024;

inline bool IsValidAiClassId(int class_id) {
  return class_id >= 0 && class_id < kAiClassIdLimit;
}

inline int EncodeDetLabel(int det_class_id) {
  return IsValidAiClassId(det_class_id) ? det_class_id : -1;
}

inline int EncodeSegLabel(int seg_class_id) {
  return IsValidAiClassId(seg_class_id) ? kAiSegLabelBase + seg_class_id : -1;
}

inline bool IsDetPointLabel(int label) {
  return label >= 0 && label < kAiClassIdLimit;
}

inline bool IsSegPointLabel(int label) {
  return label >= kAiSegLabelBase &&
         label < kAiSegLabelBase + kAiClassIdLimit;
}

inline int DecodeSegClassId(int label) {
  return IsSegPointLabel(label) ? label - kAiSegLabelBase : -1;
}

#endif  // PERCEP_DEMO_SEMANTIC_POINT_LABEL_HPP_
