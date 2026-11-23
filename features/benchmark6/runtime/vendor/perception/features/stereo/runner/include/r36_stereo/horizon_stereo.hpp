#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>

namespace r36stereo {

// The model must implement left-minus-right horizontal disparity, in pixels
// at its input width. Calibration, rectification and depth remain downstream.
class HorizonStereoModel {
 public:
  // require_nv12 rejects RGB/F32 models at startup instead of silently
  // performing a host-side colour conversion or guessing normalization.
  explicit HorizonStereoModel(const std::string& model, bool require_nv12 = false);
  ~HorizonStereoModel();
  HorizonStereoModel(const HorizonStereoModel&) = delete;
  HorizonStereoModel& operator=(const HorizonStereoModel&) = delete;

  cv::Size input_size() const;
  bool supports_nv12() const;
  // Direct camera/ROS/recording path. Mat is CV_8UC1, rows=H*3/2, cols=W,
  // with Y followed by interleaved UV. Source row padding is supported.
  // Both images must already equal input_size(); no hidden resize or /255.
  cv::Mat infer_nv12(const cv::Mat& left_nv12, const cv::Mat& right_nv12);
  // Requires two CV_8UC3 BGR images of exactly input_size(). No hidden resize.
  // Returns an owned CV_32FC1 matrix. Nonfinite/nonpositive predictions are
  // preserved here; downstream code explicitly converts invalid values to Q8=0.
  cv::Mat infer(const cv::Mat& left_bgr, const cv::Mat& right_bgr);
  std::string contract_json() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace r36stereo
