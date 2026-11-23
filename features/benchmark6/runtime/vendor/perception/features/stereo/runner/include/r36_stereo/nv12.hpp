#pragma once

#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>

namespace r36stereo {

// Packed NV12 is represented by a CV_8UC1 Mat with H*3/2 rows and W columns.
// The first H rows are Y; the remaining H/2 rows contain interleaved U,V.
// A Mat may have row padding. These helpers never convert YUV through RGB.
cv::Size nv12_size(const cv::Mat& image);

// Copies a padded NV12 or I420 descriptor into an owned tight NV12 Mat.
// I420 U/V each use stride/2, matching the project's psdk_frame contract.
cv::Mat copy_yuv420_to_nv12(const std::uint8_t* data, int width, int height,
                          std::size_t stride, std::size_t bytes, bool is_nv12);

// Explicit stretch: resize Y as CV_8UC1 and UV as CV_8UC2 with INTER_LINEAR.
// A matching size returns a shared Mat view, retaining any owning Mat storage.
cv::Mat resize_nv12(const cv::Mat& image, cv::Size target);

// Strict bottom, full-width even ROI. Copies Y rows and their matching UV rows
// into owned tight NV12 storage without interpolation or color conversion.
// Padding outside each visible source row is never copied.
cv::Mat crop_nv12(const cv::Mat& image, cv::Rect source_roi);

// For decoded PNG/JPEG inputs only: BGR -> BT.601 limited I420 -> UV interleave.
// Raw camera/ROS/recording NV12 should use copy_yuv420_to_nv12 directly.
cv::Mat bgr_to_nv12(const cv::Mat& image);

}  // namespace r36stereo
