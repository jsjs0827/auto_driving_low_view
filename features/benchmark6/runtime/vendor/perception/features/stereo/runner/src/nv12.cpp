#include "r36_stereo/nv12.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace r36stereo {
namespace {
void geometry(int width, int height) {
  if (width < 2 || height < 2 || width > 8192 || height > 8192 || ((width | height) & 1))
    throw std::invalid_argument("NV12 requires even width/height in [2,8192]");
}
}  // namespace

cv::Size nv12_size(const cv::Mat& image) {
  if (image.empty() || image.type() != CV_8UC1 || image.rows % 3 != 0)
    throw std::invalid_argument("NV12 Mat must be CV_8UC1 with H*3/2 rows");
  const int height = image.rows / 3 * 2;
  geometry(image.cols, height);
  if (image.step < static_cast<std::size_t>(image.cols))
    throw std::invalid_argument("NV12 Mat row stride is shorter than width");
  return {image.cols, height};
}

cv::Mat copy_yuv420_to_nv12(const std::uint8_t* data, int width, int height,
                          std::size_t stride, std::size_t bytes, bool is_nv12) {
  geometry(width, height);
  const auto rows = static_cast<std::size_t>(height + height / 2);
  if (!data || stride < static_cast<std::size_t>(width) || (stride & 1) ||
      stride > std::numeric_limits<std::size_t>::max() / rows || bytes < stride * rows)
    throw std::invalid_argument("YUV420 input has null/short storage or invalid even row stride");
  cv::Mat result(height + height / 2, width, CV_8UC1);
  for (int y = 0; y < height; ++y)
    std::memcpy(result.ptr(y), data + static_cast<std::size_t>(y) * stride, width);
  const auto* chroma = data + stride * height;
  if (is_nv12) {
    for (int y = 0; y < height / 2; ++y)
      std::memcpy(result.ptr(height + y), chroma + static_cast<std::size_t>(y) * stride, width);
  } else {
    const auto chroma_stride = stride / 2;
    const auto* u = chroma;
    const auto* v = u + chroma_stride * (height / 2);
    for (int y = 0; y < height / 2; ++y) {
      auto* row = result.ptr<std::uint8_t>(height + y);
      for (int x = 0; x < width / 2; ++x) {
        row[2 * x] = u[static_cast<std::size_t>(y) * chroma_stride + x];
        row[2 * x + 1] = v[static_cast<std::size_t>(y) * chroma_stride + x];
      }
    }
  }
  return result;
}

cv::Mat resize_nv12(const cv::Mat& image, cv::Size target) {
  const auto source = nv12_size(image);
  geometry(target.width, target.height);
  if (source == target) return image;
  cv::Mat result(target.height + target.height / 2, target.width, CV_8UC1);
  const cv::Mat y(source.height, source.width, CV_8UC1, image.data, image.step);
  const cv::Mat uv(source.height / 2, source.width / 2, CV_8UC2,
                   image.data + image.step * source.height, image.step);
  cv::Mat out_y(target.height, target.width, CV_8UC1, result.data, result.step);
  cv::Mat out_uv(target.height / 2, target.width / 2, CV_8UC2,
                 result.data + result.step * target.height, result.step);
  cv::resize(y, out_y, out_y.size(), 0, 0, cv::INTER_LINEAR);
  cv::resize(uv, out_uv, out_uv.size(), 0, 0, cv::INTER_LINEAR);
  return result;
}

cv::Mat crop_nv12(const cv::Mat& image, cv::Rect roi) {
  const auto source = nv12_size(image);
  geometry(roi.width, roi.height);
  if (roi.x != 0 || roi.width != source.width || roi.height > source.height ||
      roi.y < 0 || (roi.y & 1) || roi.y != source.height - roi.height)
    throw std::invalid_argument("NV12 crop must be an even bottom full-width ROI inside the source");
  cv::Mat result(roi.height + roi.height / 2, roi.width, CV_8UC1);
  for (int y = 0; y < roi.height; ++y)
    std::memcpy(result.ptr(y), image.ptr(roi.y + y), roi.width);
  for (int y = 0; y < roi.height / 2; ++y)
    std::memcpy(result.ptr(roi.height + y), image.ptr(source.height + roi.y / 2 + y), roi.width);
  return result;
}

cv::Mat bgr_to_nv12(const cv::Mat& image) {
  if (image.empty() || image.type() != CV_8UC3)
    throw std::invalid_argument("BGR input must be a nonempty CV_8UC3 Mat");
  geometry(image.cols, image.rows);
  cv::Mat i420;
  cv::cvtColor(image, i420, cv::COLOR_BGR2YUV_I420);
  return copy_yuv420_to_nv12(i420.data, image.cols, image.rows, i420.step,
                            i420.total(), false);
}
}  // namespace r36stereo
