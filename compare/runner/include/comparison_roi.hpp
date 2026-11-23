#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace r36compare::runner {

inline void validateComparisonRoi(const cv::Rect& roi, cv::Size source) {
  if (source.width < 2 || source.height < 2 || source.width > 8192 || source.height > 8192 ||
      ((source.width | source.height) & 1) || roi.x < 0 || roi.y < 0 ||
      roi.width < 2 || roi.height < 2 || ((roi.x | roi.y | roi.width | roi.height) & 1) ||
      roi.x > source.width || roi.y > source.height ||
      roi.width > source.width - roi.x || roi.height > source.height - roi.y)
    throw std::invalid_argument("comparison ROI requires even x/y/width/height, positive size and containment in the unchanged source image");
}

inline cv::Rect parseComparisonRoi(const std::string& text, cv::Size source) {
  std::array<int,4> values{};
  std::size_t begin=0;
  for (std::size_t i=0;i<values.size();++i) {
    const auto end=text.find(',',begin);
    if ((i<3 && end==std::string::npos) || (i==3 && end!=std::string::npos))
      throw std::invalid_argument("--comparison-roi requires exactly x,y,width,height");
    const auto stop=end==std::string::npos?text.size():end;
    if (stop==begin || !std::all_of(text.begin()+begin,text.begin()+stop,
                                   [](unsigned char c){return c>='0'&&c<='9';}))
      throw std::invalid_argument("--comparison-roi requires four strict nonnegative decimal integers without whitespace/signs");
    const auto parsed=std::from_chars(text.data()+begin,text.data()+stop,values[i]);
    if (parsed.ec!=std::errc{} || parsed.ptr!=text.data()+stop)
      throw std::invalid_argument("--comparison-roi integer exceeds supported range");
    begin=stop+1;
  }
  const cv::Rect roi{values[0],values[1],values[2],values[3]};
  validateComparisonRoi(roi,source);
  return roi;
}

inline std::string comparisonRoiJson(const cv::Rect& roi, cv::Size source) {
  validateComparisonRoi(roi,source);
  return "{\"x\":"+std::to_string(roi.x)+",\"y\":"+std::to_string(roi.y)+
         ",\"width\":"+std::to_string(roi.width)+",\"height\":"+std::to_string(roi.height)+
         ",\"source_width\":"+std::to_string(source.width)+",\"source_height\":"+std::to_string(source.height)+
         ",\"coordinate_frame\":\"source_rectified_left\"}";
}

inline void requireMatchingCropComparisonRoi(const std::string& mode,
                                            const cv::Rect& comparison,
                                            const cv::Rect& inference) {
  if (mode=="crop" && comparison!=inference)
    throw std::invalid_argument("Crop inference ROI must equal --comparison-roi; pass the same explicit ROI to original and stereo methods");
}

// Keep full-frame Q8 storage/calibration; zero means invalid disparity.
// This operation belongs inside the runner's compute/CPU measurement interval.
inline void maskOutsideComparisonRoi(std::vector<std::uint16_t>& q8,
                                    cv::Size source,const cv::Rect& roi) {
  validateComparisonRoi(roi,source);
  if (q8.size()!=std::size_t(source.width)*source.height)
    throw std::invalid_argument("Original Q8 storage does not match full source dimensions");
  if (roi==cv::Rect(0,0,source.width,source.height)) return;
  for (int y=0;y<source.height;++y) {
    auto first=q8.begin()+std::size_t(y)*source.width;
    if (y<roi.y || y>=roi.y+roi.height) {
      std::fill(first,first+source.width,std::uint16_t(0));
    } else {
      std::fill(first,first+roi.x,std::uint16_t(0));
      std::fill(first+roi.x+roi.width,first+source.width,std::uint16_t(0));
    }
  }
}
}  // namespace r36compare::runner
