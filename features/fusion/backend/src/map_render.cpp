#include "mapping_algorithms.hpp"
#include "r33/mapping.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
namespace r33 {
namespace {
// Match OpenCV 4.5.4 INTER_NEAREST's double division/reciprocal order.
// This TU otherwise uses fast-math for the original mapping kernels; keep
// these coordinates strict, including display scales such as 49 where
// floor(x * (1.0 / scale)) can differ from integer x / scale.
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((noinline, optimize("no-fast-math", "fp-contract=off")))
#endif
std::vector<int> nearestAxisIndices(int source_size, int output_size) {
  const double scale = static_cast<double>(output_size) / source_size;
  const double inverse_scale = 1.0 / scale;
  std::vector<int> indices(output_size);
  for (int x = 0; x < output_size; ++x)
    indices[x] = std::min(static_cast<int>(std::floor(x * inverse_scale)),
                          source_size - 1);
  return indices;
}

cv::Mat resizeNearestBgrToRgbSerial(const cv::Mat& source, int width, int height) {
  const auto x_indices = nearestAxisIndices(source.cols, width);
  const auto y_indices = nearestAxisIndices(source.rows, height);
  cv::Mat rgb(height, width, CV_8UC3);
  const auto row_bytes = static_cast<std::size_t>(width) * 3;
  for (int y = 0; y < height; ++y) {
    auto* target = rgb.ptr<std::uint8_t>(y);
    if (y > 0 && y_indices[y] == y_indices[y - 1]) {
      std::memcpy(target, rgb.ptr<std::uint8_t>(y - 1), row_bytes);
      continue;
    }
    const auto* input = source.ptr<std::uint8_t>(y_indices[y]);
    for (int x = 0; x < width; ++x) {
      const auto* bgr = input + x_indices[x] * 3;
      target[x * 3] = bgr[2];
      target[x * 3 + 1] = bgr[1];
      target[x * 3 + 2] = bgr[0];
    }
  }
  return rgb;
}
}  // namespace
MappingRgbImage renderOriginalMap(const std::vector<MapPoint>& grid_map_save,
    double MapWidth, double MapHeight, double MapResolution, double x_cali, double y_cali) {
  using namespace ai_node_percep_demo;
  const auto geometry = detail::TryMakeGridGeometry(MapWidth, MapHeight, MapResolution);
  if (!geometry) throw std::invalid_argument("invalid map render geometry");
  const auto grid_geometry_ = *geometry;
  const auto visualization_geometry_ = detail::TryMakeGridVisualizationGeometry(grid_geometry_);
  if (!visualization_geometry_) throw std::invalid_argument("invalid map visualization size");
  const int pixel_width = visualization_geometry_->source_width;
  const int pixel_height = visualization_geometry_->source_height;
  const int rows = pixel_height, cols = pixel_width;
  const cv::Vec3b green(0,255,0), red(0,0,255), black(0,0,0), white(255,255,255);
  const cv::Vec3b yellow(0,255,255), cyan(255,255,0), magenta(255,0,255);
  const cv::Vec3b blue(255,0,0), orange(0,140,255);
  auto baseToPixel = [&](float base_x, float base_y) {
    int pixel_x = (-base_y)/MapResolution + pixel_width/2;
    int pixel_y = (-base_x)/MapResolution + pixel_height;
    return cv::Point(pixel_x,pixel_y);
  };
            if(grid_map_save.size() != grid_geometry_.cell_count ||
               rows != pixel_height || cols != pixel_width ||
               !visualization_geometry_.has_value()) { throw std::invalid_argument("map render geometry mismatch");}
            //opencv可视化
            cv::Mat image(pixel_height, pixel_width, CV_8UC3); 
            image.setTo(cv::Scalar(255, 255, 255));
            //坐标转换：
            //地图坐标系:x向右，y向上，原点在左下角
            //点云坐标系:x向右，y向下，原点左上角
            for (std::size_t i = 0; i < grid_map_save.size(); ++i)
            {
                int map_y = static_cast<int>(i / static_cast<std::size_t>(cols)); // 行索引，Y坐标 (0 ~ rows-1)
                int map_x = static_cast<int>(i % static_cast<std::size_t>(cols)); // 列索引，X坐标 (0 ~ cols-1)
                int x = map_x ;
                int y = -map_y + pixel_height -1;  
                if (x>=0 && x<pixel_width && y>=0 && y < pixel_height)
                {
                    cv::Vec3b* pixel_ptr = image.ptr<cv::Vec3b>(y) + x;
                    int label_index = grid_map_save[i].type;

                    if (label_index == -1){*pixel_ptr = blue; } // V0 slope palette
                    if (label_index == 0){*pixel_ptr = yellow; } //未知
                    if (label_index == 1 ){*pixel_ptr = black; } //非草
                    if (label_index == 2){*pixel_ptr = red; } //障碍物
                    if (label_index == 3){*pixel_ptr = green; } //草
                    if (label_index == 4) {*pixel_ptr = magenta;}  //人
                    if (label_index == 5) {*pixel_ptr = cyan;}  //猫
                    if (label_index == 6) {*pixel_ptr = white;}  //狗
                    if (label_index == 8) {*pixel_ptr = orange;}  // V0 toy palette
                }
                
            }

            //绘制base_link坐标系
            float axis_length = 0.1f ;
            const cv::Vec3b x_color = cv::Vec3b(0, 0, 255);
            const cv::Vec3b y_color = cv::Vec3b(0, 255, 0);
            cv::Point x_start = baseToPixel(0, 0);
            cv::Point x_end = baseToPixel(axis_length, 0);
            cv::arrowedLine(image, x_start, x_end, cv::Scalar(x_color[0], x_color[1], x_color[2]), 1);
            cv::Point y_start = baseToPixel(0, 0);
            cv::Point y_end = baseToPixel(0, axis_length);
            cv::arrowedLine(image, y_start, y_end, cv::Scalar(y_color[0], y_color[1], y_color[2]), 1);
            cv::circle(image, baseToPixel(0, 0), 1, cv::Scalar(255, 0, 0), -1);

            // 绘制相机坐标系
            float new_origin_x = x_cali;
            float new_origin_y = y_cali;
            cv::Point new_x_start = baseToPixel(new_origin_x, new_origin_y);
            cv::Point new_x_end = baseToPixel(new_origin_x + axis_length, new_origin_y);
            cv::arrowedLine(image, new_x_start, new_x_end, 
                            cv::Scalar(x_color[0], x_color[1], x_color[2]), 1);
            cv::Point new_y_start = baseToPixel(new_origin_x, new_origin_y);
            cv::Point new_y_end = baseToPixel(new_origin_x, new_origin_y + axis_length);
            cv::arrowedLine(image, new_y_start, new_y_end, 
                            cv::Scalar(y_color[0], y_color[1], y_color[2]), 1);
            cv::circle(image, new_x_start, 1, cv::Scalar(255, 0, 0), -1);

            const int grid_step = visualization_geometry_->grid_step_px;
            const int output_width = visualization_geometry_->output_width;
            const int output_height = visualization_geometry_->output_height;
            // RGB conversion is fused into serial nearest-neighbor expansion.
            // The grid is neutral gray, so drawing it after channel exchange
            // is byte-identical to drawing in BGR and then cvtColor(BGR2RGB).
            cv::Mat resized_map = resizeNearestBgrToRgbSerial(
                image, output_width, output_height);
            int line_width = 1;
            cv::Scalar grid_color(230, 230, 230);
 
            for (int x = 0; x < output_width; x += grid_step) {
                cv::line(resized_map, 
                        cv::Point(x, 0), 
                        cv::Point(x, output_height - 1), 
                        grid_color, 
                        line_width);
            }
            
            for (int y = 0; y < output_height; y += grid_step) {
                cv::line(resized_map, 
                        cv::Point(0, y), 
                        cv::Point(output_width - 1, y), 
                        grid_color, 
                        line_width);
            }

  MappingRgbImage image_out;
  image_out.width = resized_map.cols;
  image_out.height = resized_map.rows;
  image_out.pixels.assign(resized_map.data,
                         resized_map.data + resized_map.total()*resized_map.elemSize());
  return image_out;
}
}
