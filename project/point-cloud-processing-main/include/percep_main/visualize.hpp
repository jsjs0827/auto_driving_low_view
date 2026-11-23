#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <opencv2/opencv.hpp>
#include "percep_main/common.hpp"
#include "percep_main/horizon_dnn_runner.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <zlib.h>

namespace fs = filesystem;
// 基本颜色
const cv::Vec3b blue(255, 0, 0);        // 蓝色
const cv::Vec3b green(0, 255, 0);       // 绿色
const cv::Vec3b red(0, 0, 255);         // 红色
const cv::Vec3b black(0, 0, 0);         // 黑色
const cv::Vec3b white(255, 255, 255);   // 白色
const cv::Vec3b yellow(0, 255, 255);    // 黄色
const cv::Vec3b cyan(255, 255, 0);      // 青色
const cv::Vec3b magenta(255, 0, 255);   // 洋红色
const cv::Vec3b oringe(0, 140, 255);   //橙色
class Visualize :public PclCommon{

    public:
        Visualize() = default;
        Visualize(double width,double height,double resolution,string dir,double x,double y)
        {
            MapWidth = width;
            MapHeight = height;
            MapResolution = resolution;
            visualLog_dir = dir;

            x_cali = x;
            y_cali = y;
        }
        int pixel_width,pixel_height; //可视化地图png参数
        double MapWidth,MapHeight,MapResolution;
        string visualLog_dir;

        double x_cali, y_cali;
        //车体转像素坐标
        cv::Point baseToPixel(float base_x, float base_y) const {
            //像素坐标系：原点在左上角，x向右，y向下
            //车体坐标系：原点在中心，x向上，y向左
            int pixel_x = (-base_y)/MapResolution + pixel_width  / 2;
            int pixel_y = (-base_x)/MapResolution + pixel_height  ;
            return cv::Point(pixel_x,pixel_y);
        }

        bool savePCL(PointCloudT &cloud,string  timestamp)
        {
            fs::path dir(visualLog_dir+"/data/ori_cloud/");
            if (!dir.empty() && !fs::exists(dir)) {
                fs::create_directories(dir);
            }
            std::string filePath = visualLog_dir +"/data/ori_cloud/" + timestamp + ".bin"; // 修改为你的实际输出路径

            std::ofstream ofs(filePath, std::ios::binary);
            if (!ofs.is_open())
            {
                LOG_ERROR("[Visualize ERR] savePCL, pcl open failed: %s",filePath.c_str());
                return false;
            }

            // 先写入点数量（uint64，方便读取时解析）
            uint64_t point_num = static_cast<uint64_t>(cloud.size());
            ofs.write(reinterpret_cast<const char*>(&point_num), sizeof(uint64_t));

            for (const auto& pt : cloud.points)
            {
                // 逐个字段写出，避开结构体padding
                ofs.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
                ofs.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
                ofs.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));
                ofs.write(reinterpret_cast<const char*>(&pt.label), sizeof(int));
                ofs.write(reinterpret_cast<const char*>(&pt.confidence), sizeof(float));
                ofs.write(reinterpret_cast<const char*>(&pt.r), sizeof(uint8_t));
                ofs.write(reinterpret_cast<const char*>(&pt.g), sizeof(uint8_t));
                ofs.write(reinterpret_cast<const char*>(&pt.b), sizeof(uint8_t));
            }
            ofs.close();
            LOG_INFO("[Visualize]savePCL success! size= %d -> %s",point_num,filePath.c_str());
            return true;
        }
        bool saveDisp(const std::vector<uint32_t>& u_vec,
                      const std::vector<uint32_t>& v_vec,
                      const std::vector<float>& disp,
                      const std::string& timestamp)
        {
            if (disp.empty() || u_vec.size() != disp.size() ||
                v_vec.size() != disp.size()) {
                LOG_ERROR("[Visualize ERR] saveDisp invalid vectors: u=%llu v=%llu disp=%llu",
                          static_cast<unsigned long long>(u_vec.size()),
                          static_cast<unsigned long long>(v_vec.size()),
                          static_cast<unsigned long long>(disp.size()));
                return false;
            }

            fs::path dir(visualLog_dir + "/data/disp/");
            if (!fs::exists(dir)) fs::create_directories(dir);
            const fs::path path = dir / (timestamp + ".bin");
            const fs::path temp_path = dir / (timestamp + ".bin.tmp");
            std::ofstream ofs(temp_path, std::ios::binary | std::ios::trunc);
            if (!ofs.is_open()) {
                LOG_ERROR("[Visualize ERR] saveDisp open failed: %s",
                          temp_path.string().c_str());
                return false;
            }

            const uint64_t num = static_cast<uint64_t>(disp.size());
            ofs.write(reinterpret_cast<const char*>(&num), sizeof(num));
            ofs.write(reinterpret_cast<const char*>(u_vec.data()),
                      static_cast<std::streamsize>(num * sizeof(uint32_t)));
            ofs.write(reinterpret_cast<const char*>(v_vec.data()),
                      static_cast<std::streamsize>(num * sizeof(uint32_t)));
            ofs.write(reinterpret_cast<const char*>(disp.data()),
                      static_cast<std::streamsize>(num * sizeof(float)));
            ofs.close();
            if (!ofs.good()) {
                fs::remove(temp_path);
                LOG_ERROR("[Visualize ERR] saveDisp write failed: %s",
                          path.string().c_str());
                return false;
            }
            std::error_code ec;
            fs::rename(temp_path, path, ec);
            if (ec) {
                fs::remove(temp_path);
                LOG_ERROR("[Visualize ERR] saveDisp rename failed: %s",
                          ec.message().c_str());
                return false;
            }
            return true;
        }
        bool saveMapRaw(const std::vector<MapPoint>& points,
                        const std::string& map_type, const std::string& timestamp,
                        int rows, int cols)
        {
            if (rows <= 0 || cols <= 0 ||
                points.size() != static_cast<size_t>(rows) * cols) {
                LOG_ERROR("[Visualize ERR] saveMapRaw dimensions mismatch: %s",
                          map_type.c_str());
                return false;
            }
            fs::path dir(visualLog_dir + "/map/" + map_type);
            if (!fs::exists(dir)) fs::create_directories(dir);
            const fs::path path = dir / (timestamp + ".bin");
            std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
            if (!ofs.is_open()) return false;

            const uint32_t magic = 0x50414D52;
            const uint16_t version = 4;
            const uint16_t reserved = 1;
            const uint32_t raw_rows = static_cast<uint32_t>(rows);
            const uint32_t raw_cols = static_cast<uint32_t>(cols);
            const float resolution_m = static_cast<float>(MapResolution);
            const float width_m = static_cast<float>(MapWidth);
            const float height_m = static_cast<float>(MapHeight);
            uint64_t timestamp_ms = 0;
            try { timestamp_ms = std::stoull(timestamp); } catch (...) {}
            const uint64_t point_count = points.size();
            const float camera_x_m = static_cast<float>(x_cali);
            const float camera_y_m = static_cast<float>(y_cali);
            // 且类型大面积相同，zlib 后约 1~3KB。
            std::vector<uint8_t> raw_payload;
            raw_payload.reserve(points.size());
            for (const auto& point : points) {
                const int clamped =
                    std::max(-128, std::min(127, point.type));
                raw_payload.push_back(static_cast<uint8_t>(
                    static_cast<int8_t>(clamped)));
            }
            uLongf compressed_size = compressBound(static_cast<uLong>(raw_payload.size()));
            std::vector<uint8_t> compressed(compressed_size);
            if (compress2(compressed.data(), &compressed_size, raw_payload.data(),
                          static_cast<uLong>(raw_payload.size()), Z_BEST_SPEED) != Z_OK)
                return false;
            compressed.resize(compressed_size);
            const uint64_t payload_size = compressed.size();
            ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
            ofs.write(reinterpret_cast<const char*>(&reserved), sizeof(reserved));
            ofs.write(reinterpret_cast<const char*>(&raw_rows), sizeof(raw_rows));
            ofs.write(reinterpret_cast<const char*>(&raw_cols), sizeof(raw_cols));
            ofs.write(reinterpret_cast<const char*>(&resolution_m), sizeof(resolution_m));
            ofs.write(reinterpret_cast<const char*>(&width_m), sizeof(width_m));
            ofs.write(reinterpret_cast<const char*>(&height_m), sizeof(height_m));
            ofs.write(reinterpret_cast<const char*>(&timestamp_ms), sizeof(timestamp_ms));
            ofs.write(reinterpret_cast<const char*>(&point_count), sizeof(point_count));
            ofs.write(reinterpret_cast<const char*>(&camera_x_m), sizeof(camera_x_m));
            ofs.write(reinterpret_cast<const char*>(&camera_y_m), sizeof(camera_y_m));
            ofs.write(reinterpret_cast<const char*>(&payload_size), sizeof(payload_size));
            ofs.write(reinterpret_cast<const char*>(compressed.data()),
                      static_cast<std::streamsize>(compressed.size()));
            return ofs.good();
        }

        bool saveMap(vector<MapPoint> &grid_map_save,string  header,string  timestamp,int  rows,int  cols)
        {
            fs::path dir(visualLog_dir+"/map/"+header+"/");
            if (!dir.empty() && !fs::exists(dir)) {
                fs::create_directories(dir);
            }
            if(grid_map_save.empty()) { LOG_ERROR("[Visualize ERR]saveMap grid_map为空");return false;}
            //opencv可视化
            pixel_width = static_cast<int>(std::round(MapWidth / MapResolution));
            pixel_height = static_cast<int>(std::round(MapHeight / MapResolution));
            cv::Mat image(pixel_height, pixel_width, CV_8UC3); 
            image.setTo(cv::Scalar(255, 255, 255));
            //坐标转换：
            //地图坐标系:x向右，y向上，原点在左下角
            //点云坐标系:x向右，y向下，原点左上角
            for (int i=0;i<grid_map_save.size();i++)
            {
                int map_y = i / cols; // 行索引，Y坐标 (0 ~ rows-1)
                int map_x = i % cols; // 列索引，X坐标 (0 ~ cols-1)
                int x = map_x ;
                int y = -map_y + pixel_height -1;  
                if (x>=0 && x<pixel_width && y>=0 && y < pixel_height)
                {
                    cv::Vec3b* pixel_ptr = image.ptr<cv::Vec3b>(y) + x;
                    int label_index = grid_map_save[i].type;
                    if (label_index == -1){*pixel_ptr = blue; } //坡道
                    if (label_index == 0){*pixel_ptr = yellow; } //未知
                    if (label_index == 1 ){*pixel_ptr = black; } //非草
                    if (label_index == 2){*pixel_ptr = red; } //障碍物
                    if (label_index == 3){*pixel_ptr = green; } //草
                    if (label_index == 4) {*pixel_ptr = magenta;}  //人
                    if (label_index == 5) {*pixel_ptr = cyan;}  //猫
                    if (label_index == 6) {*pixel_ptr = white;}  //狗
                    if (label_index == 8) {*pixel_ptr = oringe;}  //玩具
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

            cv::Mat resized_map;
            float display_scale_ = MapResolution *100 *4;     
            cv::resize(image, resized_map, cv::Size(pixel_width * display_scale_, pixel_height * display_scale_),0, 0, cv::INTER_NEAREST); 
            int line_width = 1;
            cv::Scalar grid_color(230, 230, 230);
 
            for (int x = 0; x <=  pixel_width* display_scale_  ; x+=display_scale_ *2) {
                cv::line(resized_map, 
                        cv::Point(x, 0), 
                        cv::Point(x, pixel_height* display_scale_ - 1), 
                        grid_color, 
                        line_width);
            }
            
            for (int y = 0; y <= pixel_height* display_scale_ ; y+=display_scale_ *2) {
                cv::line(resized_map, 
                        cv::Point(0, y), 
                        cv::Point(pixel_width* display_scale_  - 1, y), 
                        grid_color, 
                        line_width);
            }
            if (!resized_map.empty()) {
                std::vector<int> png_params = {
                    cv::IMWRITE_PNG_COMPRESSION, 1
                };
                cv::imwrite(visualLog_dir+"/map/"+header+"/"+ timestamp+".png", resized_map, png_params);
                // cv::imwrite(visualLog_dir+header+"/"+ timestamp+".jpg", resized_map, {cv::IMWRITE_JPEG_QUALITY, 70});
            }else{
                LOG_ERROR("[Visualize ERR]resized_map为空");}

            return true;
        }

        bool saveEncodedBytes(const std::string& subdir, const std::string& format,
                              int width, int height, int channels,
                              const std::vector<uint8_t>& data,
                              const std::string& timestamp)
        {
            fs::path dir(visualLog_dir + "/data/" + subdir);
            if (!fs::exists(dir)) fs::create_directories(dir);
            std::ofstream ofs((dir / (timestamp + ".bin")).string(),
                              std::ios::binary | std::ios::trunc);
            if (!ofs.is_open()) return false;
            const uint32_t magic = 0x52415731;
            const uint32_t version = 3;
            const uint32_t w = static_cast<uint32_t>(width);
            const uint32_t h = static_cast<uint32_t>(height);
            const uint32_t c = static_cast<uint32_t>(channels);
            const uint32_t format_size = static_cast<uint32_t>(format.size());
            const uint64_t data_size = data.size();
            ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
            ofs.write(reinterpret_cast<const char*>(&w), sizeof(w));
            ofs.write(reinterpret_cast<const char*>(&h), sizeof(h));
            ofs.write(reinterpret_cast<const char*>(&c), sizeof(c));
            ofs.write(reinterpret_cast<const char*>(&format_size), sizeof(format_size));
            ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
            ofs.write(format.data(), static_cast<std::streamsize>(format.size()));
            ofs.write(reinterpret_cast<const char*>(data.data()),
                      static_cast<std::streamsize>(data.size()));
            return ofs.good();
        }

        bool saveRawBytes(const std::string& subdir, const std::string& format,
                          int width, int height, int channels,
                          const std::vector<uint8_t>& data,
                          const std::string& timestamp)
        {
            fs::path dir(visualLog_dir + "/data/" + subdir);
            if (!fs::exists(dir)) fs::create_directories(dir);
            const std::string path = (dir / (timestamp + ".bin")).string();
            std::ofstream ofs(path, std::ios::binary);
            if (!ofs.is_open()) return false;
            const uint32_t magic = 0x52415731;
            const uint32_t version = 2;
            const uint32_t w = static_cast<uint32_t>(std::max(0, width));
            const uint32_t h = static_cast<uint32_t>(std::max(0, height));
            const uint32_t c = static_cast<uint32_t>(std::max(0, channels));
            const std::string compressed_format = format + "_zlib";
            const uint32_t format_size = static_cast<uint32_t>(compressed_format.size());
            uLongf compressed_bound = compressBound(static_cast<uLong>(data.size()));
            std::vector<uint8_t> compressed(compressed_bound);
            const int zret = compress2(compressed.data(), &compressed_bound,
                                        data.data(), static_cast<uLong>(data.size()),
                                        Z_BEST_SPEED);
            if (zret != Z_OK) return false;
            compressed.resize(compressed_bound);
            const uint64_t data_size = compressed.size();
            ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
            ofs.write(reinterpret_cast<const char*>(&w), sizeof(w));
            ofs.write(reinterpret_cast<const char*>(&h), sizeof(h));
            ofs.write(reinterpret_cast<const char*>(&c), sizeof(c));
            ofs.write(reinterpret_cast<const char*>(&format_size), sizeof(format_size));
            ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
            ofs.write(compressed_format.data(),
                      static_cast<std::streamsize>(compressed_format.size()));
            if (!compressed.empty()) {
                ofs.write(reinterpret_cast<const char*>(compressed.data()),
                          static_cast<std::streamsize>(compressed.size()));
            }
            return ofs.good();
        }

        void saveRGB(const std::vector<uint8_t>& rgb_vec, const std::string& timestamp)
        {
            constexpr int width = 640;
            constexpr int height = 544;
            if (rgb_vec.size() != static_cast<size_t>(width * height * 3)) {
                LOG_ERROR("[Visualize ERR]saveRGB size mismatch: %llu",
                          static_cast<unsigned long long>(rgb_vec.size()));
                return;
            }
            cv::Mat rgb(height, width, CV_8UC3,
                        const_cast<uint8_t*>(rgb_vec.data()));
            cv::Mat bgr;
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
            std::vector<uint8_t> jpeg;
            if (!cv::imencode(".jpg", bgr, jpeg,
                              {cv::IMWRITE_JPEG_QUALITY, 80}) ||
                !saveEncodedBytes("rgb", "JPEG", width, height, 3,
                                  jpeg, timestamp)) {
                LOG_ERROR("[Visualize ERR]saveRGB JPEG BIN write failed: %s",
                          timestamp.c_str());
            }
        }
        void saveGDC(std::vector<uint8_t> &rgb_vec, std::string timestamp)
        {
            // 创建保存目录
            std::filesystem::path dir(visualLog_dir + "/data/gdc/");
            if (!std::filesystem::exists(dir))
            {
                std::filesystem::create_directories(dir);
            }            
            // 固定图像分辨率
            const int gdc_width = 1280;
            const int gdc_height = 1088;
            const int channels = 3;
            size_t expect_size = static_cast<size_t>(gdc_width * gdc_height * channels);

            // 校验数据长度是否合法
            if (rgb_vec.size() != expect_size)
            {
                LOG_ERROR("[Visualize ERR]saveGDC, GDC buffer size error! expect:%d ,actual:%d",expect_size,rgb_vec.size());
                return;
            }

            cv::Mat rgb_mat(gdc_height, gdc_width, CV_8UC3, rgb_vec.data());
            // cv::Mat rgb_mat(gdc_height, gdc_width, CV_8UC3);
            // memcpy(rgb_mat.data, rgb_vec.data(), rgb_vec.size());
            cv::Mat bgr_mat;
            cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);

            std::string save_path = visualLog_dir + "/data/gdc/" + timestamp + ".jpg";
            std::vector<int> jpg_param;
            jpg_param.push_back(cv::IMWRITE_JPEG_QUALITY);
            jpg_param.push_back(70);
            bool ret = cv::imwrite(save_path, bgr_mat,jpg_param);
        }
        // AI 结果图（dnn overlay：RGB底图+分割着色+检测框）落盘，与 saveGDC 同风格。
        // 尺寸随发布配置（overlay_pub_width/height 或原图），不校验固定分辨率。
        bool saveAiOverlayJson(const std::vector<perception::DnnDetection>& dets,
                                const std::vector<uint8_t>& seg_mask,
                                int width, int height,
                                const std::string& timestamp)
        {
            fs::path dir(visualLog_dir + "/data/ai_det");
            if (!fs::exists(dir)) fs::create_directories(dir);
            std::ofstream ofs((dir / (timestamp + ".json")).string());
            if (!ofs.is_open()) return false;
            ofs << "{\"version\":3,\"det_source\":\"raw_model\""
                << ",\"det_count\":" << dets.size()
                << ",\"width\":" << width
                << ",\"height\":" << height << ",\"detections\":[";
            for (size_t i = 0; i < dets.size(); ++i) {
                const auto& d = dets[i];
                if (i) ofs << ',';
                ofs << "{\"class_id\":" << d.class_id
                    << ",\"class_name\":\"" << d.class_name
                    << "\",\"confidence\":" << d.confidence
                    << ",\"x1\":" << d.x1 << ",\"y1\":" << d.y1
                    << ",\"x2\":" << d.x2 << ",\"y2\":" << d.y2 << '}';
            }
            ofs << "],\"seg_mask_file\":\"" << timestamp
                << ".bin\",\"seg_mask_size\":" << seg_mask.size() << "}\n";
            return ofs.good();
        }

        bool saveAiOverlayMaskRaw(const std::vector<uint8_t>& seg_mask,
                                  int width, int height,
                                  const std::string& timestamp)
        {
            if (seg_mask.empty() || width <= 0 || height <= 0 ||
                seg_mask.size() != static_cast<size_t>(width) * height) return false;

            // 分割类别只有少量离散值，按行程编码保存；PC脚本可无损解码。
            std::vector<uint8_t> encoded;
            encoded.reserve(seg_mask.size() / 8);
            size_t i = 0;
            while (i < seg_mask.size()) {
                const uint8_t value = seg_mask[i];
                size_t run = 1;
                while (i + run < seg_mask.size() && seg_mask[i + run] == value &&
                       run < std::numeric_limits<uint16_t>::max()) {
                    ++run;
                }
                const uint16_t count = static_cast<uint16_t>(run);
                encoded.push_back(value);
                encoded.push_back(static_cast<uint8_t>(count & 0xff));
                encoded.push_back(static_cast<uint8_t>(count >> 8));
                i += run;
            }
            return saveRawBytes("ai_mask", "mask_rle_u8_v1", width, height, 1,
                                encoded, timestamp);
        }

        bool saveAiCloudOverlayRaw(const std::vector<uint8_t>& point_classes,
                                   int width, int height,
                                   const std::string& timestamp)
        {
            // format=7：类别数组与同时间戳disp.bin中的u/v逐点对齐。
            constexpr uint32_t kMagic = 0x44595249;
            constexpr uint16_t kVersion = 1;
            constexpr uint16_t kFormatSparseClassU8 = 7;
            if (width <= 0 || height <= 0 || point_classes.empty()) return false;

            fs::path dir(visualLog_dir + "/data/ai_cloud_overlay_raw/");
            if (!fs::exists(dir)) fs::create_directories(dir);
            const std::string path = (dir / (timestamp + ".bin")).string();
            std::ofstream ofs(path, std::ios::binary);
            if (!ofs.is_open()) return false;

            uint64_t timestamp_ms = 0;
            try {
                timestamp_ms = static_cast<uint64_t>(std::stoull(timestamp));
            } catch (...) {
            }
            const uint32_t width_u32 = static_cast<uint32_t>(width);
            const uint32_t height_u32 = static_cast<uint32_t>(height);
            const uint32_t stride = 0;  // 稀疏格式无图像行跨度
            const uint64_t data_size = static_cast<uint64_t>(point_classes.size());
            ofs.write(reinterpret_cast<const char*>(&kMagic), sizeof(kMagic));
            ofs.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));
            ofs.write(reinterpret_cast<const char*>(&kFormatSparseClassU8),
                      sizeof(kFormatSparseClassU8));
            ofs.write(reinterpret_cast<const char*>(&width_u32), sizeof(width_u32));
            ofs.write(reinterpret_cast<const char*>(&height_u32), sizeof(height_u32));
            ofs.write(reinterpret_cast<const char*>(&stride), sizeof(stride));
            ofs.write(reinterpret_cast<const char*>(&timestamp_ms), sizeof(timestamp_ms));
            ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
            ofs.write(reinterpret_cast<const char*>(point_classes.data()),
                      static_cast<std::streamsize>(point_classes.size()));
            return ofs.good();
        }

        // 已废弃：saveAiOverlayAsync 为空返回，本函数永不被调用。
        // 若将来重新启用，输出目录改为 ai_mask（与 RLE mask 一致）。
        bool saveOverlay(const std::vector<uint8_t> &rgb_vec, int width, int height, std::string timestamp)
        {
            std::filesystem::path dir(visualLog_dir + "/data/ai_mask/");
            if (!std::filesystem::exists(dir))
            {
                std::filesystem::create_directories(dir);
            }            
            if (rgb_vec.empty() || width <= 0 || height <= 0 ||
                rgb_vec.size() != static_cast<size_t>(width) * height * 3)
            {
                LOG_ERROR("[Visualize ERR]saveOverlay, Overlay buffer size error! expect:%d ,actual:%d",static_cast<size_t>(std::max(0, width) * std::max(0, height)) * 3,rgb_vec.size());
                return false;
            }

            // Store the rendered RGB pixels as a BIN payload for offline rendering.
            return saveRawBytes("ai_overlay", "rgb8_overlay", width, height, 3,
                                rgb_vec, timestamp);
        }
        void createColoredMap(const std::vector<MapPoint>& map_points,visualization_msgs::msg::MarkerArray& marker_array,string header,double mapWidth,double mapHeight,double mapResolution,int index)
        {
            int cols = static_cast<int>(mapWidth /mapResolution);  //列数
            int rows = static_cast<int>(mapHeight /mapResolution);  //行数 
            // 创建一个Marker用于显示彩色点
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = rclcpp::Clock().now();
            marker.header.frame_id = parent_frame;
            marker.ns = header;
            marker.id = index;
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            // marker.lifetime = rclcpp::Duration::from_seconds(0.1); 
            // 设置点的大小
            marker.scale.x = mapResolution;  // 点的大小与分辨率匹配
            marker.scale.y = mapResolution;
            marker.scale.z = 0.01;  // 稍微有点高度
            
        // 设置姿态（无旋转）
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            
            // 原点位置
            marker.pose.position.x = 0.0;
            if (header == "fusion"){marker.pose.position.y = mapWidth / 2.0;}
            if (header == "ai"){marker.pose.position.y = mapWidth / 2.0 + mapWidth + 0.1;}
            if (header == "pcl"){marker.pose.position.y = mapWidth / 2.0 - mapWidth - 0.1;}
            marker.pose.position.z = 0.0;


            std_msgs::msg::ColorRGBA colors[] = {
                this->createColorRGBA(255, 0, 0),     // 红色
                this->createColorRGBA(0, 255, 0),     // 绿色
                this->createColorRGBA(0, 0, 255),     // 蓝色
                this->createColorRGBA(255, 255, 0),   // 黄色
                this->createColorRGBA(255, 165, 0),   // 橙色
                this->createColorRGBA(255, 0, 255),   // 粉色
                this->createColorRGBA(255, 255, 255), // 白色
                this->createColorRGBA(0, 0, 0)        // 黑色
            };
            for (int i=0;i<map_points.size();i++) {
                int map_x = i % cols; //列索引
                int map_y = i / cols ;  //行索引

                geometry_msgs::msg::Point p;
                p.x = (map_y + 0.5) * mapResolution;
                p.y = -(map_x + 0.5) * mapResolution; 
                p.z = map_points[i].avg_h; 
                marker.points.push_back(p);
                int label = map_points[i].type;

                std_msgs::msg::ColorRGBA color;
                //0=未知, 1=非草, 2=其他障碍，3 =草，>=4 ai
                if (label == 0){color=colors[3];}  //"unknown"
                if (label == 1 ){color=colors[7];} //"nonegrass"
                if (label == 2 ){color=colors[0];} //"obs"
                if (label == 3){color=colors[1];} //"grass"
                if (label > 3) {color=colors[5];}
                marker.colors.push_back(color);
            }

            marker_array.markers.push_back(marker);

        }
        std_msgs::msg::ColorRGBA createColorRGBA(uint8_t r, uint8_t g, uint8_t b)
        {
            std_msgs::msg::ColorRGBA color;
            color.r = r / 255.0f;
            color.g = g / 255.0f;
            color.b = b / 255.0f;
            color.a = 1.0f;
            return color;
        }
        PointT createBboxMarker(visualization_msgs::msg::Marker& bbox_marker,PointCloudT::Ptr& cluster,int id,uint8_t r,uint8_t g,uint8_t b)
        {
            PointT min_pt, max_pt,center;
            pcl::getMinMax3D<PointT>(*cluster, min_pt, max_pt);
            bbox_marker.header.stamp = rclcpp::Clock().now();
            bbox_marker.header.frame_id = parent_frame;
            bbox_marker.ns = "bbox";
            bbox_marker.id = id;
            bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
            bbox_marker.action = visualization_msgs::msg::Marker::ADD;
            bbox_marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
            bbox_marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
            bbox_marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
            bbox_marker.pose.orientation.x = 0.0;  // Identity orientation
            bbox_marker.pose.orientation.y = 0.0;
            bbox_marker.pose.orientation.w = 1.0;
            bbox_marker.scale.x = std::max((max_pt.x - min_pt.x), 0.1f);
            bbox_marker.scale.y = std::max((max_pt.y - min_pt.y), 0.1f);
            bbox_marker.scale.z = std::max((max_pt.z - min_pt.z), 0.1f);
            bbox_marker.color.r = static_cast<float>(r) / 255.0;
            bbox_marker.color.g = static_cast<float>(g) / 255.0;
            bbox_marker.color.b = static_cast<float>(b) / 255.0;
            bbox_marker.color.a = 0.5;
            // bbox_marker.lifetime() = rclcpp::Duration::from_seconds(1.0);
            center.x = bbox_marker.pose.position.x;
            center.y = bbox_marker.pose.position.y;
            center.z = bbox_marker.pose.position.z;
            return center;
        }
        visualization_msgs::msg::Marker  createPlaneMarker(Eigen::VectorXf normal,double d_,Eigen::VectorXf center,int id)
        {
            visualization_msgs::msg::Marker marker;
            
            // 设置基本信息
            marker.header.stamp = rclcpp::Clock().now();
            marker.header.frame_id = parent_frame;
            marker.ns = "plane";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = center[0];
            marker.pose.position.y = center[1];
            marker.pose.position.z = center[2];
            // 设置外观
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;  // 半透明
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            // marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            float a = normal[0], b = normal[1], c = normal[2], d = d_;
            // 找到平面上的一个点（当z=0时）
            Eigen::Vector3f point_on_plane;
            if (std::abs(c) > 1e-6) {
                point_on_plane = Eigen::Vector3f(0, 0, -d/c);
            } else if (std::abs(b) > 1e-6) {
                point_on_plane = Eigen::Vector3f(0, -d/b, 0);
            } else {
                point_on_plane = Eigen::Vector3f(-d/a, 0, 0);
            }
            // 创建平面的两个正交方向向量
            Eigen::Vector3f normal_vec(a, b, c);
            normal_vec.normalize();
            
            // 找到第一个正交向量
            Eigen::Vector3f u_vec;
            if (std::abs(normal_vec.x()) > 1e-6 || std::abs(normal_vec.y()) > 1e-6) {
                u_vec = Eigen::Vector3f(-normal_vec.y(), normal_vec.x(), 0).normalized();
            } else {
                u_vec = Eigen::Vector3f(1, 0, 0);
            }
            
            // 找到第二个正交向量（叉积）
            Eigen::Vector3f v_vec = normal_vec.cross(u_vec).normalized();
            
            // 定义平面大小
            float plane_size = 2.0f;
            
            // 创建四个角点
            std::vector<Eigen::Vector3f> corners = {
                point_on_plane + u_vec * plane_size + v_vec * plane_size,
                point_on_plane + u_vec * plane_size - v_vec * plane_size,
                point_on_plane - u_vec * plane_size - v_vec * plane_size,
                point_on_plane - u_vec * plane_size + v_vec * plane_size
            };
            
            // 创建两个三角形来表示平面
            std::vector<std::array<int, 3>> triangles = {
                {0, 1, 2}, {0, 2, 3}
            };
            
            // 添加顶点到marker
            for (const auto& triangle : triangles) {
                for (int idx : triangle) {
                    geometry_msgs::msg::Point point;
                    point.x = corners[idx].x();
                    point.y = corners[idx].y();
                    point.z = corners[idx].z();
                    marker.points.push_back(point);
                }
            }
            
            return marker;
        }
        visualization_msgs::msg::Marker  createNormalMarker(const Eigen::Vector3f& point, const Eigen::Vector3f& normal,int id) 
        {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = rclcpp::Clock().now();
            marker.header.frame_id = parent_frame;
            marker.ns = "plane_normal";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // 线段起点（平面上的点）
            geometry_msgs::msg::Point start;
            start.x = point.x();
            start.y = point.y();
            start.z = point.z();
            marker.points.push_back(start);

            // 线段终点（沿法向量方向延伸）
            geometry_msgs::msg::Point end;
            end.x = point.x() + normal.x() * 0.5;  // 长度缩放
            end.y = point.y() + normal.y() * 0.5;
            end.z = point.z() + normal.z() * 0.5;
            marker.points.push_back(end);

            // 设置颜色（红色）和尺寸
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 0.02;  // 箭头轴直径
            marker.scale.y = 0.04;  // 箭头头直径
            marker.scale.z = 0.0;   // 未使用
            // marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            return marker;
        }
            
};
#endif