#ifndef VISUALIZE_H
#define VISUALIZE_H
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include "percep_demo/common.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

class Visualize :public PclCommon{

    public:
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
                std::cerr << "[savePCL] open failed: " << filePath << std::endl;
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
            std::cout << "[savePCL] save " << point_num << " points to " << filePath << std::endl;
            return true;
        }
        bool saveDisp(const std::vector<uint32_t>& u_vec,const std::vector<uint32_t>& v_vec,std::vector<float> &disp,string  timestamp)
        {
            if(disp.empty())
            {
                std::cerr << "[saveDisp] disp empty "<<std::endl;
                return false;
            }
            fs::path dir(visualLog_dir+"/data/disp/");
            if (!dir.empty() && !fs::exists(dir)) {
                fs::create_directories(dir);
            }
            std::string path = visualLog_dir + "/data/disp/" + timestamp + ".bin";
            std::ofstream ofs(path, std::ios::binary);
            if(!ofs.is_open())
            {
                std::cerr << "[saveDisp] open failed: " << path << std::endl;
                return false;
            }

            uint64_t num = static_cast<uint64_t>(disp.size());
            ofs.write(reinterpret_cast<const char*>(&num), sizeof(uint64_t));
            ofs.write(reinterpret_cast<const char*>(u_vec.data()), num * sizeof(uint32_t));
            ofs.write(reinterpret_cast<const char*>(v_vec.data()), num * sizeof(uint32_t));
            ofs.write(reinterpret_cast<const char*>(disp.data()), num * sizeof(float));

            ofs.close();
            if(!ofs) throw std::runtime_error("disparity binary write/close failed");
            std::cout << "[saveDisp] save disp size="<<num<<" -> "<<path<<std::endl;
            return true;
        }
        bool saveMap(vector<MapPoint> &grid_map_save,string  header,string  timestamp,int  rows,int  cols)
        {
            if(grid_map_save.empty()) { return false;}
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

                    if (label_index == 0){*pixel_ptr = yellow; } //未知
                    if (label_index == 1 ){*pixel_ptr = black; } //非草
                    if (label_index == 2){*pixel_ptr = red; } //障碍物
                    if (label_index == 3){*pixel_ptr = green; } //草
                    if (label_index == 4) {*pixel_ptr = magenta;}  //人
                    if (label_index == 5) {*pixel_ptr = cyan;}  //猫
                    if (label_index == 6) {*pixel_ptr = white;}  //狗
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
                fs::path dir(visualLog_dir+"/map/"+header+"/");
                if (!dir.empty() && !fs::exists(dir)) {
                    fs::create_directories(dir);
                }
                std::vector<int> png_params = {
                    cv::IMWRITE_PNG_COMPRESSION, 9
                };

                if(!cv::imwrite(visualLog_dir+"/map/"+header+"/"+ timestamp+".png", resized_map, png_params))
                    throw std::runtime_error("map PNG encoding/write failed");
                // cv::imwrite(visualLog_dir+header+"/"+ timestamp+".jpg", resized_map, {cv::IMWRITE_JPEG_QUALITY, 70});
            }else{return false;}

            return true;
        }

        void saveRGB(std::vector<uint8_t> &rgb_vec, std::string timestamp)
        {
            // 固定图像分辨率
            const int rgb_width = 640;
            const int rgb_height = 544;
            const int channels = 3;
            size_t expect_size = static_cast<size_t>(rgb_width * rgb_height * channels);

            // 校验数据长度是否合法
            if (rgb_vec.size() != expect_size)
            {
                std::cout << "RGB buffer size error! expect:" << expect_size 
                        << " actual:" << rgb_vec.size() << std::endl;
                throw std::runtime_error("RGB recording buffer size invalid");
            }

            // cv::Mat rgb_mat(rgb_height, rgb_width, CV_8UC3, rgb_vec.data());
            cv::Mat rgb_mat(rgb_height, rgb_width, CV_8UC3);
            memcpy(rgb_mat.data, rgb_vec.data(), rgb_vec.size());
            cv::Mat bgr_mat;
            cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);

            // 创建保存目录
            std::filesystem::path dir(visualLog_dir + "/data/rgb/");
            if (!std::filesystem::exists(dir))
            {
                std::filesystem::create_directories(dir);
            }
            std::string save_path = visualLog_dir + "/data/rgb/" + timestamp + ".jpg";
            std::vector<int> jpg_param;
            jpg_param.push_back(cv::IMWRITE_JPEG_QUALITY);
            jpg_param.push_back(70);
            bool ret = cv::imwrite(save_path, bgr_mat,jpg_param);
            if(!ret) throw std::runtime_error("RGB JPEG encoding/write failed");

        }
        void saveGDC(std::vector<uint8_t> &rgb_vec, std::string timestamp)
        {
            // 固定图像分辨率
            const int gdc_width = 1280;
            const int gdc_height = 1088;
            const int channels = 3;
            size_t expect_size = static_cast<size_t>(gdc_width * gdc_height * channels);

            // 校验数据长度是否合法
            if (rgb_vec.size() != expect_size)
            {
                std::cout << "RGB buffer size error! expect:" << expect_size 
                        << " actual:" << rgb_vec.size() << std::endl;
                throw std::runtime_error("GDC recording buffer size invalid");
            }

            // cv::Mat rgb_mat(gdc_height, gdc_width, CV_8UC3, rgb_vec.data());
            cv::Mat rgb_mat(gdc_height, gdc_width, CV_8UC3);
            memcpy(rgb_mat.data, rgb_vec.data(), rgb_vec.size());
            cv::Mat bgr_mat;
            cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);

            // 创建保存目录
            std::filesystem::path dir(visualLog_dir + "/data/gdc/");
            if (!std::filesystem::exists(dir))
            {
                std::filesystem::create_directories(dir);
            }
            std::string save_path = visualLog_dir + "/data/gdc/" + timestamp + ".jpg";
            std::vector<int> jpg_param;
            jpg_param.push_back(cv::IMWRITE_JPEG_QUALITY);
            jpg_param.push_back(70);
            bool ret = cv::imwrite(save_path, bgr_mat,jpg_param);
            if(!ret) throw std::runtime_error("GDC JPEG encoding/write failed");

        }
        PointT createBboxMarker(visualization_msgs::msg::Marker& bbox_marker,PointCloudT::Ptr& cluster,int id,uint8_t r,uint8_t g,uint8_t b)
        {
            PointT min_pt, max_pt,center;
            pcl::getMinMax3D<PointT>(*cluster, min_pt, max_pt);
            
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
