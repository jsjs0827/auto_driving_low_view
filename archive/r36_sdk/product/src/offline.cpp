/**
 * @file offline.cpp
 * @brief 原几何离线回放：严格同帧 JPG/RAW 配对，文件读取通过事件 TaskGraph。
 */
#include "percep_demo/percep_demo.h"
#include "product/offline_input.hpp"

namespace ai_node_percep_demo {

void PercepDemo::offline_loop()
{
    if (!load_files()) {
        cerr << "Failed to start offline traceback!" << endl;
        return;
    }
    const auto rate = std::chrono::milliseconds(1);
    size_t current_index_ = 0;
    if (interactive) {
        cout << "控制命令: [回车]下一帧, p上一帧, n <从0开始的帧号>跳转, r重新播放, q退出" << endl;
    }
    while (g_running) {
        if (interactive) {
            std::string input;
            if (!std::getline(std::cin, input)) break;
            if (input == "q" || input == "Q") break;
            if (input == "r" || input == "R") {
                current_index_ = 0;
                this->playFrame(current_index_);
                continue;
            }
            if (input == "p" || input == "P") {
                if (current_index_ > 0) --current_index_;
                this->playFrame(current_index_);
                continue;
            }
            if (!input.empty()) {
                size_t requested = current_index_;
                if (!r30::product::parseOfflineJump(input, nv12_files.size(), requested)) {
                    cout << "未知命令或帧号越界；使用 n <从0开始的帧号>" << endl;
                    continue;
                }
                current_index_ = requested;
            }
        }
        this->playFrame(current_index_);
        ++current_index_;
        std::this_thread::sleep_for(rate);
    }
}

bool PercepDemo::load_files()
{
    nv12_files.clear(); nv12_names.clear(); disp_files.clear(); disp_names.clear();
    const std::map<std::string, std::function<void()>> bindings = {{"offline_read", [&] {
        const auto frames = r30::product::indexOfflineInput(offline_rgb_dir_,
            offline_disparity_dir_, static_cast<int>(rgb_width), static_cast<int>(rgb_height));
        for (const auto& frame : frames) {
            nv12_files.push_back(frame.rgb_path); nv12_names.push_back(frame.stem);
            disp_files.push_back(frame.disparity_path); disp_names.push_back(frame.stem);
        }
    }}};
    if (runProductEvent("offline_read", bindings, {}, 0) != 0) {
        nv12_files.clear(); nv12_names.clear(); disp_files.clear(); disp_names.clear();
        return false;
    }
    return true;
}

void PercepDemo::playFrame(size_t idx)
{
    if (idx >= nv12_files.size() || idx >= disp_files.size()) {
        printf("Frame index %zu out of total frames, stop playback\n", idx);
        g_running = false;
        return;
    }
    perception::FrameInfo frame; frame.id=idx;
    std::vector<uint8_t> gdc_buf, rgb_buf, disp_buf;
    sensor_msgs::msg::Image left_img_msg;
    // Decode from bytes opened with filesystem::u8path, so Windows UTF-8 paths
    // do not depend on OpenCV's narrow filename encoding. Geometry is unchanged.
    const std::map<std::string, std::function<void()>> input_bindings = {{"offline_read", [&] {
        if (nv12_names[idx] != disp_names[idx]) throw std::runtime_error("offline frame stem mismatch");
        const auto jpeg = r30::product::readOfflineImage(nv12_files[idx]);
        cv::Mat img_cache_ = cv::imdecode(jpeg, cv::IMREAD_COLOR);
        if (img_cache_.empty()) throw std::runtime_error("offline JPEG decode failed: " + nv12_files[idx]);
        disp_buf = r30::product::readOfflineDisparity(disp_files[idx],
            static_cast<int>(rgb_width), static_cast<int>(rgb_height));
        cv::cvtColor(img_cache_, img_cache_, cv::COLOR_BGR2RGB);
        const int source_width=img_cache_.cols, source_height=img_cache_.rows;
        const size_t total_byte_gdc=img_cache_.total()*img_cache_.elemSize();
        gdc_buf.assign(img_cache_.data, img_cache_.data + total_byte_gdc);
        if (img_cache_.cols != static_cast<int>(rgb_width) || img_cache_.rows != static_cast<int>(rgb_height))
            cv::resize(img_cache_, img_cache_, cv::Size(rgb_width, rgb_height));
        const size_t total_byte=static_cast<size_t>(rgb_width)*rgb_height*3;
        rgb_buf.assign(img_cache_.data, img_cache_.data + total_byte);
        left_img_msg.header.frame_id="percep_demo";
        left_img_msg.header.stamp=rclcpp::Clock().now();
        left_img_msg.width=source_width; left_img_msg.height=source_height;
        left_img_msg.encoding="rgb8"; left_img_msg.is_bigendian=false;
        left_img_msg.step=source_width*3; left_img_msg.data=gdc_buf;
    }}};
    if (runProductEvent("offline_read", input_bindings, frame, 0) != 0) return;

    std::vector<float> disp_float(rgb_width * rgb_height);
    const uint16_t* disp_raw = reinterpret_cast<const uint16_t*>(disp_buf.data());
    float max_disp = 144.0f; 
    float min_disp = 1.0f;

    for (int i = 0; i < rgb_width * rgb_height; i++)
    {
        float d = static_cast<float>(disp_raw[i]) / 256.0f;
        disp_float[i] = std::clamp(d, min_disp, max_disp);
    }

    // 归一化映射到0~255 uint8灰度
    cv::Mat disp_mat(rgb_height, rgb_width, CV_32FC1, disp_float.data());
    cv::Mat disp_gray;
    disp_mat = (disp_mat - min_disp) / (max_disp - min_disp) * 255.0f;
    disp_mat.convertTo(disp_gray, CV_8UC1);

    // 封装ROS2 mono8图像消息
    sensor_msgs::msg::Image disp_gray_msg;
    disp_gray_msg.header.frame_id = "percep_demo";
    disp_gray_msg.header.stamp = rclcpp::Clock().now();
    disp_gray_msg.width = rgb_width;
    disp_gray_msg.height = rgb_height;
    disp_gray_msg.encoding = "mono8";
    disp_gray_msg.step = rgb_width;
    disp_gray_msg.data.assign(disp_gray.data, disp_gray.data + disp_gray.total());



    // ====================== 生成稀疏点云 ======================
    PointCloudT::Ptr sparse_cloud(new PointCloudT());
    int sample_step = 1;    
    sparse_cloud->reserve((rgb_width / sample_step) * (rgb_height / sample_step)); // 预分配内存

    // fx:288.129 fy:288.129 cx:308.542 cy:278.213 bxf:17.3211 baseline:60.1156    
    float fx = 718.750793457 / 2.0;
    float fy = 718.975646973 / 2.0;
    float cx = 624.340698242 / 2.0;
    float cy = 556.446472168 / 2.0;
    float baseline = 60.1156;
    // float fx = 288.129;
    // float fy = 288.129;
    // float cx = 308.542;
    // float cy = 278.213;
    double bxf = baseline * fx / 1000.0f;


    r28::product::ProductDag::Bindings bindings;
    if(visualize) bindings["rgb"]=[&]{rgb_img_pub_.publish(left_img_msg);disp_img_pub_.publish(disp_gray_msg);};
    bindings["cloud"]=[&]{
    for (int v = 0; v < rgb_height; v += sample_step)
    {
        for (int u = 0; u < rgb_width; u += sample_step)
        {
            size_t pix_idx = static_cast<size_t>(v) * rgb_width + u;
            uint16_t disp_raw_val = disp_raw[pix_idx];
            
            float disp = static_cast<float>(disp_raw_val) / 256.0;
            // 过滤无效视差
            // if (disp < 5 || disp > 12000)
            //     continue;

            // 深度 Z 单位 m
            float Z_m = static_cast<float>(bxf)/ disp;
            if (Z_m < 0.001f) // 过滤过近无效点
                continue;

            // X Y 单位 m
            float X_m = (static_cast<float>(u) - cx) * Z_m / fx;
            float Y_m = (static_cast<float>(v) - cy) * Z_m / fy;
            if (Z_m < pcl_pub_cfg_.box_min_z || Z_m > pcl_pub_cfg_.box_max_z 
                || X_m < pcl_pub_cfg_.box_min_x || X_m > pcl_pub_cfg_.box_max_x 
                || Y_m < pcl_pub_cfg_.box_min_y || Y_m > pcl_pub_cfg_.box_max_y
                )
            {
                continue;
            }
            sparse_cloud->emplace_back(X_m, Y_m, Z_m, 0, 0, 0, 0.f, 0.f);
            // cout<<"X_m:"<<X_m<<" Y_m:"<<Y_m<<" Z_m:"<<Z_m<<endl;
        }
    }

    if (sparse_cloud->empty())
    {
        std::cerr << "[PclCommon] sparse cloud generate empty\n";
        throw std::runtime_error("offline disparity point cloud is empty");
    }
    };
    cali.roll = angle_to_rad(-90.0);
    cali.pitch = angle_to_rad(-2.0);
    const std::vector<uint8_t> *proj_rgb = &rgb_buf;
    if(auto_cali) bindings["calibration"]=[&]{autocali(sparse_cloud);};
    else {
    bindings["transform"]=[&]{postprocess(sparse_cloud,*proj_rgb,rgb_width,rgb_height,fx,fy,cx,cy);};
    if(patchwork_cfg_.enabled) bindings["ground"]=[&]{runGroundSegmentation(sparse_cloud);};
    bindings["maps"]=[&]{buildProductMaps();};
    bindings["publish"]=[&]{publishProductMaps(static_cast<int>(idx));};
    if(save_log) bindings["record"]=[&]{recordProductMaps();};
    }
    if(runProductBindings(bindings,frame)==0) printf("Publish frame %zu done\n", idx);
}

}  // namespace ai_node_percep_demo


