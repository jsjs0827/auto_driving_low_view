/**
 * @file offline.cpp
 * @brief 离线回放本地数据
 */
#include "percep_main/percep_main.h"

static double fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f, base_line = 0.f;
// RGB24 (R-G-B) → NV12
static void rgb24_to_nv12(const uint8_t* rgb, uint8_t* nv12, int width, int height)
{
    int y_idx = 0;
    int uv_idx = width * height;

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            size_t pixel_pos = (j * width + i) * 3;
            uint8_t r = rgb[pixel_pos + 0];
            uint8_t g = rgb[pixel_pos + 1];
            uint8_t b = rgb[pixel_pos + 2];

            // BT.601 公式
            uint8_t y = static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);
            nv12[y_idx++] = y;

            // 2x2下采样UV，只在偶数行、偶数列输出UV
            if ((j % 2 == 0) && (i % 2 == 0))
            {
                float uf =
                    -0.14713f * r -
                    0.28886f * g +
                    0.436f * b +
                    128.0f;

                float vf =
                    0.615f * r -
                    0.51499f * g -
                    0.10001f * b +
                    128.0f;

                uint8_t u = static_cast<uint8_t>(
                    std::clamp(uf, 0.0f, 255.0f)
                );

                uint8_t v = static_cast<uint8_t>(
                    std::clamp(vf, 0.0f, 255.0f)
                );
                nv12[uv_idx++] = u;
                nv12[uv_idx++] = v;
            }
        }
    }
}

namespace perception {

// 等一行 stdin，期间每 100ms 回头检查一次退出标志。
// 返回 false 表示该退出了：收到退出信号，或者 stdin 到了 EOF
// (比如用管道喂输入、输入喂完了，此时也不该继续空转)。
bool PercepMain::waitStdinLine(std::string& out)
{
    out.clear();
    while (offline_running)
    {
        struct pollfd pfd;
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        pfd.revents = 0;
        const int r = ::poll(&pfd, 1, 100);   // 100ms 超时
        if (r < 0)
        {
            if (errno == EINTR) continue;     // 被信号打断，回头看标志
            return false;
        }
        if (r == 0) continue;                 // 超时，回头看标志
        if (!std::getline(std::cin, out)) return false;  // EOF 或读失败
        return true;
    }
    return false;
}
bool PercepMain::initCameraParams()
{
    rapidjson::Document params_json;
    std::ifstream ifs(camparams_dir, std::ios::binary);
    if (!ifs.is_open()) {
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    string start_config = ss.str();
    ifs.close();  
    params_json.Parse(start_config.c_str());
    if (params_json.HasParseError()){

      return false;
    } 
    if (!params_json.IsObject()) {
        return false; 
    }

    if (params_json.HasMember("intrinsics") && params_json["intrinsics"].IsObject()) {
        const rapidjson::Value& pd = params_json["intrinsics"];
        if (pd.HasMember("fx") && pd["fx"].IsDouble()) {
            fx = pd["fx"].GetDouble();
        }
        if (pd.HasMember("fy") && pd["fy"].IsDouble()) {
            fy = pd["fy"].GetDouble();
        }
        if (pd.HasMember("cx") && pd["cx"].IsDouble()) {
            cx = pd["cx"].GetDouble();
        }
        if (pd.HasMember("cy") && pd["cy"].IsDouble()) {
            cy = pd["cy"].GetDouble();
        }
        if (pd.HasMember("base_line") && pd["base_line"].IsDouble()) {
            base_line = pd["base_line"].GetDouble();
        }
    }

    camera_params.fx = fx;
    camera_params.fy = fy;
    camera_params.cx = cx;
    camera_params.cy = cy;
    camera_params.base_line = base_line;
    camera_params.bxf = base_line * fx;
    camera_params.width = rgb_width;
    camera_params.height = rgb_height;
    receive_params.store(true);

    return true;
}
void PercepMain::offline_loop() 
{
    if (!load_files()) {
      cerr<<"Failed to start offline traceback!"<<endl;
      return;
    } 
    const auto rate = std::chrono::milliseconds(1);  
    size_t current_index_ = 0;
    if (interactive)
    {
      cout << std::string(60, '=') << std::endl;
      cout << "\n控制命令:" << std::endl;
      cout << "  [回车]      - 播放下一帧" << std::endl;
      cout << "  p           - 播放上一帧" << std::endl;
      cout << "  n [帧号]    - 跳转到指定帧" << std::endl;
      cout << "  r           - 重新开始播放" << std::endl;
      cout << "  q           - 退出" << std::endl;
      cout << std::string(60, '=') << std::endl;      
    }
    while (offline_running) 
    {
      if (interactive)
      {
        std::string input;
        // 可中断的读取：收到退出信号或 stdin EOF 就跳出循环，
        // 让线程能正常结束，析构里的 join() 才不会卡死。
        if (!waitStdinLine(input)) break;

        if (input == "q" || input == "Q") {
          break;
        } else if (input == "r" || input == "R") {
          current_index_ = 0;
          cout << "已重新开始播放" << std::endl;
          this->playFrame(current_index_);
          continue;
        } else if (input == "p" || input == "P") {
          // 播放上一帧
          if(current_index_ > 0 ) { current_index_--;}
          this->playFrame(current_index_);
          continue;
        } else if (input.empty()) {
          // 播放下一帧
          this->playFrame(current_index_);
          current_index_++;
          continue;
        } else if ( (input.size() >= 2 && input.substr(0,2) == "n ") || (input.size() >=2 && input.substr(0,2) == "N ") ){
            std::string num_str = input.substr(2);
            try {
                size_t target = std::stoul(num_str);
                const size_t max_frame = std::min(nv12_files.size(), disp_files.size());
                if (target >= max_frame) {
                    std::cerr << "帧号越界！总帧数：" << max_frame << std::endl;
                } else {
                    current_index_ = target;
                    cout << "跳转到第 " << current_index_ << " 帧" << std::endl;
                    playFrame(current_index_);
                }
            } catch (...) {
                std::cerr << "解析失败！用法示例：n 10" << std::endl;
            }
            continue;
        }  
        else {
          cout << "未知命令，请参考上面的命令列表" << std::endl;
        }                
      }else{
        if(auto_loop)
        {
            if(current_index_ >= nv12_files.size() || current_index_ >= disp_files.size()) {current_index_=0; }
        }
        if(!this->playFrame(current_index_)) {break;}
        current_index_++;
        cout<<endl;
      }
      std::this_thread::sleep_for(rate);
    }
}
bool PercepMain::load_files() 
{

    if (!fs::exists(rgb_dir)) {
      cerr<<"错误,离线rgb文件夹不存在："<<rgb_dir<<endl;
    }
    else{
      for (const auto& entry : fs::directory_iterator(rgb_dir)) {
          if (entry.is_regular_file() && (entry.path().extension() == ".jpg" || entry.path().extension() == ".png")) {
            nv12_files.push_back(entry.path().string());
            nv12_names.push_back(entry.path().stem().string());
          }        
      }     
      std::sort(nv12_files.begin(),nv12_files.end());
      std::sort(nv12_names.begin(),nv12_names.end());       
    }
    if (!fs::exists(disp_dir)) {
      cerr<<"错误,离线disp数据文件夹不存在："<<disp_dir<<endl;
    }else{
        if (type == "raw")
        {
            for (const auto& entry : fs::directory_iterator(disp_dir)) {
                if (entry.is_regular_file() && entry.path().extension() == ".raw") {
                disp_files.push_back(entry.path().string());
                disp_names.push_back(entry.path().stem().string());
                }       
        
            }            
        }else if (type == "bin")
        {
            for (const auto& entry : fs::directory_iterator(disp_dir)) {
                if (entry.is_regular_file() && entry.path().extension() == ".bin") {
                disp_files.push_back(entry.path().string());
                disp_names.push_back(entry.path().stem().string());
                }       
            } 
        }
        std::sort(disp_files.begin(), disp_files.end());   
        std::sort(disp_names.begin(), disp_names.end());  
    }

    if (nv12_files.empty() || disp_files.empty()) {
      cerr<<"存在空数据文件夹！"<<endl;
      return false;
    }        

    initCameraParams();
    return true;
}
bool PercepMain::convertGDC(const std::string& nv12_path,
                            std::vector<uint8_t> &rgb_buf,
                            std::unique_ptr<CameraData>* out_camera_data,
                            std::unique_ptr<CameraGdcData>* out_gdc_data)
{
    // 空指针保护
    if(out_camera_data == nullptr || out_gdc_data == nullptr)
    {
        printf("out ptr is null!\n");
        return false;
    }

    // 解析JPG
    cv::Mat img_cache_ = cv::imread(nv12_path, cv::IMREAD_COLOR);
    if (img_cache_.empty()) {
        printf("read jpg failed: %s\n", nv12_path.c_str());
        return false;
    }

    cv::Mat rgb_img;
    cv::cvtColor(img_cache_, rgb_img, cv::COLOR_BGR2RGB);

    if (!rgb_img.isContinuous()) {
        rgb_img = rgb_img.clone();
    }

    const int src_w = rgb_img.cols;
    const int src_h = rgb_img.rows;

    std::vector<uint8_t> rgb_msg_data(
        rgb_img.total() * rgb_img.elemSize()
    );

    std::memcpy(
        rgb_msg_data.data(),
        rgb_img.data,
        rgb_msg_data.size()
    );
    printf(
        "RGB Mat: %dx%d channels=%d elemSize=%zu step=%zu continuous=%d total=%zu\n",
        rgb_img.cols,
        rgb_img.rows,
        rgb_img.channels(),
        rgb_img.elemSize(),
        rgb_img.step,
        rgb_img.isContinuous(),
        rgb_img.total()
    );
    sensor_msgs::msg::Image left_img_msg;
    left_img_msg.header.frame_id = "percep_main";
    left_img_msg.header.stamp = rclcpp::Clock().now();

    left_img_msg.width = src_w;
    left_img_msg.height = src_h;
    left_img_msg.encoding = "rgb8";
    left_img_msg.is_bigendian = false;
    left_img_msg.step = static_cast<uint32_t>(rgb_img.step);
    left_img_msg.data = std::move(rgb_msg_data);

    rgb_img_pub_.publish(left_img_msg);
    // 全局帧计数器
    static uint64_t frame_cnt = 0;
    uint64_t ts_ns = rclcpp::Clock().now().nanoseconds();

    // ====================== 填充 CameraGdcData (1280*1088 GDC大图) ======================
    cv::Mat img_gdc = rgb_img;
    if(src_w != gdc_width || src_h != gdc_height)
    {
        cv::resize(img_gdc, img_gdc, cv::Size(gdc_width, gdc_height));
    }
    img_gdc = img_gdc.clone(); //保证内存连续
    std::vector<uint8_t> rgb_gdc_buf(img_gdc.data, img_gdc.data + static_cast<size_t>(gdc_width)*gdc_height*3);
    std::vector<uint8_t> nv12_gdc(gdc_width * gdc_height*3/2);
    rgb24_to_nv12(rgb_gdc_buf.data(), nv12_gdc.data(), gdc_width, gdc_height);

    // 外部unique_ptr已经分配，直接解引用填充
    auto& latest_gdc = *out_gdc_data;
    latest_gdc->left.time_ns = ts_ns;
    latest_gdc->left.frame_id = frame_cnt;
    latest_gdc->left.stream_type = 0; //0代表NV12
    latest_gdc->left.sensor_id = 0;
    latest_gdc->left.width = gdc_width;
    latest_gdc->left.height = gdc_height;
    latest_gdc->left.bits_per_pixel = 8;
    latest_gdc->left.bytes_used = gdc_width * gdc_height*3/2;
    latest_gdc->left.filllight_status = 0;
    latest_gdc->left.filllight_brightness = 0;
    latest_gdc->left.rgd_image_type = 0;
    latest_gdc->left.exp_time = 0.0f;
    latest_gdc->left.analog_gain = 0.0f;
    memset(latest_gdc->left.reserved, 0, sizeof(latest_gdc->left.reserved));
    memcpy(latest_gdc->left_data, nv12_gdc.data(), gdc_width * gdc_height*3/2);

    // 单目输入，right帧头拷贝left，图像buffer清零
    latest_gdc->right = latest_gdc->left;
    memset(latest_gdc->right_data, 0, sizeof(latest_gdc->right_data));

    // ====================== 填充 CameraData (640*544 推理小图) ======================
    cv::Mat img_infer = rgb_img;
    if (src_w != rgb_width || src_h != rgb_height)
    {
        cv::resize(img_infer, img_infer, cv::Size(rgb_width, rgb_height));
    }
    img_infer = img_infer.clone();
    rgb_buf.assign(img_infer.data, img_infer.data + static_cast<size_t>(rgb_width)*rgb_height*3);

    std::vector<uint8_t> nv12_infer(rgb_width * rgb_height*3/2);
    rgb24_to_nv12(rgb_buf.data(), nv12_infer.data(), rgb_width, rgb_height);

    auto& latest_rgb = *out_camera_data;
    latest_rgb->left.time_ns = ts_ns;
    latest_rgb->left.frame_id = frame_cnt;
    latest_rgb->left.stream_type = 0;
    latest_rgb->left.sensor_id = 0;
    latest_rgb->left.width = rgb_width;
    latest_rgb->left.height = rgb_height;
    latest_rgb->left.bits_per_pixel = 8;
    latest_rgb->left.bytes_used = rgb_width * rgb_height*3/2;
    latest_rgb->left.filllight_status = 0;
    latest_rgb->left.filllight_brightness = 0;
    latest_rgb->left.rgd_image_type = 0;
    latest_rgb->left.exp_time =0.0f;
    latest_rgb->left.analog_gain =0.0f;
    memset(latest_rgb->left.reserved,0,sizeof(latest_rgb->left.reserved));
    memcpy(latest_rgb->left_data, nv12_infer.data(), rgb_width * rgb_height*3/2);

    // right、视差图清零
    latest_rgb->right = latest_rgb->left;
    memset(latest_rgb->right_data,0,sizeof(latest_rgb->right_data));
    latest_rgb->disp = latest_rgb->left;
    memset(latest_rgb->disp_data,0,sizeof(latest_rgb->disp_data));

    frame_cnt ++;
    return true;
}
bool PercepMain::convertDispRaw(const string& disp_path,PointCloudT::Ptr &offline_sparse_cloud,std::vector<uint32_t> &u_buf,std::vector<uint32_t> &v_buf)
{
   //解析视差图
    const size_t disp_total_bytes = rgb_width * rgb_height * sizeof(uint16_t);
    std::vector<uint8_t> disp_buf(disp_total_bytes, 0);
    {
        std::ifstream disp_fs(disp_path, std::ios::binary);
        if (!disp_fs.is_open())
        {
            printf("Open disp raw failed: %s\n", disp_path.c_str());
            return false;
        }
                
        disp_fs.seekg(0, std::ios::end);
        size_t file_size = disp_fs.tellg();
        disp_fs.seekg(0, std::ios::beg);

        if (file_size != disp_total_bytes)
        {
            printf("DISP size mismatch, expect:%zu actual:%zu path:%s\n",
                        disp_total_bytes, file_size, disp_path.c_str());
            return false;
        }
        disp_fs.read(reinterpret_cast<char*>(disp_buf.data()), disp_total_bytes);
        if (!disp_fs.good())
        {
            printf("Read disp raw incomplete\n");
            return false;
        }
        disp_fs.close();
    }

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
    disp_img_pub_.publish(disp_gray_msg);
    // ====================== 生成稀疏点云 ======================
    offline_sparse_cloud->clear();
    u_buf.clear();
    v_buf.clear();
    const size_t reserve_n = (rgb_width / sample_step) * (rgb_height / sample_step);
    offline_sparse_cloud->reserve(reserve_n); // 预分配内存
    u_buf.reserve(reserve_n);
    v_buf.reserve(reserve_n);
    disp_vec.clear();
    disp_vec.reserve(reserve_n);
    double bxf = base_line * fx / 1000.0f;
    for (int v = 0; v < rgb_height; v += sample_step)
    {
        for (int u = 0; u < rgb_width; u += sample_step)
        {
            size_t pix_idx = static_cast<size_t>(v) * rgb_width + u;
            uint16_t disp_raw_val = disp_raw[pix_idx];
            float disp = static_cast<float>(disp_raw_val) / 256.0;
            float Z_m = static_cast<float>(bxf)/ disp;
            if (Z_m < 0.001f) { continue; } // 视差为 0(无效) 时 Z 为 inf，后面会被 box 挡掉
            float X_m = (static_cast<float>(u) - cx) * Z_m / fx;
            float Y_m = (static_cast<float>(v) - cy) * Z_m / fy;
            if (Z_m < box_min_z || Z_m > box_max_z
                || X_m < box_min_x || X_m > box_max_x
                || Y_m < box_min_y || Y_m > box_max_y)
            {
                continue;
            }
            offline_sparse_cloud->emplace_back(X_m, Y_m, Z_m, 0, 0, 0, 0.f, 0.f);
            u_buf.push_back(static_cast<uint32_t>(u));
            v_buf.push_back(static_cast<uint32_t>(v));
            disp_vec.push_back(disp);
        }
    }
    return true;
}
bool PercepMain::convertDispBin(const string& disp_path,PointCloudT::Ptr &offline_sparse_cloud,std::vector<uint32_t> &u_buf,std::vector<uint32_t> &v_buf)
{
    std::ifstream disp_fs(disp_path, std::ios::binary);
    if (!disp_fs.is_open())
    {
        printf("Open sparse uv‑disp bin failed: %s\n", disp_path.c_str());
        return false;
    }

    uint64_t point_num{0};
    disp_fs.read(reinterpret_cast<char*>(&point_num), sizeof(uint64_t));
    if(!disp_fs.good())
    {
        printf("read sparse disp bin header error\n");
        disp_fs.close();
        return false;
    }
    //一次性分配缓冲区
    u_buf.resize(point_num);
    v_buf.resize(point_num);
    std::vector<float> disp_buf(point_num);

    disp_fs.read(reinterpret_cast<char*>(u_buf.data()), point_num * sizeof(uint32_t));
    disp_fs.read(reinterpret_cast<char*>(v_buf.data()), point_num * sizeof(uint32_t));
    disp_fs.read(reinterpret_cast<char*>(disp_buf.data()), point_num * sizeof(float));

    if(!disp_fs.good())
    {
        printf("read bin body error, maybe file truncated\n");
        disp_fs.close();
        return false;
    }
    disp_fs.close();

    offline_sparse_cloud->clear();
    offline_sparse_cloud->reserve(point_num);
    disp_vec.clear();
    disp_vec.reserve(point_num);
    double bxf = base_line * fx / 1000.0;

    for(uint64_t i = 0; i < point_num; ++i)
    {
        uint32_t u = u_buf[i];
        uint32_t v = v_buf[i];
        float disp = disp_buf[i];

        float fu = static_cast<float>(u);
        float fv = static_cast<float>(v);

        float Z_m = static_cast<float>(bxf) / disp;
        float X_m = (fu - cx) * Z_m / fx;
        float Y_m = (fv - cy) * Z_m / fy;

        offline_sparse_cloud->emplace_back(X_m, Y_m, Z_m, 0, 0, 0, 0, 0.f);
        disp_vec.push_back(disp);
    }

    return true;
}

bool PercepMain::playFrame(size_t idx)
{
    // 索引越界保护
    if (idx >= nv12_files.size() || idx >= disp_files.size())
    {
        printf("Frame index %zu out of total frames, stop playback", idx);
        return false;
    }

    // 文件名校验帧匹配
    const std::string& nv12_name = nv12_names[idx];
    const std::string& disp_name = disp_names[idx];

    const std::string& nv12_path = nv12_files[idx];
    const std::string& disp_path = disp_files[idx];
    printf("Play frame %zu, nv12 name:%s, disp name:%s\n", idx, nv12_name.c_str(),disp_name.c_str());

    std::vector<uint8_t> rgb_buf;
    auto latest_rgb_ptr = std::make_unique<CameraData>();
    auto latest_gdc_ptr = std::make_unique<CameraGdcData>();
    if(!this->convertGDC(nv12_path,rgb_buf,&latest_rgb_ptr, &latest_gdc_ptr)){std::cerr << "convertGDC error\n";return false;}
    if (type == "raw") {
        if(!this->convertDispRaw(disp_path,offline_sparse_cloud,u_vec,v_vec)){std::cerr << "convertDispRaw error\n";return false;}
    }
    else if (type == "bin")
    {    
        if(!this->convertDispBin(disp_path,offline_sparse_cloud,u_vec,v_vec)){std::cerr << "convertDispBin error\n";return false;}
    }else{
        std::cerr << "disp type error\n";
        return false;
    }

    if (offline_sparse_cloud->empty())
    {
        std::cerr << "[PclCommon] sparse cloud generate empty\n";
        return false;
    }
    percep_map = true;
    rgb_rec.store(true);
    this->run_compute(*latest_rgb_ptr,*latest_gdc_ptr); 
    frame_save_time_str_ = disp_name;
    return true;
}

}  // namespace perception


