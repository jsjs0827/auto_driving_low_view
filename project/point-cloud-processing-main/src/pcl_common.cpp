#include "percep_main/pcl_common.hpp"
#include <cmath>
#include <algorithm>


PclCommon::PclCommon() {
    // num_threads = omp_get_max_threads();
    // omp_set_num_threads(num_threads);
    // std::cout << "使用 " << num_threads << " 个线程处理点云" << std::endl;
}

//---------------------------------通用方法----------------------------------------------------
namespace fs = std::filesystem;
bool PclCommon::keepNewestFiles(const std::string& folder_path, size_t n) {
    try {
        if (!fs::exists(folder_path)) return true;

        std::vector<fs::directory_entry> files;
        for (const auto& entry : fs::directory_iterator(folder_path)) {
            if (entry.is_regular_file()) files.push_back(entry);
        }
        if (files.size() <= n) return true;

        const size_t to_delete = files.size() - n;
        std::partial_sort(
            files.begin(), files.begin() + static_cast<std::ptrdiff_t>(to_delete),
            files.end(),
            [](const fs::directory_entry& lhs, const fs::directory_entry& rhs) {
                return lhs.last_write_time() < rhs.last_write_time();
            });

        size_t deleted = 0;
        for (size_t i = 0; i < to_delete; ++i) {
            if (fs::remove(files[i].path())) ++deleted;
        }
        return deleted == to_delete;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return false;
    }
}
bool PclCommon::pointCloudToBin(const PointCloudT::Ptr& cloud, const std::string& filepath) {
    fs::path path(filepath);
    fs::path dir = path.parent_path();
    if (!dir.empty() && !fs::exists(dir)) {
        try {
            if (fs::create_directories(dir)) {
                std::cout << "已创建目录: " << dir << std::endl;
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "创建目录失败: " << e.what() << std::endl;
            return false;
        }
    }
    std::ofstream outfile(filepath, std::ios::binary);
    if (!outfile.is_open()) {
        std::cerr << "无法打开文件: " << filepath << std::endl;
        return false;
    }
    try {
        for (const auto& point : cloud->points) {
            outfile.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
        }
    } catch (const std::exception& e) {
        std::cerr << "写入文件时发生错误: " << e.what() << std::endl;
        outfile.close();
        return false;
    }

    outfile.close();
    // std::cout << "成功保存 " << cloud->size() << " 个点到: " << filepath << std::endl;
    return true;
}

std::string PclCommon::ts_to_str(double ts) {
    // 整数部分：秒
    std::time_t seconds = static_cast<std::time_t>(ts);
    // 小数部分：微秒
    double fractional = ts - seconds;
    int milliseconds = static_cast<int>(fractional * 1000);
    // 转成本地时间
    struct tm timeinfo;

    localtime_r(&seconds, &timeinfo);
    std::ostringstream oss;
    oss << std::put_time(&timeinfo, "%Y-%m-%d_%H-%M-%S");
    oss << "." << std::setfill('0') << std::setw(3) << milliseconds;
    return oss.str();
}
void PclCommon::compute_now(uint64_t& sec, uint32_t& nsec) {
  auto now = std::chrono::system_clock::now();                                                     // 获取当前时间点
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();  // 转换为纳秒
  // 转换为秒+纳秒
  sec = ns / 1'000'000'000;
  nsec = ns % 1'000'000'000;
}
PointCloudT::Ptr PclCommon::eigenMatrixToPointCloud(const Eigen::MatrixX3f& mat) {
  PointCloudT::Ptr cloud(new PointCloudT);
  cloud->resize(mat.rows());  // 预分配空间提高效率
  for (int i = 0; i < mat.rows(); ++i) {
    cloud->points[i].x = mat(i, 0);
    cloud->points[i].y = mat(i, 1);
    cloud->points[i].z = mat(i, 2);
  }

  return cloud;
}
void PclCommon::generate_rgb(int index, uint8_t& r, uint8_t& g, uint8_t& b) {
  std::mt19937 gen(index);
  std::uniform_int_distribution<> dis(0, 255);
  r = static_cast<uint8_t>(dis(gen));
  g = static_cast<uint8_t>(dis(gen));
  b = static_cast<uint8_t>(dis(gen));
}

//---------------------------------pcl----------------------------------------------------
void PclCommon::extractCloudFromIndices(PointCloudT::Ptr &input,pcl::PointIndices::Ptr &indices,PointCloudT::Ptr &remove)
{
    if (!input || input->empty()) {
        return ;
    }

    if (!remove) {
        remove.reset(new PointCloudT);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setNegative(false);  //索引对应的点
    extract.filter(*remove);
    extract.setNegative(true);
    extract.filter(*input);
}

pcl::PointIndices::Ptr PclCommon::extractPointPlaneDis(PointCloudT::Ptr &input,double distance_threshold,Eigen::Vector3f &plane_center,Eigen::Vector3f &plane_normal,std::vector<float> &distances)
{
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    // 计算每个点到平面的有符号距离
    distances.clear();
    distances.reserve(input->size());
    for (size_t i = 0; i < input->size(); ++i) {
        Eigen::Vector3f pt(input->points[i].x, input->points[i].y, input->points[i].z);
        float distance = (pt-plane_center).dot(plane_normal);
        distances.push_back(distance);
        // 根据距离阈值分类
        if (distance <= distance_threshold) {
            indices->indices.push_back(i);
        } 
    }
    return indices;
}
// AngleCriterion
void PclCommon::ACextractBoundPoints(const PointCloudT::Ptr& cluster,
                                      PointCloudT::Ptr boundary,
                                      string searchMethod,
                                      double NormalsRadiusSearch,
                                      int NormalsKSearch,
                                      double BoundaryEstimationAngleThreshold,
                                      int BoundaryEstimationKSearch) {
  // std::cout << "points sieze is:" << cluster->size() << std::endl;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  // pcl::KdTreeFLANN<PointT> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
  // kdtree.setInputCloud(cluster);
  // float everagedistance =0;
  // for (int i =0; i < cluster->size()/2;i++)
  // {
  //         vector<int> nnh ;
  //         vector<float> squaredistance;
  //         kdtree.nearestKSearch(cluster->points[i],2,nnh,squaredistance);
  //         everagedistance += sqrt(squaredistance[1]);
  // }
  // everagedistance = everagedistance/(cluster->size()/2);

  pcl::NormalEstimation<PointT, pcl::Normal> normEst;  // 其中pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
  normEst.setInputCloud(cluster);
  normEst.setSearchMethod(tree);
  if (searchMethod == "R") {
    normEst.setRadiusSearch(NormalsRadiusSearch);  // 法向估计的半径
  }
  if (searchMethod == "K") {
    normEst.setKSearch(NormalsKSearch);  // 法向估计的点数
  } else {
    normEst.setKSearch(NormalsKSearch);
  }
  normEst.compute(*normals);
  // cout << "normal size is " << normals->size() << endl;

  pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> est;
  pcl::PointCloud<pcl::Boundary> boundaries;
  // boundaries.resize(cluster->size()); //初始化大小
  // normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
  est.setInputCloud(cluster);
  est.setInputNormals(normals);
  est.setAngleThreshold(M_PI * BoundaryEstimationAngleThreshold);
  est.setSearchMethod(tree);
  est.setKSearch(BoundaryEstimationKSearch);  // 一般这里的数值越高，最终边界识别的精度越好
  // est.setRadiusSearch(everagedistance);  //搜索半径,与setKSearch二选一
  est.compute(boundaries);

  PointCloudT::Ptr boundPoints(new PointCloudT);
  int countBoundaries = 0;
  for (int i = 0; i < cluster->size(); i++) {
    uint8_t x = (boundaries.points[i].boundary_point);
    // int a = static_cast<int>(x);
    // if (a == 1)
    // {
    //     (*boundPoints).push_back(cluster->points[i]);
    //     countBoundaries++;
    // }
    if (x > 0) {
      boundPoints->push_back(cluster->points[i]);
    }
  }
  boundary = boundPoints;
  // std::cout << "boudary size is：" << countBoundaries << std::endl;
}


Eigen::Quaternionf PclCommon::EulerToMatrix(Trans& trans,Eigen::Matrix4f& transform_matrix) {
  Eigen::Translation3f tl_btol(trans.x, trans.y, trans.z);
  Eigen::Matrix3f rotation_matrix_first, rotation_matrix_second, rotation_matrix_final;

  Eigen::AngleAxisf rollAngle(trans.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(trans.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(trans.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_target = yawAngle * pitchAngle * rollAngle;  // 万向锁，最后乘上的维度丢失
  rotation_matrix_first = q_target.toRotationMatrix();

  Eigen::AngleAxisf rollAngle2(trans.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle2(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle2(0.0, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_target2 = yawAngle2 * pitchAngle2 * rollAngle2;  // 相当于第一次旋转后沿当前的x,y,z轴顺序转
  rotation_matrix_second = q_target2.toRotationMatrix();
  // slerp插值
  //  Eigen::Quaterniond q_ori(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  //  Eigen::Quaterniond q_interpolated = q_ori.slerp(1, q_target);
  double pitch = rad_to_angle(trans.pitch);
  if ((60 <= pitch && pitch <= 120) || (-60 >= pitch && pitch >= -120)) {
    rotation_matrix_final = rotation_matrix_second * (yawAngle * pitchAngle).toRotationMatrix();
  } else {
    rotation_matrix_final = rotation_matrix_first;
  }
  Eigen::Quaternionf q_final(rotation_matrix_final);
  transform_matrix = (tl_btol * rotation_matrix_final).matrix();
  return q_final;
}

double  PclCommon::calculate_by_normal(PointCloudT pcl_in,float &roll,float &pitch,float &yaw)
{

    PointCloudT::Ptr filtered_cloud(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(pcl_in.makeShared());
    voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f); // 设置滤波参数
    voxel_filter.filter(*filtered_cloud);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05); // 设置距离阈值

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        return 0;
    }

    PointCloudT::Ptr ground_cloud(new PointCloudT);
    copyPointCloud(*filtered_cloud, *inliers, *ground_cloud);

    // 提取法线 ax + by + cz +d =0
    float a = coefficients->values[0]; // 法线的x分量
    float b = coefficients->values[1]; // 法线的y分量
    float c = coefficients->values[2]; // 法线的z分量
    float d = coefficients->values[3]; // 
    // 6. 使用最小二乘法估计roll和pitch角
    double dis = abs(d) / sqrt(a * a + b * b + c * c);
    // cout<<"AutoCalibrated Height:"<<dis-0.01<<endl;
    double height = dis-0.01;
    roll = atan2(b, c);
    pitch = atan2(-a, sqrt(b * b + c * c));
    yaw = atan2(d,a);
    // cout<<"Original Roll:"<<rad_to_angle(roll)<<"  Pitch: "<<rad_to_angle(pitch)<<" Yaw: "<<rad_to_angle(yaw)<<endl;
    return height;
}


// B样条拟合
// void PclCommon::fitBSplineToContour(PointCloudT::Ptr contour,
//                                      RGBPointCloudT::Ptr& output,
//                                      int degree,
//                                      int resolution,
//                                      double smoothness,
//                                      double rScale,
//                                      int control_points) {
//   if (contour->empty() || contour->points.size() < 3) {
//     std::cerr << "错误：输入点云为空！" << std::endl;
//     return;
//   }
//   // 检查用于拟合的初始点云
//   pcl::PCA<PointT> pca;
//   pca.setInputCloud(contour);
//   Eigen::Vector3f eigenvalues = pca.getEigenValues();
//   if (eigenvalues[1] < 1e-6) {  // 如果次小特征值接近0，说明点共线
//     std::cerr << "输入点共线！" << std::endl;
//   }
//   pcl::on_nurbs::NurbsDataCurve2d curve_data;
//   // 转换为2D点(忽略z坐标)
//   curve_data.interior.clear();
//   for (const auto& pt : *contour) {
//     curve_data.interior.push_back(Eigen::Vector2d(pt.x, pt.y));
//   }

//   // 初始化NURBS曲线(degree=3为三次B样条)
//   ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dPDM::initNurbsCurve2D(degree, curve_data.interior, control_points);

//   // 拟合参数设置
//   pcl::on_nurbs::FittingCurve2dPDM::Parameter params;
//   params.smoothness = smoothness;  // 平滑项权重
//   params.rScale = rScale;          // 正则化缩放

//   pcl::on_nurbs::FittingCurve2dPDM fit_pdm(&curve_data, curve);
//   fit_pdm.assemble(params);
//   fit_pdm.solve();

//   // pcl::on_nurbs::FittingCurve2dSDM fit_sdm(&curve_data, curve);
//   // fit_sdm.assemble(params);
//   // fit_sdm.solve();

//   // pcl::on_nurbs::FittingCurve2dTDM fit_tdm(&curve_data, curve);
//   // fit_tdm.assemble(params);
//   // fit_tdm.solve();
//   pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit_pdm.m_nurbs, output, resolution);

//   // int size = 1000;
//   // output->points.reserve(size);
//   // double t_min = fit.m_nurbs.Knot(0);
//   // double t_max = fit.m_nurbs.Knot(fit.m_nurbs.KnotCount()-1);
//   // double step = (t_max - t_min) / (size - 1);

//   // for (int i = 0; i < size; i++) {
//   //     double t = t_min + i * step;
//   //     ON_2dPoint p2d;
//   //     fit.m_nurbs.Evaluate(t, 0, 2, &p2d.x);
//   //     output->points.push_back(RGBPointT(p2d.x, p2d.y, 0,0,0,0));
//   // }
// }

void PclCommon::CubeFilter(PointCloudT::Ptr &cloud,Cube cube,bool mask)
{
    if (cloud->empty()){return;}
    //立方体滤波
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *pcl_cloud);
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setNegative(mask); //true滤除，false保留
    crop.setMin(Eigen::Vector4f(cube.min_x, cube.min_y, cube.min_z, 1.0));
    crop.setMax(Eigen::Vector4f(cube.max_x, cube.max_y, cube.max_z, 1.0));

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    crop.setInputCloud(pcl_cloud);
    crop.filter(*cloud_filtered);
    pcl::fromPCLPointCloud2(*cloud_filtered, *cloud);
    
}

void PclCommon::RadiusOutlierRemoval(PointCloudT::Ptr &cloud_in,double RadiusSearch,int MinNeighborsInRadius)
{
  if (cloud_in->empty()){return;}
  int original_size_r = cloud_in->points.size();
  // 半径离群点去除
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(cloud_in);
  ror.setRadiusSearch(RadiusSearch);                  // 设置搜索半径
  ror.setMinNeighborsInRadius(MinNeighborsInRadius);  // 设置半径内最少点数
  ror.filter(*cloud_in);                              
  int filtered_size_r = cloud_in->points.size();
  // 检查滤波效果
  float removal_ratio_radius = (original_size_r - filtered_size_r) / float(original_size_r);
//   cout<<"removal_ratio_radius: "<<removal_ratio_radius<<endl;
  // if (removal_ratio_radius < 0.05) {
  //     // 移除点太少，参数需严格
  //     ror.setMinNeighborsInRadius(ror.getMinNeighborsInRadius() * 1.2);
  // } 
  // else if (removal_ratio_radius > 0.2) {
  //     // 移除点太多，参数需宽松
  //     ror.setMinNeighborsInRadius(ror.getMinNeighborsInRadius() * 0.8);
  // }
}

void PclCommon::StatisticalOutlierRemoval(PointCloudT::Ptr &cloud_in,int intmeanK,double stddevMulThresh)
{

  if (cloud_in->empty()){return;}
  // 统计离群点去除
  int original_size_s = cloud_in->points.size();
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud_in);
  sor.setMeanK(intmeanK);                   // 设置每个点的邻域点数
  sor.setStddevMulThresh(stddevMulThresh);  // 设置标准差乘数阈值
  sor.filter(*cloud_in);                    
//   int filtered_size_s = cloud_in->points.size();
  // 检查滤波效果
//   float removal_ratio_statistic = (original_size_s - filtered_size_s) / float(original_size_s);
//   cout<<"removal_ratio_statistic: "<<removal_ratio_statistic<<endl;
  // if (removal_ratio_statistic < 0.05) {
  //     // 移除点太少，参数需严格
  //     sor.setStddevMulThresh(sor.getStddevMulThresh() * 0.8);
  // } 
  // else if (removal_ratio_statistic > 0.2) {
  //     // 移除点太多，参数需宽松
  //     sor.setStddevMulThresh(sor.getStddevMulThresh() * 1.2);
  // }
}

void PclCommon::DownSample(PointCloudT::Ptr &cloud_in, double resolution,string method) {
    if (!cloud_in || cloud_in->empty())
  {
      std::cerr << "[DownSample] input cloud empty, skip filter" << std::endl;
      return;
  }
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  if(method == "voxel")
  {
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(cloud_in);
    filter.setLeafSize(resolution, resolution, resolution);
    filter.filter(*cloud_filtered);
    cloud_in.swap(cloud_filtered);
  }
  else if (method == "uniform")
  {
    pcl::UniformSampling<PointT> filter;
    filter.setInputCloud(cloud_in);
    filter.setRadiusSearch(resolution);
    filter.filter(*cloud_filtered);   
    cloud_in.swap(cloud_filtered); 
  }
  else if (method == "approximate")
  {
    pcl::ApproximateVoxelGrid<PointT> filter;
    filter.setLeafSize(resolution, resolution, resolution);
    filter.setInputCloud(cloud_in);
    filter.filter(*cloud_filtered);   
    cloud_in.swap(cloud_filtered); 
  }
  
}

PointCloudT::Ptr PclCommon::passThrough_filter(PointCloudT& pcl_remain,string direction, double min, double max) {
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  PointCloudT::Ptr cloud_removed(new PointCloudT);    
  if (pcl_remain.points.empty()) {return cloud_removed;}
  //直通过滤z方向
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (pcl_remain.makeShared());
  pass.setFilterFieldName (direction);
  pass.setNegative (false);    //true为过滤范围，false为保留范围，默认为false
  pass.setFilterLimits (min, max);
  pass.filter (*cloud_filtered);
  pcl_remain = *cloud_filtered;
  pass.setNegative (true);
  pass.filter (*cloud_removed);
  return cloud_removed;
}

// 将3D点云投影到XY平面
void PclCommon::projectTo2D(PointCloudT::Ptr cloud) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = 0;  // 平面法向量x
  coefficients->values[1] = 0;  // 平面法向量y
  coefficients->values[2] = 1;  // 平面法向量z
  coefficients->values[3] = 0;  // 平面偏移

  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud);
}

bool PclCommon::yuv_nv12_to_rgb24(const uint8_t *data, int w, int h, int stride_y, std::vector<uint8_t> *rgb_out)
{
    if (!data || w <= 0 || h <= 0 || (w & 1) || (h & 1))
        return false;
    if (stride_y < w) stride_y = w;

    const size_t total_rgb = static_cast<size_t>(w) * h * 3;
    if (rgb_out->capacity() < total_rgb)
        rgb_out->reserve(total_rgb);
    rgb_out->resize(total_rgb);
    uint8_t* dst = rgb_out->data();

    const uint8_t* y_ptr = data;
    const uint8_t* uv_ptr = data + static_cast<size_t>(stride_y) * h;

    const int half_h = h >> 1;
    const int half_w = w >> 1;

    for (int y_block = 0; y_block < half_h; ++y_block)
    {
        const uint8_t* y0 = y_ptr + (y_block * 2) * stride_y;
        const uint8_t* y1 = y_ptr + (y_block * 2 + 1) * stride_y;
        const uint8_t* uv = uv_ptr + y_block * stride_y;

        uint8_t* dst0 = dst + (y_block * 2 * w) * 3;
        uint8_t* dst1 = dst + ((y_block * 2 + 1) * w) * 3;

        for (int x_block = 0; x_block < half_w; ++x_block)
        {
            int x_off = x_block << 1;
            uint8_t u = uv[x_off];
            uint8_t v = uv[x_off + 1];

            // 上一行两个像素
            yuv601_to_rgb(y0[x_off],     u, v, dst0, dst0+1, dst0+2); dst0 +=3;
            yuv601_to_rgb(y0[x_off + 1], u, v, dst0, dst0+1, dst0+2); dst0 +=3;
            // 下一行两个像素
            yuv601_to_rgb(y1[x_off],     u, v, dst1, dst1+1, dst1+2); dst1 +=3;
            yuv601_to_rgb(y1[x_off + 1], u, v, dst1, dst1+1, dst1+2); dst1 +=3;
        }
    }
    return true;
}


std::vector<pcl::PointIndices> PclCommon::euclideanClustering(PointCloudT::Ptr &input_cloud,double cluster_tolerance , int min_cluster_size, int max_cluster_size) {
    std::vector<pcl::PointIndices> cluster_indices;
    if (!input_cloud || input_cloud->empty()) { return cluster_indices;}

    // 使用KdTree计算最近邻距离
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(input_cloud);

    std::vector<int> indices(2);
    std::vector<float> distances(2);
    float total_distance = 0.0f;

    for (size_t i = 0; i < input_cloud->size(); ++i) {
        if (kdtree.nearestKSearch(input_cloud->points[i], 2, indices, distances) > 0) {
            total_distance += sqrt(distances[1]);  // 最近邻的距离
        }
    }
    float avg_spacing = total_distance / input_cloud->size();
    float cluster_tolerance_adaptive = avg_spacing * 4.0;  // 2-3倍平均间距
    cout<<"cluster_tolerance_adaptive:"<<cluster_tolerance_adaptive<<endl;

    // 创建KD树
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input_cloud);
    
    // 欧氏聚类
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_adaptive); // 聚类距离阈值
    ec.setMinClusterSize(min_cluster_size);    // 最小聚类点数
    ec.setMaxClusterSize(max_cluster_size);    // 最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);
    return cluster_indices;
}

bool PclCommon::rgb8_resize(const std::vector<uint8_t>& src_buf,
                               int src_w, int src_h,
                               std::vector<uint8_t>& dst,
                               int dst_w, int dst_h)
{
    const int wS = src_w, hS = src_h;
    const int wD = dst_w, hD = dst_h;

    // 基础合法性校验（保留原有全部校验）
    if (src_buf.empty() || wS <= 0 || hS <= 0 || wD <= 0 || hD <= 0)
    {
        fprintf(stderr, "[rgb8_resize] invalid src buf or size src(%d,%d) dst(%d,%d)\n", wS, hS, wD, hD);
        dst.clear();
        return false;
    }

    const int MAX_RES = 8192;
    if (wS > MAX_RES || hS > MAX_RES || wD > MAX_RES || hD > MAX_RES)
    {
        fprintf(stderr, "[rgb8_resize] over max res src(%d,%d) dst(%d,%d)\n", wS, hS, wD, hD);
        dst.clear();
        return false;
    }

    const size_t pixelPerLineSrc = static_cast<size_t>(wS) * 3;
    const size_t srcTotalBytes = pixelPerLineSrc * static_cast<size_t>(hS);
    // 乘法溢出校验
    if (srcTotalBytes / static_cast<size_t>(hS) != pixelPerLineSrc)
    {
        fprintf(stderr, "[rgb8_resize] src size multiply overflow wS=%d hS=%d\n", wS, hS);
        dst.clear();
        return false;
    }
    if (src_buf.size() != srcTotalBytes)
    {
        fprintf(stderr, "[rgb8_resize] src buffer size mismatch expect:%zu real:%zu\n", srcTotalBytes, src_buf.size());
        dst.clear();
        return false;
    }

    // 原图和目标图尺寸完全一致，直接拷贝
    if (wS == wD && hS == hD)
    {
        dst = src_buf;
        return true;
    }

    // 包装src_buf为cv::Mat，不拷贝内存（外部vector生命周期要有效）
    cv::Mat src_mat(hS, wS, CV_8UC3, const_cast<uint8_t*>(src_buf.data()));

    // OpenCV resize，双线性插值，与原手写算法保持一致
    cv::Mat dst_mat;
    cv::resize(src_mat, dst_mat, cv::Size(dst_w, dst_h), 0, 0, cv::INTER_LINEAR);

    // 把cv::Mat拷贝回std::vector<uint8_t>
    dst.assign(dst_mat.data, dst_mat.data + dst_mat.total() * dst_mat.elemSize());

    return true;
}
// 使用预先转换好的 RGB 缓冲对点云着色
// rgb_buf 格式：行主序 RGB24，每像素 3 字节 (R, G, B)
// src_cloud 中点的坐标单位为米
bool PclCommon::ProjectRGBFromBuffer(
    PointCloudT::Ptr& src_cloud,
    const std::vector<uint8_t>& rgb_buf,
    int img_w, int img_h,
    float fx, float fy, float cx, float cy,
    int* colored_out)
{
    if (rgb_buf.empty() || src_cloud->empty()) return false;
    if (static_cast<int>(rgb_buf.size()) < img_w * img_h * 3) return false;

    int colored = 0;
    for (auto& pt : src_cloud->points) {
        if (pt.z < 1e-6f) {
            pt.r = pt.g = pt.b = 0;
            continue;
        }
        const float u_f = (pt.x * fx) / pt.z + cx;
        const float v_f = (pt.y * fy) / pt.z + cy;
        const int u = static_cast<int>(u_f);
        const int v = static_cast<int>(v_f);

        if (u < 0 || u >= img_w || v < 0 || v >= img_h) {
            pt.r = pt.g = pt.b = 0;
            
            continue;
        }
        const size_t pix_off = (static_cast<size_t>(v) * img_w + u) * 3;
        pt.r = rgb_buf[pix_off];
        pt.g = rgb_buf[pix_off + 1];
        pt.b = rgb_buf[pix_off + 2];
        ++colored;
        
    }
    if (colored_out) {
        *colored_out = colored;
    }
    return true;
}

bool PclCommon::AssignRGBByUV(
    PointCloudT::Ptr& src_cloud,
    const std::vector<uint8_t>& rgb_buf,
    const std::vector<uint32_t>& u_vec,
    const std::vector<uint32_t>& v_vec,
    int img_w, int img_h)
{
    if (rgb_buf.empty() || src_cloud->empty()) return false;
    if (src_cloud->points.size() != u_vec.size() || src_cloud->points.size() != v_vec.size())
    {
        LOG_ERROR("point cloud size not match u_vec/v_vec!");
        return false;
    }
    if (static_cast<int>(rgb_buf.size()) < img_w * img_h * 3) return false;

    int colored = 0;
    const size_t total_pixel_bytes = static_cast<size_t>(img_w) * 3;

    for (size_t i = 0; i < src_cloud->points.size(); ++i)
    {
        auto& pt = src_cloud->points[i];
        uint32_t u = u_vec[i];
        uint32_t v = v_vec[i];

        // 边界检查
        if (u >= static_cast<uint32_t>(img_w) || v >= static_cast<uint32_t>(img_h))
        {
            pt.r = pt.g = pt.b = 0;
            continue;
        }

        // 原图行主序RGB24偏移
        size_t pix_off = static_cast<size_t>(v) * img_w * 3 + static_cast<size_t>(u) * 3;
        pt.r = rgb_buf[pix_off];
        pt.g = rgb_buf[pix_off + 1];
        pt.b = rgb_buf[pix_off + 2];
        colored++;
    }

    return true;
}
