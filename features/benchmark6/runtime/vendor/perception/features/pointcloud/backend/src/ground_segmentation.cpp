#include "mapping_algorithms.hpp"
namespace ai_node_percep_demo {
void MappingAlgorithms::runGroundSegmentation(const PointCloudT::Ptr& raw_pts) 
{
    if (!patchwork_ || raw_pts->empty()) return;
    auto t_pw = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf cloud_mat(raw_pts->size(), 3);  
    for (size_t i = 0; i < raw_pts->size(); ++i) {
      cloud_mat.row(i) << raw_pts->points[i].x, raw_pts->points[i].y, raw_pts->points[i].z;
    }
    ground_cloud.reset(new PointCloudT);
    no_ground_cloud.reset(new PointCloudT);
    patchwork_->estimateGround(cloud_mat);
    Eigen::MatrixX3f ground = patchwork_->getGround();
    Eigen::MatrixX3f nonground = patchwork_->getNonground();
        
    vector<Eigen::VectorXf> ground_normals = patchwork_->getNormalsVec();
    vector<Eigen::VectorXf> ground_centers = patchwork_->getCentersVec();
    vector<double> d_vec = patchwork_->getDVec();    
    ground_cloud = pclCommonPtr->eigenMatrixToPointCloud(ground);
    no_ground_cloud = pclCommonPtr->eigenMatrixToPointCloud(nonground);
    double ground_dist = 0.05;
    if (!ground_normals.empty() && !ground_centers.empty())
    {
      
        double ground_pitch = std::acos(std::abs(ground_normals[0](2))) * 180.0 / M_PI;
        Eigen::Vector3f ground_normal = ground_normals[0].head<3>();
        Eigen::Vector3f ground_center = ground_centers[0].head<3>();  

        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
        PointCloudT::Ptr ground_cloud_extra(new PointCloudT);
        std::vector<float> distances;

        ground_indices = pclCommonPtr->extractPointPlaneDis(no_ground_cloud,ground_dist,ground_center,ground_normal,distances);
        pclCommonPtr->extractCloudFromIndices(no_ground_cloud,ground_indices,ground_cloud_extra);
        *ground_cloud = *ground_cloud + *ground_cloud_extra;

        // cout<<"地面提取成功，采用地面法向量滤波保护"<<endl;       
        if (!distances.empty()){
          float min_dist = *std::min_element(distances.begin(), distances.end());
          float max_dist = *std::max_element(distances.begin(), distances.end());
        //   cout<<"ground extract: min_dist:"<<min_dist<<",max_dist:"<<max_dist<<endl;      
        }  
      
    }
    else{
        // cout<<"地面提取失败，采用直通滤波滤波保护"<<endl;
        PointCloudT::Ptr ground_cloud_extra2 = pclCommonPtr->passThrough_filter(*no_ground_cloud,"z",ground_dist,100); 
        *ground_cloud = *ground_cloud + *ground_cloud_extra2;
    }

    {
      auto t_pw_end = std::chrono::high_resolution_clock::now();
      auto pw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          t_pw_end - t_pw);
      std::cout << "patchwork Time taken: " << pw_ms.count() << "ms"
                << std::endl;
    }

auto start = std::chrono::high_resolution_clock::now();    
    int MinClusterSize = 20;
    int MaxClusterSize = 20000;
    float cluster_tolerance = 0.1;
    pclCommonPtr->DownSample(no_ground_cloud,0.04,"uniform");
    cout<<"后处理滤波前的非地面点云数量："<<no_ground_cloud->points.size()<<endl;
    pclCommonPtr->RadiusOutlierRemoval(no_ground_cloud,0.1,10);
    pclCommonPtr->StatisticalOutlierRemoval(no_ground_cloud,20,1.0);   
        
    // pclCommonPtr->DownSample(ground_cloud,0.04,"uniform");
    // cout<<"后处理滤波前的地面点云数量："<<ground_cloud->points.size()<<endl;
    // pclCommonPtr->RadiusOutlierRemoval(ground_cloud,0.05,5);
    // pclCommonPtr->StatisticalOutlierRemoval(ground_cloud,20,1.0);   

    // auto obs_indices = pclCommonPtr->euclideanClustering(no_ground_cloud,cluster_tolerance,MinClusterSize,MaxClusterSize);
    // for (size_t i = 0; i < obs_indices.size(); ++i)
    // {
    //     pcl::PointIndices::Ptr indices_ptr = std::make_shared<pcl::PointIndices>(obs_indices[i]);
    //     PointCloudT::Ptr cluster_cloud(new PointCloudT);
    //     pclCommonPtr->extractCloudFromIndices(no_ground_cloud,indices_ptr,cluster_cloud);
    //     *obs_cloud += *cluster_cloud;
    // }
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end- start);
cout<<"filter Time taken: "<<duration.count()<<"ms"<<endl;        
    obs_cloud = no_ground_cloud;


}

}
