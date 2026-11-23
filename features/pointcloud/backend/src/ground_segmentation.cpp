#include "mapping_algorithms.hpp"

namespace ai_node_percep_demo {
void MappingAlgorithms::runGroundSegmentation(const PointCloudT::Ptr& raw_pts)
{
    if (!patchwork_ || raw_pts->empty()) return;
    const auto t_pw = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf cloud_mat(raw_pts->size(), 3);
    for (size_t i = 0; i < raw_pts->size(); ++i) {
        cloud_mat.row(i) << raw_pts->points[i].x,
                           raw_pts->points[i].y,
                           raw_pts->points[i].z;
    }
    ground_cloud.reset(new PointCloudT);
    no_ground_cloud.reset(new PointCloudT);
    patchwork_->estimateGround(cloud_mat);
    const Eigen::MatrixX3f ground = patchwork_->getGround();
    const Eigen::MatrixX3f nonground = patchwork_->getNonground();
    [[maybe_unused]] const auto ground_normals = patchwork_->getNormalsVec();
    [[maybe_unused]] const auto ground_centers = patchwork_->getCentersVec();
    [[maybe_unused]] const auto d_vec = patchwork_->getDVec();
    ground_cloud = pclCommonPtr->eigenMatrixToPointCloud(ground);
    no_ground_cloud = pclCommonPtr->eigenMatrixToPointCloud(nonground);

    // V0 1104: retain [obs_height_thr, 100] in no_ground_cloud and add
    // the removed points to ground. This tests robot Z, not plane distance.
    PointCloudT::Ptr ground_cloud_extra = pclCommonPtr->passThrough_filter(
        *no_ground_cloud, "z", obs_height_thr, 100);
    *ground_cloud = *ground_cloud + *ground_cloud_extra;

    const auto t_pw_end = std::chrono::high_resolution_clock::now();
    std::cout << "patchwork Time taken: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     t_pw_end - t_pw).count()
              << "ms" << std::endl;

    const auto start = std::chrono::high_resolution_clock::now();
    // The selected V0 has ramp_detect=false. Its ordinary obstacle path
    // downsamples only above 3000 points and applies SOR without an ROR pass.
    if (no_ground_cloud->size() > 3000) {
        pclCommonPtr->DownSample(no_ground_cloud, 0.04, "uniform");
    }
    pclCommonPtr->StatisticalOutlierRemoval(no_ground_cloud, 20, 1.0);
    obs_cloud = no_ground_cloud;

    std::cout << "filter Time taken: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::high_resolution_clock::now() - start).count()
              << "ms" << std::endl;
}
}  // namespace ai_node_percep_demo
