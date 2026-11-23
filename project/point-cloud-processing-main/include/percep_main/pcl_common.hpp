#ifndef PCL_COMMON_H
#define PCL_COMMON_H


#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/geometry/quad_mesh.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/registration/icp.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
// #include <pcl/surface/on_nurbs/fitting_surface_pdm.h>
// #include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
// #include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
// #include <pcl/surface/on_nurbs/fitting_curve_2d_tdm.h>
// #include <pcl/surface/on_nurbs/nurbs_tools.h>
// #include <pcl/surface/on_nurbs/fitting_curve_2d.h>
// #include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/concave_hull.h>

// #include <boost/lexical_cast.hpp>
// #include <boost/lambda/lambda.hpp>
// #include <boost/timer/timer.hpp>

#include "percep_main/patchworkpp.hpp"
#include "third_party/MoTypes.h"
#include <opencv2/opencv.hpp>
#include "percep_main/common.hpp"


class PclCommon {
 public:
  PclCommon();
  bool debug;
  int num_threads;
  std::string ts_to_str(double ts);
  void compute_now(uint64_t& sec, uint32_t& nsec);
  Eigen::Quaternionf EulerToMatrix(Trans& trans,Eigen::Matrix4f& transform_matrix);                                  
  void CubeFilter(PointCloudT::Ptr &cloud,Cube cube,bool mask);
  void RadiusOutlierRemoval(PointCloudT::Ptr &cloud_in,double RadiusSearch,int MinNeighborsInRadius);
  void StatisticalOutlierRemoval(PointCloudT::Ptr &cloud_in,int intmeanK,double stddevMulThresh);
  void DownSample(PointCloudT::Ptr &cloud_in, double resolution,string method);
  void ACextractBoundPoints(const PointCloudT::Ptr& cluster,
                            PointCloudT::Ptr boundary,
                            string searchMethod,
                            double NormalsRadiusSearch,
                            int NormalsKSearch,
                            double BoundaryEstimationAngleThreshold,
                            int BoundaryEstimationKSearch);
  PointCloudT::Ptr passThrough_filter(PointCloudT& pcl_in, string direction, double min, double max);
  void projectTo2D(PointCloudT::Ptr cloud);
  double  calculate_by_normal(PointCloudT pcl_in,float &roll,float &pitch,float &yaw);
  // void fitBSplineToContour(PointCloudT::Ptr contour,
  //                          RGBPointCloudT::Ptr& output,
  //                          int degree,
  //                          int resolution,
  //                          double smoothness,
  //                          double rScale,
  //                          int control_points);

  void generate_rgb(int index, uint8_t& r, uint8_t& g, uint8_t& b);
  PointCloudT::Ptr eigenMatrixToPointCloud(const Eigen::MatrixX3f& mat);

  bool keepNewestFiles(const std::string& folder_path, size_t n);
  bool pointCloudToBin(const PointCloudT::Ptr& cloud, const std::string& filepath);
  pcl::PointIndices::Ptr extractPointPlaneDis(PointCloudT::Ptr &input,double distance_threshold,Eigen::Vector3f &plane_center,Eigen::Vector3f &plane_normal,std::vector<float> &distances);
  void extractCloudFromIndices(PointCloudT::Ptr &input,pcl::PointIndices::Ptr &indices,PointCloudT::Ptr &remove);

  // 使用预先转换好的 RGB 缓冲对点云着色（点坐标单位：米）
  bool ProjectRGBFromBuffer(PointCloudT::Ptr& src_cloud,
                             const std::vector<uint8_t>& rgb_buf,
                             int img_w, int img_h,
                             float fx, float fy, float cx, float cy,
                             int* colored_out = nullptr);
  bool AssignRGBByUV(
    PointCloudT::Ptr& src_cloud,
    const std::vector<uint8_t>& rgb_buf,
    const std::vector<uint32_t>& u_vec,
    const std::vector<uint32_t>& v_vec,
    int img_w, int img_h);
//   uint8_t clip_u8(int v); 
//   void yuv601_to_rgb(int y, int u, int v, uint8_t *r, uint8_t *g, uint8_t *b);
  bool yuv_nv12_to_rgb24(const uint8_t *data, int w, int h, int stride_y, vector<uint8_t> *rgb_out); 
  bool rgb8_resize(const std::vector<uint8_t>& src_buf,
                               int src_w, int src_h,
                               std::vector<uint8_t>& dst,
                               int dst_w, int dst_h);
  std::vector<pcl::PointIndices> euclideanClustering(PointCloudT::Ptr &input_cloud,double cluster_tolerance , int min_cluster_size, int max_cluster_size); 
                 
private:
    inline uint8_t clip_u8(int val) {
        return static_cast<uint8_t>(val < 0 ? 0 : (val > 255 ? 255 : val));
    }

    inline void yuv601_to_rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
        const int c = static_cast<int>(y) - 16;
        const int d = static_cast<int>(u) - 128;
        const int e = static_cast<int>(v) - 128;
        const int c298 = 298 * c;
        *r = clip_u8((c298 + 409 * e + 128) >> 8);
        *g = clip_u8((c298 - 100 * d - 208 * e + 128) >> 8);
        *b = clip_u8((c298 + 516 * d + 128) >> 8);
    }
};



#endif