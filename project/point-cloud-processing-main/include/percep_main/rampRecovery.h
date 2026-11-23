#pragma once

#include "percep_main/pcl_common.hpp"
#include "percep_main/visualize.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <vector>


class RampRecovery : public Visualize
{
public:
    enum class Type
    {
        OBSTACLE = 0,
        RAMP,
        STEP   // 保留 ABI/API 兼容；V2 暂时不做台阶检测
    };

    struct Params
    {
        float cluster_tolerance = 0.1f;
        int min_cluster_size = 50;
        int max_cluster_size = 20000;

        float plane_dist_thr = 0.045f;
        float min_plane_ratio = 0.8f;
        float max_plane_rmse = 0.025f;

        float ramp_min_slope_deg = 0.0f;
        float ramp_max_slope_deg = 25.0f;
        float ramp_min_length = 0.20f;
        float ramp_max_length = 2.50f;
        float ramp_min_width = 0.20f;
        float ramp_max_width = 2.50f;
        float ramp_min_aspect_ratio = 0.9f;
        float ramp_min_height_gain = 0.0f;
        float ramp_max_height_gain = 1.20f;

        float profile_bin_size = 0.08f;
        int min_profile_bins = 5;
        int max_profile_bins = 32;
        float ramp_profile_rmse = 0.035f;
        float max_profile_slope_error_deg = 6.0f;
        float ramp_max_height_jump = 0.06f;

        float min_expected_gain_ratio = 0.0f;
        float max_expected_gain_ratio = 3.0f;
        float max_width_variation_ratio = 0.45f;

        float ground_connect_dist = 0.12f;
        int min_ground_connect_points = 4;

        int max_input_points = 3000;
        int max_feature_points = 2000;
        float min_plane_planarity = 0.2f;
        // RANSAC 默认关闭：PCA + 迭代内点重拟合更适合 X5 CPU。
        bool enable_ransac = false;
        int ransac_max_iter = 50;

         // 已知车体坐标系前向轴时可开启，可进一步过滤侧向倾斜物体。
        bool use_forward_axis = false;
        float forward_axis_x = 1.0f;
        float forward_axis_y = 0.0f;
        float min_forward_cos = 0.50f;
        // 真正坡道应具有“低端接地 + 中段连续上升 + 高端平台/近水平段”。
        float platform_min_height_gain = 0.08f;
        float platform_max_slope_deg = 6.0f;
        int min_platform_bins = 2;
        // 单帧 watchdog。建议 RampRecovery 自身控制在 35~45ms。
        double time_budget_ms = 60.0;

        bool enable_visualization = true;
        bool verbose = true;
    };

    struct Result
    {
        PointCloudT::Ptr ramp;
        PointCloudT::Ptr step;
        PointCloudT::Ptr obstacle;
        double slope_angle = -1;
        Result()
        {
            ramp.reset(new PointCloudT);
            step.reset(new PointCloudT);
            obstacle.reset(new PointCloudT);
        }

        void clear()
        {
            ramp->clear();
            step->clear();
            obstacle->clear();
        }
    };

    RampRecovery();
    // explicit RampRecovery(const Params& params);

    void setVisualize(bool enable);
    const Params& getParams() const { return params_; }

    /**
     * @brief nonground -> RAMP / OBSTACLE。
     * STEP 类型保留接口，但 V2 默认不再执行台阶检测。
     */
    void process(const PointCloudT::ConstPtr& nonground,
                 const PointCloudT::ConstPtr& ground,
                 Result& result);

private:
    struct HeightProfile
    {
        std::vector<float> position;
        std::vector<float> height;
        std::vector<int> count;
        std::vector<float> width;
    };

    struct PlaneFeature
    {
        bool valid = false;
        Eigen::Vector4f coeff = Eigen::Vector4f::Zero();
        Eigen::Vector3f normal = Eigen::Vector3f::Zero();
        float ratio = 0.0f;
        float rmse = std::numeric_limits<float>::max();
        float slope_deg = 0.0f;
        float linearity = 0.0f;
        float planarity = 0.0f;
        float scattering = 0.0f;
        float flatness = 0.0f;
        float width_ratio = 0.0f;
    };

    struct Classification
    {
        Type type = Type::OBSTACLE;
        PlaneFeature plane;

        float length = 0.0f;
        float width = 0.0f;
        float height_gain = 0.0f;
        float profile_rmse = std::numeric_limits<float>::max();
        float profile_slope_deg = 0.0f;
        float width_variation = std::numeric_limits<float>::max();
        float expected_gain_ratio = 0.0f;

        // bool ground_connected = false;
        bool platform_valid = false;
        bool forward_aligned = true;
        bool geometry_valid = false;
        bool profile_valid = false;
    };

    using Clock = std::chrono::steady_clock;

    void processCluster(const PointCloudT::Ptr& cluster,
                        const PointCloudT::ConstPtr& ground,
                        Result& result,
                        const Clock::time_point& start_time);

    Classification classify(const PointCloudT::Ptr& cluster,
                            const PointCloudT::ConstPtr& ground,
                            const Clock::time_point& start_time) const;

    PlaneFeature fitPlaneFast(const PointCloudT::Ptr& cloud) const;
    PlaneFeature fitPlaneRansac(const PointCloudT::Ptr& cloud) const;

    HeightProfile buildHeightProfile(const PointCloudT::Ptr& cloud,
                                     const Eigen::Vector2f& direction) const;

    Eigen::Vector2f getPrincipalDirection(const PointCloudT::Ptr& cloud) const;

    bool calcGeometry(const PointCloudT::Ptr& cloud,
                      const Eigen::Vector2f& direction,
                      Classification& result) const;

    bool calcProfile(const HeightProfile& profile,
                     const PlaneFeature& plane,
                     Classification& result) const;

    bool checkGroundConnection(const PointCloudT::Ptr& cloud,
                               const Eigen::Vector2f& direction,
                               const PointCloudT::ConstPtr& ground,
                               const pcl::search::KdTree<PointT>::Ptr& ground_tree) const;

    bool checkPlatform(const HeightProfile& profile,
                       const PlaneFeature& plane,
                       Classification& result) const;

    bool checkForwardAlignment(const Eigen::Vector2f& direction) const;

    PointCloudT::Ptr makeFeatureCloud(const PointCloudT::Ptr& cluster) const;

    void splitClusterByRampPlane(const PointCloudT::Ptr& cluster,
                                 const PlaneFeature& plane,
                                 Result& result) const;

    void copyClusterAsObstacle(const PointCloudT::Ptr& cluster,
                               Result& result) const;

    bool budgetExceeded(const Clock::time_point& start_time) const;

private:
    Params params_;
    daye::RosPublisher box_pub_;
};
