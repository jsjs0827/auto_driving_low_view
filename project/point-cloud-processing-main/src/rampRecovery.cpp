#include "percep_main/rampRecovery.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

namespace
{
    inline float rad2deg(float r)
    {
        return r * 180.0f / static_cast<float>(M_PI);
    }

    inline float clamp01(float x)
    {
        return std::max(0.0f, std::min(1.0f, x));
    }

    inline bool finitePoint(const PointT& p)
    {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }
    struct GridKey
    {
        int x;
        int y;
        int z;

        inline bool operator==(const GridKey& other) const noexcept
        {
            return x == other.x &&
                y == other.y &&
                z == other.z;
        }
    };
    struct GridKeyHash
    {
        inline std::size_t operator()(const GridKey& k) const noexcept
        {
            // 32-bit 整数混合
            const std::uint32_t x =
                static_cast<std::uint32_t>(k.x);

            const std::uint32_t y =
                static_cast<std::uint32_t>(k.y);

            const std::uint32_t z =
                static_cast<std::uint32_t>(k.z);

            std::uint64_t h =
                static_cast<std::uint64_t>(x) * 73856093ULL;

            h ^=
                static_cast<std::uint64_t>(y) * 19349663ULL;

            h ^=
                static_cast<std::uint64_t>(z) * 83492791ULL;

            // final mixing
            h ^= h >> 33;
            h *= 0xff51afd7ed558ccdULL;
            h ^= h >> 33;
            h *= 0xc4ceb9fe1a85ec53ULL;
            h ^= h >> 33;

            return static_cast<std::size_t>(h);
        }
    };

    template <typename PointT>
    inline GridKey pointToGrid(
        const PointT& p,
        const float cell_size) noexcept
    {
        const float inv_cell = 1.0f / cell_size;

        return GridKey{
            static_cast<int>(std::floor(p.x * inv_cell)),
            static_cast<int>(std::floor(p.y * inv_cell)),
            static_cast<int>(std::floor(p.z * inv_cell))
        };
    }

template <typename PointT>
static void hashGridCluster(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    const float tolerance,
    const int min_cluster_size,
    const int max_cluster_size,
    std::vector<pcl::PointIndices>& clusters)
{
    clusters.clear();

    if (!cloud || cloud->empty())
        return;

    if (!(tolerance > 0.0f))
        return;

    const int point_count =
        static_cast<int>(cloud->size());

    if (point_count <= 0)
        return;

    const float inv_cell = 1.0f / tolerance;
    const float tolerance_sq =
        tolerance * tolerance;

    using CellPoints = std::vector<int>;

    std::unordered_map<
        GridKey,
        CellPoints,
        GridKeyHash
    > grid;

    grid.reserve(
        static_cast<std::size_t>(
            std::max(16, point_count * 2)
        )
    );

    // ========================================================
    // 1. 建立 Hash Grid
    // ========================================================

    for (int i = 0; i < point_count; ++i)
    {
        const PointT& p = cloud->points[i];

        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
        {
            continue;
        }

        const GridKey key{
            static_cast<int>(
                std::floor(p.x * inv_cell)
            ),
            static_cast<int>(
                std::floor(p.y * inv_cell)
            ),
            static_cast<int>(
                std::floor(p.z * inv_cell)
            )
        };

        auto it = grid.find(key);

        if (it == grid.end())
        {
            CellPoints points;
            points.reserve(8);
            points.push_back(i);

            grid.emplace(
                key,
                std::move(points)
            );
        }
        else
        {
            it->second.push_back(i);
        }
    }

    // ========================================================
    // 2. visited
    // ========================================================

    std::vector<std::uint8_t> visited(
        static_cast<std::size_t>(point_count),
        0
    );

    // ========================================================
    // 3. BFS queue
    // ========================================================

    std::vector<int> bfs_queue;

    bfs_queue.reserve(
        static_cast<std::size_t>(
            std::max(64, min_cluster_size)
        )
    );

    // ========================================================
    // 4. 当前 cluster
    // ========================================================

    std::vector<int> current_cluster;

    current_cluster.reserve(
        static_cast<std::size_t>(
            std::max(64, min_cluster_size)
        )
    );

    // ========================================================
    // 5. BFS clustering
    // ========================================================

    for (int seed = 0;
         seed < point_count;
         ++seed)
    {
        const PointT& seed_point =
            cloud->points[seed];

        if (!std::isfinite(seed_point.x) ||
            !std::isfinite(seed_point.y) ||
            !std::isfinite(seed_point.z))
        {
            continue;
        }

        if (visited[seed])
            continue;

        bfs_queue.clear();
        current_cluster.clear();

        visited[seed] = 1;
        bfs_queue.push_back(seed);

        std::size_t head = 0;

        while (head < bfs_queue.size())
        {
            const int current =
                bfs_queue[head++];

            current_cluster.push_back(current);

            const PointT& p =
                cloud->points[current];

            const GridKey center{
                static_cast<int>(
                    std::floor(p.x * inv_cell)
                ),
                static_cast<int>(
                    std::floor(p.y * inv_cell)
                ),
                static_cast<int>(
                    std::floor(p.z * inv_cell)
                )
            };

            // =================================================
            // 搜索周围 27 个 cell
            // =================================================

            for (int dx = -1;
                 dx <= 1;
                 ++dx)
            {
                for (int dy = -1;
                     dy <= 1;
                     ++dy)
                {
                    for (int dz = -1;
                         dz <= 1;
                         ++dz)
                    {
                        const GridKey neighbor_key{
                            center.x + dx,
                            center.y + dy,
                            center.z + dz
                        };

                        auto cell_it =
                            grid.find(neighbor_key);

                        if (cell_it == grid.end())
                            continue;

                        const CellPoints& cell_points =
                            cell_it->second;

                        for (const int idx : cell_points)
                        {
                            if (visited[idx])
                                continue;

                            const PointT& q =
                                cloud->points[idx];

                            const float vx =
                                p.x - q.x;

                            const float vy =
                                p.y - q.y;

                            const float vz =
                                p.z - q.z;

                            const float dist_sq =
                                vx * vx +
                                vy * vy +
                                vz * vz;

                            if (dist_sq <= tolerance_sq)
                            {
                                visited[idx] = 1;
                                bfs_queue.push_back(idx);
                            }
                        }
                    }
                }
            }
        }

        // =====================================================
        // 6. Cluster size filtering
        // =====================================================

        const std::size_t cluster_size =
            current_cluster.size();

        if (cluster_size <
            static_cast<std::size_t>(
                std::max(1, min_cluster_size)))
        {
            continue;
        }

        if (max_cluster_size > 0 &&
            cluster_size >
            static_cast<std::size_t>(
                max_cluster_size))
        {
            continue;
        }

        // =====================================================
        // 7. 保存 cluster 原始索引
        // =====================================================

        pcl::PointIndices indices;

        indices.indices.swap(
            current_cluster
        );

        clusters.emplace_back(
            std::move(indices)
        );
    }
}
}  // namespace

RampRecovery::RampRecovery()
{
    box_pub_ = daye::CreatePublish<visualization_msgs::msg::MarkerArray>("/percep_main/cluster_bbox", rclcpp::QoS(5));  
}

// RampRecovery::RampRecovery(const Params& params)
//     : params_(params)
// {

// }

void RampRecovery::setVisualize(bool enable)
{
    params_.enable_visualization = enable;
}

bool RampRecovery::budgetExceeded(const Clock::time_point& start_time) const
{
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start_time).count();
    return elapsed >= params_.time_budget_ms;
}

PointCloudT::Ptr RampRecovery::makeFeatureCloud(const PointCloudT::Ptr& cluster) const
{
    PointCloudT::Ptr sampled(new PointCloudT);
    if (!cluster || cluster->empty())
        return sampled;

    const size_t n = cluster->size();
    const size_t max_n = static_cast<size_t>(std::max(100, params_.max_feature_points));

    if (n <= max_n)
    {
        *sampled = *cluster;
        return sampled;
    }

    sampled->reserve(max_n);

    // 均匀抽样，避免只取 cluster 前部导致 profile 偏置。
    for (size_t i = 0; i < max_n; ++i)
    {
        const size_t idx = (i * (n - 1)) / (max_n - 1);
        if (finitePoint(cluster->points[idx]))
            sampled->push_back(cluster->points[idx]);
    }

    sampled->width = static_cast<uint32_t>(sampled->size());
    sampled->height = 1;
    sampled->is_dense = true;
    return sampled;
}

Eigen::Vector2f RampRecovery::getPrincipalDirection(const PointCloudT::Ptr& cloud) const
{
    Eigen::Vector2f dir(1.0f, 0.0f);
    if (!cloud || cloud->size() < 3)
        return dir;

    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    int count = 0;

    for (const auto& p : cloud->points)
    {
        if (!finitePoint(p))
            continue;
        mean.x() += p.x;
        mean.y() += p.y;
        ++count;
    }

    if (count < 3)
        return dir;

    mean /= static_cast<float>(count);

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : cloud->points)
    {
        if (!finitePoint(p))
            continue;

        const Eigen::Vector2f q(p.x - mean.x(), p.y - mean.y());
        cov.noalias() += q * q.transpose();
    }

    cov /= static_cast<float>(std::max(1, count - 1));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    if (solver.info() != Eigen::Success)
        return dir;

    dir = solver.eigenvectors().col(1);
    if (dir.norm() < 1e-6f)
        return Eigen::Vector2f(1.0f, 0.0f);

    dir.normalize();
    return dir;
}

// RampRecovery::PlaneFeature RampRecovery::fitPlaneFast(const PointCloudT::Ptr& cloud) const
// {
//     PlaneFeature feature;
//     if (!cloud || cloud->size() < 10)
//         return feature;

//     Eigen::Vector3f mean = Eigen::Vector3f::Zero();
//     int count = 0;

//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;

//         mean += Eigen::Vector3f(p.x, p.y, p.z);
//         ++count;
//     }

//     if (count < 10)
//         return feature;

//     mean /= static_cast<float>(count);

//     Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;

//         const Eigen::Vector3f q(p.x - mean.x(),
//                                 p.y - mean.y(),
//                                 p.z - mean.z());
//         cov.noalias() += q * q.transpose();
//     }

//     cov /= static_cast<float>(std::max(1, count - 1));

//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
//     if (solver.info() != Eigen::Success)
//         return feature;

//     Eigen::Vector3f normal = solver.eigenvectors().col(0);
//     if (normal.norm() < 1e-6f)
//         return feature;

//     normal.normalize();
//     if (normal.z() < 0.0f)
//         normal *= -1.0f;

//     const float d = -normal.dot(mean);
//     feature.normal = normal;
//     feature.coeff << normal.x(), normal.y(), normal.z(), d;

//     // 第一次 PCA 后做一次内点重拟合，比直接 RANSAC 快很多。
//     std::vector<int> inliers;
//     inliers.reserve(cloud->size());

//     double sq_sum = 0.0;
//     for (size_t i = 0; i < cloud->size(); ++i)
//     {
//         const auto& p = cloud->points[i];
//         if (!finitePoint(p))
//             continue;

//         const float dist =
//             std::abs(normal.x() * p.x +
//                      normal.y() * p.y +
//                      normal.z() * p.z + d);

//         if (dist <= params_.plane_dist_thr)
//             inliers.push_back(static_cast<int>(i));
//     }

//     if (inliers.size() < 10)
//         return feature;

//     feature.ratio =
//         static_cast<float>(inliers.size()) /
//         static_cast<float>(std::max<size_t>(1, cloud->size()));

//     if (feature.ratio < params_.min_plane_ratio)
//         return feature;

//     Eigen::Vector3f mean_in = Eigen::Vector3f::Zero();
//     for (int idx : inliers)
//     {
//         const auto& p = cloud->points[idx];
//         mean_in += Eigen::Vector3f(p.x, p.y, p.z);
//     }
//     mean_in /= static_cast<float>(inliers.size());

//     Eigen::Matrix3f cov_in = Eigen::Matrix3f::Zero();
//     for (int idx : inliers)
//     {
//         const auto& p = cloud->points[idx];
//         const Eigen::Vector3f q(p.x - mean_in.x(),
//                                 p.y - mean_in.y(),
//                                 p.z - mean_in.z());
//         cov_in.noalias() += q * q.transpose();
//     }

//     cov_in /= static_cast<float>(std::max<size_t>(1, inliers.size() - 1));

//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver_in(cov_in);
//     if (solver_in.info() != Eigen::Success)
//         return feature;

//     normal = solver_in.eigenvectors().col(0);
//     if (normal.norm() < 1e-6f)
//         return feature;

//     normal.normalize();
//     if (normal.z() < 0.0f)
//         normal *= -1.0f;

//     const float d2 = -normal.dot(mean_in);

//     double in_sq = 0.0;
//     for (int idx : inliers)
//     {
//         const auto& p = cloud->points[idx];
//         const float dist =
//             normal.x() * p.x +
//             normal.y() * p.y +
//             normal.z() * p.z + d2;
//         in_sq += static_cast<double>(dist) * dist;
//     }

//     feature.normal = normal;
//     feature.coeff << normal.x(), normal.y(), normal.z(), d2;
//     feature.rmse = static_cast<float>(std::sqrt(in_sq / static_cast<double>(inliers.size())));
//     feature.slope_deg =rad2deg(std::acos(clamp01(std::abs(normal.z()))));
//     feature.valid = std::isfinite(feature.rmse);

//     return feature;
// }

RampRecovery::PlaneFeature RampRecovery::fitPlaneRansac(const PointCloudT::Ptr& cloud) const
{
    PlaneFeature feature;

    if (!cloud || cloud->size() < 10)
        return feature;

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(std::max(10, params_.ransac_max_iter));
    seg.setDistanceThreshold(params_.plane_dist_thr);
    seg.setInputCloud(cloud);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeff;

    seg.segment(inliers, coeff);

    if (inliers.indices.size() < 10 || coeff.values.size() < 4)
        return feature;

    // ============================================================
    // 1. 归一化平面参数
    // ============================================================

    Eigen::Vector3f n(coeff.values[0],coeff.values[1],coeff.values[2]);
    const float norm = n.norm();

    if (norm < 1e-6f)return feature;

    n /= norm;

    float d = coeff.values[3] / norm;

    // 保证 normal.z >= 0
    if (n.z() < 0.0f)
    {
        n *= -1.0f;
        d *= -1.0f;
    }

    // ============================================================
    // 2. 计算 RANSAC 内点 RMSE
    // ============================================================

    double sq = 0.0;

    for (int idx : inliers.indices)
    {
        const auto& p = cloud->points[idx];
        const float e =n.x() * p.x +n.y() * p.y +n.z() * p.z +d;
        sq += static_cast<double>(e) * e;
    }

    //plane_ratio:RANSAC内点数量/CLUSTER总点数，越大表示聚类找到的平面能够把整个 cluster 都作为内点
    const float ratio =static_cast<float>(inliers.indices.size()) /static_cast<float>(cloud->size());
    //plane_rmse：RANSAC 内点到拟合平面的均方根距离，即表示平面拟合的程度,越小表示拟合越好
    const float rmse =static_cast<float>(std::sqrt(sq /static_cast<double>(inliers.indices.size())));

    // ============================================================
    // 3. 对 RANSAC 内点做 PCA
    //
    //    用来判断：
    //
    //    λ0 << λ1 ≈ λ2  -> 真正二维平面
    //    λ0 << λ1 << λ2  -> 线状结构
    //    λ0 ≈ λ1 ≈ λ2    -> 三维散乱
    // ============================================================
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    int valid_count = 0;

    for (int idx : inliers.indices)
    {
        const auto& p = cloud->points[idx];

        if (!finitePoint(p))
            continue;

        centroid.x() += p.x;
        centroid.y() += p.y;
        centroid.z() += p.z;

        ++valid_count;
    }

    if (valid_count < 10)
        return feature;

    centroid /= static_cast<float>(valid_count);

    Eigen::Matrix3f covariance =Eigen::Matrix3f::Zero();
    for (int idx : inliers.indices)
    {
        const auto& p = cloud->points[idx];

        if (!finitePoint(p))
            continue;

        Eigen::Vector3f q(p.x - centroid.x(),p.y - centroid.y(),p.z - centroid.z());
        covariance.noalias() += q * q.transpose();
    }

    covariance /= static_cast<float>(valid_count);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success)
        return feature;


    // Eigenvalues 默认从小到大排列
    const Eigen::Vector3f eigenvalues =solver.eigenvalues();
    const float lambda0 = eigenvalues[0];
    const float lambda1 = eigenvalues[1];
    const float lambda2 = eigenvalues[2];

    if (!std::isfinite(lambda0) ||
        !std::isfinite(lambda1) ||
        !std::isfinite(lambda2) ||
        lambda2 < 1e-6f)
    {
        return feature;
    }
    // STREAM_INFO<<"lambda0:"<<lambda0<<" lambda1:"<<lambda1<<" lambda2:"<<lambda2;
    // ============================================================
    // 4. PCA 几何特征
    // ============================================================
    //linearity：线性度，1：非常像一条线，0：不明显是线状
    //planarity：平面度，1：非常像一个平面，0：不明显是平面
    //scattering: 散乱度，1：非常散乱，0：不明显散乱
    const float linearity =(lambda2 - lambda1) /std::max(lambda2, 1e-6f);
    const float planarity =(lambda1 - lambda0) /std::max(lambda2, 1e-6f);
    const float scattering =lambda0 /std::max(lambda2, 1e-6f);
    const float flatness =lambda0 / std::max(lambda1, 1e-6f);
    const float width_ratio =lambda1 / std::max(lambda2, 1e-6f);
    // ============================================================
    // 5. 保存 PlaneFeature
    // ============================================================

    feature.valid = true;
    feature.normal = n;
    feature.coeff <<n.x(),n.y(),n.z(),d;
    feature.ratio = ratio;
    feature.rmse = rmse;

    feature.slope_deg =rad2deg(std::acos(clamp01(std::abs(n.z()))));
    feature.planarity = planarity;
    feature.linearity = linearity;
    feature.scattering = scattering;
    feature.flatness = flatness;
    feature.width_ratio = width_ratio;
    
    return feature;
}
// RampRecovery::HeightProfile RampRecovery::buildHeightProfile(const PointCloudT::Ptr& cloud,const Eigen::Vector2f& direction) const
// {
//     HeightProfile profile;
//     if (!cloud || cloud->empty())
//         return profile;

//     Eigen::Vector2f mean = Eigen::Vector2f::Zero();
//     int valid_count = 0;

//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;
//         mean += Eigen::Vector2f(p.x, p.y);
//         ++valid_count;
//     }

//     if (valid_count < 10)
//         return profile;

//     mean /= static_cast<float>(valid_count);

//     float min_s = std::numeric_limits<float>::max();
//     float max_s = -std::numeric_limits<float>::max();

//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;

//         const float s =
//             (Eigen::Vector2f(p.x, p.y) - mean).dot(direction);
//         min_s = std::min(min_s, s);
//         max_s = std::max(max_s, s);
//     }

//     const float range = max_s - min_s;
//     if (range < 0.25f)
//         return profile;

//     int num_bins =
//         static_cast<int>(std::ceil(range / params_.profile_bin_size));
//     num_bins = std::max(params_.min_profile_bins, num_bins);
//     num_bins = std::min(params_.max_profile_bins, num_bins);

//     if (num_bins < params_.min_profile_bins)
//         return profile;

//     const float bin_size = range / static_cast<float>(num_bins);
//     const Eigen::Vector2f ortho(-direction.y(), direction.x());

//     // 每个 bin 使用 nth_element 求 median：
//     // 比完整 sort 更快，同时比 mean 对障碍物/离群点更鲁棒。
//     std::vector<std::vector<float>> z_bins(num_bins);
//     std::vector<float> min_lat(num_bins, std::numeric_limits<float>::max());
//     std::vector<float> max_lat(num_bins, -std::numeric_limits<float>::max());
//     std::vector<int> count(num_bins, 0);

//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;

//         const Eigen::Vector2f q(p.x - mean.x(), p.y - mean.y());
//         const float s = q.dot(direction);
//         const float t = q.dot(ortho);

//         int idx = static_cast<int>((s - min_s) / bin_size);
//         idx = std::max(0, std::min(num_bins - 1, idx));

//         z_bins[idx].push_back(p.z);
//         min_lat[idx] = std::min(min_lat[idx], t);
//         max_lat[idx] = std::max(max_lat[idx], t);
//         ++count[idx];
//     }

//     profile.position.reserve(num_bins);
//     profile.height.reserve(num_bins);
//     profile.count.reserve(num_bins);
//     profile.width.reserve(num_bins);

//     for (int i = 0; i < num_bins; ++i)
//     {
//         if (count[i] < 2)
//             continue;

//         auto& z = z_bins[i];
//         const size_t mid = z.size() / 2;
//         std::nth_element(z.begin(), z.begin() + mid, z.end());
//         float median = z[mid];

//         if ((z.size() & 1U) == 0U)
//         {
//             auto left_it =std::max_element(z.begin(), z.begin() + mid);
//             median = 0.5f * (median + *left_it);
//         }

//         profile.position.push_back(min_s + (static_cast<float>(i) + 0.5f) * bin_size);
//         profile.height.push_back(median);
//         profile.count.push_back(count[i]);
//         profile.width.push_back(max_lat[i] - min_lat[i]);
//     }

//     return profile;
// }

bool RampRecovery::calcGeometry(const PointCloudT::Ptr& cloud,const Eigen::Vector2f& direction,Classification& result) const
{
    if (!cloud || cloud->size() < 10)
        return false;

    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    int n = 0;
    for (const auto& p : cloud->points)
    {
        if (!finitePoint(p))
            continue;
        mean += Eigen::Vector2f(p.x, p.y);
        ++n;
    }

    if (n < 10) {return false;}

    mean /= static_cast<float>(n);
    const Eigen::Vector2f ortho(-direction.y(), direction.x());

    float min_long = std::numeric_limits<float>::max();
    float max_long = -std::numeric_limits<float>::max();
    float min_lat = std::numeric_limits<float>::max();
    float max_lat = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const auto& p : cloud->points)
    {
        if (!finitePoint(p))
            continue;

        const Eigen::Vector2f q(p.x - mean.x(), p.y - mean.y());
        const float s = q.dot(direction);
        const float t = q.dot(ortho);

        min_long = std::min(min_long, s);
        max_long = std::max(max_long, s);
        min_lat = std::min(min_lat, t);
        max_lat = std::max(max_lat, t);
        min_z = std::min(min_z, p.z);
        max_z = std::max(max_z, p.z);
    }

    result.length = max_long - min_long;
    result.width = max_lat - min_lat;
    // result.height_gain = max_z - min_z;

    if (result.length < params_.ramp_min_length ) {
        STREAM_WARN<<"length:"<<result.length<<" < "<<params_.ramp_min_length;
        return false;}

    if (result.width < params_.ramp_min_width ){
        STREAM_WARN<<"width:"<<result.width<<" < "<<params_.ramp_min_width;
        return false;}

    float aspect = result.length / std::max(result.width, 1e-3f);
    if (aspect < params_.ramp_min_aspect_ratio) {
        STREAM_WARN<<"aspect:"<<aspect<<" < "<<params_.ramp_min_aspect_ratio;
        return false;}

    // if (result.height_gain < params_.ramp_min_height_gain || result.height_gain > params_.ramp_max_height_gain){cout<<"3"<<endl;return false;}

    return true;
}

// bool RampRecovery::calcProfile(const HeightProfile& profile,const PlaneFeature& plane,Classification& result) const
// {
//     const size_t N = profile.height.size();
//     if (N < static_cast<size_t>(params_.min_profile_bins))
//         {return false;}

//     // 过滤空洞过大的 profile。
//     for (size_t i = 0; i < N; ++i)
//     {
//         if (profile.count[i] < 2)
//             {return false;}

//         if (i > 0)
//         {
//             const float dz = profile.height[i] - profile.height[i - 1];
//             // 真坡道应整体向上，不允许出现明显向下跳变。
//             if (dz < -params_.ramp_max_height_jump)
//                 {return false;}
//         }
//     }

//     // 找到末端连续近水平区，作为平台候选。
//     int platform_begin = static_cast<int>(N);
//     int flat_count = 0;

//     for (int i = static_cast<int>(N) - 1; i > 0; --i)
//     {
//         const float ds =
//             std::max(1e-3f,
//                      profile.position[i] - profile.position[i - 1]);
//         const float local_deg =
//             rad2deg(std::atan2(
//                 std::abs(profile.height[i] - profile.height[i - 1]), ds));

//         if (local_deg <= params_.platform_max_slope_deg)
//         {
//             ++flat_count;
//             if (flat_count >= params_.min_platform_bins)
//             {
//                 platform_begin = i - 1;
//             }
//         }
//         else
//         {
//             break;
//         }
//     }

//     const int ramp_end =
//         (platform_begin < static_cast<int>(N))
//             ? platform_begin
//             : static_cast<int>(N) - 1;

//     if (ramp_end < params_.min_profile_bins - 1)
//         {return false;}

//     // 至少留下 3 个点拟合坡道段。
//     const int fit_end = std::max(2, ramp_end);

//     double sx = 0.0;
//     double sz = 0.0;
//     double sxx = 0.0;
//     double sxz = 0.0;

//     const int fit_N = fit_end + 1;
//     for (int i = 0; i <= fit_end; ++i)
//     {
//         const double x = profile.position[i];
//         const double z = profile.height[i];
//         sx += x;
//         sz += z;
//         sxx += x * x;
//         sxz += x * z;
//     }

//     const double den =static_cast<double>(fit_N) * sxx - sx * sx;
//     if (std::abs(den) < 1e-8){return false;}

//     const double slope =(static_cast<double>(fit_N) * sxz - sx * sz) / den;
//     const double intercept =(sz - slope * sx) / static_cast<double>(fit_N);

//     double mse = 0.0;
//     for (int i = 0; i <= fit_end; ++i)
//     {
//         const double pred =
//             slope * profile.position[i] + intercept;
//         const double e = profile.height[i] - pred;
//         mse += e * e;
//     }

//     result.profile_rmse =static_cast<float>(std::sqrt(mse / static_cast<double>(fit_N)));
//     result.profile_slope_deg =rad2deg(std::atan(std::abs(static_cast<float>(slope))));
//     const float plane_slope =std::tan(plane.slope_deg *static_cast<float>(M_PI) / 180.0f);
//     const float profile_slope =std::abs(static_cast<float>(slope));
//     const float slope_error =rad2deg(std::abs(std::atan(profile_slope) -std::atan(plane_slope)));

//     if (result.profile_rmse > params_.ramp_profile_rmse)
//         {cout<<"6666666666666666"<<endl;return false;}

//     if (slope_error > params_.max_profile_slope_error_deg)
//         {cout<<"77777777777777"<<endl;return false;}


//     if (result.height_gain < params_.ramp_min_height_gain)
//         {cout<<"88888888888888"<<endl;return false;}

//     const float expected_gain =std::max(0.01f,result.length * std::tan(plane.slope_deg *static_cast<float>(M_PI) / 180.0f));
//     result.expected_gain_ratio =result.height_gain / expected_gain;
//     if (result.expected_gain_ratio <params_.min_expected_gain_ratio ||result.expected_gain_ratio >params_.max_expected_gain_ratio){cout<<"123123213213"<<endl;return false;}

//     // 平台必须比起点明显更高。
//     if (platform_begin < static_cast<int>(N))
//     {
//         const float platform_gain =profile.height[N - 1] - profile.height[0];
//         result.platform_valid =platform_gain >= params_.platform_min_height_gain;
//     }
//     else
//     {
//         result.platform_valid = false;
//     }

//     // 末端没有平台时，默认拒绝。
//     if (!result.platform_valid)
//         {cout<<"9999999999999"<<endl;return false;}

//     // 宽度稳定性：用每个 longitudinal bin 的 Z spread 作为粗略厚度，
//     // 防止“斜墙/车身”这种局部高度结构通过。
//     if (!profile.width.empty())
//     {
//         double mean_w = 0.0;
//         int wc = 0;
//         for (float w : profile.width)
//         {
//             if (std::isfinite(w) && w > 0.0f)
//             {
//                 mean_w += w;
//                 ++wc;
//             }
//         }

//         if (wc >= 3)
//         {
//             mean_w /= static_cast<double>(wc);
//             double var = 0.0;
//             for (float w : profile.width)
//             {
//                 if (std::isfinite(w) && w > 0.0f)
//                 {
//                     const double e =
//                         static_cast<double>(w) - mean_w;
//                     var += e * e;
//                 }
//             }

//             var /= static_cast<double>(wc);
//             result.width_variation =static_cast<float>(std::sqrt(var) /std::max(0.02, mean_w));
//         }
//     }

//     if (std::isfinite(result.width_variation) &&result.width_variation > params_.max_width_variation_ratio)
//         {cout<<"000000000000000"<<endl;return false;}

//     return true;
// }

// bool RampRecovery::checkGroundConnection(const PointCloudT::Ptr& cloud,const Eigen::Vector2f& direction,const PointCloudT::ConstPtr& ground,const pcl::search::KdTree<PointT>::Ptr& ground_tree) const
// {
//     if (!cloud || cloud->empty() ||
//         !ground || ground->empty() ||
//         !ground_tree)
//         return false;

//     Eigen::Vector2f mean = Eigen::Vector2f::Zero();
//     int n = 0;

//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;
//         mean += Eigen::Vector2f(p.x, p.y);
//         ++n;
//     }

//     if (n < 10)
//         return false;

//     mean /= static_cast<float>(n);

//     float min_s = std::numeric_limits<float>::max();
//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;

//         const float s =
//             (Eigen::Vector2f(p.x, p.y) - mean).dot(direction);
//         min_s = std::min(min_s, s);
//     }

//     const float start_band = std::max(
//         0.16f, params_.profile_bin_size * 2.0f);

//     std::vector<int> nn_idx(1);
//     std::vector<float> nn_dist(1);

//     int connected = 0;

//     for (const auto& p : cloud->points)
//     {
//         if (!finitePoint(p))
//             continue;

//         const float s =
//             (Eigen::Vector2f(p.x, p.y) - mean).dot(direction);

//         if (s > min_s + start_band)
//             continue;

//         if (ground_tree->nearestKSearch(p, 1, nn_idx, nn_dist) <= 0)
//             continue;

//         if (nn_dist[0] <=
//             params_.ground_connect_dist *
//             params_.ground_connect_dist)
//         {
//             ++connected;
//             if (connected >= params_.min_ground_connect_points)
//                 return true;
//         }
//     }

//     return false;
// }

// bool RampRecovery::checkPlatform(const HeightProfile& profile,const PlaneFeature& plane,Classification& result) const
// {
//     (void)plane;
//     const size_t N = profile.height.size();

//     if (N < static_cast<size_t>(params_.min_profile_bins + 1))
//         return false;

//     int flat = 0;

//     for (int i = static_cast<int>(N) - 1; i > 0; --i)
//     {
//         const float ds =
//             std::max(1e-3f,
//                      profile.position[i] - profile.position[i - 1]);

//         const float deg =
//             rad2deg(std::atan2(
//                 std::abs(profile.height[i] -
//                          profile.height[i - 1]), ds));

//         if (deg <= params_.platform_max_slope_deg)
//             ++flat;
//         else
//             break;
//     }

//     if (flat < params_.min_platform_bins)
//         return false;

//     result.platform_valid =
//         (profile.height.back() - profile.height.front()) >=
//         params_.platform_min_height_gain;

//     return result.platform_valid;
// }

bool RampRecovery::checkForwardAlignment(const Eigen::Vector2f& direction) const
{
    if (!params_.use_forward_axis)
        return true;

    Eigen::Vector2f forward(params_.forward_axis_x,params_.forward_axis_y);

    if (forward.norm() < 1e-6f)
        return true;

    forward.normalize();

    const float c = std::abs(direction.dot(forward));
    return c >= params_.min_forward_cos;
}

RampRecovery::Classification RampRecovery::classify(const PointCloudT::Ptr& cluster,const PointCloudT::ConstPtr& ground,const Clock::time_point& start_time) const
{

    Classification result;
    if (!cluster || cluster->size() < 50)return result;
    // 大 cluster 不直接 RANSAC；特征计算使用均匀采样。
    PointCloudT::Ptr feature_cloud = makeFeatureCloud(cluster);
    if (!feature_cloud || feature_cloud->size() < 10){STREAM_WARN<<"feature_cloud too small!";return result;}

    const Eigen::Vector2f pca_direction =getPrincipalDirection(feature_cloud);
    result.forward_aligned =checkForwardAlignment(pca_direction);
    if (!result.forward_aligned){STREAM_WARN<<"forward_aligned false!";return result;}

    // 1) 几何过滤。
    if (!calcGeometry(feature_cloud, pca_direction, result)){STREAM_WARN<<"calcGeometry failed!";return result;}

    if (budgetExceeded(start_time))
    {
        return result;
    }

    // 2) PCA 平面拟合。
    result.plane = fitPlaneRansac(feature_cloud);
    if (result.plane.planarity < params_.min_plane_planarity){STREAM_WARN<<"planarity为"<<result.plane.planarity<<"小于阈值"<<params_.min_plane_planarity;return result;}
    if (!result.plane.valid){STREAM_WARN<<"平面无效！";return result;}
    // slope / plane quality 是第一道硬门。
    if (!std::isfinite(result.plane.slope_deg) || result.plane.slope_deg < params_.ramp_min_slope_deg ||result.plane.slope_deg > params_.ramp_max_slope_deg)
        {STREAM_WARN<<"slope_deg无效或超出阈值: slope_deg:"<<result.plane.slope_deg<<" < "<<params_.ramp_min_slope_deg<<"或 >"<<params_.ramp_max_slope_deg;return result;}
    if (result.plane.ratio < params_.min_plane_ratio ||result.plane.rmse > params_.max_plane_rmse)
        {STREAM_WARN<<"plane_ratio或plane_rmse超出阈值: plane_ratio:"<<result.plane.ratio<<" < "<<params_.min_plane_ratio
            <<"或plane_rmse:"<<result.plane.rmse<<" > "<<params_.max_plane_rmse;
            return result;}
    if (budgetExceeded(start_time))
    {
        return result;
    }

    // 3) 如果是边界候选且开启 RANSAC，再做一次高成本验证。
    // 默认关闭，X5 建议先不开。
    if (params_.enable_ransac)
    {
        const bool borderline =
            result.plane.ratio < params_.min_plane_ratio + 0.08f ||
            result.plane.rmse > params_.max_plane_rmse * 0.70f;

        if (borderline)
        {
            PlaneFeature ransac_plane =fitPlaneRansac(feature_cloud);

            if (!ransac_plane.valid ||
                ransac_plane.ratio < params_.min_plane_ratio ||
                ransac_plane.rmse > params_.max_plane_rmse ||
                ransac_plane.slope_deg < params_.ramp_min_slope_deg ||
                ransac_plane.slope_deg > params_.ramp_max_slope_deg)
            {
                return result;
            }

            result.plane = ransac_plane;
        }
    }
    if (budgetExceeded(start_time))
    {
        return result;
    }


    // 4) profile + 平台拓扑。
    // HeightProfile profile =buildHeightProfile(feature_cloud, pca_direction);
    // if (!calcProfile(profile, result.plane, result)){cout<<"calcProfile failed!"<<endl;return result;}
    // if (!checkPlatform(profile, result.plane, result)){cout<<"checkPlatform failed!"<<endl;return result;}

    if (budgetExceeded(start_time))
    {
        return result;
    }

    // // 5) 起点必须和 PatchWork++ ground 接壤。
    // if (!checkGroundConnection(feature_cloud,ramp_direction,ground,ground_tree)){cout<<"checkGroundConnection failed!"<<endl;return result;}
    // result.ground_connected = true;
    // result.geometry_valid = true;
    // result.profile_valid = true;
    result.type = Type::RAMP;

    return result;
}

void RampRecovery::copyClusterAsObstacle(const PointCloudT::Ptr& cluster,Result& result) const
{
    if (!cluster)
        return;

    *result.obstacle += *cluster;
}

void RampRecovery::splitClusterByRampPlane(const PointCloudT::Ptr& cluster,const PlaneFeature& plane,Result& result) const
{
    if (!cluster || !plane.valid)
    {
        copyClusterAsObstacle(cluster, result);
        return;
    }

    const float a = plane.coeff[0];
    const float b = plane.coeff[1];
    const float c = plane.coeff[2];
    const float d = plane.coeff[3];

    // 平面拟合阈值和 Ramp 恢复阈值分开
    const float thr = params_.plane_dist_thr;
    const std::size_t ramp_begin = result.ramp->points.size();

    for (const auto& p : cluster->points)
    {
        if (!finitePoint(p))
        {
            result.obstacle->push_back(p);
            continue;
        }

        const float dist =std::abs(a * p.x +b * p.y +c * p.z +d);
        if (dist <= thr)
        {
            result.ramp->push_back(p);
        }
        else
        {
            // Cluster 内偏离 Ramp 平面的点保留为 obstacle
            result.obstacle->push_back(p);
        }
    }
    const std::size_t ramp_count =result.ramp->points.size() - ramp_begin;

    // Ramp 恢复点太少，则本次恢复全部撤销
    if (ramp_count < static_cast<std::size_t>(params_.min_cluster_size))
    {
        result.obstacle->insert(
            result.obstacle->end(),
            result.ramp->points.begin() + ramp_begin,
            result.ramp->points.end());

        result.ramp->points.erase(
            result.ramp->points.begin() + ramp_begin,
            result.ramp->points.end());
    }

}

void RampRecovery::processCluster(const PointCloudT::Ptr& cluster,const PointCloudT::ConstPtr& ground,Result& result,const Clock::time_point& start_time)
{
    Classification cls =classify(cluster, ground, start_time);

    if (params_.verbose)
    {
        if (cls.plane.valid)
        {
            STREAM_INFO<<"[RampRecovery] before split points="<<cluster->size()
                    << " slope=" << cls.plane.slope_deg
                    << " plane_ratio=" << cls.plane.ratio
                    << " plane_rmse=" << cls.plane.rmse
                    << " plane_linearity="<<cls.plane.linearity
                    << " plane_planarity="<<cls.plane.planarity
                    << " plane_scattering="<<cls.plane.scattering
                    << " plane_flatness="<<cls.plane.flatness
                    << " plane_width_ratio="<<cls.plane.width_ratio
                    << " length=" << cls.length
                    << " width=" << cls.width
                    << " type="<< (cls.type == Type::RAMP ? "RAMP" : "OBSTACLE");
                //   << " height_gain=" << cls.height_gain
                //   << " profile_rmse=" << cls.profile_rmse
                //   << " profile_slope=" << cls.profile_slope_deg
                //   << " platform=" << cls.platform_valid
                //   << " ground=" << cls.ground_connected
                //   << " width_var=" << cls.width_variation;                    
        }
    }

    if (cls.type == Type::RAMP)
        {splitClusterByRampPlane(cluster, cls.plane, result);
        result.slope_angle = cls.plane.slope_deg;}
    else
        copyClusterAsObstacle(cluster, result);
}

void RampRecovery::process(const PointCloudT::ConstPtr& nonground,const PointCloudT::ConstPtr& ground,Result& result)
{
    const Clock::time_point start_time = Clock::now();

    result.clear();
    visualization_msgs::msg::MarkerArray delete_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = parent_frame;
    delete_marker.ns =  "bbox";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_array.markers.push_back(delete_marker);
    box_pub_.publish(delete_array);    

    if (!nonground || nonground->empty())
        return;

    // 防止极端帧直接把 CPU 时间拖爆。
    // 正常情况下 PatchWork++ 的 nonground 应明显低于此值。
    if (nonground->size()>params_.max_input_points)
    {
        if (params_.verbose)
        {
            STREAM_INFO<< "[RampRecovery] input="
                        << nonground->size()
                        << " > max_input_points="
                        << params_.max_input_points
                        << ", skip ramp recovery." ;
        }
        *result.obstacle = *nonground;
        return;
    }
    
    auto start_1 = std::chrono::high_resolution_clock::now();
    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(nonground);
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(params_.cluster_tolerance);
    // ec.setMinClusterSize(params_.min_cluster_size);
    // ec.setMaxClusterSize(params_.max_cluster_size);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(nonground);
    // std::vector<pcl::PointIndices> cluster_indices;
    // ec.extract(cluster_indices);

    std::vector<pcl::PointIndices> cluster_indices;

    hashGridCluster<PointT>(
        nonground,
        params_.cluster_tolerance,
        params_.min_cluster_size,
        params_.max_cluster_size,
        cluster_indices
    );
    auto end_1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_1-start_1);
    STREAM_INFO<<"[RampRecovery]EuclideanClusterExtraction Time taken: "<<duration1.count()<<"ms";

    visualization_msgs::msg::MarkerArray bbox_marker_array;
    int cluster_id = 0;
    STREAM_INFO<<"-------------坡道检测开始---------------";
    for (const auto& indices : cluster_indices)
    {
        STREAM_INFO
        << "Cluster " << cluster_id <<" 开始计算坡道特征";
        if (budgetExceeded(start_time))
        {

            // watchdog 触发：剩余未处理 cluster 保守当 obstacle。
            if (params_.verbose)
                STREAM_INFO << "[RampRecovery] watchdog reached, "
                            << "remaining clusters -> obstacle.";

            // 当前 cluster 以及后面的所有 cluster
            // 都不再做坡道检测，直接作为 obstacle。
            for (size_t remain_id = cluster_id;
                remain_id < cluster_indices.size();
                ++remain_id)
            {
                const auto& remain = cluster_indices[remain_id];

                for (int idx : remain.indices)
                {
                    if (idx >= 0 &&
                        idx < static_cast<int>(nonground->size()))
                    {
                        result.obstacle->push_back(
                            nonground->points[idx]);
                    }
                }
            }
            break;
        }

        PointCloudT::Ptr cluster(new PointCloudT);
        cluster->reserve(indices.indices.size());

        for (int idx : indices.indices)
        {
            if (idx >= 0 &&
                idx < static_cast<int>(nonground->size()))
            {
                cluster->push_back(nonground->points[idx]);
            }
        }

        cluster->width = static_cast<uint32_t>(cluster->size());
        cluster->height = 1;
        cluster->is_dense = true;

        if (params_.enable_visualization)
        {

            uint8_t r, g, b;
            this->generate_rgb(cluster_id, r, g, b);

            visualization_msgs::msg::Marker bbox_marker;
            this->createBboxMarker(
                bbox_marker, cluster, cluster_id, r, g, b);
            bbox_marker_array.markers.push_back(bbox_marker);
        }

        processCluster(cluster,ground,result,start_time);
        STREAM_INFO<< "点个数：input=" << cluster->size()
        << " ramp=" << result.ramp->size()
        << " obstacle=" << result.obstacle->size();
        ++cluster_id;
    }

    // PCL EuclideanClusterExtraction 可能产生少量未进入任何 cluster 的点。
    // 为了保证输出不丢点，这里把未被 cluster 覆盖的点作为 obstacle。
    // 对通常已经经过 min_cluster_size 过滤的输入，数量很小。
    std::vector<uint8_t> assigned(nonground->size(), 0);
    for (const auto& indices : cluster_indices)
    {
        for (int idx : indices.indices)
        {
            if (idx >= 0 &&
                idx < static_cast<int>(assigned.size()))
                assigned[idx] = 1;
        }
    }

    for (size_t i = 0; i < nonground->size(); ++i)
    {
        if (!assigned[i])
            result.obstacle->push_back(nonground->points[i]);
    }

    if (params_.enable_visualization)
        box_pub_.publish(bbox_marker_array);


    STREAM_INFO<<"-------------坡道检测结束---------------";
    if (params_.verbose)
    {
        const double elapsed =std::chrono::duration<double, std::milli>(Clock::now() - start_time).count();
        STREAM_INFO<< "[RampRecovery] 当前帧 total time elapse=" << elapsed<< " ms"
                << " input = "<<nonground->size()
                << " ramp=" << result.ramp->size()
                << " obstacle=" << result.obstacle->size();
    }

}
