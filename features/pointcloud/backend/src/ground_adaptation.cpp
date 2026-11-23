#include "mapping_algorithms.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ai_node_percep_demo {
namespace {
// Keep V0 diagnostics without a ROS/daye dependency. Numerical statements below
// are copied from the user-confirmed 1104 source; no fit/guard/EMA simplification.
template<class... Args>
void adaptLog(const char* format, Args... args) {
  std::printf(format, args...);
  std::putchar('\n');
}
}  // namespace

Trans MappingAlgorithms::adaptGroundTrans(const PointCloudT& cam_cloud)
{
    if (!gadapt_enable) return cali_default;

    Trans  est;
    double h = 0.0, tilt = 0.0, resid = 0.0;
    size_t inl = 0;
    const bool ok = solveExtrinsicsFromGround(cam_cloud, cali_max_range, est, h, tilt, resid, inl);

    const char* why = nullptr;
    if (!ok)                                                    why = "拟合失败";
    else if (static_cast<int>(inl) < gadapt_min_inliers)        why = "内点不足";
    else if (resid > gadapt_max_resid_cm)                       why = "残差过大";
    // 门限配置是度，Trans 是弧度，比较前统一到弧度
    else if (std::fabs(rad_to_angle(est.roll  - cali.roll))  > gadapt_max_roll_dev)  why = "roll 偏离过大";
    else if (std::fabs(rad_to_angle(est.pitch - cali.pitch)) > gadapt_max_pitch_dev) why = "pitch 偏离过大";
    // else if (std::fabs(rad_to_angle(est.yaw   - cali.yaw))   > gadapt_max_yaw_dev)   why = "yaw 偏离过大";
    else if (std::fabs(h - cali.z) > gadapt_max_h_dev)                               why = "高度偏离过大";

    if (why)
    {
        ++gadapt_rej_;
        if (gadapt_verbose)
            adaptLog("[PercepCompute] 拒绝(%s): roll=%.3f pitch=%.3f yaw=%.3f h=%.4f 内点=%zu 残差=%.2fcm"
                   " -> 回退", why, rad_to_angle(est.roll), rad_to_angle(est.pitch),
                   rad_to_angle(est.yaw), h, inl, resid);
        return cali_default;
    }

    ++gadapt_ok_;
    if (!gadapt_inited_)
    {
        gadapt_cur_ = cali;          // 从静态值起步，避免第一帧突跳
        gadapt_inited_ = true;
    }

    const double a = gadapt_ema_alpha;
    gadapt_cur_.roll  = (1.0 - a) * gadapt_cur_.roll  + a * est.roll;
    gadapt_cur_.pitch = (1.0 - a) * gadapt_cur_.pitch + a * est.pitch;
    // gadapt_cur_.yaw   = (1.0 - a) * gadapt_cur_.yaw   + a * est.yaw;
    gadapt_cur_.z     = (1.0 - a) * gadapt_cur_.z     + a * h;
    gadapt_cur_.x = cali.x;          // 前向/横向安装偏移观测不到，保持静态值
    gadapt_cur_.y = cali.y;

    if (gadapt_verbose)
        adaptLog("[PercepCompute] 本帧 roll=%.3f pitch=%.3f yaw=%.3f h=%.4f (内点=%zu 残差=%.2fcm)"
               " -> 平滑后 roll=%.3f pitch=%.3f yaw=%.3f z=%.4f  [采纳%zu/拒绝%zu]",
               rad_to_angle(est.roll), rad_to_angle(est.pitch), rad_to_angle(est.yaw),
               h, inl, resid,
               rad_to_angle(gadapt_cur_.roll), rad_to_angle(gadapt_cur_.pitch),
               rad_to_angle(gadapt_cur_.yaw), gadapt_cur_.z,
               gadapt_ok_, gadapt_rej_);
    return gadapt_cur_;
}
// 从地面反解相机安装外参。
//
// 做法：在相机系(X右 Y下 Z前)稳健拟合地面平面，得到向上的法向 up，
// 再用 (前, 左, 上) 三轴直接构造旋转矩阵 R —— base_link 的 X=前 Y=左 Z=上，
// 所以 R 的三个行向量就是这三个轴在相机系下的表示。最后把 R 分解成工程里
// EulerToMatrix 用的 Rz(yaw)·Ry(pitch)·Rx(roll)。
// 这样是精确构造，不做小角度近似，也不需要初值。
bool MappingAlgorithms::solveExtrinsicsFromGround(const PointCloudT& cam_cloud,double max_range,
                                               Trans& out,double& out_height,
                                               double& out_tilt_deg,double& out_resid_cm,
                                               size_t& out_inliers)
{
    out_inliers = 0;
    // 1) 近距离点。远处双目噪声大，且容易把墙面/天花板拉进来
    std::vector<Eigen::Vector3d> P;
    P.reserve(cam_cloud.size());
    for (const PointT& p : cam_cloud.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z < 0.3f || p.z > static_cast<float>(max_range)) continue;
        P.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y),
                       static_cast<double>(p.z));
    }
    if (P.size() < 500) { adaptLog("[PercepCompute] 近距离点太少(%zu)", P.size()); return false; }

    // 2) 种子：相机系 Y 向下，地面点 Y 偏大，取下半部
    std::vector<double> ys;
    ys.reserve(P.size());
    for (const auto& p : P) ys.push_back(p.y());
    std::nth_element(ys.begin(), ys.begin() + static_cast<long>(ys.size()/2), ys.end());
    const double y_mid = ys[ys.size()/2];
    std::vector<Eigen::Vector3d> sel;
    for (const auto& p : P) if (p.y() > y_mid) sel.push_back(p);
    if (sel.size() < 200) sel = P;

    // 3) 迭代重加权：PCA 拟平面 -> 收内点 -> 再拟
    Eigen::Vector3d nrm(0,-1,0);
    double d = 0.0;
    for (int it = 0; it < 10 && sel.size() >= 100; ++it)
    {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto& p : sel) c += p;
        c /= static_cast<double>(sel.size());
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        for (const auto& p : sel) { const Eigen::Vector3d q = p - c; C += q * q.transpose(); }
        C /= static_cast<double>(sel.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
        nrm = es.eigenvectors().col(0).normalized();   // 最小特征值 -> 平面法向
        d   = -nrm.dot(c);

        std::vector<double> dist;
        dist.reserve(P.size());
        for (const auto& p : P) dist.push_back(std::fabs(nrm.dot(p) + d));
        std::vector<double> sorted = dist;
        std::nth_element(sorted.begin(), sorted.begin() + static_cast<long>(sorted.size()*4/10),
                         sorted.end());
        const double thr = std::max(0.03, sorted[sorted.size()*4/10]);
        std::vector<Eigen::Vector3d> next;
        for (size_t i = 0; i < P.size(); ++i) if (dist[i] < thr) next.push_back(P[i]);
        if (next.size() < 100) break;
        sel.swap(next);
    }
    // 4) 强制法向朝上：相机系 Y 向下 => "上" 的 Y 分量必须为负
    if (nrm.y() > 0.0) { nrm = -nrm; d = -d; }

    out_height = std::fabs(d);
    double s = 0.0; size_t cnt = 0;
    for (const auto& p : P)
    {
        const double e = nrm.dot(p) + d;
        if (std::fabs(e) < 0.05) { s += e * e; ++cnt; }
    }
    out_inliers  = cnt;
    out_resid_cm = (cnt > 1) ? std::sqrt(s / static_cast<double>(cnt)) * 100.0 : 999.0;

    // 5) 构造 base_link 三轴并分解欧拉角
    const Eigen::Vector3d up = nrm;
    Eigen::Vector3d fwd(0,0,1);
    fwd = fwd - fwd.dot(up) * up;
    if (fwd.norm() < 1e-6) return false;
    fwd.normalize();
    const Eigen::Vector3d left = up.cross(fwd).normalized();
    Eigen::Matrix3d R;
    R.row(0) = fwd; R.row(1) = left; R.row(2) = up;

    // Trans 全项目约定用弧度(cali 也是 angle_to_rad 存的)，这里必须跟着用弧度，
    // 否则直接喂给 EulerToMatrix/publish3dTF 会得到完全错误的变换。
    out.pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2,0))));
    out.roll  = std::atan2(R(2,1), R(2,2));
    out.yaw   = std::atan2(R(1,0), R(0,0));
    out.x = cali.x; out.y = cali.y; out.z = out_height;

    // 构造法保证地面在 base_link 下严格水平，残余倾斜的真实校验由 autocali 回代完成
    out_tilt_deg = 0.0;
    return cnt > 200;
}

}  // namespace ai_node_percep_demo
