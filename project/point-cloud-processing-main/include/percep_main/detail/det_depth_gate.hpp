#ifndef PERCEP_MAIN_DETAIL_DET_DEPTH_GATE_HPP
#define PERCEP_MAIN_DETAIL_DET_DEPTH_GATE_HPP

#include <algorithm>
#include <cstddef>
#include <vector>

namespace perception {
namespace detail {

struct DepthGate {
    float min_mm = -1e30f;
    float max_mm = 1e30f;
};

// zs contains valid positive camera depths in millimetres and is sorted in place.
// Weighting by z^2 compensates for the lower point density of distant objects.
inline DepthGate SelectWeightedDepthGate(std::vector<float>& zs,
                                        float span_mm, float gap_mm,
                                        int min_depth_pts,
                                        float margin_mm = 30.f) {
    if (zs.empty()) return {};
    std::sort(zs.begin(), zs.end());
    min_depth_pts = std::max(1, min_depth_pts);

    // Preserve production's sparse-box protection: never make the entire box
    // valid just because too few depth samples are available.
    if (zs.size() < static_cast<std::size_t>(min_depth_pts)) {
        return {std::max(0.f, zs.front() - margin_mm),
                zs.front() + (span_mm > 1.f ? span_mm : 600.f) + margin_mm};
    }

    const int need = std::max(
        3, std::min(min_depth_pts, static_cast<int>(zs.size() / 10)));
    std::size_t best_s = 0, best_e = 0;
    bool have_best = false;
    double best_weight = -1.0;
    for (std::size_t i = 0; i < zs.size();) {
        std::size_t j = i;
        while (j + 1 < zs.size() && zs[j + 1] - zs[j] <= gap_mm) ++j;
        if (j - i + 1 >= static_cast<std::size_t>(need)) {
            double weight = 0.0;
            for (std::size_t k = i; k <= j; ++k) {
                const double z_m = static_cast<double>(zs[k]) * 1e-3;
                weight += z_m * z_m;
            }
            const bool better = !have_best || weight > best_weight * 1.05 ||
                (weight >= best_weight * 0.95 && zs[i] < zs[best_s]);
            if (better) {
                best_s = i;
                best_e = j;
                best_weight = weight;
                have_best = true;
            }
        }
        i = j + 1;
    }

    if (!have_best) {
        const float center = zs[zs.size() / 2];
        const float half = (span_mm > 1.f ? span_mm : 800.f) * 0.5f;
        return {std::max(0.f, center - half - margin_mm),
                center + half + margin_mm};
    }
    const float z0 = zs[best_s];
    const float z1 = span_mm > 1.f ? std::min(zs[best_e], z0 + span_mm)
                                  : zs[best_e];
    return {std::max(0.f, z0 - margin_mm), z1 + margin_mm};
}

}  // namespace detail
}  // namespace perception

#endif  // PERCEP_MAIN_DETAIL_DET_DEPTH_GATE_HPP
