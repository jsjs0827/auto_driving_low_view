#pragma once

#include "perception/semantic_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace perception::integrations {

// Parameters consumed by the product AiFusion constructor. They deliberately
// live outside the portable GridConfig: changing one of these must never alter
// the public map geometry by accident.
struct ProductAiFusionOptions {
    double obstacle_height_threshold_m = 0.05;
    int fill_grass_neighbor_threshold = 5;
    int remove_noise_neighbor_threshold = 6;
    double camera_x_calibration_m = 0.0;
    double camera_y_calibration_m = 0.0;
    float minimum_forward_x_m = 0.0f;

    fusion::CoordinateFrame required_frame = fusion::CoordinateFrame::BaseLink;
    bool enforce_required_frame = true;

    void validate() const {
        const auto finite = [](double value) { return std::isfinite(value); };
        if (!finite(obstacle_height_threshold_m) || obstacle_height_threshold_m < 0.0)
            throw std::invalid_argument(
                "product AiFusion obstacle-height threshold must be finite and non-negative");
        if (fill_grass_neighbor_threshold < 0 || fill_grass_neighbor_threshold > 9)
            throw std::invalid_argument(
                "product AiFusion fill-grass threshold must be in [0,9]");
        if (remove_noise_neighbor_threshold < 0 || remove_noise_neighbor_threshold > 9)
            throw std::invalid_argument(
                "product AiFusion remove-noise threshold must be in [0,9]");
        if (!finite(camera_x_calibration_m) || !finite(camera_y_calibration_m) ||
            !finite(minimum_forward_x_m))
            throw std::invalid_argument("product AiFusion calibration values must be finite");
    }
};

// The upstream AiFusion map has a fixed coordinate convention: x starts at
// zero in the forward direction and y is centred around zero. Validate it
// before converting points so a custom portable grid cannot be silently
// interpreted with a different origin.
inline void validateProductAiFusionContract(
    const fusion::GridConfig &grid, const ProductAiFusionOptions &options,
    fusion::CoordinateFrame ground_frame, fusion::CoordinateFrame obstacle_frame,
    fusion::CoordinateFrame semantic_frame) {
    options.validate();
    grid.validate();

    const float width_m = static_cast<float>(grid.cols) * grid.resolution_m;
    const float expected_origin_y = -0.5f * width_m;
    const float tolerance = std::max(1.0e-5f, grid.resolution_m * 1.0e-4f);
    if (std::fabs(grid.origin_x_m) > tolerance ||
        std::fabs(grid.origin_y_m - expected_origin_y) > tolerance)
        throw std::invalid_argument(
            "product AiFusion requires grid origin (0,-width/2)");

    if (ground_frame != obstacle_frame || ground_frame != semantic_frame)
        throw std::invalid_argument(
            "product AiFusion input clouds must use the same coordinate frame");
    if (options.enforce_required_frame && ground_frame != options.required_frame)
        throw std::invalid_argument(
            "product AiFusion input cloud does not use the configured required frame");
}

// Translate the portable semantic contract to the product's stable 0..8 map
// types. nullopt means that the point must not be sent to product AiFusion.
inline std::optional<int> productAiFusionType(
    const fusion::SemanticPoint &point, const fusion::AiGridPolicy &policy) {
    switch (point.semantic) {
    case fusion::SemanticKind::Unknown:
        return std::nullopt;
    case fusion::SemanticKind::Passable:
        return 3;
    case fusion::SemanticKind::SegmentationObstacle:
        return policy.nongrass_seg_as_obstacle ? std::optional<int>{1} : std::nullopt;
    case fusion::SemanticKind::DetectionObstacle:
        if (point.class_id == policy.person_detection_id)
            return 4;
        if (point.class_id == policy.cat_detection_id)
            return 5;
        if (point.class_id == policy.dog_detection_id)
            return 6;
        if (point.class_id == policy.charger_detection_id)
            return 7;
        if (point.class_id == policy.toy_detection_id)
            return 8;
        return 2;
    case fusion::SemanticKind::Ground:
        return std::nullopt;
    }
    return std::nullopt;
}

// Concrete bridge to the product class declared by percep_demo/ai_fusion.hpp.
// Its header remains product-free; only product_ai_fusion_adapter.cpp includes
// PCL/ROS/product headers. A fresh product AiFusion instance is created per
// call, so separate PerceptionEngine branches do not share its mutable maps.
class ProductAiFusionAdapter final : public fusion::AiFusionAdapter {
  public:
    explicit ProductAiFusionAdapter(ProductAiFusionOptions options = {})
        : options_(options) {
        options_.validate();
    }

    fusion::AiFusionMaps integrate(
        const fusion::FramedPointCloud &ground_cloud,
        const fusion::FramedPointCloud &obstacle_cloud,
        const fusion::FramedSemanticCloud &semantic_cloud,
        const fusion::GridConfig &grid,
        const fusion::AiGridPolicy &policy) const override;

    const ProductAiFusionOptions &options() const noexcept { return options_; }

  private:
    ProductAiFusionOptions options_;
};

} // namespace perception::integrations
