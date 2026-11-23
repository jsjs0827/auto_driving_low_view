#include "perception/semantic_fusion.hpp"
#include "perception/error.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace perception::fusion {
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::invalid_argument(message);
}
void require(bool condition, const std::string &message) {
    if (!condition)
        throw std::invalid_argument(message);
}
bool finite(float value) { return std::isfinite(value); }

bool contains(const std::vector<int> &values, int value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

float scaled(float value, int from, int to) {
    return from > 0 && to > 0 ? value * static_cast<float>(to) / static_cast<float>(from)
                              : value;
}

std::array<std::uint8_t, 3> sampleColor(const ImageView &image, int u, int v,
                                        int reference_width, int reference_height) {
    if (!image.data || image.width <= 0 || image.height <= 0)
        return {128, 128, 128};
    const int x = std::clamp(static_cast<int>((static_cast<double>(u) + 0.5) * image.width /
                                               std::max(1, reference_width)),
                             0, image.width - 1);
    const int y = std::clamp(static_cast<int>((static_cast<double>(v) + 0.5) * image.height /
                                               std::max(1, reference_height)),
                             0, image.height - 1);
    if (image.format == PixelFormat::RGB || image.format == PixelFormat::BGR) {
        const auto *pixel = image.data + static_cast<std::size_t>(y) * image.stride +
                            static_cast<std::size_t>(x) * 3;
        return image.format == PixelFormat::RGB
                   ? std::array<std::uint8_t, 3>{pixel[0], pixel[1], pixel[2]}
                   : std::array<std::uint8_t, 3>{pixel[2], pixel[1], pixel[0]};
    }
    const auto luma = image.data[static_cast<std::size_t>(y) * image.stride + x];
    return {luma, luma, luma};
}

using VoxelKey = VoxelCoordinate;
struct VoxelHash {
    std::size_t operator()(const VoxelKey &key) const noexcept {
        std::size_t value = std::hash<std::int64_t>{}(key.x);
        value ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (value << 6) +
                 (value >> 2);
        value ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (value << 6) +
                 (value >> 2);
        return value;
    }
};
template <class Integer> bool checkedFloor(double value, Integer &output) {
    static_assert(std::numeric_limits<Integer>::is_integer &&
                      std::numeric_limits<Integer>::is_signed,
                  "checkedFloor requires a signed integer");
    if (!std::isfinite(value))
        return false;
    const double floored = std::floor(value);
    // Signed two's-complement minima are exact powers of two in double. Using
    // -lowest as an exclusive upper bound also avoids double(max) rounding up
    // to 2^63 before an int64_t conversion.
    const double lower = static_cast<double>(std::numeric_limits<Integer>::lowest());
    const double upper_exclusive = -lower;
    if (floored < lower || floored >= upper_exclusive)
        return false;
    output = static_cast<Integer>(floored);
    return true;
}

bool offsetCoordinate(std::int64_t value, int offset, std::int64_t &output) {
    if ((offset < 0 && value == std::numeric_limits<std::int64_t>::lowest()) ||
        (offset > 0 && value == std::numeric_limits<std::int64_t>::max()))
        return false;
    output = value + offset;
    return true;
}

bool voxelOf(const PointXYZRGB &point, float size, VoxelKey &output) {
    output.valid = checkedFloor(static_cast<double>(point.x) / size, output.x) &&
                   checkedFloor(static_cast<double>(point.y) / size, output.y) &&
                   checkedFloor(static_cast<double>(point.z) / size, output.z);
    return output.valid;
}

int gridTypePriority(int type) noexcept {
    if (type >= 4)
        return 4;
    if (type == 2)
        return 3;
    if (type == 3)
        return 2;
    if (type == 1)
        return 1;
    return 0;
}

int mergeGridType(int current, int candidate) noexcept {
    const int current_priority = gridTypePriority(current);
    const int candidate_priority = gridTypePriority(candidate);
    if (candidate_priority == 0)
        return current_priority == 0 ? 0 : current;
    if (current_priority == 0 || candidate_priority > current_priority)
        return candidate;
    if (candidate_priority < current_priority)
        return current;
    return std::min(current, candidate);
}

void validateGridCells(const GridMap &map, int maximum_type,
                       const char *description) {
    const auto expected = static_cast<std::size_t>(map.config.rows) *
                          static_cast<std::size_t>(map.config.cols);
    require(map.cells.size() == expected,
            std::string(description) + " grid cell count does not match its geometry");
    for (const auto &cell : map.cells) {
        require(cell.type >= 0 && cell.type <= maximum_type,
                std::string(description) + " grid contains an out-of-contract type");
        require(finite(cell.min_height) && finite(cell.max_height) &&
                    finite(cell.average_height),
                std::string(description) + " grid contains a non-finite height");
        if (cell.point_count) {
            constexpr float tolerance = 1e-5f;
            require(cell.min_height <= cell.max_height &&
                        cell.average_height + tolerance >= cell.min_height &&
                        cell.average_height <= cell.max_height + tolerance,
                    std::string(description) + " grid contains invalid height statistics");
        }
    }
}

SemanticKind semanticForGridType(int type) noexcept {
    if (type == 3)
        return SemanticKind::Passable;
    if (type == 1)
        return SemanticKind::SegmentationObstacle;
    if (type == 2 || type >= 4)
        return SemanticKind::DetectionObstacle;
    return SemanticKind::Unknown;
}

int aiGridType(const SemanticPoint &point, const AiGridPolicy &policy) noexcept {
    if (point.semantic == SemanticKind::Passable)
        return 3;
    if (point.semantic == SemanticKind::SegmentationObstacle)
        return policy.nongrass_seg_as_obstacle ? 1 : 0;
    if (point.semantic != SemanticKind::DetectionObstacle)
        return 0;
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
}

int geometricGridType(const GridCell &cell) noexcept {
    if (gridTypePriority(cell.type) != 0)
        return cell.type;
    if (cell.semantic == SemanticKind::Passable)
        return 1;
    if (cell.semantic == SemanticKind::DetectionObstacle ||
        cell.semantic == SemanticKind::SegmentationObstacle)
        return 2;
    return 0;
}
} // namespace

void StereoCalibration::validate() const {
    require(finite(fx) && finite(fy) && fx > 0 && fy > 0, "invalid camera focal length");
    require(finite(cx) && finite(cy), "invalid camera principal point");
    require(finite(baseline_m) && baseline_m > 0, "invalid stereo baseline");
    require(calibration_width >= 0 && calibration_height >= 0,
            "invalid calibration resolution");
}

PointCloud disparityToPointCloud(const DisparityView &disparity, const ImageView &image,
                                 const StereoCalibration &calibration,
                                 const CloudGenerationConfig &config) {
    calibration.validate();
    require(disparity.data && disparity.width > 0 && disparity.height > 0,
            "invalid disparity image");
    require(finite(disparity.scale) && disparity.scale > 0, "invalid disparity scale");
    require(config.sample_step > 0, "sample_step must be positive");
    require(finite(config.min_disparity) && config.min_disparity > 0,
            "invalid minimum disparity");
    require(finite(config.min_depth_m) && finite(config.max_depth_m) &&
                config.min_depth_m >= 0 && config.max_depth_m > config.min_depth_m,
            "invalid depth range");
    const std::size_t element = disparity.type == DisparityType::UInt16 ? sizeof(std::uint16_t)
                                                                        : sizeof(float);
    const std::size_t tight = static_cast<std::size_t>(disparity.width) * element;
    const std::size_t stride = disparity.stride_bytes ? disparity.stride_bytes : tight;
    require(stride >= tight, "disparity stride is smaller than a row");
    const auto height = static_cast<std::size_t>(disparity.height);
    require(height - 1 <= (std::numeric_limits<std::size_t>::max() - tight) / stride,
            "disparity buffer size overflows");
    const std::size_t minimum_bytes = (height - 1) * stride + tight;
    require(disparity.bytes >= minimum_bytes, "disparity buffer is truncated");
    if (image.data)
        validateCameraImage(image);

    const float fx = scaled(calibration.fx, calibration.calibration_width, disparity.width);
    const float fy = scaled(calibration.fy, calibration.calibration_height, disparity.height);
    const float cx = scaled(calibration.cx, calibration.calibration_width, disparity.width);
    const float cy = scaled(calibration.cy, calibration.calibration_height, disparity.height);
    const std::size_t sample_step = static_cast<std::size_t>(config.sample_step);
    const auto width = static_cast<std::size_t>(disparity.width);
    const auto *base = static_cast<const std::uint8_t *>(disparity.data);
    const auto validDepth = [&](std::size_t u, std::size_t v, float &z) {
        const auto *row = base + v * stride;
        float raw = 0;
        if (disparity.type == DisparityType::UInt16) {
            std::uint16_t value = 0;
            std::memcpy(&value, row + u * element, element);
            raw = static_cast<float>(value);
        } else {
            std::memcpy(&raw, row + u * element, element);
        }
        const float d = raw / disparity.scale;
        if (!finite(d) || d < config.min_disparity)
            return false;
        z = calibration.baseline_m * fx / d;
        return finite(z) && z >= config.min_depth_m && z <= config.max_depth_m;
    };

    // Count valid samples before applying max_points. Basing a cap on the dense
    // image dimensions drops most samples from sparse saveDisp data after it is
    // expanded with zeroes. The second pass selects valid points uniformly in
    // source order while keeping memory bounded by the requested cap.
    std::size_t valid_count = 0;
    for (std::size_t v = 0; v < height; v += sample_step) {
        for (std::size_t u = 0; u < width; u += sample_step) {
            float z = 0;
            if (validDepth(u, v, z))
                ++valid_count;
        }
    }
    const std::size_t limit = config.max_points ? std::min(config.max_points, valid_count)
                                                 : valid_count;
    PointCloud output;
    output.reserve(limit);
    if (limit == 0)
        return output;
    const std::size_t selection_threshold = valid_count - limit;
    std::size_t selection_accumulator = selection_threshold; // Select the first valid sample.
    for (std::size_t v = 0; v < height; v += sample_step) {
        for (std::size_t u = 0; u < width; u += sample_step) {
            float z = 0;
            if (!validDepth(u, v, z))
                continue;
            if (selection_accumulator >= selection_threshold) {
                selection_accumulator -= selection_threshold;
            } else {
                selection_accumulator += limit;
                continue;
            }
            PointXYZRGB point;
            point.x = (static_cast<float>(u) - cx) * z / fx;
            point.y = (static_cast<float>(v) - cy) * z / fy;
            point.z = z;
            point.image_u = static_cast<int>(u);
            point.image_v = static_cast<int>(v);
            const auto color = sampleColor(image, point.image_u, point.image_v,
                                           disparity.width, disparity.height);
            point.r = color[0];
            point.g = color[1];
            point.b = color[2];
            output.push_back(point);
        }
    }
    require(output.size() == limit, "internal point cap selection failed");
    return output;
}

PointCloud projectPointCloudToImage(const PointCloud &cloud,
                                    const StereoCalibration &calibration, int image_width,
                                    int image_height) {
    calibration.validate();
    require(image_width > 0 && image_height > 0, "invalid projection image size");
    const float fx = scaled(calibration.fx, calibration.calibration_width, image_width);
    const float fy = scaled(calibration.fy, calibration.calibration_height, image_height);
    const float cx = scaled(calibration.cx, calibration.calibration_width, image_width);
    const float cy = scaled(calibration.cy, calibration.calibration_height, image_height);
    PointCloud result = cloud;
    for (auto &point : result) {
        point.image_u = -1;
        point.image_v = -1;
        if (!finite(point.x) || !finite(point.y) || !finite(point.z) || point.z <= 0)
            continue;
        int u = 0, v = 0;
        const double projected_u = static_cast<double>(fx) * point.x / point.z + cx;
        const double projected_v = static_cast<double>(fy) * point.y / point.z + cy;
        if (!checkedFloor(projected_u + 0.5, u) ||
            !checkedFloor(projected_v + 0.5, v))
            continue;
        if (u >= 0 && u < image_width && v >= 0 && v < image_height) {
            point.image_u = u;
            point.image_v = v;
        }
    }
    return result;
}

std::vector<MatchRecord> matchDetectionsAndMask(const PointCloud &cloud,
                                                const std::vector<Detection> &detections,
                                                const SegmentationResult &segmentation,
                                                int image_width, int image_height,
                                                const MatchConfig &config) {
    require(image_width > 0 && image_height > 0, "invalid matching image size");
    require(finite(config.detection_score_min) && config.detection_score_min >= 0 &&
                config.detection_score_min <= 1,
            "invalid detection score minimum");
    require(config.segmentation_boundary_radius >= 0 &&
                config.segmentation_boundary_radius <= 64,
            "invalid segmentation boundary radius");
    require(segmentation.width > 0 && segmentation.height > 0,
            "invalid segmentation dimensions");
    const auto mask_size = static_cast<std::size_t>(segmentation.width) *
                           static_cast<std::size_t>(segmentation.height);
    require(segmentation.mask.size() == mask_size,
            "segmentation mask size does not match its dimensions");
    std::vector<MatchRecord> result(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto &point = cloud[i];
        auto &match = result[i];
        const int u = point.image_u;
        const int v = point.image_v;
        if (u < 0 || u >= image_width || v < 0 || v >= image_height)
            continue;
        match.projection_valid = true;
        const int su = std::clamp(static_cast<int>((static_cast<double>(u) + 0.5) *
                                                    segmentation.width / image_width),
                                  0, segmentation.width - 1);
        const int sv = std::clamp(static_cast<int>((static_cast<double>(v) + 0.5) *
                                                    segmentation.height / image_height),
                                  0, segmentation.height - 1);
        match.segmentation_class =
            segmentation.mask[static_cast<std::size_t>(sv) * segmentation.width + su];
        match.segmentation_stable = true;
        if (!contains(config.passable_segmentation_ids, match.segmentation_class)) {
            const int radius = config.segmentation_boundary_radius;
            for (int dy = -radius; match.segmentation_stable && dy <= radius; ++dy) {
                const int yy = sv + dy;
                if (yy < 0 || yy >= segmentation.height) {
                    match.segmentation_stable = false;
                    break;
                }
                for (int dx = -radius; dx <= radius; ++dx) {
                    const int xx = su + dx;
                    if (xx < 0 || xx >= segmentation.width ||
                        contains(config.passable_segmentation_ids,
                                 segmentation.mask[static_cast<std::size_t>(yy) *
                                                       segmentation.width + xx])) {
                        match.segmentation_stable = false;
                        break;
                    }
                }
            }
        }
        // Detection matching is deliberately independent from the segmentation class.
        // A grass pixel must not suppress a high-confidence thin obstacle detection.
        float best = config.detection_score_min;
        for (std::size_t detection = 0; detection < detections.size(); ++detection) {
            const auto &box = detections[detection];
            if (!config.detection_include_ids.empty() &&
                !contains(config.detection_include_ids, box.class_id))
                continue;
            if (!finite(box.score) || box.score < best)
                continue;
            if (u >= box.x1 && u <= box.x2 && v >= box.y1 && v <= box.y2) {
                best = box.score;
                match.detection_index = static_cast<int>(detection);
                match.detection_score = box.score;
            }
        }
    }
    return result;
}

void filterDetectionDepth(const PointCloud &cloud, const std::vector<Detection> &detections,
                          const MatchConfig &matching, const DepthFilterConfig &config,
                          std::vector<MatchRecord> &matches) {
    require(matches.size() == cloud.size(), "cloud/match size mismatch");
    require(config.minimum_points > 0, "depth minimum_points must be positive");
    require(finite(config.cluster_gap_m) && config.cluster_gap_m > 0 &&
                finite(config.cluster_span_m) && config.cluster_span_m > 0 &&
                finite(config.margin_m) && config.margin_m >= 0,
            "invalid depth filter configuration");
    std::vector<std::vector<float>> depths(detections.size());
    for (std::size_t i = 0; i < matches.size(); ++i) {
        auto &match = matches[i];
        match.detection_depth_valid = false;
        if (match.detection_index < 0 ||
            static_cast<std::size_t>(match.detection_index) >= detections.size())
            continue;
        const auto &detection = detections[match.detection_index];
        if (contains(matching.passable_detection_ids, detection.class_id)) {
            match.detection_depth_valid = true;
            continue;
        }
        if (finite(cloud[i].z) && cloud[i].z > 0)
            depths[match.detection_index].push_back(cloud[i].z);
    }
    std::vector<std::pair<float, float>> windows(detections.size(),
                                                 {1.0f, -1.0f});
    for (std::size_t index = 0; index < depths.size(); ++index) {
        auto &values = depths[index];
        if (static_cast<int>(values.size()) < config.minimum_points)
            continue; // Insufficient support is rejected instead of accepting all background.
        std::sort(values.begin(), values.end());
        std::size_t best_begin = 0, best_end = 0;
        std::size_t begin = 0;
        while (begin < values.size()) {
            std::size_t end = begin;
            while (end + 1 < values.size() &&
                   values[end + 1] - values[end] <= config.cluster_gap_m)
                ++end;
            const auto best_count = best_end - best_begin + 1;
            const auto count = end - begin + 1;
            if (count > best_count ||
                (count == best_count && values[begin] < values[best_begin])) {
                best_begin = begin;
                best_end = end;
            }
            begin = end + 1;
        }
        if (static_cast<int>(best_end - best_begin + 1) < config.minimum_points)
            continue;
        const float low = values[best_begin] - config.margin_m;
        const float high = std::min(values[best_end], values[best_begin] + config.cluster_span_m) +
                           config.margin_m;
        windows[index] = {low, high};
    }
    for (std::size_t i = 0; i < matches.size(); ++i) {
        auto &match = matches[i];
        if (match.detection_index < 0 ||
            static_cast<std::size_t>(match.detection_index) >= detections.size())
            continue;
        const auto &detection = detections[match.detection_index];
        if (contains(matching.passable_detection_ids, detection.class_id))
            continue;
        const auto window = windows[match.detection_index];
        match.detection_depth_valid = window.first <= window.second &&
                                      cloud[i].z >= window.first && cloud[i].z <= window.second;
    }
}

VoxelFilterResult filterVoxelMinimumPoints(const PointCloud &cloud,
                                           const std::vector<Detection> &detections,
                                           const VoxelMinimumFilterConfig &config,
                                           std::vector<MatchRecord> matches) {
    require(matches.size() == cloud.size(), "cloud/match size mismatch");
    require(finite(config.voxel_size_m) && config.voxel_size_m > 0,
            "invalid voxel size");
    require(config.minimum_points_per_voxel > 0,
            "invalid minimum points per voxel");

    VoxelFilterResult result;
    result.matches = std::move(matches);
    result.coordinates.resize(cloud.size());
    result.filter_segmentation = config.filter_segmentation;
    for (std::size_t i = 0; i < cloud.size(); ++i)
        voxelOf(cloud[i], config.voxel_size_m, result.coordinates[i]);

    for (auto &match : result.matches) {
        match.detection_voxel_valid = false;
        match.segmentation_voxel_valid = !config.filter_segmentation;
    }

    using VoxelPoints = std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelHash>;
    const auto markPopulated = [&](const VoxelPoints &voxels,
                                   const std::function<void(std::size_t)> &mark) {
        for (const auto &entry : voxels)
            if (static_cast<int>(entry.second.size()) >= config.minimum_points_per_voxel)
                for (const auto point : entry.second)
                    mark(point);
    };

    for (std::size_t detection = 0; detection < detections.size(); ++detection) {
        VoxelPoints voxels;
        for (std::size_t i = 0; i < result.matches.size(); ++i)
            if (result.matches[i].detection_index == static_cast<int>(detection) &&
                result.matches[i].detection_depth_valid && result.coordinates[i].valid)
                voxels[result.coordinates[i]].push_back(i);
        markPopulated(voxels, [&](std::size_t point) {
            result.matches[point].detection_voxel_valid = true;
        });
    }

    if (config.filter_segmentation) {
        std::unordered_map<int, VoxelPoints> class_voxels;
        for (std::size_t i = 0; i < result.matches.size(); ++i)
            if (result.matches[i].segmentation_class >= 0 &&
                result.matches[i].segmentation_stable && result.coordinates[i].valid)
                class_voxels[result.matches[i].segmentation_class][result.coordinates[i]]
                    .push_back(i);
        for (const auto &entry : class_voxels)
            markPopulated(entry.second, [&](std::size_t point) {
                result.matches[point].segmentation_voxel_valid = true;
            });
    }
    return result;
}

std::vector<MatchRecord> filterConnectedComponents(
    const PointCloud &cloud, const std::vector<Detection> &detections,
    const ConnectedComponentFilterConfig &config, const VoxelFilterResult &input) {
    require(input.matches.size() == cloud.size(), "cloud/match size mismatch");
    require(input.coordinates.size() == cloud.size(), "cloud/voxel size mismatch");
    require(config.minimum_component_points > 0,
            "invalid minimum component points");
    require(config.filter_segmentation == input.filter_segmentation,
            "voxel/component segmentation policy mismatch");

    auto matches = input.matches;
    for (auto &match : matches) {
        match.detection_component_valid = false;
        match.segmentation_component_valid = !config.filter_segmentation;
    }

    using VoxelPoints = std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelHash>;
    const auto components = [&](const VoxelPoints &voxels, bool keep_only_largest,
                                const std::function<void(std::size_t, bool)> &mark) {
        std::unordered_set<VoxelKey, VoxelHash> visited;
        struct Component {
            std::vector<std::size_t> points;
            double mean_depth = std::numeric_limits<double>::infinity();
            std::size_t minimum_index = std::numeric_limits<std::size_t>::max();
        };
        Component largest;
        bool have_largest = false;
        std::vector<std::vector<std::size_t>> valid_components;
        for (const auto &seed_entry : voxels) {
            const auto &seed = seed_entry.first;
            if (!visited.insert(seed).second)
                continue;
            std::queue<VoxelKey> queue;
            queue.push(seed);
            std::vector<std::size_t> points;
            while (!queue.empty()) {
                const auto current = queue.front();
                queue.pop();
                const auto found = voxels.find(current);
                if (found != voxels.end())
                    points.insert(points.end(), found->second.begin(), found->second.end());
                for (int dx = -1; dx <= 1; ++dx)
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dz = -1; dz <= 1; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0)
                                continue;
                            VoxelKey next;
                            if (!offsetCoordinate(current.x, dx, next.x) ||
                                !offsetCoordinate(current.y, dy, next.y) ||
                                !offsetCoordinate(current.z, dz, next.z))
                                continue;
                            next.valid = true;
                            if (voxels.count(next) && visited.insert(next).second)
                                queue.push(next);
                        }
            }
            if (static_cast<int>(points.size()) < config.minimum_component_points)
                continue;
            std::sort(points.begin(), points.end());
            if (keep_only_largest) {
                double depth_sum = 0.0;
                for (const auto point : points)
                    depth_sum += cloud[point].z;
                Component candidate{std::move(points), 0.0, 0};
                candidate.mean_depth = depth_sum / candidate.points.size();
                candidate.minimum_index = candidate.points.front();
                // Hash traversal is intentionally not part of the result
                // contract. Ties select nearer depth, then source index.
                const bool better =
                    !have_largest || candidate.points.size() > largest.points.size() ||
                    (candidate.points.size() == largest.points.size() &&
                     (candidate.mean_depth < largest.mean_depth ||
                      (candidate.mean_depth == largest.mean_depth &&
                       candidate.minimum_index < largest.minimum_index)));
                if (better) {
                    largest = std::move(candidate);
                    have_largest = true;
                }
            } else {
                valid_components.push_back(std::move(points));
            }
        }
        std::unordered_set<std::size_t> keep;
        if (keep_only_largest && have_largest)
            keep.insert(largest.points.begin(), largest.points.end());
        else
            for (const auto &component : valid_components)
                keep.insert(component.begin(), component.end());
        for (const auto &voxel : voxels)
            for (const auto point : voxel.second)
                mark(point, keep.count(point) != 0);
    };

    for (std::size_t detection = 0; detection < detections.size(); ++detection) {
        VoxelPoints voxels;
        for (std::size_t i = 0; i < matches.size(); ++i)
            if (matches[i].detection_index == static_cast<int>(detection) &&
                matches[i].detection_depth_valid && matches[i].detection_voxel_valid &&
                input.coordinates[i].valid)
                voxels[input.coordinates[i]].push_back(i);
        components(voxels, true, [&](std::size_t point, bool keep) {
            matches[point].detection_component_valid = keep;
        });
    }

    if (config.filter_segmentation) {
        std::unordered_map<int, VoxelPoints> class_voxels;
        for (std::size_t i = 0; i < matches.size(); ++i)
            if (matches[i].segmentation_class >= 0 && matches[i].segmentation_stable &&
                matches[i].segmentation_voxel_valid && input.coordinates[i].valid)
                class_voxels[matches[i].segmentation_class][input.coordinates[i]].push_back(i);
        // Different semantic classes never bridge merely because their voxels
        // touch in 3D.
        for (const auto &entry : class_voxels)
            components(entry.second, false, [&](std::size_t point, bool keep) {
                matches[point].segmentation_component_valid = keep;
            });
    }
    return matches;
}

void filterVoxelComponents(const PointCloud &cloud, const std::vector<Detection> &detections,
                           const VoxelFilterConfig &config,
                           std::vector<MatchRecord> &matches) {
    VoxelMinimumFilterConfig voxel;
    voxel.voxel_size_m = config.voxel_size_m;
    voxel.minimum_points_per_voxel = config.minimum_points_per_voxel;
    voxel.filter_segmentation = config.filter_segmentation;
    auto intermediate = filterVoxelMinimumPoints(cloud, detections, voxel, matches);
    ConnectedComponentFilterConfig component;
    component.minimum_component_points = config.minimum_component_points;
    component.filter_segmentation = config.filter_segmentation;
    matches = filterConnectedComponents(cloud, detections, component, intermediate);
}

SemanticCloud generateSemanticCloud(const PointCloud &cloud,
                                    const std::vector<Detection> &detections,
                                    const std::vector<MatchRecord> &matches,
                                    const SemanticConfig &config) {
    require(matches.size() == cloud.size(), "cloud/match size mismatch");
    require(config.min_depth_m >= 0 && config.max_depth_m > config.min_depth_m &&
                config.max_y_m > config.min_y_m,
            "invalid semantic range");
    SemanticCloud result;
    result.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        SemanticPoint output;
        bool raw_segmentation_class_present = false;
        static_cast<PointXYZRGB &>(output) = cloud[i];
        const auto &match = matches[i];
        const bool geometry_valid = finite(output.x) && finite(output.y) && finite(output.z) &&
                                    output.z >= config.min_depth_m &&
                                    output.z <= config.max_depth_m &&
                                    output.y >= config.min_y_m && output.y <= config.max_y_m;
        if (geometry_valid && match.detection_index >= 0 &&
            static_cast<std::size_t>(match.detection_index) < detections.size()) {
            const auto &detection = detections[match.detection_index];
            output.class_id = detection.class_id;
            output.confidence = detection.score;
            if (contains(config.matching.passable_detection_ids, detection.class_id)) {
                output.semantic = SemanticKind::Passable;
            } else if (match.detection_depth_valid && match.detection_component_valid) {
                // Reliable detection obstacles override passable segmentation at thin edges.
                output.semantic = SemanticKind::DetectionObstacle;
            }
        }
        if (geometry_valid && output.semantic == SemanticKind::Unknown &&
            match.segmentation_class >= 0) {
            output.class_id = kSegmentationClassBase + match.segmentation_class;
            output.confidence = 1.0f;
            raw_segmentation_class_present = true;
            if (contains(config.matching.passable_segmentation_ids,
                         match.segmentation_class)) {
                output.semantic = SemanticKind::Passable;
            } else if (config.nongrass_seg_as_obstacle && match.segmentation_stable &&
                       match.segmentation_component_valid) {
                output.semantic = SemanticKind::SegmentationObstacle;
            }
        }
        std::array<std::uint8_t, 3> color = config.unknown_color;
        if (output.semantic == SemanticKind::Passable) {
            color = config.passable_color;
        } else if (output.semantic == SemanticKind::SegmentationObstacle) {
            color = config.segmentation_obstacle_color;
        } else if (output.semantic == SemanticKind::DetectionObstacle) {
            if (output.class_id >= 0 &&
                static_cast<std::size_t>(output.class_id) < config.detection_colors.size())
                color = config.detection_colors[output.class_id];
            else
                color = {230, 30, 30};
        } else {
            // Unknown is the product semantic policy, not loss of the raw
            // network label. Keeping 1024+seg_id supports logging/debugging
            // without publishing an obstacle Marker.
            if (!raw_segmentation_class_present) {
                output.class_id = -1;
                output.confidence = 0;
            }
        }
        output.r = color[0];
        output.g = color[1];
        output.b = color[2];
        result.push_back(output); // Unknown points are intentionally retained.
    }
    return result;
}

void RigidTransform::validate() const {
    for (float value : matrix)
        require(finite(value), "transform contains nonfinite value");
    require(std::fabs(matrix[12]) < 1e-5f && std::fabs(matrix[13]) < 1e-5f &&
                std::fabs(matrix[14]) < 1e-5f && std::fabs(matrix[15] - 1.0f) < 1e-5f,
            "transform must be affine row-major");
    const auto dot = [&](int a, int b) {
        return matrix[a] * matrix[b] + matrix[a + 1] * matrix[b + 1] +
               matrix[a + 2] * matrix[b + 2];
    };
    require(std::fabs(dot(0, 0) - 1.0f) < 1e-2f &&
                std::fabs(dot(4, 4) - 1.0f) < 1e-2f &&
                std::fabs(dot(8, 8) - 1.0f) < 1e-2f &&
                std::fabs(dot(0, 4)) < 1e-2f && std::fabs(dot(0, 8)) < 1e-2f &&
                std::fabs(dot(4, 8)) < 1e-2f,
            "transform rotation is not orthonormal");
    const float determinant =
        matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
        matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
        matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
    require(std::fabs(determinant - 1.0f) < 2e-2f,
            "transform rotation determinant is not +1");
    require(source != target, "transform source and target are identical");
}

FramedSemanticCloud transformSemanticCloud(const FramedSemanticCloud &input,
                                           const RigidTransform &transform) {
    transform.validate();
    require(input.frame == transform.source, "point cloud transform source frame mismatch");
    FramedSemanticCloud output;
    output.frame = transform.target;
    output.points = input.points;
    for (auto &point : output.points) {
        const float x = point.x, y = point.y, z = point.z;
        point.x = transform.matrix[0] * x + transform.matrix[1] * y +
                  transform.matrix[2] * z + transform.matrix[3];
        point.y = transform.matrix[4] * x + transform.matrix[5] * y +
                  transform.matrix[6] * z + transform.matrix[7];
        point.z = transform.matrix[8] * x + transform.matrix[9] * y +
                  transform.matrix[10] * z + transform.matrix[11];
    }
    return output;
}

namespace {
Publication makePublicationImpl(
    const FramedSemanticCloud &input,
    const std::shared_ptr<const FramedSemanticCloud> &owner, FrameInfo frame,
    bool include_unknown, bool make_markers) {
    Publication output;
    output.frame = frame;
    output.coordinate_frame = input.frame;
    std::shared_ptr<SemanticCloud> filtered;
    if (include_unknown && owner) {
        output.cloud = std::shared_ptr<const SemanticCloud>(owner, &owner->points);
    } else {
        filtered = std::make_shared<SemanticCloud>();
        filtered->reserve(input.points.size());
    }
    if (make_markers)
        output.markers.reserve(input.points.size());
    for (const auto &point : input.points) {
        if (!include_unknown && point.semantic == SemanticKind::Unknown)
            continue;
        if (filtered)
            filtered->push_back(point);
        if (make_markers && point.semantic != SemanticKind::Unknown) {
            Marker marker;
            marker.id = output.markers.size();
            marker.x = point.x;
            marker.y = point.y;
            marker.z = point.z;
            marker.r = point.r;
            marker.g = point.g;
            marker.b = point.b;
            marker.semantic = point.semantic;
            marker.class_id = point.class_id;
            marker.text = std::to_string(point.class_id);
            output.markers.push_back(std::move(marker));
        }
    }
    if (filtered)
        output.cloud = std::move(filtered);
    return output;
}
} // namespace

Publication makePublication(const FramedSemanticCloud &input, FrameInfo frame,
                            bool include_unknown, bool make_markers) {
    return makePublicationImpl(input, {}, frame, include_unknown, make_markers);
}

Publication makePublication(std::shared_ptr<const FramedSemanticCloud> input,
                            FrameInfo frame, bool include_unknown,
                            bool make_markers) {
    if (!input)
        throw std::invalid_argument("publication cloud is null");
    return makePublicationImpl(*input, input, frame, include_unknown, make_markers);
}

void GridConfig::validate() const {
    require(rows > 0 && cols > 0 && rows <= 10000 && cols <= 10000,
            "invalid grid dimensions");
    require(static_cast<std::uint64_t>(rows) * static_cast<std::uint64_t>(cols) <=
                4000000ULL,
            "grid contains too many cells");
    require(finite(resolution_m) && resolution_m > 0 && finite(origin_x_m) &&
                finite(origin_y_m),
            "invalid grid geometry");
}

GridMap projectToAiMap(const FramedSemanticCloud &cloud, const GridConfig &config,
                       const AiGridPolicy &policy) {
    config.validate();
    require(cloud.frame != CoordinateFrame::CameraOptical,
            "AI map requires base_link or percep_main coordinates");
    GridMap result;
    result.frame = cloud.frame;
    result.config = config;
    result.cells.resize(static_cast<std::size_t>(config.rows) * config.cols);
    std::vector<double> sums(result.cells.size(), 0.0);
    for (const auto &point : cloud.points) {
        if (!finite(point.x) || !finite(point.y) || !finite(point.z))
            continue;
        const int candidate_type = aiGridType(point, policy);
        if (candidate_type == 0)
            continue;
        std::int64_t column = 0, row = 0;
        const double column_value =
            (-static_cast<double>(point.y) - config.origin_y_m) / config.resolution_m;
        const double row_value =
            (static_cast<double>(point.x) - config.origin_x_m) / config.resolution_m;
        if (!checkedFloor(column_value, column) || !checkedFloor(row_value, row) ||
            row < 0 || row >= config.rows || column < 0 || column >= config.cols)
            continue;
        const auto cell_index = static_cast<std::size_t>(row) *
                                    static_cast<std::size_t>(config.cols) +
                                static_cast<std::size_t>(column);
        auto &cell = result.cells[cell_index];
        if (cell.point_count == 0) {
            cell.min_height = cell.max_height = point.z;
        } else {
            cell.min_height = std::min(cell.min_height, point.z);
            cell.max_height = std::max(cell.max_height, point.z);
        }
        ++cell.point_count;
        sums[cell_index] += point.z;
        cell.type = mergeGridType(cell.type, candidate_type);
        cell.semantic = semanticForGridType(cell.type);
    }
    for (std::size_t i = 0; i < result.cells.size(); ++i)
        if (result.cells[i].point_count)
            result.cells[i].average_height =
                static_cast<float>(sums[i] / result.cells[i].point_count);
    return result;
}

GridMap fuseMaps(const GridMap &ai_map, const GridMap &geometric_map) {
    ai_map.config.validate();
    geometric_map.config.validate();
    require(ai_map.frame != CoordinateFrame::CameraOptical &&
                ai_map.frame == geometric_map.frame,
            "grid maps must use the same explicit coordinate frame");
    validateGridCells(ai_map, 8, "AI");
    validateGridCells(geometric_map, 2, "geometric");
    const auto expected_cells = static_cast<std::size_t>(ai_map.config.rows) *
                                static_cast<std::size_t>(ai_map.config.cols);
    require(ai_map.config.rows == geometric_map.config.rows &&
                ai_map.config.cols == geometric_map.config.cols &&
                std::fabs(ai_map.config.resolution_m - geometric_map.config.resolution_m) < 1e-6f &&
                std::fabs(ai_map.config.origin_x_m - geometric_map.config.origin_x_m) < 1e-6f &&
                std::fabs(ai_map.config.origin_y_m - geometric_map.config.origin_y_m) < 1e-6f &&
                ai_map.cells.size() == expected_cells &&
                geometric_map.cells.size() == expected_cells,
            "grid maps are not geometrically compatible");
    GridMap result = geometric_map;
    for (std::size_t i = 0; i < result.cells.size(); ++i) {
        const auto &ai = ai_map.cells[i];
        const auto &geometry = geometric_map.cells[i];
        auto &out = result.cells[i];
        const int ai_type = gridTypePriority(ai.type) ? ai.type :
                            (ai.semantic == SemanticKind::Passable ? 3 :
                             ai.semantic == SemanticKind::SegmentationObstacle ? 1 :
                             ai.semantic == SemanticKind::DetectionObstacle ? 2 : 0);
        const int pcl_type = geometricGridType(geometry);
        int fused_type = 0;
        if (ai_type == 0)
            fused_type = pcl_type;
        else if (pcl_type == 0)
            fused_type = ai_type;
        if (ai_type == 3)
            fused_type = 3;
        else if (ai_type == 1)
            fused_type = pcl_type == 0 ? 0 : pcl_type;
        else if (ai_type == 2)
            fused_type = pcl_type == 1 ? 1 : 2;
        else if (ai_type >= 4)
            fused_type = pcl_type == 1 ? 1 : ai_type;
        out.type = fused_type;
        out.semantic = fused_type == 1 ? SemanticKind::Ground
                                       : semanticForGridType(fused_type);
        const auto total = static_cast<std::uint64_t>(ai.point_count) + geometry.point_count;
        if (total) {
            if (!geometry.point_count) {
                out.min_height = ai.min_height;
                out.max_height = ai.max_height;
            } else if (ai.point_count) {
                out.min_height = std::min(ai.min_height, geometry.min_height);
                out.max_height = std::max(ai.max_height, geometry.max_height);
            }
            out.average_height = static_cast<float>(
                (static_cast<double>(ai.average_height) * ai.point_count +
                 static_cast<double>(geometry.average_height) * geometry.point_count) /
                total);
            out.point_count = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                total, std::numeric_limits<std::uint32_t>::max()));
        }
    }
    return result;
}

} // namespace perception::fusion
