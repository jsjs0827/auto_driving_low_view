#include "perception/decoders.hpp"
#include "detail/decoder_trace.hpp"
#include "detail/decoder_workspace.hpp"
#include "detail/math.hpp"
#include "detail/native_detection.hpp"
#include <cfenv>
#include <limits>
#include <set>
namespace perception {
namespace {
thread_local detail::SegmentationDecodeTrace last_segmentation_decode_trace;

void stableScoreSort(std::vector<Detection> &boxes, std::vector<Detection> &scratch) {
    if (boxes.size() < 2)
        return;
    scratch.resize(boxes.size());
    bool data_in_boxes = true;
    for (std::size_t width = 1; width < boxes.size();) {
        const auto &source = data_in_boxes ? boxes : scratch;
        auto &target = data_in_boxes ? scratch : boxes;
        for (std::size_t first = 0; first < boxes.size(); first += width * 2) {
            const auto middle = std::min(first + width, boxes.size());
            const auto last = std::min(first + width * 2, boxes.size());
            std::size_t left = first, right = middle, output = first;
            while (left < middle || right < last) {
                // Take the left element on score ties, matching std::stable_sort
                // with the original strict-greater comparator.
                if (right < last &&
                    (left == middle || source[right].score > source[left].score))
                    target[output++] = source[right++];
                else
                    target[output++] = source[left++];
            }
        }
        data_in_boxes = !data_in_boxes;
        if (width > boxes.size() / 2)
            break;
        width *= 2;
    }
    if (!data_in_boxes)
        std::copy(scratch.begin(), scratch.end(), boxes.begin());
}
void finishDetections(const DetectionConfig &cfg, detail::DetectionDecoderWorkspace &workspace,
                      std::vector<Detection> &kept) {
    auto &boxes = workspace.boxes;
    stableScoreSort(boxes, workspace.sort_buffer);
    if (boxes.size() > std::size_t(cfg.candidate_budget)) {
        workspace.groups.resize(cfg.classes);
        for (auto &group : workspace.groups)
            group.clear();
        for (std::size_t i = 0; i < boxes.size(); ++i)
            workspace.groups[boxes[i].class_id].push_back(i);
        workspace.chosen.resize(boxes.size());
        std::fill(workspace.chosen.begin(), workspace.chosen.end(), std::uint8_t{0});
        int used = 0;
        for (std::size_t rank = 0; used < cfg.candidate_budget; ++rank)
            for (const auto &group : workspace.groups)
                if (rank < group.size() && used < cfg.candidate_budget) {
                    workspace.chosen[group[rank]] = 1;
                    ++used;
                }
        std::size_t write = 0;
        for (std::size_t i = 0; i < boxes.size(); ++i)
            if (workspace.chosen[i])
                boxes[write++] = boxes[i];
        boxes.resize(write);
    }
    kept.clear();
    for (const auto &b : boxes) {
        bool suppressed = false;
        for (const auto &k : kept)
            if (k.class_id == b.class_id && iou(k, b) > cfg.iou_threshold) {
                suppressed = true;
                break;
            }
        if (!suppressed)
            kept.push_back(b);
        if (kept.size() == std::size_t(cfg.max_detections))
            break;
    }
}
} // namespace

void detail::decodeDetectionValidated(const std::vector<Tensor> &tensors,
                                      const ModelPlan &plan, const Geometry &g,
                                      DetectionDecoderWorkspace &workspace,
                                      std::vector<Detection> &kept,
                                      std::size_t &rejected) {
    const auto &cfg = plan.detection();
    rejected = 0;
    const auto &order = plan.heads();
    auto &boxes = workspace.boxes;
    boxes.clear();
    const double raw_threshold = std::log(double(cfg.score_threshold) / (1 - cfg.score_threshold));
    for (int level = 0; level < 3; ++level) {
        const auto &cls = tensors[order[level * 2]];
        const auto &reg = tensors[order[level * 2 + 1]];
        const int stride = 8 << level;
        const std::size_t plane = std::size_t(cls.width) * cls.height;
        // ModelPlan validated owned, contiguous CHW tensors above. Scan adjacent
        // anchors within each class plane instead of making a strided, out-of-line
        // Tensor::at call for every class. Fixed tiles keep scratch below 3 KiB,
        // independent of image size, and retain the original anchor order.
        constexpr std::size_t tile_size = 256;
        std::array<float, tile_size> scores;
        std::array<int, tile_size> classes;
        std::array<unsigned char, tile_size> finite;
        for (std::size_t first = 0; first < plane; first += tile_size) {
            const auto count = std::min(tile_size, plane - first);
            const float *first_class = cls.data.data() + first;
            for (std::size_t i = 0; i < count; ++i) {
                scores[i] = first_class[i];
                classes[i] = 0;
                finite[i] = std::isfinite(first_class[i]);
            }
            for (int c = 1; c < cfg.classes; ++c) {
                const float *values = cls.data.data() + std::size_t(c) * plane + first;
                for (std::size_t i = 0; i < count; ++i) {
                    const float v = values[i];
                    finite[i] = static_cast<unsigned char>(finite[i] && std::isfinite(v));
                    // Strict > preserves the first class on ties, including +/-0.
                    if (v > scores[i]) {
                        scores[i] = v;
                        classes[i] = c;
                    }
                }
            }
            for (std::size_t i = 0; i < count; ++i) {
                const float best = scores[i];
                const int id = classes[i];
                bool valid = finite[i];
                if (!valid) {
                    ++rejected;
                    continue;
                }
                if (best < raw_threshold)
                    continue;
                const auto position = first + i;
                const int y = int(position / cls.width), x = int(position % cls.width);
                double d[4]{};
                for (int side = 0; side < 4 && valid; ++side) {
                    double maximum = -std::numeric_limits<double>::infinity();
                    for (int bin = 0; bin < cfg.reg_bins; ++bin) {
                        const float v = reg.data[std::size_t(side * cfg.reg_bins + bin) * plane + position];
                        if (!std::isfinite(v)) {
                            valid = false;
                            break;
                        }
                        maximum = std::max(maximum, double(v));
                    }
                    if (!valid)
                        break;
                    double sum = 0, weighted = 0;
                    for (int bin = 0; bin < cfg.reg_bins; ++bin) {
                        const double p =
                            std::exp(double(reg.data[std::size_t(side * cfg.reg_bins + bin) * plane + position]) - maximum);
                        sum += p;
                        weighted += p * bin;
                    }
                    d[side] = weighted / sum;
                }
                if (!valid) {
                    ++rejected;
                    continue;
                }
                Detection b{id,
                            float(1 / (1 + std::exp(-double(best)))),
                            float((x + .5 - d[0]) * stride),
                            float((y + .5 - d[1]) * stride),
                            float((x + .5 + d[2]) * stride),
                            float((y + .5 + d[3]) * stride)};
                // Map using the actual rounded resize dimensions; clip before NMS.
                b.x1 =
                    std::clamp((b.x1 - g.left) * g.source_w / g.resized_w, 0.f, float(g.source_w));
                b.x2 =
                    std::clamp((b.x2 - g.left) * g.source_w / g.resized_w, 0.f, float(g.source_w));
                b.y1 =
                    std::clamp((b.y1 - g.top) * g.source_h / g.resized_h, 0.f, float(g.source_h));
                b.y2 =
                    std::clamp((b.y2 - g.top) * g.source_h / g.resized_h, 0.f, float(g.source_h));
                if (b.x2 > b.x1 && b.y2 > b.y1)
                    boxes.push_back(b);
            }
        }
    }
    finishDetections(cfg, workspace, kept);
}

void detail::decodeDetectionNativeValidated(const std::vector<NativeTensorView> &views,
                                             const ModelPlan &plan, const Geometry &g,
                                             DetectionDecoderWorkspace &workspace,
                                             std::vector<Detection> &kept,
                                             std::size_t &rejected) {
    const auto &cfg = plan.detection();
    const auto &order = plan.heads();
    if (views.size() != plan.info().outputs.size())
        throw std::invalid_argument("native detection output count differs from model");
    for (const int index : order)
        if (index < 0 || std::size_t(index) >= views.size() ||
            !(views[std::size_t(index)].shape == plan.info().outputs[std::size_t(index)]) ||
            !views[std::size_t(index)].data)
            throw std::invalid_argument("native detection output shape differs from model");
    rejected = 0;
    auto &boxes = workspace.boxes;
    boxes.clear();
    const double raw_threshold = std::log(double(cfg.score_threshold) / (1 - cfg.score_threshold));
    for (int level = 0; level < 3; ++level) {
        const auto &cls = views[order[level * 2]];
        const auto &reg = views[order[level * 2 + 1]];
        const int stride = 8 << level;
        for (int y = 0; y < cls.shape.height; ++y) {
            for (int x = 0; x < cls.shape.width; ++x) {
                int id = 0;
                float best = cls.at(0, y, x);
                bool valid = std::isfinite(best);
                for (int c = 1; c < cfg.classes; ++c) {
                    const float value = cls.at(c, y, x);
                    valid = valid && std::isfinite(value);
                    if (value > best) {
                        best = value;
                        id = c;
                    }
                }
                if (!valid) {
                    ++rejected;
                    continue;
                }
                if (best < raw_threshold)
                    continue;
                double d[4]{};
                for (int side = 0; side < 4 && valid; ++side) {
                    double maximum = -std::numeric_limits<double>::infinity();
                    for (int bin = 0; bin < cfg.reg_bins; ++bin) {
                        const float value = reg.at(side * cfg.reg_bins + bin, y, x);
                        if (!std::isfinite(value)) {
                            valid = false;
                            break;
                        }
                        maximum = std::max(maximum, double(value));
                    }
                    if (!valid)
                        break;
                    double sum = 0, weighted = 0;
                    for (int bin = 0; bin < cfg.reg_bins; ++bin) {
                        const double p =
                            std::exp(double(reg.at(side * cfg.reg_bins + bin, y, x)) - maximum);
                        sum += p;
                        weighted += p * bin;
                    }
                    d[side] = weighted / sum;
                }
                if (!valid) {
                    ++rejected;
                    continue;
                }
                Detection b{id,
                            float(1 / (1 + std::exp(-double(best)))),
                            float((x + .5 - d[0]) * stride),
                            float((y + .5 - d[1]) * stride),
                            float((x + .5 + d[2]) * stride),
                            float((y + .5 + d[3]) * stride)};
                b.x1 = std::clamp((b.x1 - g.left) * g.source_w / g.resized_w, 0.f,
                                  float(g.source_w));
                b.x2 = std::clamp((b.x2 - g.left) * g.source_w / g.resized_w, 0.f,
                                  float(g.source_w));
                b.y1 = std::clamp((b.y1 - g.top) * g.source_h / g.resized_h, 0.f,
                                  float(g.source_h));
                b.y2 = std::clamp((b.y2 - g.top) * g.source_h / g.resized_h, 0.f,
                                  float(g.source_h));
                if (b.x2 > b.x1 && b.y2 > b.y1)
                    boxes.push_back(b);
            }
        }
    }
    finishDetections(cfg, workspace, kept);
}

std::vector<Detection> YoloDflDecoder::decode(const std::vector<Tensor> &tensors,
                                              const ModelPlan &plan, const Geometry &g,
                                              std::size_t &rejected) {
    plan.validateOutputs(tensors);
    geometryValid(g);
    detail::DetectionDecoderWorkspace workspace;
    std::vector<Detection> result;
    detail::decodeDetectionValidated(tensors, plan, g, workspace, result, rejected);
    return result;
}

namespace {
void ensureSegmentationAxes(const Geometry &g, const Tensor &tensor,
                            const SegmentationConfig &config,
                            detail::SegmentationDecoderWorkspace &workspace) {
    const detail::SegmentationAxisKey next{g.source_w,
                                           g.source_h,
                                           g.resized_w,
                                           g.resized_h,
                                           g.left,
                                           g.top,
                                           g.input_w,
                                           g.input_h,
                                           tensor.width,
                                           tensor.height,
                                           config.align_corners,
                                           config.bilinear_logits,
                                           std::fegetround()};
    if (workspace.axis_key_valid && workspace.axis_key == next)
        return;
    workspace.axis_key = next;
    workspace.axis_key_valid = true;
    const auto fill = [&](std::vector<detail::SegmentationAxisSample> &result, int source,
                          int resized, int padding, int input, int logits) {
        result.clear();
        result.reserve(source);
        for (int i = 0; i < source; ++i) {
            const float position = coordinate(i, resized, source, config.align_corners) + padding;
            const float mapped = config.align_corners
                                     ? (input == 1 ? 0
                                                   : position * (logits - 1) / (input - 1))
                                     : (position + .5f) * logits / input - .5f;
            if (config.bilinear_logits) {
                const float clamped = std::clamp(mapped, 0.f, float(logits - 1));
                const int first = int(clamped);
                result.push_back(
                    {first, std::min(first + 1, logits - 1), clamped - first});
            } else {
                const int nearest = std::clamp(int(std::lround(mapped)), 0, logits - 1);
                result.push_back({nearest, nearest, 0});
            }
        }
    };
    fill(workspace.columns, g.source_w, g.resized_w, g.left, g.input_w, tensor.width);
    fill(workspace.rows, g.source_h, g.resized_h, g.top, g.input_h, tensor.height);
}
} // namespace

detail::SegmentationDecodeTrace detail::takeLastSegmentationDecodeTrace() noexcept {
    const auto result = last_segmentation_decode_trace;
    last_segmentation_decode_trace = {};
    return result;
}

void detail::decodeSegmentationValidated(const std::vector<Tensor> &tensors,
                                         const ModelPlan &plan, const Geometry &g,
                                         SegmentationDecoderWorkspace &workspace,
                                         std::vector<std::uint8_t> &mask) {
    last_segmentation_decode_trace = {};
    const auto &cfg = plan.segmentation();
    const auto &t = tensors[cfg.seg_output];
    mask.resize(pixels(g.source_w, g.source_h));
    ensureSegmentationAxes(g, t, cfg, workspace);
    const auto &columns = workspace.columns;
    const auto &rows = workspace.rows;
    const std::size_t plane = std::size_t(t.width) * t.height;
    // Tensor validation above guarantees owned, contiguous CHW storage and all
    // indices below are clamped. Avoid four out-of-line Tensor::at calls per class.
    if (cfg.bilinear_logits) {
        struct LogitRow {
            int index = -1;
            std::vector<PointSummary> *points = nullptr;
        };
        // In uniform regions interpolation cannot change the winning class, but
        // float rounding can turn a small lead into a first-class tie. Only skip
        // interpolation when the corner margins also cover rounding error.
        // Avoid extra class scans unless at least one dimension is upsampled.
#if defined(__FAST_MATH__)
        bool shortcut = false;
#else
        bool shortcut = g.source_w >= t.width && g.source_h >= t.height &&
                        (g.source_w > t.width || g.source_h > t.height) &&
                        std::numeric_limits<float>::is_iec559 &&
                        std::numeric_limits<double>::is_iec559 &&
                        std::numeric_limits<float>::radix == 2 &&
                        std::numeric_limits<double>::radix == 2 &&
                        std::numeric_limits<float>::digits == 24 &&
                        std::numeric_limits<double>::digits >= 53 &&
                        std::fegetround() == FE_TONEAREST;
#endif
        last_segmentation_decode_trace.shortcut_eligible = shortcut;
        LogitRow upper{-1, &workspace.upper_points};
        LogitRow lower{-1, &workspace.lower_points};
        auto &cell_winner = workspace.cell_winner;
        if (shortcut) {
            upper.points->resize(t.width);
            lower.points->resize(t.width);
            cell_winner.resize(t.width, -2);
        }
        int cached_first = -1, cached_second = -1;
        int sampled_cells = 0, certified_cells = 0;
        const auto reset_row = [](LogitRow &cache, int row) {
            cache.index = row;
            for (auto &point : *cache.points)
                point.winner = -1;
        };
        const auto point_summary = [&](LogitRow &cache, int x) -> const PointSummary & {
            auto &result = (*cache.points)[x];
            if (result.winner >= 0)
                return result;
            const float *point = t.data.data() + std::size_t(cache.index) * t.width + x;
            double largest = -std::numeric_limits<double>::infinity();
            double second = largest;
            double max_abs = 0;
            int winner = 0;
            for (int c = 0; c < t.channels; ++c) {
                const double value = double(point[std::size_t(c) * plane]);
                max_abs = std::max(max_abs, std::abs(value));
                if (value > largest) {
                    second = largest;
                    largest = value;
                    winner = c;
                } else if (value > second) {
                    second = value;
                }
            }
            result = {largest - second, max_abs, winner};
            return result;
        };
        const auto certify_cell = [&](int left, int right, bool same_row) {
            const auto &a = point_summary(upper, left);
            const auto &b = point_summary(upper, right);
            if (a.winner != b.winner)
                return -1;
            auto &bottom = same_row ? upper : lower;
            const auto &c = point_summary(bottom, left);
            if (a.winner != c.winner)
                return -1;
            const auto &d = point_summary(bottom, right);
            if (a.winner != d.winner)
                return -1;
            const double margin = std::min({a.margin, b.margin, c.margin, d.margin});
            const double max_abs = std::max({a.max_abs, b.max_abs, c.max_abs, d.max_abs});
            // Double arithmetic avoids overflow even for opposite FLT_MAX logits.
            // The normal-number floor also covers environments flushing float
            // subnormals to zero; near-zero/tied scores retain the full path.
            const double guard = 64.0 * std::numeric_limits<float>::epsilon() * max_abs +
                                 64.0 * std::numeric_limits<float>::min();
            return margin > guard ? a.winner : -1;
        };
        for (int y = 0; y < g.source_h; ++y) {
            const auto &row = rows[y];
            if (shortcut && (cached_first != row.first || cached_second != row.second)) {
                // Reuse the previous lower row as the next upper row. Only two
                // logit rows and one cell row are retained: O(logit width) memory.
                if (upper.index != row.first) {
                    if (lower.index == row.first)
                        std::swap(upper, lower);
                    else
                        reset_row(upper, row.first);
                }
                if (row.second != row.first && lower.index != row.second)
                    reset_row(lower, row.second);
                std::fill(cell_winner.begin(), cell_winner.end(), -2);
                cached_first = row.first;
                cached_second = row.second;
            }
            const float *first_row = t.data.data() + std::size_t(row.first) * t.width;
            const float *second_row = t.data.data() + std::size_t(row.second) * t.width;
            auto *output = mask.data() + std::size_t(y) * g.source_w;
            const double wy = row.weight;
            for (int x = 0; x < g.source_w; ++x) {
                const auto &column = columns[x];
                if (shortcut) {
                    int winner = cell_winner[column.first];
                    if (winner == -2) {
                        winner = certify_cell(column.first, column.second,
                                              row.first == row.second);
                        cell_winner[column.first] = winner;
                        ++sampled_cells;
                        certified_cells += winner >= 0;
                        // Each block of 128 newly visited cells probes whether
                        // their margins justify the extra class scans. Mixed
                        // regions disable caching for the rest of this frame.
                        // This only selects an algorithm; both paths have the
                        // same interpolation/argmax result.
                        if (sampled_cells == 128) {
                            if (last_segmentation_decode_trace.first_128_certified < 0)
                                last_segmentation_decode_trace.first_128_certified = certified_cells;
                            if (certified_cells < 32) {
                                shortcut = false;
                                last_segmentation_decode_trace.fallback = true;
                                last_segmentation_decode_trace.fallback_y = y;
                                last_segmentation_decode_trace.fallback_certified = certified_cells;
                            }
                            sampled_cells = certified_cells = 0;
                        }
                    }
                    if (winner >= 0) {
                        output[x] = static_cast<std::uint8_t>(winner);
                        continue;
                    }
                }
                const double wx = column.weight;
                int best = 0;
                float score = -std::numeric_limits<float>::infinity();
                for (int c = 0; c < t.channels; ++c) {
                    const float *top = first_row + std::size_t(c) * plane;
                    const float *bottom = second_row + std::size_t(c) * plane;
                    // Preserve double interpolation, operation grouping, and the
                    // float rounding before argmax, including first-class ties.
                    const float v = float((1 - wy) * ((1 - wx) * top[column.first] +
                                                      wx * top[column.second]) +
                                          wy * ((1 - wx) * bottom[column.first] +
                                                wx * bottom[column.second]));
                    if (v > score) {
                        score = v;
                        best = c;
                    }
                }
                output[x] = static_cast<std::uint8_t>(best);
            }
        }
    } else {
        for (int y = 0; y < g.source_h; ++y) {
            const float *row = t.data.data() + std::size_t(rows[y].first) * t.width;
            auto *output = mask.data() + std::size_t(y) * g.source_w;
            for (int x = 0; x < g.source_w; ++x) {
                const float *pixel = row + columns[x].first;
                int best = 0;
                float score = -std::numeric_limits<float>::infinity();
                for (int c = 0; c < t.channels; ++c) {
                    const float value = pixel[std::size_t(c) * plane];
                    if (value > score) {
                        score = value;
                        best = c;
                    }
                }
                output[x] = static_cast<std::uint8_t>(best);
            }
        }
    }
    last_segmentation_decode_trace.valid = true;
}

std::vector<std::uint8_t> SemanticDecoder::decode(const std::vector<Tensor> &tensors,
                                                  const ModelPlan &plan, const Geometry &g) {
    const auto &cfg = plan.segmentation();
    plan.validateOutputs(tensors);
    geometryValid(g);
    require(cfg.seg_output >= 0 && std::size_t(cfg.seg_output) < tensors.size(),
            "seg_output out of range");
    const auto &tensor = tensors[cfg.seg_output];
    require(tensor.channels == cfg.classes, "segmentation class count mismatch");
    for (float value : tensor.data)
        require(std::isfinite(value), "segmentation contains NaN/Inf");
    detail::SegmentationDecoderWorkspace workspace;
    std::vector<std::uint8_t> result;
    detail::decodeSegmentationValidated(tensors, plan, g, workspace, result);
    return result;
}

} // namespace perception
