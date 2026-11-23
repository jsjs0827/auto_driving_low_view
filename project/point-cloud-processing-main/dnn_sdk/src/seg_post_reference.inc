static float ResizeCoord(int dst_index, int dst_size, int src_size,
                  bool align_corners) {
    if (src_size <= 1 || dst_size <= 1) return 0.f;
    if (align_corners) {
        return static_cast<float>(dst_index) * (src_size - 1) /
               static_cast<float>(dst_size - 1);
    }
    return std::max(
        0.f, std::min(static_cast<float>(src_size - 1),
                      (static_cast<float>(dst_index) + 0.5f) * src_size /
                              static_cast<float>(dst_size) -
                          0.5f));
}

void referenceMask(const std::vector<float>& chw, int num_classes,
                   int out_h,int out_w,int input_h,int input_w,
                   int orig_h,int orig_w,int resize_type,const Geometry& meta,
                   bool align_corners,std::vector<uint8_t>& mask) {
    std::vector<uint8_t> coarse_mask;
    coarse(chw, num_classes, out_h, out_w, coarse_mask);
    const size_t plane = static_cast<size_t>(out_h) * out_w;
    mask.resize(static_cast<size_t>(orig_w) * orig_h);

    std::vector<int> x0_map(orig_w), x1_map(orig_w);
    std::vector<float> wx_map(orig_w);
    for (int x = 0; x < orig_w; ++x) {
        float ox = ResizeCoord(x, orig_w, out_w, align_corners);
        if (resize_type != 0) {
            const float valid_x =
                ResizeCoord(x, orig_w, meta.valid_w, align_corners);
            const float input_x = meta.x_shift + valid_x;
            ox = (input_w > 1 && out_w > 1)
                     ? input_x * (out_w - 1) /
                           static_cast<float>(input_w - 1)
                     : 0.f;
        }
        ox = std::max(0.f, std::min(static_cast<float>(out_w - 1), ox));
        x0_map[x] = static_cast<int>(std::floor(ox));
        x1_map[x] = std::min(out_w - 1, x0_map[x] + 1);
        wx_map[x] = ox - x0_map[x];
    }

    auto logit = [&](int c, int y, int x) -> float {
        return chw[static_cast<size_t>(c) * plane +
                   static_cast<size_t>(y) * out_w + static_cast<size_t>(x)];
    };

    for (int y = 0; y < orig_h; ++y) {
        float oy = ResizeCoord(y, orig_h, out_h, align_corners);
        if (resize_type != 0) {
            const float valid_y =
                ResizeCoord(y, orig_h, meta.valid_h, align_corners);
            const float input_y = meta.y_shift + valid_y;
            oy = (input_h > 1 && out_h > 1)
                     ? input_y * (out_h - 1) / static_cast<float>(input_h - 1)
                     : 0.f;
        }
        oy = std::max(0.f, std::min(static_cast<float>(out_h - 1), oy));
        const int y0 = static_cast<int>(std::floor(oy));
        const int y1 = std::min(out_h - 1, y0 + 1);
        const float wy = oy - y0;
        const size_t row0 = static_cast<size_t>(y0) * out_w;
        const size_t row1 = static_cast<size_t>(y1) * out_w;
        for (int x = 0; x < orig_w; ++x) {
            const int x0 = x0_map[x];
            const int x1 = x1_map[x];
            const uint8_t c00 = coarse_mask[row0 + x0];
            const uint8_t c01 = coarse_mask[row0 + x1];
            const uint8_t c10 = coarse_mask[row1 + x0];
            const uint8_t c11 = coarse_mask[row1 + x1];
            const float wx = wx_map[x];
            // 凸组合保持 argmax：单点、水平边、垂直边、四角同类
            if ((wx == 0.f && wy == 0.f) ||
                (c00 == c01 && c00 == c10 && c00 == c11) ||
                (wx == 0.f && c00 == c10) || (wy == 0.f && c00 == c01)) {
                mask[static_cast<size_t>(y) * orig_w + x] = c00;
                continue;
            }
            int best = 0;
            float best_val = -std::numeric_limits<float>::infinity();
            for (int c = 0; c < num_classes; ++c) {
                const float v00 = logit(c, y0, x0);
                const float v01 = logit(c, y0, x1);
                const float v10 = logit(c, y1, x0);
                const float v11 = logit(c, y1, x1);
                const float top = v00 + (v01 - v00) * wx;
                const float bottom = v10 + (v11 - v10) * wx;
                const float val = top + (bottom - top) * wy;
                if (val > best_val) {
                    best_val = val;
                    best = c;
                }
            }
            mask[static_cast<size_t>(y) * orig_w + x] =
                static_cast<uint8_t>(best);
        }
    }
}
