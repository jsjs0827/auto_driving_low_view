#include "perception/preprocess.hpp"
#include "detail/preprocess_workspace.hpp"
#include "detail/math.hpp"
namespace perception {
void Tensor::validate() const {
    require(channels > 0 && channels <= 1024 && height > 0 && height <= 4096 && width > 0 &&
                width <= 4096,
            "invalid CHW tensor shape");
    const auto count = static_cast<std::size_t>(channels) * height * width;
    require(count <= 64U * 1024U * 1024U && data.size() == count, "tensor size mismatch/limit");
}
float Tensor::at(int c, int y, int x) const {
    return data[(std::size_t(c) * height + y) * width + x];
}

PreparedImage prepare(const ImageView &im, int w, int h, bool letterbox, std::uint8_t pad) {
    PreprocessConfig config;
    config.width = w;
    config.height = h;
    config.letterbox = letterbox;
    config.pad = pad;
    detail::PreprocessWorkspace workspace;
    const auto result = workspace.execute(im, config, false, false);
    return {result.geometry, workspace.releaseBytes()};
}
std::vector<std::uint8_t> rgbToNv12(const std::vector<std::uint8_t> &rgb, int w, int h) {
    const auto n = pixels(w, h);
    std::vector<std::uint8_t> out(n + n / 2);
    detail::rgbToNv12Into(rgb.data(), rgb.size(), w, h, out.data(), out.size());
    return out;
}
std::vector<float> normalize(const std::vector<std::uint8_t> &rgb, const PreprocessConfig &cfg) {
    require(rgb.size() % 3 == 0, "invalid RGB normalization input");
    std::vector<float> out(rgb.size());
    const auto n = rgb.size() / 3;
    for (int c = 0; c < 3; ++c)
        for (std::size_t i = 0; i < n; ++i)
            out[std::size_t(c) * n + i] =
                (rgb[i * 3 + (cfg.input_bgr ? 2 - c : c)] - cfg.mean[c]) / cfg.stddev[c];
    return out;
}

} // namespace perception
