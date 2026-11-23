#include "perception/preprocess.hpp"
#include "detail/preprocess_workspace.hpp"

namespace perception {
PreparedNv12 prepareNv12(const ImageView &im, int w, int h, bool letterbox, std::uint8_t rgb_pad) {
    PreprocessConfig config;
    config.width = w;
    config.height = h;
    config.letterbox = letterbox;
    config.pad = rgb_pad;
    detail::PreprocessWorkspace workspace;
    const auto result = workspace.execute(im, config, true, false);
    return {result.geometry, workspace.releaseBytes()};
}
} // namespace perception
