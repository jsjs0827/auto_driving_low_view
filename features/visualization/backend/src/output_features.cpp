#include "output_features.hpp"
#include "runtime_status.hpp"
#include "visualization.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <system_error>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace r28 {
namespace {
namespace rt = perception::runtime;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now()-start).count();
}
std::string quote(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 32) out << "\\u00" << std::hex << std::setw(2)
                             << std::setfill('0') << unsigned(c) << std::dec;
        else out << char(c);
    }
    return out.str() + '"';
}
std::string parameter(const rt::Parameters& p, const char* key, const char* fallback) {
    const auto i = p.find(key); return i == p.end() ? fallback : i->second;
}
std::vector<std::string> layers(const std::string& value) {
    std::vector<std::string> out; std::set<std::string> seen;
    std::istringstream input(value); std::string item;
    while (std::getline(input,item,',')) {
        const auto first = item.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) throw std::invalid_argument("empty visualization layer");
        item = item.substr(first,item.find_last_not_of(" \t\r\n")-first+1);
        if (item != "raw" && item != "detection" && item != "segmentation" &&
            item != "overlay" && item != "disparity")
            throw std::invalid_argument("unknown visualization layer: " + item);
        if (!seen.insert(item).second) throw std::invalid_argument("duplicate visualization layer: " + item);
        out.push_back(std::move(item));
    }
    if (out.empty()) throw std::invalid_argument("visualization layers is empty");
    return out;
}
void writeNew(const fs::path& path, const std::string& text) {
#ifdef _WIN32
    int fd = _wopen(path.c_str(), _O_WRONLY|_O_CREAT|_O_EXCL|_O_BINARY, _S_IREAD|_S_IWRITE);
#else
    int fd = ::open(path.c_str(), O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC, 0666);
#endif
    if (fd < 0) throw std::system_error(errno,std::generic_category(),"cannot create visualization metadata");
    try {
        std::size_t offset = 0;
        while (offset < text.size()) {
#ifdef _WIN32
            const auto n = _write(fd,text.data()+offset,static_cast<unsigned>(text.size()-offset));
#else
            const auto n = ::write(fd,text.data()+offset,text.size()-offset);
#endif
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) throw std::system_error(n < 0 ? errno : EIO,std::generic_category(),"cannot write visualization metadata");
            offset += std::size_t(n);
        }
    } catch (...) {
#ifdef _WIN32
        _close(fd);
#else
        ::close(fd);
#endif
        throw;
    }
#ifdef _WIN32
    if (_close(fd) != 0)
#else
    if (::close(fd) != 0)
#endif
        throw std::system_error(errno,std::generic_category(),"cannot close visualization metadata");
}
void identity(rt::FeatureContext& c, const char* key, perception::FrameInfo frame, bool required) {
    if (!c.contains(key)) {
        if (required) throw std::invalid_argument(std::string("missing visualization result identity: ")+key);
        return;
    }
    const auto actual = c.get<perception::FrameInfo>(key);
    if (actual->id != frame.id || actual->timestamp_ns != frame.timestamp_ns)
        throw std::invalid_argument(std::string("visualization result frame mismatch: ")+key);
}
void disparityRgb(const perception::fusion::DisparityView& d, std::vector<std::uint8_t>& out) {
    const auto sample_bytes = d.type == perception::fusion::DisparityType::UInt16 ? 2U : 4U;
    if (!d.data || d.width <= 0 || d.height <= 0 || d.width > 4096 || d.height > 4096 ||
        (d.type != perception::fusion::DisparityType::UInt16 && d.type != perception::fusion::DisparityType::Float32) ||
        d.stride_bytes > 16U*1024U*1024U ||
        d.stride_bytes < std::size_t(d.width)*sample_bytes ||
        d.bytes < d.stride_bytes*std::size_t(d.height-1)+std::size_t(d.width)*sample_bytes ||
        !std::isfinite(d.scale) || d.scale <= 0)
        throw std::invalid_argument("invalid visualization disparity buffer");
    out.resize(std::size_t(d.width)*d.height*3);
    // Fixed display range 0..64 pixels, documented in metadata. Black denotes
    // invalid/non-positive data; coloring is output-only and never feeds AI.
    for (int y = 0; y < d.height; ++y) for (int x = 0; x < d.width; ++x) {
        const auto* p = static_cast<const std::uint8_t*>(d.data)+std::size_t(y)*d.stride_bytes+std::size_t(x)*sample_bytes;
        float value;
        if (sample_bytes == 2) { std::uint16_t v; std::memcpy(&v,p,2); value = float(v)/d.scale; }
        else { std::memcpy(&value,p,4); value /= d.scale; }
        auto* rgb = out.data()+(std::size_t(y)*d.width+x)*3;
        if (!std::isfinite(value) || value <= 0) { rgb[0]=rgb[1]=rgb[2]=0; continue; }
        const float t = std::clamp(value/64.f,0.f,1.f);
        rgb[0] = std::uint8_t(std::lround(255*t));
        rgb[1] = std::uint8_t(std::lround(255*(1-std::abs(2*t-1))));
        rgb[2] = std::uint8_t(std::lround(255*(1-t)));
    }
}
class Visualize final : public rt::Feature {
    fs::path dir_;
    std::size_t every_ = 1;
    std::vector<std::string> layers_;
    demo_vis::Style style_;
    bool require_identity_ = false;
    std::uint64_t invocation_ = 0;
    std::vector<std::uint8_t> base_, work_;
    std::function<void(const Status&)> on_error_;
public:
    Visualize(const rt::Parameters& p, std::function<void(const Status&)> on_error)
        : dir_(fs::u8path(parameter(p,"dir","visualization"))),
        layers_(layers(parameter(p,"layers","overlay"))), on_error_(std::move(on_error)) {
        const auto value = parameter(p,"every","1"); std::size_t used=0;
        const auto count = std::stoll(value,&used);
        if (used != value.size() || count < 1 || count > 1000000)
            throw std::invalid_argument("visualization every must be 1..1000000");
        every_ = std::size_t(count);
        const auto alpha = parameter(p,"mask_alpha","0.4");
        style_.mask_alpha = std::stof(alpha,&used);
        if (used != alpha.size() || !std::isfinite(style_.mask_alpha) || style_.mask_alpha < 0 || style_.mask_alpha > 1)
            throw std::invalid_argument("visualization mask_alpha must be in [0,1]");
        const auto strict = parameter(p,"require_identity","false");
        if (strict != "true" && strict != "false" && strict != "1" && strict != "0")
            throw std::invalid_argument("visualization require_identity must be boolean");
        require_identity_ = strict == "true" || strict == "1";
        if (dir_.empty()) throw std::invalid_argument("visualization directory is empty");
        style_.segmentation_names = {"no_grass","grass","flagstone"};
    }
    void execute(rt::FeatureContext& c) override {
        OutputVisualization output; output.invocation = ++invocation_;
        const auto start = Clock::now();
        try {
            output.frame = *c.get<perception::FrameInfo>("input.frame");
            if (invocation_%every_) { output.skipped=true; c.set("output.visualization",output); return; }
            const auto image = c.get<perception::ImageView>("input.image");
            const auto det = c.contains("ai.detection") ? c.get<perception::DetectionResult>("ai.detection") : nullptr;
            const auto seg = c.contains("ai.segmentation") ? c.get<perception::SegmentationResult>("ai.segmentation") : nullptr;
            if (det) identity(c,"ai.detection.frame",output.frame,require_identity_);
            if (seg) identity(c,"ai.segmentation.frame",output.frame,require_identity_);
            for (const auto& layer : layers_) {
                if (layer == "detection" && !det) throw std::invalid_argument("detection visualization requested without ai.detection");
                if (layer == "segmentation" && !seg) throw std::invalid_argument("segmentation visualization requested without ai.segmentation");
                if (layer == "disparity" && !c.contains("input.disparity")) throw std::invalid_argument("disparity visualization requested without input.disparity");
                if (layer == "disparity") identity(c,"input.disparity.frame",output.frame,false);
            }
            const auto render_start = Clock::now();
            demo_vis::convertRgbInto(*image,base_);
            // Validate even when only raw images were requested: stale output
            // dimensions must not produce apparently paired metadata.
            demo_vis::validateOverlay(image->width,image->height,det.get(),seg.get(),style_);
            output.render_ms += elapsed(render_start);
            std::ostringstream name; name << 'f' << std::setw(12) << std::setfill('0') << output.frame.id
                << "-t" << output.frame.timestamp_ns << "-n" << invocation_;
            const auto item = dir_/name.str();
            const auto write_start = Clock::now();
            fs::create_directories(dir_);
            if (!fs::create_directory(item)) throw std::runtime_error("visualization frame directory already exists");
            output.encode_write_ms += elapsed(write_start);
            for (const auto& layer : layers_) {
                int width=image->width,height=image->height;
                const std::vector<std::uint8_t>* rendered=&work_;
                const auto render = Clock::now();
                if (layer == "raw") {
                    rendered=&base_;
                } else if (layer == "disparity") {
                    if (!c.contains("input.disparity")) throw std::invalid_argument("disparity visualization requested without input.disparity");
                    const auto disparity = c.get<perception::fusion::DisparityView>("input.disparity");
                    disparityRgb(*disparity,work_); width=disparity->width; height=disparity->height;
                } else {
                    if (layer == "detection" && !det) throw std::invalid_argument("detection visualization requested without ai.detection");
                    if (layer == "segmentation" && !seg) throw std::invalid_argument("segmentation visualization requested without ai.segmentation");
                    work_.assign(base_.begin(),base_.end());
                    demo_vis::overlayRgb(work_,width,height,
                        layer == "detection" || layer == "overlay" ? det.get() : nullptr,
                        layer == "segmentation" || layer == "overlay" ? seg.get() : nullptr,style_);
                }
                output.render_ms += elapsed(render);
                const auto path=item/(layer+".png"); const auto encode=Clock::now();
                demo_vis::saveRgbPng(path,*rendered,width,height);
                output.encode_write_ms += elapsed(encode);
                output.files.push_back({layer,fs::absolute(path).u8string(),width,height});
            }
            output.output_ms=elapsed(start);
            std::ostringstream json; json.imbue(std::locale::classic()); json << std::setprecision(12)
                << "{\"schema\":\"r28-visualization-v1\",\"frame_id\":" << output.frame.id
                << ",\"timestamp_ns\":" << output.frame.timestamp_ns
                << ",\"timestamp_available\":" << (output.frame.timestamp_ns ? "true" : "false")
                << ",\"width\":" << image->width << ",\"height\":" << image->height
                << ",\"invocation\":" << invocation_ << ",\"identity\":"
                << quote(require_identity_ ? "explicit-result-frame" : "graph-frame-with-optional-result-check")
                << ",\"detection_count\":" << (det ? det->detections.size() : 0)
                << ",\"segmentation_present\":" << (seg ? "true" : "false")
                << ",\"detections\":[";
            bool first_detection=true;
            if (det) for (const auto& box : det->detections) {
                if (!first_detection) json << ','; first_detection=false;
                json << "{\"class_id\":" << box.class_id << ",\"score\":" << box.score
                     << ",\"x1\":" << box.x1 << ",\"y1\":" << box.y1
                     << ",\"x2\":" << box.x2 << ",\"y2\":" << box.y2 << '}';
            }
            json << ']'
                << ",\"mask_alpha\":" << style_.mask_alpha
                << ",\"disparity_display_max_pixels\":64,\"timing_scope\":\"output-only; excluded from AI stage timings\""
                << ",\"render_ms\":" << output.render_ms << ",\"encode_write_ms\":" << output.encode_write_ms
                << ",\"output_before_metadata_ms\":" << output.output_ms << ",\"files\":[";
            bool first=true;
            for (const auto& file : output.files) { if(!first)json<<','; first=false;
                json << "{\"layer\":" << quote(file.layer) << ",\"path\":" << quote(fs::u8path(file.path).filename().u8string())
                     << ",\"width\":" << file.width << ",\"height\":" << file.height << '}'; }
            json << "]}\n";
            const auto metadata=item/"frame.json"; const auto before=Clock::now();
            writeNew(metadata,json.str()); output.encode_write_ms+=elapsed(before);
            output.metadata_path=fs::absolute(metadata).u8string(); output.output_ms=elapsed(start);
            c.set("output.visualization",std::move(output));
        } catch (const std::exception& e) {
            const auto status=statusFromException(e,OutputFailed,"output.visualize",c.taskId(),output.frame);
            if (on_error_) on_error_(status);
            throw Failure(status);
        }
    }
};
}
void registerOutputFeatures(perception::runtime::FeatureRegistry& registry,
                            std::function<void(const Status&)> on_error) {
    using T = rt::FeatureParameterType;
    registry.add("output.visualize",[on_error](const rt::Parameters& p){ return std::make_unique<Visualize>(p,on_error); },
        {{"dir",T::String},{"every",T::Integer},{"layers",T::String},{"mask_alpha",T::Float},{"require_identity",T::Boolean}},
        [](const rt::Parameters& p){ return std::vector<std::string>{"output.visualize:"+fs::u8path(parameter(p,"dir","visualization")).lexically_normal().u8string()}; });
}
}
