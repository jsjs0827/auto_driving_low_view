#include "../include/percep_main/raw_lux_runner.hpp"
#include "../vendor/models.generated.hpp"
#include "raw_feature_kernel.hpp"
#include <chrono>
#include <mutex>
#include <set>
#include <stdexcept>

namespace perception {
namespace {
using Clock = std::chrono::steady_clock;
constexpr std::size_t frame_bytes = 1088u * 1280u * 2u;
static_assert(raw_lux_detail::kWidth == raw_lux1::width &&
              raw_lux_detail::kHeight == raw_lux1::height &&
              raw_lux_detail::kRoiX == raw_lux1::roi[0] &&
              raw_lux_detail::kRoiY == raw_lux1::roi[1] &&
              raw_lux_detail::kRoiX + raw_lux_detail::kRoiWidth == raw_lux1::roi[2] &&
              raw_lux_detail::kRoiY + raw_lux_detail::kRoiHeight == raw_lux1::roi[3],
              "Native RAW kernel must match the training image and ROI");
double ms(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double,std::milli>(b-a).count();
}
void image_contract(const RawLuxImageView& image) {
    if (!image.data || image.bytes != frame_bytes || image.width != raw_lux1::width ||
        image.height != raw_lux1::height || image.stride_bytes != 2176 ||
        image.format != RawLuxPixelFormat::UInt16LE)
        throw std::invalid_argument("RAW lux requires native BGGR1088x1280, unpacked uint16LE, stride2176 and exactly2785280 bytes; no resize/rotation/packed input");
}
// In-memory adaptation of vendor extract_features. The source file function
// remains unmodified for parity checks. Arithmetic/order is deliberately equal.
std::vector<double> extract_buffer(const RawLuxImageView& image) {
    image_contract(image);
    const auto counts = raw_lux_detail::native_counts(image.data);
    std::array<std::uint64_t,1024> cumulative{};
    std::partial_sum(counts.begin(),counts.end(),cumulative.begin());
    const auto n=cumulative.back();
    const auto percentile = [&](double fraction) {
        const double rank=static_cast<double>(n-1)*fraction;
        const auto lo=static_cast<std::uint64_t>(std::floor(rank));
        const auto hi=static_cast<std::uint64_t>(std::ceil(rank));
        const auto a=std::upper_bound(cumulative.begin(),cumulative.end(),lo)-cumulative.begin();
        const auto b=std::upper_bound(cumulative.begin(),cumulative.end(),hi)-cumulative.begin();
        return static_cast<double>(a)+(rank-static_cast<double>(lo))*static_cast<double>(b-a);
    };
    std::vector<double> features{percentile(.1),percentile(.5)};
    for(int bin=0;bin<64;++bin) {
        const auto count=std::accumulate(counts.begin()+bin*16,counts.begin()+(bin+1)*16,std::uint64_t{0});
        features.push_back(static_cast<double>(count)/static_cast<double>(n));
    }
    return features;
}
}

struct RawLuxRunner::Impl {
    mutable std::mutex mutex;
    RawLuxConfig config;
    const raw_lux1::Model* model=nullptr;
    std::string error;
    RawLuxResult compute(const std::string& group_id,
                         const std::vector<RawLuxImageView>& left,
                         const std::vector<RawLuxImageView>& right,
                         Clock::time_point start, Clock::time_point acquired);
};
RawLuxRunner::RawLuxRunner():impl_(std::make_unique<Impl>()) {}
RawLuxRunner::~RawLuxRunner()=default;

bool RawLuxRunner::init(const RawLuxConfig& config) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->model=nullptr; impl_->error.clear();
    try {
        if(config.max_session_pairs<1 || config.max_session_pairs>4096)
            throw std::invalid_argument("max_session_pairs must be in[1,4096]");
        const raw_lux1::Model* model=nullptr;
        for(const auto& candidate:raw_lux1::bundled_models())
            if(candidate.name==config.model_name) model=&candidate;
        if(!model) throw std::invalid_argument("Unsupported RAW lux model: "+config.model_name);
        if(model->requires_metadata) {
            if(config.metadata_source!="org_proxy" && config.metadata_source!="verified_raw")
                throw std::invalid_argument("Main RAW lux model requires explicit metadata_source=org_proxy or verified_raw");
        } else if(!config.metadata_source.empty() && config.metadata_source!="not_used") {
            throw std::invalid_argument("RAW-only model does not take a metadata_source; choose empty or not_used explicitly");
        }
        impl_->config=config; impl_->model=model; return true;
    } catch(const std::exception& e) { impl_->error=e.what(); }
      catch(...) { impl_->error="Unknown RAW lux initialization failure"; }
    return false;
}

bool RawLuxRunner::predictPair(const std::string& group_id,const RawLuxImageView& left,
                              const RawLuxImageView& right,RawLuxResult* result) {
    return predictSession(group_id,{{left,right}},result);
}

RawLuxResult RawLuxRunner::Impl::compute(const std::string& group_id,
                                        const std::vector<RawLuxImageView>& left,
                                        const std::vector<RawLuxImageView>& right,
                                        Clock::time_point start, Clock::time_point acquired) {
    if(!model) throw std::logic_error("RawLuxRunner has not been initialized");
    if(group_id.empty() || group_id.size()>4096)
        throw std::invalid_argument("RAW lux group_id must contain1..4096 bytes");
    if(left.empty() || right.empty() || left.size()>config.max_session_pairs || right.size()>config.max_session_pairs)
        throw std::invalid_argument("RAW lux session requires both eyes within max_session_pairs per eye");
    const auto& model=*this->model;
    const auto feature_start=Clock::now();
    std::vector<raw_lux1::Record> records;records.reserve(left.size()+right.size());
    std::set<const std::uint8_t*> addresses;
    for(std::size_t eye=0;eye<2;++eye) {
        const auto& frames=eye?right:left;
        std::set<std::uint64_t> ids,stamps;
        for(const auto& image:frames) {
            image_contract(image);
            if(image.frame_id && !ids.insert(image.frame_id).second)
                throw std::invalid_argument("RAW lux session repeats a known capture frame ID within one eye");
            if(image.timestamp_ns && !stamps.insert(image.timestamp_ns).second)
                throw std::invalid_argument("RAW lux session repeats a known capture timestamp within one eye");
            if(!addresses.insert(image.data).second)
                throw std::invalid_argument("RAW lux must not reuse the same image buffer within a session, including across eyes");
            auto values=extract_buffer(image);
            if(model.requires_metadata) {
                values.push_back(raw_lux1::positive(image.exposure_us,"exposure_us"));
                values.push_back(raw_lux1::positive(image.analog_gain,"analog_gain(linear multiplier)"));
            }
            records.push_back({group_id,eye?"right":"left",std::move(values)});
        }
    }
        const auto classify_start=Clock::now();
        const auto prediction=raw_lux1::predict(model,group_id,records);
        const auto classify_end=Clock::now();
        RawLuxResult done;
        done.valid=true; done.group_id=prediction.group_id;
        done.predicted_class=raw_lux1::classes[prediction.class_index];done.class_index=prediction.class_index;
        done.probabilities=prediction.probabilities; done.features=prediction.features;
        done.feature_names=model.feature_names;done.frame_counts=prediction.frame_counts;
        done.outside_training=prediction.outside_training;
        done.model_name=model.name;done.model_source_sha256=model.source_sha256;
        done.metadata_source=model.requires_metadata?config.metadata_source:"not_used";
        done.label_definitions=model.label_definitions;done.warning=model.warning;
        done.single_sample_accuracy_unvalidated=prediction.frame_counts[0]==1 && prediction.frame_counts[1]==1;
        // This flag remains false as in the original CLI: identity checks do not
        // independently prove the relationship between stereo exposure telemetry.
        done.stereo_same_exposure_verified=false;
        done.metadata_source_transfer_unvalidated=model.requires_metadata && config.metadata_source=="verified_raw";
        done.timing.feature_ms=ms(feature_start,classify_start);
        done.timing.classification_ms=ms(classify_start,classify_end);
        done.timing.lock_wait_ms=ms(start,acquired);
        done.timing.total_ms=ms(start,Clock::now());
    return done;
}

bool RawLuxRunner::predictSession(const std::string& group_id,
                                 const std::vector<RawLuxStereoPair>& pairs,RawLuxResult* result) {
    const auto start=Clock::now();if(result)*result={};
    std::lock_guard<std::mutex> lock(impl_->mutex);const auto acquired=Clock::now();impl_->error.clear();
    try {
        if(!result)throw std::invalid_argument("RAW lux result pointer is null");
        if(pairs.empty() || pairs.size()>impl_->config.max_session_pairs)
            throw std::invalid_argument("RAW lux session has no stereo pairs or exceeds max_session_pairs");
        std::vector<RawLuxImageView> left,right;left.reserve(pairs.size());right.reserve(pairs.size());
        for(const auto& pair:pairs) {
            if((pair.left.frame_id||pair.right.frame_id)&&pair.left.frame_id!=pair.right.frame_id)
                throw std::invalid_argument("RAW lux left/right known frame IDs must match and both be present");
            if((pair.left.timestamp_ns||pair.right.timestamp_ns)&&pair.left.timestamp_ns!=pair.right.timestamp_ns)
                throw std::invalid_argument("RAW lux left/right known capture timestamps must match and both be present");
            left.push_back(pair.left);right.push_back(pair.right);
        }
        *result=impl_->compute(group_id,left,right,start,acquired);return true;
    } catch(const std::exception& e){impl_->error=e.what();}
      catch(...){impl_->error="Unknown RAW lux inference failure";}
    if(result){*result={};result->reason=impl_->error;result->timing.total_ms=ms(start,Clock::now());}
    return false;
}

bool RawLuxRunner::predictEyeSession(const std::string& group_id,
                                    const std::vector<RawLuxImageView>& left,
                                    const std::vector<RawLuxImageView>& right,RawLuxResult* result) {
    const auto start=Clock::now();if(result)*result={};
    std::lock_guard<std::mutex> lock(impl_->mutex);const auto acquired=Clock::now();impl_->error.clear();
    try {
        if(!result)throw std::invalid_argument("RAW lux result pointer is null");
        *result=impl_->compute(group_id,left,right,start,acquired);return true;
    } catch(const std::exception& e){impl_->error=e.what();}
      catch(...){impl_->error="Unknown RAW lux eye-session inference failure";}
    if(result){*result={};result->reason=impl_->error;result->timing.total_ms=ms(start,Clock::now());}
    return false;
}
std::string RawLuxRunner::lastError() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->error;
}
} // namespace perception
