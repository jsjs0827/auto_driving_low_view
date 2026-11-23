#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#if defined(__GNUC__)
#define RAW_LUX_API __attribute__((visibility("default")))
#else
#define RAW_LUX_API
#endif
namespace perception {
// Fixed training profile: native unrotated BGGR1088x1280, unpacked LE uint16,
// stride2176, DN0..1023. First uint16 is excluded; no black/scale/shift transform.
enum class RawLuxPixelFormat { UInt16LE = 0 };
struct RawLuxConfig {
    std::string model_name = "raw_hist_org_logistic";
    // Main model requires explicit org_proxy or verified_raw; raw-only uses empty/not_used.
    std::string metadata_source;
    std::size_t max_session_pairs = 256; // upper bound per eye for either session API
};
struct RawLuxImageView {
    const std::uint8_t* data = nullptr;
    std::size_t bytes = 0;
    int width = 1088, height = 1280;
    std::size_t stride_bytes = 2176;
    RawLuxPixelFormat format = RawLuxPixelFormat::UInt16LE;
    std::uint64_t frame_id = 0, timestamp_ns = 0;
    double exposure_us = 0, analog_gain = 0;
};
struct RawLuxStereoPair { RawLuxImageView left, right; };
struct RawLuxTiming { double feature_ms = 0, classification_ms = 0, total_ms = 0, lock_wait_ms = 0; };
struct RawLuxResult {
    bool valid = false;
    std::string predicted_class = "UNKNOWN";
    std::size_t class_index = 0;
    std::array<double,3> probabilities{};
    std::vector<std::string> feature_names;
    std::vector<double> features;
    std::array<std::size_t,2> frame_counts{};
    std::vector<std::string> outside_training;
    std::string group_id, model_name, model_source_sha256, metadata_source;
    std::array<std::string,3> class_names{{"NOT_LOW_LIGHT","LOW_LIGHT","VERY_LOW_LIGHT"}};
    std::vector<std::string> label_definitions;
    bool probabilities_calibrated = false;
    bool single_sample_accuracy_unvalidated = true;
    bool stereo_same_exposure_verified = false;
    bool metadata_source_transfer_unvalidated = false;
    std::string warning, reason;
    RawLuxTiming timing;
};
// Synchronous borrowed inputs: all data must remain alive/unchanged until return.
// Known nonzero pair IDs/timestamps must agree; zeros mean unknown synchronization.
// Missing metadata never changes the configured model or enables a silent fallback.
class RAW_LUX_API RawLuxRunner {
public:
    RawLuxRunner();
    ~RawLuxRunner();
    RawLuxRunner(const RawLuxRunner&) = delete;
    RawLuxRunner& operator=(const RawLuxRunner&) = delete;
    bool init(const RawLuxConfig& config);
    // Single-left deployment: extracts exactly this one native RAW frame, without
    // synthesizing a right eye. Reuses the bundled model coefficients with the
    // left feature vector directly. Its training-domain/absolute accuracy has
    // not been validated; result.frame_counts is {1,0} and warning states this.
    bool predictLeft(const std::string& group_id, const RawLuxImageView& left,
                     RawLuxResult* result);
    bool predictPair(const std::string& group_id, const RawLuxImageView& left,
                     const RawLuxImageView& right, RawLuxResult* result);
    bool predictSession(const std::string& group_id,
                        const std::vector<RawLuxStereoPair>& pairs, RawLuxResult* result);
    // Original CLI session form: eye counts may differ; no index-wise capture
    // synchronization is claimed. Known identities are deduplicated within each eye.
    bool predictEyeSession(const std::string& group_id,
                           const std::vector<RawLuxImageView>& left_frames,
                           const std::vector<RawLuxImageView>& right_frames, RawLuxResult* result);
    std::string lastError() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace perception
