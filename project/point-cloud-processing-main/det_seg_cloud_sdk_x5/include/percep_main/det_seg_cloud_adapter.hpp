#pragma once

#include "percep_main/horizon_dnn_runner.hpp"  // Existing application value types.
#include "det_seg.h"

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace perception {

// Owns the C SDK handles and copies borrowed results into the original
// application's containers before releasing them. Both models can run in
// parallel when configured; separate calls are serialized across handles.
class DetSegCloudAdapter {
public:
    DetSegCloudAdapter() = default;
    ~DetSegCloudAdapter() = default;
    DetSegCloudAdapter(const DetSegCloudAdapter&) = delete;
    DetSegCloudAdapter& operator=(const DetSegCloudAdapter&) = delete;

    bool init(const DnnConfig& config);
    // Query the actual loaded model input shape. mode is DS_DETECTION or
    // DS_SEGMENTATION. No model handle is created by this const query.
    // Width/height are cleared on failure; error is optional and does not
    // overwrite lastError() from initialization/inference.
    bool modelInputSize(uint32_t mode, int* width, int* height,
                        std::string* error = nullptr) const;
    bool inferNv12(const uint8_t* yuv, int width, int height, int stride_y,
                   size_t bytes_used, bool source_is_nv12,
                   std::vector<DnnDetection>* detections,
                   std::vector<uint8_t>* seg_mask,
                   std::vector<uint8_t>* overlay_rgb,
                   DnnInferTiming* timing = nullptr);

    // Config is fixed after init. Configure/init on the owning thread only.
    const DnnConfig& config() const { return config_; }
    std::string lastError() const;
    // Returned string pointers borrow config() storage and remain valid until
    // the next init()/destruction. Pass this contract to ds_match_det_cloud_ex.
    ds_model_contract modelContract() const;

    // Processing identity for the most recent successful call. The old entry
    // point provides no capture timestamp, so timestampNs() is explicitly 0;
    // the ID is a local monotonic sequence, not a camera hardware frame ID.
    uint64_t lastFrameId() const;
    uint64_t lastTimestampNs() const { return 0; }

private:
    struct HandleDeleter {
        void operator()(ds_handle* handle) const noexcept {
            if (handle) ds_destroy(&handle);
        }
    };
    using Handle = std::unique_ptr<ds_handle, HandleDeleter>;
    bool createHandle(uint32_t mode);
    ds_model_contract modelContractUnlocked() const;
    void clearHandles();

    mutable std::mutex mutex_;
    DnnConfig config_{};
    std::array<Handle, 4> handles_{}; // Indexed by DS_DETECTION/SEGMENTATION/BOTH.
    std::string error_;
    uint64_t next_frame_id_ = 0;
    uint64_t last_frame_id_ = 0;
    bool initialized_ = false;
};

}  // namespace perception
