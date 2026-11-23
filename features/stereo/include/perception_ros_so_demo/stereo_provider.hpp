#pragma once
#include "perception_full_sdk.h"
#include <memory>
#include <string>

namespace ros_so_demo {
struct StereoSettings {
    std::string model;
    std::string input_mode="crop";
    int input_width=640;
    int input_height=352;
};

// One model is loaded for the lifetime of this provider. Use serially:
// setPair -> synchronous Caller::process -> clearPair. The caller owns both
// image buffers and must retain them, unchanged, until process has returned.
// Conversion and inference execute within the SDK disparity task.
class StereoProvider {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    explicit StereoProvider(const StereoSettings& settings);
    ~StereoProvider();
    StereoProvider(const StereoProvider&)=delete;
    StereoProvider& operator=(const StereoProvider&)=delete;
    void setPair(const psdk_frame& left,const psdk_frame& right);
    void clearPair();
    const psdk_full_hooks* hooks() const;
    double elapsedMs() const;
    bool called() const;
    std::string metadataJson() const;
};
} // namespace ros_so_demo
