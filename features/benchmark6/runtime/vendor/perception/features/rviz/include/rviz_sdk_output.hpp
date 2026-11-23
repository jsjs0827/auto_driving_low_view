#pragma once
#include "rviz_settings.hpp"
#include "rviz_transport.hpp"
#include "perception_full_sdk.h"
#include "perception_slip_sdk.h"
#include <chrono>
#include <memory>

namespace perception::rviz {
// Display-only copies of completed SDK results; model/camera buffers are never
// changed or retained. The original capture clock and coordinate frames survive.
Frame sdkFrame(const RvizSettings&, const psdk_frame&, const psdk_full_result_view&,
               const std::string& optical_frame, const std::string& body_frame,
               const psdk_slip_view* slip = nullptr);
class SdkPublisher {
 public:
  explicit SdkPublisher(const std::filesystem::path& config);
  ~SdkPublisher();
  bool publish(const psdk_frame&, const psdk_full_result_view&,
               const std::string& optical_frame, const std::string& body_frame,
               const psdk_slip_view* slip = nullptr);
  bool wantsSlip() const;
  bool connected() const;
  const RvizSettings& settings() const { return settings_; }
  PublisherStats stats() const;
 private:
  RvizSettings settings_;
  std::unique_ptr<AsyncPublisher> publisher_;
  std::uint64_t seen_ = 0;
  std::chrono::steady_clock::time_point last_{};
};
} // namespace perception::rviz
