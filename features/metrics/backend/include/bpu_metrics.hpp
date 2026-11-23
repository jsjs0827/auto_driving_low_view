#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace r29 {

// X5's bpu_debug.c ratio_show emits "%d\n" in percent. Parse the entire
// trimmed value; never extract an arbitrary first number from an error message.
std::optional<double> parseBpuRatioPercent(const std::string& text);

struct BpuSamplerOptions {
  bool enabled = true;
  unsigned sample_interval_ms = 100;
  // Empty selects /sys/devices/system/bpu/bpu0/ratio on X5. The first path
  // supplies the top-level statistics. Other paths are reported separately:
  // a bus ratio and its core ratio must not be averaged or added together.
  std::vector<std::string> paths;
};

// Read-only, device-wide utilization snapshots, not process/model attribution.
// Call begin only after warmup, and end immediately after the measured work.
// Driver counters are not reset: their internal averaging window can overlap
// work before begin. The reported window filters observation times only.
// begin/end must be externally serialized; JSON getters are thread-safe.
class BpuSampler {
 public:
  explicit BpuSampler(BpuSamplerOptions options = {});
  ~BpuSampler();
  BpuSampler(const BpuSampler&) = delete;
  BpuSampler& operator=(const BpuSampler&) = delete;

  // An empty filename keeps samples in memory. Otherwise creates/truncates
  // JSONL before the measurement begins, buffering writes until end().
  void begin(const std::string& samples_jsonl_path = {});
  // Idempotent. Joins the sampler and writes JSONL outside the measured window.
  // File failures throw; destruction performs a best-effort nonthrowing end.
  void end();
  std::string summaryJson() const;
  // Last observed primary value, with age_ms. This is not a per-frame reading.
  std::string latestJson() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace r29
