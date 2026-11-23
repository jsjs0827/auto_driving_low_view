#include "bpu_metrics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace r29 {
namespace {
using Clock = std::chrono::steady_clock;

std::string quote(const std::string& text, bool escape_high_bytes = false) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : text) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20 || (escape_high_bytes && c >= 0x7f)) {
          out << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(c) << std::dec;
        } else {
          out << c;
        }
    }
  }
  out << '"';
  return out.str();
}

std::string number(std::optional<double> value) {
  if (!value) return "null";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(12) << *value;
  return out.str();
}

double milliseconds(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

struct Reading {
  std::optional<double> percent;
  std::string status;
  std::string raw;
  bool raw_truncated = false;
};

Reading readRatio(const std::string& path) {
  Reading reading;
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    reading.status = "unreadable";
    return reading;
  }
  std::array<char, 4097> buffer{};
  input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
  const auto size = static_cast<std::size_t>(input.gcount());
  if (input.bad() || (input.fail() && !input.eof())) {
    reading.status = "read_error";
    return reading;
  }
  const std::string text(buffer.data(), size);
  reading.raw = text.substr(0, 256);
  reading.raw_truncated = size > reading.raw.size();
  if (size > 4096) {
    reading.status = "too_large";
    return reading;
  }
  reading.percent = parseBpuRatioPercent(text);
  reading.status = reading.percent ? "ok" : "invalid_format";
  return reading;
}

struct Sample {
  Clock::time_point observed;
  double elapsed_ms = 0;
  std::vector<Reading> readings;
};

std::string readingsJson(const std::vector<std::string>& paths,
                         const std::vector<Reading>& readings) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < readings.size(); ++i) {
    if (i) out << ',';
    out << "{\"path\":" << quote(paths[i])
        << ",\"status\":" << quote(readings[i].status)
        << ",\"percent\":" << number(readings[i].percent)
        << ",\"raw\":" << quote(readings[i].raw, true)
        << ",\"raw_truncated\":" << (readings[i].raw_truncated ? "true" : "false") << '}';
  }
  out << ']';
  return out.str();
}

std::string statisticsJson(const std::vector<double>& values, std::size_t count) {
  std::optional<double> mean, p95, maximum;
  if (!values.empty()) {
    auto sorted = values;
    std::sort(sorted.begin(), sorted.end());
    mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
    const double position = (sorted.size() - 1) * 0.95;
    const auto lower = static_cast<std::size_t>(position);
    const auto upper = std::min(lower + 1, sorted.size() - 1);
    p95 = sorted[lower] + (sorted[upper] - sorted[lower]) * (position - lower);
    maximum = sorted.back();
  }
  const char* status = values.empty() ? "unavailable" : values.size() == count ? "available" : "partial";
  std::ostringstream out;
  out << "\"status\":" << quote(status)
      << ",\"sample_count\":" << count
      << ",\"valid_sample_count\":" << values.size()
      << ",\"missing_sample_count\":" << count - values.size()
      << ",\"mean_pct\":" << number(mean)
      << ",\"p95_pct\":" << number(p95)
      << ",\"max_pct\":" << number(maximum);
  return out.str();
}
}  // namespace

std::optional<double> parseBpuRatioPercent(const std::string& text) {
  constexpr const char* whitespace = " \t\r\n\f\v";
  const auto first = text.find_first_not_of(whitespace);
  if (first == std::string::npos) return std::nullopt;
  const auto last = text.find_last_not_of(whitespace);
  unsigned value = 0;
  for (std::size_t i = first; i <= last; ++i) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    if (c < '0' || c > '9') return std::nullopt;
    value = value * 10 + (c - '0');
    if (value > 100) return std::nullopt;
  }
  return static_cast<double>(value);
}

struct BpuSampler::Impl {
  explicit Impl(BpuSamplerOptions initial) : options(std::move(initial)) {
    if (options.sample_interval_ms == 0 || options.sample_interval_ms > 60000)
      throw std::invalid_argument("BPU sample_interval_ms must be in [1, 60000]");
    if (options.paths.empty()) options.paths = {"/sys/devices/system/bpu/bpu0/ratio"};
    std::set<std::string> unique;
    for (const auto& path : options.paths) {
      if (path.empty() || path.find('\0') != std::string::npos || !unique.insert(path).second)
        throw std::invalid_argument("BPU paths must be nonempty and unique");
    }
  }

  void sample() {
    Sample value;
    // Timestamp is taken before reads, so it identifies this observation.
    value.observed = Clock::now();
    value.elapsed_ms = milliseconds(start, value.observed);
    for (const auto& path : options.paths) value.readings.push_back(readRatio(path));
    std::lock_guard<std::mutex> lock(mutex);
    samples.push_back(std::move(value));
  }

  void loop() noexcept {
    try {
      std::unique_lock<std::mutex> lock(mutex);
      while (!wake.wait_for(lock, std::chrono::milliseconds(options.sample_interval_ms),
                            [this] { return stop_requested; })) {
        lock.unlock();
        sample();
        lock.lock();
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(mutex);
      thread_error = error.what();
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      thread_error = "unknown sampler failure";
    }
  }

  BpuSamplerOptions options;
  mutable std::mutex mutex;
  std::condition_variable wake;
  std::thread worker;
  bool started = false;
  bool running = false;
  bool stop_requested = false;
  Clock::time_point start{}, finish{};
  std::vector<Sample> samples;
  std::string thread_error;
  std::ofstream output;
};

BpuSampler::BpuSampler(BpuSamplerOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}

BpuSampler::~BpuSampler() {
  try { end(); } catch (...) {}
}

void BpuSampler::begin(const std::string& samples_jsonl_path) {
  auto& state = *impl_;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.running) throw std::logic_error("BPU sampler already running");
  }
  if (state.output.is_open()) state.output.close();
  state.output.clear();
  if (!samples_jsonl_path.empty()) {
    state.output.open(samples_jsonl_path, std::ios::binary | std::ios::trunc);
    if (!state.output) throw std::runtime_error("Cannot create BPU samples JSONL: " + samples_jsonl_path);
  }
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.samples.clear();
    state.thread_error.clear();
    state.stop_requested = false;
    state.started = true;
    state.running = state.options.enabled;
    state.start = state.finish = Clock::now();
  }
  if (!state.options.enabled) return;
  try {
    state.sample();
    state.worker = std::thread([&state] { state.loop(); });
  } catch (...) {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.running = false;
    state.finish = Clock::now();
    throw;
  }
}

void BpuSampler::end() {
  auto& state = *impl_;
  bool was_running;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    was_running = state.running;
    state.stop_requested = true;
  }
  state.wake.notify_all();
  if (state.worker.joinable()) state.worker.join();
  if (was_running) {
    try {
      state.sample();
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(state.mutex);
      state.thread_error = error.what();
    }
    std::lock_guard<std::mutex> lock(state.mutex);
    state.finish = Clock::now();
    state.running = false;
  }
  if (state.output.is_open()) {
    // Sampling has stopped. Serialization and disk I/O are outside the window.
    for (const auto& sample : state.samples) {
      state.output << "{\"scope\":\"device_shared\",\"elapsed_ms\":" << number(sample.elapsed_ms)
                   << ",\"percent\":" << number(sample.readings.front().percent)
                   << ",\"status\":" << quote(sample.readings.front().status)
                   << ",\"paths\":" << readingsJson(state.options.paths, sample.readings) << "}\n";
    }
    state.output.flush();
    state.output.close();
    if (!state.output) throw std::runtime_error("Failed writing BPU samples JSONL");
  }
}

std::string BpuSampler::summaryJson() const {
  const auto& state = *impl_;
  std::lock_guard<std::mutex> lock(state.mutex);
  std::vector<std::vector<double>> values(state.options.paths.size());
  for (const auto& sample : state.samples) {
    for (std::size_t i = 0; i < sample.readings.size(); ++i)
      if (sample.readings[i].percent) values[i].push_back(*sample.readings[i].percent);
  }
  std::ostringstream out;
  out << "{\"enabled\":" << (state.options.enabled ? "true" : "false")
      << ",\"scope\":\"device_shared\",\"aggregation\":\"first_configured_path\""
      << ",\"statistic\":\"arithmetic_snapshot_mean\",\"kernel_window\":\"driver_defined_not_reset\""
      << ",\"warmup_filter\":\"observations_after_begin_only\""
      << ",\"primary_path\":" << quote(state.options.paths.front())
      << ",\"sample_interval_ms\":" << state.options.sample_interval_ms
      << ",\"window_ms\":" << number(state.started ? std::optional<double>(milliseconds(state.start, state.running ? Clock::now() : state.finish)) : std::nullopt)
      << ",\"running\":" << (state.running ? "true" : "false") << ',';
  if (!state.options.enabled || !state.started) {
    out << "\"status\":" << quote(state.options.enabled ? "not_started" : "disabled")
        << ",\"sample_count\":0,\"valid_sample_count\":0,\"missing_sample_count\":0"
        << ",\"mean_pct\":null,\"p95_pct\":null,\"max_pct\":null";
  } else {
    out << statisticsJson(values.front(), state.samples.size());
  }
  out << ",\"sampler_error\":" << (state.thread_error.empty() ? "null" : quote(state.thread_error))
      << ",\"paths\":[";
  for (std::size_t i = 0; i < state.options.paths.size(); ++i) {
    if (i) out << ',';
    out << "{\"path\":" << quote(state.options.paths[i]) << ','
        << statisticsJson(values[i], state.samples.size())
        << ",\"last_status\":" << (state.samples.empty() ? "null" : quote(state.samples.back().readings[i].status)) << '}';
  }
  out << "]}";
  return out.str();
}

std::string BpuSampler::latestJson() const {
  const auto& state = *impl_;
  std::lock_guard<std::mutex> lock(state.mutex);
  std::ostringstream out;
  out << "{\"scope\":\"device_shared\",\"path\":" << quote(state.options.paths.front());
  if (state.samples.empty()) {
    out << ",\"status\":" << quote(!state.options.enabled ? "disabled" : state.started ? "unavailable" : "not_started")
        << ",\"percent\":null,\"elapsed_ms\":null,\"age_ms\":null";
  } else {
    const auto& sample = state.samples.back();
    out << ",\"status\":" << quote(sample.readings.front().status)
        << ",\"percent\":" << number(sample.readings.front().percent)
        << ",\"elapsed_ms\":" << number(sample.elapsed_ms)
        << ",\"age_ms\":" << number(milliseconds(sample.observed, Clock::now()));
  }
  out << '}';
  return out.str();
}
}  // namespace r29
