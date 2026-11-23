#include "bpu_metrics.hpp"
#include "json.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Json = r33::json::JsonValue;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
Json json(const std::string& text) { return r33::json::JsonParser(text, "bpu-test").parse(); }
double numeric(const Json& object, const char* key) {
  const auto& value = object.object.at(key);
  require(value.kind == Json::Kind::Number, "expected JSON number");
  return std::stod(value.text);
}
void status(const Json& object, const char* expected) {
  require(object.object.at("status").text == expected, "unexpected BPU status");
}
void nullMetric(const Json& object, const char* key) {
  require(object.object.at(key).kind == Json::Kind::Null, "missing BPU metric was invented");
}
struct Files {
  std::filesystem::path root = std::filesystem::temp_directory_path() /
      ("perception_bpu_contract_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  Files() { std::filesystem::create_directory(root); }
  ~Files() {
    // Remove only the named fixtures owned by this test, never recursively.
    std::error_code error;
    for (const auto* name : {"ratio", "other", "samples.jsonl", "reset.jsonl", "disabled.jsonl"})
      std::filesystem::remove(root / name, error);
    std::filesystem::remove(root, error);
  }
  std::string path(const char* name) const { return (root / name).string(); }
  void write(const char* name, const std::string& text) const {
    std::ofstream output(root / name, std::ios::binary | std::ios::trunc);
    output << text;
    require(static_cast<bool>(output), "failed writing test fixture");
  }
};
}

int main() {
  try {
    // Actual X5 kernel bpu_debug.c emits an integer followed by a newline.
    for (const auto* text : {"0\n", "100\n", " \t42\r\n", "007"})
      require(r29::parseBpuRatioPercent(text).has_value(), "valid driver ratio rejected");
    for (const auto& text : {std::string(), std::string(" \n"), std::string("-1"),
                            std::string("101"), std::string("bpu0: 42"), std::string("42%"),
                            std::string("1 99"), std::string("42.5"), std::string("nan"),
                            std::string("bpu not inited!\n"), std::string("4\0", 2),
                            std::string(50, '9')})
      require(!r29::parseBpuRatioPercent(text), "ambiguous/invalid ratio accepted");
    require(*r29::parseBpuRatioPercent("0\n") == 0, "real zero became missing");

    Files files;
    files.write("other", "80\n");
    r29::BpuSamplerOptions options;
    options.sample_interval_ms = 5;
    options.paths = {files.path("ratio"), files.path("other")};
    r29::BpuSampler sampler(options);
    status(json(sampler.summaryJson()), "not_started");
    sampler.begin(files.path("samples.jsonl"));
    status(json(sampler.latestJson()), "unreadable");
    nullMetric(json(sampler.latestJson()), "percent");
    bool rejected = false;
    try { sampler.begin(); } catch (const std::logic_error&) { rejected = true; }
    require(rejected, "overlapping measurement windows accepted");
    files.write("ratio", "40\n");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (json(sampler.latestJson()).object.at("status").text != "ok" &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    sampler.end();
    sampler.end();
    const auto summary = json(sampler.summaryJson());
    status(summary, "partial");
    require(summary.object.at("scope").text == "device_shared", "BPU attribution scope lost");
    require(summary.object.at("aggregation").text == "first_configured_path", "bus/core ratios conflated");
    require(numeric(summary, "mean_pct") == 40 && numeric(summary, "p95_pct") == 40 &&
            numeric(summary, "max_pct") == 40, "primary BPU statistics incorrect");
    require(numeric(summary, "missing_sample_count") >= 1 && numeric(summary, "valid_sample_count") >= 1,
            "missing observations were dropped");
    const auto& secondary = summary.object.at("paths").array.at(1);
    require(numeric(secondary, "mean_pct") == 80, "secondary path statistics lost");
    std::ifstream input(files.path("samples.jsonl"));
    std::string line;
    unsigned lines = 0;
    while (std::getline(input, line)) {
      const auto sample = json(line);
      require(numeric(sample, "elapsed_ms") >= 0 && numeric(sample, "elapsed_ms") <= numeric(summary, "window_ms"),
              "sample outside measurement observation window");
      require(sample.object.at("paths").array.size() == 2, "JSONL path evidence missing");
      ++lines;
    }
    require(lines == numeric(summary, "sample_count"), "JSONL did not retain all observations");

    // Old/warmup values are not sampled until begin. Reuse clears the old group.
    files.write("ratio", "99\n");
    files.write("ratio", "0\n");
    sampler.begin(files.path("reset.jsonl"));
    sampler.end();
    const auto reset = json(sampler.summaryJson());
    status(reset, "available");
    require(numeric(reset, "mean_pct") == 0 && numeric(reset, "missing_sample_count") == 0,
            "previous group or warmup observation leaked into reset window");

    files.write("ratio", "bpu not inited!\n");
    sampler.begin();
    sampler.end();
    const auto unavailable = json(sampler.summaryJson());
    status(unavailable, "unavailable");
    nullMetric(unavailable, "mean_pct");
    status(json(sampler.latestJson()), "invalid_format");
    files.write("ratio", std::string(4097, '0'));
    sampler.begin();
    sampler.end();
    status(json(sampler.latestJson()), "too_large");

    options.enabled = false;
    r29::BpuSampler disabled(options);
    disabled.begin(files.path("disabled.jsonl"));
    disabled.end();
    const auto disabled_summary = json(disabled.summaryJson());
    status(disabled_summary, "disabled");
    require(numeric(disabled_summary, "sample_count") == 0, "disabled sampler read sources");
    nullMetric(disabled_summary, "mean_pct");
    require(std::filesystem::file_size(files.path("disabled.jsonl")) == 0, "disabled JSONL contains invented data");

    rejected = false;
    options.sample_interval_ms = 0;
    try { r29::BpuSampler invalid(options); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "zero sample interval accepted");
    options.sample_interval_ms = 100;
    options.paths = {files.path("ratio"), files.path("ratio")};
    rejected = false;
    try { r29::BpuSampler invalid(options); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "duplicate BPU paths accepted");
    std::cout << "BPU strict parsing, missing/null, window reset, background sampling and JSONL passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
