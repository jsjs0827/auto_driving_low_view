#pragma once
#include "native_pipeline.hpp"
#include <filesystem>
#include <string>

namespace camera_compare {
struct Options {
  std::filesystem::path configuration,output;
  std::string method="all";
  bool prepare_only=false;
};
// Pure JSON preparation and serial child dispatch. Camera/model ownership is
// exclusively in each runner process, never in this controller.
int run(const Options& options);
// Independent live runs are never paired by sequence number or frame ID.
// Rebuild summaries only; no same-frame quality difference is generated.
v3::Json report(const std::filesystem::path& output);
} // namespace camera_compare
