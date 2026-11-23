#pragma once
#include "json.hpp"
#include <filesystem>
#include <map>
#include <string>
#include <vector>

// Configuration is compiled once before starting a frame-processing binary.
// This library never opens a camera, loads a model or runs an inference session.
namespace v3 {
using Json = r33::json::JsonValue;
using Path = std::filesystem::path;
struct PreparedPipeline {
  Json requested, full, original, parameters, graph, features, recording, stereo, slip_input;
  std::string mode, method, sdk_root, run_directory, project_root;
  std::string deployment_mode = "standalone", algorithm_configuration_sha256;
  Json deployment;
  std::vector<std::string> argv;
  std::map<std::string, std::string> environment;
};
PreparedPipeline prepare(const Path &configuration, const Path &run_directory,
                         bool create_files = true, const std::string &deployment_override = {});
std::string deploymentMode(const Path &configuration, const std::string &override_mode = {});
std::vector<std::string> cameraArguments(const Path &configuration, const std::string &mode);
std::vector<std::string> captureArguments(const Path &configuration, const Path &output,
                                          int frames = 100, const std::string &sdk = {},
                                          const std::string &camera_config = {},
                                          int timeout_ms = 0);
std::map<std::string, std::string> cameraEnvironment(const std::vector<std::string> &arguments);
// Replaces this process on POSIX. Windows reports the prepared argv instead of
// silently changing shell quoting or launching an unsupported target binary.
int launch(const std::vector<std::string> &arguments,
           const std::map<std::string, std::string> &environment = {});
int launch(const PreparedPipeline &pipeline);
Json readJson(const Path &path);
Json readJson(const std::string &utf8_path);
inline Json readJson(const char *utf8_path) { return readJson(std::string(utf8_path)); }
std::string jsonText(const Json &value, int indentation = 2);
// Compile user tasks + dag.sequence/parallel into the existing product graph.
// Legacy graph.nodes configurations remain accepted. No file is written here.
Json normalizePipeline(const Json &requested);
void writeJson(const Path &path, const Json &value);
std::string resolvePath(const Json &value, const Path &base, const std::string &name,
                        const std::string &kind = {}, bool allow_empty = false);
} // namespace v3
