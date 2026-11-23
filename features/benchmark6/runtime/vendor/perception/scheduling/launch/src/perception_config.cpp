#include "native_config_detail.hpp"
#include "product_graph.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#endif
namespace {
void help() {
  std::cout << "Pure C++ pipeline configuration and launcher\n"
               "  perception_config [run] --config FILE --run-dir NEW [--prepare-only] [--both]\n"
               "  perception_config camera --config FILE --mode infer|map|both|recharge "
               "[--ros-args ...]\n"
               "  perception_config capture --config FILE --output NEW [--frames 100]\n"
               "      [--sdk FILE --camera-config FILE --timeout-ms 3000]\n"
               "All relative user configuration paths are based on the main pipeline file.\n";
}
} // namespace
int configurationMain(int argc, char **argv) {
  try {
    std::string command = "run", mode = "infer", sdk, camera;
    v3::Path config, run, output;
    bool prepare = false, both = false;
    int frames = 100, timeout = 0;
    std::vector<std::string> ros;
    int i = 1;
    if (i < argc && argv[i][0] != '-')
      command = argv[i++];
    for (; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--help" || option == "-h") {
        help();
        return 0;
      }
      if (option == "--prepare-only") {
        prepare = true;
        continue;
      }
      if (option == "--both") {
        both = true;
        continue;
      }
      if (option == "--ros-args") {
        for (; i < argc; ++i)
          ros.push_back(argv[i]);
        break;
      }
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing value for " + option);
      const std::string value = argv[++i];
      if (option == "--config")
        config = std::filesystem::u8path(value);
      else if (option == "--run-dir")
        run = std::filesystem::u8path(value);
      else if (option == "--output")
        output = std::filesystem::u8path(value);
      else if (option == "--mode")
        mode = value;
      else if (option == "--sdk")
        sdk = value;
      else if (option == "--camera-config")
        camera = value;
      else if (option == "--frames" || option == "--timeout-ms") {
        std::size_t used = 0;
        const int n = std::stoi(value, &used);
        if (used != value.size())
          throw std::invalid_argument("Invalid integer for " + option);
        if (option == "--frames")
          frames = n;
        else
          timeout = n;
      } else
        throw std::invalid_argument("Unknown option: " + option);
    }
    if (config.empty())
      config =
          v3::detail::projectRoot(std::filesystem::absolute(std::filesystem::u8path(argv[0]))) /
          "pipeline.json";
    if (command == "camera") {
      if (!run.empty() || !output.empty() || both)
        throw std::invalid_argument("camera does not accept run/output/both options");
      auto args = v3::cameraArguments(config, mode);
      args.insert(args.end(), ros.begin(), ros.end());
      if (prepare) {
        for (const auto &item : args)
          std::cout << item << '\n';
        return 0;
      }
      return v3::launch(args, v3::cameraEnvironment(args));
    }
    if (command == "capture") {
      if (output.empty())
        throw std::invalid_argument("capture requires --output NEW");
      auto args = v3::captureArguments(config, output, frames, sdk, camera, timeout);
      if (prepare) {
        for (const auto &item : args)
          std::cout << item << '\n';
        return 0;
      }
      return v3::launch(args, v3::cameraEnvironment(args));
    }
    if (command != "run")
      throw std::invalid_argument("Unknown command: " + command);
    if (!output.empty() || !ros.empty() || !sdk.empty() || !camera.empty() || timeout)
      throw std::invalid_argument("run accepts only --config, --run-dir, --prepare-only, --both");
    if (both) {
      using namespace v3::detail;
      const auto request = v3::readJson(config);
      if(get(get(request,"recording"),"suite").text == "three_way")
        throw std::invalid_argument("recording.suite=three_way already selects all three groups; use run_pipeline.sh without --both");
      const auto &input = get(request, "input");
      const auto plan = r33::full::parseProductGraph(get(request, "graph"));
      if (get(input, "mode").text != "recording" ||
          get(input, "disparity_source", str("raft_stereo")).text != "raft_stereo" ||
          !(plan.enabled("cloud") || plan.enabled("disparity")))
        throw std::invalid_argument("--both requires recording input, raft_stereo model "
                                    "configuration and enabled cloud/disparity");
    }
    if (run.empty()) {
      const auto tick = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
      run = std::filesystem::temp_directory_path() / ("perception-cpp-v3-" + std::to_string(tick));
    }
    auto prepared = v3::prepare(config, run);
    if (both) {
      if (prepared.mode != "recording" || prepared.method != "raft_stereo")
        throw std::invalid_argument(
            "--both requires recording input and raft_stereo model configuration");
      prepared.argv.push_back("--both");
    }
    std::cout << "Run directory: " << prepared.run_directory << '\n';
    if (prepare) {
      std::cout << "PREPARED ONLY: no camera, subscriber, inference or benchmark was started.\n";
      return 0;
    }
    return v3::launch(prepared);
  } catch (const std::exception &error) {
    std::cerr << "Configuration error: " << error.what() << '\n';
    return 2;
  }
}
#if defined(_WIN32)
int wmain(int argc, wchar_t **arguments) {
  std::vector<std::string> utf8;
  std::vector<char *> argv;
  for (int i = 0; i < argc; ++i) {
    const int bytes =
        WideCharToMultiByte(CP_UTF8, 0, arguments[i], -1, nullptr, 0, nullptr, nullptr);
    if (bytes < 1)
      return 2;
    std::string value(static_cast<std::size_t>(bytes), '\0');
    WideCharToMultiByte(CP_UTF8, 0, arguments[i], -1, value.data(), bytes, nullptr, nullptr);
    value.pop_back();
    utf8.push_back(std::move(value));
  }
  for (auto &value : utf8)
    argv.push_back(value.data());
  argv.push_back(nullptr);
  return configurationMain(argc, argv.data());
}
#else
int main(int argc, char **argv) { return configurationMain(argc, argv); }
#endif
