#include "rviz_settings.hpp"
#include "json.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#ifndef _WIN32
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {
using namespace perception::rviz;
using Path = std::filesystem::path;
namespace json = r33::json;
int checks = 0;
void require(bool condition, const std::string& message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
void write(const Path& path, const std::string& text) {
  std::ofstream output(path, std::ios::binary);
  output << text;
  if (!output) throw std::runtime_error("Cannot write fixture: " + path.u8string());
}
std::string read(const Path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return {};
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}
void env(const char* key, const std::string& value) {
#ifdef _WIN32
  _putenv_s(key, value.c_str());
#else
  setenv(key, value.c_str(), 1);
#endif
}
Path checkedTemp(const Path& path, const Path& base) {
  const auto name = path.filename().u8string();
  require(path.is_absolute() && name.rfind("rviz-settings-", 0) == 0 && name.size() > 14,
          "unsafe test cleanup filename");
  require(!std::filesystem::is_symlink(std::filesystem::symlink_status(path)), "unsafe test symlink cleanup");
  const auto resolved = std::filesystem::canonical(path);
  require(std::filesystem::equivalent(resolved.parent_path(), base) && resolved.filename() == path.filename(),
          "unsafe test cleanup target");
  return resolved;
}
void invalid(const Path& config, const std::string& value, const std::string& message) {
  write(config, value);
  bool rejected = false;
  try { (void)loadSettings(config); } catch (const std::exception&) { rejected = true; }
  require(rejected, message);
}
#ifndef _WIN32
void fakeLog(const std::string& category, int argc, char** argv) {
  const char* destination = std::getenv("RVIZ_TEST_LOG");
  if (!destination) throw std::runtime_error("Missing fake log path");
  std::ofstream log(destination, std::ios::app);
  log << json::quote(category) << " [";
  for (int i = 0; i < argc; ++i) log << (i ? "," : "") << json::quote(argv[i]);
  log << "]\n";
}
int fakeCommand(const std::string& name, int argc, char** argv) {
  if (name == "r36_rviz_receiver") {
    fakeLog("receiver", argc, argv);
    write(std::getenv("RVIZ_TEST_RECEIVER_PID"), std::to_string(::getpid()));
    if (const char* delay = std::getenv("RVIZ_TEST_RECEIVER_EXIT_AFTER_MS")) {
      if (*delay) { std::this_thread::sleep_for(std::chrono::milliseconds(std::stoi(delay))); return 0; }
    }
    for (int i = 0; i < 200; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return 0;
  }
  if (name == "fake rviz '$()") {
    fakeLog("rviz", argc, argv);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return 0;
  }
  if (name != "fake adb '$()") return -1;
  fakeLog("adb", argc, argv);
  const std::string mode = std::getenv("RVIZ_TEST_ADB_MODE") ? std::getenv("RVIZ_TEST_ADB_MODE") : "online";
  if (argc == 3 && std::string(argv[1]) == "devices" && std::string(argv[2]) == "-l") {
    std::cout << "List of devices attached\n";
    if (mode == "offline") std::cout << "X5TEST offline\n";
    else if (mode == "unauthorized") std::cout << "X5TEST unauthorized\n";
    else {
      std::cout << "X5TEST device product:x5 model:TEST\n";
      if (mode == "multiple") std::cout << "OTHER device product:other\n";
    }
    return 0;
  }
  if (argc < 5 || std::string(argv[1]) != "-s" || std::string(argv[2]) != "X5TEST" || std::string(argv[3]) != "forward") return 10;
  const auto state = std::filesystem::u8path(std::getenv("RVIZ_TEST_FORWARD_STATE"));
  const std::string action(argv[4]);
  if (argc == 5 && action == "--list") {
    if (mode == "collision") std::cout << "OTHER tcp:18765 tcp:9999\n";
    std::cout << read(state);
    return 0;
  }
  if (argc == 7 && action == "--no-rebind" && std::string(argv[5]) == "tcp:18765" && std::string(argv[6]) == "tcp:8765") {
    if (mode == "collision" || mode == "race" || !read(state).empty()) return 11;
    write(state, "X5TEST tcp:18765 tcp:8765\n");
    return 0;
  }
  if (argc == 6 && action == "--remove" && std::string(argv[5]) == "tcp:18765") {
    write(state, "");
    return 0;
  }
  return 12;
}
int launch(const Path& executable, const std::vector<std::string>& arguments) {
  std::vector<std::string> values{executable.u8string()};
  values.insert(values.end(), arguments.begin(), arguments.end());
  std::vector<char*> pointers;
  for (auto& value : values) pointers.push_back(value.data());
  pointers.push_back(nullptr);
  const auto child = ::fork();
  if (child == 0) { ::execv(pointers[0], pointers.data()); _exit(127); }
  if (child < 0) throw std::runtime_error("Cannot fork test launcher");
  int status = 0;
  while (::waitpid(child, &status, 0) < 0) if (errno != EINTR) throw std::runtime_error("Cannot wait for test launcher");
  return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}
void executableCopy(const Path& source, const Path& destination) {
  std::filesystem::copy_file(source, destination);
  std::filesystem::permissions(destination, std::filesystem::perms::owner_all);
}
void launcherContracts(const Path& root, const Path& launcher, const Path& self) {
  const auto binaries = root / "fake tools '$()";
  std::filesystem::create_directory(binaries);
  const auto local_launcher = binaries / "r36_rviz_launcher";
  executableCopy(launcher, local_launcher);
  executableCopy(self, binaries / "r36_rviz_receiver");
  executableCopy(self, binaries / "fake adb '$()");
  executableCopy(self, binaries / "fake rviz '$()");
  auto settings = defaultSettings();
  settings.transport.adb_executable = (binaries / "fake adb '$()").u8string();
  settings.rviz.rviz_executable = (binaries / "fake rviz '$()").u8string();
  const auto config = root / "rviz config '$().json";
  write(config, settingsJson(settings));
  const auto log = root / "argv.jsonl";
  const auto state = root / "forward-state.txt";
  const auto receiver_pid = root / "receiver-pid.txt";
  env("RVIZ_TEST_LOG", log.u8string());
  env("RVIZ_TEST_FORWARD_STATE", state.u8string());
  env("RVIZ_TEST_RECEIVER_PID", receiver_pid.u8string());
  const auto prepare = root / "prepared '$()";
  require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", prepare.u8string(), "--prepare-only"}) == 0,
          "prepare-only must succeed with quoted paths");
  require(!std::filesystem::exists(log), "prepare-only must never start ADB, ROS, or RViz");
  const auto before = read(prepare / "view.rviz");
  require(!before.empty(), "prepare-only must write RViz configuration");
  require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", prepare.u8string(), "--prepare-only"}) != 0,
          "launcher must reject an existing output directory");
  require(read(prepare / "view.rviz") == before, "launcher must not overwrite prepared output");
  env("RVIZ_TEST_ADB_MODE", "online");
  const auto normal = root / "normal '$()";
  require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", normal.u8string()}) == 0,
          "fake ADB + receiver + RViz flow must succeed");
  const auto recorded = read(log);
  require(recorded.find("\"--no-rebind\",\"tcp:18765\",\"tcp:8765\"") != std::string::npos,
          "launcher must use --no-rebind with configured port arguments");
  require(recorded.find("\"--remove\",\"tcp:18765\"") != std::string::npos && read(state).empty(),
          "launcher must remove its owned forward");
  require(recorded.find("\"--display-config\"," + json::quote((normal / "view.rviz").u8string())) != std::string::npos,
          "RViz configuration path must remain one argv value");
  require(recorded.find("\"--config\"," + json::quote((normal / "rviz.resolved.json").u8string()) +
                        ",\"--host\",\"127.0.0.1\",\"--port\",\"18765\"") != std::string::npos,
          "receiver must use resolved JSON and localhost forwarded port");
  const auto pid_text = read(receiver_pid);
  require(!pid_text.empty(), "fake receiver must have started");
  require(::kill(static_cast<pid_t>(std::stol(pid_text)), 0) < 0 && errno == ESRCH, "launcher must reap its receiver child");
  write(log, "");
  env("RVIZ_TEST_RECEIVER_EXIT_AFTER_MS", "100");
  require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", (root / "no-gui").u8string(), "--no-gui"}) == 0,
          "--no-gui must run the receiver and finish cleanly");
  require(read(log).find("\"receiver\"") != std::string::npos && read(log).find("\"rviz\"") == std::string::npos,
          "--no-gui must start receiver without RViz");
  env("RVIZ_TEST_RECEIVER_EXIT_AFTER_MS", "");
  for (const auto* mode : {"multiple", "offline", "unauthorized", "collision", "race"}) {
    write(log, "");
    env("RVIZ_TEST_ADB_MODE", mode);
    require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", (root / mode).u8string()}) != 0,
            std::string("ADB failure must be reported: ") + mode);
    const auto failure_log = read(log);
    require(failure_log.find("\"receiver\"") == std::string::npos && failure_log.find("\"rviz\"") == std::string::npos,
            std::string("ADB failure must not start ROS/RViz: ") + mode);
    require(failure_log.find("\"--remove\"") == std::string::npos, "launcher must not remove an unowned forward");
  }
  write(log, "");
  env("RVIZ_TEST_ADB_MODE", "multiple");
  settings.transport.serial = "X5TEST";
  write(config, settingsJson(settings));
  require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", (root / "explicit-serial").u8string()}) == 0,
          "configured serial must select one device from multiple online devices");
  settings.enabled = false;
  write(config, settingsJson(settings));
  write(log, "");
  require(launch(local_launcher, {"--config", config.u8string(), "--run-dir", (root / "disabled").u8string()}) == 0,
          "disabled configuration must succeed without side effects");
  require(read(log).empty(), "disabled launcher must never invoke ADB/receiver/RViz");
}
#endif
} // namespace

int main(int argc, char** argv) {
  Path temp;
  Path base;
  try {
#ifndef _WIN32
    const auto fake = fakeCommand(std::filesystem::path(argv[0]).filename().u8string(), argc, argv);
    if (fake >= 0) return fake;
#endif
    if (argc < 2 || argc > 3) throw std::invalid_argument("rviz_settings_test PROJECT_ROOT [LAUNCHER_BINARY]");
    const auto project = std::filesystem::absolute(std::filesystem::u8path(argv[1]));
    base = std::filesystem::canonical(std::filesystem::temp_directory_path());
    temp = base / ("rviz-settings-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    require(std::filesystem::create_directory(temp), "fixture directory must be new");
    temp = checkedTemp(temp, base);
    const auto config = temp / "settings.json";
    const auto shipped = loadSettings(project / "rviz.json");
    require(shipped.enabled && shipped.transport.bind_address == "127.0.0.1", "shipped ADB configuration must bind loopback");
    require(shipped.transport.local_port == 18765 && shipped.transport.remote_port == 8765, "ADB port defaults changed");
    require(shipped.publish.max_points == 60000 && shipped.publish.point_stride == 2 && shipped.publish.every_n == 2,
            "USB bandwidth defaults changed");
    require(streamEnabled(shipped, "clouds/camera") && streamEnabled(shipped, "input_rgb"), "camera cloud/RGB must be enabled");
    require(!streamEnabled(shipped, "clouds/robot") && !shipped.rviz.motion_arrow, "body-frame displays must default off");
    write(config, settingsJson(shipped));
    const auto roundtrip = loadSettings(config);
    require(settingsJson(roundtrip) == settingsJson(shipped), "resolved JSON must roundtrip without changes");
    const auto rendered = rvizConfig(roundtrip);
    require(rendered.find("Fixed Frame: \"camera_optical_frame\"") != std::string::npos &&
            rendered.find("Frame Rate: 15") != std::string::npos, "RViz global options must follow JSON");
    require(rendered.find("Class: rviz_default_plugins/PointCloud2") != std::string::npos &&
            rendered.find("Class: rviz_default_plugins/Image") != std::string::npos &&
            rendered.find("Class: rviz_default_plugins/MarkerArray") != std::string::npos,
            "RViz rendering must include cloud, image, and status display types");
    require(rendered.find("Reliability Policy: Best Effort") != std::string::npos &&
            rendered.find("Durability Policy: Volatile") != std::string::npos, "RViz QoS must match receiver defaults");
    for (const auto& value : {
      R"({"schema_version":2})", R"({"schema_version":1,"unexpected":true})",
      R"({"schema_version":1,"transport":{"locl_port":18765}})",
      R"({"schema_version":1,"transport":{"local_port":65536}})",
      R"({"schema_version":1,"transport":{"remote_port":0}})",
      R"({"schema_version":1,"transport":{"max_frame_bytes":67108865}})",
      R"({"schema_version":1,"transport":{"bind_address":"127.0.0.1;echo bad"}})",
      R"({"schema_version":1,"transport":{"serial":"-invalid"}})",
      R"({"schema_version":1,"transport":{"adb_executable":"adb\nother"}})",
      R"({"schema_version":1,"publish":{"max_fps":0}})",
      R"({"schema_version":1,"publish":{"every_n":1.5}})",
      R"({"schema_version":1,"publish":{"max_points":1000001}})",
      R"({"schema_version":1,"publish":{"overlay_alpha":1.1}})",
      R"({"schema_version":1,"publish":{"disparity_min_px":144,"disparity_max_px":144}})",
      R"({"schema_version":1,"publish":{"streams":{"made_up":true}}})",
      R"({"schema_version":1,"rviz":{"frame_rate":0}})",
      R"({"schema_version":1,"rviz":{"fixed_frame":"bad\nframe"}})",
      R"({"schema_version":1,"rviz":{"displays":{"clouds/camera":{"topic":"relative/topic"}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"slip":{"topic":"/perception_rviz/input_rgb"}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"input_rgb":{"topic":"/perception_rviz/status"}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"input_rgb":{"topic":"/perception_rviz/slip/json"}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"slip":{"qos":{"reliability":"unknown"}}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"clouds/camera":{"point_size":101}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"clouds/camera":{"style":"unknown"}}}})",
      R"({"schema_version":1,"rviz":{"displays":{"clouds/camera":{"qos":{"depth":0}}}}})",
      R"({"schema_version":1,"schema_version":1})"
    }) invalid(config, value, "invalid RViz settings must be rejected: " + std::string(value));
    write(config, R"json({"schema_version":1,"transport":{"adb_executable":"tools/adb '$()"},"publish":{"disparity_min_px":4,"disparity_max_px":72,"streams":{"input_rgb":false}},"rviz":{"fixed_frame":"custom_camera","body_frame":"custom_body","rviz_executable":"tools/rviz '$()","displays":{"clouds/camera":{"enabled":false},"slip":{"topic":"/custom/slip","qos":{"reliability":"reliable","durability":"transient_local","depth":3}}}}})json");
    const auto custom = loadSettings(config);
    require(std::filesystem::u8path(custom.transport.adb_executable) == (temp / "tools/adb '$()").lexically_normal() &&
            std::filesystem::u8path(custom.rviz.rviz_executable) == (temp / "tools/rviz '$()").lexically_normal(), "executable paths must resolve relative to the JSON file");
    require(!streamEnabled(custom, "input_rgb") && !streamEnabled(custom, "clouds/camera"),
            "stream publication must require both publish and display switches");
    require(displaySettings(custom, "slip").frame_id == "custom_camera" &&
            displaySettings(custom, "clouds/robot").frame_id == "custom_body" &&
            displaySettings(custom, "input_rgb").frame_id == "camera_optical_frame",
            "fixed_frame must not change expected source camera frames");
    const auto custom_view = rvizConfig(custom);
    require(custom_view.find("Value: \"/custom/slip\"") != std::string::npos &&
            custom_view.find("Reliability Policy: Reliable") != std::string::npos &&
            custom_view.find("Durability Policy: Transient Local") != std::string::npos,
            "RViz topics and QoS must follow custom JSON");
    require(custom_view.find("Min Value: 1024") != std::string::npos &&
            custom_view.find("Max Value: 18432") != std::string::npos,
            "raw Q8 Image display range must follow configured pixel disparity limits times 256");
#ifndef _WIN32
    if (argc == 3) {
      std::error_code error;
      auto self = std::filesystem::read_symlink("/proc/self/exe", error);
      if (error) self = std::filesystem::absolute(argv[0]);
      launcherContracts(temp, std::filesystem::absolute(argv[2]), self);
    }
#endif
    temp = checkedTemp(temp, base);
    std::filesystem::remove_all(temp);
    std::cout << "RViz settings/launcher contracts passed: " << checks << " checks\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RViz contracts: " << error.what() << '\n';
    if (!temp.empty()) std::cerr << "Preserved fixtures: " << temp.u8string() << '\n';
    return 1;
  }
}
