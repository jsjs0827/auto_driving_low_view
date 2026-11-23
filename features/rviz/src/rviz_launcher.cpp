#include "rviz_settings.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#ifndef _WIN32
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {
using namespace perception::rviz;
using Path = std::filesystem::path;
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int value) { interrupted = value; }
struct Arguments {
  Path config = "rviz.json";
  Path run_dir;
  bool prepare_only = false;
  bool no_gui = false;
};
void usage() {
  std::cout << "r36_rviz_launcher --config RVIZ_JSON --run-dir NEW_DIRECTORY [--prepare-only] [--no-gui]\n"
               "X5 publication must be started separately. This launcher only forwards ADB, starts\n"
               "the local ROS receiver, and opens RViz. --prepare-only never invokes ADB or ROS.\n";
}
Arguments arguments(int argc, char** argv) {
  Arguments result;
  std::set<std::string> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string option(argv[i]);
    if (!seen.insert(option).second) throw std::invalid_argument("Duplicate option: " + option);
    if (option == "--help" || option == "-h") { usage(); std::exit(0); }
    if (option == "--prepare-only") result.prepare_only = true;
    else if (option == "--no-gui") result.no_gui = true;
    else if ((option == "--config" || option == "--run-dir") && i + 1 < argc) {
      auto value = std::filesystem::u8path(argv[++i]);
      if (value.empty()) throw std::invalid_argument("Empty argument: " + option);
      if (option == "--config") result.config = value;
      else result.run_dir = value;
    } else throw std::invalid_argument("Unknown or incomplete option: " + option);
  }
  if (result.run_dir.empty()) throw std::invalid_argument("--run-dir must name a new output directory");
  result.config = std::filesystem::absolute(result.config).lexically_normal();
  result.run_dir = std::filesystem::absolute(result.run_dir).lexically_normal();
  return result;
}
void outputFile(const Path& path, const std::string& contents) {
  if (std::filesystem::exists(path)) throw std::runtime_error("Refusing to overwrite: " + path.u8string());
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Cannot write: " + path.u8string());
  output << contents;
  if (!output) throw std::runtime_error("Cannot finish: " + path.u8string());
}
#ifndef _WIN32
struct CommandResult { int status = -1; std::string output; };
int statusCode(int status) {
  if (WIFEXITED(status)) return WEXITSTATUS(status);
  if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
  return 1;
}
std::vector<char*> commandArguments(const std::vector<std::string>& arguments) {
  std::vector<char*> values;
  for (const auto& argument : arguments) values.push_back(const_cast<char*>(argument.c_str()));
  values.push_back(nullptr);
  return values;
}
void closeFd(int& fd) { if (fd >= 0) { ::close(fd); fd = -1; } }
void terminateChild(pid_t& child) {
  if (child <= 0) return;
  int status = 0;
  if (::waitpid(child, &status, WNOHANG) == 0) {
    ::kill(child, SIGTERM);
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < until) {
      const auto waited = ::waitpid(child, &status, WNOHANG);
      if (waited == child || (waited < 0 && errno == ECHILD)) { child = -1; return; }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ::kill(child, SIGKILL);
    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
  }
  child = -1;
}
pid_t spawn(const std::vector<std::string>& args, int stdout_fd = -1) {
  if (args.empty()) throw std::invalid_argument("Empty subprocess arguments");
  int report[2] = {-1, -1};
  if (::pipe2(report, O_CLOEXEC) < 0) throw std::runtime_error("Cannot create exec report pipe");
  auto values = commandArguments(args);
  const auto child = ::fork();
  if (child == 0) {
    ::close(report[0]);
    if (stdout_fd >= 0 && ::dup2(stdout_fd, STDOUT_FILENO) < 0) {
      const int failure = errno;
      (void)::write(report[1], &failure, sizeof(failure));
      _exit(127);
    }
    if (stdout_fd > STDERR_FILENO) ::close(stdout_fd);
    ::execvp(values[0], values.data());
    const int failure = errno;
    (void)::write(report[1], &failure, sizeof(failure));
    _exit(127);
  }
  closeFd(report[1]);
  if (child < 0) { closeFd(report[0]); throw std::runtime_error("Cannot fork subprocess"); }
  int failure = 0;
  ssize_t count;
  do { count = ::read(report[0], &failure, sizeof(failure)); } while (count < 0 && errno == EINTR);
  closeFd(report[0]);
  if (count > 0) {
    int status;
    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
    throw std::runtime_error("Cannot execute " + args[0] + ": " + std::strerror(failure));
  }
  return child;
}
CommandResult command(const std::vector<std::string>& args, bool honor_interrupt = true) {
  int capture[2] = {-1, -1};
  if (::pipe2(capture, O_CLOEXEC) < 0) throw std::runtime_error("Cannot create command output pipe");
  if (::fcntl(capture[0], F_SETFL, O_NONBLOCK) < 0) {
    closeFd(capture[0]); closeFd(capture[1]);
    throw std::runtime_error("Cannot configure command output pipe");
  }
  pid_t child = -1;
  try { child = spawn(args, capture[1]); }
  catch (...) { closeFd(capture[0]); closeFd(capture[1]); throw; }
  closeFd(capture[1]);
  CommandResult result;
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  int status = 0;
  bool finished = false;
  try {
    while (true) {
      char buffer[4096];
      const auto bytes = ::read(capture[0], buffer, sizeof(buffer));
      if (bytes > 0) {
        if (result.output.size() + static_cast<std::size_t>(bytes) > 1024 * 1024)
          throw std::runtime_error("ADB command output exceeds 1 MiB");
        result.output.append(buffer, static_cast<std::size_t>(bytes));
      }
      if (!finished) {
        const auto waited = ::waitpid(child, &status, WNOHANG);
        if (waited == child) { finished = true; child = -1; result.status = statusCode(status); }
        else if (waited < 0 && errno != EINTR) throw std::runtime_error("Cannot wait for ADB command");
      }
      if (finished && bytes <= 0) break;
      if ((honor_interrupt && interrupted) || std::chrono::steady_clock::now() > until)
        throw std::runtime_error(interrupted ? "Interrupted during ADB command" : "ADB command timed out after 15 seconds");
      if (bytes <= 0) {
        pollfd poll_file{capture[0], POLLIN, 0};
        (void)::poll(&poll_file, 1, 20);
      }
    }
    closeFd(capture[0]);
    return result;
  } catch (...) { terminateChild(child); closeFd(capture[0]); throw; }
}
std::string selectDevice(const RvizSettings& settings) {
  const auto found = command({settings.transport.adb_executable, "devices", "-l"});
  if (found.status != 0) throw std::runtime_error("adb devices failed with exit code " + std::to_string(found.status));
  std::vector<std::string> online;
  std::map<std::string, std::string> states;
  std::istringstream lines(found.output);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream fields(line);
    std::string serial, state;
    if (!(fields >> serial >> state) || serial == "List" || serial[0] == '*') continue;
    states[serial] = state;
    if (state == "device") online.push_back(serial);
  }
  if (!settings.transport.serial.empty()) {
    const auto state = states.find(settings.transport.serial);
    if (state == states.end() || state->second != "device")
      throw std::runtime_error("Configured ADB device is not online/authorized: " + settings.transport.serial);
    return settings.transport.serial;
  }
  if (online.empty()) throw std::runtime_error("No online ADB device. Connect and authorize X5 before launching RViz.");
  if (online.size() != 1) throw std::runtime_error("Multiple online ADB devices: set transport.serial in rviz.json.");
  return online.front();
}
std::vector<std::string> adb(const RvizSettings& settings, const std::string& serial, std::initializer_list<std::string> arguments) {
  std::vector<std::string> result{settings.transport.adb_executable, "-s", serial};
  result.insert(result.end(), arguments.begin(), arguments.end());
  return result;
}
struct Forward {
  RvizSettings settings;
  std::string serial;
  bool owned = false;
  std::string local() const { return "tcp:" + std::to_string(settings.transport.local_port); }
  std::string remote() const { return "tcp:" + std::to_string(settings.transport.remote_port); }
  void start() {
    // Check all serials, since the host TCP port belongs to the entire ADB server.
    const auto existing = command(adb(settings, serial, {"forward", "--list"}));
    if (existing.status != 0) throw std::runtime_error("Cannot inspect existing ADB forwards");
    std::istringstream lines(existing.output);
    std::string line;
    while (std::getline(lines, line)) {
      std::istringstream fields(line);
      std::string device, source, destination;
      if ((fields >> device >> source >> destination) && source == local())
        throw std::runtime_error("ADB local port already forwarded; choose another transport.local_port: " + local());
    }
    const auto created = command(adb(settings, serial, {"forward", "--no-rebind", local(), remote()}));
    if (created.status != 0) throw std::runtime_error("Cannot create ADB forward without replacing existing mappings");
    owned = true;
  }
  void cleanup() noexcept {
    if (!owned) return;
    owned = false;
    try {
      const auto listing = command(adb(settings, serial, {"forward", "--list"}), false);
      if (listing.status != 0) return;
      std::istringstream lines(listing.output);
      std::string line;
      while (std::getline(lines, line)) {
        std::istringstream fields(line);
        std::string device, source, destination;
        if ((fields >> device >> source >> destination) && device == serial && source == local() && destination == remote()) {
          const auto removed = command(adb(settings, serial, {"forward", "--remove", local()}), false);
          if (removed.status != 0) std::cerr << "RViz cleanup: could not remove owned ADB forward " << local() << '\n';
          return;
        }
      }
    } catch (const std::exception& error) { std::cerr << "RViz cleanup: " << error.what() << '\n'; }
  }
  ~Forward() { cleanup(); }
};
struct Children {
  pid_t receiver = -1;
  pid_t rviz = -1;
  ~Children() { terminateChild(rviz); terminateChild(receiver); }
};
Path receiverExecutable(const char* launcher) {
  std::error_code error;
  auto self = std::filesystem::read_symlink("/proc/self/exe", error);
  if (error) self = std::filesystem::absolute(std::filesystem::u8path(launcher));
  const auto receiver = self.parent_path() / "r36_rviz_receiver";
  if (!std::filesystem::is_regular_file(receiver) || ::access(receiver.c_str(), X_OK) != 0)
    throw std::runtime_error("ROS receiver must be installed beside the launcher: " + receiver.u8string());
  return receiver;
}
int run(const Arguments& args, const RvizSettings& settings, const char* launcher) {
  const auto receiver = receiverExecutable(launcher);
  Forward forward{settings, selectDevice(settings), false};
  forward.start();
  std::cout << "ADB " << forward.serial << ": 127.0.0.1:" << settings.transport.local_port
            << " -> X5:" << settings.transport.remote_port << '\n' << std::flush;
  Children children;
  children.receiver = spawn({receiver.u8string(), "--config", (args.run_dir / "rviz.resolved.json").u8string(),
                             "--host", "127.0.0.1", "--port", std::to_string(settings.transport.local_port)});
  if (!args.no_gui)
    children.rviz = spawn({settings.rviz.rviz_executable, "--display-config", (args.run_dir / "view.rviz").u8string()});
  while (!interrupted) {
    for (auto* child : {&children.receiver, &children.rviz}) {
      if (*child <= 0) continue;
      int status = 0;
      const auto waited = ::waitpid(*child, &status, WNOHANG);
      if (waited == *child) {
        const bool receiver_exited = child == &children.receiver;
        *child = -1;
        const int code = statusCode(status);
        if (receiver_exited) std::cerr << "RViz receiver exited with code " << code << '\n';
        return code;
      }
      if (waited < 0 && errno != EINTR) throw std::runtime_error("Cannot wait for RViz child");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return 128 + interrupted;
}
#endif
} // namespace

int main(int argc, char** argv) {
  try {
    const auto args = arguments(argc, argv);
    const auto settings = loadSettings(args.config);
    if (!settings.enabled && !args.prepare_only) {
      std::cout << "RViz is disabled by the configuration. No ADB or ROS processes started.\n";
      return 0;
    }
    std::filesystem::create_directories(args.run_dir.parent_path());
    if (!std::filesystem::create_directory(args.run_dir))
      throw std::runtime_error("--run-dir must not exist: " + args.run_dir.u8string());
    outputFile(args.run_dir / "rviz.resolved.json", settingsJson(settings));
    outputFile(args.run_dir / "view.rviz", rvizConfig(settings));
    std::cout << "RViz configuration: " << (args.run_dir / "view.rviz").u8string() << '\n';
    if (args.prepare_only) return 0;
#ifdef _WIN32
    throw std::runtime_error("ADB + ROS RViz execution requires Ubuntu/Linux. Windows supports --prepare-only.");
#else
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    return run(args, settings, argv[0]);
#endif
  } catch (const std::exception& error) {
    std::cerr << "r36_rviz_launcher: " << error.what() << '\n';
    return 1;
  }
}
