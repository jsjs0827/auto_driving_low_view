#pragma once
#include "perception/error.hpp"
#include <memory>

namespace perception {
enum class LogLevel { Debug, Info, Warning, Error };
const char *logLevelName(LogLevel) noexcept;
struct LogConfig {
    bool enabled = false, console = false;
    LogLevel min_level = LogLevel::Info;
    std::string file_path =
        "logs/perception.jsonl"; // Empty => console only (console must be true).
    std::size_t max_file_bytes = 16U * 1024U * 1024U;
    unsigned max_files = 3;                    // Total: current + .1 ... .(max_files-1).
    std::uint64_t progress_every_frames = 100; // 0 disables periodic Info progress.
};
struct LogStats {
    bool enabled = false;
    std::uint64_t written = 0, io_failures = 0;
    std::string last_io_error;
};
// One logger per file path, shared by all modules. Synchronous and bounded, no worker queue.
// write() never throws; file failures are visible through stats(), not inference status.
class Logger {
  public:
    explicit Logger(LogConfig = {});
    ~Logger();
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;
    bool enabled(LogLevel = LogLevel::Info) const noexcept;
    bool setEnabled(bool) noexcept; // False on open/flush failure. False -> no logging I/O.
    std::uint64_t progressEveryFrames() const noexcept;
    void write(LogLevel, const char *event, const char *component, const std::string &detail = {},
               FrameInfo = {}, double elapsed_ms = 0, const ErrorInfo *error = nullptr) noexcept;
    void error(const ErrorInfo &) noexcept;
    LogStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
// Upstream camera/map/health reporting works even when engine initialization failed.
// Returns an owning fault for the application's message/status channel; no sticky global state.
ErrorInfo reportError(const std::shared_ptr<Logger> &, ErrorCode, std::string detail,
                      const char *component, FrameInfo = {}, double elapsed_ms = 0);
// Measures a COMPLETED upstream operation. 0 limit disables the gate; this is not cancellation.
ErrorInfo checkProcessingTime(const std::shared_ptr<Logger> &, ErrorCode timing_code,
                              double elapsed_ms, double limit_ms, const char *component,
                              FrameInfo = {});
} // namespace perception
