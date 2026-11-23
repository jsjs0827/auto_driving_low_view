#include "perception/logging.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <thread>

namespace perception {
namespace {
std::string clipped(const std::string &s, std::size_t limit) {
    if (s.size() <= limit)
        return s;
    while (limit && (static_cast<unsigned char>(s[limit]) & 0xc0) == 0x80)
        --limit;
    return s.substr(0, limit) + "...";
}
std::string quote(const std::string &s) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\')
            out << '\\' << c;
        else if (c < 32)
            out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << unsigned(c)
                << std::dec;
        else
            out << c;
    }
    out << '"';
    return out.str();
}
std::string number(double value) {
    if (!std::isfinite(value))
        return "null";
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(12) << value;
    return out.str();
}
} // namespace
const char *logLevelName(LogLevel level) noexcept {
    switch (level) {
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warning:
        return "WARNING";
    case LogLevel::Error:
        return "ERROR";
    }
    return "UNKNOWN";
}
struct Logger::Impl {
    LogConfig cfg;
    std::atomic<bool> active{false};
    mutable std::mutex mutex;
    std::ofstream file;
    std::size_t bytes = 0;
    std::uint64_t written = 0, failures = 0;
    std::string last_error;
    explicit Impl(LogConfig config) : cfg(std::move(config)) {
        if (cfg.min_level < LogLevel::Debug || cfg.min_level > LogLevel::Error ||
            cfg.max_file_bytes < 16384 || cfg.max_file_bytes > 1024U * 1024U * 1024U ||
            cfg.max_files < 1 || cfg.max_files > 32 || (cfg.file_path.empty() && !cfg.console))
            throw PerceptionException(ErrorCode::PerceptionInitFailed,
                                      "invalid logging configuration", "logger.init");
    }
    void open() {
        if (cfg.file_path.empty() || file.is_open())
            return;
        const auto path = std::filesystem::u8path(cfg.file_path);
        if (!path.parent_path().empty())
            std::filesystem::create_directories(path.parent_path());
        bytes = std::filesystem::exists(path) ? std::filesystem::file_size(path) : 0;
        file.clear();
        file.open(path, std::ios::binary | std::ios::app);
        if (!file)
            throw std::runtime_error("cannot open log file: " + cfg.file_path);
    }
    void rotate() {
        file.flush();
        if (!file)
            throw std::runtime_error("log flush failed before rotation");
        file.close();
        const auto base = std::filesystem::u8path(cfg.file_path);
        if (cfg.max_files == 1)
            std::filesystem::remove(base);
        for (unsigned i = cfg.max_files - 1; i > 0; --i) {
            const auto source =
                i == 1 ? base
                       : std::filesystem::u8path(cfg.file_path + "." + std::to_string(i - 1));
            const auto target = std::filesystem::u8path(cfg.file_path + "." + std::to_string(i));
            std::filesystem::remove(target);
            if (std::filesystem::exists(source))
                std::filesystem::rename(source, target);
        }
        open();
    }
    void failed(const char *why) noexcept {
        ++failures;
        try {
            last_error = clipped(why ? why : "unknown log I/O failure", 512);
        } catch (...) {
        }
        // Recoverable sink failure: retry reopening on the next enabled record.
        try {
            if (file.is_open())
                file.close();
            file.clear();
        } catch (...) {
        }
    }
};
Logger::Logger(LogConfig config) : impl_(new Impl(std::move(config))) {
    if (impl_->cfg.enabled && !setEnabled(true))
        throw PerceptionException(ErrorCode::PerceptionInitFailed, stats().last_io_error,
                                  "logger.init");
}
Logger::~Logger() = default;
bool Logger::enabled(LogLevel level) const noexcept {
    return impl_->active.load(std::memory_order_relaxed) && level >= impl_->cfg.min_level;
}
bool Logger::setEnabled(bool value) noexcept {
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->active = false;
        try {
            if (value)
                impl_->open();
            else if (impl_->file.is_open()) {
                impl_->file.flush();
                if (!impl_->file)
                    throw std::runtime_error("log flush failed while disabling");
                impl_->file.close();
            }
            impl_->active = value;
            return true;
        } catch (const std::exception &e) {
            impl_->failed(e.what());
        } catch (...) {
            impl_->failed("unknown log enable/disable failure");
        }
    } catch (...) {
    }
    return false;
}
std::uint64_t Logger::progressEveryFrames() const noexcept {
    return impl_->cfg.progress_every_frames;
}
void Logger::write(LogLevel level, const char *event, const char *component,
                   const std::string &detail, FrameInfo frame, double elapsed,
                   const ErrorInfo *error) noexcept {
    if (!enabled(level))
        return;
    try {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!enabled(level))
            return;
        try {
            const auto wall = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
            const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
            const auto code = error ? error->code : ErrorCode::None;
            std::ostringstream thread;
            thread << std::this_thread::get_id();
            std::ostringstream out;
            out.imbue(std::locale::classic());
            out << "{\"wall_time_ms\":" << wall << ",\"steady_time_ns\":" << steady
                << ",\"thread\":" << quote(thread.str())
                << ",\"level\":" << quote(logLevelName(level))
                << ",\"event\":" << quote(clipped(event ? event : "", 96))
                << ",\"component\":" << quote(clipped(component ? component : "", 96))
                << ",\"frame_id\":" << frame.id
                << ",\"capture_timestamp_ns\":" << frame.timestamp_ns
                << ",\"elapsed_ms\":" << number(elapsed)
                << ",\"code\":" << static_cast<std::uint32_t>(code)
                << ",\"code_name\":" << quote(errorCodeName(code))
                << ",\"message\":" << quote(errorCodeMessage(code)) << ",\"context_codes\":[";
            if (error)
                for (std::size_t i = 0; i < error->context_codes.size() && i < 16; ++i) {
                    if (i)
                        out << ',';
                    out << static_cast<std::uint32_t>(error->context_codes[i]);
                }
            out << "],\"detail\":" << quote(clipped(detail, 2048))
                << ",\"detail_truncated\":" << (detail.size() > 2048 ? "true" : "false") << "}\n";
            const auto record = out.str();
            if (!impl_->cfg.file_path.empty()) {
                impl_->open();
                if (impl_->bytes && impl_->bytes + record.size() > impl_->cfg.max_file_bytes)
                    impl_->rotate();
                impl_->file.write(record.data(), static_cast<std::streamsize>(record.size()));
                // Important nodes and errors survive a subsequent process/driver failure.
                if (level >= LogLevel::Info)
                    impl_->file.flush();
                if (!impl_->file)
                    throw std::runtime_error("log write/flush failed");
                impl_->bytes += record.size();
            }
            if (impl_->cfg.console) {
                if (std::fwrite(record.data(), 1, record.size(), stderr) != record.size() ||
                    std::fflush(stderr) != 0)
                    throw std::runtime_error("log console write failed");
            }
            ++impl_->written;
        } catch (const std::exception &e) {
            impl_->failed(e.what());
        } catch (...) {
            impl_->failed("unknown log serialization/write failure");
        }
    } catch (...) {
    } // Never turn logging failure into an inference failure.
}
void Logger::error(const ErrorInfo &error) noexcept {
    if (!error.ok())
        write(LogLevel::Error, "error", error.component.c_str(), error.detail, error.frame,
              error.elapsed_ms, &error);
}
LogStats Logger::stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return {impl_->active.load(), impl_->written, impl_->failures, impl_->last_error};
}
ErrorInfo reportError(const std::shared_ptr<Logger> &logger, ErrorCode code, std::string detail,
                      const char *component, FrameInfo frame, double elapsed) {
    if (!isKnownErrorCode(code) || code == ErrorCode::None)
        throw std::invalid_argument("reportError requires one of the defined nonzero error codes");
    ErrorInfo error{code,  {},     component ? component : "application", std::move(detail),
                    frame, elapsed};
    if (logger)
        logger->error(error);
    return error;
}
ErrorInfo checkProcessingTime(const std::shared_ptr<Logger> &logger, ErrorCode code, double elapsed,
                              double limit, const char *component, FrameInfo frame) {
    if ((code != ErrorCode::MapProcessingTooSlow) ||
        !std::isfinite(elapsed) || !std::isfinite(limit) || elapsed < 0 || limit < 0)
        throw std::invalid_argument("invalid processing-time measurement/code/limit");
    if (limit == 0 || elapsed <= limit)
        return {};
    return reportError(logger, code, "completed operation exceeded limit_ms=" + number(limit),
                       component, frame, elapsed);
}
} // namespace perception
