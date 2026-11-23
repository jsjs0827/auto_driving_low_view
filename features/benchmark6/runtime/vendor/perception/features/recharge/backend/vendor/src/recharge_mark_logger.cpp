#include "recharge_mark_logger.h"

#include <condition_variable>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <mutex>
#include <string>
#include <cstdarg>
#include <deque>
#include <thread>

#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace recharge_log {
namespace {

constexpr const char* kDefaultDir = "/userdata/log/ai_node";
constexpr const char* kFileName = "recharge_mark.log";
constexpr std::size_t kDefaultMaxSize = 10u * 1024u * 1024u;  // 10MB，仿 ai_node
constexpr std::size_t kDefaultMaxFiles = 5;
constexpr std::size_t kQueueCap = 1024;

struct Record {
    std::string line;
};

struct LogState {
    std::mutex mtx;
    std::condition_variable cv;
    std::deque<Record> queue;
    std::thread worker;
    bool stop = false;
    bool started = false;

    std::string dir = kDefaultDir;
    std::string path;  // dir + "/" + kFileName
    std::size_t max_size = kDefaultMaxSize;
    std::size_t max_files = kDefaultMaxFiles;

    FILE* fp = nullptr;
    std::size_t cur_size = 0;
    bool file_ok = false;  // false 时退化为 stderr
};

LogState& State() {
    static LogState s;
    return s;
}

#if defined(_WIN32)
bool MkDirRecursive(const std::string& path) {
    std::error_code error;
    if (std::filesystem::is_directory(path,error)) return true;
    error.clear();return std::filesystem::create_directories(path,error) && !error;
}
#else
bool MkDirRecursive(const std::string& path) {
    if (path.empty()) return false;
    std::string cur;
    cur.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        cur.push_back(path[i]);
        if (path[i] == '/' || i + 1 == path.size()) {
            while (cur.size() > 1 && cur.back() == '/') cur.pop_back();
            if (cur.empty() || cur == "/") continue;
            struct stat st {};
            if (::stat(cur.c_str(), &st) == 0) {
                if (!S_ISDIR(st.st_mode)) return false;
                continue;
            }
            if (::mkdir(cur.c_str(), 0755) != 0 &&
                (::stat(cur.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))) {
                return false;
            }
        }
    }
    return true;
}

#endif

std::string NowStamp() {
    timespec ts{};
#if defined(_WIN32)
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    ts.tv_sec=static_cast<time_t>(ns/1000000000LL);ts.tv_nsec=static_cast<long>(ns%1000000000LL);
#else
    clock_gettime(CLOCK_REALTIME, &ts);
#endif
    std::tm tm_buf {};
#if defined(_WIN32)
    localtime_s(&tm_buf,&ts.tv_sec);
#else
    localtime_r(&ts.tv_sec, &tm_buf);
#endif
    char buf[64];
    const long ms = ts.tv_nsec / 1000000L;
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, ms);
    return buf;
}

void RotateIfNeeded(LogState& s) {
    if (!s.file_ok || !s.fp || s.cur_size < s.max_size) return;
    std::fclose(s.fp);
    s.fp = nullptr;
    // recharge_mark.log.4 <- .3 <- .2 <- .1 <- .log，最旧(.max)直接删除
    for (std::size_t i = s.max_files; i >= 2; --i) {
        const std::string from =
            s.path + "." + std::to_string(i - 1);
        const std::string to = s.path + "." + std::to_string(i);
        ::remove(to.c_str());
        ::rename(from.c_str(), to.c_str());
    }
    ::remove((s.path + ".1").c_str());
    ::rename(s.path.c_str(), (s.path + ".1").c_str());
    s.fp = std::fopen(s.path.c_str(), "ab");
    s.file_ok = (s.fp != nullptr);
    s.cur_size = 0;
}

bool OpenFile(LogState& s) {
    s.fp = std::fopen(s.path.c_str(), "ab");
    s.file_ok = (s.fp != nullptr);
    s.cur_size = 0;
    if (!s.file_ok) {
        std::fprintf(stderr,
                     "[recharge_mark] log file open failed: %s, fallback stderr\n",
                     s.path.c_str());
    }
    return s.file_ok;
}

void WorkerLoop() {
    LogState& s = State();
    for (;;) {
        std::deque<Record> batch;
        {
            std::unique_lock<std::mutex> lk(s.mtx);
            s.cv.wait(lk, [&] { return s.stop || !s.queue.empty(); });
            batch.swap(s.queue);
            if (s.stop && batch.empty()) break;
        }
        for (auto& r : batch) {
            if (s.file_ok && s.fp) {
                const int n = std::fwrite(r.line.data(), 1, r.line.size(), s.fp);
                if (n > 0) s.cur_size += static_cast<std::size_t>(n);
                std::fflush(s.fp);
                RotateIfNeeded(s);
            } else {
                std::fwrite(r.line.data(), 1, r.line.size(), stderr);
            }
        }
    }
    if (s.fp) {
        std::fflush(s.fp);
        std::fclose(s.fp);
        s.fp = nullptr;
    }
}

}  // namespace

bool Init(const char* dir, std::size_t max_file_size, std::size_t max_files) {
    LogState& s = State();
    std::lock_guard<std::mutex> lk(s.mtx);
    if (s.started) return true;
    if (dir && dir[0]) s.dir = dir;
    if (max_file_size > 0) s.max_size = max_file_size;
    if (max_files > 0) s.max_files = max_files;
    while (!s.dir.empty() && s.dir.back() == '/') s.dir.pop_back();
    s.path = s.dir + "/" + kFileName;
    s.stop = false;
    MkDirRecursive(s.dir);
    OpenFile(s);
    s.worker = std::thread(WorkerLoop);
    s.started = true;
    return true;
}

void Shutdown() {
    LogState& s = State();
    {
        std::lock_guard<std::mutex> lk(s.mtx);
        if (!s.started) return;
        s.stop = true;
    }
    s.cv.notify_all();
    if (s.worker.joinable()) s.worker.join();
    std::lock_guard<std::mutex> lk(s.mtx);
    s.started = false;
    if (s.fp) {
        std::fflush(s.fp);
        std::fclose(s.fp);
        s.fp = nullptr;
    }
    s.file_ok = false;
}

void Write(const char* level, const char* fmt, ...) {
    if (!fmt) return;
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    std::string body = (static_cast<size_t>(n) < sizeof(buf))
                           ? std::string(buf, static_cast<size_t>(n))
                           : std::string(buf);

    LogState& s = State();
    Record rec;
    rec.line = "[" + NowStamp() + "][" + (level ? level : "info") + "] " +
               body + "\n";
    {
        std::lock_guard<std::mutex> lk(s.mtx);
        if (!s.started) {
            std::fwrite(rec.line.data(), 1, rec.line.size(), stderr);
            return;
        }
        if (s.queue.size() >= kQueueCap) s.queue.pop_front();  // 丢最旧保最新
        s.queue.push_back(std::move(rec));
    }
    s.cv.notify_one();
}

}  // namespace recharge_log
