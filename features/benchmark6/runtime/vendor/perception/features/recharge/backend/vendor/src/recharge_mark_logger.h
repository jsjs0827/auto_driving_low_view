/*
 * @Description: 回充二维码独立文件日志（so 内自管，仿 ai_node spdlog 写入）
 *
 * - 文件固定 /userdata/log/ai_node/recharge_mark.log（可用 Init 参数改目录）
 * - 大小轮转：默认 10MB/个 × 5 个（recharge_mark.log → .1 ... .5），与
 *   ai_node 的 spdlog rotating sink 行为一致
 * - 行格式仿 ai_node pattern "[%Y-%m-%d %H:%M:%S.%e][%l] %v"
 * - 后台线程异步落盘；队列满丢弃最旧，保证最新二维码结果可见
 * - 目录不可创建/文件打不开（离线 x86 等）时退化为 stderr 输出，不阻塞业务
 *
 * 仅本仓 .cpp 内部使用，不进对外头。
 */
#ifndef RECHARGE_MARK_LOGGER_H_
#define RECHARGE_MARK_LOGGER_H_

#include <cstddef>

namespace recharge_log {

// 初始化（进程内幂等，已初始化直接返回 true）。
// dir 为空用默认 /userdata/log/ai_node；max_file_size/max_files 为 0 取默认。
bool Init(const char* dir = nullptr, std::size_t max_file_size = 0,
          std::size_t max_files = 0);

// 停止后台线程并落盘关闭。宿主 Destroy 时调用；之后可重新 Init。
void Shutdown();

// printf 风格写入。未初始化/打开失败时同步写 stderr。
void Write(const char* level, const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

}  // namespace recharge_log

#define RM_LOG_INFO(...) recharge_log::Write("info", __VA_ARGS__)
#define RM_LOG_WARN(...) recharge_log::Write("warn", __VA_ARGS__)
#define RM_LOG_ERROR(...) recharge_log::Write("error", __VA_ARGS__)

#endif  // RECHARGE_MARK_LOGGER_H_
