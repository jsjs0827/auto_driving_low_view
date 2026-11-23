#include "product/ros_io_identity.hpp"
#include "runtime_status.hpp"
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <system_error>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#include <unistd.h>
#endif

namespace {
constexpr const char* kBuildId = "r12-v31-original-ros-io-v1";
// An internal object identifies this module even if an external symbol with
// the same name as our exported build-id function is interposed by a loader.
const unsigned char kModuleAnchor = 0;
std::atomic<unsigned long long> temporary_sequence{0};

std::string jsonString(const std::string& value) {
    constexpr char hex[] = "0123456789abcdef";
    std::string result = "\"";
    for (unsigned char byte : value) {
        if (byte == '"' || byte == '\\') { result += '\\'; result += char(byte); }
        else if (byte < 0x20) {
            result += "\\u00"; result += hex[byte >> 4]; result += hex[byte & 15];
        } else result += char(byte);
    }
    result += '"';
    return result;
}

std::filesystem::path modulePath() {
#ifdef _WIN32
    HMODULE module = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                              GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&kModuleAnchor), &module))
        throw std::system_error(GetLastError(), std::system_category(), "locate loaded product module");
    std::wstring path(32768, L'\0');
    const DWORD count = GetModuleFileNameW(module, path.data(), static_cast<DWORD>(path.size()));
    if (!count || count >= path.size())
        throw std::system_error(count ? ERROR_INSUFFICIENT_BUFFER : GetLastError(),
                                std::system_category(), "read loaded product module path");
    path.resize(count);
    return std::filesystem::canonical(path);
#else
    Dl_info info{};
    if (!dladdr(&kModuleAnchor, &info) || !info.dli_fname || !*info.dli_fname)
        throw std::runtime_error("dladdr could not identify the loaded product module");
    return std::filesystem::canonical(info.dli_fname);
#endif
}

std::uint64_t processId() noexcept {
#ifdef _WIN32
    return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
    return static_cast<std::uint64_t>(getpid());
#endif
}

std::string receiptJson() {
    return "{\"receipt_version\":1,\"state\":\"loaded\",\"build_id\":" + jsonString(kBuildId) +
        ",\"actual_plugin_path\":" + jsonString(modulePath().u8string()) +
        ",\"pid\":" + std::to_string(processId()) +
        ",\"framework\":\"original-dag-task-feature\",\"input_transport\":\"original-ros-shm\","
        "\"input_contract\":\"original-map-rgbd-left-640x544-nv12\","
        "\"camera_owner\":\"original-camera-node\",\"single_frame\":true,\"double_buffer\":false,"
        "\"frame_processing_verified\":false}\n";
}
}

extern "C" const char* perception_product_build_id() noexcept { return kBuildId; }

namespace r31::product {
void writeRosIoReceipt(const std::string& destination) {
    std::filesystem::path temporary;
    try {
        if (destination.empty()) throw std::invalid_argument("ROS I/O receipt path is empty");
        const auto target = std::filesystem::absolute(std::filesystem::u8path(destination));
        const auto json = receiptJson();
        temporary = target;
        temporary += ".tmp-" + std::to_string(processId()) + "-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
            std::to_string(++temporary_sequence);
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) throw std::system_error(errno ? errno : EIO, std::generic_category(), "open ROS I/O receipt temporary file");
        output.write(json.data(), static_cast<std::streamsize>(json.size()));
        output.flush();
        if (!output) throw std::system_error(errno ? errno : EIO, std::generic_category(), "write ROS I/O receipt");
        output.close();
        if (!output) throw std::system_error(errno ? errno : EIO, std::generic_category(), "close ROS I/O receipt");
#ifdef _WIN32
        if (!MoveFileExW(temporary.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            throw std::system_error(GetLastError(), std::system_category(), "replace ROS I/O receipt atomically");
#else
        std::filesystem::rename(temporary, target);
#endif
    } catch (const std::exception& error) {
        if (!temporary.empty()) { std::error_code ignored; std::filesystem::remove(temporary, ignored); }
        throw r28::Failure(r28::statusFromException(error, 1004, "product.ros_io.receipt"));
    }
}

void writeRosIoReceiptFromEnvironment() {
    const char* destination = std::getenv("PERCEPTION_ROS_IO_RECEIPT");
    if (destination && *destination) { writeRosIoReceipt(destination); return; }
    static std::once_flag marker;
    std::call_once(marker, [] {
        std::fprintf(stderr, "[perception-product] build_id=%s state=loaded receipt=disabled frame_processing_verified=false\n", kBuildId);
    });
}
}
