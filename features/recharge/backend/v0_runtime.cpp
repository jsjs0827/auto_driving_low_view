#include "v0_runtime.hpp"
#include "v0_sha256.hpp"
#include <zlib.h>
#include <dlfcn.h>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace {
constexpr const char* kOriginalSha =
    "8e458ce9a48aa8555c2452e09b3edc57baa27fbe44a6db77ddbfc8fc751e9f0b";
constexpr const char* kHeaderSha =
    "bd3cbfa3df3883b59a9039579c137c8f885f53f53232d3b16fb8c0c8018cda3e";
constexpr const char* kFactory = "_ZN14Aiper_Recharge21createRechargeMarkApiEv";
using Api = Aiper_Recharge::RechargeMarkApi;
struct Lease { Api* api = nullptr; };

struct Runtime {
    // Intentional process-lifetime handles. The original SDK owns global
    // logger/save threads and globals; do not dlclose it while exit handlers
    // might still refer to its code. Every API is nevertheless destroyed.
    void* original = nullptr;
    void* zlib = nullptr;
    Api* (*factory)() = nullptr;
    std::string path;
    std::string zlib_path;
    std::string identity;
    std::unordered_set<Lease*> leases;
    std::mutex mutex;
};
Runtime& runtime() {
    static Runtime* value = new Runtime;
    return *value;
}

void load(Runtime& state) {
    if (state.original && state.factory) return;
    Dl_info self{};
    if (!dladdr(reinterpret_cast<void*>(&r36_recharge_v0_open), &self) || !self.dli_fname)
        throw std::runtime_error("V0 shim cannot resolve its own runtime directory");
    const auto directory = std::filesystem::canonical(self.dli_fname).parent_path();
    const auto original = directory / "librecharge_mark.so";
    if (!std::filesystem::is_regular_file(original))
        throw std::runtime_error("Missing selected V0 SDK beside private shim: " + original.string());
    if (r36_recharge_integrity::fileSha256(original.string()) != kOriginalSha)
        throw std::runtime_error("V0 SDK SHA mismatch beside private shim; refusing any fallback: " + original.string());

    // Real, direct zlib reference keeps DT_NEEDED libz.so.1 even with as-needed.
    // V0 omitted its zlib DT_NEEDED but references gzeof. Promote only zlib,
    // never the V0 SDK/OpenCV symbols, into the global lookup scope.
    const char* linked_zlib_version = zlibVersion();
    if (!linked_zlib_version || !linked_zlib_version[0])
        throw std::runtime_error("Private V0 shim has no real zlib dependency");
    void* zlib = dlopen("libz.so.1", RTLD_NOW | RTLD_GLOBAL);
    if (!zlib) throw std::runtime_error(std::string("Cannot preload required libz.so.1: ") + dlerror());
    dlerror();
    void* gzeof = dlsym(zlib, "gzeof");
    const char* zerror = dlerror();
    if (zerror || !gzeof) throw std::runtime_error("Required system zlib does not export gzeof");
    Dl_info zinfo{};
    if (!dladdr(gzeof, &zinfo) || !zinfo.dli_fname)
        throw std::runtime_error("Cannot identify actual required zlib library");

    // No SONAME search, environment override, system SDK, or older donor
    // fallback: the unchanged, hash-bound original is loaded by absolute path.
    void* library = dlopen(original.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(std::string("Selected V0 SDK dlopen failed: ") + dlerror());
    dlerror();
    void* symbol = dlsym(library, kFactory);
    const char* error = dlerror();
    if (error || !symbol) throw std::runtime_error(std::string("Selected V0 SDK factory unavailable: ") + (error ? error : "null"));
    Dl_info owner{};
    if (!dladdr(symbol, &owner) || !owner.dli_fname ||
        !std::filesystem::equivalent(original, owner.dli_fname))
        throw std::runtime_error("Factory symbol did not resolve inside the selected V0 SDK");
    if (r36_recharge_integrity::fileSha256(original.string()) != kOriginalSha)
        throw std::runtime_error("Selected V0 SDK changed while loading");

    state.path = std::filesystem::canonical(original).string();
    state.zlib_path = std::filesystem::canonical(zinfo.dli_fname).string();
    state.original = library;
    state.zlib = zlib;
    state.factory = reinterpret_cast<Api* (*)()>(symbol);
    // Paths in the deployed owned directory do not contain JSON special
    // characters. Diagnostics below remain useful even for arbitrary paths.
    state.identity = "original_sha256=" + std::string(kOriginalSha) +
        ";header_sha256=" + kHeaderSha + ";library=" + state.path +
        ";zlib=" + state.zlib_path + ";zlib_version=" + linked_zlib_version +
        ";sdk_scope=RTLD_NOW|RTLD_LOCAL;handles=process_lifetime";
    std::fprintf(stderr, "[recharge_v0] %s\n", state.identity.c_str());
}
} // namespace

extern "C" int r36_recharge_v0_open(Api** api, void** lease,
                                    char* error, std::size_t error_size) {
    if (api) *api = nullptr;
    if (lease) *lease = nullptr;
    if (error && error_size) error[0] = 0;
    if (!api || !lease) return -1;
    auto& state = runtime();
    std::lock_guard<std::mutex> lock(state.mutex);
    try {
        if (!state.leases.empty())
            throw std::runtime_error("Selected V0 SDK uses process-global state: another API lease is active");
        load(state);
        auto* result = state.factory();
        if (!result || !result->init || !result->loadConfig || !result->resetSession ||
            !result->processLeftNv12 || !result->freeResult || !result->destroy ||
            !result->getHostConfig || !result->setSaveResultEnable || !result->useSdkCameraMatrix) {
            if (result) { if (result->destroy) result->destroy(false); delete result; }
            throw std::runtime_error("Selected V0 SDK returned an incomplete matching-header API");
        }
        auto* owned = new Lease{result};
        state.leases.insert(owned);
        *api = result;
        *lease = owned;
        return 0;
    } catch (const std::exception& exception) {
        if (error && error_size) std::snprintf(error, error_size, "%s", exception.what());
        return -1;
    }
}

extern "C" void r36_recharge_v0_close(void* pointer) {
    if (!pointer) return;
    auto& state = runtime();
    std::lock_guard<std::mutex> lock(state.mutex);
    auto* lease = static_cast<Lease*>(pointer);
    const auto found = state.leases.find(lease);
    if (found == state.leases.end()) return;
    if (lease->api) {
        lease->api->destroy(false);
        delete lease->api;
    }
    state.leases.erase(found);
    delete lease;
    // Keep real SDK/zlib handles mapped until process exit; see Runtime.
}

extern "C" const char* r36_recharge_v0_identity() {
    auto& state = runtime();
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.identity.empty() ? "V0 SDK not loaded (recharge disabled/uninitialized)" : state.identity.c_str();
}
