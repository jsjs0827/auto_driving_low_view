#include "percep_main/v2_sdk_runtime.hpp"

namespace perception {
namespace {
bool checked(uint32_t code, const psdk_status& status, std::string& error)
{
    if (!code) return true;
    error = "V2 SDK " + std::to_string(code) + " " + status.detail;
    return false;
}
}

V2SdkRuntime::~V2SdkRuntime()
{
    if (engine_) psdk_full_destroy(&engine_, nullptr);
}

bool V2SdkRuntime::open(const std::string& path, std::string& error)
{
    psdk_status status{};
    const psdk_full_config config{sizeof(config), PSDK_ABI_VERSION, path.c_str(), nullptr, nullptr, nullptr};
    return checked(psdk_full_create(&config, &engine_, &status), status, error)
        && command(21, error);
}

bool V2SdkRuntime::command(int mode, std::string& error)
{
    psdk_status status{};
    const auto code = psdk_full_command(engine_, mode, &status);
    return checked(code, status, error);
}

V2Result V2SdkRuntime::process(const psdk_full_frame& frame, psdk_full_result_view& view, std::string& error)
{
    psdk_status status{};
    psdk_full_result* handle = nullptr;
    const auto code = psdk_full_process(engine_, &frame, &handle, &status);
    V2Result result(handle);
    if (!checked(code, status, error)) return {};
    view = {};
    view.struct_size = sizeof(view);
    view.abi_version = PSDK_ABI_VERSION;
    const auto view_code = psdk_full_result_get_view(result.get(), &view, &status);
    if (!checked(view_code, status, error)) return {};
    return result;
}

}  // namespace perception
