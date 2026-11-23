#include "perception_temporal_sdk.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace fs = std::filesystem;
namespace {
void need(bool v, const char *s) { if (!v) throw std::runtime_error(s); }
fs::path checkedFixtureRoot(const fs::path& root,const fs::path& parent) {
  const std::string prefix="temporal-json-contract-",name=root.filename().u8string();
  need(root.is_absolute() && name.size()>prefix.size() && name.rfind(prefix,0)==0 &&
       std::all_of(name.begin()+prefix.size(),name.end(),[](unsigned char c){return c>='0'&&c<='9';}),
       "unsafe cleanup path name");
  need(!fs::is_symlink(fs::symlink_status(root)) && fs::is_directory(root),"unsafe cleanup root type");
  const auto resolved=fs::canonical(root),base=fs::canonical(parent);
  need(fs::equivalent(root.parent_path(),base) && fs::equivalent(resolved.parent_path(),base) &&
       resolved.filename()==root.filename(),"unsafe cleanup path");
  return resolved;
}
void ok(uint32_t code, const psdk_status &s) { if (code) throw std::runtime_error(std::to_string(code) + ":" + s.detail); }
std::string read(const fs::path &p) {
  std::ifstream in(p, std::ios::binary); need(bool(in), "missing saved JSON");
  return std::string(std::istreambuf_iterator<char>(in), {});
}
void analysis(const std::string &json) {
  need(json.find("\"difference_valid\":true") != std::string::npos, "JSON was serialized before difference analysis");
  need(json.find("\"mean_abs_luma\":12") != std::string::npos, "JSON lost measured luma difference");
  need(json.find("\"previous_frame_id\":1") != std::string::npos, "JSON lost previous capture identity");
}
struct Engine { psdk_full_engine *p = nullptr; ~Engine() { psdk_full_destroy(&p, nullptr); } };
using Result = std::unique_ptr<psdk_full_result, decltype(&psdk_full_result_release)>;
}
int main(int argc, char **argv) { fs::path temp; try {
  need(argc == 2, "config required");
  const auto temp_parent=fs::canonical(fs::temp_directory_path());
  temp = temp_parent / ("temporal-json-contract-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  need(fs::create_directory(temp),"fixture directory must be new");
  temp=checkedFixtureRoot(temp,temp_parent);
  fs::copy_file(argv[1], temp / "full.json");
  const auto config_file = (temp / "full.json").u8string();
  psdk_full_config config{sizeof(config), 1, config_file.c_str(), nullptr, nullptr, nullptr};
  Engine engine; psdk_status error{}; ok(psdk_full_create(&config, &engine.p, &error), error);
  std::vector<uint8_t> bytes(64 * 48 * 3 / 2, 128);
  psdk_full_frame frame{}; frame.struct_size = sizeof(frame); frame.abi_version = 1;
  frame.left = {sizeof(psdk_frame), 1, PSDK_FORMAT_NV12, 64, 48, 64, bytes.data(), bytes.size(), 0, 0};
  for (unsigned i = 1; i <= 2; ++i) {
    std::fill(bytes.begin(), bytes.begin() + 64 * 48, i == 1 ? 10 : 22);
    frame.left.frame_id = i; frame.left.timestamp_ns = i * 1000;
    psdk_full_result *raw = nullptr; ok(psdk_full_process(engine.p, &frame, &raw, &error), error);
    Result result(raw, psdk_full_result_release);
    psdk_temporal_view view{}; view.struct_size = sizeof(view); view.abi_version = 1;
    ok(psdk_full_result_get_temporal_view(result.get(), &view, &error), error);
    need(view.frame_count == i, "unexpected temporal sample count");
    if (i == 2) {
      need(view.difference_valid && view.mean_abs_luma == 12, "difference computation missing");
      analysis(view.analysis_json);
      ok(psdk_full_save_result(result.get(), (temp / "explicit").u8string().c_str(), &error), error);
      analysis(read(temp / "explicit/temporal.json"));
      analysis(read(temp / "explicit/result.json"));
      const auto automatic = temp / "recordings/frame-2-2-2000";
      analysis(read(automatic / "temporal.json"));
      analysis(read(automatic / "result.json"));
    }
  }
  ok(psdk_full_destroy(&engine.p, &error), error);
  const auto cleanup_root=checkedFixtureRoot(temp,temp_parent);
  fs::remove_all(cleanup_root);
  std::cout << "Temporal analysis JSON is current without extension: public view, explicit save and graph record.\n";
  return 0;
} catch (const std::exception &e) { std::cerr << e.what() << "\nFixture retained: " << temp << '\n'; return 1; } }
