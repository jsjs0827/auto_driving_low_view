#include "perception_full_sdk.h"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void need(bool value, const char *detail) {
  if (!value) throw std::runtime_error(detail);
}
void ok(uint32_t code, const psdk_status &status) {
  if (code) throw std::runtime_error(std::to_string(code) + ": " + status.detail);
}
struct State { std::vector<uint16_t> q8 = std::vector<uint16_t>(64 * 48, 513); unsigned calls = 0, published = 0; };
uint32_t provide(void *user, const psdk_frame *left, psdk_disparity_q8 *out, psdk_status *) {
  auto &state = *static_cast<State *>(user);
  ++state.calls;
  *out = {sizeof(*out), 1, 64, 48, 128, 0, state.q8.data(), state.q8.size() * 2,
          left->frame_id, left->timestamp_ns};
  return 0;
}
uint32_t publish(void *user, const psdk_full_result_view *view, psdk_status *) {
  auto &state = *static_cast<State *>(user);
  ++state.published;
  need(view->disparity.data && view->disparity.data[0] == 513, "publish did not join disparity producer");
  need(!view->has_inference && !view->cloud_count && !view->map_count, "standalone disparity invoked unrelated algorithms");
  return 0;
}
struct Engine { psdk_full_engine *p = nullptr; ~Engine() { psdk_full_destroy(&p, nullptr); } };
using Result = std::unique_ptr<psdk_full_result, decltype(&psdk_full_result_release)>;
psdk_full_result_view view(const Result &result) {
  psdk_full_result_view value{}; value.struct_size = sizeof(value); value.abi_version = 1;
  psdk_status status{}; ok(psdk_full_result_get_view(result.get(), &value, &status), status);
  return value;
}
}
int main(int argc, char **argv) { try {
  need(argc == 2, "config required");
  State state;
  psdk_full_hooks hooks{sizeof(hooks), 1, &state, provide, publish};
  psdk_full_config config{sizeof(config), 1, argv[1], nullptr, nullptr, &hooks};
  Engine engine; psdk_status status{};
  ok(psdk_full_create(&config, &engine.p, &status), status);
  std::vector<uint8_t> image(64 * 48 * 3 / 2, 128);
  psdk_full_frame frame{}; frame.struct_size = PSDK_FULL_FRAME_V1_SIZE; frame.abi_version = 1;
  frame.left = {sizeof(psdk_frame), 1, PSDK_FORMAT_NV12, 64, 48, 64, image.data(), image.size(), 1, 1000};
  frame.intrinsics = {64, 48, 40, 41, 32, 24, 2400};
  psdk_full_result *raw = nullptr;
  ok(psdk_full_process(engine.p, &frame, &raw, &status), status);
  Result provided(raw, psdk_full_result_release);
  // ABI v1 callers compiled before the optional right field remain valid.
  frame.struct_size = sizeof(frame);
  auto first = view(provided);
  need(state.calls == 1 && state.published == 1, "provider/publish skipped when mapping disabled");
  need(first.disparity.data != state.q8.data() && first.disparity.bytes == 64 * 48 * 2,
       "disparity result does not own its tight Q8 buffer");
  need(std::string(first.task_trace_json).find("\"task\":\"disp\",\"state\":\"succeeded\"") != std::string::npos,
       "actual disparity task missing from trace");
  std::fill(state.q8.begin(), state.q8.end(), 777);
  need(view(provided).disparity.data[0] == 513, "borrowed provider buffer leaked into owned result");
  // Supplied camera Q8 takes precedence over the model callback.
  frame.left.frame_id = 2; frame.left.timestamp_ns = 2000;
  psdk_disparity_q8 supplied{sizeof(supplied), 1, 64, 48, 128, 0, state.q8.data(), state.q8.size() * 2, 2, 2000};
  frame.disparity = &supplied;
  // This frame has a distinct expected Q8 value, so do not install the first-frame assertion hook.
  hooks.publication = nullptr;
  Engine camera;
  ok(psdk_full_create(&config, &camera.p, &status), status);
  raw = nullptr;
  ok(psdk_full_process(camera.p, &frame, &raw, &status), status);
  Result captured(raw, psdk_full_result_release);
  need(state.calls == 1 && view(captured).disparity.data[0] == 777, "camera Q8 was replaced by model provider");
  supplied.timestamp_ns = 1999;
  raw = nullptr;
  need(psdk_full_process(camera.p, &frame, &raw, &status) != 0 && !raw,
       "mismatched camera disparity identity was accepted");
  supplied.timestamp_ns = frame.left.timestamp_ns;
  const auto fixture_dir=std::filesystem::u8path(argv[1]).parent_path();
  const auto original_file=(fixture_dir/"v4_disparity_original.json").u8string();
  const auto raft_file=(fixture_dir/"v4_disparity_raft.json").u8string();
  auto selected=config;selected.json_file=original_file.c_str();
  Engine original_engine;ok(psdk_full_create(&selected,&original_engine.p,&status),status);
  const auto calls_before=state.calls;
  frame.disparity=nullptr;raw=nullptr;
  need(psdk_full_process(original_engine.p,&frame,&raw,&status)==1002&&!raw&&state.calls==calls_before,
       "original_q8 silently fell back to a model provider");
  frame.disparity=&supplied;
  ok(psdk_full_process(original_engine.p,&frame,&raw,&status),status);
  Result original_result(raw,psdk_full_result_release);
  need(view(original_result).disparity.data[0]==777&&state.calls==calls_before,
       "original_q8 did not use its genuine supplied Q8 input");
  selected.json_file=raft_file.c_str();
  Engine raft_engine;ok(psdk_full_create(&selected,&raft_engine.p,&status),status);
  raw=nullptr;
  need(psdk_full_process(raft_engine.p,&frame,&raw,&status)==1002&&!raw&&state.calls==calls_before,
       "raft_stereo silently accepted Q8 and bypassed its paired model input");
  frame.disparity=nullptr;
  need(psdk_full_process(raft_engine.p,&frame,&raw,&status)==1002&&!raw&&state.calls==calls_before,
       "raft_stereo accepted a missing right input");
  auto right=frame.left;frame.right=&right;
  ok(psdk_full_process(raft_engine.p,&frame,&raw,&status),status);
  Result stereo_result(raw,psdk_full_result_release);
  need(state.calls==calls_before+1&&view(stereo_result).disparity.data[0]==777,
       "raft_stereo did not execute its model provider with a valid pair");
  auto no_provider_hooks=hooks;no_provider_hooks.disparity_provider=nullptr;
  selected.hooks=&no_provider_hooks;
  Engine missing_provider;ok(psdk_full_create(&selected,&missing_provider.p,&status),status);
  raw=nullptr;
  need(psdk_full_process(missing_provider.p,&frame,&raw,&status)==1002&&!raw,
       "raft_stereo accepted a missing model provider");
  ok(psdk_full_destroy(&engine.p, &status), status);
  need(view(provided).disparity.data[0] == 513, "disparity result did not survive engine destruction");
  std::cout << "Independent disparity C ABI passed: strict original/raft source choice, right/provider contracts, old-size ABI, identity, publication join and owned result; fake provider, no model/hardware accuracy claim.\n";
  return 0;
} catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; } }
