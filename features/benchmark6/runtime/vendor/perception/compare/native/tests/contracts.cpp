#include "json_util.hpp"
#include "perception_compare/native_compare.hpp"
#include "ply.hpp"
#include "product_graph.hpp"
#include "disparity_compare.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

using namespace perception_compare;
using namespace perception_compare::detail;
namespace {
void check(bool condition, const std::string &why) {
  if (!condition)
    throw std::runtime_error("Test failure: " + why);
}
template <class F> void rejects(F &&f, const std::string &why) {
  bool rejected = false;
  try {
    f();
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected, why);
}
double value(const Json &j) {
  const auto n = numeric(j);
  check(bool(n), "expected actual numeric field");
  return *n;
}
void near(double actual, double expected, const std::string &why) {
  check(std::abs(actual - expected) < 1e-7, why);
}
std::string ply(double x = 1) {
  std::ostringstream s;
  s << "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty "
       "float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty int "
       "label\nend_header\n"
    << x << " 2 3 255 0 0 4\n4 5 6 0 255 0 2\n7 8 9 0 0 255 1\n";
  return s.str();
}
Json row(const std::string &method, const std::string &stem, double wall, double cpu) {
  return obj({{"method", str(method)},
              {"stem", str(stem)},
              {"repeat", integer(0)},
              {"warmup", boolean(false)},
              {"has_inference", boolean(true)},
              {"compute_wall_ms", number(wall)},
              {"cpu_wall_ms", number(wall)},
              {"total_wall_ms", number(wall + 5)},
              {"cpu_ms", number(cpu)},
              {"online_cpus", integer(4)},
              {"rss_kib", integer(1024)},
              {"inference_ms", obj({{"det_infer", number(3)}})},
              {"task_trace", obj({{"tasks", array()}})}});
}
void reporting(const fs::path &root) {
  const auto output = root / "report";
  fs::create_directories(output / "original");
  fs::create_directory(output / "stereo");
  writeJson(output / "run.json", obj({{"evidence", str("synthetic_validation")}}));
  writeText(output / "original" / "cloud.ply", ply());
  writeText(output / "stereo" / "cloud.ply", ply(1.1));
  auto a = row("original", "pairA", 10, 10), b = row("original", "pairB", 30, 90),
       warm = row("original", "warmup", 1000, 1000), failure = row("original", "failed", 999, 999);
  warm.object["warmup"] = boolean(true);
  failure.object["success"] = boolean(false);
  a.object["clouds"] = obj({{"robot", str("cloud.ply")}});
  writeText(output / "original" / "frames.jsonl",
            dump(a) + "\n" + dump(warm) + "\n" + dump(b) + "\n" + dump(failure) + "\n");
  a = row("stereo", "pairA", 20, 20);
  b = row("stereo", "pairB", 40, 20);
  a.object["clouds"] = obj({{"robot", str("cloud.ply")}});
  writeText(output / "stereo" / "frames.jsonl", dump(a) + "\n" + dump(b) + "\n");
  writeJson(output / "original" / "complete.json", obj({{"complete", boolean(true)},
                                                        {"measured_frames", integer(2)},
                                                        {"measured_wall_ms", number(100)},
                                                        {"measured_cpu_ms", number(200)},
                                                        {"online_cpus", integer(4)}}));
  generateReport(output, 2);
  const auto summary = readJson(output / "summary.json"),
             original = field(field(summary, "methods"), "original");
  near(value(field(original, "cpu_percent")), 250,
       "CPU uses sum(cpu)/sum(wall), not arithmetic mean");
  near(value(field(original, "compute_fps")), 50, "compute FPS excludes warmup/failures");
  near(value(field(original, "observed_replay_fps")), 20,
       "observed FPS requires real complete-loop measurement");
  check(field(field(field(original, "metrics"), "stereo_ms"), "mean").kind == Json::Kind::Null,
        "missing original stereo metric stays null");
  check(perception_compare::detail::uint64(
            field(field(field(original, "metrics"), "det_pre_ms"), "missing"), "missing") == 2,
        "missing timing samples counted");
  check(perception_compare::detail::uint64(field(summary, "paired_frame_count"), "paired") == 2,
        "match actual capture stem+repeat");
  const auto html = readText(output / "index.html");
  check(html.find("__DATA__") == std::string::npos &&
            html.find("pointermove") != std::string::npos &&
            html.find("synthetic_validation") != std::string::npos,
        "self-contained rotatable viewer with honest source label");
  check(html.find("\"displayed_points\":2") != std::string::npos &&
            html.find("\"total_points\":3") != std::string::npos,
        "preview sampling does not change full point count");
  check(readPly(output / "original" / "cloud.ply").vertices.size() == 3, "ASCII PLY parsing");
  fs::create_directory(output / "detseg_only");
  writeText(output / "detseg_only/frames.jsonl",dump(row("detseg_only","pairA",5,5))+"\n"+dump(row("detseg_only","pairB",5,5))+"\n");
  writeJson(output / "detseg_only/complete.json", obj({{"complete",boolean(true)}, {"measured_frames",integer(2)},
      {"bpu",obj({{"status",str("ok")},{"scope",str("device_shared")},{"mean_pct",number(0)},{"p95_pct",number(0)},{"max_pct",number(0)}})}}));
  generateReport(output,2);
  const auto triple=readJson(output/"summary.json");
  check(field(field(triple,"methods"),"detseg_only").kind==Json::Kind::Object,"third method appears in summary");
  near(value(field(triple,"paired_frame_count")),2,"three-way common stem/repeat intersection");
  near(value(field(field(field(triple,"methods"),"detseg_only"),"bpu").object.at("mean_pct")),0,"actual BPU zero preserved");
  check(field(field(field(triple,"methods"),"original"),"bpu").kind==Json::Kind::Null,"missing BPU stays null");
  check(readText(output/"summary.csv").find("bpu_device_mean_pct")!=std::string::npos,"BPU exported to CSV");
  // Canonical product feature names and public pointcloud/fusion aliases must
  // classify the same renamed mapping tasks; AI/unconfigured trace work is excluded.
  auto alias_graph=parse(R"({"nodes":[
    {"id":"det_alias","feature":"product.ai.detection"},
    {"id":"seg_alias","feature":"product.ai.segmentation"},
    {"id":"depth_alias","feature":"product.stereo.disparity"},
    {"id":"cloud_alias","feature":"product.pointcloud.from_disparity","depends_on":["depth_alias"]},
    {"id":"calibration_alias","feature":"product.calibration.auto","depends_on":["cloud_alias"]},
    {"id":"labels_alias","feature":"product.fusion.semantic_labels","depends_on":["calibration_alias","det_alias","seg_alias"]},
    {"id":"transform_alias","feature":"product.pointcloud.transform_color","depends_on":["labels_alias"]},
    {"id":"ground_alias","feature":"product.pointcloud.patchwork","depends_on":["transform_alias"]},
    {"id":"maps_alias","feature":"product.fusion.ai_fusion","depends_on":["ground_alias"]}]})","report alias graph");
  auto alias_tasks=array();unsigned stage_time=0;
  for(const auto* id:{"depth_alias","cloud_alias","calibration_alias","labels_alias","transform_alias","ground_alias","maps_alias"})
    alias_tasks.array.push_back(obj({{"task",str(id)},{"state",str("succeeded")},{"ms",integer(++stage_time)}}));
  for(const auto* id:{"det_alias","seg_alias","not_configured_stage"})
    alias_tasks.array.push_back(obj({{"task",str(id)},{"state",str("succeeded")},{"ms",integer(100)}}));
  a=row("original","pairA",10,10);b=row("original","pairB",30,90);
  a.object["task_trace"]=b.object["task_trace"]=obj({{"tasks",alias_tasks}});
  writeText(output/"original/frames.jsonl",dump(a)+"\n"+dump(b)+"\n");
  for(bool public_short_aliases:{false,true}) {
    if(public_short_aliases)for(auto& node:alias_graph.object["nodes"].array) {
      const auto stage=r33::full::productStageKey(textValue(field(node,"feature")));
      node.object["feature"]=str(stage=="cloud"?"pointcloud":stage=="maps"?"fusion":stage);
    }
    writeJson(output/"full_original.json",obj({{"graph",alias_graph}}));
    generateReport(output,2);
    const auto alias_report=readJson(output/"summary.json");
    near(value(field(field(field(field(alias_report,"methods"),"original"),"metrics"),"mapping_work_ms").object.at("mean")),28,
         "Mapping work must use normalized product stages and renamed node ids");
  }
  writeText(output / "bad.ply", "ply\nformat ascii 1.0\nelement vertex 99999999\nproperty float "
                                "x\nproperty float y\nproperty float z\nend_header\n");
  rejects([&] { readPly(output / "bad.ply"); },
          "truncated/count-corrupted PLY rejected before allocation");
  writeText(output / "original" / "frames.jsonl", dump(row("original", "pairA", 10, 10)) + "\n" +
                                                      dump(row("original", "pairA", 30, 90)) +
                                                      "\n");
  rejects([&] { generateReport(output, 2); }, "duplicate measured identity rejected");
}
Options prepareFixture(const fs::path &root) {
  const auto capture = root / "capture";
  fs::create_directory(capture);
  writeJson(capture / "metadata.json", obj({{"validation_only", boolean(true)},
                                            {"intrinsics", obj({{"width", integer(640)},
                                                                {"height", integer(544)},
                                                                {"fx", number(300)},
                                                                {"fy", number(300)},
                                                                {"cx", number(320)},
                                                                {"cy", number(272)},
                                                                {"bxf", number(18000)}})}}));
  std::string left(640 * 544, char(80));
  left.append(640 * 544 / 2, char(128));
  std::string right(640 * 544, char(90));
  right.append(640 * 544 / 2, char(128));
  writeText(capture / "left.nv12", left);
  writeText(capture / "right.nv12", right);
  writeText(capture / "q8.bin", std::string(640 * 544 * 2, '\1'));
  auto frame = obj({{"frame_id", integer(9007199254740993ULL)},
                    {"timestamp_ns", integer(9007199254740995ULL)},
                    {"width", integer(640)},
                    {"height", integer(544)},
                    {"left_format", str("nv12")},
                    {"right_format", str("nv12")},
                    {"disparity_format", str("q8_le")},
                    {"left", str("left.nv12")},
                    {"right", str("right.nv12")},
                    {"disparity", str("q8.bin")}});
  writeText(capture / "manifest.jsonl", dump(frame) + "\n");
  writeJson(root / "mapping.json", obj({{"dnn", object()}, {"auto_cali", boolean(true)}}));
  writeJson(root / "lane_det.json", object());
  writeJson(root / "lane_seg.json", object());
  writeJson(
      root / "full.json",
      obj({{"inference", obj({{"enabled", boolean(false)},
                                {"detection_dag", str("lane_det.json")},
                                {"segmentation_dag", str("lane_seg.json")}})},
           {"input", obj({{"disparity_source", str("original_q8")}})},
           {"mapping", obj({{"enabled", boolean(true)}, {"original_config", str("mapping.json")}})},
           {"recharge", object()},
           {"history", obj({{"capacity", integer(3)},
                            {"frame_interval", integer(2)},
                            {"max_memory_mb", integer(96)},
                            {"difference", boolean(true)},
                            {"match", boolean(true)}})},
           {"output", object()}}));
  writeText(root / "model.bin", "not_a_runtime_model: prepare-only-contract-fixture\n");
  Options options;
  options.capture = capture;
  options.sdk_config = root / "full.json";
  options.output = root / "prepared";
  options.model = root / "model.bin";
  options.prepare_only = true;
  return options;
}
void recording(const fs::path &root) {
  writeText(root / "abc", "abc");
  check(sha256(root / "abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "standard SHA256 abc vector");
  auto options = prepareFixture(root);
  fs::rename(options.capture / "q8.bin", options.capture / "q8.hidden");
  rejects([&] { runCompare(options); }, "missing original Q8 rejected");
  check(!fs::exists(options.output), "validation failure precedes output creation");
  fs::rename(options.capture / "q8.hidden", options.capture / "q8.bin");
  runCompare(options);
  const auto prepared_sdk = readJson(options.output / "full.json");
  const auto input_sdk = readJson(options.sdk_config);
  check(dump(field(prepared_sdk, "history")) == dump(field(input_sdk, "history")),
        "Replay preserves the requested capacity, sampling interval, memory and temporal analysis");
  check(textValue(field(field(prepared_sdk, "input"), "disparity_source")) == "raft_stereo" &&
            textValue(field(field(readJson(options.output / "full_stereo.json"), "input"),
                           "disparity_source")) == "raft_stereo",
        "Explicit stereo comparison method must select model inference over inherited Q8");
  for (const auto &lane : {"detection_dag", "segmentation_dag"})
    check(fs::path(textValue(field(field(prepared_sdk, "inference"), lane))).is_absolute(),
          "Relative lane DAG path was not resolved before moving the full config");
  check(field(readJson(options.output / "original_start.json"), "auto_cali").boolean,
        "Replay preserves the requested original automatic calibration");
  const auto inventory = readJson(options.output / "input_inventory.json");
  check(field(inventory.array[0], "frame_id").text == "9007199254740993",
        "uint64 frame identity preserved above IEEE double exact range");
  check(field(inventory.array[0], "timestamp_ns").text == "9007199254740995",
        "uint64 timestamp preserved");
  const auto manifest = readText(options.output / "prepared/pairs.tsv");
  check(manifest.find("9007199254740993") != std::string::npos,
        "stable pair stem includes exact identity");
  check(cv::imread(
            (options.output / "prepared/capture-000001-9007199254740993-9007199254740995/left.png")
                .string())
                .size() == cv::Size(640, 544),
        "C++ NV12 decoding prepares original-size PNG");
  check(!fs::exists(options.output / "original") && !fs::exists(options.output / "summary.json"),
        "prepare-only produces no algorithm measurements");
  rejects([&] { runCompare(options); }, "existing output cannot be overwritten");
  auto comparison_input = input_sdk;
  comparison_input.object["input"].object["disparity_source"] = str("raft_stereo");
  writeJson(root / "explicit_raft.json", comparison_input);
  auto original_options = options;
  original_options.sdk_config = root / "explicit_raft.json";
  original_options.method = "original";
  original_options.output = root / "explicit_original";
  runCompare(original_options);
  check(textValue(field(field(readJson(original_options.output / "full_original.json"), "input"),
                             "disparity_source")) == "original_q8",
        "Explicit original comparison method did not override inherited Stereo selector");
  auto both_options = options;
  both_options.both = true;
  both_options.output = root / "both_prepared";
  runCompare(both_options);
  for (const auto &method : {"original", "stereo"}) {
    const auto selected = readJson(both_options.output / (std::string("full_") + method + ".json"));
    check(textValue(field(field(selected, "input"), "disparity_source")) ==
                (std::string(method) == "original" ? "original_q8" : "raft_stereo"),
          "Both comparison methods share an incorrect frozen disparity selector");
    check(dump(field(selected, "history")) == dump(field(input_sdk, "history")),
          "Per-method configuration changed the requested History behavior");
  }
  auto triple_full=input_sdk;
  triple_full.object["mode"]=str("map_and_recharge");
  triple_full.object["inference"].object["enabled"]=boolean(true);
  triple_full.object["inference"].object["dag"]=str("triple_dag.json");
  triple_full.object["inference"].object["detection_model"]=str("triple_det.bin");
  triple_full.object["inference"].object["segmentation_model"]=str("triple_seg.bin");
  writeText(root/"triple_det.bin","prepare-only det fixture"); writeText(root/"triple_seg.bin","prepare-only seg fixture");
  writeJson(root/"triple_dag.json",parse(R"({"tasks":{"det":{"features":["ai.exact.det.backend"]},"seg":{"features":["ai.exact.seg.backend"]}}})","triple fixture"));
  triple_full.object["graph"]=parse(R"({"nodes":[
    {"id":"det","feature":"inference","algorithm":"det"},
    {"id":"seg","feature":"inference","algorithm":"seg"},
    {"id":"stereo","feature":"inference","algorithm":"stereo"},
    {"id":"cloud","feature":"cloud","depends_on":["stereo"]},
    {"id":"publish","feature":"publish","depends_on":["det","seg","cloud"]}]})","triple graph");
  writeJson(root/"triple_full.json",triple_full);
  auto three=options;three.all=true;three.order="detseg_only,original,stereo";three.sdk_config=root/"triple_full.json";three.output=root/"three_prepared";
  runCompare(three);
  const auto only=readJson(three.output/"full_detseg_only.json");
  check(!field(field(only,"mapping"),"enabled").boolean,"baseline mapping disabled");
  check(textValue(field(only,"mode"))=="map"&&!field(field(only,"history"),"difference").boolean&&
        !field(field(only,"history"),"match").boolean,"baseline mode and temporal flags match pruned graph");
  for(const auto &node:field(field(only,"graph"),"nodes").array)
    if(textValue(field(node,"id"))=="stereo"||textValue(field(node,"id"))=="cloud") check(!field(node,"enabled").boolean,"baseline geometry pruned");
  for(const auto &method:{"original","stereo"})
    check(field(field(readJson(three.output/(std::string("full_")+method+".json")),"mapping"),"enabled").boolean,"baseline pruning must not leak into geometry methods");
  const auto triple_manifest=readText(three.output/"prepared/pairs.tsv");
  check(std::count(triple_manifest.begin(),triple_manifest.end(),'\t')==5,"six-column manifest preserves original right NV12");
  // Exercise the actual three-way preparer with both remaining feature-name
  // families supported by parseProductGraph, rather than testing a copied map.
  for(bool canonical_names:{true,false}) {
    auto aliased=triple_full;
    for(auto& node:aliased.object["graph"].object["nodes"].array) {
      auto stage=r33::full::productStageKey(textValue(field(node,"feature")));
      const auto algorithm=textValue(field(node,"algorithm"));
      if(!algorithm.empty())stage=algorithm=="stereo"?"disparity":algorithm;
      node.object["feature"]=str(canonical_names?r33::full::productFeatureName(stage):stage=="cloud"?"pointcloud":stage);
      node.object.erase("algorithm");
    }
    const auto family=std::string(canonical_names?"canonical":"short");
    const auto path=root/("triple_"+family+".json");writeJson(path,aliased);
    auto alias_options=three;alias_options.sdk_config=path;alias_options.output=root/("three_"+family);
    runCompare(alias_options);
    const auto selected=readJson(alias_options.output/"full_detseg_only.json");
    const auto baseline=r33::full::parseProductGraph(field(selected,"graph"));
    check(baseline.enabled("det")&&baseline.enabled("seg")&&!baseline.enabled("cloud")&&!baseline.enabled("disparity"),
          "Alias inference lanes must survive detseg baseline pruning while geometry is disabled");
    for(const auto* method:{"original","stereo"}) {
      const auto fusion=readJson(alias_options.output/(std::string("full_")+method+".json"));
      check(dump(field(fusion,"graph"))==dump(field(aliased,"graph")),"Alias geometry graph changed between comparison methods");
    }
  }
  three.order="original,stereo";three.output=root/"bad_three_order";
  rejects([&]{runCompare(three);},"incomplete three-way order rejected");
  auto disparity_only = input_sdk;
  disparity_only.object["mapping"].object["enabled"] = boolean(false);
  disparity_only.object["mapping"].object.erase("original_config");
  disparity_only.object["graph"] = parse(R"({"nodes":[
    {"id":"stereo","feature":"inference","algorithm":"stereo"},
    {"id":"publish","feature":"publish","depends_on":["stereo"]}]})", "disparity graph");
  writeJson(root / "disparity_only.json", disparity_only);
  auto disparity_options = options;
  disparity_options.sdk_config = root / "disparity_only.json";
  disparity_options.output = root / "disparity_only_prepared";
  runCompare(disparity_options);
  check(!field(field(readJson(disparity_options.output / "full_stereo.json"), "mapping"), "enabled").boolean,
        "Stereo-only recording was silently forced to enable mapping");
  check(field(readJson(disparity_options.output / "original_start.json"), "dnn").kind == Json::Kind::Object,
        "Disabled mapping without an original_config did not prepare safely");
  auto missing_mapping = disparity_only;
  missing_mapping.object["mapping"].object["enabled"] = boolean(true);
  writeJson(root / "missing_mapping.json", missing_mapping);
  auto missing_mapping_options = disparity_options;
  missing_mapping_options.sdk_config = root / "missing_mapping.json";
  missing_mapping_options.output = root / "missing_mapping_prepared";
  rejects([&] { runCompare(missing_mapping_options); },
          "Enabled mapping without original_config accepted");
  check(!fs::exists(missing_mapping_options.output),
        "Missing enabled mapping configuration was not rejected before output creation");
  disparity_only.object["graph"].object["nodes"].array[0].object["algorithm"] = str("det");
  writeJson(root / "no_disparity.json", disparity_only);
  disparity_options.sdk_config = root / "no_disparity.json";
  disparity_options.output = root / "no_disparity_prepared";
  rejects([&] { runCompare(disparity_options); }, "Stereo comparison without disparity work accepted");
  // Actual model identity follows the effective split graph, even when the
  // compatibility DAG still lists another lane that is deliberately absent.
  auto split_detector = input_sdk;
  split_detector.object["mapping"].object["enabled"] = boolean(false);
  split_detector.object["inference"].object["enabled"] = boolean(true);
  split_detector.object["inference"].object["dag"] = str("combined_dag.json");
  split_detector.object["inference"].object["detection_model"] = str("det_model.bin");
  split_detector.object["inference"].object["segmentation_model"] = str("unused_missing_seg.bin");
  split_detector.object["graph"] = parse(R"({"nodes":[
    {"id":"det","feature":"inference","algorithm":"det"},
    {"id":"publish","feature":"publish","depends_on":["det"]}]})", "det graph");
  writeJson(root / "combined_dag.json", parse(R"({"tasks":{
    "detection":{"features":["ai.exact.det.backend"]},
    "segmentation":{"features":["ai.exact.seg.backend"]}}})", "compatibility DAG"));
  writeText(root / "det_model.bin", "det-only identity fixture\n");
  writeJson(root / "split_detector.json", split_detector);
  auto detector_options = original_options;
  detector_options.sdk_config = root / "split_detector.json";
  detector_options.output = root / "split_detector_prepared";
  runCompare(detector_options);
  const auto detector_identity = field(readJson(detector_options.output / "run.json"), "actual_models");
  check(field(detector_identity, "detection_model").kind == Json::Kind::String &&
            field(detector_identity, "segmentation_model").kind == Json::Kind::Null,
        "Split detector model identity incorrectly required the inactive segmentation model");
  auto legacy_full = input_sdk;
  legacy_full.object.erase("history");
  legacy_full.object["mapping"].object["original_config"] = str("legacy_mapping.json");
  writeJson(root / "legacy_mapping.json", obj({{"dnn", object()}}));
  writeJson(root / "legacy_full.json", legacy_full);
  options.sdk_config = root / "legacy_full.json";
  options.output = root / "legacy_prepared";
  runCompare(options);
  check(field(field(readJson(options.output / "full.json"), "history"), "capacity").text == "1",
        "Legacy configurations without history retain the one-frame fallback");
  check(!field(readJson(options.output / "original_start.json"), "auto_cali").boolean,
        "Legacy configurations without auto_cali retain the disabled fallback");
}
void versions(const fs::path &root) {
  const auto source = root / "prepared", baseline = root / "baseline",
             candidate = root / "candidate";
  fs::copy(source, baseline, fs::copy_options::recursive);
  fs::copy(source, candidate, fs::copy_options::recursive);
  for (const auto &output : {baseline, candidate}) {
    fs::create_directories(output / "original/frame");
    auto r = row("original", "captureA", 10, 10);
    r.object["output_dir"] = str("frame");
    r.object["segmentation_enabled"] = boolean(true);
    writeText(output / "original/frames.jsonl", dump(r) + "\n");
    writeJson(output / "original/complete.json",
              obj({{"complete", boolean(true)}, {"measured_frames", integer(1)}}));
    writeJson(output / "original/frame/result.json", obj({{"detections", array()}}));
    writeText(output / "original/frame/mask.bin", "abc");
    writeText(output / "original/frame/pcl.cells.bin", "cells");
    writeText(output / "original/frame/robot.ply", ply());
  }
  check(checkVersions(baseline, candidate, root / "check_equal"),
        "same-data identical artifacts pass");
  auto legacy = readJson(baseline / "run.json");
  legacy.object["intrinsics"].object["fx"] = parse("300.0", "legacy number");
  // Original-only v2 runs recorded null while an early v3 writer used an
  // irrelevant resize object. Neither may alter original-Q8 run identity.
  legacy.object["stereo_input_requested"] = obj({{"stereo-input-mode", str("resize")}});
  writeJson(baseline / "run.json", legacy);
  check(checkVersions(baseline, candidate, root / "check_legacy_numeric"),
        "legacy numeric JSON spellings and unused stereo options are equivalent");
  writeText(candidate / "original/frame/mask.bin", "abd");
  check(!checkVersions(baseline, candidate, root / "check_mask"),
        "mask byte difference is detected");
  writeText(candidate / "original/frame/mask.bin", "abc");
  writeText(candidate / "original/frame/robot.ply", ply(1.1));
  check(!checkVersions(baseline, candidate, root / "check_xyz"), "PLY XYZ difference detected");
  check(checkVersions(baseline, candidate, root / "check_tolerance", .11),
        "explicit XYZ tolerance applies without point alignment");
  const auto diff = readJson(root / "check_xyz/version_diff.json");
  check(!field(diff, "equal").boolean, "diff summary reflects actual artifact comparison");
  auto changed_roi=readJson(candidate/"run.json");
  changed_roi.object["comparison_roi"]=comparisonRoiJson(
      comparisonRoi("crop",640,352,Intrinsics{640,544,300,300,320,272,18000}),
      Intrinsics{640,544,300,300,320,272,18000});
  writeJson(candidate/"run.json",changed_roi);
  rejects([&]{checkVersions(baseline,candidate,root/"check_mismatched_roi");},
          "Full-frame v5 and cropped v6 outputs cannot share a code-version comparison identity");
  check(!fs::exists(root/"check_mismatched_roi"),"ROI mismatch must fail before version output creation");
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("Provide test output parent");
    const auto root =
        fs::absolute(fs::u8path(argv[1])) /
        ("test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);
    recording(root);
    reporting(root);
    versions(root);
    std::cout << "PASS: native capture validation, SHA256/uint64, PNG decode, report weighted "
                 "metrics, PLY preview, version artifact regression.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
