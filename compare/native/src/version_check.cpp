#include "perception_compare/native_compare.hpp"
#include "ply.hpp"
#include "disparity_compare.hpp"
#include <set>

namespace perception_compare {
using namespace detail;
namespace {
fs::path resolve(const fs::path &base, const std::string &name) {
  if (name.empty())
    throw std::runtime_error("Missing identity path");
  return regular(base / fs::u8path(name));
}
Json fileIdentity(const fs::path &p) {
  return obj({{"sha256", str(sha256(regular(p)))}, {"bytes", integer(fs::file_size(p))}});
}
// JSON writers may serialize the same double as 300, 300.0, or 3e2. Only
// calibration/benchmark values are normalized; frame IDs remain exact strings.
Json canonicalNumbers(Json value) {
  if (value.kind == Json::Kind::Number) {
    std::size_t used = 0;
    const auto n = std::stod(value.text, &used);
    if (used != value.text.size() || !std::isfinite(n))
      throw std::runtime_error("Invalid numeric run identity");
    return number(n);
  }
  for (auto &item : value.array)
    item = canonicalNumbers(std::move(item));
  for (auto &item : value.object)
    item.second = canonicalNumbers(std::move(item.second));
  return value;
}
Json stereoInputIdentity(const Json &run) {
  if (field(run, "stereo_model").kind != Json::Kind::Object)
    return {};
  const auto &requested = field(run, "stereo_input_requested");
  auto option = [&](const char *key, const char *legacy) -> const Json & {
    const auto &value = field(requested, key);
    return value.kind == Json::Kind::Null ? field(requested, legacy) : value;
  };
  auto result =
      obj({{"input_mode", str(textValue(option("input_mode", "stereo-input-mode"), "resize"))}});
  for (const auto &names : {std::pair{"input_width", "stereo-input-width"},
                            std::pair{"input_height", "stereo-input-height"}}) {
    const auto &value = option(names.first, names.second);
    const auto n = numeric(value);
    if (n && *n > 0)
      result.object[names.first] = canonicalNumbers(value);
  }
  return result;
}
Json modelIdentities(const fs::path &root, const Json &run, const Json &full) {
  auto models = object();
  const auto &inference = field(full, "inference");
  std::set<std::string> enabled_lanes;
  if (field(inference, "enabled").boolean) {
    bool split = false;
    for (const auto &node : field(field(full, "graph"), "nodes").array) {
      if (field(node, "enabled").kind != Json::Kind::Null && !field(node, "enabled").boolean)
        continue;
      const auto feature = textValue(field(node, "feature"));
      const auto algorithm = textValue(field(node, "algorithm"));
      if ((feature == "inference" && algorithm == "det") || feature == "det" ||
          feature == "product.ai.detection") { enabled_lanes.insert("detection"); split = true; }
      if ((feature == "inference" && algorithm == "seg") || feature == "seg" ||
          feature == "product.ai.segmentation") { enabled_lanes.insert("segmentation"); split = true; }
    }
    // The combined compatibility DAG can contain unused lanes. Split runtime
    // instances are selected by the effective product graph instead.
    if (!split) {
      const auto dag_path = textValue(field(inference, "dag"));
      if (dag_path.empty())
        enabled_lanes = {"detection", "segmentation"};
      else {
        const auto dag = readJson(resolve(root, dag_path));
        const auto scan = [&](const auto &self, const Json &j) -> void {
          if (j.kind == Json::Kind::String) {
            if (j.text == "ai.exact.det.backend")
              enabled_lanes.insert("detection");
            if (j.text == "ai.exact.seg.backend")
              enabled_lanes.insert("segmentation");
          }
          for (const auto &item : j.array)
            self(self, item);
          for (const auto &item : j.object)
            self(self, item.second);
        };
        scan(scan, dag);
      }
    }
  }
  for (const auto &[name, meta] : field(run, "model_overrides").object) {
    const auto p = resolve(root, textValue(field(meta, "path")));
    const auto hash = sha256(p);
    if (!textValue(field(meta, "sha256")).empty() && hash != textValue(field(meta, "sha256")))
      throw std::runtime_error("Model changed since recorded run identity");
    models.object[name] = str(hash);
  }
  auto cpu = Json{};
  const auto cpu_path = textValue(field(inference, "cpu_profile"));
  if (!cpu_path.empty() && !enabled_lanes.empty())
    cpu = readJson(resolve(root, cpu_path));
  std::map<std::string, std::string> original;
  const auto profile = textValue(field(inference, "original_profile"));
  if (!profile.empty() && !enabled_lanes.empty()) {
    std::istringstream stream(readText(resolve(root, profile)));
    std::string line;
    while (std::getline(stream, line)) {
      line = line.substr(0, line.find('#'));
      const auto equals = line.find('=');
      if (equals == std::string::npos)
        continue;
      auto trim = [](std::string s) {
        const auto a = s.find_first_not_of(" \t\r"), b = s.find_last_not_of(" \t\r");
        return a == std::string::npos ? std::string{} : s.substr(a, b - a + 1);
      };
      original[trim(line.substr(0, equals))] = trim(line.substr(equals + 1));
    }
  }
  for (const auto &lane : std::vector<std::pair<std::string, std::string>>{
           {"detection", "det_model"}, {"segmentation", "seg_model"}}) {
    if (!enabled_lanes.count(lane.first))
      continue;
    const auto key = lane.first + "_model";
    if (field(models, key).kind != Json::Kind::Null)
      continue;
    auto name = textValue(field(inference, key));
    fs::path base = root;
    if (name.empty() && cpu.kind == Json::Kind::Object) {
      name = textValue(field(field(cpu, lane.first), "model"));
      base = resolve(root, cpu_path).parent_path();
    }
    if (name.empty() && !profile.empty()) {
      name = original[lane.second];
      base = resolve(root, profile).parent_path();
    }
    if (name.empty())
      throw std::runtime_error("Cannot identify actual " + lane.first +
                               " model for correctness comparison");
    models.object[key] = str(sha256(resolve(base, name)));
  }
  const auto &stereo = field(run, "stereo_model");
  if (stereo.kind == Json::Kind::Object) {
    const auto hash = sha256(resolve(root, textValue(field(stereo, "path"))));
    if (hash != textValue(field(stereo, "sha256")))
      throw std::runtime_error("Stereo model changed after run");
    models.object["stereo_model"] = str(hash);
  }
  const auto &snapshot = field(run, "actual_models");
  if (snapshot.kind == Json::Kind::Object && dump(snapshot) != dump(models))
    throw std::runtime_error("Actual model bytes changed after measurement identity was captured");
  return models;
}
Json identity(const fs::path &root) {
  const auto run = readJson(root / "run.json"), full = readJson(root / "full.json");
  auto inputs = array();
  const auto inventory = readJson(root / "input_inventory.json");
  std::size_t index = 0;
  std::ifstream manifest(root / "prepared/pairs.tsv");
  if (!manifest)
    throw std::runtime_error("Correctness check requires prepared/pairs.tsv");
  std::string line;
  while (std::getline(manifest, line)) {
    if (line.empty())
      continue;
    std::vector<std::string> columns;
    std::istringstream stream(line);
    std::string item;
    while (std::getline(stream, item, '\t'))
      columns.push_back(item);
    if (columns.size() != 5 && columns.size() != 6)
      throw std::runtime_error("Invalid prepared pair manifest");
    if (index >= inventory.array.size())
      throw std::runtime_error("Prepared pair count differs from original input inventory");
    auto pair = array();
    pair.array.push_back(str(columns[0]));
    const auto &captured = inventory.array[index++];
    if (!textValue(field(captured, "stem")).empty() &&
        textValue(field(captured, "stem")) != columns[0])
      throw std::runtime_error("Prepared capture identity/order changed");
    const char *hash_fields[] = {"left_sha256", "prepared_left_sha256", "prepared_right_sha256",
                                 "disparity_sha256"};
    for (std::size_t i = 1; i < 5; ++i) {
      const auto hash = sha256(resolve(root / "prepared", columns[i]));
      const auto expected = textValue(field(captured, hash_fields[i - 1]));
      if (!expected.empty() && expected != hash)
        throw std::runtime_error("Input bytes changed after capture/preparation");
      pair.array.push_back(str(hash));
    }
    if(columns.size()==6) {
      const auto right_hash=sha256(resolve(root/"prepared",columns[5]));
      const auto expected=textValue(field(captured,"right_sha256"));
      if(expected.empty() || right_hash!=expected)
        throw std::runtime_error("Recorded right NV12 bytes changed after preparation");
      // Keep the canonical five-column identity compatible with v4 original-Q8
      // runs; the extra raw eye is independently verified against the inventory.
    }
    inputs.array.push_back(std::move(pair));
  }
  if (index != inventory.array.size())
    throw std::runtime_error("Input inventory coverage changed");
  if (inputs.array.empty())
    throw std::runtime_error("No prepared same-frame inputs");
  const auto models = modelIdentities(root, run, full);
  return obj({{"recording_manifest_sha256", field(run, "recording_manifest_sha256")},
              {"intrinsics", canonicalNumbers(field(run, "intrinsics"))},
              {"comparison_roi", canonicalNumbers(comparisonRoiIdentity(run))},
              {"prepared_inputs", inputs},
              {"models", models},
              {"stereo_input", stereoInputIdentity(run)},
              {"benchmark", canonicalNumbers(field(run, "benchmark"))}});
}
using SavedRows = std::map<std::string, Json>;
SavedRows rows(const fs::path &root, const std::string &method) {
  const auto complete = readJson(root / method / "complete.json");
  if (!field(complete, "complete").boolean)
    throw std::runtime_error("Incomplete version execution " + (root / method).u8string());
  std::ifstream f(root / method / "frames.jsonl");
  if (!f)
    throw std::runtime_error("Missing frames.jsonl");
  SavedRows out;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;
    auto j = parse(line, "version frame");
    if (field(j, "warmup").boolean)
      continue;
    const auto stem = textValue(field(j, "stem"));
    if (stem.empty())
      throw std::runtime_error("Version check needs stable capture stem");
    const auto repeat =
        field(j, "repeat").kind == Json::Kind::Number ? field(j, "repeat").text : "0";
    if (!out.emplace(stem + '\0' + repeat, j).second)
      throw std::runtime_error("Duplicate measured version frame");
  }
  if (out.empty() || uint64(field(complete, "measured_frames"), "measured_frames") != out.size())
    throw std::runtime_error("Incomplete measured frame coverage");
  return out;
}
fs::path artifact(const fs::path &root, const std::string &method, const Json &row,
                  const std::string &name) {
  const auto directory = textValue(field(row, "output_dir"));
  if (directory.empty())
    throw std::runtime_error("Every measured frame must be saved (set save_every_n=1)");
  const auto path = fs::weakly_canonical(root / method / fs::u8path(directory) / fs::u8path(name));
  if (!contained(root, path))
    throw std::runtime_error("Artifact path escapes run root");
  return path;
}
Json byteDiff(const fs::path &a, const fs::path &b) {
  if (!fs::is_regular_file(a) || !fs::is_regular_file(b))
    return obj({{"equal", boolean(false)},
                {"baseline_exists", boolean(fs::is_regular_file(a))},
                {"candidate_exists", boolean(fs::is_regular_file(b))}});
  const auto x = fileIdentity(a), y = fileIdentity(b);
  const bool equal = dump(x) == dump(y);
  return obj({{"equal", boolean(equal)}, {"baseline", x}, {"candidate", y}});
}
Json plyDiff(const fs::path &a, const fs::path &b, double tolerance) {
  if (!fs::is_regular_file(a) || !fs::is_regular_file(b))
    return obj({{"equal", boolean(false)}, {"error", str("Missing saved PLY")}});
  try {
    const auto x = readPly(a), y = readPly(b);
    bool equal = x.vertices.size() == y.vertices.size(), rgb_equal = equal, finite = true;
    double maximum = 0;
    std::uint64_t changed = 0;
    const auto count = std::min(x.vertices.size(), y.vertices.size());
    for (std::size_t i = 0; i < count; ++i) {
      bool different = false;
      for (int d = 0; d < 3; ++d) {
        const auto p = x.vertices[i].xyz[d], q = y.vertices[i].xyz[d];
        if (!std::isfinite(p) || !std::isfinite(q)) {
          finite = false;
          different = true;
          continue;
        }
        const auto delta = std::abs(double(p) - q);
        maximum = std::max(maximum, delta);
        if (delta > tolerance)
          different = true;
      }
      if (x.vertices[i].rgb != y.vertices[i].rgb)
        rgb_equal = false;
      if (different)
        ++changed;
    }
    equal = equal && !changed && rgb_equal && finite;
    return obj({{"equal", boolean(equal)},
                {"baseline_point_count", integer(x.vertices.size())},
                {"candidate_point_count", integer(y.vertices.size())},
                {"max_xyz_abs_m", finite ? number(maximum) : Json{}},
                {"finite_xyz", boolean(finite)},
                {"changed_xyz_vertices", integer(changed)},
                {"rgb_equal", boolean(rgb_equal)},
                {"xyz_tolerance_m", number(tolerance)}});
  } catch (const std::exception &e) {
    return obj({{"equal", boolean(false)}, {"error", str(e.what())}});
  }
}
} // namespace
namespace detail {
Json actualModels(const fs::path &root, const Json &run, const Json &full) {
  return modelIdentities(root, run, full);
}
} // namespace detail
bool checkVersions(const fs::path &baseline, const fs::path &candidate, const fs::path &output,
                   double tolerance) {
  if (!std::isfinite(tolerance) || tolerance < 0)
    throw std::runtime_error("xyz-tolerance must be finite and nonnegative");
  const auto a = fs::canonical(baseline), b = fs::canonical(candidate);
  const auto id_a = identity(a), id_b = identity(b);
  if (dump(id_a) != dump(id_b))
    throw std::runtime_error("Input/model/calibration/ROI/benchmark identities differ; this is not a "
                             "same-data code-version comparison");
  auto frames = array(), coverage = object();
  bool equal = true;
  std::size_t paired = 0;
  for (const auto *method : {"original", "stereo"}) {
    const bool has_a = fs::is_regular_file(a / method / "frames.jsonl"),
               has_b = fs::is_regular_file(b / method / "frames.jsonl");
    if (!has_a && !has_b)
      continue;
    if (has_a != has_b) {
      equal = false;
      coverage.object[method] = str("missing method on one side");
      continue;
    }
    const auto x = rows(a, method), y = rows(b, method);
    coverage.object[method] =
        obj({{"baseline_frames", integer(x.size())}, {"candidate_frames", integer(y.size())}});
    std::set<std::string> keys;
    for (const auto &item : x)
      keys.insert(item.first);
    for (const auto &item : y)
      keys.insert(item.first);
    for (const auto &key : keys) {
      const auto i = x.find(key), j = y.find(key);
      auto frame = obj({{"method", str(method)},
                        {"stem", str(key.substr(0, key.find('\0')))},
                        {"equal", boolean(false)}});
      if (i == x.end() || j == y.end()) {
        equal = false;
        frame.object["error"] = str("Missing frame on one side");
        frames.array.push_back(std::move(frame));
        continue;
      }
      ++paired;
      bool same = true;
      auto artifacts = object();
      const auto result_a = artifact(a, method, i->second, "result.json"),
                 result_b = artifact(b, method, j->second, "result.json");
      const auto ra = readJson(regular(result_a)), rb = readJson(regular(result_b));
      // Ignore run clocks, performance, local paths; compare actual detections.
      const bool detections = dump(field(ra, "detections")) == dump(field(rb, "detections"));
      artifacts.object["detections_equal"] = boolean(detections);
      same &= detections;
      for (const auto *name : {"mask.bin", "disparity_q8.u16", "disparity.png", "grid_type.u8",
                               "grid_avg_h_cm.i16", "grid_min_h_cm.i16", "grid_max_h_cm.i16"}) {
        const auto p = artifact(a, method, i->second, name),
                   q = artifact(b, method, j->second, name);
        if (!fs::exists(p) && !fs::exists(q)) {
          if (std::string(name) == "mask.bin" &&
              (field(i->second, "segmentation_enabled").kind == Json::Kind::Null ||
               field(i->second, "segmentation_enabled").boolean)) {
            same = false;
            artifacts.object[name] = obj(
                {{"equal", boolean(false)}, {"error", str("Enabled segmentation mask missing")}});
          }
          continue;
        }
        auto diff = byteDiff(p, q);
        same &= field(diff, "equal").boolean;
        artifacts.object[name] = std::move(diff);
      }
      // SDK map names can differ across revisions; inspect every map binary/PNG
      // in both saved directories, without silently omitting newly added maps.
      std::set<std::string> mapfiles;
      for (const auto &dir : {result_a.parent_path(), result_b.parent_path()})
        for (const auto &entry : fs::directory_iterator(dir)) {
          const auto name = entry.path().filename().u8string();
          if (name.find(".cells.bin") != std::string::npos || name == "pcl.png" ||
              name == "ai.png" || name == "fusion.png" ||
              (name.find("map") != std::string::npos &&
               (entry.path().extension() == ".bin" || entry.path().extension() == ".png")))
            mapfiles.insert(name);
        }
      for (const auto &name : mapfiles) {
        auto diff = byteDiff(result_a.parent_path() / name, result_b.parent_path() / name);
        same &= field(diff, "equal").boolean;
        artifacts.object[name] = std::move(diff);
      }
      for (const auto *type : {"camera", "robot", "ground", "obstacle", "semantic"}) {
        const auto p = artifact(a, method, i->second, std::string(type) + ".ply"),
                   q = artifact(b, method, j->second, std::string(type) + ".ply");
        if (!fs::exists(p) && !fs::exists(q))
          continue;
        auto diff = plyDiff(p, q, tolerance);
        same &= field(diff, "equal").boolean;
        artifacts.object[std::string(type) + ".ply"] = std::move(diff);
      }
      frame.object["artifacts"] = std::move(artifacts);
      frame.object["equal"] = boolean(same);
      equal &= same;
      frames.array.push_back(std::move(frame));
    }
  }
  if (!paired)
    throw std::runtime_error("No paired measured frames for version check");
  const auto out = fs::absolute(output);
  if (fs::exists(out))
    throw std::runtime_error("Version check output must be new");
  fs::create_directories(out);
  auto report = obj({{"schema_version", integer(3)},
                     {"baseline", str(a.u8string())},
                     {"candidate", str(b.u8string())},
                     {"same_input_identity", boolean(true)},
                     {"identity", id_a},
                     {"paired_frames", integer(paired)},
                     {"equal", boolean(equal)},
                     {"coverage", coverage},
                     {"frames", frames}});
  writeJson(out / "version_diff.json", report);
  std::ostringstream md;
  md << "# 同数据代码版本回归\n\n结果：" << (equal ? "一致" : "存在差异") << "；配对测量帧："
     << paired << "；XYZ容差：" << std::setprecision(17) << tolerance
     << " m。\n\n输入、模型、标定、回放参数身份已一致核对；warmup不比较。检测比较实际detections，ma"
        "sk/"
        "地图比较原文件字节，PLY比较点数、按原顶点顺序的XYZ最大绝对误差与RGB。不会将耗时、运行时间"
        "戳或输出路径变化当作算法差异，也不进行点云配准来掩盖差异。\n\n| 方法 | 捕获帧 | 一致 "
        "|\n|---|---|---|\n";
  for (const auto &frame : frames.array)
    md << '|' << textValue(field(frame, "method")) << '|' << textValue(field(frame, "stem")) << '|'
       << (field(frame, "equal").boolean ? "是" : "否") << "|\n";
  writeText(out / "version_diff.md", md.str());
  return equal;
}
} // namespace perception_compare
