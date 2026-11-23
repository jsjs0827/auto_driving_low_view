#include "json_util.hpp"
#include "disparity_compare.hpp"
#include "product_graph.hpp"
#include "perception_compare/native_compare.hpp"
#include <numeric>
#include <set>

namespace perception_compare {
using namespace detail;
namespace {
const std::vector<std::string> metrics = {
    "load_ms",       "compute_wall_ms", "sdk_call_ms",     "stereo_ms",  "save_ms",
    "total_wall_ms", "cpu_ms",          "cpu_wall_ms",     "rss_kib",    "peak_rss_kib",
    "graph_ms",      "process_ms",      "mapping_work_ms", "det_pre_ms", "det_infer_ms",
    "det_post_ms",   "seg_pre_ms",      "seg_infer_ms",    "seg_post_ms",
    "stereo_pre_ms", "stereo_backend_ms", "stereo_post_ms", "original_disparity_generation_ms"};
const std::set<std::string> mapping_tasks = {"disparity", "cloud",  "calibration", "labels",
                                             "transform", "ground", "maps"};
struct Row {
  Json data;
  fs::path base;
  std::string stem, key;
  std::int64_t repeat = 0;
  bool warmup = false, success = true;
  std::map<std::string, double> tasks;
};
using Rows = std::vector<Row>;
std::string canonical(std::string method) {
  if (method == "original_q8" || method == "old" || method == "q8")
    return "original";
  if (method == "raft_stereo" || method == "new" || method == "raft")
    return "stereo";
  return method;
}
bool failed(const Json &j) {
  const auto &success = field(j, "success");
  if (success.kind == Json::Kind::Boolean && !success.boolean)
    return true;
  const auto status = textValue(field(j, "status"));
  if (status == "failed" || status == "error")
    return true;
  const auto &code = field(j, "code");
  if (code.kind != Json::Kind::Null && (!numeric(code) || *numeric(code) != 0))
    return true;
  const auto &error = field(j, "error");
  return !(error.kind == Json::Kind::Null ||
           (error.kind == Json::Kind::String && error.text.empty()) ||
           (error.kind == Json::Kind::Object && error.object.empty()));
}
std::int64_t signedInteger(const Json &j, const std::string &name, std::int64_t fallback = 0) {
  if (j.kind == Json::Kind::Null)
    return fallback;
  if (j.kind != Json::Kind::Number)
    throw std::runtime_error(name + " must be integer");
  std::size_t used = 0;
  const auto n = std::stoll(j.text, &used);
  if (used != j.text.size())
    throw std::runtime_error(name + " must be integer");
  return n;
}
fs::path resultPath(const fs::path &root, const fs::path &base, const std::string &name) {
  const auto p = fs::u8path(name);
  auto candidate = fs::weakly_canonical(p.is_absolute() ? p : base / p);
  if (!fs::exists(candidate) && !p.is_absolute())
    candidate = fs::weakly_canonical(root / p);
  if (!contained(root, candidate))
    throw std::runtime_error("Result reference escapes output directory: " + name);
  return candidate;
}
Json trace(const fs::path &root, const Row &row) {
  auto j = field(row.data, "task_trace");
  if (j.kind == Json::Kind::Null)
    j = field(row.data, "task_trace_json");
  if (j.kind == Json::Kind::String)
    j = parse(j.text, "task_trace_json");
  if (j.kind == Json::Kind::Null) {
    auto path = textValue(field(row.data, "task_trace_path"));
    if (!path.empty())
      j = readJson(resultPath(root, row.base, path));
    else if (!textValue(field(row.data, "output_dir")).empty()) {
      const auto p =
          resultPath(root, row.base, textValue(field(row.data, "output_dir"))) / "task_trace.json";
      if (fs::is_regular_file(p))
        j = readJson(p);
    }
  }
  return j.kind == Json::Kind::Object ? j : object();
}
std::map<std::string, Rows> load(const fs::path &root) {
  std::vector<fs::path> files;
  if (fs::is_regular_file(root / "frames.jsonl"))
    files.push_back(root / "frames.jsonl");
  for (const auto &entry : fs::directory_iterator(root))
    if (entry.is_directory() && fs::is_regular_file(entry.path() / "frames.jsonl"))
      files.push_back(entry.path() / "frames.jsonl");
  if (files.empty())
    throw std::runtime_error("No frames.jsonl in " + root.u8string());
  std::sort(files.begin(), files.end());
  std::map<std::string, Rows> methods;
  std::map<std::string, std::set<std::string>> seen;
  for (const auto &path : files) {
    auto mapping_ids = mapping_tasks;
    const auto effective = root / ("full_"+path.parent_path().filename().u8string()+".json");
    if(fs::is_regular_file(effective)) {
      const auto full=readJson(effective);
      if(field(full,"graph").kind==Json::Kind::Object) {
        mapping_ids.clear();
        const auto graph=r33::full::parseProductGraph(field(full,"graph"));
        for(const auto &task:graph.config.tasks)
          if(task.enabled && mapping_tasks.count(graph.stage_for_task.at(task.id)))
            mapping_ids.insert(task.id);
      }
    }
    std::ifstream f(path);
    std::string line;
    std::size_t index = 0;
    while (std::getline(f, line)) {
      ++index;
      if (line.find_first_not_of(" \t\r") == std::string::npos)
        continue;
      try {
        Row row;
        row.data = parse(line, path.u8string() + ":" + std::to_string(index));
        row.base = path.parent_path();
        if (row.data.kind != Json::Kind::Object)
          throw std::runtime_error("Frame row must be object");
        const auto method =
            canonical(textValue(field(row.data, "method"), row.base.filename().u8string()));
        if (method != "original" && method != "stereo" && method != "detseg_only")
          throw std::runtime_error("Unknown method " + method);
        const auto &stem = field(row.data, "stem");
        row.stem = stem.kind == Json::Kind::String ? stem.text : field(row.data, "frame_id").text;
        if (row.stem.empty())
          throw std::runtime_error("Frame needs a nonempty stem/frame_id");
        row.repeat = signedInteger(field(row.data, "repeat"), "repeat");
        row.key = row.stem + '\0' + std::to_string(row.repeat);
        const auto &warm = field(row.data, "warmup");
        if (warm.kind != Json::Kind::Null && warm.kind != Json::Kind::Boolean)
          throw std::runtime_error("warmup must be boolean");
        row.warmup = warm.boolean;
        row.success = !failed(row.data);
        if (!row.warmup && !seen[method].insert(row.key).second)
          throw std::runtime_error("Duplicate measured frame identity");
        const auto t = trace(root, row);
        if (field(row.data, "graph_ms").kind == Json::Kind::Null)
          row.data.object["graph_ms"] = field(t, "graph_ms");
        const auto &tasks = field(t, "tasks");
        if (tasks.kind != Json::Kind::Null && tasks.kind != Json::Kind::Array)
          throw std::runtime_error("task_trace.tasks must be array");
        double work = 0;
        bool has_work = false;
        for (const auto &task : tasks.array) {
          const auto name = textValue(field(task, "task"),
                                      textValue(field(task, "id"), textValue(field(task, "name"))));
          auto value = numeric(field(task, "ms"));
          if (!value)
            value = numeric(field(task, "elapsed_ms"));
          if (!value)
            value = numeric(field(task, "duration_ms"));
          const auto state = textValue(field(task, "state"), "succeeded");
          if (value && (state == "succeeded" || state == "success" || state == "completed" ||
                        state == "complete" || state == "ok")) {
            if (!row.tasks.emplace(name, *value).second)
              throw std::runtime_error("Duplicate SDK trace task " + name);
            if (mapping_ids.count(name)) {
              work += *value;
              has_work = true;
            }
          }
        }
        row.data.object["mapping_work_ms"] = has_work ? number(work) : Json{};
        auto timings = field(row.data, "inference_ms");
        if (timings.kind != Json::Kind::Object)
          timings = field(field(row.data, "result"), "inference_ms");
        for (const auto *name :
             {"det_pre", "det_infer", "det_post", "seg_pre", "seg_infer", "seg_post"})
          if (field(row.data, std::string(name) + "_ms").kind == Json::Kind::Null)
            row.data.object[std::string(name) + "_ms"] = field(timings, name);
        if (field(row.data, "cpu_wall_ms").kind == Json::Kind::Null)
          row.data.object["cpu_wall_ms"] =
              field(row.data, textValue(field(row.data, "cpu_scope"), "compute") == "compute"
                                  ? "compute_wall_ms"
                                  : "total_wall_ms");
        methods[method].push_back(std::move(row));
      } catch (const std::exception &e) {
        throw std::runtime_error(path.u8string() + ":" + std::to_string(index) + ": " + e.what());
      }
    }
  }
  return methods;
}
Json describe(std::vector<double> values, std::size_t expected) {
  std::sort(values.begin(), values.end());
  auto result = obj({{"count", integer(values.size())},
                     {"missing", integer(expected - values.size())},
                     {"mean", Json{}},
                     {"p50", Json{}},
                     {"p95", Json{}},
                     {"min", Json{}},
                     {"max", Json{}}});
  if (values.empty())
    return result;
  auto percentile = [&](double q) {
    const double at = (values.size() - 1) * q;
    const auto i = std::size_t(at);
    return values[i] * (1 - (at - i)) + values[std::min(i + 1, values.size() - 1)] * (at - i);
  };
  result.object["mean"] =
      number(std::accumulate(values.begin(), values.end(), 0.0) / values.size());
  result.object["p50"] = number(percentile(.5));
  result.object["p95"] = number(percentile(.95));
  result.object["min"] = number(values.front());
  result.object["max"] = number(values.back());
  return result;
}
Json summarize(const Rows &rows, const Json &complete) {
  std::vector<const Row *> accepted;
  std::set<std::string> stems, task_names;
  std::size_t warmup = 0, failures = 0;
  auto statistics = object();
  for (const auto &row : rows) {
    if (row.warmup) {
      ++warmup;
      continue;
    }
    if (!row.success) {
      ++failures;
      continue;
    }
    accepted.push_back(&row);
    stems.insert(row.stem);
    for (const auto &task : row.tasks)
      task_names.insert(task.first);
  }
  for (const auto &metric : metrics) {
    std::vector<double> values;
    for (const auto *row : accepted)
      if (auto n = numeric(field(row->data, metric)))
        values.push_back(*n);
    statistics.object[metric] = describe(values, accepted.size());
  }
  double cpu = 0, wall = 0, machine_cpu = 0, machine_wall = 0, compute = 0, total = 0;
  std::size_t cpu_count = 0, compute_count = 0, total_count = 0, machine_count = 0;
  for (const auto *row : accepted) {
    const auto c = numeric(field(row->data, "cpu_ms")),
               w = numeric(field(row->data, "cpu_wall_ms")),
               cores = numeric(field(row->data, "online_cpus"));
    if (c && w && *w > 0) {
      cpu += *c;
      wall += *w;
      ++cpu_count;
      if (cores && *cores > 0) {
        machine_cpu += *c / *cores;
        machine_wall += *w;
        ++machine_count;
      }
    }
    if (auto n = numeric(field(row->data, "compute_wall_ms")); n && *n > 0) {
      compute += *n;
      ++compute_count;
    }
    if (auto n = numeric(field(row->data, "total_wall_ms")); n && *n > 0) {
      total += *n;
      ++total_count;
    }
  }
  auto tasks = object();
  for (const auto &name : task_names) {
    std::vector<double> values;
    for (const auto *row : accepted) {
      const auto i = row->tasks.find(name);
      if (i != row->tasks.end())
        values.push_back(i->second);
    }
    tasks.object[name] = describe(values, accepted.size());
  }
  auto result = obj({{"input_rows", integer(rows.size())},
                     {"measured_frames", integer(accepted.size())},
                     {"warmup_frames", integer(warmup)},
                     {"failed_frames", integer(failures)},
                     {"unique_stems", integer(stems.size())},
                     {"compute_fps", compute > 0 ? number(compute_count * 1000 / compute) : Json{}},
                     {"compute_fps_frames", integer(compute_count)},
                     {"compute_sum_ms", number(compute)},
                     {"total_fps", total > 0 ? number(total_count * 1000 / total) : Json{}},
                     {"total_fps_frames", integer(total_count)},
                     {"total_sum_ms", number(total)},
                     {"cpu_percent", wall > 0 ? number(cpu * 100 / wall) : Json{}},
                     {"cpu_samples", integer(cpu_count)},
                     {"cpu_sum_ms", number(cpu)},
                     {"cpu_wall_sum_ms", number(wall)},
                     {"cpu_machine_percent",
                      machine_wall > 0 ? number(machine_cpu * 100 / machine_wall) : Json{}},
                     {"cpu_machine_samples", integer(machine_count)},
                     {"rss_max_kib", field(field(statistics, "rss_kib"), "max")},
                     {"rss_p95_kib", field(field(statistics, "rss_kib"), "p95")},
                     {"peak_rss_max_kib", field(field(statistics, "peak_rss_kib"), "max")},
                     {"metrics", statistics},
                     {"tasks_ms", tasks},
                     {"observed_replay_fps", Json{}},
                     {"observed_replay_wall_ms", Json{}},
                     {"observed_replay_frames", integer(0)},
                     {"observed_replay_cpu_ms", Json{}},
                     {"observed_replay_cpu_percent", Json{}},
                     {"observed_replay_cpu_machine_percent", Json{}}});
  // Group-level device snapshots are independent of the paired frame cohort.
  // Never average per-frame copies of the latest BPU observation.
  result.object["bpu"] = field(complete,"bpu");
  result.object["bpu_cohort_scope"] = str("whole_method_replay_not_restricted_to_paired_frames");
  const auto frames = numeric(field(complete, "measured_frames")),
             elapsed = numeric(field(complete, "measured_wall_ms")),
             measured_cpu = numeric(field(complete, "measured_cpu_ms")),
             cores = numeric(field(complete, "online_cpus"));
  if ((field(complete, "complete").boolean || field(complete, "completed").boolean) && frames &&
      *frames == accepted.size() && *frames > 0 && elapsed && *elapsed > 0) {
    result.object["observed_replay_fps"] = number(*frames * 1000 / *elapsed);
    result.object["observed_replay_wall_ms"] = number(*elapsed);
    result.object["observed_replay_frames"] = number(*frames);
    result.object["observed_replay_cpu_ms"] = measured_cpu ? number(*measured_cpu) : Json{};
    if (measured_cpu) {
      result.object["observed_replay_cpu_percent"] = number(*measured_cpu * 100 / *elapsed);
      if (cores && *cores > 0)
        result.object["observed_replay_cpu_machine_percent"] =
            number(*measured_cpu * 100 / *elapsed / *cores);
    }
  }
  return result;
}
std::string csv(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"')
      out += '"';
    out += c;
  }
  return out + '"';
}
std::string display(const Json &j) {
  const auto n = numeric(j);
  if (!n)
    return "—";
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << *n;
  return out.str();
}
} // namespace

void generateReport(const fs::path &output, std::size_t max_points) {
  if (!max_points || max_points > 200000)
    throw std::runtime_error("max-points must be 1..200000");
  const auto root = fs::canonical(output);
  const auto methods = load(root);
  Json metadata = fs::is_regular_file(root / "run.json") ? readJson(root / "run.json") : object();
  std::set<std::string> common;
  bool first = true;
  for (const auto &[name, rows] : methods) {
    (void)name;
    std::set<std::string> keys;
    for (const auto &row : rows)
      if (!row.warmup && row.success)
        keys.insert(row.key);
    if (first) {
      common = std::move(keys);
      first = false;
    } else {
      std::set<std::string> intersection;
      std::set_intersection(common.begin(), common.end(), keys.begin(), keys.end(),
                            std::inserter(intersection, intersection.end()));
      common = std::move(intersection);
    }
  }
  auto all_stats = object(), paired_stats = object(), unmatched = object(), warnings = array(),
       pairs = array();
  std::map<std::string, Json> previews;
  for (const auto &[name, rows] : methods) {
    Rows paired;
    std::size_t rejected = 0;
    const auto complete = fs::is_regular_file(root / name / "complete.json")
                              ? readJson(root / name / "complete.json")
                              : Json{};
    for (const auto &row : rows) {
      if (row.warmup || !row.success)
        continue;
      if (common.count(row.key))
        paired.push_back(row);
      else
        ++rejected;
      auto &preview = previews[row.key];
      if (preview.kind == Json::Kind::Null)
        preview = obj({{"stem", str(row.stem)},
                       {"repeat", number(double(row.repeat))},
                       {"clouds", object()}});
      auto clouds = field(row.data, "clouds");
      if (clouds.kind != Json::Kind::Object)
        clouds = object();
      const auto directory = textValue(field(row.data, "output_dir"));
      if (!directory.empty())
        for (const auto *type : {"camera", "robot", "ground", "obstacle", "semantic"}) {
          const auto p = resultPath(root, row.base, directory) / (std::string(type) + ".ply");
          if (field(clouds, type).kind == Json::Kind::Null && fs::is_regular_file(p))
            clouds.object[type] = str(p.u8string());
        }
      for (const auto &[type, path] : clouds.object) {
        const auto text = textValue(path);
        if (text.empty())
          continue;
        auto &entry = preview.object["clouds"].object[type];
        if (entry.kind == Json::Kind::Null)
          entry = object();
        entry.object[name] = str(resultPath(root, row.base, text).u8string());
      }
    }
    all_stats.object[name] = summarize(rows, complete);
    paired_stats.object[name] = summarize(paired, complete);
    unmatched.object[name] = integer(rejected);
    if (rejected)
      warnings.array.push_back(
          str(name + ": 存在未配对测量帧；配对表仅比较同 stem/repeat 的成功帧。"));
  }
  for (auto &[key, pair] : previews) {
    (void)key;
    if (!field(pair, "clouds").object.empty())
      pairs.array.push_back(std::move(pair));
  }
  if (methods.count("original"))
    warnings.array.push_back(str("Original 使用录制 Q8，未计入旧视差生成耗时/CPU；新 Stereo "
                                 "包含模型推理，两者不是完整相机链路端到端对比。"));
  if (common.empty())
    warnings.array.push_back(str("没有共同成功测量帧，配对性能表为空。"));
  const auto evidence = textValue(field(metadata, "evidence"), "unknown");
  if (evidence == "synthetic_validation")
    warnings.array.push_back(str("合成测试数据仅验证功能，不能作为 X5 性能或算法质量证据。"));
  if (evidence == "unknown")
    warnings.array.push_back(str("未声明数据来源；不能据此声称 X5 实测或模型精度。"));
  const auto disparity_comparison=generateDisparityComparison(root,metadata);
  if(textValue(field(disparity_comparison,"status"))!="available")
    warnings.array.push_back(str("视差质量报告："+textValue(field(disparity_comparison,"message"))));
  auto summary =
      obj({{"schema_version", integer(3)},
           {"evidence", str(evidence)},
           {"evidence_label", str(evidence + "；平台/模型/标定以 run.json 为准。")},
           {"run_metadata", metadata},
           {"disparity_comparison", disparity_comparison},
           {"paired_frame_count", integer(common.size())},
           {"unmatched_frames", unmatched},
           {"methods", all_stats},
           {"paired_methods", paired_stats},
           {"warnings", warnings},
           {"definitions",
            obj({{"fps", str("valid_frames*1000/sum(actual_wall_ms)")},
                 {"cpu", str("sum(process_cpu_ms)*100/sum(matching_wall_ms); 100%=one core")},
                 {"quality", str("same-ROI disparity difference on valid intersection; original Q8 is reference, not ground truth; no accuracy ranking")}})}});
  auto capture = obj({{"metrics", object()},
                      {"whole_recording", field(metadata, "capture_summary")},
                      {"selected_capture_frames", integer(0)},
                      {"acquisition_fps", Json{}},
                      {"acquisition_cpu_percent", Json{}},
                      {"acquisition_cpu_machine_percent", Json{}}});
  if (fs::is_regular_file(root / "input_inventory.json")) {
    const auto inventory = readJson(root / "input_inventory.json");
    capture.object["selected_capture_frames"] = integer(inventory.array.size());
    double acquire_sum = 0, cpu_sum = 0, cpu_wall = 0, machine_cpu = 0, machine_wall = 0;
    std::size_t acquire_count = 0;
    for (const auto *metric : {"capture_acquire_ms", "capture_cpu_ms", "capture_rss_kib",
                               "capture_peak_rss_kib", "write_ms", "total_wall_ms"}) {
      std::vector<double> values;
      for (const auto &item : inventory.array)
        if (auto n = numeric(field(item, metric)))
          values.push_back(*n);
      capture.object["metrics"].object[metric] = describe(values, inventory.array.size());
    }
    for (const auto &item : inventory.array) {
      const auto wall = numeric(field(item, "capture_acquire_ms")),
                 cpu = numeric(field(item, "capture_cpu_ms")),
                 cores = numeric(field(item, "online_cpus"));
      if (wall && *wall > 0) {
        acquire_sum += *wall;
        ++acquire_count;
        if (cpu) {
          cpu_sum += *cpu;
          cpu_wall += *wall;
          if (cores && *cores > 0) {
            machine_cpu += *cpu / *cores;
            machine_wall += *wall;
          }
        }
      }
    }
    if (acquire_sum > 0)
      capture.object["acquisition_fps"] = number(acquire_count * 1000 / acquire_sum);
    if (cpu_wall > 0)
      capture.object["acquisition_cpu_percent"] = number(cpu_sum * 100 / cpu_wall);
    if (machine_wall > 0)
      capture.object["acquisition_cpu_machine_percent"] = number(machine_cpu * 100 / machine_wall);
  }
  summary.object["capture_source"] = capture;
  summary.object["stereo_input_adaptation"] =
      obj({{"requested", field(metadata, "stereo_input_requested")},
           {"display_rows", array()},
           {"note", str("模型与 source 的横向比例用于视差单位恢复，原左图/内参不变。")}});
  summary.object["viewer"] = buildViewer(root, pairs, summary, max_points);
  writeJson(root / "summary.json", summary);
  std::ostringstream table;
  table << "cohort,method,metric,count,missing,mean,p50,p95,min,max\n";
  for (const auto &[cohort, cohorts] :
       std::vector<std::pair<std::string, Json>>{{"all", all_stats}, {"paired", paired_stats}})
    for (const auto &[name, stats] : cohorts.object) {
      for (const auto &[metric, values] : field(stats, "metrics").object) {
        table << csv(cohort) << ',' << csv(name) << ',' << csv(metric);
        for (const auto *key : {"count", "missing", "mean", "p50", "p95", "min", "max"})
          table << ','
                << (field(values, key).kind == Json::Kind::Number ? field(values, key).text : "");
        table << '\n';
      }
      for (const auto *metric :
           {"compute_fps", "total_fps", "cpu_percent", "cpu_machine_percent", "observed_replay_fps",
            "observed_replay_cpu_percent", "observed_replay_cpu_machine_percent"})
        table << csv(cohort) << ',' << csv(name) << ',' << csv(metric) << ",,,"
              << (field(stats, metric).kind == Json::Kind::Number ? field(stats, metric).text : "")
              << ",,,,\n";
      const auto &bpu = field(stats,"bpu");
      for (const auto *metric : {"mean_pct", "p95_pct", "max_pct"})
        table << csv(cohort) << ',' << csv(name) << ',' << csv(std::string("bpu_device_")+metric)
              << ",,," << (field(bpu,metric).kind==Json::Kind::Number ? field(bpu,metric).text : "") << ",,,,\n";
    }
  writeText(root / "summary.csv", table.str());
  std::ostringstream bpu_csv;
  bpu_csv << "method,status,scope,sample_count,valid_sample_count,missing_sample_count,primary_path,mean_pct,p95_pct,max_pct\n";
  for(const auto &[name,stats]:all_stats.object) {
    const auto &bpu=field(stats,"bpu");
    bpu_csv << csv(name) << ',' << csv(textValue(field(bpu,"status"),"not_recorded")) << ',' << csv(textValue(field(bpu,"scope"),"device_shared"));
    for(const auto *key:{"sample_count","valid_sample_count","missing_sample_count"}) bpu_csv << ',' << (field(bpu,key).kind==Json::Kind::Number?field(bpu,key).text:"");
    bpu_csv << ',' << csv(textValue(field(bpu,"primary_path")));
    for(const auto *key:{"mean_pct","p95_pct","max_pct"}) bpu_csv << ',' << (field(bpu,key).kind==Json::Kind::Number?field(bpu,key).text:"");
    bpu_csv << '\n';
  }
  writeText(root/"bpu_summary.csv",bpu_csv.str());
  std::ostringstream markdown;
  markdown << "# 检测分割 / 原Q8融合 / 新Stereo融合同帧对比\n\n数据来源：" << evidence << "。共同成功帧："
           << common.size()
           << "。\n\n| 方法 | 帧数 | compute FPS | 实际回放 FPS | 进程 CPU % | RSS p95 KiB | BPU设备均值 % | BPU设备p95 % "
              "|\n|---|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const auto &[name, stats] : paired_stats.object)
    markdown << '|' << name << '|' << display(field(stats, "measured_frames")) << '|'
             << display(field(stats, "compute_fps")) << '|'
             << display(field(stats, "observed_replay_fps")) << '|'
             << display(field(stats, "cpu_percent")) << '|' << display(field(stats, "rss_p95_kib"))
             << '|' << display(field(field(stats,"bpu"),"mean_pct")) << '|'
             << display(field(field(stats,"bpu"),"p95_pct")) << "|\n";
  markdown << "\n| 方法 | BPU采样状态 | 有效样本 / 总样本 | 主路径 |\n|---|---|---:|---|\n";
  for(const auto &[name,stats]:all_stats.object) {
    const auto &bpu=field(stats,"bpu");
    markdown << '|' << name << '|' << textValue(field(bpu,"status"),"not_recorded") << '|'
      << display(field(bpu,"valid_sample_count")) << " / " << display(field(bpu,"sample_count")) << '|'
      << textValue(field(bpu,"primary_path"),"—") << "|\n";
  }
  markdown << "\n| 阶段（配对样本） | 方法 | n | 均值 ms | p50 ms | p95 ms "
              "|\n|---|---|---:|---:|---:|---:|\n";
  for (const auto &[name, stats] : paired_stats.object)
    for (const auto &[metric, values] : field(stats, "metrics").object)
      markdown << '|' << metric << '|' << name << '|' << display(field(values, "count")) << '|'
               << display(field(values, "mean")) << '|' << display(field(values, "p50")) << '|'
               << display(field(values, "p95")) << "|\n";
  markdown << "\n口径：warmup/失败帧不进入统计；缺失值保持为空。CPU 按累计 "
              "CPU/同范围墙钟计算，单核满载为100%，可超过100%；BPU为驱动device_shared快照，"
              "缺失/不可解析为null，无法归因到单模型；驱动自身统计窗口不重置，可能覆盖warmup。compute "
              "BPU窗口为整组回放（含加载/保存/日志），配对表中的BPU也是整组观测，不按共同帧重算。"
              "FPS排除加载/"
              "保存；实际回放FPS仅使用匹配完整样本的runner完整测量循环，包含日志与结果释放。det/"
              "seg、SDK与Stereo阶段可能重叠，不能相加当作关键路径。mapping_work_"
              "ms是成功mapping任务工作时间之和，存在并行重叠。RSS为逐帧采样，peak "
              "RSS包含初始化/warmup。点云完整XYZ/"
              "RGB保持不变，两侧同视角、同完整范围，只抽样HTML预览；点数多不自动表示效果好。\n\n";
  for (const auto &warning : warnings.array)
    markdown << "- " << textValue(warning) << '\n';
  markdown << "\n同 ROI 视差差异：[交互查看](disparity_compare.html) · [JSON](disparity_comparison.json) · [CSV](disparity_comparison.csv)。\n\n"
           << "状态：" << textValue(field(disparity_comparison,"status")) << "；分析保存配对："
           << field(disparity_comparison,"analyzed_pair_count").text << " / "
           << field(disparity_comparison,"saved_pair_count").text
           << "。原 Q8 仅 reference，不是 GT；只有双方有效交集进入差异统计。质量分析全部位于 runner 计时窗口之后。\n";
  writeText(root / "report_zh.md", markdown.str());
}
} // namespace perception_compare
