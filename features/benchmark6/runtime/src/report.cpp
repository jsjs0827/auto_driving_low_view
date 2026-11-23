#include "ab/config.hpp"
#include <fstream>
#include <iostream>
void writePairing(const ab::fs::path &);
int main(int argc, char **argv) {
    try {
        if (argc != 2)
            throw std::invalid_argument("ab_report MATRIX_OUTPUT_DIRECTORY");
        ab::fs::path root = ab::fs::absolute(ab::fs::u8path(argv[1]));
        std::ofstream csv(root / "summary.csv"), md(root / "report_zh.md"),
            html(root / "index.html");
        csv << "source,suite,method,frames,process_mean_ms,process_p95_ms,pipeline_fps,process_cpu_"
               "percent,cpu_ms_per_frame,rss_mean_mib,bpu_mean_percent,bpu_p95_percent\n";
        md << "# 三算法实验结果\n\nCPU 100%=一个核；BPU 是设备共享快照。native 的 Stereo 使用 "
              "ROI，Q8 使用全图，不能把全部差额解释成模型变化；matched_roi 才统一点云区域。offline "
              "的原 Q8 已提前生成。ROS 的默认资源范围是订阅算法进程，group_cpu_percent "
              "才包含显式指定的同机发布进程。\n\n| 输入 | 区域策略 | 方法 | 帧数 | 均值 ms | p95 "
              "ms | 实际 FPS | CPU % | CPU ms/帧 | RSS均值 MiB "
              "|\n|---|---|---|---:|---:|---:|---:|---:|---:|---:|\n";
        html << "<!doctype html><meta charset=utf-8><title>三算法对比</title><style>body{font:16px "
                "system-ui;max-width:1300px;margin:40px "
                "auto}table{border-collapse:collapse}td,th{padding:12px;border-bottom:1px solid "
                "#ddd}</style><h1>原检测分割 / 优化检测分割 / 新Stereo</h1><p>全图 native 与统一 "
                "ROI matched_roi 分开比较；各输入的资源范围见 summary.json。逐帧保存目录包含 "
                "camera、semantic、robot、ground、obstacle "
                "PLY。</p><table><tr><th>输入</th><th>区域</th><th>方法</th><th>平均 "
                "ms</th><th>p95 ms</th><th>FPS</th><th>CPU %</th><th>RSS MiB</th></tr>";
        int count = 0;
        for (auto &e : ab::fs::recursive_directory_iterator(root)) {
            if (e.path().filename() != "summary.json")
                continue;
            auto j = ab::j::load(e.path().u8string());
            if (!ab::j::boolean(ab::j::required(j, "complete")))
                continue;
            auto &metrics = ab::j::required(j, "metrics");
            auto &process = ab::j::required(metrics, "process");
            auto src = ab::str(j, "source"), suite = ab::str(j, "suite"),
                 method = ab::str(j, "method");
            double mean = ab::real(process, "mean"), p95 = ab::real(process, "p95"),
                   fps = ab::real(j, "pipeline_fps"), cpu = ab::real(j, "process_cpu_percent"),
                   rss = ab::real(ab::j::required(metrics, "rss_kib"), "mean") / 1024;
            auto &bpu = ab::j::required(j, "bpu");
            auto bp = ab::j::optional(bpu, "mean_pct"), b95 = ab::j::optional(bpu, "p95_pct");
            csv << src << ',' << suite << ',' << method << ',' << ab::num(j, "measured_frames")
                << ',' << mean << ',' << p95 << ',' << fps << ',' << cpu << ','
                << ab::real(j, "cpu_ms_per_frame") << ',' << rss << ','
                << (bp ? ab::dump(*bp) : "null") << ',' << (b95 ? ab::dump(*b95) : "null") << '\n';
            md << '|' << src << '|' << suite << '|' << method << '|'
               << ab::num(j, "measured_frames") << '|' << mean << '|' << p95 << '|' << fps << '|'
               << cpu << '|' << ab::real(j, "cpu_ms_per_frame") << '|' << rss << "|\n";
            html << "<tr><td>" << src << "</td><td>" << suite << "</td><td>" << method
                 << "</td><td>" << mean << "</td><td>" << p95 << "</td><td>" << fps << "</td><td>"
                 << cpu << "</td><td>" << rss << "</td></tr>";
            ++count;
        }
        html << "</table><p>算法阶段细分在各组 summary.json.metrics；输入逐帧摘要在 "
                "frames.jsonl，配对时必须同时验证 timestamp_ns 和左右图 digest。</p>";
        if (!count)
            throw std::runtime_error("No successful experiment summaries");
        writePairing(root);
        std::cout << "Report groups=" << count << " at " << root.u8string() << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
