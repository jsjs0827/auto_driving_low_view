#include "ab/config.hpp"
#include <fstream>
namespace {
using Rows = std::map<std::string, ab::Json>;
Rows read(const ab::fs::path &p) {
    Rows out;
    std::ifstream f(p);
    std::string line;
    while (std::getline(f, line)) {
        auto j = ab::j::JsonParser(line, p.u8string()).parse();
        auto key = ab::str(j, "timestamp_ns") + "/" + ab::str(j, "left_digest") + "/" +
                   ab::str(j, "right_digest");
        if (auto k = ab::j::optional(j, "intrinsics"))
            key += "/" + ab::dump(*k);
        if (!out.emplace(key, std::move(j)).second)
            throw std::runtime_error("Repeated input identity in benchmark metrics");
    }
    return out;
}
} // namespace
void writePairing(const ab::fs::path &root) {
    std::ofstream f(root / "pairing.json");
    f << "{\"note\":\"Pairs require original timestamp, identical normalized left/right bytes and "
         "intrinsics. Exact output equality is a regression indicator, not accuracy ground "
         "truth.\",\"comparisons\":[";
    bool first = true;
    for (auto &e : ab::fs::recursive_directory_iterator(root)) {
        if (e.path().filename() != "summary.json")
            continue;
        auto s = ab::j::load(e.path().u8string());
        if (ab::str(s, "method") != "original")
            continue;
        auto baseline = e.path().parent_path();
        auto a = read(baseline / "frames.jsonl");
        for (auto method : {"optimized_q8", "optimized_stereo"}) {
            auto path = baseline.parent_path() / method;
            if (!ab::fs::is_regular_file(path / "summary.json"))
                continue;
            auto b = read(path / "frames.jsonl");
            std::size_t matched = 0, mask_equal = 0, q8_equal = 0, detection_count_equal = 0;
            for (auto &x : a) {
                auto it = b.find(x.first);
                if (it == b.end())
                    continue;
                ++matched;
                auto &l = x.second;
                auto &r = it->second;
                if (ab::str(l, "mask_digest") == ab::str(r, "mask_digest"))
                    ++mask_equal;
                if (ab::str(l, "q8_digest") == ab::str(r, "q8_digest"))
                    ++q8_equal;
                if (ab::num(l, "detection_count") == ab::num(r, "detection_count"))
                    ++detection_count_equal;
            }
            if (!first)
                f << ',';
            first = false;
            f << "{\"source\":" << ab::j::quote(ab::str(s, "source"))
              << ",\"suite\":" << ab::j::quote(ab::str(s, "suite"))
              << ",\"candidate\":" << ab::j::quote(method) << ",\"baseline_frames\":" << a.size()
              << ",\"candidate_frames\":" << b.size() << ",\"same_input_pairs\":" << matched
              << ",\"all_inputs_paired\":"
              << ((matched == a.size() && matched == b.size()) ? "true" : "false")
              << ",\"exact_mask_equal_pairs\":" << mask_equal
              << ",\"exact_q8_equal_pairs\":" << q8_equal
              << ",\"detection_count_equal_pairs\":" << detection_count_equal << "}";
        }
    }
    f << "]}\n";
    if (!f)
        throw std::runtime_error("Pairing report write failed");
}
