#include "ab/config.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
void writePairing(const ab::fs::path &);
int main() {
    try {
        auto root = ab::fs::temp_directory_path() /
                    ("ab-pairing-test-" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        for (auto method : {"original", "optimized_q8"}) {
            auto dir = root / "native" / method;
            ab::fs::create_directories(dir);
            std::ofstream summary(dir / "summary.json");
            summary << "{\"method\":" << ab::j::quote(method)
                    << ",\"source\":\"offline\",\"suite\":\"native\"}";
            std::ofstream frames(dir / "frames.jsonl");
            for (int i = 0; i < 3; ++i) {
                bool different = std::string(method) == "optimized_q8" && i == 2;
                frames << "{\"timestamp_ns\":\"" << i + 1 << "\",\"left_digest\":\""
                       << (different ? "different" : "same")
                       << "\",\"right_digest\":\"right\",\"mask_digest\":\"mask\",\"q8_digest\":"
                          "\"q8\",\"detection_count\":1,\"intrinsics\":[640,544,288,288,313,273,"
                          "17280]}\n";
            }
        }
        writePairing(root);
        auto d = ab::j::load((root / "pairing.json").u8string());
        auto &a = ab::j::required(d, "comparisons").array;
        if (a.size() != 1 || ab::num(a[0], "same_input_pairs") != 2 ||
            ab::j::boolean(ab::j::required(a[0], "all_inputs_paired")))
            throw std::runtime_error("Mismatched image content incorrectly paired");
        ab::fs::remove_all(root);
        std::cout << "Same stamp with different image bytes is not paired\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
