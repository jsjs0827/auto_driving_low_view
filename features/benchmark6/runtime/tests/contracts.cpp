#include "ab/config.hpp"
#include <iostream>
void require(bool b, const char *s) {
    if (!b)
        throw std::runtime_error(s);
}
int main() {
    try {
        ab::Config c;
        c.width = 640;
        c.height = 544;
        c.roi = {0, 192, 640, 352};
        require(ab::region(c, "original", "native").h == 544, "native original lost full image");
        require(ab::region(c, "optimized_q8", "native").h == 544,
                "native optimized Q8 lost full image");
        require(ab::region(c, "original", "matched_roi").y == 192, "matched baseline did not crop");
        require(ab::region(c, "optimized_stereo", "native").h == 352, "Stereo must keep model ROI");
        ab::Intrinsics k{640, 544, 288, 288, 313, 273, 17278};
        ab::validate(k);
        k.bxf = 0;
        bool failed = false;
        try {
            ab::validate(k);
        } catch (...) {
            failed = true;
        }
        require(failed, "zero Bxf accepted");
        require(std::abs(ab::quantile({1, 2, 3, 4}, .95) - 3.85) < 1e-12, "p95 interpolation");
        failed = false;
        try {
            ab::j::JsonParser("{\"x\":1,\"x\":2}", "test").parse();
        } catch (...) {
            failed = true;
        }
        require(failed, "duplicate JSON key accepted");
        std::cout << "Configuration, ROI, calibration and statistics contracts passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
