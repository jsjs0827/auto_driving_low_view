#include "percep_main/detail/det_depth_gate.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
bool Close(float a, float b) { return std::abs(a - b) < 0.01f; }
}

int main() {
    using perception::detail::SelectWeightedDepthGate;
    {
        // Twenty nearby stray points must not displace an eight-point body
        // at 2 m: z^2 compensation gives weights 20 versus 32.
        std::vector<float> zs(20, 1000.f);
        zs.insert(zs.end(), 8, 2000.f);
        const auto gate = SelectWeightedDepthGate(zs, 600.f, 200.f, 6);
        Check(Close(gate.min_mm, 1970.f) && Close(gate.max_mm, 2030.f),
              "distant sparse body was displaced by near dense samples");
    }
    {
        // 12*1^2 versus 3*2^2 have equal physical support: prefer near.
        std::vector<float> zs(12, 1000.f);
        zs.insert(zs.end(), 3, 2000.f);
        const auto gate = SelectWeightedDepthGate(zs, 600.f, 200.f, 6);
        Check(Close(gate.min_mm, 970.f) && Close(gate.max_mm, 1030.f),
              "equal support did not prefer nearer cluster");
    }
    {
        std::vector<float> zs{4500.f, 1000.f};
        const auto gate = SelectWeightedDepthGate(zs, 600.f, 200.f, 6);
        Check(Close(gate.min_mm, 970.f) && Close(gate.max_mm, 1630.f),
              "sparse candidates removed the finite production depth gate");
        Check(4500.f > gate.max_mm, "sparse depth gate admitted a distant tail");
    }
    {
        std::vector<float> zs{1000.f, 1100.f, 1200.f, 1300.f, 1400.f, 1500.f};
        const auto gate = SelectWeightedDepthGate(zs, 250.f, 150.f, 6);
        Check(Close(gate.min_mm, 970.f) && Close(gate.max_mm, 1280.f),
              "continuous cluster was not clipped to configured span");
    }
    {
        std::vector<float> zs{1000.f, 2000.f, 3000.f, 4000.f, 5000.f, 6000.f};
        const auto gate = SelectWeightedDepthGate(zs, 600.f, 100.f, 6);
        Check(Close(gate.min_mm, 3670.f) && Close(gate.max_mm, 4330.f),
              "fragmented candidates did not use bounded median fallback");
    }
    {
        std::vector<float> zs;
        const auto gate = SelectWeightedDepthGate(zs, 600.f, 200.f, 6);
        Check(gate.min_mm < -1e20f && gate.max_mm > 1e20f,
              "empty candidates should retain default gate");
    }
    std::cout << "fusion depth gate: 6 checks passed\n";
}
