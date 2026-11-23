#include "exact_input.hpp"
#include "seg_post.hpp"
#include <array>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

int main(int argc, char**) {
    try {
        std::mt19937 random(20261028);
        int count = 0;
        r22::setVariant(r22::Variant::CompactDirect);
        r22::Workspace workspaces[2][2];
        std::vector<uint8_t> destinations[2][2]; // Persistent Tensor storage, as in production.
        const bool quick = argc > 1; // QEMU exercises the same ARM code at smaller sizes.
        const auto shapes = quick ? std::vector<std::array<int,3>>{{160,136,192}} :
            std::vector<std::array<int,3>>{{640,544,640},{640,544,672},{320,240,352}};
        // Exercise both layouts, non-tight row strides, workspace reuse, and
        // geometry changes; compare every prepared Y/UV byte to the scalar path.
        for (const auto shape : shapes) {
            for (bool nv12 : {true,false}) for (int lane = 0; lane < 2; ++lane) {
                auto& ws = workspaces[nv12 ? 0 : 1][lane];
                for (int frame = 0; frame < 2; ++frame) {
                    const int sw = shape[0], sh = shape[1], stride = shape[2], dw = (lane ? 832 : 864)/(quick?4:1), dh = 480/(quick?4:1);
                    std::vector<uint8_t> input(size_t(stride)*sh*3/2);
                    for (auto& v : input) v = static_cast<uint8_t>(random());
                    std::vector<uint8_t> expected;
                    auto& actual = destinations[nv12 ? 0 : 1][lane];
                    actual.resize(size_t(dw)*dh*3/2);
                    r20::Geometry a,b;
                    if (!r20::reference::PreprocessYuv420(input.data(),sw,sh,stride,input.size(),nv12,dw,dh,lane?0:1,&a,expected,127) ||
                        !ws.prepare(input.data(),sw,sh,stride,input.size(),nv12,dw,dh,lane?0:1,b,actual.data(),actual.size()) || expected != actual ||
                        a.x_scale != b.x_scale || a.y_scale != b.y_scale || a.x_shift != b.x_shift || a.y_shift != b.y_shift || a.valid_w != b.valid_w || a.valid_h != b.valid_h)
                        throw std::runtime_error("Exact preprocessing differs from reference");
                    ++count;
                }
            }
        }
        // Compare full masks, including class ties, broad uniform regions,
        // mixed boundaries, align_corners and reuse with different output sizes.
        r20::SegPostWorkspace post;
        const auto class_counts = quick ? std::vector<int>{3} : std::vector<int>{3,9};
        const auto destinations_mask = quick ? std::vector<std::array<int,2>>{{160,136}} :
            std::vector<std::array<int,2>>{{640,544},{320,240}};
        for (int pattern = 0; pattern < 3; ++pattern) for (int classes : class_counts) {
            const int ow=104, oh=60;
            std::vector<float> logits(size_t(classes)*ow*oh);
            for (auto& v : logits) v = pattern == 0 ? 0.f : static_cast<float>(int(random()%200)-100)/17.f;
            if (pattern == 2) for (int c=0;c<classes;++c) for (int y=0;y<oh/2;++y) for (int x=0;x<ow;++x)
                logits[size_t(c)*ow*oh+size_t(y)*ow+x] = c == 1 ? 12.f : -12.f;
            for (const auto dest : destinations_mask) {
                r20::Geometry g;g.valid_w=832;g.valid_h=480;
                std::vector<uint8_t> expected,actual;
                r20::referenceMask(logits,classes,oh,ow,480,832,dest[1],dest[0],0,g,true,expected);
                post.run(logits,classes,oh,ow,480,832,dest[1],dest[0],0,g,true,actual);
                if(expected != actual) throw std::runtime_error("Optimized segmentation mask differs from reference");
                ++count;
            }
        }
        std::cout << "PASS: " << count << " numerical checks; exact input bytes/geometries and full masks match reference.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
