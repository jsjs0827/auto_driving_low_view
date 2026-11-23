#pragma once
#include "legacy_preprocess.hpp"
#include <array>
namespace r20 {
void referenceMask(const std::vector<float>&,int,int,int,int,int,int,int,int,const Geometry&,bool,std::vector<uint8_t>&);
struct PostStats {std::size_t filled_pixels=0,shortcut_pixels=0,interpolated_pixels=0;};
class SegPostWorkspace {
    struct Axis {int a,b;float w;};
    struct Run {int begin,end,a,b;};
    std::array<int,12> key_{};
    bool cached_=false;
    std::vector<Axis> x_,y_;
    std::vector<Run> runs_;
    std::vector<uint8_t> coarse_;
public:
    std::vector<float> logits;
    PostStats stats;
    void run(const std::vector<float>&,int,int,int,int,int,int,int,int,const Geometry&,bool,std::vector<uint8_t>&);
};
SegPostWorkspace& segPostWorkspace();
}
