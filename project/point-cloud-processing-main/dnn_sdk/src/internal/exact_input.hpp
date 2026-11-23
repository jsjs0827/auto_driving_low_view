#pragma once
#include "legacy_preprocess.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace r22 {
enum class Variant { R21, Cached, Direct, CompactCached, CompactDirect };
bool compactVariant(Variant) noexcept;
bool directVariant(Variant) noexcept;
void setProfiling(bool) noexcept;
bool profiling() noexcept;
Variant variant() noexcept;
void setVariant(Variant) noexcept;
Variant parseVariant(const std::string&);
const char* variantName(Variant) noexcept;
std::uint64_t originalSourceContext() noexcept;
void setOriginalSourceContext(std::uint64_t) noexcept;
class OriginalSourceScope {
    std::uint64_t previous_;
public:
    OriginalSourceScope(bool nv12,int stride):previous_(originalSourceContext()) {
        setOriginalSourceContext((std::uint64_t(stride)<<2)|2|(nv12?1:0));
    }
    ~OriginalSourceScope(){setOriginalSourceContext(previous_);}
};
class VariantScope {
    Variant previous_;
public:
    explicit VariantScope(Variant v):previous_(variant()){setVariant(v);}
    ~VariantScope(){setVariant(previous_);}
};
struct InputStats {
    double prepare_ms=0,transfer_ms=0;
    double pack_ms=0,calibration_ms=0,map_ms=0,padding_ms=0,y_ms=0,uv_ms=0,tail_ms=0,account_ms=0;
    double copy_ms=0,clean_ms=0,audit_ms=0;
    std::size_t block_bytes=0,tail_bytes=0;
    std::size_t padding_bytes=0,copied_bytes=0,table_blocks=0,gather_blocks=0;
    bool initialized=false,direct=false;
    int math_mode=-1;
    std::array<int,3> math_modes{{-1,-1,-1}};
};
struct View {const std::uint8_t* data=nullptr;std::size_t bytes=0;bool direct=false;};
class Workspace {
    struct Axis {int a,b;float w,iw;};
    struct Block {
        std::array<int,8> a{},b{};
        std::array<std::uint8_t,8> ia{},ib{};
        std::array<float,8> w{},iw{};
        int base=0;
        int table_bytes=0;
    };
    // Dense table blocks do not carry 16 full-width fallback positions.
    // a/b are packed into one 16-byte TBL index, so each row needs one TBL.
    struct alignas(16) CompactBlock {
        std::array<std::uint8_t,16> indices{};
        std::array<float,8> w{},iw{};
        int base=0,table_bytes=0,gather=-1;
    };
    struct Gather {std::array<std::uint16_t,16> positions{};};
    static_assert(sizeof(Block)==152 && sizeof(CompactBlock)==96 && sizeof(Gather)==32,
                  "Documented block working-set sizes must match the target ABI");
    r20::PackWorkspace pack_;
    r20::DetectorWorkspace oracle_;
    std::vector<Axis> lx_,ly_,cx_,cy_;
    std::vector<Block> lb_,cb_;
    std::vector<CompactBlock> clb_,ccb_;
    std::vector<Gather> lgb_,cgb_;
    std::size_t ltable_=0,ctable_=0;
    std::array<int,5> map_key_{};
    bool mapped_=false,padding_valid_=false;
    std::uint8_t* last_destination_=nullptr;
    std::array<int,6> padding_key_{};
    int math_=-1;
    std::array<int,3> math_modes_{{-1,-1,-1}};
    r20::ReferencePreprocess reference_=r20::reference::PreprocessYuv420;
    std::uint64_t reference_context_=0;
    void maps(int,int,int,int);
    template<int XM,int YM,int Channels>
    void plane(const std::uint8_t*,int,const std::vector<Axis>&,const std::vector<Axis>&,
               const std::vector<Block>&,std::uint8_t*,int,int,int,int=-1);
    template<int XM,int YM,int Channels>
    void compactPlane(const std::uint8_t*,int,const std::vector<Axis>&,const std::vector<Axis>&,
                      const std::vector<CompactBlock>&,const std::vector<Gather>&,
                      std::uint8_t*,int,int,int,int=-1);
    template<int Channels>
    void dispatchPlane(int,const std::uint8_t*,int,const std::vector<Axis>&,const std::vector<Axis>&,
                       const std::vector<Block>&,std::uint8_t*,int,int,int,int=-1);
    void render(const std::uint8_t*,int,int,int,int,const r20::Geometry&,std::uint8_t*);
public:
    std::vector<std::uint8_t> output;
    InputStats stats;
    bool prepare(const std::uint8_t*,int,int,int,std::size_t,bool,int,int,int,
                 r20::Geometry&,std::uint8_t* destination=nullptr,std::size_t capacity=0);
    void invalidateDestination() noexcept {padding_valid_=false;}
    int mathMode() const noexcept{return math_;}
    std::array<int,3> mathModes() const noexcept{return math_modes_;}
    void setReference(r20::ReferencePreprocess callback,std::uint64_t context=0) {
        if(!callback)callback=r20::reference::PreprocessYuv420;
        if(reference_!=callback||reference_context_!=context){reference_=callback;reference_context_=context;oracle_.setReference(callback,context);mapped_=false;padding_valid_=false;}
    }
};
Workspace& workspace(int lane);
InputStats& stats(int lane);
void resetStats();
void invalidateDestination(int lane);
void captureInput(const char*,View,const void*,std::size_t,int,int,const r20::Geometry&);
// Linked only by the real runner or the explicitly marked lifecycle harness.
r20::ReferencePreprocess runnerReferencePreprocessor();
}
