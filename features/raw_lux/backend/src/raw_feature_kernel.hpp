#pragma once
// Private native RAW integer kernel; RawLuxRunner checks image geometry first.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#ifndef RAW_LUX_KERNEL_NEON
#define RAW_LUX_KERNEL_NEON 1
#endif
#ifndef RAW_LUX_HIST_SHARDS
#define RAW_LUX_HIST_SHARDS 4
#endif
// Byte loads are unaligned-safe. Other architectures/endian use scalar LE decode.
#if RAW_LUX_KERNEL_NEON && defined(__aarch64__) && defined(__ARM_NEON) && \
    defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#include <arm_neon.h>
#define RAW_LUX_NATIVE_NEON 1
#else
#define RAW_LUX_NATIVE_NEON 0
#endif
namespace perception::raw_lux_detail {
constexpr std::size_t kWidth=1088,kHeight=1280,kFrameBytes=kWidth*kHeight*2;
constexpr std::size_t kRoiX=435,kRoiY=128,kRoiWidth=598,kRoiHeight=1024,kLevels=1024;
constexpr std::size_t kShards=RAW_LUX_HIST_SHARDS;
// Shift shard bases by a cache line, avoiding 4 KiB cache-index aliasing.
constexpr std::size_t kShardStride=kLevels+16;
static_assert(kShards==1 || kShards==4 || kShards==8,"Unsupported shard count");
static_assert(kRoiWidth*kRoiHeight==612352,"Training ROI must not change");
static_assert(kRoiWidth*kRoiHeight<UINT32_MAX,"Shard counters must not overflow");
inline std::uint16_t load_le16(const std::uint8_t* p) {
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    std::uint16_t value;std::memcpy(&value,p,sizeof(value));return value;
#else
    return static_cast<std::uint16_t>(std::uint16_t(p[0])|(std::uint16_t(p[1])<<8));
#endif
}
[[noreturn]] inline void invalid_dn() {
    throw std::runtime_error("RAW contains values above1023 outside first uint16");
}
inline void validate_dn(const std::uint8_t* data) {
    std::size_t offset=2; // Embedded first uint16 excluded exactly as in source.
#if RAW_LUX_NATIVE_NEON
    uint8x16_t a=vdupq_n_u8(0),b=a,c=a,d=a;
    for(;offset+64<=kFrameBytes;offset+=64) {
        a=vorrq_u8(a,vld1q_u8(data+offset));
        b=vorrq_u8(b,vld1q_u8(data+offset+16));
        c=vorrq_u8(c,vld1q_u8(data+offset+32));
        d=vorrq_u8(d,vld1q_u8(data+offset+48));
    }
    a=vorrq_u8(vorrq_u8(a,b),vorrq_u8(c,d));
    for(;offset+16<=kFrameBytes;offset+=16)a=vorrq_u8(a,vld1q_u8(data+offset));
    // Only bits 2..7 in a LE word's high byte violate the 10-bit DN domain.
    static constexpr std::uint8_t mask[16]={0,252,0,252,0,252,0,252,0,252,0,252,0,252,0,252};
    if(vmaxvq_u8(vandq_u8(a,vld1q_u8(mask)))!=0)invalid_dn();
#endif
    for(;offset<kFrameBytes;offset+=2)if(load_le16(data+offset)>1023)invalid_dn();
}
inline std::array<std::uint64_t,kLevels> native_counts(const std::uint8_t* data) {
    validate_dn(data); // Entire non-embedded frame, including outside the ROI.
    alignas(64) std::array<std::array<std::uint32_t,kShardStride>,kShards> hist{};
    for(std::size_t y=kRoiY;y<kRoiY+kRoiHeight;++y) {
        const auto* p=data+(y*kWidth+kRoiX)*2;std::size_t x=0;
        for(;x+8<=kRoiWidth;x+=8,p+=16) {
#if RAW_LUX_NATIVE_NEON
            const uint16x8_t v=vreinterpretq_u16_u8(vld1q_u8(p));
            ++hist[0%kShards][vgetq_lane_u16(v,0)];
            ++hist[1%kShards][vgetq_lane_u16(v,1)];
            ++hist[2%kShards][vgetq_lane_u16(v,2)];
            ++hist[3%kShards][vgetq_lane_u16(v,3)];
            ++hist[4%kShards][vgetq_lane_u16(v,4)];
            ++hist[5%kShards][vgetq_lane_u16(v,5)];
            ++hist[6%kShards][vgetq_lane_u16(v,6)];
            ++hist[7%kShards][vgetq_lane_u16(v,7)];
#else
            ++hist[0%kShards][load_le16(p)];
            ++hist[1%kShards][load_le16(p+2)];
            ++hist[2%kShards][load_le16(p+4)];
            ++hist[3%kShards][load_le16(p+6)];
            ++hist[4%kShards][load_le16(p+8)];
            ++hist[5%kShards][load_le16(p+10)];
            ++hist[6%kShards][load_le16(p+12)];
            ++hist[7%kShards][load_le16(p+14)];
#endif
        }
        for(;x<kRoiWidth;++x,p+=2)++hist[x%kShards][load_le16(p)];
    }
    std::array<std::uint64_t,kLevels> counts{};
    for(std::size_t shard=0;shard<kShards;++shard)
        for(std::size_t value=0;value<kLevels;++value)counts[value]+=hist[shard][value];
    return counts;
}
} // namespace perception::raw_lux_detail
