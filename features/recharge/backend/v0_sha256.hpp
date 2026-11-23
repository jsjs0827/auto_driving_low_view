#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace r36_recharge_integrity {
// Self-contained SHA-256, avoiding a target OpenSSL/runtime dependency.
class Sha256 {
    std::array<std::uint32_t,8> h_{{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19}};
    std::array<std::uint8_t,64> pending_{};
    std::size_t used_=0;
    std::uint64_t bytes_=0;
    static std::uint32_t r(std::uint32_t v,unsigned n){return (v>>n)|(v<<(32-n));}
    void block(const std::uint8_t* p){
        static constexpr std::uint32_t k[]={
            0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
            0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
            0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
            0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
            0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
            0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
            0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
            0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
        std::uint32_t w[64]{};
        for(unsigned i=0;i<16;++i)w[i]=(std::uint32_t(p[i*4])<<24)|(std::uint32_t(p[i*4+1])<<16)|(std::uint32_t(p[i*4+2])<<8)|p[i*4+3];
        for(unsigned i=16;i<64;++i)w[i]=w[i-16]+(r(w[i-15],7)^r(w[i-15],18)^(w[i-15]>>3))+w[i-7]+(r(w[i-2],17)^r(w[i-2],19)^(w[i-2]>>10));
        auto a=h_[0],b=h_[1],c=h_[2],d=h_[3],e=h_[4],f=h_[5],g=h_[6],h=h_[7];
        for(unsigned i=0;i<64;++i){
            const auto t1=h+(r(e,6)^r(e,11)^r(e,25))+((e&f)^((~e)&g))+k[i]+w[i];
            const auto t2=(r(a,2)^r(a,13)^r(a,22))+((a&b)^(a&c)^(b&c));
            h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
        }
        h_[0]+=a;h_[1]+=b;h_[2]+=c;h_[3]+=d;h_[4]+=e;h_[5]+=f;h_[6]+=g;h_[7]+=h;
    }
public:
    void update(const void* data,std::size_t length){
        auto* p=static_cast<const std::uint8_t*>(data);bytes_+=length;
        while(length){const auto n=std::min(length,pending_.size()-used_);std::memcpy(pending_.data()+used_,p,n);p+=n;used_+=n;length-=n;
            if(used_==64){block(pending_.data());used_=0;}}
    }
    std::string finish()const{
        auto copy=*this;const auto bits=bytes_*8;const std::uint8_t one=0x80,zero=0;
        copy.update(&one,1);while(copy.used_!=56)copy.update(&zero,1);
        std::uint8_t size[8];for(unsigned i=0;i<8;++i)size[7-i]=std::uint8_t(bits>>(i*8));copy.update(size,8);
        std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto v:copy.h_)out<<std::setw(8)<<v;return out.str();
    }
};
inline std::string sha256(const void* data,std::size_t bytes){Sha256 hash;hash.update(data,bytes);return hash.finish();}
inline std::string fileSha256(const std::string& path){
    std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("Cannot hash file: "+path);
    Sha256 hash;std::array<char,65536> block{};while(in){in.read(block.data(),block.size());hash.update(block.data(),static_cast<std::size_t>(in.gcount()));}
    if(!in.eof())throw std::runtime_error("Failed hashing file: "+path);
    return hash.finish();
}
}
