#include "json_util.hpp"
#include <array>
#include <cstring>

namespace perception_compare::detail {
namespace {
constexpr std::array<std::uint32_t, 64> constants = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
std::uint32_t rotate(std::uint32_t v, int n) { return (v >> n) | (v << (32 - n)); }
struct Hash {
  std::array<std::uint32_t, 8> h = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::array<unsigned char, 64> block{};
  std::size_t used = 0;
  std::uint64_t bytes = 0;
  void compress() {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i)
      w[i] = (std::uint32_t(block[i * 4]) << 24) | (std::uint32_t(block[i * 4 + 1]) << 16) |
             (std::uint32_t(block[i * 4 + 2]) << 8) | block[i * 4 + 3];
    for (int i = 16; i < 64; ++i) {
      auto a = w[i - 15], b = w[i - 2];
      w[i] = w[i - 16] + (rotate(a, 7) ^ rotate(a, 18) ^ (a >> 3)) + w[i - 7] +
             (rotate(b, 17) ^ rotate(b, 19) ^ (b >> 10));
    }
    auto a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], z = h[7];
    for (int i = 0; i < 64; ++i) {
      auto t = z + (rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25)) + ((e & f) ^ (~e & g)) +
               constants[i] + w[i];
      auto u = (rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
      z = g;
      g = f;
      f = e;
      e = d + t;
      d = c;
      c = b;
      b = a;
      a = t + u;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += z;
  }
  void add(const unsigned char *p, std::size_t n) {
    bytes += n;
    while (n) {
      const auto take = std::min(n, 64 - used);
      std::memcpy(block.data() + used, p, take);
      used += take;
      p += take;
      n -= take;
      if (used == 64) {
        compress();
        used = 0;
      }
    }
  }
  std::string finish() {
    const auto bits = bytes * 8;
    block[used++] = 0x80;
    if (used > 56) {
      std::fill(block.begin() + used, block.end(), 0);
      compress();
      used = 0;
    }
    std::fill(block.begin() + used, block.begin() + 56, 0);
    for (int i = 0; i < 8; ++i)
      block[63 - i] = unsigned((bits >> (i * 8)) & 255);
    compress();
    std::ostringstream s;
    s << std::hex << std::setfill('0');
    for (auto v : h)
      s << std::setw(8) << v;
    return s.str();
  }
};
} // namespace
std::string sha256(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("Cannot hash " + path.u8string());
  Hash h;
  std::array<unsigned char, 65536> buffer{};
  while (f) {
    f.read(reinterpret_cast<char *>(buffer.data()), buffer.size());
    h.add(buffer.data(), std::size_t(f.gcount()));
  }
  if (!f.eof())
    throw std::runtime_error("Hash read failed " + path.u8string());
  return h.finish();
}
} // namespace perception_compare::detail
