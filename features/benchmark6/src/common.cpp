#include "benchmark6.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
namespace b6 {
std::uint64_t identity(const std::string &s) {
  if (s.empty() || s.find_first_not_of("0123456789") != std::string::npos)
    throw std::invalid_argument("Capture identity must be an unsigned decimal string");
  std::size_t used = 0; auto n = std::stoull(s, &used);
  if (used != s.size()) throw std::invalid_argument("Capture identity overflow");
  return n;
}
std::string digest(const void *data, std::size_t n) {
  const auto *p = static_cast<const std::uint8_t *>(data);
  std::uint64_t h = 14695981039346656037ULL;
  for (std::size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
  std::ostringstream o; o << std::hex << std::setfill('0') << std::setw(16) << h;
  return o.str();
}
std::vector<std::uint8_t> readBytes(const fs::path &path, std::size_t expected) {
  if (!fs::is_regular_file(path)) throw std::invalid_argument("Missing data file: " + path.u8string());
  const auto n = fs::file_size(path);
  if ((expected && n != expected) || n > 512ULL * 1024 * 1024)
    throw std::invalid_argument("Wrong data byte count: " + path.u8string());
  std::vector<std::uint8_t> out(static_cast<std::size_t>(n));
  std::ifstream f(path, std::ios::binary);
  f.read(reinterpret_cast<char *>(out.data()), std::streamsize(out.size()));
  if (!f) throw std::runtime_error("Read failed: " + path.u8string());
  return out;
}
std::vector<std::uint16_t> readQ8(const fs::path &path, std::size_t pixels) {
  const auto bytes = readBytes(path, pixels * 2);
  std::vector<std::uint16_t> out(pixels);
  for (std::size_t i = 0; i < pixels; ++i) out[i] = bytes[i*2] | (std::uint16_t(bytes[i*2+1]) << 8);
  return out;
}
void writeBytes(const fs::path &path, const void *data, std::size_t size) {
  std::ofstream f(path, std::ios::binary); f.write(static_cast<const char *>(data), std::streamsize(size));
  f.close(); if (!f) throw std::runtime_error("Write failed: " + path.u8string());
}
void writeQ8(const fs::path &path, const std::uint16_t *data, std::size_t pixels) {
  std::vector<std::uint8_t> bytes(pixels * 2);
  for (std::size_t i = 0; i < pixels; ++i) { bytes[i*2] = std::uint8_t(data[i]); bytes[i*2+1] = std::uint8_t(data[i] >> 8); }
  writeBytes(path, bytes.data(), bytes.size());
}
Json stats(std::vector<double> v) {
  auto j = object(); j.object["count"] = num(double(v.size()));
  if (v.empty()) { for (const auto *k : {"mean", "p50", "p95", "max", "min"}) j.object[k] = Json{}; return j; }
  for (double x : v) if (!std::isfinite(x)) throw std::invalid_argument("Nonfinite metric sample");
  std::sort(v.begin(), v.end());
  auto q = [&](double p) { double x = p * (v.size()-1); auto i = std::size_t(x); return v[i] + (v[std::min(i+1,v.size()-1)]-v[i])*(x-i); };
  j.object["mean"] = num(std::accumulate(v.begin(),v.end(),0.0)/v.size());
  j.object["p50"] = num(q(.5)); j.object["p95"] = num(q(.95));
  j.object["min"] = num(v.front()); j.object["max"] = num(v.back()); return j;
}
Json boxJson(const Box &b) {
  auto j=object();j.object["class_id"]=integer(b.label);j.object["score"]=num(b.score);
  j.object["x1"]=num(b.x1);j.object["y1"]=num(b.y1);j.object["x2"]=num(b.x2);j.object["y2"]=num(b.y2);return j;
}
std::vector<Box> boxes(const Json &j) {
  if(j.kind!=Json::Kind::Array)throw std::invalid_argument("detections must be an array");
  std::vector<Box> out;
  for(const auto &v:j.array) {
    keys(v,{"class_id","score","x1","y1","x2","y2"},"detection");Box b;
    b.label=count(get(v,"class_id"),"class_id",0,10000);b.score=number(get(v,"score",num(1)),"score",0,1);
    b.x1=number(get(v,"x1"),"x1",0,4096);b.y1=number(get(v,"y1"),"y1",0,4096);
    b.x2=number(get(v,"x2"),"x2",0,4096);b.y2=number(get(v,"y2"),"y2",0,4096);
    if(b.x2<=b.x1||b.y2<=b.y1)throw std::invalid_argument("Detection box must have positive area");
    out.push_back(b);
  }return out;
}
double iou(const Box &a,const Box &b) {
  const double intersection=std::max(0.0,std::min(a.x2,b.x2)-std::max(a.x1,b.x1))*std::max(0.0,std::min(a.y2,b.y2)-std::max(a.y1,b.y1));
  return intersection/((a.x2-a.x1)*(a.y2-a.y1)+(b.x2-b.x1)*(b.y2-b.y1)-intersection);
}
fs::path resolved(const fs::path &base,const std::string &s) {auto p=fs::u8path(s);return fs::absolute(p.is_absolute()?p:base/p).lexically_normal();}
const std::vector<std::string>& sources(){static const std::vector<std::string> v{"camera","offline","ros"};return v;}
const std::vector<std::string>& methods(){static const std::vector<std::string> v{"original_q8","stereo_crop"};return v;}
std::string groupId(const std::string &source,const std::string &method){return source+"/"+method;}
} // namespace b6
