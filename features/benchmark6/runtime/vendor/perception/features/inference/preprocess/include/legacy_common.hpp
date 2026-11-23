#pragma once
#include <cstdint>
#include <string>
#include <vector>
inline std::string TrimCsvToken(std::string s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
  return s;
}

// 支持 "no_grass,grass,flagstone" 或 "0:no_grass,1:grass,2:flagstone"
inline std::vector<std::string> ParseSegClassNames(const std::string& csv,
                                                   int min_count = 0) {
  std::vector<std::string> names;
  int seq = 0;
  std::string cur;
  auto flush = [&]() {
    const std::string tok = TrimCsvToken(cur);
    cur.clear();
    if (tok.empty()) return;
    int id = seq;
    std::string name = tok;
    const auto colon = tok.find(':');
    if (colon != std::string::npos && colon > 0) {
      bool digits = true;
      for (size_t i = 0; i < colon; ++i) {
        if (tok[i] < '0' || tok[i] > '9') {
          digits = false;
          break;
        }
      }
      if (digits) {
        id = 0;
        for (size_t i = 0; i < colon; ++i) id = id * 10 + (tok[i] - '0');
        name = TrimCsvToken(tok.substr(colon + 1));
      }
    }
    if (id < 0 || id > 255) return;
    if (static_cast<int>(names.size()) <= id) names.resize(static_cast<size_t>(id) + 1);
    names[static_cast<size_t>(id)] =
        name.empty() ? ("cls_" + std::to_string(id)) : name;
    seq = id + 1;
  };
  for (char c : csv) {
    if (c == ',') flush();
    else cur.push_back(c);
  }
  flush();
  if (min_count > static_cast<int>(names.size())) {
    names.resize(static_cast<size_t>(min_count));
  }
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty()) names[i] = "cls_" + std::to_string(i);
  }
  return names;
}

inline std::string JoinCsv(const std::vector<std::string>& names) {
  std::string out;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i) out += ',';
    out += names[i];
  }
  return out;
}

inline void SegClassVisRgb(int cls, const std::string& class_name, uint8_t& r,
                           uint8_t& g, uint8_t& b) {
  static const uint8_t kPal[16][3] = {
      {128, 64, 128}, {0, 180, 0},    {244, 35, 232}, {70, 70, 70},
      {102, 102, 156}, {190, 153, 153}, {153, 153, 153}, {250, 170, 30},
      {220, 220, 0},  {0, 128, 255},  {255, 128, 0},  {0, 255, 255},
      {139, 69, 19},  {192, 192, 192}, {64, 64, 64},   {180, 180, 40},
  };
  if (class_name == "grass") {
    r = 0;
    g = 180;
    b = 0;
    return;
  }
  if (class_name == "no_grass") {
    r = 128;
    g = 64;
    b = 128;
    return;
  }
  if (class_name == "flagstone") {
    r = 250;
    g = 170;
    b = 30;
    return;
  }
  if (class_name == "fence") {
    r = 244;
    g = 35;
    b = 232;
    return;
  }
  if (class_name == "puddle" || class_name == "waterpipe") {
    r = 0;
    g = 128;
    b = 255;
    return;
  }
  const int i = (cls >= 0) ? (cls % 16) : 0;
  r = kPal[i][0];
  g = kPal[i][1];
  b = kPal[i][2];
}


