#ifndef PERCEP_DEMO_SEG_CLASS_NAMES_HPP_
#define PERCEP_DEMO_SEG_CLASS_NAMES_HPP_

#include <charconv>
#include <cstddef>
#include <string>
#include <system_error>
#include <vector>

// Kept in the global namespace for compatibility with the legacy common.hpp
// helpers. This header is deliberately platform independent so the exact
// production parser can be exercised by host tests and fake-SDK builds.
inline std::string TrimCsvToken(std::string value) {
  while (!value.empty() &&
         (value.front() == ' ' || value.front() == '\t')) {
    value.erase(value.begin());
  }
  while (!value.empty() &&
         (value.back() == ' ' || value.back() == '\t')) {
    value.pop_back();
  }
  return value;
}

// Supports either "no_grass,grass" or "0:no_grass,1:grass". Explicit IDs
// outside the uint8 mask domain, including an overflowing decimal prefix, are
// rejected instead of silently falling back to sequential numbering.
inline std::vector<std::string> ParseSegClassNames(const std::string& csv,
                                                   int min_count = 0) {
  std::vector<std::string> names;
  int next_sequential_id = 0;
  std::string current;
  auto flush = [&]() {
    const std::string token = TrimCsvToken(current);
    current.clear();
    if (token.empty()) return;

    int id = next_sequential_id;
    std::string name = token;
    const std::size_t colon = token.find(':');
    if (colon != std::string::npos && colon > 0U) {
      bool digits_only = true;
      for (std::size_t index = 0; index < colon; ++index) {
        if (token[index] < '0' || token[index] > '9') {
          digits_only = false;
          break;
        }
      }
      if (digits_only) {
        const char* first = token.data();
        const char* last = first + colon;
        const auto parsed = std::from_chars(first, last, id);
        if (parsed.ec != std::errc{} || parsed.ptr != last || id < 0 ||
            id > 255) {
          return;
        }
        name = TrimCsvToken(token.substr(colon + 1U));
      }
    }

    if (id < 0 || id > 255) return;
    if (static_cast<int>(names.size()) <= id) {
      names.resize(static_cast<std::size_t>(id) + 1U);
    }
    names[static_cast<std::size_t>(id)] =
        name.empty() ? ("cls_" + std::to_string(id)) : name;
    next_sequential_id = id + 1;
  };

  for (char value : csv) {
    if (value == ',') {
      flush();
    } else {
      current.push_back(value);
    }
  }
  flush();

  if (min_count > static_cast<int>(names.size())) {
    names.resize(static_cast<std::size_t>(min_count));
  }
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (names[index].empty()) {
      names[index] = "cls_" + std::to_string(index);
    }
  }
  return names;
}

inline std::string JoinCsv(const std::vector<std::string>& names) {
  std::string joined;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0U) joined.push_back(',');
    joined += names[index];
  }
  return joined;
}

#endif  // PERCEP_DEMO_SEG_CLASS_NAMES_HPP_
