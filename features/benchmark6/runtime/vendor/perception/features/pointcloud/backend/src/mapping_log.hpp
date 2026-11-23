#pragma once
#include <cstdio>
// Standalone logging implementation; no middleware or SDK ABI is impersonated.
#define LOG_INFO(...) do { std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); } while (false)
#define LOG_WARN(...) LOG_INFO(__VA_ARGS__)
#define LOG_ERROR(...) LOG_INFO(__VA_ARGS__)
