#pragma once
#include "ab/config.hpp"
#include "r33/mapping.hpp"
#include <opencv2/core.hpp>
#include <atomic>
#include <map>
#include <memory>
namespace ab {
extern std::atomic<bool> running;
struct Frame {
    std::uint64_t id = 0, ns = 0;
    std::string key;
    cv::Mat left, right;
    std::vector<std::uint16_t> q8;
    Intrinsics k;
};
class Source {
  public:
    virtual ~Source() = default;
    virtual Frame next() = 0;
    virtual std::string metadata() const = 0;
};
std::unique_ptr<Source> offlineSource(const Config &, bool need_q8);
std::unique_ptr<Source> cameraSource(const Config &, bool need_q8);
std::unique_ptr<Source> rosSource(const Config &, bool need_q8);
std::unique_ptr<Source> nativeRosSource(const Config &, bool need_q8);
std::unique_ptr<Source> tcpSource(const Config &, bool need_q8);
void bridgeForward(const Config &, bool need_q8);
void rosPublish(const Config &, const std::string &source, bool need_q8);
cv::Mat loadImage(const fs::path &, int w, int h);
std::vector<std::uint16_t> readQ8(const fs::path &, int w, int h);
void maskQ8(std::vector<std::uint16_t> &, int w, int h, Roi);
struct Output {
    std::shared_ptr<const r33::MappingResult> mapping;
    std::vector<std::uint16_t> q8;
    std::vector<std::uint8_t> mask;
    std::vector<r33::MappingDetection> detections;
    std::map<std::string, double> ms;
};
class Pipeline {
    struct Impl;
    std::unique_ptr<Impl> p_;

  public:
    Pipeline(const Config &, const std::string &method, const std::string &suite,
             const fs::path &output);
    ~Pipeline();
    Output process(const Frame &);
    std::string metadata() const;
};
void saveOutput(const fs::path &, const Frame &, const Output &);
} // namespace ab
