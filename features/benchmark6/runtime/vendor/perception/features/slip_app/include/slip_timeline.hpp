#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace perception::slip_app {
struct TimelineRow {
    std::uint64_t timestamp_ns=0, frame_id=0;
    std::filesystem::path left_nv12, q8_u16, right_nv12;
    std::string identity_source;
};
struct MotionRow {
    std::uint64_t timestamp_ns=0;
    double linear_speed_mps=0, yaw_rate_radps=0;
};
struct TimelineOptions {
    int width=640, height=544;
    bool original_q8=true;
    std::size_t max_rows=1000000;
    std::uintmax_t max_bytes=64*1024*1024;
};
// CSV paths resolve against the CSV directory. All capture timestamps must be
// nonzero, strictly increasing and in the same calibrated clock as motion.
// frame_id, when absent, is the source timestamp, never a playback counter.
std::vector<TimelineRow> readTimeline(const std::filesystem::path&,const TimelineOptions& = {});
std::vector<MotionRow> readMotionCsv(const std::filesystem::path&,std::size_t max_rows=1000000,
                                   std::uintmax_t max_bytes=64*1024*1024);
std::vector<std::uint8_t> readExactBytes(const std::filesystem::path&,std::size_t bytes);
std::vector<std::uint16_t> readQ8LittleEndian(const std::filesystem::path&,std::size_t pixels);
// Incremental reference feeding includes one future endpoint for interpolation.
// Choose the last preceding sample at initialization, rather than loading an
// entire old/future recording into the SDK's bounded temporal buffer.
std::size_t initialMotionIndex(const std::vector<MotionRow>&,std::uint64_t first_frame_ns);
std::size_t motionEndpoint(const std::vector<MotionRow>&,std::size_t begin,std::uint64_t frame_ns);
} // namespace perception::slip_app
