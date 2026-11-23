#include "perception/frame_input.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

extern "C" int input_sensor_slots_header_is_c(void);
namespace {
using perception::input::FrameBundle;
namespace rt = perception::runtime;
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& action) {
    try { action(); } catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("Malformed optional input was accepted");
}
psdk_frame image(const std::vector<std::uint8_t>& bytes, unsigned format,
                 unsigned width, unsigned height, unsigned stride) {
    psdk_frame value{};
    value.struct_size = sizeof(value); value.abi_version = PSDK_ABI_VERSION;
    value.format = format; value.width = width; value.height = height; value.stride = stride;
    value.data = bytes.data(); value.bytes = bytes.size();
    value.frame_id = 7; value.timestamp_ns = 1000;
    return value;
}
class ReadSlots : public rt::Feature {
public:
    void execute(rt::FeatureContext& context) override {
        require(context.get<psdk_frame>("input.right")->frame_id == 7, "Right input missing in task");
        require(context.get<psdk_frame>("input.rear")->frame_id == 99, "Rear input missing in task");
        require(context.get<psdk_imu_input>("input.imu")->samples[1].timestamp_ns == 950, "IMU batch missing in task");
        require(context.get<psdk_wheel_input>("input.wheel")->samples[0].linear_speed_mps == .5, "Wheel batch missing in task");
        require(context.get<psdk_radar_input>("input.radar")->points[0].x_m == 2, "Radar scan missing in task");
        context.set("slots.checked", true);
    }
};
}

int main() { try {
    require(input_sensor_slots_header_is_c() != 0, "Public input headers are not C compatible");
    std::vector<std::uint8_t> front_bytes(12, 80), rear_bytes(27, 50);
    auto left = image(front_bytes, PSDK_FORMAT_NV12, 4, 2, 4);
    auto right = left;
    auto rear = image(rear_bytes, PSDK_FORMAT_RGB, 3, 3, 9);
    rear.frame_id = 99; rear.timestamp_ns = 980; // Independent rear geometry/capture.
    psdk_full_frame frame{};
    frame.struct_size = sizeof(frame); frame.abi_version = PSDK_FULL_ABI_VERSION;
    frame.left = left;
    auto absent = FrameBundle::borrowed(frame);
    rt::ArtifactStore absent_store; absent.seed(absent_store);
    for (const auto* key : {"input.right", "input.rear", "input.imu", "input.wheel", "input.radar"})
        require(!absent_store.contains(key), "Missing sensor data was fabricated");

    psdk_imu_sample imu_samples[2]{};
    imu_samples[0].timestamp_ns = 900; imu_samples[1].timestamp_ns = 950;
    imu_samples[0].linear_acceleration_mps2[2] = 9.81;
    psdk_imu_input imu{sizeof(imu), PSDK_SENSOR_INPUT_ABI_VERSION, imu_samples, 2, 0};
    psdk_motion_sample wheel_sample{};
    wheel_sample.struct_size = sizeof(wheel_sample); wheel_sample.abi_version = PSDK_SENSOR_INPUT_ABI_VERSION;
    wheel_sample.timestamp_ns = 940; wheel_sample.linear_speed_mps = .5;
    psdk_wheel_input wheel{sizeof(wheel), PSDK_SENSOR_INPUT_ABI_VERSION, &wheel_sample, 1, 0};
    psdk_radar_point point{2, 1, 0, -.25f, 42};
    psdk_radar_input radar{sizeof(radar), PSDK_SENSOR_INPUT_ABI_VERSION,
        PSDK_RADAR_HAS_RADIAL_VELOCITY | PSDK_RADAR_HAS_INTENSITY, 0, 970, &point, 1};
    frame.right = &right; frame.rear = &rear; frame.imu = &imu; frame.wheel = &wheel; frame.radar = &radar;
    auto packet = FrameBundle::borrowed(frame);
    rt::ArtifactStore store; packet.seed(store);
    require(store.get<psdk_frame>("input.rear")->data == rear_bytes.data(), "Rear pixels were replaced");
    require(store.get<psdk_imu_input>("input.imu")->samples == imu_samples, "IMU storage was replaced");
    require(store.get<psdk_wheel_input>("input.wheel")->samples == &wheel_sample, "Wheel storage was replaced");
    require(store.get<psdk_radar_input>("input.radar")->points == &point, "Radar storage was replaced");
    rt::FeatureRegistry registry;
    registry.add("test.read_slots", [](const rt::Parameters&) { return std::make_unique<ReadSlots>(); });
    rt::TaskGraphConfig graph_config;
    rt::TaskSpec task; task.id = "read"; task.features = {"test.read_slots"};
    graph_config.tasks.push_back(task); graph_config.result_artifacts = {"slots.checked"};
    rt::TaskGraph graph(graph_config, registry);
    require(graph.run(store).ok() && *store.get<bool>("slots.checked"), "DAG could not consume optional slots");

    // The old right-only size must not inspect the new optional-pointer tail.
    frame.struct_size = PSDK_FULL_FRAME_V2_SIZE;
    frame.rear = reinterpret_cast<const psdk_frame*>(1);
    frame.imu = reinterpret_cast<const psdk_imu_input*>(1);
    frame.wheel = reinterpret_cast<const psdk_wheel_input*>(1);
    frame.radar = reinterpret_cast<const psdk_radar_input*>(1);
    auto legacy = FrameBundle::borrowed(frame);
    require(legacy.right && !legacy.rear && !legacy.imu && !legacy.wheel && !legacy.radar,
            "Right-only ABI read beyond its declared size");
    frame.struct_size = PSDK_FULL_FRAME_V1_SIZE; frame.right = reinterpret_cast<const psdk_frame*>(1);
    require(!FrameBundle::borrowed(frame).right, "Original ABI read the right tail");
    frame.struct_size = sizeof(frame) - 1; rejects([&] { FrameBundle::borrowed(frame); });
    frame.struct_size = sizeof(frame);
    frame.right = &right; frame.rear = &rear; frame.imu = &imu; frame.wheel = &wheel; frame.radar = &radar;

    right.timestamp_ns++; rejects([&] { FrameBundle::borrowed(frame); }); right.timestamp_ns--;
    rear.bytes = 1; rejects([&] { FrameBundle::borrowed(frame); }); rear.bytes = rear_bytes.size();
    rear.timestamp_ns = 0; rejects([&] { FrameBundle::borrowed(frame); }); rear.timestamp_ns = 980;
    imu.abi_version++; rejects([&] { FrameBundle::borrowed(frame); }); imu.abi_version--;
    imu.reserved = 1; rejects([&] { FrameBundle::borrowed(frame); }); imu.reserved = 0;
    imu.samples = nullptr; rejects([&] { FrameBundle::borrowed(frame); }); imu.samples = imu_samples;
    imu.count = PSDK_SENSOR_MAX_SAMPLES + 1; rejects([&] { FrameBundle::borrowed(frame); }); imu.count = 2;
    imu_samples[1].timestamp_ns = 900; rejects([&] { FrameBundle::borrowed(frame); }); imu_samples[1].timestamp_ns = 950;
    imu_samples[0].angular_velocity_radps[0] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { FrameBundle::borrowed(frame); }); imu_samples[0].angular_velocity_radps[0] = 0;
    wheel_sample.struct_size--; rejects([&] { FrameBundle::borrowed(frame); }); wheel_sample.struct_size++;
    wheel_sample.reserved[1] = 1; rejects([&] { FrameBundle::borrowed(frame); }); wheel_sample.reserved[1] = 0;
    wheel_sample.linear_speed_mps = std::numeric_limits<double>::infinity();
    rejects([&] { FrameBundle::borrowed(frame); }); wheel_sample.linear_speed_mps = .5;
    radar.fields |= 4; rejects([&] { FrameBundle::borrowed(frame); }); radar.fields &= ~4u;
    radar.timestamp_ns = 0; rejects([&] { FrameBundle::borrowed(frame); }); radar.timestamp_ns = 970;
    radar.points = nullptr; rejects([&] { FrameBundle::borrowed(frame); }); radar.points = &point;
    radar.point_count = PSDK_RADAR_MAX_POINTS + 1ULL; rejects([&] { FrameBundle::borrowed(frame); }); radar.point_count = 1;
    point.x_m = std::numeric_limits<float>::quiet_NaN(); rejects([&] { FrameBundle::borrowed(frame); }); point.x_m = 2;
    point.radial_velocity_mps = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { FrameBundle::borrowed(frame); });
    radar.fields = PSDK_RADAR_HAS_INTENSITY;
    FrameBundle::borrowed(frame); // Undeclared optional velocity is ignored.

    imu.count = 0; imu.samples = nullptr; wheel.count = 0; wheel.samples = nullptr;
    radar.point_count = 0; radar.points = nullptr;
    auto empty = FrameBundle::borrowed(frame);
    require(empty.imu && empty.wheel && empty.radar, "Empty observation confused with absent sensor");
    std::cout << "Optional inputs: C headers, old ABI, borrowed storage, DAG reads and malformed data passed\n";
    return 0;
} catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; } }
