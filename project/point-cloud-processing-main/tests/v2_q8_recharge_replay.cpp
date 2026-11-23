#include "percep_main/v2_frame_input.hpp"
#include "percep_demo/api_recharge_mark.h"

#include <dlfcn.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace perception;

namespace {

template<typename T, size_t N>
void load(const std::string& path, T (&data)[N])
{
    std::ifstream file(path, std::ios::binary);
    if (!file.read(reinterpret_cast<char*>(data), sizeof(data)) || file.peek() != EOF)
        throw std::runtime_error("unexpected input size: " + path);
}

PicData metadata(uint64_t id, uint64_t ns, uint16_t w, uint16_t h, uint32_t bits, uint32_t bytes)
{
    PicData data{};
    data.frame_id = id;
    data.time_ns = ns;
    data.width = w;
    data.height = h;
    data.bits_per_pixel = bits;
    data.bytes_used = bytes;
    return data;
}

void writeMaps(const psdk_full_result_view& view, const std::string& path)
{
    if (view.map_count != 3) throw std::runtime_error("missing topviews");
    for (uint32_t i = 0; i < view.map_count; ++i) {
        const auto& m = view.maps[i];
        std::ofstream f(path + "." + m.name + ".bin", std::ios::binary);
        f.write(reinterpret_cast<const char*>(m.cells), m.cell_count * sizeof(psdk_grid_cell));
        if (!f) throw std::runtime_error("failed to write topview");
    }
}

void writePose(std::ostream& out, bool mark, bool pose, float x, float y, float yaw)
{
    out << mark << '\t' << pose << '\t' << std::hexfloat << x << '\t' << y << '\t' << yaw << std::defaultfloat;
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        if (argc != 6) throw std::runtime_error("mode map|recharge|original-recharge config manifest intrinsics output_directory");
        const std::string mode = argv[1], output = argv[5];
        const bool map = mode == "map", original = mode == "original-recharge";
        if (!map && !original && mode != "recharge") throw std::runtime_error("unknown mode");
        psdk_camera_intrinsics k{};
        std::ifstream intrinsics(argv[4]);
        if (!(intrinsics >> k.width >> k.height >> k.fx >> k.fy >> k.cx >> k.cy >> k.bxf))
            throw std::runtime_error("invalid intrinsics");
        std::ifstream manifest(argv[3]);
        std::ofstream records(output + "/records.tsv");
        if (!manifest || !records) throw std::runtime_error("manifest or output unavailable");
        std::unique_ptr<V2SdkRuntime> runtime;
        std::string error;
        Aiper_Recharge::RechargeMarkApi* legacy = nullptr;
        if (original) {
            auto* library = dlopen("librecharge_mark.so", RTLD_NOW | RTLD_LOCAL);
            if (!library) throw std::runtime_error(dlerror());
            using Factory = decltype(&Aiper_Recharge::createRechargeMarkApi);
            auto create = reinterpret_cast<Factory>(dlsym(library, "_ZN14Aiper_Recharge21createRechargeMarkApiEv"));
            if (!create) throw std::runtime_error(dlerror());
            legacy = create();
            if (!legacy || legacy->init("", argv[2])) throw std::runtime_error("original recharge init");
            legacy->setSaveResultEnable(false);
        }
        auto rgb = std::make_unique<CameraData>();
        auto gdc = std::make_unique<CameraGdcData>();
        std::string id, left, disparity, previous_group, group;
        uint64_t frame_id, ns;
        while (manifest >> id >> group >> frame_id >> ns >> left >> disparity) {
            if (group != previous_group) {
                if (original) legacy->resetSession();
                else {
                    runtime.reset();
                    runtime = std::make_unique<V2SdkRuntime>();
                    if (!runtime->open(argv[2], error) || !runtime->command(map ? 1 : 2, error))
                        throw std::runtime_error(error);
                }
                previous_group = group;
            }
            psdk_full_frame frame{};
            psdk_disparity_q8 q8{};
            psdk_frame hi{};
            if (map) {
                load(left, rgb->left_data);
                load(disparity, rgb->disp_data);
                rgb->left = metadata(frame_id, ns, 640, 544, 8, sizeof(rgb->left_data));
                rgb->disp = metadata(frame_id, ns, 640, 544, 16, sizeof(rgb->disp_data));
                if (!makeV2MapInput(*rgb, k, q8, frame)) throw std::runtime_error("invalid Q8 packet " + id);
            } else {
                load(left, gdc->left_data);
                gdc->left = metadata(frame_id, ns, 1280, 1088, 8, sizeof(gdc->left_data));
                if (!makeV2RechargeInput(*gdc, k, hi, frame)) throw std::runtime_error("invalid GDC packet " + id);
            }
            using Clock = std::chrono::steady_clock;
            const auto start = Clock::now();
            if (original) {
                auto* r = legacy->processLeftNv12(hi.data, hi.width, hi.height, hi.stride, hi.frame_id);
                if (!r) throw std::runtime_error("null original recharge result");
                const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                records << id << '\t' << ms << '\t';
                writePose(records, r->ok, r->pose_ok, r->pose_ok ? r->post.x : 999.f,
                          r->pose_ok ? r->post.y : 999.f, r->pose_ok ? r->post.yaw : 999.f);
                legacy->freeResult(r);
            } else {
                psdk_full_result_view view{};
                auto result = runtime->process(frame, view, error);
                const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                if (!result) throw std::runtime_error(id + ": " + error);
                records << id << '\t' << ms << '\t';
                if (map) {
                    writeMaps(view, output + "/" + id);
                    records << view.map_count;
                } else {
                    if (!view.recharge.executed) throw std::runtime_error("recharge skipped " + id);
                    const auto values = v2DockValues(view.recharge);
                    writePose(records, view.recharge.mark_ok, view.recharge.pose_ok,
                              values.x, values.y, values.yaw);
                }
            }
            records << '\n' << std::flush;
        }
        if (original) legacy->destroy(false);
        if (!manifest.eof()) throw std::runtime_error("malformed manifest");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
