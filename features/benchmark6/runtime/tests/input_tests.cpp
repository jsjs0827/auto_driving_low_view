#include "ab/runtime.hpp"
#include "r36_stereo/nv12.hpp"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
int main(int argc, char **argv) {
    try {
        auto dir =
            argc >= 2
                ? ab::fs::absolute(argv[1])
                : ab::fs::temp_directory_path() /
                      ("ab-input-test-" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        if (ab::fs::exists(dir))
            throw std::runtime_error("Fixture path exists");
        ab::fs::create_directories(dir);
        cv::Mat rgb(544, 640, CV_8UC3, cv::Scalar(40, 120, 80));
        cv::imwrite((dir / "left.png").u8string(), rgb);
        auto yuv = ab::loadImage(dir / "left.png", 640, 544);
        if (yuv.rows != 816 || yuv.cols != 640 || yuv.total() != 522240)
            throw std::runtime_error("PNG normalization failed");
        auto roi = r36stereo::crop_nv12(yuv, {0, 192, 640, 352});
        if (roi.rows != 528 || roi.total() != 337920)
            throw std::runtime_error("NV12 crop geometry failed");
        std::vector<std::uint16_t> q(640 * 544, 2560);
        ab::maskQ8(q, 640, 544, {0, 192, 640, 352});
        for (int y = 0; y < 544; ++y)
            if (q[y * 640] != (y < 192 ? 0 : 2560))
                throw std::runtime_error("Q8 ROI coordinates failed");
        std::ofstream raw(dir / "left.nv12", std::ios::binary);
        raw.write(reinterpret_cast<const char *>(yuv.ptr()), yuv.total());
        raw.close();
        auto decoded = ab::loadImage(dir / "left.nv12", 640, 544);
        if (cv::norm(yuv, decoded, cv::NORM_INF) != 0)
            throw std::runtime_error("NV12 byte roundtrip failed");
        bool failed = false;
        try {
            ab::loadImage(dir / "left.png", 640, 352);
        } catch (...) {
            failed = true;
        }
        if (!failed)
            throw std::runtime_error("Hidden resize accepted");
        if (argc >= 2) {
            std::ofstream k(dir / "intrinsics.json");
            k << "{\"width\":640,\"height\":544,\"fx\":288,\"fy\":288,\"cx\":313,\"cy\":273,"
                 "\"bxf\":17280,\"bxf_unit\":\"pixel_mm\"}\n";
            std::vector<std::uint16_t> full(640 * 544, 2560);
            std::ofstream d(dir / "q8.u16", std::ios::binary);
            for (auto v : full) {
                d.put(char(v & 255));
                d.put(char(v >> 8));
            }
            d.close();
            std::ofstream manifest(dir / "manifest.json");
            manifest << "{\"schema_version\":1,\"frames\":[";
            for (int i = 0; i < 100; ++i) {
                if (i)
                    manifest << ',';
                manifest << "{\"key\":\"SYNTHETIC-TRANSPORT-ONLY-" << i << "\",\"frame_id\":\"" << i
                         << "\",\"timestamp_ns\":\"" << (1000000000ULL + 40000000ULL * i)
                         << "\",\"left\":\"left.nv12\",\"right\":\"left.nv12\",\"q8\":\"q8.u16\"}";
            }
            manifest << "]}\n";
            manifest.close();
            if (argc >= 3) {
                auto cfg = ab::load(argv[2]).root;
                auto setstr = [&](ab::Json &parent, const char *name, const std::string &value) {
                    ab::Json v;
                    v.kind = ab::Json::Kind::String;
                    v.text = value;
                    parent.object[name] = std::move(v);
                };
                auto setnum = [&](ab::Json &parent, const char *name, int value) {
                    ab::Json v;
                    v.kind = ab::Json::Kind::Number;
                    v.text = std::to_string(value);
                    parent.object[name] = std::move(v);
                };
                auto &off = cfg.object["input"].object["offline"];
                setstr(off, "manifest", (dir / "manifest.json").u8string());
                setstr(off, "intrinsics", (dir / "intrinsics.json").u8string());
                setnum(cfg.object["experiment"], "frames", 3);
                setnum(cfg.object["experiment"], "warmup", 0);
                setnum(cfg.object["experiment"], "timeout_ms", 10000);
                setnum(cfg.object["ros"], "bridge_port", 19093);
                setstr(cfg.object["ros"], "transport", "dds");
                std::ofstream dds(dir / "dds.json");
                dds << ab::dump(cfg) << '\n';
                setstr(cfg.object["ros"], "transport", "tcp");
                std::ofstream tcp(dir / "tcp.json");
                tcp << ab::dump(cfg) << '\n';
            }
        } else
            ab::fs::remove_all(dir);
        std::cout << "PNG/NV12/crop/ROI/no-hidden-resize checks passed"
                  << (argc >= 2 ? "; fixture is SYNTHETIC TRANSPORT ONLY" : "") << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
