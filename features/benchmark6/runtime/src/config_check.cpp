#include "ab/config.hpp"
#include <iostream>
int main(int argc, char **argv) {
    try {
        if (argc != 2 && argc != 3)
            throw std::invalid_argument("ab_config_check CONFIG.json [--matrix]");
        bool matrix = argc == 3 && std::string(argv[2]) == "--matrix";
        bool models = argc == 3 && std::string(argv[2]) == "--models";
        bool transport = argc == 3 && std::string(argv[2]) == "--transport";
        bool bridge_port = argc == 3 && std::string(argv[2]) == "--bridge-port";
        if (argc == 3 && !matrix && !models && !transport && !bridge_port)
            throw std::invalid_argument("Unknown option");
        auto c = ab::load(argv[1]);
        if (bridge_port) {
            std::cout << c.bridge_port << '\n';
            return 0;
        }
        if (transport) {
            std::cout << c.ros_transport << '\n';
            return 0;
        }
        if (models) {
            std::cout << c.det.u8string() << '\n'
                      << c.seg.u8string() << '\n'
                      << c.stereo.u8string() << '\n';
            return 0;
        }
        if (!matrix)
            std::cout << "Configuration valid; source=" << c.source << " frames=" << c.frames
                      << "\n";
        for (auto &suite : c.suites)
            for (auto &m : c.methods) {
                auto r = ab::region(c, m, suite);
                std::cout << suite << ' ' << m;
                if (!matrix)
                    std::cout << " ROI=" << r.x << ',' << r.y << ',' << r.w << ',' << r.h;
                std::cout << '\n';
            }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
