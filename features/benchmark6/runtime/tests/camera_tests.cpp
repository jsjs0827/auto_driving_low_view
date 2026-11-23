#include "ab/runtime.hpp"
#include <dlfcn.h>
#include <iostream>
int main(int argc, char** argv) {
    void* handle = nullptr;
    try {
        if (argc != 2)
            throw std::invalid_argument("Camera ABI fixture path required");
        handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
        if (!handle)
            throw std::runtime_error(dlerror());
        auto reset = reinterpret_cast<void (*)(int)>(dlsym(handle, "fixture_reset"));
        auto count = reinterpret_cast<int (*)(int)>(dlsym(handle, "fixture_count"));
        if (!reset || !count)
            throw std::runtime_error("Missing test-only fixture controls");
        reset(0);
        ab::Config c;
        c.camera_sdk = argv[1];
        c.camera_config = argv[1]; // Test fixture ignores config contents.
        {
            auto source = ab::cameraSource(c, true);
            auto f = source->next();
            if (f.left.ptr()[0] != 17 || f.right.ptr()[0] != 19 || f.q8.front() != 513)
                throw std::runtime_error("Owned frame changed after vendor buffer release");
            if (f.k.fx != 300 || f.k.fy != 300 || f.k.cx != 320 || f.k.cy != 272 ||
                f.k.bxf != 18000 || f.k.w != 640 || f.k.h != 544)
                throw std::runtime_error("SDK reference intrinsics were not scaled correctly");
            if (count(4) != 2)
                throw std::runtime_error("Q8 method did not request camera mode2");
        }
        if (count(3) != 1)
            throw std::runtime_error("First camera handle was not closed exactly once");
        {
            auto source = ab::cameraSource(c, false);
            auto f = source->next();
            if (!f.q8.empty() || count(4) != 1)
                throw std::runtime_error("Stereo method did not disable legacy Q8 producer");
        }
        if (count(3) != 2)
            throw std::runtime_error("Second camera handle was not closed exactly once");
        dlclose(handle);
        std::cout << "Camera ABI ownership, SDK intrinsics, mode1/mode2 and close checks passed; "
                     "fixture only, no physical camera or BPU\n";
        return 0;
    } catch (const std::exception& e) {
        if (handle)
            dlclose(handle);
        std::cerr << e.what() << '\n';
        return 1;
    }
}
