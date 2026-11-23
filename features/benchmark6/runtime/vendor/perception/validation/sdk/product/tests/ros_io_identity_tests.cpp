#include "product/ros_io_identity.hpp"
#include "runtime_status.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>

namespace fs = std::filesystem;
namespace {
void check(bool result, const char* message) { if (!result) throw std::runtime_error(message); }
std::string contents(const fs::path& path) {
    std::ifstream stream(path, std::ios::binary);
    check(bool(stream), "receipt was not written");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
void environment(const std::string& value) {
#ifdef _WIN32
    check(_putenv_s("PERCEPTION_ROS_IO_RECEIPT", value.c_str()) == 0, "set receipt environment");
#else
    check(setenv("PERCEPTION_ROS_IO_RECEIPT", value.c_str(), 1) == 0, "set receipt environment");
#endif
}
template<class F> void rejects(F action) {
    try { action(); } catch(const r28::Failure& error) {
        check(error.status().code == 1004, "receipt failure lost initialization code");
        check(error.status().feature == "product.ros_io.receipt", "receipt failure lost feature");
        return;
    }
    throw std::runtime_error("bad receipt destination was accepted");
}
}

int main(int argc, char** argv) {
    // A separate JSON parser can inspect this output without any ROS/SDK ABI.
    if(argc == 3 && std::string(argv[1]) == "--write") {
        try { r31::product::writeRosIoReceipt(argv[2]); return 0; }
        catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    }
    const auto root = fs::temp_directory_path()/fs::u8path("r31 receipt " + std::to_string(
        std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    try {
        check(std::string(perception_product_build_id()) == "r12-v31-original-ros-io-v1", "wrong build id");
        fs::create_directories(root);
        const auto target = root/fs::u8path("loaded_凭据.json");
        r31::product::writeRosIoReceipt(target.u8string());
        const auto expected = contents(target);
        for(const char* field : {"\"state\":\"loaded\"", "\"actual_plugin_path\":", "\"pid\":",
            "\"framework\":\"original-dag-task-feature\"", "\"camera_owner\":\"original-camera-node\"",
            "\"input_contract\":\"original-map-rgbd-left-640x544-nv12\"", "\"single_frame\":true",
            "\"double_buffer\":false", "\"frame_processing_verified\":false"})
            check(expected.find(field) != std::string::npos, "receipt omitted a required field");
        check(expected.front() == '{' && expected.substr(expected.size()-2) == "}\n", "incomplete receipt");
        // Replacing an existing receipt must not expose a partially written file.
#ifdef _WIN32
        // Windows file handles may deny replacement while a reader holds the
        // destination open. Verify complete replacements with readers closed.
        for(int i=0;i<30;++i) {
            r31::product::writeRosIoReceipt(target.u8string());
            check(contents(target) == expected, "replacement exposed a partial receipt");
        }
#else
        std::exception_ptr writer_error;
        std::thread writer([&] {
            try { for(int i=0;i<30;++i) r31::product::writeRosIoReceipt(target.u8string()); }
            catch(...) { writer_error=std::current_exception(); }
        });
        bool complete = true;
        for(int i=0;i<100;++i) {
            try { complete = complete && contents(target) == expected; }
            catch(...) { complete = false; }
        }
        writer.join();
        if(writer_error) std::rethrow_exception(writer_error);
        check(complete && contents(target) == expected, "replacement exposed a partial receipt");
#endif
        environment(target.u8string());
        r31::product::writeRosIoReceiptFromEnvironment();
        check(contents(target) == expected, "environment receipt changed its identity");
        environment("");
        r31::product::writeRosIoReceiptFromEnvironment();
        r31::product::writeRosIoReceiptFromEnvironment();
        rejects([] { r31::product::writeRosIoReceipt(""); });
        rejects([&] { r31::product::writeRosIoReceipt((root/"missing"/"loaded.json").u8string()); });
        rejects([&] { r31::product::writeRosIoReceipt(root.u8string()); });
        for(const auto& file : fs::directory_iterator(root))
            check(file.path() == target, "failed receipt left a temporary file");
        fs::remove_all(root);
        std::cout << "receipt module identity, real file replacement, environment and 1004 failures passed; no ROS/frame verification\n";
        return 0;
    } catch(const std::exception& error) {
        std::error_code ignored; fs::remove_all(root, ignored);
        std::cerr << error.what() << '\n'; return 1;
    }
}
