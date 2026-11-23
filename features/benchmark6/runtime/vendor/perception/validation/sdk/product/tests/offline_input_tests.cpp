#include "product/offline_input.hpp"
#include "MoTypes.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;
void check(bool result, const char* text) { if (!result) throw std::runtime_error(text); }
template<class F> void rejects(F action) {
    bool failed=false; try { action(); } catch(const std::exception&) {failed=true;}
    check(failed, "invalid offline input was accepted");
}
void file(const fs::path& path, std::size_t bytes) {
    std::ofstream stream(path, std::ios::binary);
    const std::string data(bytes, '\x35'); stream.write(data.data(), data.size());
    check(static_cast<bool>(stream), "fixture write failed");
}
int main() {
    const auto root=fs::temp_directory_path()/fs::u8path("r30_离线_"+std::to_string(
        std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    try {
        const auto rgb=root/"rgb", disp=root/"disp";
        fs::create_directories(rgb);fs::create_directories(disp);
        file(rgb/fs::u8path("帧02.JPG"),4);file(disp/fs::u8path("帧02.RAW"),12);
        file(rgb/fs::u8path("帧01.jpeg"),6);file(disp/fs::u8path("帧01.raw"),12);
        auto frames=r30::product::indexOfflineInput(rgb.u8string(),disp.u8string(),3,2);
        check(frames.size()==2 && frames[0].stem==u8"帧01" && frames[1].stem==u8"帧02", "UTF-8 stem ordering/pairing");
        check(r30::product::readOfflineImage(frames[0].rgb_path).size()==6,"image bytes");
        check(r30::product::readOfflineDisparity(frames[0].disparity_path,3,2).size()==12,"disparity bytes");
        rejects([&]{r30::product::indexOfflineInput((root/"missing").u8string(),disp.u8string(),3,2);});
        rejects([&]{r30::product::indexOfflineInput(rgb.u8string(),disp.u8string(),0,2);});
        file(rgb/fs::u8path("帧01.jpg"),4);
        rejects([&]{r30::product::indexOfflineInput(rgb.u8string(),disp.u8string(),3,2);});
        fs::remove(rgb/fs::u8path("帧01.jpg"));
        fs::remove(disp/fs::u8path("帧01.raw"));
        rejects([&]{r30::product::indexOfflineInput(rgb.u8string(),disp.u8string(),3,2);});
        file(disp/fs::u8path("帧01.raw"),11);
        rejects([&]{r30::product::indexOfflineInput(rgb.u8string(),disp.u8string(),3,2);});
        rejects([&]{r30::product::readOfflineDisparity(frames[0].disparity_path,3,2);});
        file(disp/fs::u8path("帧01.raw"),12);file(disp/"orphan.raw",12);
        rejects([&]{r30::product::indexOfflineInput(rgb.u8string(),disp.u8string(),3,2);});
        std::size_t index=7;
        check(r30::product::parseOfflineJump("n 1",2,index)&&index==1,"valid jump");
        for(const auto* command:{"n -1","n 2","n 1x","n 9999999999999999999999999999999999","n ","n  0x","n  x"}) {
            index=7;check(!r30::product::parseOfflineJump(command,2,index)&&index==7,"invalid jump mutated index");
        }
        check(r30::product::parseOfflineJump("N  0 ",2,index)&&index==0,"uppercase jump");
        fs::remove_all(root);std::cout<<"offline input: real files, pairing, size and UTF-8 checks passed; no JPEG decoder/PCL/ROS validation\n";
        return 0;
    } catch(const std::exception& e) {fs::remove_all(root);std::cerr<<e.what()<<'\n';return 1;}
}
