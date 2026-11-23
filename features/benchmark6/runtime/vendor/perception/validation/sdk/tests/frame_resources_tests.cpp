#include "perception/frame_resources.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void require(bool value,const char* reason) {
    if (!value) throw std::runtime_error(reason);
}
void sharedConversionOnce() {
    std::vector<std::uint8_t> source(8U*4U*3U,17);
    perception::ImageView image{source.data(),source.size(),8U*3U,8,4,perception::PixelFormat::RGB};
    std::atomic<unsigned> calls{0};
    perception::FrameResources resources(image,{41,123456},[&](const perception::ImageView& view) {
        ++calls;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return std::vector<std::uint8_t>(view.data,view.data+view.bytes);
    });
    require(calls==0,"RGB was converted before any consumer requested it");
    std::array<const perception::RgbFrame*,8> results{};
    std::array<std::exception_ptr,8> errors{};
    std::array<std::thread,8> workers;
    for (std::size_t i=0;i<workers.size();++i) workers[i]=std::thread([&,i] {
        try {results[i]=&resources.leftRgb();} catch (...) {errors[i]=std::current_exception();}
    });
    for (auto& worker:workers) worker.join();
    for (std::size_t i=0;i<workers.size();++i) {
        if (errors[i]) std::rethrow_exception(errors[i]);
        require(results[i]==results[0],"consumers received separate frame RGB buffers");
    }
    require(calls==1,"concurrent consumers converted the same frame more than once");
    const auto& first=*results[0];
    require(first.frame.id==41 && first.frame.timestamp_ns==123456,"RGB cache lost frame identity");
    require(first.width==8 && first.height==4 && first.pixels==source,"RGB cache changed the converter output");
    require(first.conversion_ms>=1,"conversion timing did not measure actual work");
    source[0]=99;
    require(first.pixels[0]==17,"RGB cache borrowed the caller's source storage");
    perception::FrameResources next(image,{42,123457},[&](const perception::ImageView& view) {
        ++calls;return std::vector<std::uint8_t>(view.data,view.data+view.bytes);
    });
    const auto& second=next.leftRgb();
    require(second.frame.id==42 && second.pixels[0]==99,"a previous frame's RGB crossed the frame boundary");
    require(second.pixels.data()!=first.pixels.data(),"two live frames share mutable RGB storage");
    require(calls==2,"a new frame incorrectly reused the old cache");
}
void failureDoesNotPublish() {
    std::vector<std::uint8_t> source(12,3);
    perception::ImageView image{source.data(),source.size(),6,2,2,perception::PixelFormat::RGB};
    unsigned calls=0;
    perception::FrameResources resources(image,{9,19},[&](const perception::ImageView&) {
        if (++calls==1) {
            std::vector<std::uint8_t> partial(12,77);
            throw std::runtime_error("converter rejected source");
        }
        if (calls==2) return std::vector<std::uint8_t>(11,88);
        return std::vector<std::uint8_t>(12,42);
    });
    for (int i=0;i<2;++i) {
        bool rejected=false;
        try {resources.leftRgb();} catch (const std::runtime_error&) {rejected=true;}
        require(rejected,"a failed or malformed conversion was published");
    }
    const auto& rgb=resources.leftRgb();
    require(calls==3 && rgb.pixels==std::vector<std::uint8_t>(12,42),"RGB cache did not retry cleanly after failure");
    require(&resources.leftRgb()==&rgb && calls==3,"successful RGB cache was rebuilt");
}
}
int main() {
    try {
        sharedConversionOnce();failureDoesNotPublish();
        std::cout<<"per-frame lazy RGB conversion, shared ownership, failure retry and timing passed\n";
        return 0;
    } catch (const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
