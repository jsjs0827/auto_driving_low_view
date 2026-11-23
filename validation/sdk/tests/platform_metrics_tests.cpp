#include "platform_metrics.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {try {
    const auto before=r29::processCpuMs();
    const auto limit=std::chrono::steady_clock::now()+std::chrono::milliseconds(40);
    volatile double value=1;
    while(std::chrono::steady_clock::now()<limit)value=std::sqrt(value+1.25);
    const auto after=r29::processCpuMs();
    if(!std::isfinite(before)||after<=before)throw std::runtime_error("process CPU clock did not advance under CPU work");
    const auto memory=r29::processResources();
    if(!memory.count("VmRSS:")||memory.at("VmRSS:")<=0||
       !memory.count("VmHWM:")||memory.at("VmHWM:")<memory.at("VmRSS:"))
        throw std::runtime_error("process working-set measurement unavailable/contradictory");
#ifdef _WIN32
    if(memory.count("CmaFree:"))throw std::runtime_error("Windows must not invent Linux CMA measurements");
#endif
    std::cout<<"process CPU and working-set measurement passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
