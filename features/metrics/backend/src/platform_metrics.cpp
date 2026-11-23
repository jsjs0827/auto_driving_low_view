#include "platform_metrics.hpp"
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>
#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <psapi.h>
#else
#include <time.h>
#endif

namespace r29 {
double processCpuMs() {
#ifdef _WIN32
    FILETIME created{},exited{},kernel{},user{};
    if(!GetProcessTimes(GetCurrentProcess(),&created,&exited,&kernel,&user))
        throw std::system_error(static_cast<int>(GetLastError()),std::system_category(),"GetProcessTimes");
    ULARGE_INTEGER k{},u{};
    k.LowPart=kernel.dwLowDateTime;k.HighPart=kernel.dwHighDateTime;
    u.LowPart=user.dwLowDateTime;u.HighPart=user.dwHighDateTime;
    return static_cast<double>(k.QuadPart+u.QuadPart)/10000.0;
#else
    timespec value{};
    if(clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&value)!=0)
        throw std::system_error(errno,std::generic_category(),"clock_gettime process CPU");
    return value.tv_sec*1000.0+value.tv_nsec/1000000.0;
#endif
}
std::map<std::string,long> processResources() {
    std::map<std::string,long> result;
#ifdef _WIN32
    const auto kib=[](unsigned long long bytes) {
        const auto count=bytes/1024;
        return count>static_cast<unsigned long long>(std::numeric_limits<long>::max())
            ?std::numeric_limits<long>::max():static_cast<long>(count);
    };
    PROCESS_MEMORY_COUNTERS_EX memory{};memory.cb=sizeof(memory);
    if(GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&memory),sizeof(memory))) {
        result["VmRSS:"]=kib(memory.WorkingSetSize);
        result["VmHWM:"]=kib(memory.PeakWorkingSetSize);
        result["PrivateBytes:"]=kib(memory.PrivateUsage);
    }
    MEMORYSTATUSEX physical{};physical.dwLength=sizeof(physical);
    if(GlobalMemoryStatusEx(&physical))result["MemAvailable:"]=kib(physical.ullAvailPhys);
#else
    for(const auto* path:{"/proc/self/status","/proc/meminfo"}) {
        std::ifstream stream(path);std::string line;
        while(std::getline(stream,line)) {
            std::istringstream input(line);std::string key;long value;
            if(input>>key>>value)result[key]=value;
        }
    }
#endif
    return result;
}
}
