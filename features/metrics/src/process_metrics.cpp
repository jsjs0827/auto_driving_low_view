#include "perception_ros_so_demo/process_metrics.hpp"
#include <ctime>
#include <fstream>
#include <sstream>

namespace ros_so_demo {
double milliseconds(Steady::time_point begin) {
    return std::chrono::duration<double,std::milli>(Steady::now()-begin).count();
}
double cpuMs() {
    timespec value{};
    if(clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&value))return 0;
    return double(value.tv_sec)*1000.0+double(value.tv_nsec)/1e6;
}
long rssKib() {
    std::ifstream file("/proc/self/status");std::string line;
    while(std::getline(file,line))if(line.rfind("VmRSS:",0)==0){std::istringstream in(line.substr(6));long n=0;in>>n;return n;}
    return 0;
}

} // namespace ros_so_demo
