#include "perception_ros_so_demo/json_text.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ros_so_demo {
std::string quote(const std::string& s) {
    std::ostringstream o;o<<'"';
    for(unsigned char c:s){
        if(c=='"'||c=='\\')o<<'\\'<<char(c);
        else if(c<32)o<<"\\u00"<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c)<<std::dec;
        else o<<char(c);
    }o<<'"';return o.str();
}
std::string number(double v) {
    if(!std::isfinite(v))return "null";
    std::ostringstream o;o<<std::setprecision(12)<<v;return o.str();
}
} // namespace ros_so_demo
