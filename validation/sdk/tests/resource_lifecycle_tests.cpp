#include "resource_probe.hpp"
#include "camera_close_policy.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
void require(bool ok){if(!ok)throw std::runtime_error("resource lifecycle assertion failed");}
int main(){try {
    namespace fs=std::filesystem;
    const auto root=fs::temp_directory_path()/"r24-proc-fixture";
    fs::remove_all(root);fs::create_directories(root/"fd");fs::create_directories(root/"fdinfo");
    fs::create_directories(root/"task"/"42");
    {std::ofstream f(root/"status");f<<"VmRSS:\t1024 kB\n";}
    {std::ofstream f(root/"task"/"42"/"comm");f<<"camera-thread\n";}
    const auto value=r24::procSnapshot(root,root/"missing");
    require(value.find("1024 kB")!=std::string::npos);
    require(value.find("\"fd_available\":true")!=std::string::npos);
    require(value.find("\"smaps_rollup\":null")!=std::string::npos);
    require(value.find("camera-thread")!=std::string::npos);
    require(r24::procSnapshot(root/"absent",root/"missing").find("\"fd_available\":false")!=std::string::npos);
    require(r24::jsonQuote(std::string("a\n\"\\")+char(0x87))=="\"a\\u000a\\\"\\\\\\u0087\"");
    for(bool restore:{false,true})for(int close_code:{0,-3}) {
        std::vector<int> calls;std::vector<std::string> names;
        bool safe=demo_camera::closeWithPolicy(restore,[&]{calls.push_back(1);return -2;},
            [&]{calls.push_back(2);return close_code;},[&](const char* name,int){names.push_back(name);});
        require(safe==(close_code==0));require(calls==(restore?std::vector<int>{1,2}:std::vector<int>{2}));
        require(names.back()=="moCloseCamera");
    }
    fs::remove_all(root);std::cout<<"Resource availability, escaping, and SDK close ordering/failure checks passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
