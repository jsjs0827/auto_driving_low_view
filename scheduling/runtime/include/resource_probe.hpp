#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace r24 {
inline std::string jsonQuote(const std::string& value) {
    std::ostringstream s;s<<'"';
    for(unsigned char c:value) {
        if(c=='"'||c=='\\')s<<'\\'<<char(c);
        else if(c<32 || c>=128)s<<"\\u00"<<std::hex<<std::setw(2)<<std::setfill('0')<<int(c)<<std::dec;
        else s<<char(c);
    }
    return s.str()+'"';
}
inline std::string procText(const std::filesystem::path& file) {
    std::ifstream f(file,std::ios::binary);
    if(!f)return "null";
    return jsonQuote({std::istreambuf_iterator<char>(f),{}});
}
inline std::string procSnapshot(const std::filesystem::path& self="/proc/self",
                                const std::filesystem::path& meminfo="/proc/meminfo") {
    namespace fs=std::filesystem;
    std::ostringstream s;s<<"{\"schema\":1,\"status\":"<<procText(self/"status")
      <<",\"smaps_rollup\":"<<procText(self/"smaps_rollup")<<",\"maps\":"<<procText(self/"maps")
      <<",\"meminfo\":"<<procText(meminfo);
    const auto entries=[&](const char* name) {
        std::vector<fs::path> paths;std::error_code ec;
        {fs::directory_iterator it(self/name,ec),end;
         while(!ec && it!=end){paths.push_back(it->path());it.increment(ec);}}
        // Close the directory iterator before resolving targets: its own FD
        // must not become a reported persistent camera descriptor.
        s<<",\""<<name<<"_available\":"<<(ec?"false":"true")<<",\""<<name<<"\":[";
        bool first=true;
        for(const auto& path:paths) {
            if(!first)s<<',';first=false;
            s<<"{\"id\":"<<jsonQuote(path.filename().string());
            if(std::string(name)=="fd") {
                std::error_code error;const auto target=fs::read_symlink(path,error);
                s<<",\"target\":"<<(error?"null":jsonQuote(target.string()))
                 <<",\"error\":"<<(error?jsonQuote(error.message()):"null")
                 <<",\"info\":"<<procText(self/"fdinfo"/path.filename());
            } else s<<",\"comm\":"<<procText(path/"comm")<<",\"stat\":"<<procText(path/"stat");
            s<<'}';
        }
        s<<']';
    };
    entries("fd");entries("task");return s.str()+"}\n";
}
} // namespace r24
