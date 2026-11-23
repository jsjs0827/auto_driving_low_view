#include "slip_timeline.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs=std::filesystem;
namespace app=perception::slip_app;
namespace {
void need(bool value,const char* message){if(!value)throw std::runtime_error(message);}
template<class F> void reject(F action,const char* message) {
    try{action();}catch(const std::exception&){return;}throw std::runtime_error(message);
}
void write(const fs::path& path,const std::string& value) {
    std::ofstream out(path,std::ios::binary);out<<value;out.close();need(bool(out),"fixture write failed");
}
fs::path checked(const fs::path& path,const fs::path& parent) {
    const std::string prefix="slip-timeline-contract-",name=path.filename().u8string();
    need(path.is_absolute()&&name.size()>prefix.size()&&name.rfind(prefix,0)==0&&
         std::all_of(name.begin()+std::ptrdiff_t(prefix.size()),name.end(),[](unsigned char c){return c>='0'&&c<='9';}),"unsafe fixture name");
    need(!fs::is_symlink(fs::symlink_status(path))&&fs::is_directory(path),"unsafe fixture type");
    const auto resolved=fs::canonical(path);
    need(fs::equivalent(path.parent_path(),parent)&&fs::equivalent(resolved.parent_path(),parent)&&resolved.filename()==path.filename(),"unsafe fixture boundary");
    return resolved;
}
}
int main() {
    try {
        const auto parent=fs::canonical(fs::temp_directory_path());
        const auto tick=std::chrono::steady_clock::now().time_since_epoch().count();
        auto root=parent/("slip-timeline-contract-"+std::to_string(tick));
        need(fs::create_directory(root),"fixture must be new");root=checked(root,parent);
        const auto left=root/"left, eye.nv12",right=root/"right.nv12",q8=root/"q8.raw",csv=root/"timeline.csv";
        write(left,std::string(24,'a'));write(right,std::string(24,'b'));write(q8,std::string("\x01\x02",2)+std::string(30,'\0'));
        app::TimelineOptions options;options.width=4;options.height=4;
        const std::string columns="timestamp_ns,left_nv12,q8_u16,frame_id\n";
        write(csv,"\xef\xbb\xbf"+columns+"18446744073709551614,\"left, eye.nv12\",q8.raw,7\r\n18446744073709551615,\"left, eye.nv12\",q8.raw,8\r\n");
        auto rows=app::readTimeline(csv,options);
        need(rows.size()==2&&rows[0].frame_id==7&&rows[0].timestamp_ns==18446744073709551614ULL,"uint64/explicit identity not preserved");
        need(fs::equivalent(rows[0].left_nv12,left)&&rows[0].identity_source=="recorded_frame_id","relative/quoted path resolution failed");
        need(app::readQ8LittleEndian(q8,16)[0]==513,"Q8 must be decoded exactly as little endian uint16");
        write(csv,"timestamp_ns,left_nv12,q8_u16\n100,\"left, eye.nv12\",q8.raw\n");
        rows=app::readTimeline(csv,options);need(rows[0].frame_id==100&&rows[0].identity_source=="source_timestamp_ns","timestamp identity fallback failed");
        const auto invalid=[&](const std::string& contents) {write(csv,contents);reject([&]{app::readTimeline(csv,options);},"bad timeline accepted");};
        invalid(columns+"0,\"left, eye.nv12\",q8.raw,1\n");
        invalid(columns+"18446744073709551616,\"left, eye.nv12\",q8.raw,1\n");
        invalid(columns+"+100,\"left, eye.nv12\",q8.raw,1\n");
        invalid(columns+" 100,\"left, eye.nv12\",q8.raw,1\n");
        invalid(columns+"100,\"left, eye.nv12\",q8.raw,1\n100,\"left, eye.nv12\",q8.raw,2\n");
        invalid(columns+"100,\"left, eye.nv12\",q8.raw,1\n99,\"left, eye.nv12\",q8.raw,2\n");
        invalid(columns+"100,\"left, eye.nv12\",q8.raw,1\n101,\"left, eye.nv12\",q8.raw,1\n");
        invalid(columns+"100,,q8.raw,1\n");
        invalid(columns+"100,\"left, eye.nv12\",,1\n");
        invalid(columns+"100,\"left, eye.nv12\",q8.raw\n");
        invalid("timestamp_ns;left_nv12;q8_u16\n100;left.nv12;q8.raw\n");
        invalid("timestamp_ns,left_nv12,left_nv12,q8_u16\n100,a,a,q\n");
        invalid(columns+"100,\"left, eye.nv12\"junk,q8.raw,1\n");
        invalid(columns+"100,\"left, eye.nv12,q8.raw,1\n");
        invalid(columns+"100,\"left, eye.nv12\",q8.raw,1\n\n");
        options.original_q8=false;
        write(csv,"timestamp_ns,left_nv12,right_nv12,q8_u16\n100,\"left, eye.nv12\",right.nv12,\n");
        rows=app::readTimeline(csv,options);need(rows[0].q8_u16.empty()&&fs::equivalent(rows[0].right_nv12,right),"Stereo must not require original Q8");
        reject([&]{app::TimelineOptions q=options;q.original_q8=true;app::readTimeline(csv,q);},"original accepted missing Q8");
        invalid("timestamp_ns,left_nv12,right_nv12\n100,\"left, eye.nv12\",\"left, eye.nv12\"\n");
        invalid("timestamp_ns,left_nv12,right_nv12\n100,\"left, eye.nv12\",\n");
        write(csv,"timestamp_ns,left_nv12,right_nv12\n100,\"left, eye.nv12\",right.nv12\n101,\"left, eye.nv12\",right.nv12\n");
        reject([&]{auto q=options;q.max_rows=1;app::readTimeline(csv,q);},"row cap ignored");
        reject([&]{auto q=options;q.max_bytes=1;app::readTimeline(csv,q);},"file byte cap ignored");
        write(right,std::string(25,'b'));reject([&]{app::readTimeline(csv,options);},"wrong byte size accepted");
        const auto motion=root/"motion.csv";
        write(motion,"timestamp_ns,linear_speed_mps,yaw_rate_radps\n100,-1.25,0.1\n200,2,0\n300,3,0\n400,4,0\n");
        const auto samples=app::readMotionCsv(motion);
        need(samples.size()==4&&samples[0].linear_speed_mps==-1.25,"motion values not preserved");
        need(app::initialMotionIndex(samples,250)==1,"initial cursor must retain preceding endpoint");
        auto cursor=app::motionEndpoint(samples,1,250);need(cursor==3,"future interpolation endpoint missing");
        need(app::motionEndpoint(samples,cursor,260)==cursor,"must not push a second unused future sample");
        need(app::motionEndpoint(samples,cursor,300)==4,"next future endpoint missing after current endpoint reached");
        need(app::initialMotionIndex(samples,50)==0&&app::initialMotionIndex(samples,999)==3,"initial boundary cursor invalid");
        for(const auto& value:{"nan","inf","1x"," 1","1 "}) {
            write(motion,std::string("timestamp_ns,linear_speed_mps,yaw_rate_radps\n100,")+value+",0\n");
            reject([&]{app::readMotionCsv(motion);},"invalid finite motion value accepted");
        }
        write(motion,"timestamp_ns,linear_speed_mps,yaw_rate_radps\n100,1,0\n100,2,0\n");
        reject([&]{app::readMotionCsv(motion);},"duplicate motion timestamp accepted");
        write(motion,"timestamp_ns,linear_speed_mps,yaw_rate_radps\n100,1,0\n99,2,0\n");
        reject([&]{app::readMotionCsv(motion);},"backward motion timestamp accepted");
        write(motion,"timestamp_ns,linear_speed_mps\n100,1\n");
        reject([&]{app::readMotionCsv(motion);},"missing motion header accepted");
        const auto cleanup=checked(root,parent);fs::remove_all(cleanup);
        std::cout<<"slip timeline contract passed: original timestamps/IDs, NV12/Q8 bytes, CSV bounds, motion bracket cursor and invalid input rejection\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
