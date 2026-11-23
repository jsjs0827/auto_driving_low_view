#include "perception_ros_so_demo/mo_camera.hpp"
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <unistd.h>

namespace fs=std::filesystem;
namespace raw=ros_so_demo::raw;
using Clock=std::chrono::steady_clock;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signalStop(int) { interrupted=1; }
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
std::string quote(const std::string& text) {
    std::ostringstream out;out<<'"';
    for(unsigned char c:text) {
        if(c=='"'||c=='\\')out<<'\\'<<char(c);
        else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
        else out<<char(c);
    }
    out<<'"';return out.str();
}
unsigned number(const std::string& text,unsigned limit,const char* key) {
    if(text.empty()||text.find_first_not_of("0123456789")!=std::string::npos)
        throw std::invalid_argument(std::string(key)+" must be a positive integer");
    const auto value=std::stoull(text);
    if(!value||value>limit)throw std::invalid_argument(std::string(key)+" is out of range");
    return unsigned(value);
}
struct Options {
    raw::CameraOptions camera;
    fs::path output;
    unsigned frames=100;
};
Options parse(int argc,char** argv) {
    Options out;out.camera.need_q8=true;
    std::set<std::string> seen;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(key=="--help") {
            std::cout<<"Record identical SDK RGBD left/right NV12 + Q8 captures.\n"
                     <<"r36_stereo_capture --sdk LIB --camera-config CFG --output NEW_DIR "
                       "[--frames 100] [--timeout-ms 3000]\n"
                     <<"The camera must be available exclusively. SDK rectified reference "
                       "intrinsics are queried; no raw lens calibration is substituted.\n"
                     <<"Writes manifest.jsonl and timestamped timeline.csv for native slip replay.\n";
            std::exit(0);
        }
        if(key!="--sdk"&&key!="--camera-config"&&key!="--output"&&key!="--frames"&&key!="--timeout-ms")
            throw std::invalid_argument("Unknown argument: "+key);
        if(!seen.insert(key).second)throw std::invalid_argument("Duplicate argument: "+key);
        if(++i>=argc||!argv[i][0])throw std::invalid_argument("Missing value for "+key);
        const std::string value=argv[i];
        if(key=="--sdk")out.camera.sdk=value;
        else if(key=="--camera-config")out.camera.config=value;
        else if(key=="--output")out.output=fs::u8path(value);
        else if(key=="--frames")out.frames=number(value,1000000,"--frames");
        else out.camera.timeout_ms=int(number(value,unsigned(std::numeric_limits<int>::max()),"--timeout-ms"));
    }
    if(out.output.empty())throw std::invalid_argument("--output NEW_DIR is required");
    out.camera.retry_ms=std::min(5,out.camera.timeout_ms);
    const std::uint16_t endian=1;
    if(*reinterpret_cast<const std::uint8_t*>(&endian)!=1)
        throw std::runtime_error("Raw SDK Q8 capture currently requires a little-endian host");
    return out;
}
std::ofstream output(const fs::path& path) {
    std::ofstream file(path,std::ios::binary);
    if(!file)throw std::runtime_error("Cannot create file: "+path.u8string());
    file<<std::setprecision(17);return file;
}
void finish(std::ofstream& file,const fs::path& path) {
    file.close();if(!file)throw std::runtime_error("Write failed: "+path.u8string());
}
void write(const fs::path& path,const raw::Plane& plane) {
    auto file=output(path);
    file.write(reinterpret_cast<const char*>(plane.bytes.data()),std::streamsize(plane.bytes.size()));
    finish(file,path);
}
void metadata(const Options& o,const raw::CameraParameters& p) {
    // The retained original CameraIntrinsics::scaledTo rule multiplies focal,
    // principal point and bxf by image scale, with no half-pixel offset. Only
    // the known equal-aspect RGBD/reference geometries are accepted here.
    if(!((p.width==640&&p.height==544)||(p.width==1280&&p.height==1088)))
        throw std::runtime_error("SDK rectified reference is "+std::to_string(p.width)+"x"+std::to_string(p.height)+
            "; RGBD reference/crop scaling is unverified. Obtain verified 640x544 rectified intrinsics "
            "and configure the SDK reference parameters before recording; no intrinsics are guessed.");
    const double sx=640.0/p.width,sy=544.0/p.height;
    const auto path=o.output/"metadata.json";auto file=output(path);const auto& k=p.rectified;
    file<<"{\"schema_version\":1,\"source\":\"camera_sdk_rgbd\",\"width\":640,\"height\":544,"
        <<"\"left_format\":\"nv12\",\"right_format\":\"nv12\",\"disparity_format\":\"q8_le\","
        <<"\"left_bytes\":522240,\"right_bytes\":522240,\"disparity_bytes\":696320,"
        <<"\"frame_id_source\":\"camera_sdk\",\"timestamp_source\":\"camera_sdk\",\"timestamp_unit\":\"ns\","
        <<"\"camera_sdk\":"<<quote(o.camera.sdk)<<",\"camera_config\":"<<quote(o.camera.config)<<','
        <<"\"geometry\":\"SDK_rectified_RGBD_reference\","
        <<"\"intrinsics\":{\"width\":640,\"height\":544,\"fx\":"<<k.fx*sx<<",\"fy\":"<<k.fy*sy
        <<",\"cx\":"<<k.cx*sx<<",\"cy\":"<<k.cy*sy<<",\"bxf\":"<<p.bxf*sx<<"},"
        <<"\"bxf_unit\":\"focal_px_times_baseline_mm\","
        <<"\"sdk_reference\":{\"api\":\"moGetCameraParams\",\"return_code\":"<<p.source_code
        <<",\"parameter_source\":"<<quote(p.source_code==0?"camera_module":"x5_configuration")
        <<",\"width\":"<<p.width<<",\"height\":"<<p.height<<",\"fx\":"<<k.fx<<",\"fy\":"<<k.fy
        <<",\"cx\":"<<k.cx<<",\"cy\":"<<k.cy<<",\"bxf\":"<<p.bxf<<",\"baseline_sdk\":"<<p.baseline
        <<",\"max_disparity\":"<<p.max_disparity<<",\"radial_distortion\":[";
    for(unsigned i=0;i<6;++i){if(i)file<<',';file<<k.radial_distortion_k[i];}
    file<<"],\"tangential_distortion\":["<<k.tangential_distortion_p[0]<<','<<k.tangential_distortion_p[1]<<"]},"
        <<"\"intrinsics_scaling\":{\"sx\":"<<sx<<",\"sy\":"<<sy
        <<",\"rule\":\"fx,cx,bxf*=sx;fy,cy*=sy;no_half_pixel_offset\","
        <<"\"reference\":\"original_pre_pointcloud_types.hpp_CameraIntrinsics::scaledTo\"},"
        <<"\"resource_scope\":\"whole_recorder_process_including_camera_SDK_threads\","
        <<"\"timing_note\":\"capture_acquire_ms includes SDK wait, owned copies and release; "
          "it is not the internal legacy stereo computation time. Capture CPU is whole-process CPU "
          "over acquisition intervals; do not add it to separate replay CPU. 100 percent means one CPU core.\"}\n";
    finish(file,path);
}
std::int64_t memoryKib(const char* key) {
    std::ifstream file("/proc/self/status");std::string line;
    const std::string prefix=std::string(key)+':';
    while(std::getline(file,line))if(line.rfind(prefix,0)==0) {
        std::istringstream in(line.substr(prefix.size()));std::int64_t value=-1;in>>value;return value;
    }
    return -1;
}
double cpuMs() {
    rusage usage{};if(getrusage(RUSAGE_SELF,&usage))return -1;
    return 1000.0*(usage.ru_utime.tv_sec+usage.ru_stime.tv_sec)+
        (usage.ru_utime.tv_usec+usage.ru_stime.tv_usec)/1000.0;
}
struct Stats {
    unsigned frames=0,requested=0;
    double wall_ms=0,acquire_ms=0,write_ms=0,cpu_ms=0,acquire_cpu_ms=0;
    std::int64_t rss_kib=-1,peak_rss_kib=-1;
    long online_cpus=sysconf(_SC_NPROCESSORS_ONLN);
};
void stats(const fs::path& directory,const Stats& s,const std::string& error) {
    const auto path=directory/(error.empty()?"complete.json":"failed.json");auto file=output(path);
    file<<"{\"completed\":"<<(error.empty()&&s.frames==s.requested?"true":"false")
        <<",\"interrupted\":"<<(interrupted?"true":"false")<<",\"frames_recorded\":"<<s.frames
        <<",\"frames_requested\":"<<s.requested<<",\"wall_ms\":"<<s.wall_ms
        <<",\"capture_acquire_ms_total\":"<<s.acquire_ms<<",\"write_ms_total\":"<<s.write_ms
        <<",\"cpu_ms\":"<<s.cpu_ms<<",\"capture_cpu_ms_total\":"<<s.acquire_cpu_ms
        <<",\"capture_cpu_percent\":"<<(s.acquire_ms>0?100*s.acquire_cpu_ms/s.acquire_ms:0)
        <<",\"rss_kib\":"<<s.rss_kib<<",\"peak_rss_kib\":"<<s.peak_rss_kib
        <<",\"online_cpus\":"<<s.online_cpus<<",\"capture_acquire_s\":"<<s.acquire_ms/1000
        <<",\"whole_recording_s\":"<<s.wall_ms/1000<<",\"capture_acquire_fps\":"
        <<(s.acquire_ms>0?s.frames*1000.0/s.acquire_ms:0)<<",\"capture_fps\":"
        <<(s.wall_ms>0?s.frames*1000.0/s.wall_ms:0)
        <<",\"resource_scope\":\"whole_recorder_process_including_camera_SDK_threads\",\"error\":"<<quote(error)<<"}\n";
    finish(file,path);
}
}
int main(int argc,char** argv) {
    fs::path active;Stats summary;const auto started=Clock::now();const double cpu_start=cpuMs();
    try {
        const auto options=parse(argc,argv);summary.requested=options.frames;
        if(fs::exists(options.output)||!fs::create_directories(options.output))
            throw std::runtime_error("--output must be a NEW directory: "+options.output.u8string());
        active=options.output;
        std::signal(SIGINT,signalStop);std::signal(SIGTERM,signalStop);
        raw::Camera camera(options.camera);
        metadata(options,camera.parameters());
        const auto manifest_path=active/"manifest.jsonl";auto manifest=output(manifest_path);
        const auto timeline_path=active/"timeline.csv";auto timeline=output(timeline_path);
        timeline<<"timestamp_ns,left_nv12,q8_u16,right_nv12,frame_id\n";
        timeline.flush();if(!timeline)throw std::runtime_error("Cannot commit timeline header");
        std::uint64_t last_stamp=0;
        while(summary.frames<options.frames&&!interrupted) {
            const auto frame_start=Clock::now();
            const double acquire_cpu_start=cpuMs();
            const auto acquired=Clock::now();raw::Capture capture;
            try { capture=camera.next([] { return !interrupted; }); }
            catch(const raw::FatalCameraError&) { throw; }
            catch(const std::exception&) { if(interrupted)break;throw; }
            const double acquire_ms=elapsed(acquired);
            const double acquire_cpu_ms=cpuMs()-acquire_cpu_start;
            const auto acquire_rss_kib=memoryKib("VmRSS"),acquire_peak_rss_kib=memoryKib("VmHWM");
            const auto id=capture.left.capture;
            raw::sameCapture(id,capture.right.capture,"record left/right");
            raw::sameCapture(id,capture.disparity.capture,"record left/Q8");
            if(id.ns<=last_stamp)throw std::runtime_error("Duplicate/backward SDK timestamp; capture not recorded");
            last_stamp=id.ns;
            const auto writing=Clock::now();std::ostringstream name;
            name<<"frame-"<<std::setw(6)<<std::setfill('0')<<(summary.frames+1);
            const auto frame_dir=active/name.str();
            if(!fs::create_directory(frame_dir))throw std::runtime_error("Cannot create new frame directory");
            write(frame_dir/"left.nv12",capture.left);write(frame_dir/"right.nv12",capture.right);
            write(frame_dir/"q8.bin",capture.disparity);
            const double write_ms=elapsed(writing);
            manifest<<"{\"sequence\":"<<(summary.frames+1)<<",\"frame_id\":"<<id.id
                <<",\"timestamp_ns\":"<<id.ns<<",\"width\":640,\"height\":544,\"left\":"
                <<quote(name.str()+"/left.nv12")<<",\"right\":"<<quote(name.str()+"/right.nv12")
                <<",\"disparity\":"<<quote(name.str()+"/q8.bin")
                <<",\"left_format\":\"nv12\",\"right_format\":\"nv12\",\"disparity_format\":\"q8_le\","
                <<"\"capture_acquire_ms\":"<<acquire_ms<<",\"capture_cpu_ms\":"<<acquire_cpu_ms
                <<",\"capture_cpu_percent\":"<<(acquire_ms>0?100*acquire_cpu_ms/acquire_ms:0)
                <<",\"capture_rss_kib\":"<<acquire_rss_kib<<",\"capture_peak_rss_kib\":"<<acquire_peak_rss_kib
                <<",\"online_cpus\":"<<summary.online_cpus<<",\"write_ms\":"<<write_ms
                <<",\"total_wall_ms\":"<<elapsed(frame_start)
                <<",\"recording_elapsed_ms\":"<<elapsed(started)<<"}\n";
            manifest.flush();if(!manifest)throw std::runtime_error("Cannot commit manifest record");
            // Files are closed successfully before either index is committed.
            // Both rows retain the original SDK identity; CSV paths are relative
            // to this capture directory. A commit failure produces failed.json,
            // never a successful recording, and does not advance frames_recorded.
            timeline<<id.ns<<','<<name.str()<<"/left.nv12,"<<name.str()<<"/q8.bin,"
                    <<name.str()<<"/right.nv12,"<<id.id<<'\n';
            timeline.flush();if(!timeline)throw std::runtime_error("Cannot commit timeline record");
            ++summary.frames;summary.acquire_ms+=acquire_ms;summary.write_ms+=write_ms;
            summary.acquire_cpu_ms+=acquire_cpu_ms;
            summary.rss_kib=std::max(summary.rss_kib,acquire_rss_kib);
            summary.peak_rss_kib=std::max(summary.peak_rss_kib,acquire_peak_rss_kib);
            std::cout<<summary.frames<<'/'<<options.frames<<" frame_id="<<id.id<<" timestamp_ns="<<id.ns
                     <<" acquire_ms="<<acquire_ms<<'\n';
        }
        finish(timeline,timeline_path);finish(manifest,manifest_path);camera.close();
        summary.wall_ms=elapsed(started);summary.cpu_ms=cpuMs()-cpu_start;
        stats(active,summary,{});
        std::cout<<"Recorded "<<summary.frames<<" identical left/right/Q8 captures: "<<active.u8string()<<'\n';
        return interrupted?130:0;
    } catch(const std::exception& error) {
        summary.wall_ms=elapsed(started);summary.cpu_ms=cpuMs()-cpu_start;
        std::cerr<<"Capture error: "<<error.what()<<'\n';
        if(!active.empty())try { stats(active,summary,error.what()); }
        catch(const std::exception& nested) { std::cerr<<nested.what()<<'\n'; }
        return 1;
    }
}
