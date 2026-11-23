#include "perception_slip_sdk.h"
#include "perception_temporal_sdk.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>
namespace {
namespace fs=std::filesystem;
void need(bool value,const char*message){if(!value)throw std::runtime_error(message);}
void ok(std::uint32_t code,const psdk_status&s){if(code)throw std::runtime_error(std::to_string(code)+":"+s.feature+":"+s.task+":"+s.detail);}
using Result=std::unique_ptr<psdk_full_result,decltype(&psdk_full_result_release)>;
struct Engine {
    psdk_full_engine* value=nullptr;
    explicit Engine(const fs::path&path,const psdk_full_hooks* hooks=nullptr){const auto source=path.u8string();psdk_full_config config{sizeof(config),PSDK_FULL_ABI_VERSION,source.c_str(),nullptr,nullptr,hooks};psdk_status error{};ok(psdk_full_create(&config,&value,&error),error);}
    ~Engine(){psdk_full_destroy(&value,nullptr);}
    Engine(const Engine&)=delete;Engine&operator=(const Engine&)=delete;
};
struct Fixture {
    static constexpr unsigned width=640,height=544;
    std::vector<std::uint8_t> image=std::vector<std::uint8_t>(width*height*3/2,std::uint8_t(128));
    std::vector<std::uint16_t> disparity=std::vector<std::uint16_t>(width*height,std::uint16_t(2048));
    psdk_full_frame frame{};psdk_disparity_q8 q{};
    Fixture(){
        std::uint32_t random=123456789;for(unsigned i=0;i<width*height;++i){random^=random<<13;random^=random>>17;random^=random<<5;image[i]=std::uint8_t(random&255);}
        frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
        frame.left={sizeof(psdk_frame),PSDK_ABI_VERSION,PSDK_FORMAT_NV12,width,height,width,image.data(),image.size(),0,0};
        frame.intrinsics={width,height,320,320,319.5,271.5,32000};
        q={sizeof(q),PSDK_FULL_ABI_VERSION,width,height,width*2,0,disparity.data(),disparity.size()*2,0,0};frame.disparity=&q;
    }
    void capture(std::uint64_t id,std::uint64_t stamp){frame.left.frame_id=q.frame_id=id;frame.left.timestamp_ns=q.timestamp_ns=stamp;}
};
Result process(Engine&e,Fixture&f,std::uint64_t id,std::uint64_t stamp){f.capture(id,stamp);psdk_full_result* result=nullptr;psdk_status error{};ok(psdk_full_process(e.value,&f.frame,&result,&error),error);need(result,"successful SDK process returns owned result");return Result(result,psdk_full_result_release);}
psdk_slip_view slip(const Result&r){psdk_slip_view v{};v.struct_size=sizeof(v);v.abi_version=PSDK_SLIP_ABI_VERSION;psdk_status error{};ok(psdk_full_result_get_slip_view(r.get(),&v,&error),error);return v;}
psdk_temporal_view temporal(const Result&r){psdk_temporal_view v{};v.struct_size=sizeof(v);v.abi_version=PSDK_TEMPORAL_ABI_VERSION;psdk_status error{};ok(psdk_full_result_get_temporal_view(r.get(),&v,&error),error);return v;}
psdk_motion_sample sample(std::uint64_t stamp,double speed=.4){psdk_motion_sample s{};s.struct_size=sizeof(s);s.abi_version=PSDK_SLIP_ABI_VERSION;s.timestamp_ns=stamp;s.linear_speed_mps=speed;return s;}
void push(Engine&e,const std::vector<psdk_motion_sample>&samples){psdk_status error{};ok(psdk_full_push_motion_samples(e.value,samples.data(),std::uint32_t(samples.size()),&error),error);}
void wheel(Engine&e,std::uint64_t begin=1000000000,std::uint64_t end=2000000000){std::vector<psdk_motion_sample> samples;for(auto stamp=begin;stamp<=end;stamp+=20000000)samples.push_back(sample(stamp));push(e,samples);}
std::string read(const fs::path&path){std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot read fixture configuration");return {std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};}
std::string replaced(std::string text,const std::string&from,const std::string&to){const auto at=text.find(from);need(at!=std::string::npos,"fixture replacement exists");need(text.find(from,at+from.size())==std::string::npos,"fixture replacement is unique");text.replace(at,from.size(),to);return text;}
struct TemporaryConfigs {
    fs::path directory;std::vector<fs::path> files;
    TemporaryConfigs(){const auto suffix=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());directory=fs::temp_directory_path()/fs::u8path("r36-slip-sdk-contract-"+suffix);need(fs::create_directory(directory),"create new isolated fixture directory");}
    fs::path write(const char* name,const std::string&text){const auto path=directory/fs::u8path(name);need(!fs::exists(path),"do not replace existing fixture file");std::ofstream f(path,std::ios::binary);f<<text;need(bool(f),"save fixture variant");files.push_back(path);return path;}
    ~TemporaryConfigs(){std::error_code error;for(const auto&file:files)fs::remove(file,error);fs::remove(directory,error);} // only exact files created above; never recursive
};
void successAndLifetime(const fs::path&config){
    Fixture f;Engine e(config);wheel(e);
    auto first=process(e,f,1,1000000000);need(slip(first).state==PSDK_SLIP_INITIALIZING,"first capture initializes motion history");
    auto second=process(e,f,2,1100000000);auto v=slip(second);need(v.state==PSDK_SLIP_SUSPECTED&&v.visual_valid&&v.reference_valid,"stationary camera / moving wheel is suspected after 100ms");
    need(v.previous_frame_id==1&&v.frame_id==2&&v.start_timestamp_ns==1000000000&&v.end_timestamp_ns==1100000000,"motion view preserves real capture interval");
    need(std::abs(v.visual_forward_m)<1e-8&&std::abs(v.reference_forward_m-.04)<1e-8&&std::abs(v.slip_ratio-1)<1e-8,"visual static geometry and wheel integral have correct units");
    auto third=process(e,f,3,1200000000);v=slip(third);need(v.state==PSDK_SLIP_SLIPPING&&v.matches>=12&&v.inliers>=10,"slip enters after true 200ms duration");
    need(v.details_json&&std::string(v.details_json).find("slipping")!=std::string::npos,"SDK owns real slip JSON");
    psdk_full_result_view full{};full.struct_size=sizeof(full);full.abi_version=PSDK_FULL_ABI_VERSION;psdk_status error{};ok(psdk_full_result_get_view(third.get(),&full,&error),error);
    const std::string trace=full.task_trace_json;need(trace.find("visual")!=std::string::npos&&trace.find("reference")!=std::string::npos&&trace.find("slip")!=std::string::npos,"actual SDK DAG runs the three motion tasks");
    const auto stable_json=std::string(v.details_json);const auto stable_pixel=temporal(third).frames[2].left.data[0];
    std::fill(f.image.begin(),f.image.end(),std::uint8_t(7));std::fill(f.disparity.begin(),f.disparity.end(),std::uint16_t(0));
    ok(psdk_full_destroy(&e.value,&error),error);v=slip(third);const auto history=temporal(third);
    need(v.state==PSDK_SLIP_SLIPPING&&std::string(v.details_json)==stable_json,"slip result remains valid after engine destruction");
    need(history.frame_count==3&&history.frames[2].left.data[0]==stable_pixel&&history.frames[2].disparity.data[0]==2048,"history retains owned image and Q8 after caller reuse and engine destruction");
    psdk_slip_view bad{};need(psdk_full_result_get_slip_view(third.get(),&bad,&error)==1012,"slip result getter validates ABI");
}
void unavailableCases(const fs::path&config,const fs::path&unconfirmed,const fs::path&clock_unconfirmed){
    Fixture f;
    {Engine e(config);auto a=process(e,f,1,1000000000);auto b=process(e,f,2,1100000000);const auto v=slip(b);need(v.state==PSDK_SLIP_UNAVAILABLE&&v.visual_valid&&!v.reference_valid,"no wheel data is unavailable, never normal/slipping");need(std::isnan(v.reference_speed_mps)&&std::isnan(v.slip_ratio),"unavailable numerical values must not look like zero motion/slip");}
    {Engine e(unconfirmed);wheel(e);auto a=process(e,f,1,1000000000);auto b=process(e,f,2,1100000000);const auto v=slip(b);need(v.state==PSDK_SLIP_UNAVAILABLE&&!v.body_calibration_confirmed,"unconfirmed camera/body calibration cannot decide");need(v.visual_valid&&v.reference_valid&&std::abs(v.reference_speed_mps-.4)<1e-8&&std::abs(v.visual_speed_mps)<1e-8&&std::isnan(v.slip_ratio),"valid motion speeds must survive slip quality gating");}
    {Engine e(clock_unconfirmed);wheel(e);auto a=process(e,f,1,1000000000);auto b=process(e,f,2,1100000000);need(slip(b).state==PSDK_SLIP_UNAVAILABLE&&!slip(b).reference_valid,"unconfirmed clock cannot compare motion");}
    {Engine e(config);push(e,{sample(1000000000),sample(1300000000)});auto a=process(e,f,1,1000000000);auto b=process(e,f,2,1100000000);need(slip(b).state==PSDK_SLIP_UNAVAILABLE&&!slip(b).reference_valid,"internal wheel sample gap cannot be hidden by interpolation");}
    {Engine e(config);wheel(e);auto a=process(e,f,1,1000000000);auto b=process(e,f,2,1400000000);need(slip(b).state==PSDK_SLIP_UNAVAILABLE&&!slip(b).visual_valid,"large visual capture gap cannot decide");}
}
void batchAtomicity(const fs::path&config){
    Engine e(config);Fixture f;push(e,{sample(1000000000)});
    const psdk_motion_sample batch[]={sample(1020000000),sample(1040000000,std::numeric_limits<double>::quiet_NaN())};psdk_status error{};
    need(psdk_full_push_motion_samples(e.value,batch,2,&error)==1002,"invalid batch fails");
    push(e,{sample(1020000000)}); // succeeds only if the first member of failed batch was not committed
    wheel(e,1040000000,1200000000);auto a=process(e,f,1,1000000000);auto b=process(e,f,2,1100000000);need(slip(b).reference_valid&&slip(b).state==PSDK_SLIP_SUSPECTED,"invalid batch leaves previous valid reference buffer intact");
    auto bad=sample(1220000000);bad.reserved[0]=1;need(psdk_full_push_motion_samples(e.value,&bad,1,&error)==1002,"reserved motion fields rejected");
    need(psdk_full_push_motion_samples(e.value,nullptr,1,&error)==1002,"null nonempty batch rejected");need(psdk_full_push_motion_samples(e.value,nullptr,8193,&error)==1002,"oversize batch rejected");
}
struct Publication {bool fail=false;unsigned calls=0;};
std::uint32_t publication(void*user,const psdk_full_result_view*,psdk_status*error){auto&state=*static_cast<Publication*>(user);++state.calls;if(state.fail){error->native_code=-77;constexpr char detail[]="Synthetic publication failure";std::memcpy(error->detail,detail,sizeof(detail));return 0xf123abcdU;}return 0;}
void rollback(const fs::path&config){
    Publication state;psdk_full_hooks hooks{sizeof(hooks),PSDK_FULL_ABI_VERSION,&state,nullptr,publication};Engine e(config,&hooks);Fixture f;wheel(e);
    auto a=process(e,f,1,1000000000);need(slip(a).state==PSDK_SLIP_INITIALIZING,"rollback fixture initial capture");
    const auto fail_capture=[&](std::uint64_t id,std::uint64_t stamp){state.fail=true;f.capture(id,stamp);psdk_full_result* failed=nullptr;psdk_status error{};need(psdk_full_process(e.value,&f.frame,&failed,&error)==0xf123abcdU&&!failed&&error.native_code==-77,"late publication failure returns no committed result");state.fail=false;};
    fail_capture(2,1100000000);need(temporal(a).frame_count==1,"failed graph leaves prior history snapshot immutable");
    auto b=process(e,f,2,1100000000);need(slip(b).state==PSDK_SLIP_SUSPECTED&&temporal(b).frame_count==2,"retry fails neither history identity nor candidate duration");
    fail_capture(3,1200000000);need(slip(b).state==PSDK_SLIP_SUSPECTED&&temporal(b).frame_count==2,"failed slip transition cannot alter prior result");
    auto c=process(e,f,3,1200000000);const auto history=temporal(c);need(slip(c).state==PSDK_SLIP_SLIPPING&&history.frame_count==3,"failed candidate not committed; exact retry advances once");
    need(history.frames[0].left.frame_id==1&&history.frames[1].left.frame_id==2&&history.frames[2].left.frame_id==3,"failed frames do not duplicate history captures");
    need(history.frames[2].sequence==3&&state.calls==5,"only successful graph captures advance history sequence");
}
void generateDemo(const fs::path&destination,const fs::path&project){
    const auto root=fs::absolute(project).lexically_normal();const auto directory=fs::absolute(destination).lexically_normal();
    need(fs::is_regular_file(root/"sdk/include/perception_slip_sdk.h"),"PROJECT_ROOT must identify this v8 project");
    const auto base=read(root/"validation/slip_sdk_contract.json");
    need(!fs::exists(directory)&&!fs::is_symlink(directory),"demo output must be a NEW directory");
    need(fs::create_directories(directory),"create synthetic demo directory");
    const auto write=[&](const fs::path&relative,const std::string&contents){const auto file=directory/relative;need(!fs::exists(file),"never replace demo output");std::ofstream stream(file,std::ios::binary);stream.write(contents.data(),std::streamsize(contents.size()));stream.close();need(bool(stream),"write synthetic fixture file");};
    fs::create_directory(directory/"left");fs::create_directory(directory/"q8");fs::create_directory(directory/"features");
    Fixture fixture;std::string q8bytes(fixture.disparity.size()*2,'\0');for(std::size_t i=0;i<fixture.disparity.size();++i){q8bytes[2*i]=char(fixture.disparity[i]&255);q8bytes[2*i+1]=char(fixture.disparity[i]>>8);}
    std::ostringstream timeline;timeline<<"timestamp_ns,frame_id,left_nv12,q8_u16\n";
    for(unsigned i=0;i<10;++i){std::ostringstream name;name<<std::setfill('0')<<std::setw(6)<<i+1;const auto left="left/"+name.str()+".nv12",q="q8/"+name.str()+".u16";
        write(fs::u8path(left),std::string(reinterpret_cast<const char*>(fixture.image.data()),fixture.image.size()));write(fs::u8path(q),q8bytes);
        timeline<<1000000000ULL+std::uint64_t(i)*100000000ULL<<','<<i+1<<','<<left<<','<<q<<'\n';}
    write("timeline.csv",timeline.str());std::ostringstream motion;motion<<"timestamp_ns,linear_speed_mps,yaw_rate_radps\n";
    for(std::uint64_t t=980000000;t<=1920000000;t+=20000000)motion<<t<<",0.4,0\n";write("motion.csv",motion.str());
    write("intrinsics.json","{\"width\":640,\"height\":544,\"fx\":320,\"fy\":320,\"cx\":319.5,\"cy\":271.5,\"bxf\":32000}\n");
    write("sdk_base.json",base);write("camera.json","{}\n");write("visualization.json","{\"save_root\":\"\",\"save_every_n\":0,\"render_maps\":false}\n");
    for(const auto* name:{"ros","offline","stereo","inference","detection","segmentation","pointcloud","fusion","metrics","recharge"}){const auto folder=directory/"features"/name;fs::create_directory(folder);write(fs::path("features")/name/"config.json","{}\n");}
    for(const auto* name:{"temporal","visual_motion","motion_reference","slip_detection"})fs::create_directory(directory/"features"/name);
    write("features/temporal/config.json","{\"capacity\":3,\"frame_interval\":1,\"max_memory_mb\":16}\n");
    write("features/visual_motion/config.json",R"({"roi":{"x":0,"y":192,"width":640,"height":352},"grid_step":32,"search_radius":4,"max_features":80,"min_matches":12,"min_inliers":10,"max_gap_ms":250,"extrinsics":{"rotation":[0,0,1,-1,0,0,0,-1,0],"translation_m":[0,0,0],"confirmed":true}})" "\n");
    write("features/motion_reference/config.json",R"({"input_csv":"motion.csv","clock_domain":"synthetic_capture","same_clock_confirmed":true,"max_samples":1024,"max_age_ms":10000,"max_sample_gap_ms":100})" "\n");
    write("features/slip_detection/config.json","{\"enter_duration_ms\":200,\"exit_duration_ms\":200}\n");
    write("pipeline.json",R"({
  "schema_version":4,
  "input":{"mode":"recording","disparity_source":"original_q8","timeline_csv":"timeline.csv"},
  "sdk":{"root":"${SDK_ROOT}","base_config":"sdk_base.json"},
  "camera_config":"camera.json",
  "visualization_config":"visualization.json",
  "recording":{"intrinsics_file":"intrinsics.json","limit":10,"warmup_frames":0,"repeat":1,"save_every_n":0,"bpu":{"enabled":false}},
  "graph":{"worker_threads":2,"nodes":[
    {"id":"disp","feature":"disparity","depends_on":[],"enabled":true},
    {"id":"history","feature":"history","depends_on":["disp"],"enabled":true},
    {"id":"visual","feature":"visual_motion","depends_on":["history"],"enabled":true},
    {"id":"reference","feature":"motion_reference","depends_on":["history"],"enabled":true},
    {"id":"slip","feature":"slip_detection","depends_on":["visual","reference"],"enabled":true},
    {"id":"out","feature":"publish","depends_on":["slip"],"enabled":false}
  ]}
})" "\n");
    write("README.md","# Synthetic SDK replay fixture\n\nGenerated by the v8 C++ SDK contract tool. Ten fixed random-texture NV12 640x544 captures at 100 ms intervals; constant positive Q8.8=2048 (8 pixels); simulated measured wheel speed 0.4 m/s at 20 ms intervals. Timestamps share the explicit synthetic_capture clock.\n\nExpected states: initializing, suspected, then slipping. The camera-body transform and shared clock are confirmed only for this synthetic fixture. They must be measured and confirmed again for a real camera. This validates code paths and state transitions, and is not camera hardware, X5 performance or real slip accuracy evidence.\n\nSet SDK_ROOT to your actual SDK installation/build asset directory, then run:\n\n```text\nr36_slip_runner --config <this-directory>/pipeline.json --output <NEW-results-directory>\n```\n\nRead slip.jsonl, metrics.jsonl and complete.json in the result directory. No AI model, proprietary camera SDK or IMU fusion is used.\n");
    std::cout<<"Synthetic fixture generated: "<<directory.u8string()<<"\n10 frames; camera static, simulated wheel 0.4 m/s; no hardware/performance/accuracy evidence.\n";
}
}
int main(int argc,char**argv){try{
    if(argc==4&&std::string(argv[1])=="--generate-demo"){generateDemo(fs::u8path(argv[2]),fs::u8path(argv[3]));return 0;}
    need(argc==2,"Usage: slip_sdk_contract_test validation/slip_sdk_contract.json");need((psdk_full_capabilities()&PSDK_CAP_SLIP)!=0,"SDK advertises implemented slip API");
    const auto source=fs::absolute(fs::u8path(argv[1]));const auto text=read(source);TemporaryConfigs temporary;
    const auto unconfirmed=temporary.write("unconfirmed.json",replaced(text,"\"confirmed\": true","\"confirmed\": false"));
    const auto clock_unconfirmed=temporary.write("clock-unconfirmed.json",replaced(text,"\"same_clock_confirmed\": true","\"same_clock_confirmed\": false"));
    const auto publish_config=temporary.write("publication-failure.json",replaced(text,"\"id\": \"out\", \"feature\": \"publish\", \"depends_on\": [\"slip\"], \"enabled\": false","\"id\": \"out\", \"feature\": \"publish\", \"depends_on\": [\"slip\"], \"enabled\": true"));
    successAndLifetime(source);unavailableCases(source,unconfirmed,clock_unconfirmed);batchAtomicity(source);rollback(publish_config);
    std::cout<<"Slip SDK contract PASS: actual DAG, synthetic static camera / wheel motion, unavailable gates, owned result lifetime, atomic motion batches and late-failure rollback. No camera hardware, X5 speed or real slip-accuracy claim.\n";return 0;
}catch(const std::exception&e){std::cerr<<"Slip SDK contract FAIL: "<<e.what()<<'\n';return 1;}}
