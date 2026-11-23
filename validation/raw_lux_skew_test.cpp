#include "features/raw_lux/include/raw_lux.hpp"
#include "perception/frame_input.hpp"
#include "json.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace {
int checks=0;
void require(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
std::vector<std::uint8_t> fixture(const std::string& path){std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f),"Cannot open RAW fixture");require(f.tellg()==2785280,"Wrong fixture size");f.seekg(0);std::vector<std::uint8_t> b(2785280);f.read(reinterpret_cast<char*>(b.data()),b.size());require(bool(f),"Incomplete RAW fixture");return b;}
class Neighbor:public perception::runtime::Feature {void execute(perception::runtime::FeatureContext& c)override{c.set("neighbor.ok",true);}};
}
int main(int argc,char** argv){try{
    using namespace perception;using namespace r33::json;namespace rt=perception::runtime;
    require(argc==2,"raw_lux_skew_test RAW_EXAMPLES_DIRECTORY");auto l=fixture(std::string(argv[1])+"/left.raw"),r=fixture(std::string(argv[1])+"/right.raw");std::vector<std::uint8_t> yuv(12,128);
    psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    auto& current=frame.left;current.struct_size=sizeof(current);current.abi_version=PSDK_ABI_VERSION;current.format=PSDK_FORMAT_NV12;
    current.width=4;current.height=2;current.stride=4;current.data=yuv.data();current.bytes=yuv.size();current.frame_id=7;current.timestamp_ns=1010200000;
    psdk_raw_stereo_input raw{};raw.struct_size=sizeof(raw);raw.abi_version=PSDK_RAW_INPUT_ABI_VERSION;raw.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
    auto& left=raw.left;left.struct_size=sizeof(left);left.abi_version=PSDK_RAW_INPUT_ABI_VERSION;left.format=PSDK_RAW_FORMAT_UINT16_LE;
    left.width=1088;left.height=1280;left.stride_bytes=2176;left.valid_bits=10;left.bayer=PSDK_RAW_BAYER_BGGR;left.data=l.data();left.bytes=l.size();left.frame_id=7;left.timestamp_ns=1000000000;left.sensor_id=1;
    raw.right=left;raw.right.data=r.data();raw.right.sensor_id=2;auto& org=raw.left_org;org.struct_size=sizeof(org);org.abi_version=PSDK_RAW_INPUT_ABI_VERSION;org.present=1;
    org.exposure_ms=10;org.analog_gain=2.95365238;org.frame_id=7;org.timestamp_ns=current.timestamp_ns;org.sensor_id=1;raw.right_org=org;raw.right_org.sensor_id=2;frame.raw_light=&raw;
    const auto invocation=[&](const std::string& mode,std::uint64_t max,std::uint64_t period){algorithms::Invocation i;i.algorithm="raw_lux";i.task_id="lux";i.inputs={{"raw","input.raw_light"}};
        i.parameters=JsonParser("{\"model_name\":\"raw_hist_org_logistic\",\"metadata_source\":\"org_proxy\",\"include_features\":true,\"cross_stream_sync\":"+quote(mode)+",\"max_cross_stream_skew_ns\":"+std::to_string(max)+",\"capture_frame_period_ns\":"+std::to_string(period)+"}","policy").parse();return i;};
    const auto run=[&](const std::string& mode="sdk_frame_id_bounded",std::uint64_t max=15000000,std::uint64_t period=40000000){const auto i=invocation(mode,max,period);rt::FeatureRegistry registry;
        registry.add("test.lux",[i](const rt::Parameters&){return raw_lux_feature::create(i);});registry.add("test.neighbor",[](const rt::Parameters&){return std::make_unique<Neighbor>();});rt::TaskGraphConfig c;
        rt::TaskSpec a;a.id="lux";a.features={"test.lux"};c.tasks.push_back(a);a.id="neighbor";a.features={"test.neighbor"};c.tasks.push_back(a);c.result_artifacts={algorithms::resultKey("lux"),"neighbor.ok"};
        rt::TaskGraph graph(c,registry);auto bundle=perception::input::FrameBundle::borrowed(frame);rt::ArtifactStore store;bundle.seed(store);require(graph.run(store).ok(),"Skew validation aborted whole DAG");require(*store.get<bool>("neighbor.ok"),"Neighbor did not execute");
        return JsonParser(store.get<algorithms::Output>(algorithms::resultKey("lux"))->json,"policy result").parse();};
    const auto strict=run("exact_timestamps",0,0);require(!boolean(required(strict,"valid")),"Default exact policy accepted cross-stream offset");
    const auto good=run();require(boolean(required(good,"valid")),"Explicit bounded same-ID input rejected");require(string(required(good,"predicted_class"))=="LOW_LIGHT","Timestamp association changed model result");
    const auto& sync=required(good,"capture_association");require(string(required(sync,"association"))=="bounded_unverified","Bounded capture was falsely verified");require(!boolean(required(sync,"cross_stream_sync_verified")),"Cross-stream sync falsely verified");
    require(number(required(required(sync,"raw_left"),"timestamp_ns"))==1000000000,"RAW timestamp was rewritten");require(number(required(required(sync,"org_left"),"timestamp_ns"))==1010200000,"ORG timestamp was rewritten");require(number(required(required(sync,"yuv"),"timestamp_ns"))==1010200000,"YUV timestamp was rewritten");
    require(number(required(required(sync,"observed_skew_ns"),"raw_minus_yuv"))==-10200000,"Observed signed skew not reported");
    require(raw.left.timestamp_ns==1000000000&&org.timestamp_ns==1010200000&&current.timestamp_ns==1010200000,"Input timestamps were mutated");
    current.timestamp_ns=left.timestamp_ns;org.timestamp_ns=raw.right_org.timestamp_ns=left.timestamp_ns;
    const auto exact=run("exact_timestamps",0,0);require(boolean(required(exact,"valid")),"Exact capture regression failed");
    const auto& ep=required(exact,"probabilities");const auto& gp=required(good,"probabilities");for(const auto& item:ep.object){const auto a=number(item.second),b=number(required(gp,item.first.c_str()));require(std::memcmp(&a,&b,sizeof(double))==0,"Policy changed probability bits");}
    const auto reset=[&](){current.frame_id=left.frame_id=raw.right.frame_id=org.frame_id=raw.right_org.frame_id=7;left.timestamp_ns=raw.right.timestamp_ns=1000000000;current.timestamp_ns=org.timestamp_ns=raw.right_org.timestamp_ns=1010200000;};
    const auto rejected=[&](){const auto out=run();require(!boolean(required(out,"valid")),"Invalid bounded association accepted");require(string(required(out,"status"))=="invalid_input","Invalid association lacks status");};
    reset();raw.right.timestamp_ns++;rejected();reset();raw.right_org.timestamp_ns++;rejected();
    reset();raw.right.frame_id++;rejected();reset();left.frame_id=raw.right.frame_id=8;rejected();
    reset();org.timestamp_ns=raw.right_org.timestamp_ns=1020000000;rejected();
    reset();current.timestamp_ns=1014000000;org.timestamp_ns=raw.right_org.timestamp_ns=986000000;rejected(); // each raw offset 14ms, YUV/ORG differs 28ms.
    reset();current.timestamp_ns=org.timestamp_ns=raw.right_org.timestamp_ns=1015000000;require(boolean(required(run(),"valid")),"Inclusive skew boundary rejected");
    current.timestamp_ns=org.timestamp_ns=raw.right_org.timestamp_ns=1015000001;rejected();
    reset();left.timestamp_ns=raw.right.timestamp_ns=0;rejected();
    const auto badConfig=[&](const std::string& mode,std::uint64_t max,std::uint64_t period){++checks;try{auto f=raw_lux_feature::create(invocation(mode,max,period));}catch(const std::invalid_argument&){return;}throw std::runtime_error("Invalid bounded policy configuration accepted");};
    badConfig("sdk_frame_id_bounded",0,40000000);badConfig("sdk_frame_id_bounded",15000000,0);badConfig("sdk_frame_id_bounded",15000000,15000000);badConfig("sdk_frame_id_bounded",20000001,40000000);badConfig("exact_timestamps",1,0);badConfig("unknown",0,0);
    std::cout<<"PASS "<<checks<<" RAW cross-stream policy checks; strict/default, bounded provenance, raw/org stereo exactness and original timestamps passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
