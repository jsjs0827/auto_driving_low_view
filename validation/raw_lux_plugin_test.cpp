#include "features/raw_lux/include/raw_lux.hpp"
#include "features/raw_lux/backend/include/percep_main/raw_lux_runner.hpp"
#include "perception/frame_input.hpp"
#include "json.hpp"
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>
namespace {
int checks=0;
void require(bool ok,const std::string& why){++checks;if(!ok)throw std::runtime_error(why);}
std::vector<std::uint8_t> loadRawFixture(const std::string& path){std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f),"Cannot open RAW fixture "+path);require(f.tellg()==2785280,"Wrong RAW fixture size");f.seekg(0);std::vector<std::uint8_t> v(2785280);f.read(reinterpret_cast<char*>(v.data()),v.size());require(bool(f),"Incomplete RAW fixture");return v;}
class Neighbor final:public perception::runtime::Feature{void execute(perception::runtime::FeatureContext& c)override{c.set("neighbor.completed",true);}};
}
int main(int argc,char** argv){try{
    using namespace perception;namespace rt=perception::runtime;using namespace r33::json;
    require(argc==2,"raw_lux_plugin_test RAW_EXAMPLES_DIRECTORY");
    auto left=loadRawFixture(std::string(argv[1])+"/left.raw"),right=loadRawFixture(std::string(argv[1])+"/right.raw");
    std::vector<std::uint8_t> yuv(12,128);psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    auto& f=frame.left;f.struct_size=sizeof(f);f.abi_version=PSDK_ABI_VERSION;f.format=PSDK_FORMAT_NV12;
    f.width=4;f.height=2;f.stride=4;f.data=yuv.data();f.bytes=yuv.size();f.frame_id=7;f.timestamp_ns=1000;
    psdk_raw_stereo_input input{};input.struct_size=sizeof(input);input.abi_version=PSDK_RAW_INPUT_ABI_VERSION;input.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
    auto& l=input.left;l.struct_size=sizeof(l);l.abi_version=PSDK_RAW_INPUT_ABI_VERSION;l.width=1088;l.height=1280;l.stride_bytes=2176;
    l.format=PSDK_RAW_FORMAT_UINT16_LE;l.valid_bits=10;l.bayer=PSDK_RAW_BAYER_BGGR;l.data=left.data();l.bytes=left.size();l.frame_id=7;l.timestamp_ns=1000;
    input.right=l;input.right.data=right.data();auto& o=input.left_org;o.struct_size=sizeof(o);o.abi_version=PSDK_RAW_INPUT_ABI_VERSION;
    o.present=1;o.exposure_ms=10;o.analog_gain=2.95365238;o.frame_id=7;o.timestamp_ns=1000;input.right_org=o;
    algorithms::Invocation invocation;invocation.algorithm="raw_lux";invocation.task_id="lux";invocation.inputs={{"raw","input.raw_light"}};
    invocation.parameters=JsonParser("{\"model_name\":\"raw_hist_org_logistic\",\"metadata_source\":\"org_proxy\",\"include_features\":true}","RAW lux test parameters").parse();
    rt::FeatureRegistry registry;registry.add("test.raw_lux",[&](const rt::Parameters&){return raw_lux_feature::create(invocation);});
    registry.add("test.neighbor",[](const rt::Parameters&){return std::make_unique<Neighbor>();});
    rt::TaskGraphConfig config;rt::TaskSpec lux;lux.id="lux";lux.features={"test.raw_lux"};config.tasks.push_back(lux);
    rt::TaskSpec neighbor;neighbor.id="neighbor";neighbor.features={"test.neighbor"};config.tasks.push_back(neighbor);
    config.result_artifacts={algorithms::resultKey("lux"),"neighbor.completed"};rt::TaskGraph graph(config,registry);
    const auto run=[&](){auto bundle=perception::input::FrameBundle::borrowed(frame);rt::ArtifactStore store;bundle.seed(store);require(graph.run(store).ok(),"RAW task aborted neighboring task");require(*store.get<bool>("neighbor.completed"),"Neighbor did not execute");return JsonParser(store.get<algorithms::Output>(algorithms::resultKey("lux"))->json,"RAW result").parse();};
    auto absent=run();require(!boolean(required(absent,"valid"))&&string(required(absent,"status"))=="missing_input","Missing RAW did not return explicit status");
    frame.raw_light=&input;const auto result=run();require(boolean(required(result,"valid")),"Valid RAW rejected");
    require(string(required(result,"predicted_class"))=="LOW_LIGHT","Unexpected fixture RAW class");
    require(string(required(result,"metadata_source"))=="org_proxy","ORG proxy was relabeled");
    require(!boolean(required(result,"stereo_same_exposure_verified")),"Unverified RAW exposure was falsely verified");
    RawLuxRunner direct;RawLuxConfig direct_config;direct_config.metadata_source="org_proxy";require(direct.init(direct_config),"Direct runner init failed");
    RawLuxImageView dl,dr;dl.data=left.data();dl.bytes=left.size();dl.frame_id=7;dl.timestamp_ns=1000;dl.exposure_us=10000;dl.analog_gain=o.analog_gain;dr=dl;dr.data=right.data();RawLuxResult expected;
    require(direct.predictPair("7",dl,dr,&expected),"Direct RAW fixture prediction failed");
    const auto& probabilities=required(result,"probabilities");
    for(std::size_t i=0;i<3;++i){const double got=number(required(probabilities,expected.class_names[i].c_str()));require(std::memcmp(&got,&expected.probabilities[i],sizeof(double))==0,"Plugin probability differs from direct SDK");}
    const auto& features=required(result,"features");
    for(std::size_t i=0;i<expected.features.size();++i){const double got=number(required(features,expected.feature_names[i].c_str()));require(std::memcmp(&got,&expected.features[i],sizeof(double))==0,"Plugin feature differs from direct SDK");}
    const auto invalid=[&](const char* reason){const auto v=run();require(!boolean(required(v,"valid")),reason);require(string(required(v,"status"))=="invalid_input","Invalid RAW status missing");};
    input.left_org.present=0;invalid("Missing ORG accepted");input.left_org.present=1;
    input.left_org.exposure_ms=0;invalid("Zero exposure accepted");input.left_org.exposure_ms=10;
    input.left_org.exposure_ms=std::numeric_limits<double>::quiet_NaN();invalid("NaN exposure accepted");input.left_org.exposure_ms=10;
    input.right_org.analog_gain=-1;invalid("Negative gain accepted");input.right_org.analog_gain=o.analog_gain;
    input.left_org.timestamp_ns++;invalid("Mismatched ORG timestamp accepted");input.left_org.timestamp_ns--;
    input.right.frame_id++;invalid("Mismatched RAW eye frame accepted");input.right.frame_id--;
    f.timestamp_ns++;invalid("Different current DAG frame accepted");f.timestamp_ns--;
    input.right.data=left.data();invalid("Same RAW buffer reused for both eyes");input.right.data=right.data();
    l.valid_bits=12;invalid("Wrong RAW bit depth accepted");l.valid_bits=10;
    l.bayer=0;invalid("Unknown Bayer pattern accepted");l.bayer=PSDK_RAW_BAYER_BGGR;
    const auto save=left[3];left[3]=255;invalid("Out-of-range DN silently accepted");left[3]=save;
    input.left.data=nullptr;input.left.bytes=0;const auto missing=run();require(string(required(missing,"status"))=="missing_input","Missing eye did not return status");input.left.data=left.data();input.left.bytes=left.size();
    input.metadata_source=PSDK_RAW_METADATA_UNKNOWN;invalid("Unknown metadata source accepted");input.metadata_source=PSDK_RAW_METADATA_ORG_PROXY;
    auto wrong=invocation;wrong.parameters.object["metadata_source"].text="verified_raw";
    try{raw_lux_feature::create(wrong);require(false,"Unsupported model source accepted");}catch(const std::invalid_argument&){++checks;}
    std::cout<<"PASS "<<checks<<" RAW-lux plugin checks; real fixture/direct SDK parity, units, missing-data and neighboring task isolation passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
