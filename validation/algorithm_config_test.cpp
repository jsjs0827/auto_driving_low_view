#include "native_config_detail.hpp"
#include "algorithm_catalog.hpp"
#include "product_graph.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
namespace {
using namespace v3;
using namespace v3::detail;
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
void save(const Path& p,const Json& value){std::ofstream out(p);out<<v3::jsonText(value);if(!out)throw std::runtime_error("Cannot write fixture");}
template<class F>void rejected(F action,const std::string& expected){try{action();}catch(const std::exception& e){require(std::string(e.what()).find(expected)!=std::string::npos,e.what());return;}throw std::runtime_error("Expected rejection: "+expected);}
void env(const char* key,const std::string& value){
#ifdef _WIN32
_putenv_s(key,value.c_str());
#else
setenv(key,value.c_str(),1);
#endif
}
}
int main(int argc,char** argv){
 try{
    require(argc==3,"algorithm_config_test PROJECT_ROOT FIXTURE_DIR");
    const auto root=Path(argv[1]);const auto temp=std::filesystem::absolute(Path(argv[2]));
    const auto suffix=std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::filesystem::create_directories(temp/"scheduling");
    const auto copy_options=std::filesystem::copy_options::recursive|std::filesystem::copy_options::overwrite_existing;
    // Only configuration is needed; copying vendored source/build trees also
    // exceeds Windows path limits and obscures the contract under test.
    for(const auto& feature:std::filesystem::directory_iterator(root/"features")) {
        if(!feature.is_directory()||!std::filesystem::is_regular_file(feature.path()/"config.json"))continue;
        const auto destination=temp/"features"/feature.path().filename();
        std::filesystem::create_directories(destination);
        std::filesystem::copy_file(feature.path()/"config.json",destination/"config.json",std::filesystem::copy_options::overwrite_existing);
    }
    std::filesystem::copy_file(root/"features/inference/cpu_profile.json",temp/"features/inference/cpu_profile.json",std::filesystem::copy_options::overwrite_existing);
    for(const auto* file:{"camera.json","visualization.json"})std::filesystem::copy_file(root/file,temp/file,std::filesystem::copy_options::overwrite_existing);
    std::filesystem::create_directories(temp/"sdk");
    std::filesystem::copy(root/"scheduling/config/sdk_templates",temp/"sdk/config",copy_options);
    std::filesystem::copy(root/"scheduling/config/profiles",temp/"sdk/profiles",copy_options);
    std::filesystem::copy_file(root/"validation/fixtures/scene_cls_rgb_mean.onnx",temp/"model.onnx",std::filesystem::copy_options::overwrite_existing);
    env("PERCEPTION_PROJECT_ROOT",temp.u8string());env("SDK_ROOT",(temp/"sdk").u8string());
    save(temp/"features/scene_cls/config.json",object());
    auto request=r33::json::JsonParser(R"({"schema_version":5,"input":{"mode":"ros","disparity_source":"original_q8"},"sdk":{"root":"${SDK_ROOT}"},"tasks":{"one":{"feature":"scene_cls","parameters":{"model":"model.onnx","backend":"cpu","top_k":1}},"two":{"feature":"inference","algorithm":"scene_cls","parameters":{"model":"model.onnx","backend":"cpu","top_k":2}}},"dag":{"parallel":["one","two"]}})","fixture").parse();
    save(temp/"pipeline.json",request);
    const auto prepared=v3::prepare(temp/"pipeline.json",temp/("run-"+suffix));
    std::filesystem::copy_file(temp/("run-"+suffix)/"effective_full.json",temp/"effective_full.json",std::filesystem::copy_options::overwrite_existing);
    const auto algorithms=get(prepared.full,"algorithms");require(algorithms.object.size()==2,"Independent instances were lost");
    require(!get(get(prepared.full,"inference"),"enabled").boolean,"Classification enabled legacy det/seg");
    require(std::filesystem::u8path(get(get(get(algorithms,"one"),"parameters"),"model").text).is_absolute(),"Model path is not absolute");
    require(get(get(get(algorithms,"two"),"parameters"),"top_k").text=="2","Per-instance override lost");
    auto bad=request;bad.object["tasks"].object["one"].object["parameters"].object["topk"]=integer(1);save(temp/"bad.json",bad);
    rejected([&]{v3::prepare(temp/"bad.json",temp/"bad-run");},"Unknown scene_cls parameter");
    bad=request;bad.object["tasks"].object["one"].object["parameters"].object["model"]=str("missing.onnx");save(temp/"bad.json",bad);
    rejected([&]{v3::prepare(temp/"bad.json",temp/"missing-run");},"file does not exist");
    bad.object["tasks"].object["one"].object["enabled"]=boolean(false);save(temp/"disabled.json",bad);
    const auto disabled=v3::prepare(temp/"disabled.json",temp/("disabled-run-"+suffix));
    require(get(disabled.full,"algorithms").object.size()==1,"Disabled instance still requires a model");
    auto mismatch=algorithms;mismatch.object["one"].object["algorithm"]=str("other");
    rejected([&]{perception::algorithms::validateConfigurations(prepared.graph,mismatch,temp);},"must match graph task");
    auto graph=r33::json::JsonParser(R"({"nodes":[{"id":"a","feature":"scene_cls"},{"id":"b","feature":"scene_cls"},{"id":"p","feature":"publish","depends_on":["a"]}]})","bad join").parse();
    rejected([&]{r33::full::parseProductGraph(graph);},"ancestor task b");
    std::cout<<"Algorithm configuration contracts passed; SDK config: "<<(temp/"effective_full.json")<<'\n';
    return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
