#include "cpu_stages.hpp"
#include "dag_pipeline.hpp"
#include "runtime_status.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace {
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
r29::CpuConfig config(const std::string& dir){
    r29::CpuConfig c;c.dnn.enabled=true;c.dnn.detect.model_path=(std::filesystem::u8path(dir)/"detection.onnx").u8string();c.dnn.seg.model_path=(std::filesystem::u8path(dir)/"segmentation.onnx").u8string();c.dnn.detect.score_threshold=.25f;c.dnn.detect.nms_threshold=.7f;c.dnn.detect.nms_top_k=300;
    for(int i=0;i<2;++i){auto& lane=i?c.segmentation:c.detection;lane.width=i?832:864;lane.height=480;lane.classes=i?3:24;for(int k=0;k<lane.classes;++k)lane.class_names.push_back("cls_"+std::to_string(k));lane.input.name=i?"input":"images";lane.input.color="RGB";lane.input.yuv="bt601_limited_integer";lane.input.scale=1.f/255;lane.input.mean={{0,0,0}};lane.input.stddev={{1,1,1}};lane.output.layout=i?"NCHW":"NHWC";lane.output.format=i?"logits":"raw_dfl";lane.output.reg=16;}
    c.detection.output.names={"580","581","582","583","584","585"};c.segmentation.output.name="output";return c;
}
bool boxesEqual(const std::vector<ai_node_percep_demo::DnnDetection>& a,const std::vector<ai_node_percep_demo::DnnDetection>& b){if(a.size()!=b.size())return false;for(std::size_t i=0;i<a.size();++i)if(a[i].class_id!=b[i].class_id||a[i].confidence!=b[i].confidence||a[i].x1!=b[i].x1||a[i].x2!=b[i].x2||a[i].y1!=b[i].y1||a[i].y2!=b[i].y2)return false;return true;}
std::vector<std::uint8_t> solid(int yy,int u,int v){std::vector<std::uint8_t> a(640*544*3/2,std::uint8_t(yy));for(std::size_t i=640*544;i<a.size();i+=2){a[i]=std::uint8_t(u);a[i+1]=std::uint8_t(v);}return a;}
float floatAt(const std::vector<std::uint8_t>& a,std::size_t n){float f;std::memcpy(&f,a.data()+n*sizeof(float),sizeof(float));return f;}
void strictConfigChecks(const std::filesystem::path& example){
    std::ifstream file(example,std::ios::binary);std::string text((std::istreambuf_iterator<char>(file)),{});require(!text.empty(),"CPU example absent");
    const auto temp=std::filesystem::temp_directory_path()/("cpu-profile-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
    const auto rejects=[&](std::string value){bool refused=false;{std::ofstream out(temp,std::ios::binary);out<<value;}try{(void)r29::loadCpuConfig(temp.u8string());}catch(const r28::Failure& e){refused=e.status().code==2001;}std::filesystem::remove(temp);require(refused,"Strict CPU profile accepted missing/duplicate/unknown normalization field");};
    auto value=text;value.insert(value.find('{')+1,"\"schema_version\":1,");rejects(value);
    value=text;value.insert(value.find('{')+1,"\"guess_normalization\":true,");rejects(value);
    value=text;const auto scale=value.find("\"scale\"");const auto comma=value.find(',',scale);require(scale!=std::string::npos&&comma!=std::string::npos,"Example scale field absent");value.erase(scale,comma-scale+1);rejects(value);
}
}
int main(int argc,char** argv){
    try{
        if(argc!=3)throw std::runtime_error("Usage: cpu_stages_tests FIXTURE_DIR PROFILES_DIR");auto cfg=config(argv[1]);const auto profiles=std::filesystem::u8path(argv[2]);
        auto parsed=r29::loadCpuConfig((profiles/"cpu_contract.example.json").u8string());require(parsed.detection.width==864&&parsed.segmentation.width==832,"Strict CPU profile parse failed");strictConfigChecks(profiles/"cpu_contract.example.json");
        auto bad=cfg;bad.detection.input.name="not_images";bool refused=false;try{r29::CpuStages s(bad);}catch(const r28::Failure& e){refused=e.status().code==1010;}require(refused,"Incorrect ONNX input contract accepted");
        bad=cfg;bad.detection.output.names[0]="missing";refused=false;try{r29::CpuStages s(bad);}catch(const r28::Failure& e){refused=e.status().code==1010;}require(refused,"Missing output name accepted");
        bad=cfg;bad.segmentation.input.stddev[0]=0;refused=false;try{r29::validateCpuConfig(bad);}catch(...){refused=true;}require(refused,"Invalid normalization accepted");
        auto stages=std::make_shared<r29::CpuStages>(cfg);require(stages->metadataJson().find("CPUExecutionProvider")!=std::string::npos,"CPU provider metadata absent");
        r25::Pipeline pipeline((profiles/"dag_exact.json").u8string(),stages);r25::RunOptions options;options.capture=true;options.variant=r22::Variant::CompactDirect;
        std::vector<std::uint8_t> source(640*544*3/2);std::uint32_t seed=19;for(auto& v:source){seed=seed*1664525u+1013904223u;v=std::uint8_t(seed>>24);}
        perception::ImageView image{source.data(),source.size(),640,640,544,perception::PixelFormat::NV12};options.optimized=false;const auto scalar=pipeline.run(image,{91,123456},options);options.optimized=true;const auto exact=pipeline.run(image,{91,123456},options);
        require(scalar.frame.frame.id==91&&exact.frame.frame.id==91,"Same-frame identity failed");
        require(scalar.det.size()==1&&scalar.det[0].class_id==8,"Real raw_dfl ONNX decode produced unexpected detection");require(boxesEqual(scalar.det,exact.det),"CPU original/optimized detection differs");require(scalar.mask()==exact.mask(),"CPU original/optimized segmentation differs");
        for(const char* tag:{"detection","segmentation"}){const auto& a=scalar.audit.at(tag);const auto& b=exact.audit.at(tag);require(a.prepared==b.prepared,"Prepared NV12 bytes differ");require(a.storage==b.storage,"Actual CPU float32 input bytes differ");require(a.output_storage==b.output_storage,"Real ORT outputs differ");require(!a.output_storage.empty(),"Actual model output audit missing");}
        require(exact.audit.at("detection").prepared.size()==622080&&exact.audit.at("segmentation").prepared.size()==599040,"Original model input geometry changed");require(exact.timing.detect_api.submit_ms==0&&exact.timing.detect_api.wait_ms==0&&exact.timing.seg_api.submit_ms==0&&exact.timing.seg_api.wait_ms==0,"CPU falsely reported BPU submit/wait calls");require(exact.features[2].end_ms>exact.features[2].start_ms&&exact.features[5].end_ms>exact.features[5].start_ms,"CPU inference duration is not measured");
        auto red=solid(82,90,240),blue=solid(41,240,110);image.data=red.data();image.bytes=red.size();const auto r=pipeline.run(image,{92,123457},options);image.data=blue.data();const auto b=pipeline.run(image,{93,123458},options);
        require(std::all_of(r.mask().begin(),r.mask().end(),[](std::uint8_t c){return c==0;}),"Input-dependent ONNX segmentation red output wrong");require(std::all_of(b.mask().begin(),b.mask().end(),[](std::uint8_t c){return c==2;}),"Input-dependent ONNX segmentation blue output wrong");require(r.audit.at("detection").output_storage!=b.audit.at("detection").output_storage,"Real detection model output ignores input");
        const auto& normalized=r.audit.at("segmentation").storage;require(floatAt(normalized,0)==1.f,"BT.601 RGB input normalization unexpected");require(floatAt(normalized,832*480*2)==0.f,"CPU tensor layout is not NCHW RGB");
        // Layout adapters change tensor access only, retaining original geometry/math.
        auto other=cfg;other.dnn.detect.model_path=(std::filesystem::u8path(argv[1])/"detection_nchw.onnx").u8string();other.dnn.seg.model_path=(std::filesystem::u8path(argv[1])/"segmentation_nhwc.onnx").u8string();other.detection.output.layout="NCHW";other.segmentation.output.layout="NHWC";
        {auto adapter=std::make_shared<r29::CpuStages>(other);r25::Pipeline variant((profiles/"dag_exact.json").u8string(),adapter);image.data=source.data();image.bytes=source.size();const auto transposed=variant.run(image,{94,123459},options);require(boxesEqual(exact.det,transposed.det)&&exact.mask()==transposed.mask(),"Explicit NCHW detection/NHWC logits layouts change result");}
        other=cfg;other.dnn.detect.model_path=(std::filesystem::u8path(argv[1])/"detection_decoded.onnx").u8string();other.detection.output.format="decoded_xyxy";other.detection.output.layout="NXC";other.detection.output.name="boxes";other.detection.output.names.clear();
        {auto adapter=std::make_shared<r29::CpuStages>(other);r25::Pipeline variant((profiles/"dag_exact.json").u8string(),adapter);const auto decoded=variant.run(image,{95,123460},options);require(decoded.det.size()==2&&decoded.det[0].class_id==8&&decoded.det[1].class_id==9,"decoded_xyxy class-wise NMS failed");require(decoded.det[0].x1>=0&&decoded.det[0].x2<=640&&decoded.det[0].y2<=544,"Decoded boxes did not restore original letterbox coordinates");}
        other=cfg;other.dnn.seg.model_path=(std::filesystem::u8path(argv[1])/"segmentation_nan.onnx").u8string();
        {auto adapter=std::make_shared<r29::CpuStages>(other);r25::Pipeline variant((profiles/"dag_exact.json").u8string(),adapter);r25::Output output;r28::Status status;const auto code=variant.tryRun(image,{96,123461},options,output,status);require(code==1012&&status.code==1012&&status.frame.id==96&&output.mask().empty(),"Nonfinite ONNX output did not preserve full error and suppress partial result");}
        // Retained frame results must survive later model and workspace reuse.
        require(scalar.mask()==exact.mask()&&scalar.frame.frame.id==91,"Previously returned snapshot mutated");
        stages->abortFrame();stages->close();refused=false;try{stages->width(0);}catch(...){refused=true;}require(refused,"Closed sessions remained usable");
        std::cout<<"Real ORT CPU two-branch DAG: original NV12 and float inputs byte-exact, masks pixel-exact, input-dependent inference, identity and lifecycle passed\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
