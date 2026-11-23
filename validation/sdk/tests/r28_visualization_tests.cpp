#include "output_features.hpp"
#include "visualization.hpp"
#include "image_input.hpp"
#include "perception/preprocess.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs=std::filesystem;
namespace rt=perception::runtime;
void expect(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
struct Harness {
    rt::FeatureRegistry registry;
    r28::Status last;
    std::unique_ptr<rt::TaskGraph> graph;
    explicit Harness(const fs::path& dir,rt::Parameters parameters={}) {
        r28::registerOutputFeatures(registry,[this](const r28::Status& s){last=s;});
        rt::TaskSpec task;task.id="draw";task.features={"output.visualize"};
        task.parameters={{"dir",dir.u8string()},{"every","1"},{"layers","raw,detection,segmentation,overlay,disparity"},{"require_identity","true"}};
        for(auto& p:parameters)task.parameters[p.first]=p.second;
        rt::TaskGraphConfig config;config.tasks={task};graph=std::make_unique<rt::TaskGraph>(config,registry);
    }
};
void populate(rt::ArtifactStore& store,example::Image& image,bool mismatch=false,bool bad_size=false) {
    const perception::FrameInfo frame{7,123456};
    store.put("input.image",image.view());store.put("input.frame",frame);
    perception::DetectionResult det;det.width=image.width;det.height=image.height;
    store.put("ai.detection",det);store.put("ai.detection.frame",mismatch?perception::FrameInfo{8,123456}:frame);
    perception::SegmentationResult seg;seg.width=image.width+(bad_size?1:0);seg.height=image.height;
    seg.mask.assign(std::size_t(seg.width)*seg.height,1);store.put("ai.segmentation",std::move(seg));store.put("ai.segmentation.frame",frame);
}
int main(){fs::path dir;try{
    dir=fs::temp_directory_path()/("r28-vis-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    example::Image image;image.width=8;image.height=6;image.stride=8;image.format=perception::PixelFormat::NV12;
    image.bytes.assign(72,128);std::fill_n(image.bytes.begin(),48,std::uint8_t(81));const auto before=image.bytes;
    std::uint32_t seed=12345;
    for(int format=0;format<4;++format)for(int trial=0;trial<8;++trial) {
        example::Image varied;varied.width=8;varied.height=6;
        varied.format=static_cast<perception::PixelFormat>(format);
        const bool yuv=format>=2;varied.stride=yuv?12:30;
        const auto plane=varied.stride*varied.height;
        varied.bytes.resize(yuv?plane+plane/2:plane);
        for(auto& pixel:varied.bytes){seed=1664525*seed+1013904223;pixel=std::uint8_t(seed>>24);}
        const auto old=perception::prepare(varied.view(),varied.width,varied.height,false,0).rgb;
        std::vector<std::uint8_t> current;demo_vis::convertRgbInto(varied.view(),current);
        expect(old==current,"output-only direct RGB conversion differs from original renderer");
    }
    Harness test(dir/"full");expect(!fs::exists(dir),"registration caused filesystem side effects");
    rt::ArtifactStore store;populate(store,image);
    std::vector<std::uint16_t> depth(48,256);depth[0]=0;
    store.put("input.disparity",perception::fusion::DisparityView{depth.data(),96,16,8,6,perception::fusion::DisparityType::UInt16,256});
    expect(test.graph->run(store).ok(),"visualization DAG failed");
    const auto result=store.get<r28::OutputVisualization>("output.visualization");
    expect(result->frame.id==7&&result->frame.timestamp_ns==123456&&result->files.size()==5,"visualization result identity/layers differ");
    expect(result->output_ms>=result->render_ms&&result->output_ms>=result->encode_write_ms,"output timing scope invalid");
    for(const auto& file:result->files){auto decoded=example::loadImage(file.path,{});expect(decoded.width==8&&decoded.height==6,"saved PNG dimensions differ");}
    expect(image.bytes==before,"visualization modified inference input");
    std::string json;
    {std::ifstream meta(result->metadata_path);json.assign(std::istreambuf_iterator<char>(meta),{});}
    expect(json.find("\"frame_id\":7")!=std::string::npos&&json.find("\"timestamp_ns\":123456")!=std::string::npos,"metadata is not tied to frame");
    Harness cadence(dir/"cadence",{{"every","2"},{"layers","overlay"}});
    rt::ArtifactStore first;populate(first,image);expect(cadence.graph->run(first).ok(),"skipped invocation failed");
    expect(first.get<r28::OutputVisualization>("output.visualization")->skipped&&!fs::exists(dir/"cadence"),"cadence did not skip output side effects");
    rt::ArtifactStore second;populate(second,image);expect(cadence.graph->run(second).ok(),"Nth invocation was not saved");
    expect(!second.get<r28::OutputVisualization>("output.visualization")->skipped,"Nth invocation was skipped");
    Harness stale(dir/"stale",{{"layers","overlay"}});rt::ArtifactStore stale_store;populate(stale_store,image,true);
    expect(!stale.graph->run(stale_store).ok()&&stale.last.code==r28::OutputFailed&&stale.last.feature=="output.visualize"&&stale.last.task=="draw"&&stale.last.frame.id==7,"stale result/error identity accepted");
    expect(!fs::exists(dir/"stale")&&!stale_store.contains("output.visualization"),"failed identity published output");
    Harness wrong(dir/"wrong",{{"layers","overlay"}});rt::ArtifactStore wrong_store;populate(wrong_store,image,false,true);
    expect(!wrong.graph->run(wrong_store).ok()&&!fs::exists(dir/"wrong"),"mismatched mask dimensions accepted");
    Harness missing(dir/"missing",{{"layers","detection"}});rt::ArtifactStore empty;empty.put("input.image",image.view());empty.put("input.frame",perception::FrameInfo{9,100});
    expect(!missing.graph->run(empty).ok()&&missing.last.code==r28::OutputFailed,"missing requested detections accepted");
    Harness raw(dir/"raw",{{"layers","raw,overlay"}});rt::ArtifactStore raw_store;raw_store.put("input.image",image.view());raw_store.put("input.frame",perception::FrameInfo{10,100});
    expect(raw.graph->run(raw_store).ok(),"empty result raw/overlay was not accepted");
    const auto raw_output=raw_store.get<r28::OutputVisualization>("output.visualization");
    expect(example::loadImage(raw_output->files[0].path,{}).bytes==example::loadImage(raw_output->files[1].path,{}).bytes,"empty overlay differs from raw");
    {std::ofstream blocked(dir/"file");blocked<<"file";}
    Harness failure(dir/"file"/"child",{{"layers","raw"}});rt::ArtifactStore failed;populate(failed,image);
    expect(!failure.graph->run(failed).ok()&&failure.last.code==r28::OutputFailed&&!failed.contains("output.visualization"),"save failure was not returned");
    fs::remove_all(dir);std::cout<<"PASS same-frame visualization, empty results, cadence, shape/identity rejection and OUTPUT_FAILED propagation\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';if(!dir.empty()){std::error_code ignored;fs::remove_all(dir,ignored);}return 1;}}
