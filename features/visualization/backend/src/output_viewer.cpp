#include "output_features.hpp"
#include "image_input.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fs=std::filesystem;
namespace rt=perception::runtime;
std::vector<std::uint8_t> readBytes(const fs::path& path) {
    std::ifstream input(path,std::ios::binary|std::ios::ate);
    if(!input)throw std::runtime_error("cannot read "+path.u8string());
    const auto end=input.tellg();
    if(end<0 || end>64*1024*1024)throw std::runtime_error("invalid visualization input size");
    std::vector<std::uint8_t> out(std::size_t(end),std::uint8_t(0));
    input.seekg(0);input.read(reinterpret_cast<char*>(out.data()),std::streamsize(out.size()));
    if(!input)throw std::runtime_error("short visualization input read");
    return out;
}
int main(int argc,char** argv) {
    perception::FrameInfo frame;
    try {
        std::map<std::string,std::string> args;
        const std::vector<std::string> names={"--source","--out","--width","--height","--stride","--format",
            "--mask","--detections-tsv","--id","--timestamp","--layers","--mask-alpha",
            "--disparity","--disparity-width","--disparity-height","--disparity-stride","--disparity-scale"};
        for(int i=1;i<argc;++i) {
            const std::string key=argv[i];
            if(key=="--help") {std::cout<<"visualize_saved_frame --source raw.nv12 --width W --height H --stride S --format nv12|i420|rgb|image --out DIR [--mask mask.bin] [--detections-tsv det.tsv] [--id ID --timestamp NS] [--layers raw,detection,segmentation,overlay,disparity] [--disparity raw.u16 --disparity-width W --disparity-height H --disparity-stride BYTES --disparity-scale 256]\n";return 0;}
            if(std::find(names.begin(),names.end(),key)==names.end() || i+1>=argc || !args.emplace(key,argv[++i]).second)
                throw std::invalid_argument("unknown, missing or duplicate visualization option: "+key);
        }
        const auto required=[&](const char* key){const auto it=args.find(key);if(it==args.end()||it->second.empty())throw std::invalid_argument(std::string("required option: ")+key);return it->second;};
        const auto optional=[&](const char* key,const char* fallback){const auto it=args.find(key);return it==args.end()?std::string(fallback):it->second;};
        const auto integer=[](const std::string& text){std::size_t n=0;const auto value=std::stoull(text,&n);if(n!=text.size()||text.empty()||text[0]=='-')throw std::invalid_argument("invalid unsigned numeric option");return value;};
        frame.id=integer(optional("--id","0"));frame.timestamp_ns=integer(optional("--timestamp","0"));
        example::ImageInputOptions options;options.format=optional("--format","nv12");
        if(options.format!="image") {
            const auto width=integer(required("--width")),height=integer(required("--height")),stride=integer(required("--stride"));
            if(width>4096||height>4096||stride>16*1024*1024)throw std::invalid_argument("visualization dimensions exceed limits");
            options.width=int(width);options.height=int(height);options.stride=std::size_t(stride);
        }
        example::Image image;
        if(options.format=="rgb"||options.format=="bgr") {
            image.width=options.width;image.height=options.height;image.stride=options.stride;
            image.format=options.format=="rgb"?perception::PixelFormat::RGB:perception::PixelFormat::BGR;
            image.bytes=readBytes(fs::u8path(required("--source")));
            perception::validateCameraImage(image.view());
        } else image=example::loadImage(required("--source"),options);
        rt::ArtifactStore store;store.put("input.image",image.view());store.put("input.frame",frame);
        std::vector<std::uint8_t> disparity;
        if(args.count("--disparity")) {
            const auto width=integer(optional("--disparity-width",std::to_string(image.width).c_str()));
            const auto height=integer(optional("--disparity-height",std::to_string(image.height).c_str()));
            const auto stride=integer(optional("--disparity-stride",std::to_string(width*2).c_str()));
            if(!width||!height||width>4096||height>4096||stride<width*2||stride>65536)
                throw std::invalid_argument("invalid raw disparity dimensions/stride");
            const auto scale_text=optional("--disparity-scale","256");std::size_t used=0;
            const float scale=std::stof(scale_text,&used);
            if(used!=scale_text.size()||!std::isfinite(scale)||scale<=0)throw std::invalid_argument("invalid disparity scale");
            disparity=readBytes(fs::u8path(args.at("--disparity")));
            if(disparity.size()!=stride*height)throw std::invalid_argument("raw disparity storage size mismatch");
            store.put("input.disparity",perception::fusion::DisparityView{disparity.data(),disparity.size(),std::size_t(stride),int(width),int(height),perception::fusion::DisparityType::UInt16,scale});
            store.put("input.disparity.frame",frame);
        } else if(args.count("--disparity-width")||args.count("--disparity-height")||args.count("--disparity-stride")||args.count("--disparity-scale"))
            throw std::invalid_argument("disparity layout options require --disparity");
        if(args.count("--mask")) {
            perception::SegmentationResult seg;seg.width=image.width;seg.height=image.height;
            seg.mask=readBytes(args.at("--mask"));store.put("ai.segmentation",std::move(seg));store.put("ai.segmentation.frame",frame);
        }
        if(args.count("--detections-tsv")) {
            perception::DetectionResult det;det.width=image.width;det.height=image.height;
            std::ifstream input(args.at("--detections-tsv"));if(!input)throw std::runtime_error("cannot read detection TSV");
            std::string line;
            while(std::getline(input,line)) {
                if(line.empty())continue;std::istringstream fields(line);perception::Detection d{};std::string extra;
                if(!(fields>>d.class_id>>d.score>>d.x1>>d.y1>>d.x2>>d.y2)||(fields>>extra))throw std::invalid_argument("invalid detection TSV row");
                det.detections.push_back(d);
            }
            if(!input.eof())throw std::runtime_error("detection TSV read failed");
            store.put("ai.detection",std::move(det));store.put("ai.detection.frame",frame);
        }
        rt::FeatureRegistry registry; r28::Status status;
        r28::registerOutputFeatures(registry,[&](const r28::Status& s){status=s;});
        rt::TaskSpec task;task.id="visualization";task.features={"output.visualize"};
        task.parameters={{"dir",required("--out")},{"every","1"},{"layers",optional("--layers","overlay")},
            {"mask_alpha",optional("--mask-alpha","0.4")},{"require_identity","true"}};
        rt::TaskGraphConfig config;config.tasks={task};rt::TaskGraph graph(config,registry);
        const auto report=graph.run(store);
        if(!report.ok()) {
            if(status.ok())status={r28::OutputFailed,0,"output.visualize",task.id,"visualization DAG failed",frame};
            throw r28::Failure(status);
        }
        const auto output=store.get<r28::OutputVisualization>("output.visualization");
        std::cout<<output->metadata_path<<'\n';return 0;
    } catch(const std::exception& e) {
        const auto status=r28::statusFromException(e,r28::OutputFailed,"output.visualize","visualization",frame);
        std::cerr<<r28::statusJson(status)<<'\n';return r28::processExit(status.code);
    }
}
