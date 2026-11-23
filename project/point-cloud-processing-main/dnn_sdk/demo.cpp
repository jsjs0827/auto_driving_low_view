#include "percep_main/dnn_config_json.hpp"
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
int main(int argc,char** argv) {
    try {
        if(argc<3||std::string(argv[1])=="--help") {
            std::cout<<"Usage: demo start_config.json frame.yuv [width=640 height=544]\n"
                     <<"Uses dnn.parallel_det_seg, detect.classes_num/class_names and seg.num_classes/class_names.\n";
            return argc>1&&std::string(argv[1])=="--help"?0:2;
        }
        auto config=perception::LoadDnnConfigJson(argv[1]);
        const int width=argc>3?std::stoi(argv[3]):640,height=argc>4?std::stoi(argv[4]):544;
        if(width<2||width>4096||height<2||height>4096||width%2||height%2)throw std::invalid_argument("Invalid image dimensions");
        std::ifstream file(argv[2],std::ios::binary);
        if(!file)throw std::runtime_error("Cannot open YUV frame");
        std::vector<uint8_t> frame((std::istreambuf_iterator<char>(file)),{});
        const size_t rows=size_t(height)*3/2;
        if(frame.size()%rows||frame.size()/rows<size_t(width)||(frame.size()/rows)%2)
            throw std::runtime_error("Frame is not a complete YUV420 stride layout");
        const bool nv12=config.left_yuv_format!="i420"&&config.left_yuv_format!="yuv420p";
        perception::HorizonDnnRunner runner;
        if(!runner.init(config))throw std::runtime_error("DNN initialization failed; check model tensors against configured class counts/names");
        std::vector<perception::DnnDetection> boxes;std::vector<uint8_t> mask;
        perception::DnnInferTiming timing;
        if(!runner.inferNv12(frame.data(),width,height,int(frame.size()/rows),frame.size(),nv12,
                            config.detect.enabled?&boxes:nullptr,config.seg.enabled?&mask:nullptr,nullptr,&timing))
            throw std::runtime_error("Inference failed");
        for(const auto& d:boxes)std::cout<<d.class_id<<' '<<d.class_name<<' '<<d.confidence<<' '
            <<d.x1<<' '<<d.y1<<' '<<d.x2<<' '<<d.y2<<'\n';
        std::cout<<"seg class_names="<<runner.config().seg.class_names<<" mask_bytes="<<mask.size()
                 <<" parallel_det_seg="<<runner.config().parallel_det_seg<<'\n';
        if(!mask.empty()){std::ofstream out("mask.bin",std::ios::binary);out.write(reinterpret_cast<const char*>(mask.data()),mask.size());if(!out)throw std::runtime_error("Cannot write mask.bin");}
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
