// Adapted integration for the user-supplied Horizon RAFT-Stereo example.
// The C++ stereo provider and pointcloud pipeline handle calibration, Q8, PLY and ROS.
#include "r36_stereo/horizon_stereo.hpp"
#include "r36_stereo/nv12.hpp"
#include "compare_util.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Pair = std::pair<fs::path, fs::path>;

static std::map<std::string, std::string> arguments(int argc, char** argv) {
    std::map<std::string,std::string> values;
    const std::vector<std::string> allowed={"model_file","left","right","left_dir","right_dir","output","output_dir",
                                          "input_format","input_width","input_height","pre_mode"};
    for (int i=1; i<argc; ++i) {
        std::string arg=argv[i];
        if (arg=="--help" || arg=="-h") {values["help"]="true";continue;}
        if (arg=="--validate_pairs") {values["validate_pairs"]="true";continue;}
        if (arg=="--model_info") {values["model_info"]="true";continue;}
        if (arg.rfind("--",0)!=0) throw std::invalid_argument("Expected --option: "+arg);
        auto separator=arg.find('=');
        const auto name=arg.substr(2,separator==std::string::npos ? std::string::npos : separator-2);
        if (std::find(allowed.begin(),allowed.end(),name)==allowed.end())
            throw std::invalid_argument("Unsupported option: "+name);
        if (values.count(name))throw std::invalid_argument("Duplicate option: "+name);
        std::string value;
        if (separator!=std::string::npos)value=arg.substr(separator+1);
        else if (i+1<argc && std::string(argv[i+1]).rfind("--",0)!=0)value=argv[++i];
        if (value.empty())throw std::invalid_argument("Missing option value: "+name);
        values[name]=value;
    }
    return values;
}

static bool image_file(const fs::path& path) {
    std::string extension=path.extension().string();
    std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char c){return char(std::tolower(c));});
    return extension==".nv12" || extension==".jpg" || extension==".jpeg" || extension==".png" ||
           extension==".bmp" || extension==".webp" || extension==".tif" || extension==".tiff";
}

static std::map<std::string,fs::path> images(const fs::path& directory) {
    if (!fs::is_directory(directory))throw std::invalid_argument("Not an image directory: "+directory.string());
    std::map<std::string,fs::path> result;
    for (const auto& file : fs::directory_iterator(directory)) {
        if (!file.is_regular_file() || !image_file(file.path()))continue;
        const auto stem=file.path().stem().string();
        if (stem.empty() || !result.emplace(stem,file.path()).second)
            throw std::invalid_argument("Duplicate image stem in "+directory.string()+": "+stem);
    }
    if (result.empty())throw std::invalid_argument("No supported images in "+directory.string());
    return result;
}

static std::vector<Pair> pairs(const std::map<std::string,std::string>& args, bool& batch) {
    const bool single=args.count("left") || args.count("right");
    batch=args.count("left_dir") || args.count("right_dir");
    if (single==batch)throw std::invalid_argument("Choose exactly one: --left/--right or --left_dir/--right_dir");
    if (single) {
        if (!args.count("left") || !args.count("right"))throw std::invalid_argument("Both --left and --right are required");
        if (args.count("output_dir"))throw std::invalid_argument("Single mode uses --output");
        return {{args.at("left"),args.at("right")}};
    }
    if (!args.count("left_dir") || !args.count("right_dir"))throw std::invalid_argument("Both image directories are required");
    if (args.count("output"))throw std::invalid_argument("Batch mode uses --output_dir");
    auto left=images(args.at("left_dir")),right=images(args.at("right_dir"));
    std::vector<Pair> result;
    for (const auto& entry:left) {
        if (!right.count(entry.first))throw std::invalid_argument("Missing right image: "+entry.first);
        result.emplace_back(entry.second,right.at(entry.first));
    }
    for (const auto& entry:right)
        if (!left.count(entry.first))throw std::invalid_argument("Missing left image: "+entry.first);
    return result;
}

static int input_dimension(const std::map<std::string,std::string>& args,const char* key) {
    if(!args.count(key))throw std::invalid_argument(std::string("Raw NV12 requires --")+key);
    const auto& text=args.at(key);std::size_t consumed=0;
    const auto value=std::stol(text,&consumed);
    if(consumed!=text.size()||value<2||value>8192||(value&1))
        throw std::invalid_argument(std::string(key)+" must be an even integer in [2,8192]");
    return int(value);
}

static std::pair<cv::Mat,cv::Mat> read_pair(const Pair& pair,const std::map<std::string,std::string>& args) {
    const auto format=args.count("input_format")?args.at("input_format"):"image";
    if(format=="nv12") {
        const int width=input_dimension(args,"input_width"),height=input_dimension(args,"input_height");
        const auto read=[&](const fs::path& path) {
            const auto bytes=std::size_t(width)*height*3/2;
            if(!fs::is_regular_file(path)||fs::file_size(path)!=bytes)
                throw std::invalid_argument("Raw NV12 must be tight and exactly "+std::to_string(bytes)+" bytes: "+path.string());
            cv::Mat image(height+height/2,width,CV_8UC1);
            std::ifstream file(path,std::ios::binary);file.read(reinterpret_cast<char*>(image.data),std::streamsize(bytes));
            if(!file)throw std::runtime_error("Cannot read NV12 image: "+path.string());
            return image;
        };
        return {read(pair.first),read(pair.second)};
    }
    if(format!="image")throw std::invalid_argument("--input_format must be image or nv12");
    const int flags=cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION;
    auto left=cv::imread(pair.first.string(),flags),right=cv::imread(pair.second.string(),flags);
    if (left.empty() || right.empty())throw std::invalid_argument("Cannot decode pair: "+pair.first.string()+" | "+pair.second.string());
    if (left.size()!=right.size())throw std::invalid_argument("Left/right source dimensions differ");
    return {left,right};
}

static void save_text(const fs::path& path,const std::string& data) {
    if (fs::exists(path))throw std::invalid_argument("Output already exists: "+path.string());
    std::ofstream file(path,std::ios::binary);
    file.write(data.data(),std::streamsize(data.size()));file.close();
    if (!file)throw std::runtime_error("Cannot save: "+path.string());
}

static void save_npy(const fs::path& path,const cv::Mat& disparity) {
    if (disparity.empty() || disparity.type()!=CV_32FC1)throw std::runtime_error("Expected 2D float32 disparity");
    if (fs::exists(path))throw std::invalid_argument("Output already exists: "+path.string());
    std::string header="{'descr': '<f4', 'fortran_order': False, 'shape': ("+
        std::to_string(disparity.rows)+", "+std::to_string(disparity.cols)+"), }";
    header.append((64-(10+header.size()+1)%64)%64,' ');header += '\n';
    std::ofstream file(path,std::ios::binary);
    const char magic[]={char(0x93),'N','U','M','P','Y',1,0};file.write(magic,8);
    const char length[]={char(header.size() & 255),char((header.size() >> 8) & 255)};
    file.write(length,2);file.write(header.data(),std::streamsize(header.size()));
    for (int y=0;y<disparity.rows;++y)for (int x=0;x<disparity.cols;++x) {
        const float value=disparity.at<float>(y,x);uint32_t bits;std::memcpy(&bits,&value,4);
        const char bytes[]={char(bits & 255),char((bits >> 8) & 255),char((bits >> 16) & 255),char((bits >> 24) & 255)};
        file.write(bytes,4);
    }
    file.close();if (!file)throw std::runtime_error("Cannot save NPY: "+path.string());
}

int main(int argc,char** argv) {
    try {
        auto args=arguments(argc,argv);
        if (args.count("help")) {
            std::cout<<"stereo_model_runner --model_file MODEL.bin --left_dir DIR --right_dir DIR --output_dir NEW_OR_EMPTY_DIR\n"
                <<"stereo_model_runner --model_file MODEL.bin --left IMAGE --right IMAGE --output disparity.npy\n"
                <<"Raw NV12: add --input_format nv12 --input_width SOURCE_W --input_height SOURCE_H\n"
                <<"--pre_mode crop|resize|strict (default crop); crop copies the bottom full-width even ROI without interpolation.\n"
                <<"Model metadata only: stereo_model_runner --model_file MODEL.bin --model_info\n"
                <<"--validate_pairs validates pair names/images only; it never executes inference.\n"
                <<"Direct NV12 model input: explicit Y/UV crop or resize; no YUV->BGR->YUV round trip or host /255.\n"
                <<"Output: native model-resolution float32 NPY plus .roi.json source-ROI metadata; the C++ stereo provider and pointcloud pipeline handle calibration/depth/PLY.\n";
            return 0;
        }
        const auto pre_mode=args.count("pre_mode")?args.at("pre_mode"):"crop";
        r36compare::validateStereoInputRequest(pre_mode,{});
        if(args.count("model_info")) {
            if(!args.count("model_file")||!fs::is_regular_file(args.at("model_file")))
                throw std::invalid_argument("--model_file must name an existing model");
            r36stereo::HorizonStereoModel model(args.at("model_file"),true);
            std::cout<<model.contract_json()<<'\n';return 0;
        }
        bool batch=false;const auto selected=pairs(args,batch);
        if (args.count("validate_pairs")) {
            for (const auto& pair:selected)read_pair(pair,args);
            std::cout<<"{\"pair_count\":"<<selected.size()<<",\"images_decoded\":true,\"inference_executed\":false}\n";
            return 0;
        }
        if (!args.count("model_file") || !fs::is_regular_file(args.at("model_file")))
            throw std::invalid_argument("--model_file must name an existing model");
        fs::path output;
        if (batch) {
            if (!args.count("output_dir"))throw std::invalid_argument("--output_dir required for batch mode");
            output=args.at("output_dir");
            if (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output)))
                throw std::invalid_argument("Batch output directory must be new or empty");
            fs::create_directories(output);
        } else {
            output=args.count("output") ? fs::path(args.at("output")) : fs::path("disparity.npy");
            if (fs::exists(output))throw std::invalid_argument("Output already exists: "+output.string());
            if (!output.parent_path().empty())fs::create_directories(output.parent_path());
        }
        r36stereo::HorizonStereoModel model(args.at("model_file"),true);
        const auto target=model.input_size();
        const auto contract=batch ? output/"model_contract.json" : fs::path(output.string()+".contract.json");
        save_text(contract,model.contract_json()+"\n");
        for (const auto& pair:selected) {
            auto images=read_pair(pair,args);
            const auto left_nv12=images.first.type()==CV_8UC1?images.first:r36stereo::bgr_to_nv12(images.first);
            const auto right_nv12=images.second.type()==CV_8UC1?images.second:r36stereo::bgr_to_nv12(images.second);
            const auto adaptation=r36compare::stereoInputAdaptation(pre_mode,r36stereo::nv12_size(left_nv12),target);
            const auto prepared=r36compare::prepareStereoNv12(left_nv12,right_nv12,adaptation);
            const auto start=std::chrono::steady_clock::now();
            const auto disparity=model.infer_nv12(prepared.left,prepared.right);
            if(pre_mode=="crop" && disparity.size()!=adaptation.source_roi.size())
                throw std::runtime_error("Crop output must match the native source ROI exactly; resizing is forbidden");
            const auto path=batch ? output/(pair.first.stem().string()+".npy") : output;
            const auto roi_metadata=fs::path(path.string()+".roi.json");
            if(fs::exists(roi_metadata))throw std::invalid_argument("Output already exists: "+roi_metadata.string());
            save_npy(path,disparity);
            save_text(roi_metadata,"{\"schema_version\":1,\"disparity_file\":"+r36compare::quote(path.filename().string())+
                ",\"native_disparity_not_restored\":true,\"output_coordinate_frame\":"+
                r36compare::quote(pre_mode=="crop"?"source_bottom_roi":"native_model_output")+
                ",\"output_units\":\"horizontal_model_input_pixels\",\"input_adaptation\":"+
                r36compare::stereoInputAdaptationJson(adaptation,disparity.size())+
                ",\"crop_projection\":"+(pre_mode=="crop"?
                    r36compare::quote("source_y=roi.y+output_y; use unchanged source intrinsics, or cropped_cy=source_cy-roi.y"):
                    std::string("null"))+"}\n");
            const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            std::cout<<pair.first.stem().string()<<" "<<disparity.cols<<"x"<<disparity.rows<<" "<<elapsed<<" ms -> "<<path.string()<<std::endl;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"stereo_model_runner: "<<error.what()<<std::endl;return 1;
    }
}
