#pragma once
// Standalone demo/test loader. The production node keeps its original parseDnnConfig().
#include "horizon_dnn_runner.hpp"
#include <rapidjson/document.h>
#include <fstream>
#include <iterator>
#include <stdexcept>
namespace perception {
inline DnnConfig LoadDnnConfigJson(const std::string& path) {
    std::ifstream file(path);
    if(!file)throw std::runtime_error("Cannot read config: "+path);
    const std::string text((std::istreambuf_iterator<char>(file)),{});
    rapidjson::Document document;document.Parse(text.c_str());
    if(document.HasParseError()||!document.IsObject()||!document.HasMember("dnn")||!document["dnn"].IsObject())
        throw std::runtime_error("Expected a JSON object with dnn settings");
    const auto& node=document["dnn"];DnnConfig config;
    const auto boolean=[](const auto& o,const char* k,bool v){return o.HasMember(k)&&o[k].IsBool()?o[k].GetBool():v;};
    const auto integer=[](const auto& o,const char* k,int v){return o.HasMember(k)&&o[k].IsInt()?o[k].GetInt():v;};
    const auto real=[](const auto& o,const char* k,float v){return o.HasMember(k)&&o[k].IsNumber()?o[k].GetFloat():v;};
    const auto string=[](const auto& o,const char* k,const std::string& v){return o.HasMember(k)&&o[k].IsString()?std::string(o[k].GetString()):v;};
    config.enabled=boolean(node,"enabled",config.enabled);
    config.parallel_det_seg=boolean(node,"parallel_det_seg",config.parallel_det_seg);
    config.left_yuv_format=string(node,"left_yuv_format",config.left_yuv_format);
    config.detect.enabled=false;config.seg.enabled=false;
    if(node.HasMember("detect")&&node["detect"].IsObject()) {
        const auto& d=node["detect"];auto& out=config.detect;
        out.enabled=boolean(d,"enabled",true);out.model_path=string(d,"model_path",out.model_path);
        out.classes_num=integer(d,"classes_num",out.classes_num);out.reg=integer(d,"reg",out.reg);
        out.resize_type=integer(d,"resize_type",out.resize_type);out.nms_top_k=integer(d,"nms_top_k",out.nms_top_k);
        out.class_names=string(d,"class_names",out.class_names);out.keep_classes=string(d,"keep_classes",out.keep_classes);
        out.score_threshold=real(d,"score_threshold",out.score_threshold);out.nms_threshold=real(d,"nms_threshold",out.nms_threshold);
    }
    if(node.HasMember("seg")&&node["seg"].IsObject()) {
        const auto& s=node["seg"];auto& out=config.seg;
        out.enabled=boolean(s,"enabled",true);out.model_path=string(s,"model_path",out.model_path);
        out.num_classes=integer(s,"num_classes",out.num_classes);out.resize_type=integer(s,"resize_type",out.resize_type);
        out.class_names=string(s,"class_names",out.class_names);out.argmax_mode=string(s,"argmax_mode",out.argmax_mode);
        out.align_corners=boolean(s,"align_corners",out.align_corners);out.bgr_input=boolean(s,"bgr_input",out.bgr_input);
        out.swap_classes=boolean(s,"swap_classes",out.swap_classes);
    }
    return config;
}
}
