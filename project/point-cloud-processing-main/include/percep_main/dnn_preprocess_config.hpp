#pragma once
#include <rapidjson/document.h>
#include <sstream>
#include <string>

namespace perception {
// Expected source and preprocessor output sizes. Pair 0/0 means automatic.
// Output size always comes from the actual BIN tensor; these fields validate it.
struct DnnPreprocessConfig {
    int source_width=0, source_height=0;
    int detect_output_width=0, detect_output_height=0;
    int seg_output_width=0, seg_output_height=0;
};
inline bool DnnPreprocessFail(std::string* error,const std::string& text) {
    if(error)*error=text;
    return false;
}
inline bool DnnPreprocessPair(int w,int h,const char* name,std::string* error) {
    if(w==0&&h==0)return true;
    if(w<2||h<2||w>4096||h>4096||(w&1)||(h&1)) {
        std::ostringstream os;os<<name<<" must be 0/0 (automatic) or even width/height in [2,4096], got "<<w<<'x'<<h;
        return DnnPreprocessFail(error,os.str());
    }
    return true;
}
inline bool ParseDnnPreprocessConfig(const rapidjson::Value& dnn,
                                    DnnPreprocessConfig* output,std::string* error) {
    if(error)error->clear();
    if(!output||!dnn.IsObject())return DnnPreprocessFail(error,"dnn preprocessing parser requires an object and output");
    DnnPreprocessConfig result;
    if(!dnn.HasMember("preprocess")){*output=result;return true;}
    const auto& object=dnn["preprocess"];
    if(!object.IsObject())return DnnPreprocessFail(error,"dnn.preprocess must be an object");
    struct Field {const char* name;int DnnPreprocessConfig::*member;};
    const Field fields[]={
        {"source_width",&DnnPreprocessConfig::source_width},
        {"source_height",&DnnPreprocessConfig::source_height},
        {"detect_output_width",&DnnPreprocessConfig::detect_output_width},
        {"detect_output_height",&DnnPreprocessConfig::detect_output_height},
        {"seg_output_width",&DnnPreprocessConfig::seg_output_width},
        {"seg_output_height",&DnnPreprocessConfig::seg_output_height}};
    for(auto it=object.MemberBegin();it!=object.MemberEnd();++it) {
        const std::string name=it->name.GetString();
        if(name=="_comment")continue;
        bool known=false;
        for(const auto& field:fields)if(name==field.name) {
            known=true;
            if(!it->value.IsInt())return DnnPreprocessFail(error,"dnn.preprocess."+name+" must be an integer");
            result.*(field.member)=it->value.GetInt();break;
        }
        if(!known)return DnnPreprocessFail(error,"Unknown dnn.preprocess setting: "+name);
    }
    if(!DnnPreprocessPair(result.source_width,result.source_height,"dnn.preprocess.source",error)||
       !DnnPreprocessPair(result.detect_output_width,result.detect_output_height,"dnn.preprocess.detect_output",error)||
       !DnnPreprocessPair(result.seg_output_width,result.seg_output_height,"dnn.preprocess.seg_output",error))return false;
    *output=result;return true;
}
// Legacy numeric setting retained: 0=direct resize, 1=letterbox.
inline bool ParseDnnResizeType(const rapidjson::Value& model,const char* name,
        int default_type,int* output,std::string* error) {
    if(error)error->clear();
    if(!model.IsObject()||!output)return DnnPreprocessFail(error,"resize_type parser requires a model object and output");
    int value=default_type;
    if(model.HasMember("resize_type")) {
        if(!model["resize_type"].IsInt())return DnnPreprocessFail(error,std::string(name)+".resize_type must be integer 0 (resize) or 1 (letterbox)");
        value=model["resize_type"].GetInt();
    }
    if(value!=0&&value!=1)return DnnPreprocessFail(error,std::string(name)+".resize_type must be 0 (resize) or 1 (letterbox)");
    *output=value;return true;
}
inline bool ValidateDnnPreprocessModelSizes(const DnnPreprocessConfig& config,
        bool detection_enabled,int detection_width,int detection_height,
        bool segmentation_enabled,int segmentation_width,int segmentation_height,
        std::string* error) {
    if(error)error->clear();
    const auto check=[&](bool enabled,int expected_w,int expected_h,int actual_w,int actual_h,const char* name) {
        if(!enabled)return true;
        if(actual_w<2||actual_h<2||actual_w>4096||actual_h>4096||(actual_w&1)||(actual_h&1))
            return DnnPreprocessFail(error,std::string(name)+" model has invalid NV12 input dimensions");
        if(expected_w&& (expected_w!=actual_w||expected_h!=actual_h)) {
            std::ostringstream os;os<<name<<" configured preprocess output "<<expected_w<<'x'<<expected_h
                <<" does not match actual BIN input "<<actual_w<<'x'<<actual_h
                <<"; use a matching BIN or set this output size to 0/0 for model-derived size";
            return DnnPreprocessFail(error,os.str());
        }
        return true;
    };
    return DnnPreprocessPair(config.detect_output_width,config.detect_output_height,"detect_output",error)&&
           DnnPreprocessPair(config.seg_output_width,config.seg_output_height,"seg_output",error)&&
           check(detection_enabled,config.detect_output_width,config.detect_output_height,detection_width,detection_height,"Detection")&&
           check(segmentation_enabled,config.seg_output_width,config.seg_output_height,segmentation_width,segmentation_height,"Segmentation");
}
inline bool ValidateDnnPreprocessSource(const DnnPreprocessConfig& config,
        int actual_width,int actual_height,std::string* error) {
    if(error)error->clear();
    if(!DnnPreprocessPair(config.source_width,config.source_height,"source",error))return false;
    if(actual_width<2||actual_height<2||actual_width>4096||actual_height>4096||(actual_width&1)||(actual_height&1))
        return DnnPreprocessFail(error,"Actual source dimensions must be even and in [2,4096]");
    if(config.source_width&&(config.source_width!=actual_width||config.source_height!=actual_height)) {
        std::ostringstream os;os<<"Configured source "<<config.source_width<<'x'<<config.source_height
          <<" does not match actual frame "<<actual_width<<'x'<<actual_height;
        return DnnPreprocessFail(error,os.str());
    }
    return true;
}
} // namespace perception
