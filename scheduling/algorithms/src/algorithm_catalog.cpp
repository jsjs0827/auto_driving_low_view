#include "algorithm_catalog.hpp"
#include "algorithm_manifest_data.hpp"
#include <algorithm>
#include <cstdlib>
#include <set>
#include <sstream>

namespace perception::algorithms {
namespace {
using namespace r33::json;
Json object() { Json v; v.kind = Json::Kind::Object; return v; }
Json text(const std::string& s) { Json v; v.kind=Json::Kind::String; v.text=s; return v; }
const Json& parametersOf(const Json& config) {
    static const Json empty=object();
    const auto p=optional(config,"parameters"); return p ? *p : empty;
}
bool enabled(const Json& node) {
    const auto p=optional(node,"enabled"); return !p || boolean(*p);
}
std::string expand(const std::string& raw) {
    std::string out;
    for(std::size_t i=0;i<raw.size();) {
        if(raw[i]=='$' && i+1<raw.size() && raw[i+1]=='{') {
            const auto end=raw.find('}',i+2);
            if(end==std::string::npos) throw std::invalid_argument("Unclosed environment variable in algorithm path");
            const auto name=raw.substr(i+2,end-i-2);
            const auto value=std::getenv(name.c_str());
            if(!value || !*value) throw std::invalid_argument("Unset algorithm path variable: "+name);
            out+=value; i=end+1;
        } else out+=raw[i++];
    }
    return out;
}
void numeric(const Json& value,const Json& rule,const std::string& name,bool integral) {
    const double n=number(value);
    if(integral && std::floor(n)!=n) throw std::invalid_argument(name+" must be an integer");
    if(const auto p=optional(rule,"min")) if(n<number(*p)) throw std::invalid_argument(name+" is below minimum");
    if(const auto p=optional(rule,"max")) if(n>number(*p)) throw std::invalid_argument(name+" exceeds maximum");
}
void requireObject(const Json& value,const std::string& name) {
    if(value.kind!=Json::Kind::Object) throw std::invalid_argument(name+" must be an object");
}
}

std::string jsonText(const Json& value) {
    using K=Json::Kind;
    if(value.kind==K::Null) return "null";
    if(value.kind==K::Boolean) return value.boolean?"true":"false";
    if(value.kind==K::String) return r33::json::quote(value.text);
    if(value.kind==K::Number) { r33::json::number(value); return value.text; }
    std::ostringstream out; bool first=true;
    if(value.kind==K::Array) {
        out<<'[';for(const auto& v:value.array){if(!first)out<<',';first=false;out<<jsonText(v);}out<<']';
    } else {
        out<<'{';for(const auto& item:value.object){if(!first)out<<',';first=false;out<<r33::json::quote(item.first)<<':'<<jsonText(item.second);}out<<'}';
    }
    return out.str();
}

const std::map<std::string,Descriptor>& catalog() {
    static const auto entries=[] {
        std::map<std::string,Descriptor> out;std::set<std::string> features;
        const std::set<std::string> reserved={"inference","dnn","det","seg","stereo","disparity","rgb","cloud","pointcloud","fusion","maps","calibration","labels","transform","ground","history","recharge","publish","record","visual_motion","motion_reference","slip_detection","temporal_difference","temporal_match","temporal_extension"};
        for(const auto& source:algorithmManifestData()) {
            const auto root=r33::json::JsonParser(source,"algorithm registration").parse();
            fields(root,{"id","feature","header","factory","sources","parameters","inputs"});
            Descriptor d; d.id=string(required(root,"id")); d.feature=string(required(root,"feature"));
            if(reserved.count(d.id) || reserved.count(d.feature) || !features.insert(d.feature).second)
                throw std::invalid_argument("Duplicate/reserved algorithm registration: "+d.id);
            d.schema=required(root,"parameters");requireObject(d.schema,"algorithm parameter schema");
            for(const auto& rule:d.schema.object) {
                if(rule.first=="_help")continue;
                fields(rule.second,{"type","required","default","min","max","enum","length"});
                const auto type=string(required(rule.second,"type"));
                if(type!="string"&&type!="file"&&type!="directory"&&type!="integer"&&type!="number"&&type!="boolean"&&type!="numbers"&&type!="strings")
                    throw std::invalid_argument("Unknown algorithm parameter type: "+type);
            }
            const auto& inputs=required(root,"inputs");requireObject(inputs,"algorithm inputs");
            for(const auto& input:inputs.object)d.inputs.emplace(input.first,string(input.second));
            if(!out.emplace(d.id,std::move(d)).second)throw std::invalid_argument("Duplicate algorithm id");
        }
        return out;
    }();
    return entries;
}
const Descriptor* find(const std::string& name) {
    const auto& all=catalog();const auto it=all.find(name);if(it!=all.end())return &it->second;
    for(const auto& item:all)if(item.second.feature==name)return &item.second;
    return nullptr;
}
std::string nodeAlgorithm(const Json& node) {
    const auto feature=r33::json::string(r33::json::required(node,"feature"));
    const Descriptor* spec=nullptr;
    if(feature=="inference") {if(const auto p=r33::json::optional(node,"algorithm"))spec=find(r33::json::string(*p));}
    else spec=find(feature);
    return spec?spec->id:std::string{};
}
Json normalizeParameters(const Descriptor& spec,const Json& requested,const std::filesystem::path& base,bool active) {
    using namespace r33::json;requireObject(requested,spec.id+" parameters");Json out=object();
    for(const auto& item:requested.object)
        if(item.first!="_help"&&!spec.schema.object.count(item.first))throw std::invalid_argument("Unknown "+spec.id+" parameter: "+item.first);
    for(const auto& entry:spec.schema.object) {
        if(entry.first=="_help")continue;
        const auto& rule=entry.second;const auto name=spec.id+"."+entry.first;
        const Json* value=optional(requested,entry.first.c_str());
        if(!value||value->kind==Json::Kind::Null)value=optional(rule,"default");
        if(!value||value->kind==Json::Kind::Null) {
            if(active)if(const auto p=optional(rule,"required"))if(boolean(*p))throw std::invalid_argument("Missing required algorithm parameter: "+name);
            continue;
        }
        const auto type=string(required(rule,"type"));Json resolved=*value;
        if(type=="string"||type=="file"||type=="directory") {
            const auto raw=string(*value);
            if(type!="string") {
                auto path=std::filesystem::u8path(expand(raw));if(path.is_relative())path=base/path;
                path=std::filesystem::absolute(path).lexically_normal();
                if(active&&type=="file"&&!std::filesystem::is_regular_file(path))throw std::invalid_argument(name+" file does not exist: "+path.u8string());
                if(active&&type=="directory"&&!std::filesystem::is_directory(path))throw std::invalid_argument(name+" directory does not exist: "+path.u8string());
                resolved=text(path.u8string());
            }
        } else if(type=="integer"||type=="number")numeric(*value,rule,name,type=="integer");
        else if(type=="boolean")boolean(*value);
        else {
            if(value->kind!=Json::Kind::Array)throw std::invalid_argument(name+" must be an array");
            if(const auto len=optional(rule,"length"))if(value->array.size()!=std::size_t(integer(*len,0,4096)))throw std::invalid_argument(name+" has incorrect array length");
            for(const auto& element:value->array)if(type=="numbers")numeric(element,rule,name,false);else string(element);
        }
        if(const auto choices=optional(rule,"enum")) {
            if(choices->kind!=Json::Kind::Array)throw std::invalid_argument("Algorithm enum must be an array");
            bool match=false;for(const auto& item:choices->array)if(jsonText(item)==jsonText(*value))match=true;
            if(!match)throw std::invalid_argument("Unsupported value for "+name);
        }
        out.object[entry.first]=std::move(resolved);
    }
    return out;
}
Json compileConfigurations(const Json& graph,const Json& settings,const std::filesystem::path& base) {
    using namespace r33::json;Json result=object();
    for(const auto& node:required(graph,"nodes").array) {
        const auto algorithm=nodeAlgorithm(node);if(algorithm.empty()||!enabled(node))continue;
        const auto& spec=*find(algorithm);Json params=object();
        if(const auto values=optional(settings,algorithm.c_str())){requireObject(*values,algorithm);params=*values;}
        if(const auto overrides=optional(node,"parameters")){requireObject(*overrides,algorithm+" overrides");for(const auto& item:overrides->object)params.object[item.first]=item.second;}
        Json item=object();item.object["algorithm"]=text(algorithm);
        item.object["parameters"]=normalizeParameters(spec,params,base);
        result.object[string(required(node,"id"))]=std::move(item);
    }
    return result;
}
Json validateConfigurations(const Json& graph,const Json& configurations,const std::filesystem::path& base) {
    using namespace r33::json;requireObject(configurations,"algorithms");Json result=object();
    for(const auto& node:required(graph,"nodes").array) {
        const auto algorithm=nodeAlgorithm(node);if(algorithm.empty()||!enabled(node))continue;
        const auto id=string(required(node,"id"));const auto& config=required(configurations,id.c_str());
        fields(config,{"algorithm","parameters"});
        if(string(required(config,"algorithm"))!=algorithm)throw std::invalid_argument("Algorithm configuration must match graph task: "+id);
        Json item=object();item.object["algorithm"]=text(algorithm);
        item.object["parameters"]=normalizeParameters(*find(algorithm),parametersOf(config),base);
        result.object[id]=std::move(item);
    }
    for(const auto& entry:configurations.object)if(entry.first!="_help"&&!result.object.count(entry.first))throw std::invalid_argument("Algorithm configuration has no enabled graph task: "+entry.first);
    return result;
}
bool hasWork(const Json& configurations) {return configurations.kind==Json::Kind::Object&&!configurations.object.empty();}
}
