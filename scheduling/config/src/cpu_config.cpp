// Parser copied from the preserved framework strict JSON reader; framework is unchanged.
#include "cpu_stages.hpp"
#include "runtime_status.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
namespace r29 { namespace {
constexpr std::size_t kMaximumJsonDepth=64,kMaximumJsonNodes=65536,kMaximumStringBytes=65536,kMaximumArrayItems=16384,kMaximumObjectMembers=1024;
struct JsonValue {
    enum class Kind { Null, Boolean, Number, String, Array, Object };

    Kind kind = Kind::Null;
    bool boolean = false;
    std::string text;
    std::vector<JsonValue> array;
    std::map<std::string, JsonValue> object;
};

class JsonParser {
  public:
    JsonParser(std::string source, std::string path)
        : source_(std::move(source)), path_(std::move(path)) {}

    JsonValue parse() {
        skipWhitespace();
        auto result = value(0);
        skipWhitespace();
        if (position_ != source_.size())
            fail("unexpected content after the JSON value");
        return result;
    }

  private:
    [[noreturn]] void fail(const std::string &message) const {
        std::size_t line = 1;
        std::size_t column = 1;
        for (std::size_t index = 0; index < position_ && index < source_.size(); ++index) {
            if (source_[index] == '\n') {
                ++line;
                column = 1;
            } else {
                ++column;
            }
        }
        throw std::runtime_error(path_ + ":" + std::to_string(line) + ":" +
                                 std::to_string(column) + ": " + message);
    }

    void skipWhitespace() {
        while (position_ < source_.size()) {
            const char character = source_[position_];
            if (character != ' ' && character != '\t' && character != '\r' &&
                character != '\n')
                break;
            ++position_;
        }
    }

    bool consume(char expected) {
        if (position_ >= source_.size() || source_[position_] != expected)
            return false;
        ++position_;
        return true;
    }

    void expect(char expected, const char *description) {
        if (!consume(expected))
            fail(std::string("expected ") + description);
    }

    JsonValue value(std::size_t depth) {
        if (depth > kMaximumJsonDepth)
            fail("JSON nesting exceeds 64 levels");
        if (++node_count_ > kMaximumJsonNodes)
            fail("JSON value count exceeds the limit of 65536");
        if (position_ >= source_.size())
            fail("unexpected end of JSON input");
        switch (source_[position_]) {
        case '{':
            return object(depth + 1);
        case '[':
            return array(depth + 1);
        case '"': {
            JsonValue result;
            result.kind = JsonValue::Kind::String;
            result.text = string();
            return result;
        }
        case 't':
            literal("true");
            return boolean(true);
        case 'f':
            literal("false");
            return boolean(false);
        case 'n':
            literal("null");
            return {};
        default:
            if (source_[position_] == '-' ||
                (source_[position_] >= '0' && source_[position_] <= '9'))
                return number();
            fail("expected a JSON value");
        }
    }

    JsonValue object(std::size_t depth) {
        expect('{', "'{'");
        JsonValue result;
        result.kind = JsonValue::Kind::Object;
        skipWhitespace();
        if (consume('}'))
            return result;
        for (;;) {
            if (result.object.size() >= kMaximumObjectMembers)
                fail("JSON object exceeds the limit of 1024 members");
            if (position_ >= source_.size() || source_[position_] != '"')
                fail("expected an object field name");
            auto key = string();
            skipWhitespace();
            expect(':', "':' after an object field name");
            skipWhitespace();
            auto inserted = result.object.emplace(std::move(key), value(depth));
            if (!inserted.second)
                fail("duplicate object field: " + inserted.first->first);
            skipWhitespace();
            if (consume('}'))
                return result;
            expect(',', "',' or '}' in an object");
            skipWhitespace();
        }
    }

    JsonValue array(std::size_t depth) {
        expect('[', "'['");
        JsonValue result;
        result.kind = JsonValue::Kind::Array;
        skipWhitespace();
        if (consume(']'))
            return result;
        for (;;) {
            if (result.array.size() >= kMaximumArrayItems)
                fail("JSON array exceeds the limit of 16384 items");
            result.array.push_back(value(depth));
            skipWhitespace();
            if (consume(']'))
                return result;
            expect(',', "',' or ']' in an array");
            skipWhitespace();
        }
    }

    static int hexadecimal(char value) {
        if (value >= '0' && value <= '9')
            return value - '0';
        if (value >= 'a' && value <= 'f')
            return value - 'a' + 10;
        if (value >= 'A' && value <= 'F')
            return value - 'A' + 10;
        return -1;
    }

    std::uint32_t unicodeEscape() {
        if (position_ + 4 > source_.size())
            fail("incomplete Unicode escape");
        std::uint32_t result = 0;
        for (int index = 0; index < 4; ++index) {
            const int digit = hexadecimal(source_[position_++]);
            if (digit < 0)
                fail("invalid hexadecimal digit in Unicode escape");
            result = (result << 4U) | static_cast<std::uint32_t>(digit);
        }
        return result;
    }

    void appendUtf8(std::string &target, std::uint32_t codepoint) {
        if (codepoint <= 0x7FU) {
            target.push_back(static_cast<char>(codepoint));
        } else if (codepoint <= 0x7FFU) {
            target.push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
            target.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else if (codepoint <= 0xFFFFU) {
            target.push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
            target.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            target.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else {
            target.push_back(static_cast<char>(0xF0U | (codepoint >> 18U)));
            target.push_back(static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3FU)));
            target.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            target.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        }
    }

    std::string string() {
        expect('"', "string");
        std::string result;
        while (position_ < source_.size()) {
            const unsigned char character =
                static_cast<unsigned char>(source_[position_++]);
            if (character == '"')
                return result;
            if (character < 0x20U)
                fail("unescaped control character in a string");
            if (character != '\\') {
                result.push_back(static_cast<char>(character));
                if (result.size() > kMaximumStringBytes)
                    fail("JSON string exceeds the limit of 65536 bytes");
                continue;
            }
            if (position_ >= source_.size())
                fail("incomplete string escape");
            const char escape = source_[position_++];
            switch (escape) {
            case '"': result.push_back('"'); break;
            case '\\': result.push_back('\\'); break;
            case '/': result.push_back('/'); break;
            case 'b': result.push_back('\b'); break;
            case 'f': result.push_back('\f'); break;
            case 'n': result.push_back('\n'); break;
            case 'r': result.push_back('\r'); break;
            case 't': result.push_back('\t'); break;
            case 'u': {
                std::uint32_t codepoint = unicodeEscape();
                if (codepoint >= 0xD800U && codepoint <= 0xDBFFU) {
                    if (position_ + 2 > source_.size() || source_[position_] != '\\' ||
                        source_[position_ + 1] != 'u')
                        fail("high surrogate is not followed by a low surrogate");
                    position_ += 2;
                    const std::uint32_t low = unicodeEscape();
                    if (low < 0xDC00U || low > 0xDFFFU)
                        fail("invalid low surrogate");
                    codepoint = 0x10000U + ((codepoint - 0xD800U) << 10U) +
                                (low - 0xDC00U);
                } else if (codepoint >= 0xDC00U && codepoint <= 0xDFFFU) {
                    fail("unexpected low surrogate");
                }
                appendUtf8(result, codepoint);
                break;
            }
            default:
                fail("invalid string escape");
            }
            if (result.size() > kMaximumStringBytes)
                fail("JSON string exceeds the limit of 65536 bytes");
        }
        fail("unterminated string");
    }

    JsonValue number() {
        const std::size_t begin = position_;
        consume('-');
        if (position_ >= source_.size())
            fail("incomplete number");
        if (source_[position_] == '0') {
            ++position_;
            if (position_ < source_.size() && std::isdigit(
                    static_cast<unsigned char>(source_[position_])))
                fail("leading zero in a number");
        } else {
            if (source_[position_] < '1' || source_[position_] > '9')
                fail("invalid integer part");
            while (position_ < source_.size() && std::isdigit(
                       static_cast<unsigned char>(source_[position_])))
                ++position_;
        }
        if (consume('.')) {
            if (position_ >= source_.size() || !std::isdigit(
                    static_cast<unsigned char>(source_[position_])))
                fail("fraction requires at least one digit");
            while (position_ < source_.size() && std::isdigit(
                       static_cast<unsigned char>(source_[position_])))
                ++position_;
        }
        if (position_ < source_.size() &&
            (source_[position_] == 'e' || source_[position_] == 'E')) {
            ++position_;
            if (position_ < source_.size() &&
                (source_[position_] == '+' || source_[position_] == '-'))
                ++position_;
            if (position_ >= source_.size() || !std::isdigit(
                    static_cast<unsigned char>(source_[position_])))
                fail("exponent requires at least one digit");
            while (position_ < source_.size() && std::isdigit(
                       static_cast<unsigned char>(source_[position_])))
                ++position_;
        }
        JsonValue result;
        result.kind = JsonValue::Kind::Number;
        result.text = source_.substr(begin, position_ - begin);
        return result;
    }

    JsonValue boolean(bool value) {
        JsonValue result;
        result.kind = JsonValue::Kind::Boolean;
        result.boolean = value;
        return result;
    }

    void literal(const char *expected) {
        const std::string token(expected);
        if (source_.compare(position_, token.size(), token) != 0)
            fail("invalid JSON literal");
        position_ += token.size();
    }

    std::string source_;
    std::string path_;
    std::size_t position_ = 0;
    std::size_t node_count_ = 0;
};

[[noreturn]] void invalid(const std::string& s){throw std::invalid_argument(s);}
void fields(const JsonValue& v,std::initializer_list<const char*> allowed,const std::string& where){
    if(v.kind!=JsonValue::Kind::Object)invalid(where+" must be an object");
    for(const auto& e:v.object){bool found=false;for(auto name:allowed)if(e.first==name)found=true;if(!found)invalid(where+" unknown field: "+e.first);}
}
const JsonValue& field(const JsonValue& v,const std::string& k){auto it=v.object.find(k);if(it==v.object.end())invalid("CPU profile missing field: "+k);return it->second;}
std::string string(const JsonValue& v){if(v.kind!=JsonValue::Kind::String||v.text.empty()||v.text.find('\0')!=std::string::npos)invalid("CPU profile requires nonempty string without NUL");return v.text;}
double number(const JsonValue& v){if(v.kind!=JsonValue::Kind::Number)invalid("CPU profile requires number");std::size_t n=0;const auto a=std::stod(v.text,&n);if(n!=v.text.size()||!std::isfinite(a))invalid("CPU profile number must be finite");return a;}
float scalar(const JsonValue& v){const auto a=number(v);if(std::abs(a)>std::numeric_limits<float>::max())invalid("CPU profile number outside float32 range");return float(a);}
int integer(const JsonValue& v){const auto a=number(v);if(a!=std::floor(a)||a<1||a>65536)invalid("CPU profile requires positive integer <=65536");return int(a);}
std::vector<std::string> strings(const JsonValue& v){if(v.kind!=JsonValue::Kind::Array)invalid("CPU profile requires string array");std::vector<std::string> a;for(const auto& e:v.array)a.push_back(string(e));return a;}
std::array<float,3> triple(const JsonValue& v){if(v.kind!=JsonValue::Kind::Array||v.array.size()!=3)invalid("CPU normalization requires exactly three channel values");return {{scalar(v.array[0]),scalar(v.array[1]),scalar(v.array[2])}};}
std::string csv(const std::vector<std::string>& v){std::string s;for(const auto& a:v){if(a.find(',')!=std::string::npos)invalid("class_names cannot contain comma");if(!s.empty())s+=',';s+=a;}return s;}
CpuLaneConfig lane(const JsonValue& v,bool det){
    if(det)fields(v,{"model","width","height","classes","class_names","input","output","score_threshold","nms_threshold","nms_top_k"},"detection");
    else fields(v,{"model","width","height","classes","class_names","input","output"},"segmentation");
    CpuLaneConfig c;c.width=integer(field(v,"width"));c.height=integer(field(v,"height"));c.classes=integer(field(v,"classes"));c.class_names=strings(field(v,"class_names"));
    const auto& i=field(v,"input");fields(i,{"name","color","yuv","scale","mean","std"},"input");
    c.input.name=string(field(i,"name"));c.input.color=string(field(i,"color"));c.input.yuv=string(field(i,"yuv"));c.input.scale=scalar(field(i,"scale"));c.input.mean=triple(field(i,"mean"));c.input.stddev=triple(field(i,"std"));
    const auto& o=field(v,"output");
    if(det){c.output.format=string(field(o,"format"));if(c.output.format=="raw_dfl"){fields(o,{"format","layout","names","reg"},"detection.output");c.output.names=strings(field(o,"names"));c.output.reg=integer(field(o,"reg"));}else{fields(o,{"format","layout","name"},"detection.output");c.output.name=string(field(o,"name"));}}
    else {fields(o,{"name","layout"},"segmentation.output");c.output.format="logits";c.output.name=string(field(o,"name"));}
    c.output.layout=string(field(o,"layout"));return c;
}
}

void validateCpuConfig(const CpuConfig& c){
    if(c.intra_threads<1||c.intra_threads>64||c.inter_threads<1||c.inter_threads>64)invalid("CPU thread counts must be 1..64");
    if(c.detection.width!=864||c.detection.height!=480||c.segmentation.width!=832||c.segmentation.height!=480)invalid("Original CPU input contract requires detection864x480 and segmentation832x480");
    for(const auto* lane:{&c.detection,&c.segmentation}){
        if(lane->classes<1||lane->classes>256||int(lane->class_names.size())!=lane->classes)invalid("CPU class_names count must equal classes within1..256");
        if(lane->input.name.empty()||lane->input.name.find('\0')!=std::string::npos||(lane->input.color!="RGB"&&lane->input.color!="BGR")||lane->input.yuv!="bt601_limited_integer")invalid("CPU input requires name, RGB/BGR and explicit bt601_limited_integer conversion");
        if(!std::isfinite(lane->input.scale)||lane->input.scale<=0)invalid("CPU input scale must be positive finite");
        for(int i=0;i<3;++i)if(!std::isfinite(lane->input.mean[i])||!std::isfinite(lane->input.stddev[i])||lane->input.stddev[i]<=0)invalid("CPU normalization mean finite and std positive finite required");
        if(lane->output.layout!="NCHW"&&lane->output.layout!="NHWC"&&!(lane==&c.detection&&lane->output.format=="decoded_xyxy"&&lane->output.layout=="NXC"))invalid("CPU output layout must be explicit NCHW/NHWC; decoded_xyxy requires NXC");
        for(const auto& name:lane->class_names)if(name.empty()||name.find(',')!=std::string::npos||name.find('\0')!=std::string::npos)invalid("CPU class names must be nonempty without comma/NUL");
    }
    const auto& o=c.detection.output;
    if(o.format=="raw_dfl"){
        if(o.names.size()!=6||o.reg<2||o.reg>64)invalid("raw_dfl requires six ordered output names and reg2..64");
        std::set<std::string> unique;for(const auto& n:o.names)if(n.empty()||n.find('\0')!=std::string::npos||!unique.insert(n).second)invalid("raw_dfl output names must be six unique nonempty strings");
    }else if(o.format!="decoded_xyxy"||o.layout!="NXC"||o.name.empty())invalid("CPU detection supports explicit raw_dfl or decoded_xyxy/NXC");
    if(c.segmentation.output.name.empty()||c.segmentation.output.name.find('\0')!=std::string::npos||o.name.find('\0')!=std::string::npos)invalid("Segmentation logits output name is required without NUL");
    if(c.dnn.detect.model_path.empty()||c.dnn.seg.model_path.empty()||c.dnn.detect.model_path.find('\0')!=std::string::npos||c.dnn.seg.model_path.find('\0')!=std::string::npos)invalid("CPU requires original ONNX model paths without NUL; BIN cannot execute on x86");
    if(!(c.dnn.detect.score_threshold>0&&c.dnn.detect.score_threshold<1)||!(c.dnn.detect.nms_threshold>=0&&c.dnn.detect.nms_threshold<=1)||c.dnn.detect.nms_top_k<1||c.dnn.detect.nms_top_k>10000)invalid("CPU detector thresholds/top_k are invalid");
    if(c.dnn.detect.resize_type!=1||c.dnn.seg.resize_type!=0||!c.dnn.seg.align_corners||c.dnn.seg.argmax_mode!="bilinear"||!c.dnn.seg.fused_model_path.empty()||c.dnn.seg.fused_vse)invalid("CPU backend preserves original letterbox and bilinear-logits segmentation postprocess");
}

CpuConfig loadCpuConfig(const std::string& path){
    try{
        std::ifstream f(std::filesystem::u8path(path),std::ios::binary);if(!f)invalid("Cannot open CPU profile: "+path);
        f.seekg(0,std::ios::end);const auto size=f.tellg();if(size<0||size>4*1024*1024)invalid("CPU profile exceeds4MiB");f.seekg(0);std::string s(std::size_t(size),'\0');f.read(s.data(),size);if(!f)invalid("Cannot read CPU profile");
        if(s.size()>=3&&static_cast<unsigned char>(s[0])==0xef&&static_cast<unsigned char>(s[1])==0xbb&&static_cast<unsigned char>(s[2])==0xbf)invalid("CPU profile must be UTF-8 without BOM");
        const auto v=JsonParser(std::move(s),path).parse();fields(v,{"schema_version","runtime","detection","segmentation"},"CPU profile");if(integer(field(v,"schema_version"))!=1)invalid("Unsupported CPU schema_version");
        CpuConfig c;const auto& rt=field(v,"runtime");fields(rt,{"intra_threads","inter_threads"},"runtime");c.intra_threads=integer(field(rt,"intra_threads"));c.inter_threads=integer(field(rt,"inter_threads"));
        const auto& d=field(v,"detection");const auto& g=field(v,"segmentation");c.detection=lane(d,true);c.segmentation=lane(g,false);
        const auto resolve=[&](const std::string& text){auto p=std::filesystem::u8path(text);if(p.is_relative())p=std::filesystem::u8path(path).parent_path()/p;return p.lexically_normal().u8string();};
        c.dnn.enabled=true;c.dnn.detect.model_path=resolve(string(field(d,"model")));c.dnn.seg.model_path=resolve(string(field(g,"model")));
        c.dnn.detect.classes_num=c.detection.classes;c.dnn.detect.class_names=csv(c.detection.class_names);c.dnn.detect.keep_classes.clear();c.dnn.detect.reg=c.detection.output.reg;c.dnn.detect.score_threshold=scalar(field(d,"score_threshold"));c.dnn.detect.nms_threshold=scalar(field(d,"nms_threshold"));c.dnn.detect.nms_top_k=integer(field(d,"nms_top_k"));
        c.dnn.seg.num_classes=c.segmentation.classes;c.dnn.seg.class_names=csv(c.segmentation.class_names);c.dnn.detect.resize_type=1;c.dnn.seg.resize_type=0;c.dnn.seg.align_corners=true;c.dnn.seg.argmax_mode="bilinear";
        validateCpuConfig(c);return c;
    }catch(const r28::Failure&){throw;}catch(const std::exception& e){throw r28::Failure({r28::ConfigInvalid,0,"cpu.profile",{},e.what(),{}});}
}
}

