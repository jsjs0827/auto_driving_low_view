#pragma once
// Strict JSON reader copied unchanged from src/cpu_config.cpp (preserved framework reader).
#include <cctype>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <limits>
#include <map>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace r33::json {
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


inline JsonValue load(const std::string& path) {
    std::ifstream f(std::filesystem::u8path(path), std::ios::binary);
    if(!f) throw std::invalid_argument("Cannot open JSON: "+path);
    f.seekg(0,std::ios::end);auto n=f.tellg();
    if(n<0 || n>4*1024*1024) throw std::invalid_argument("JSON size exceeds 4 MiB: "+path);
    f.seekg(0);std::string s(static_cast<std::size_t>(n),'\0');
    f.read(s.data(),static_cast<std::streamsize>(s.size()));
    if(!f) throw std::invalid_argument("Cannot read JSON: "+path);
    return JsonParser(std::move(s),path).parse();
}
inline void fields(const JsonValue& v,std::initializer_list<const char*> allowed) {
    if(v.kind!=JsonValue::Kind::Object) throw std::invalid_argument("JSON object required");
    for(const auto& item:v.object) {bool ok=false;for(auto k:allowed)if(item.first==k)ok=true;
        if(!ok)throw std::invalid_argument("Unknown JSON field: "+item.first);}
}
inline const JsonValue* optional(const JsonValue& v,const char* key) {
    auto i=v.object.find(key);return i==v.object.end()?nullptr:&i->second;
}
inline const JsonValue& required(const JsonValue& v,const char* key) {
    auto p=optional(v,key);if(!p)throw std::invalid_argument(std::string("Missing JSON field: ")+key);return *p;
}
inline std::string string(const JsonValue& v) {
    if(v.kind!=JsonValue::Kind::String || v.text.empty() || v.text.find('\0')!=std::string::npos)
        throw std::invalid_argument("Nonempty JSON string required");return v.text;
}
inline double number(const JsonValue& v) {
    if(v.kind!=JsonValue::Kind::Number)throw std::invalid_argument("JSON number required");
    auto d=std::stod(v.text);if(!std::isfinite(d))throw std::invalid_argument("Finite JSON number required");return d;
}
inline int integer(const JsonValue& v,int lo,int hi) {
    auto d=number(v);if(d<std::numeric_limits<int>::min() || d>std::numeric_limits<int>::max() || d!=std::floor(d) || d<lo || d>hi)
        throw std::invalid_argument("JSON integer outside supported range");return int(d);
}
inline bool boolean(const JsonValue& v) {
    if(v.kind!=JsonValue::Kind::Boolean)throw std::invalid_argument("JSON boolean required");return v.boolean;
}
inline std::string quote(const std::string& s) {
    std::ostringstream o;o<<'"';constexpr char hex[]="0123456789abcdef";
    for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<char(c);else if(c<32)o<<"\\u00"<<hex[c>>4]<<hex[c&15];else o<<char(c);}o<<'"';return o.str();
}
}
