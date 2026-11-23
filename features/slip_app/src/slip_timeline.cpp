#include "slip_timeline.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace perception::slip_app {
namespace fs=std::filesystem;
namespace {
std::vector<std::string> csv(const std::string& line) {
    std::vector<std::string> out;std::string value;
    bool quoted=false,closed=false;
    for(std::size_t i=0;i<line.size();++i) {
        const char c=line[i];
        if(c=='\0'||c=='\t'||c=='\r'||c=='\n')throw std::invalid_argument("CSV contains a forbidden control/delimiter character");
        if(quoted) {
            if(c=='"') {
                if(i+1<line.size()&&line[i+1]=='"'){value+='"';++i;}
                else {quoted=false;closed=true;}
            } else value+=c;
        } else if(c==',') {out.push_back(value);value.clear();closed=false;}
        else if(c=='"'&&!closed&&value.empty())quoted=true;
        else {
            if(closed||c=='"')throw std::invalid_argument("Malformed CSV quoting");
            value+=c;
        }
    }
    if(quoted)throw std::invalid_argument("Unterminated CSV quote; multiline fields are forbidden");
    out.push_back(value);return out;
}
std::uint64_t uint64(const std::string& text,bool positive) {
    if(text.empty())throw std::invalid_argument("Empty uint64 CSV value");
    std::uint64_t value=0;
    for(const unsigned char c:text) {
        if(c<'0'||c>'9'||value>(std::numeric_limits<std::uint64_t>::max()-(c-'0'))/10)
            throw std::invalid_argument("Expected decimal uint64 CSV value: "+text);
        value=value*10+(c-'0');
    }
    if(positive&&!value)throw std::invalid_argument("Capture/motion timestamp_ns must be nonzero");
    return value;
}
double real(const std::string& text) {
    std::istringstream input(text);input.imbue(std::locale::classic());input>>std::noskipws;
    double value=0;
    if(!(input>>value)||input.peek()!=std::char_traits<char>::eof()||!std::isfinite(value))
        throw std::invalid_argument("Expected a finite decimal motion value: "+text);
    return value;
}
struct Reader {
    fs::path file;std::ifstream input;std::size_t line_number=0,max_rows;
    std::map<std::string,std::size_t> columns;
    Reader(const fs::path& path,const std::set<std::string>& allowed,const std::set<std::string>& required,
           std::size_t rows,std::uintmax_t bytes):file(fs::absolute(path)),max_rows(rows) {
        if(!rows||!bytes||!fs::is_regular_file(file)||fs::file_size(file)>bytes)
            throw std::invalid_argument("CSV missing, nonregular, or exceeds bounded file size: "+file.u8string());
        input.open(file,std::ios::binary);
        std::string header;if(!std::getline(input,header))throw std::invalid_argument("CSV header is missing");
        ++line_number;if(!header.empty()&&header.back()=='\r')header.pop_back();
        if(header.size()>=3&&static_cast<unsigned char>(header[0])==0xef&&static_cast<unsigned char>(header[1])==0xbb&&static_cast<unsigned char>(header[2])==0xbf)
            header.erase(0,3);
        const auto fields=csv(header);
        for(std::size_t i=0;i<fields.size();++i)
            if(!allowed.count(fields[i])||!columns.emplace(fields[i],i).second)
                throw std::invalid_argument("CSV has unknown/duplicate header: "+fields[i]);
        for(const auto& name:required)if(!columns.count(name))throw std::invalid_argument("CSV missing required header: "+name);
    }
    bool next(std::vector<std::string>& out) {
        std::string line;if(!std::getline(input,line)) {
            if(input.bad())throw std::runtime_error("Cannot read CSV: "+file.u8string());
            return false;
        }
        ++line_number;if(line_number-1>max_rows||line.size()>1024*1024)
            throw std::invalid_argument("CSV exceeds bounded row count/line size");
        if(!line.empty()&&line.back()=='\r')line.pop_back();
        out=csv(line);
        if(out.size()!=columns.size())throw std::invalid_argument("CSV row has wrong column count at line "+std::to_string(line_number));
        return true;
    }
    std::string get(const std::vector<std::string>& row,const char* name) const {
        const auto found=columns.find(name);return found==columns.end()?std::string():row[found->second];
    }
    fs::path image(const std::string& text,std::size_t expected,bool required) const {
        if(text.empty()) {if(required)throw std::invalid_argument("Required input path is empty");return {};}
        auto path=fs::u8path(text);if(path.is_relative())path=file.parent_path()/path;
        path=fs::weakly_canonical(path);
        if(!fs::is_regular_file(path)||fs::file_size(path)!=expected)
            throw std::invalid_argument("Input must be exactly "+std::to_string(expected)+" bytes: "+path.u8string());
        return path;
    }
};
}
std::vector<TimelineRow> readTimeline(const fs::path& path,const TimelineOptions& options) {
    if(options.width<2||options.height<2||options.width>4096||options.height>4096||((options.width|options.height)&1))
        throw std::invalid_argument("Timeline NV12 source size must be positive, even and bounded");
    const auto pixels=std::size_t(options.width)*std::size_t(options.height);
    Reader reader(path,{"timestamp_ns","frame_id","left_nv12","q8_u16","right_nv12"},
                  options.original_q8?std::set<std::string>{"timestamp_ns","left_nv12","q8_u16"}:
                                      std::set<std::string>{"timestamp_ns","left_nv12","right_nv12"},
                  options.max_rows,options.max_bytes);
    std::vector<TimelineRow> result;std::vector<std::string> values;std::set<std::uint64_t> ids;
    while(reader.next(values)) {
        TimelineRow row;row.timestamp_ns=uint64(reader.get(values,"timestamp_ns"),true);
        if(!result.empty()&&row.timestamp_ns<=result.back().timestamp_ns)
            throw std::invalid_argument("Timeline timestamp_ns must be strictly increasing; duplicates/backward captures are forbidden");
        const bool explicit_id=reader.columns.count("frame_id")!=0;
        row.frame_id=explicit_id?uint64(reader.get(values,"frame_id"),false):row.timestamp_ns;
        row.identity_source=explicit_id?"recorded_frame_id":"source_timestamp_ns";
        if(!ids.insert(row.frame_id).second)throw std::invalid_argument("Duplicate recorded frame_id");
        row.left_nv12=reader.image(reader.get(values,"left_nv12"),pixels*3/2,true);
        row.q8_u16=reader.image(reader.get(values,"q8_u16"),pixels*2,options.original_q8);
        row.right_nv12=reader.image(reader.get(values,"right_nv12"),pixels*3/2,!options.original_q8);
        if(!row.right_nv12.empty()&&fs::equivalent(row.left_nv12,row.right_nv12))
            throw std::invalid_argument("Stereo eyes must be different files");
        result.push_back(std::move(row));
    }
    if(result.empty())throw std::invalid_argument("Timeline contains no capture rows");
    return result;
}
std::vector<MotionRow> readMotionCsv(const fs::path& path,std::size_t max_rows,std::uintmax_t max_bytes) {
    Reader reader(path,{"timestamp_ns","linear_speed_mps","yaw_rate_radps"},
                  {"timestamp_ns","linear_speed_mps","yaw_rate_radps"},max_rows,max_bytes);
    std::vector<MotionRow> result;std::vector<std::string> values;
    while(reader.next(values)) {
        MotionRow row{uint64(reader.get(values,"timestamp_ns"),true),
                      real(reader.get(values,"linear_speed_mps")),real(reader.get(values,"yaw_rate_radps"))};
        if(!result.empty()&&row.timestamp_ns<=result.back().timestamp_ns)
            throw std::invalid_argument("Motion timestamps must be strictly increasing");
        result.push_back(row);
    }
    if(result.empty())throw std::invalid_argument("Motion CSV contains no samples");
    return result;
}
std::vector<std::uint8_t> readExactBytes(const fs::path& path,std::size_t expected) {
    if(!fs::is_regular_file(path)||fs::file_size(path)!=expected)
        throw std::invalid_argument("Input changed or has wrong byte size: "+path.u8string());
    std::vector<std::uint8_t> out(expected);std::ifstream in(path,std::ios::binary);
    in.read(reinterpret_cast<char*>(out.data()),std::streamsize(out.size()));
    if(!in)throw std::runtime_error("Cannot read input: "+path.u8string());
    return out;
}
std::vector<std::uint16_t> readQ8LittleEndian(const fs::path& path,std::size_t pixels) {
    const auto bytes=readExactBytes(path,pixels*2);std::vector<std::uint16_t> out(pixels);
    for(std::size_t i=0;i<pixels;++i)out[i]=std::uint16_t(bytes[2*i])|(std::uint16_t(bytes[2*i+1])<<8);
    return out;
}
std::size_t initialMotionIndex(const std::vector<MotionRow>& samples,std::uint64_t stamp) {
    const auto it=std::upper_bound(samples.begin(),samples.end(),stamp,
        [](std::uint64_t value,const MotionRow& row){return value<row.timestamp_ns;});
    return it==samples.begin()?0:std::size_t(it-samples.begin()-1);
}
std::size_t motionEndpoint(const std::vector<MotionRow>& samples,std::size_t begin,std::uint64_t stamp) {
    if(begin>samples.size())throw std::invalid_argument("Motion cursor is outside sample array");
    if(begin>0&&samples[begin-1].timestamp_ns>stamp)return begin;
    auto end=begin;while(end<samples.size()&&samples[end].timestamp_ns<=stamp)++end;
    if(end<samples.size())++end;
    return end;
}
} // namespace perception::slip_app
