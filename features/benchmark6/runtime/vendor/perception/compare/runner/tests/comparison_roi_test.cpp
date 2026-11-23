#include "comparison_roi.hpp"
#include <iostream>
#include <limits>

int main() {
  try {
    int checks=0;
    auto check=[&](bool pass,const char* message) {
      ++checks;
      if(!pass)throw std::runtime_error(message);
    };
    auto rejects=[&](const auto& operation,const char* message) {
      bool rejected=false;
      try{operation();}catch(const std::invalid_argument&){rejected=true;}
      check(rejected,message);
    };
    using namespace r36compare::runner;
    const cv::Size source(640,544);
    const cv::Rect bottom(0,192,640,352);
    check(parseComparisonRoi("0,192,640,352",source)==bottom,"Bottom model ROI parses exactly");
    check(parseComparisonRoi("0,0,640,544",source)==cv::Rect(0,0,640,544),"Full source ROI remains supported");
    check(comparisonRoiJson(bottom,source)=="{\"x\":0,\"y\":192,\"width\":640,\"height\":352,\"source_width\":640,\"source_height\":544,\"coordinate_frame\":\"source_rectified_left\"}","Metadata includes the full source coordinate dimensions");
    for(const auto* bad:{"", "0,192,640", "0,192,640,352,0", ",192,640,352", "0,,640,352",
                         "0,192,640,", "-0,192,640,352", "+0,192,640,352", "0, 192,640,352",
                         "0,192,640,352\n", "0,192,640,352.0", "0,192,640,0x160", "0,192,640,2147483648",
                         "0,192,640,999999999999999999999999", "1,192,638,352", "0,191,640,352",
                         "0,192,639,352", "0,192,640,351", "0,192,640,354", "2,192,640,352",
                         "0,192,0,352", "0,192,640,0", "640,192,2,352"})
      rejects([&]{parseComparisonRoi(bad,source);},"Reject malformed, odd, empty or out-of-source comparison ROI");
    rejects([&]{validateComparisonRoi(bottom,{639,544});},"Reject odd source width");
    rejects([&]{validateComparisonRoi(bottom,{640,543});},"Reject odd source height");
    rejects([&]{validateComparisonRoi({0,0,2,2},{8194,2});},"Reject unsupported source dimensions");
    rejects([&]{validateComparisonRoi({std::numeric_limits<int>::max()-1,0,2,2},source);},"Containment is checked without integer overflow");
    requireMatchingCropComparisonRoi("crop",bottom,bottom);
    check(true,"Matching crop/comparison ROI is accepted");
    rejects([&]{requireMatchingCropComparisonRoi("crop",{0,0,640,544},bottom);},"Crop cannot silently compare a different ROI");
    requireMatchingCropComparisonRoi("strict",bottom,{0,0,640,544});
    requireMatchingCropComparisonRoi("resize",bottom,{0,0,640,544});
    check(true,"Strict/resize output may be masked to an explicit comparison ROI");

    std::vector<std::uint16_t> original(std::size_t(source.width)*source.height);
    for(std::size_t i=0;i<original.size();++i)original[i]=std::uint16_t((i*137)%65536);
    original[std::size_t(192)*640]=65535;
    auto masked=original;
    maskOutsideComparisonRoi(masked,source,{0,0,640,544});
    check(masked==original,"Full ROI preserves every Q8 byte including zero and maximum");
    maskOutsideComparisonRoi(masked,source,bottom);
    check(masked.size()==original.size(),"Mask retains full source dimensions");
    check(std::all_of(masked.begin(),masked.begin()+192*640,[](auto v){return v==0;}),"Top 192 rows become invalid zero");
    check(std::equal(masked.begin()+192*640,masked.end(),original.begin()+192*640),"Bottom native Q8 remains bit exact without disparity scaling");
    check(masked[192*640]==65535,"Valid maximum Q8 survives ROI masking");
    auto twice=masked;maskOutsideComparisonRoi(twice,source,bottom);
    check(twice==masked,"ROI masking is idempotent");

    const cv::Size small(8,6);const cv::Rect interior(2,2,4,2);
    std::vector<std::uint16_t> values(48,123);
    maskOutsideComparisonRoi(values,small,interior);
    bool interior_exact=true;
    for(int y=0;y<small.height;++y)for(int x=0;x<small.width;++x)
      interior_exact&=values[std::size_t(y)*small.width+x]==(interior.contains({x,y})?123:0);
    check(interior_exact,"General even ROI masks top/bottom and both horizontal margins");
    std::vector<std::uint16_t> short_storage(47,123);
    rejects([&]{maskOutsideComparisonRoi(short_storage,small,interior);},"Wrong full-frame Q8 storage is rejected");
    check(short_storage==std::vector<std::uint16_t>(47,123),"Invalid storage is rejected before mutation");
    std::cout<<"comparison ROI contract: "<<checks<<" checks passed\n";
    return 0;
  }catch(const std::exception& error){
    std::cerr<<"comparison ROI contract failed: "<<error.what()<<'\n';
    return 1;
  }
}
