#include "perception_ros_so_demo/stereo_calibration.hpp"
#include "perception_ros_so_demo/cloud_output.hpp"
#include <cstring>
#include <fstream>
#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>

static void expectFailure(const std::function<void()>& test) {
    bool failed=false;try{test();}catch(const std::invalid_argument&){failed=true;}
    if(!failed)throw std::runtime_error("Expected rejection did not occur");
}
static void require(bool ok,const char* detail) {if(!ok)throw std::runtime_error(detail);}
int main(int argc,char** argv) {
    using namespace ros_so_demo;
    sensor_msgs::msg::CameraInfo left,right;left.width=right.width=640;left.height=right.height=544;
    left.header.stamp.sec=right.header.stamp.sec=123;left.header.stamp.nanosec=right.header.stamp.nanosec=456;
    left.header.frame_id="left_optical";right.header.frame_id="right_optical";
    left.p={400,0,320,0,0,401,272,0,0,0,1,0};right.p=left.p;right.p[3]=-24;
    const auto original_right=right;
    auto k=stereoIntrinsics(left,right,left.header,640,544);
    require(k.fx==400&&k.fy==401&&k.bxf==24000&&k.cx==320&&k.cy==272,"P-derived calibration/unit mismatch");
    right.header.stamp.nanosec++;expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    right.p[2]++;expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    right.p[3]=24;expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    right.p[3]=0;expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    right.p[5]=std::numeric_limits<double>::quiet_NaN();expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    right.width=1280;expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    right.p[8]=0.1;expectFailure([&]{stereoIntrinsics(left,right,left.header,640,544);});right=original_right;
    left.header.frame_id="wrong";expectFailure([&]{stereoIntrinsics(left,right,original_right.header,640,544);});left.header.frame_id="left_optical";
    psdk_cloud_point points[2]{{1,-2,3,0.25f,-7,10,20,30,0},{4,5,6,0.5f,1025,255,128,1,0}};
    psdk_cloud_view cloud{};std::strcpy(cloud.name,"camera");cloud.points=points;cloud.point_count=2;
    auto msg=makePointCloud2(cloud,left.header,"base_link");
    require(msg.header.frame_id=="left_optical"&&msg.header.stamp.sec==123&&msg.header.stamp.nanosec==456,"Optical identity lost");
    require(!msg.is_bigendian&&msg.width==2&&msg.height==1&&msg.point_step==24&&msg.row_step==48&&msg.data.size()==48,"PointCloud2 layout mismatch");
    require(msg.fields.size()==6&&msg.fields[3].name=="rgb"&&msg.fields[3].offset==12&&msg.fields[5].datatype==5,"PointCloud2 fields mismatch");
    require(msg.data[0]==0&&msg.data[1]==0&&msg.data[2]==128&&msg.data[3]==63,"XYZ not little endian float");
    require(msg.data[12]==30&&msg.data[13]==20&&msg.data[14]==10&&msg.data[15]==0,"RGB bit packing wrong");
    require(msg.data[20]==249&&msg.data[21]==255&&msg.data[22]==255&&msg.data[23]==255,"Signed label packing wrong");
    for(const auto* name:{"robot","ground","obstacle","semantic"}) {
        std::strcpy(cloud.name,name);require(makePointCloud2(cloud,left.header,"base_link").header.frame_id=="base_link","Robot cloud frame mismatch");
    }
    std::strcpy(cloud.name,"camera");
    auto broken=cloud;broken.points=nullptr;expectFailure([&]{makePointCloud2(broken,left.header,"base_link");});
    broken=cloud;std::memset(broken.name,'x',sizeof(broken.name));expectFailure([&]{makePointCloud2(broken,left.header,"base_link");});
    broken=cloud;std::strcpy(broken.name,"unknown");expectFailure([&]{makePointCloud2(broken,left.header,"base_link");});
    const auto out=std::filesystem::path(argc>1?argv[1]:"helper_output");std::filesystem::create_directories(out);
    psdk_full_result_view result{};result.frame_id=99;result.timestamp_ns=123000000456ULL;result.clouds=&cloud;result.cloud_count=1;
    savePlyClouds(result,out,left.header,"base_link");
    std::ifstream ply(out/"camera.ply",std::ios::binary);std::string text((std::istreambuf_iterator<char>(ply)),{});
    require(text.find("comment unit=m\ncomment frame_id=left_optical\n")!=std::string::npos&&text.find("element vertex 2\n")!=std::string::npos,"PLY metadata mismatch");
    require(text.find("1 -2 3 10 20 30 0.25 -7\n")!=std::string::npos,"PLY point/color/confidence/label mismatch");
    std::ifstream meta(out/"clouds.json",std::ios::binary);std::string json((std::istreambuf_iterator<char>(meta)),{});
    require(json.find("\"frame_id\":\"left_optical\"")!=std::string::npos&&json.find("\"unit\":\"m\"")!=std::string::npos,"Cloud metadata missing");
    cloud.point_count=0;cloud.points=nullptr;require(makePointCloud2(cloud,left.header,"base_link").data.empty(),"Empty cloud invalid");
    std::cout<<"PASS: calibration units/exact stamps/negative cases; PointCloud2 little-endian fields/colors/labels/frames; PLY and metadata\n";
}
