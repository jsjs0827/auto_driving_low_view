#include "perception_ros_so_demo/cloud_output.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

namespace ros_so_demo {
std::string cloudName(const psdk_cloud_view& cloud) {
    const auto end=static_cast<const char*>(std::memchr(cloud.name,0,sizeof(cloud.name)));
    if(!end)throw std::invalid_argument("SDK cloud name is not terminated");
    const std::string name(cloud.name,end);
    if(name!="camera"&&name!="robot"&&name!="ground"&&name!="obstacle"&&name!="semantic")
        throw std::invalid_argument("Unknown SDK cloud coordinate system: "+name);
    return name;
}
std::string cloudFrame(const psdk_cloud_view& cloud,const std::string& optical_frame,
                              const std::string& robot_frame) {
    const auto name=cloudName(cloud);
    const auto& frame=name=="camera"?optical_frame:robot_frame;
    if(frame.empty())throw std::invalid_argument("Cloud frame_id must not be empty");
    return frame;
}
void validateCloud(const psdk_cloud_view& cloud) {
    cloudName(cloud);
    if(cloud.point_count&&!cloud.points)throw std::invalid_argument("SDK cloud points are null");
    if(cloud.point_count>std::numeric_limits<uint32_t>::max()/24u)
        throw std::invalid_argument("SDK cloud exceeds ROS PointCloud2 row storage");
}
void cloudU32LE(uint8_t* destination,uint32_t value) {
    for(unsigned i=0;i<4;++i)destination[i]=uint8_t(value>>(8*i));
}
void cloudFloatLE(uint8_t* destination,float value) {
    static_assert(sizeof(float)==4,"PointCloud2 requires 32-bit float");
    uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));cloudU32LE(destination,bits);
}
sensor_msgs::msg::PointCloud2 makePointCloud2(const psdk_cloud_view& cloud,
        const std_msgs::msg::Header& capture,const std::string& robot_frame) {
    validateCloud(cloud);
    sensor_msgs::msg::PointCloud2 out;out.header=capture;
    out.header.frame_id=cloudFrame(cloud,capture.frame_id,robot_frame);
    out.height=1;out.width=uint32_t(cloud.point_count);out.is_bigendian=false;
    out.point_step=24;out.row_step=out.width*out.point_step;out.is_dense=true;
    auto field=[&](const char* name,uint32_t offset,uint8_t datatype) {
        sensor_msgs::msg::PointField f;f.name=name;f.offset=offset;f.datatype=datatype;f.count=1;out.fields.push_back(f);
    };
    using Field=sensor_msgs::msg::PointField;
    field("x",0,Field::FLOAT32);field("y",4,Field::FLOAT32);field("z",8,Field::FLOAT32);
    field("rgb",12,Field::FLOAT32);field("confidence",16,Field::FLOAT32);field("label",20,Field::INT32);
    out.data.resize(out.row_step);
    for(uint32_t i=0;i<out.width;++i) {
        const auto& p=cloud.points[i];auto* row=out.data.data()+std::size_t(i)*24;
        cloudFloatLE(row,p.x);cloudFloatLE(row+4,p.y);cloudFloatLE(row+8,p.z);
        cloudU32LE(row+12,(uint32_t(p.r)<<16)|(uint32_t(p.g)<<8)|p.b);
        cloudFloatLE(row+16,p.confidence);cloudU32LE(row+20,uint32_t(p.label));
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))out.is_dense=false;
    }
    return out;
}
std::string cloudJsonQuote(const std::string& text) {
    std::string out="\"";
    for(unsigned char c:text) {
        if(c=='"'||c=='\\'){out+='\\';out+=char(c);}
        else if(c<32)throw std::invalid_argument("Cloud frame_id contains a control character");
        else out+=char(c);
    }
    return out+'"';
}
void writePly(const psdk_cloud_view& cloud,const std::filesystem::path& path,
                     const std::string& frame) {
    validateCloud(cloud);cloudJsonQuote(frame);
    if(frame.empty())throw std::invalid_argument("PLY frame_id must not be empty");
    std::ofstream file(path,std::ios::binary);
    if(!file)throw std::runtime_error("Cannot create PLY: "+path.u8string());
    file<<"ply\nformat ascii 1.0\ncomment unit=m\ncomment frame_id="<<frame
        <<"\ncomment color=actual_SDK_RGB\nelement vertex "<<cloud.point_count
        <<"\nproperty float x\nproperty float y\nproperty float z"
        <<"\nproperty uchar red\nproperty uchar green\nproperty uchar blue"
        <<"\nproperty float confidence\nproperty int label\nend_header\n";
    file<<std::setprecision(std::numeric_limits<float>::max_digits10);
    for(uint64_t i=0;i<cloud.point_count;++i) {
        const auto& p=cloud.points[i];
        file<<p.x<<' '<<p.y<<' '<<p.z<<' '<<unsigned(p.r)<<' '<<unsigned(p.g)<<' '<<unsigned(p.b)
            <<' '<<p.confidence<<' '<<p.label<<'\n';
    }
    file.close();if(!file)throw std::runtime_error("Failed writing PLY: "+path.u8string());
}
void savePlyClouds(const psdk_full_result_view& result,const std::filesystem::path& directory,
        const std_msgs::msg::Header& capture,const std::string& robot_frame) {
    if(result.cloud_count&&!result.clouds)throw std::invalid_argument("SDK cloud views are null");
    std::ofstream metadata(directory/"clouds.json",std::ios::binary);
    if(!metadata)throw std::runtime_error("Cannot create clouds.json");
    metadata<<"{\"frame_id\":"<<result.frame_id<<",\"timestamp_ns\":"<<result.timestamp_ns
            <<",\"unit\":\"m\",\"color\":\"actual_SDK_RGB\",\"clouds\":[";
    for(uint32_t i=0;i<result.cloud_count;++i) {
        const auto& cloud=result.clouds[i];const auto name=cloudName(cloud);
        const auto frame=cloudFrame(cloud,capture.frame_id,robot_frame);
        writePly(cloud,directory/(name+".ply"),frame);
        if(i)metadata<<',';
        metadata<<"{\"name\":"<<cloudJsonQuote(name)<<",\"file\":"<<cloudJsonQuote(name+".ply")
                <<",\"frame_id\":"<<cloudJsonQuote(frame)<<",\"point_count\":"<<cloud.point_count
                <<",\"axes\":"<<cloudJsonQuote(name=="camera"?"x:right,y:down,z:forward":"configured_robot_extrinsics")<<'}';
    }
    metadata<<"]}\n";metadata.close();if(!metadata)throw std::runtime_error("Failed writing clouds.json");
}
} // namespace ros_so_demo
