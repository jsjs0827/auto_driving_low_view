#include "perception_ros_so_demo/image_messages.hpp"
#include <stdexcept>

namespace ros_so_demo {
using Image=sensor_msgs::msg::Image;
uint64_t timestamp(const std_msgs::msg::Header& header) {
    if(header.stamp.sec<0||header.stamp.nanosec>=1000000000u)
        throw std::invalid_argument("Invalid ROS capture timestamp");
    const auto ns=uint64_t(header.stamp.sec)*1000000000u+header.stamp.nanosec;
    if(!ns)throw std::invalid_argument("Capture timestamp is zero; do not replace it with host receive time");
    return ns;
}
psdk_frame imageView(const Image& image,uint64_t capture_id,uint64_t stamp) {
    psdk_frame f{};f.struct_size=sizeof(f);f.abi_version=PSDK_ABI_VERSION;
    if(image.encoding=="nv12"||image.encoding=="NV12")f.format=PSDK_FORMAT_NV12;
    else if(image.encoding=="i420"||image.encoding=="I420")f.format=PSDK_FORMAT_I420;
    else throw std::invalid_argument("Expected original nv12/i420, received "+image.encoding+
        "; the old percep_demo display topics are incompatible. Stop old ai_node and run tools/run_camera_x5.sh");
    if(!image.width||!image.height||image.width>4096||image.height>4096||
       ((image.width|image.height|image.step)&1u)||image.step<image.width)
        throw std::invalid_argument("Invalid YUV420 width/height/Y stride");
    const auto bytes=uint64_t(image.step)*(uint64_t(image.height)+image.height/2);
    if(image.data.size()<bytes)throw std::invalid_argument("Image buffer is shorter than Y+UV storage");
    f.width=image.width;f.height=image.height;f.stride=image.step;
    f.data=image.data.data();f.bytes=bytes;f.frame_id=capture_id;f.timestamp_ns=stamp;return f;
}

} // namespace ros_so_demo
