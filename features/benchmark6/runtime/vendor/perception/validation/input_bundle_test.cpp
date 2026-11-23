#include "perception/frame_input.hpp"
#include <iostream>
#include <vector>
#include <functional>
namespace {
void require(bool v,const char* m){if(!v)throw std::runtime_error(m);}
void rejects(const std::function<void()>& fn){bool rejected=false;try{fn();}catch(const std::exception&){rejected=true;}require(rejected,"Invalid input packet was accepted");}
psdk_frame image(std::vector<std::uint8_t>& bytes,std::uint32_t format,std::uint32_t stride){
    psdk_frame v{};v.struct_size=sizeof(v);v.abi_version=PSDK_ABI_VERSION;
    v.width=4;v.height=2;v.stride=stride;v.format=format;v.data=bytes.data();v.bytes=bytes.size();v.frame_id=9;v.timestamp_ns=123;
    return v;
}
}
int main(){try{
    using perception::input::FrameBundle;
    std::vector<std::uint8_t> left_bytes(18,42),right_bytes(30,17);
    auto left=image(left_bytes,PSDK_FORMAT_NV12,6); // Padded camera/ROS source.
    auto right=image(right_bytes,PSDK_FORMAT_BGR,15); // Padded recording BGR view.
    psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    frame.left=left;frame.right=&right;frame.intrinsics={4,2,2,2,1,1,100};
    auto packet=FrameBundle::borrowed(frame);
    perception::runtime::ArtifactStore store;packet.seed(store);
    require(store.get<psdk_frame>("input.left")->data==left_bytes.data(),"Left input was replaced");
    require(store.get<psdk_frame>("input.right")->data==right_bytes.data(),"Right input was replaced");
    require(store.get<perception::FrameInfo>("input.frame")->id==9,"Frame identity changed");
    require(!store.contains("input.disparity_q8"),"Absent Q8 was fabricated");
    right.timestamp_ns++;rejects([&]{FrameBundle::borrowed(frame);});right.timestamp_ns--;
    right.bytes=1;rejects([&]{FrameBundle::borrowed(frame);});right.bytes=right_bytes.size();
    frame.left.bytes=1;rejects([&]{FrameBundle::borrowed(frame);});frame.left=left;
    frame.left.format=PSDK_FORMAT_BGR;rejects([&]{FrameBundle::borrowed(frame);});frame.left=left;
    std::vector<std::uint16_t> q(8,512);
    psdk_disparity_q8 depth{};depth.struct_size=sizeof(depth);depth.abi_version=PSDK_FULL_ABI_VERSION;
    depth.width=4;depth.height=2;depth.stride_bytes=8;depth.data=q.data();depth.bytes=16;depth.frame_id=9;depth.timestamp_ns=123;
    frame.disparity=&depth;auto original=FrameBundle::borrowed(frame);perception::runtime::ArtifactStore original_store;original.seed(original_store);
    require(original_store.get<psdk_disparity_q8>("input.disparity_q8")->data==q.data(),"Original Q8 replaced");
    depth.frame_id++;rejects([&]{FrameBundle::borrowed(frame);});depth.frame_id--;
    frame.struct_size=PSDK_FULL_FRAME_V1_SIZE;frame.right=reinterpret_cast<const psdk_frame*>(1);
    require(!FrameBundle::borrowed(frame).right,"Old ABI read the right tail");
    // Offline I420 follows the same input contract and keeps the source format.
    frame.struct_size=sizeof(frame);frame.right=nullptr;frame.disparity=nullptr;frame.left.format=PSDK_FORMAT_I420;
    require(FrameBundle::borrowed(frame).left.format==PSDK_FORMAT_I420,"Offline format changed");
    std::cout<<"Input bundle: image layouts, exact pairing, original Q8, artifact seeding and old ABI passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
