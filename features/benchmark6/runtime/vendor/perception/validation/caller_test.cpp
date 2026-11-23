#include "perception_ros_so_demo/sdk_caller.hpp"
#include "perception_temporal_sdk.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

uint32_t callAlgorithmFromExistingRosCallback(ros_so_demo::Caller&,const psdk_full_frame&,psdk_status&);

static void require(bool valid,const char* check,const psdk_status& status) {
    if(!valid)throw std::runtime_error(std::string(check)+": code="+std::to_string(status.code)+" "+status.detail);
}
int main(int argc,char** argv) try {
    if(argc!=2)throw std::runtime_error("Pass queue_only.json");
    ros_so_demo::Caller caller;auto status=ros_so_demo::newStatus();
    require(caller.open(argv[1],status)==0,"open",status);
    require(caller.open(argv[1],status)==2001&&status.code==2001,"duplicate open",status);
    require(caller.command(1,status)==0,"activate queue-only DAG",status);
    std::vector<uint8_t> nv12(640*544*3/2,128);
    psdk_full_frame frame{};frame.struct_size=sizeof(frame);frame.abi_version=PSDK_FULL_ABI_VERSION;
    frame.left.struct_size=sizeof(frame.left);frame.left.abi_version=PSDK_ABI_VERSION;
    frame.left.format=PSDK_FORMAT_NV12;frame.left.width=640;frame.left.height=544;frame.left.stride=640;
    frame.left.data=nv12.data();frame.left.bytes=nv12.size();frame.intrinsics={640,544,298.4,298.4,313.8,274.3,17935.5};
    ros_so_demo::Result retained;
    for(uint64_t i=0;i<7;++i){
        frame.left.frame_id=100+i;frame.left.timestamp_ns=1000000000+i*30000000;
        ros_so_demo::Result result;
        require(caller.process(frame,result,status)==0,"process",status);
        psdk_full_result_view view{};require(result.view(view,status)==0,"view",status);
        require(view.frame_id==frame.left.frame_id&&view.timestamp_ns==frame.left.timestamp_ns,"identity",status);
        require(view.task_trace_json&&std::string(view.task_trace_json).find("history")!=std::string::npos,"real DAG trace",status);
        if(i==0)retained=std::move(result);
        if(i==6){psdk_temporal_view history{};history.struct_size=sizeof(history);history.abi_version=PSDK_TEMPORAL_ABI_VERSION;
            require(psdk_full_result_get_temporal_view(result.get(),&history,&status)==0,"temporal view",status);
            require(history.frame_count==3&&history.frames[0].left.frame_id==102&&history.frames[2].left.frame_id==106,"temporal interval",status);}
    }
    require(callAlgorithmFromExistingRosCallback(caller,frame,status)==0,"existing ROS callback bridge",status);
    frame.left.stride=639;ros_so_demo::Result bad;
    require(caller.process(frame,bad,status)!=0&&status.code!=0&&bad.get()==nullptr,"invalid input",status);
    require(caller.close(status)==0,"close",status);require(caller.close(status)==0,"idempotent close",status);
    psdk_full_result_view view{};require(retained.view(view,status)==0&&view.frame_id==100,"retained result",status);
    retained.reset();std::cout<<"Real r36 SO/DAG caller ownership, history and error propagation passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;
}
