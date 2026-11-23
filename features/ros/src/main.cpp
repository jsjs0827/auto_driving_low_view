#include "perception_ros_so_demo/perception_node.hpp"

int main(int argc,char** argv) {
    rclcpp::init(argc,argv);int code=0;
    try{auto node=std::make_shared<ros_so_demo::PerceptionNode>();rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);executor.spin();node->close();}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';code=1;}
    rclcpp::shutdown();return code;
}
