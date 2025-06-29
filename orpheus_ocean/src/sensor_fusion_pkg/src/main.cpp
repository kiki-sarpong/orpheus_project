// #include "rclcpp/rclcpp.hpp"
#include "FusedData/IMUDepthFusion.h"
#include "FusedData/VerticalVelocityPublisher.h"



int main (int argc, char** argv){

    rclcpp::init(argc, argv); // Initialize ROS2 node

    std::string nodeName = "fused_data"; // Node name

    // Create main node
    auto DataFusion = std::make_shared<IMUDepthFusion>(nodeName);   
    
    // Create multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    // rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(DataFusion);
    executor.spin();

    rclcpp::shutdown();  // Shutdown ROS2 node

    return 0;
}