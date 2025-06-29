#include "FusedData/VerticalVelocityPublisher.h"
#include  "FusedData/IMUDepthFusion.h"


using namespace Publishers;

VerticalVelocityPublisher::VerticalVelocityPublisher(IMUDepthFusion* main_node) : mainNode(main_node){
    // Set QOS profile for services and topics
    rclcpp::QoS qosProfile(qosDepth);
    qosProfile.reliable();  // Reliable communication
    qosProfile.durability_volatile();

    VerticalVelocityPub = mainNode->create_publisher<std_msgs::msg::Float32>(topicName, qosProfile);

    RCLCPP_INFO(mainNode->get_logger(), "%s has been initialized.", topicName.c_str());
}


void VerticalVelocityPublisher::vertical_velocity_publisher(double velocity){
    auto msg = std_msgs::msg::Float32();
    msg.data = velocity;
    VerticalVelocityPub->publish(msg);
}