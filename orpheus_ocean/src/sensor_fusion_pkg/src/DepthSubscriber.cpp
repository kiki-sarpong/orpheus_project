
#include "FusedData/DepthSubscriber.h"
#include "FusedData/IMUDepthFusion.h"


using namespace Subscribers;

DepthSubscriber::DepthSubscriber(IMUDepthFusion* main_node) : mainNode(main_node){
    // Set QOS profile for services and topics
    rclcpp::QoS qosProfile(qosDepth);
    qosProfile.reliable();  // Reliable communication
    qosProfile.durability_volatile();

    // Initialze "previous" values
    previousTime = 0.0; 
    previousDepthZ = 0.0; 

    // Initialize depth subscriber
    depthSubscriber = mainNode->create_subscription<std_msgs::msg::Float32>(
       topicName,  qosProfile,
        std::bind(&Subscribers::DepthSubscriber::depth_callback, this, std::placeholders::_1)
      );

      RCLCPP_INFO(mainNode->get_logger(), "Started the Depth sensor subscriber.");

}


void DepthSubscriber::depth_callback(const std_msgs::msg::Float32::SharedPtr msg){
    // Assign time associate with msg capture
   currentTime = mainNode->get_clock()->now().seconds();
   currentDepthZ = msg->data;
   // Call velocity calculation
   DepthSubscriber::calculateVelocity();
}

/**
 * @brief Calculates the velocity given the depth measurements
 */
void DepthSubscriber::calculateVelocity(){
    if (previousTime == 0.0){
        velocityZ = 0.0;
        previousTime = currentTime;
        previousDepthZ = currentDepthZ;
        return;
    }
        
    double dt = currentTime - previousTime;
    double depthDifference = currentDepthZ - previousDepthZ;
    
    // This avoids a ZERODIVISIONERROR
    if (dt == 0.0){
        RCLCPP_ERROR(mainNode->get_logger(), "Zero division error in depth velocity calculation!");
        return;
    }

    // Velocity calculation -> Change in distance / change in time
    velocityZ  = depthDifference/dt;
    
    // Assign "previous" data
    previousDepthZ = currentDepthZ;
    previousTime = currentTime;

    RCLCPP_DEBUG(mainNode->get_logger(), "Depth velocity = %.4f m/s", velocityZ);
}
