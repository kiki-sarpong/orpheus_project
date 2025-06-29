#include "FusedData/IMUSubscriber.h"
#include "FusedData/IMUDepthFusion.h"


using namespace Subscribers;

IMUSubscriber::IMUSubscriber(IMUDepthFusion* main_node) : mainNode(main_node) {
    // Set QOS profile for services and topics
    rclcpp::QoS qosProfile(qosDepth);
    qosProfile.reliable();  // Reliable communication
    qosProfile.durability_volatile();

    // Initialze values
    previousTime = 0.0; 
    velocityZ = 0.0;

    // Initialize IMU subscriber
    imuSubscriber = mainNode->create_subscription<sensor_msgs::msg::Imu>(
        topicName,  qosProfile,
         std::bind(&Subscribers::IMUSubscriber::imu_callback, this, std::placeholders::_1)
       );
 
       RCLCPP_INFO(mainNode->get_logger(), "Started the IMU sensor subscriber.");
}


void IMUSubscriber::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Assign time associate with msg capture
    currentTime = mainNode->get_clock()->now().seconds();
    accelerationZ = msg->linear_acceleration.z;
    // Call velocity calculation
    IMUSubscriber::calculateVelocity();
}

/**
 * @brief Calculates the velocity given the IMU Z accelerations
 */
void IMUSubscriber::calculateVelocity(){
    if (previousTime == 0.0){
        previousTime = currentTime;
        return;
    }

    // Calculate dt and velocity
    double dt = currentTime - previousTime;
    // This calculated velocity -> v = v + a * dt;
    velocityZ = velocityZ + accelerationZ * dt;

    // Assign "previous" data
    previousTime = currentTime;

    RCLCPP_DEBUG(mainNode->get_logger(), "IMU velocity = %.4f m/s", velocityZ);
}