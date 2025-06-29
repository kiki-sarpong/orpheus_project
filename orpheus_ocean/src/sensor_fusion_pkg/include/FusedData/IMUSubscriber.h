#ifndef _IMU_SUBSCRIBER_H
#define _IMU_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


class IMUDepthFusion; // Forward declaration

namespace Subscribers {

    class IMUSubscriber {
        public:
            IMUSubscriber(IMUDepthFusion* mainNode);
            void calculateVelocity();
            double previousTime, currentTime;
            double velocityZ;
            double accelerationZ;

    
        private:
            IMUDepthFusion* mainNode;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber;
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
            const int qosDepth = 10;
            std::string topicName = "/imu/data";
    
    };

}

#endif /* _IMU_SUBSCRIBER_H */