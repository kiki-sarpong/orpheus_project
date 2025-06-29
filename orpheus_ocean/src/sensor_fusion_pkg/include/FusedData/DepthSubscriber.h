#ifndef DEPTH_SUBSCRIBER_H
#define DEPTH_SUBSCRIBER_H

#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"


class IMUDepthFusion; // Forward declaration

namespace Subscribers {

    class DepthSubscriber {
        public:
            DepthSubscriber(IMUDepthFusion* mainNode);
            void depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
            void calculateVelocity();
            double previousTime, currentTime;
            double velocityZ;
            double previousDepthZ;
            double currentDepthZ;
    
    
        private:
            IMUDepthFusion* mainNode;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depthSubscriber;
            const int qosDepth = 10;
            std::string topicName = "/depth";
            
    };

}

#endif /* DEPTH_SUBSCRIBER_H */