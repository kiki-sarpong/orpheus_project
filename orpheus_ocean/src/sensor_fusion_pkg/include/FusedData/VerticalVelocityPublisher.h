#ifndef _VERTICAL_VELOCITY_PUB_H
#define _VERTICAL_VELOCITY_PUB_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


class IMUDepthFusion; // Forward declaration

namespace Publishers {

    class VerticalVelocityPublisher {
        public:
            VerticalVelocityPublisher(IMUDepthFusion* mainNode);
            // Call vertical velocity publisher
            void vertical_velocity_publisher(double velocity);
    

        private:
            IMUDepthFusion* mainNode;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr VerticalVelocityPub;
            rclcpp::TimerBase::SharedPtr timer_;
            std::string topicName = "vertical_velocity";
            const int qosDepth = 10;
    };

}

#endif /* _VERTICAL_VELOCITY_PUB_H */