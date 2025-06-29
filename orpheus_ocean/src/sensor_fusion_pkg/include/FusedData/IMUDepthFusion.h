#ifndef _IMU_DEPTH_FUSION_H
#define _IMU_DEPTH_FUSION_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"


class IMUDepthFusion : public rclcpp::Node {
    public:
        IMUDepthFusion(std::string nodeName);
        const int callbackFrequencyMilliSeconds = 500;  // Numer in milliseconds
        // Allowed difference in time b/n IMU and depth sensor
        double timeSyncThreshold = 0.5;
        // Selected weight for filtering. Should be b/n 0 - 1
        double alphaWeight = 0.8;
        static double fusedVelocity(double velocityIMU, double velocityDepth, double alphaWeight);


    private:
        void runFusion();
        rclcpp::TimerBase::SharedPtr fusionTimer_;
};

#endif /* _IMU_DEPTH_FUSION_H */