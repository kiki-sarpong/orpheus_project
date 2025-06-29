#include <cmath>
#include "FusedData/IMUDepthFusion.h"
#include "FusedData/IMUSubscriber.h"
#include "FusedData/DepthSubscriber.h"
#include "FusedData/VerticalVelocityPublisher.h"


using namespace Publishers;
using namespace Subscribers;
using namespace std::chrono_literals;

IMUDepthFusion::IMUDepthFusion(std::string nodeName) : Node(nodeName) {
    // Run fusion 
    runFusion();

}

/**
 * @brief Runs the data fusion using a timer and LAMBDA callback function
 */
void IMUDepthFusion::runFusion(){
    // Create shared ptrs for objects
    std::shared_ptr<IMUSubscriber> imuSensor = std::make_shared<IMUSubscriber>(this);
    std::shared_ptr<DepthSubscriber> depthSensor = std::make_shared<DepthSubscriber>(this);
    std::shared_ptr<VerticalVelocityPublisher> velocityPub = std::make_shared<VerticalVelocityPublisher>(this);

    RCLCPP_INFO(this->get_logger(), "All subscribers and publishers initialized successfully.");

    
    std::chrono::milliseconds frequency(callbackFrequencyMilliSeconds); // Set frequency
    // Using Lambda for callback
    fusionTimer_ = this->create_wall_timer(
        frequency, [this, imuSensor, depthSensor, velocityPub]() {

            // Make sure none of the pointers are null
            if (!imuSensor || !depthSensor || !velocityPub) {
                    RCLCPP_DEBUG(this->get_logger(), "One or more components not available.");
                    return;
                }

            // Get current time
            double imuTime = imuSensor->currentTime;
            double depthTime = depthSensor->currentTime;

            /* Moved this into the individual callbacks instead */
            // imuSensor->calculateVelocity();
            // depthSensor->calculateVelocity();

            double velocityIMU = imuSensor->velocityZ;
            double velocityDepth = depthSensor->velocityZ;

            /*
            Temporal synchronization to make sure the difference in time
            between calls is reasonable.
            */ 
            if (std::abs(imuTime - depthTime) > timeSyncThreshold){
                RCLCPP_DEBUG(this->get_logger(), "Sensor data rejected, readings are too far apart!");
                return;
            }

            double velocityFused = IMUDepthFusion::fusedVelocity(velocityIMU, velocityDepth, alphaWeight);
            RCLCPP_DEBUG(this->get_logger(), "Fused velocity = %.4f m/s", velocityFused);
            velocityPub->vertical_velocity_publisher(velocityFused);
        });

}

/**
 * @brief Fuses the velocities from sensors using a complementary filter
 */
double IMUDepthFusion::fusedVelocity(double velocityIMU, double velocityDepth, double alphaWeight){
    /*
    Using a complementary filter to fuse the velocities from the 
    two sensors
    */
    // fused_vel = a * depthsensor + (1-a)*imusensor
    double fusedVelocity = alphaWeight * velocityDepth +  \
                 (1 - alphaWeight) * velocityIMU;

    return fusedVelocity;
}