#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), depth(2.0), imu_acceleration(5.0)
    {
        depth_publisher_ = this->create_publisher<std_msgs::msg::Float32>("depth", 10);
        depth_timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::depth_callback, this));

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        imu_timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::imu_callback, this));
    }

  private:
    void imu_callback()
    {
      auto message = sensor_msgs::msg::Imu();
      message.linear_acceleration.z = imu_acceleration;
      RCLCPP_INFO(this->get_logger(), ("Publishing:  " + std::to_string(imu_acceleration)).c_str());
      imu_publisher_->publish(message);
      imu_acceleration +=0.04;
    }

    void depth_callback()
    {
      auto message = std_msgs::msg::Float32();
      message.data = depth;
      RCLCPP_INFO(this->get_logger(), ("Publishing: "+ std::to_string(depth)).c_str());
      depth_publisher_->publish(message);
      depth += 0.2;
    }


    rclcpp::TimerBase::SharedPtr depth_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    double depth;
    double imu_acceleration;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}