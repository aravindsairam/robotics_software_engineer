#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SpiralPublisher : public rclcpp::Node
{
  public:
    SpiralPublisher()
    : Node("spiral_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&SpiralPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      linear_velocity_ += increment_;
      float radius = 0.5;   
      message.linear.x = linear_velocity_;
      message.angular.z = radius;
      RCLCPP_INFO(this->get_logger(), "Driving robot with linear velocity: '%f' and angular velocity: '%f'", message.linear.x, message.angular.z);
      publisher_->publish(message);
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double increment_ = 0.01;
    double linear_velocity_  = 0.01;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpiralPublisher>());
  rclcpp::shutdown();
  return 0;
}