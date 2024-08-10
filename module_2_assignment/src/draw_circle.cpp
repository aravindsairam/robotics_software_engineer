#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CircelPublisher : public rclcpp::Node
{
  public:
    CircelPublisher(double radius)
    : Node("circel_publisher"), radius_(radius)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&CircelPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      // angular velocity = linear velocity / radius
      linear_velocity_ = 1.0;
      float angular_velocity = linear_velocity_ / radius_;   
      message.linear.x = linear_velocity_;
      message.angular.z = angular_velocity;
      RCLCPP_INFO(this->get_logger(), "Driving robot with linear velocity: '%f' and angular velocity: '%f'", message.linear.x, message.angular.z);
      publisher_->publish(message);
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double radius_;
    double linear_velocity_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // get user input for radius
  double radius = std::stod(argv[1]);
  rclcpp::spin(std::make_shared<CircelPublisher>(radius));
  rclcpp::shutdown();
  return 0;
}