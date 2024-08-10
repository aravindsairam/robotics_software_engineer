#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LogspiralPublisher : public rclcpp::Node
{
  public:
    LogspiralPublisher()
    : Node("logspiral_publisher"), theta_(0.0), a_(0.2), b_(0.2), z_(0.5)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&LogspiralPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();

      // r(t) = a * exp(b * theta)

      // linear velocity = a * b * exp(b * theta) * z where z is a constant angular velocity
      // a = constant size of the spiral
      // b = constant tightness of the spiral
      // theta = angle of the spiral

      double x = a_ * b_ * std::exp(b_ * theta_) * z_;
      message.linear.x = x;
      message.angular.z = z_;
      RCLCPP_INFO(this->get_logger(), "Driving robot with linear velocity: '%f' and angular velocity: '%f'", message.linear.x, message.angular.z);
      publisher_->publish(message);
      theta_ += z_ * 0.5;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double theta_;
    double a_, b_, z_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LogspiralPublisher>());
  rclcpp::shutdown();
  return 0;
}