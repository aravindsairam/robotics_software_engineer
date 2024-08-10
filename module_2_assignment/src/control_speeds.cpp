#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ControlSpeedPublisher : public rclcpp::Node
{
  public:
    ControlSpeedPublisher()
    : Node("controspeed_publisher")
    {  
      this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
      this->declare_parameter<float>("linear_x", 0.0);
      this->declare_parameter<float>("angular_z", 0.0);

      std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ControlSpeedPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      linear_velocity_ = this->get_parameter("linear_x").as_double();
      angular_velocity_ = this->get_parameter("angular_z").as_double();
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = linear_velocity_;
      message.angular.z = angular_velocity_;
      RCLCPP_INFO(this->get_logger(), "Driving robot with linear velocity: '%f' and angular velocity: '%f'", message.linear.x, message.angular.z);
      publisher_->publish(message);
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_velocity_, angular_velocity_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlSpeedPublisher>());
  rclcpp::shutdown();
  return 0;
}