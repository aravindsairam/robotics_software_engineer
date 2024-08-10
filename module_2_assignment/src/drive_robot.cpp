#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DriveRobot : public rclcpp::Node
{
  public:
    DriveRobot()
    : Node("drive_robot")
    {
      // parameter for cmd_vel topic, default is /turtle1/cmd_vel
      this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
      std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&DriveRobot::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      // move forward for 5 counts, then move backward for 5 counts
      message.linear.x = forward_ ? 0.5 : -0.5;

      publisher_->publish(message);
      if (++counter_ >= 5) {
            forward_ = !forward_;
            counter_ = 0;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    bool forward_ = true;
    int counter_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveRobot>());
  rclcpp::shutdown();
  return 0;
}