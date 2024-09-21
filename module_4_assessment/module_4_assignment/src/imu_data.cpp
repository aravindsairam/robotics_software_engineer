#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuSubscriber::imu_callback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ImuSubscriber::cmdVelCallback, this, std::placeholders::_1)); 

        previous_time_ = this->get_clock()->now();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - previous_time_).seconds();  // Time difference

        if (dt > 0)
        {   
            RCLCPP_INFO(this->get_logger(), "dt = %.4f", dt);
            
            // since the robot is a mobile robot, we are only interested in the X and Y linear accelerations
            // TurtleBot3 is a differential drive robot, so it can only move in the X direction 
            // Get linear acceleration from IMU message
            acceleration_x_ = msg->linear_acceleration.x;
            acceleration_y_ = msg->linear_acceleration.y;

            // we get angular velocity from the IMU message
            angular_velocity_z_ = msg->angular_velocity.z;

            // Calculate the linear velocity of the robot from the acceleration
            velocity_x_ += acceleration_x_* dt;
            velocity_y_ += acceleration_y_ * dt;

            // calculate the angular acceleration of the from the angular velocity
            angular_acceleration_z_ = (angular_velocity_z_ - prev_angular_velocity_z_) / dt;

            RCLCPP_INFO(this->get_logger(), "Linear Acceleration X, Linear Acceleration Y, Angular Acceleration Z: [%.4f, %.4f, %.4f]", 
                        acceleration_x_, acceleration_y_, angular_acceleration_z_);

            RCLCPP_INFO(this->get_logger(), "Linear Velocity X = %.4f, Linear Velocity Y = %.4f, Angular Velocity Z = %.4f", 
                        velocity_x_, velocity_y_, angular_velocity_z_);

            prev_angular_velocity_z_ = angular_velocity_z_;
        }

        previous_time_ = current_time;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Reset velocity if the robot is commanded to stop
        if (msg->linear.x == 0.0)
        {
            velocity_x_ = 0.0;
        }
        if (msg->linear.y == 0.0)
        {
            velocity_y_ = 0.0;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Time previous_time_;
    double acceleration_x_, acceleration_y_;
    double angular_acceleration_z_;
    double prev_angular_velocity_z_ = 0.0;
    double velocity_x_,velocity_y_, velocity_z_  = 0.0;
    double angular_velocity_z_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}