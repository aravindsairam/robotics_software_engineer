#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

enum class Direction
{
    MOVING_FORWARD,
    MOVING_LEFT,
    MOVING_RIGHT,
    STOP_MOVING
};


class WallFollowing : public rclcpp::Node
{
    public:
        WallFollowing() : Node("wall_following")
        {
            // Create a subscriber for lidar data
            lidarSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
                         std::bind(&WallFollowing::lidar_callback, this, std::placeholders::_1));

            cmdVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }

    private:
        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            anayseLidarData(msg);
            determineState();
            publishCmdVel();
        }
        // get lidar data for the front, left and right side of the robot
        void anayseLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            // get 0-20 and 340-360 degrees for the front side of the robot
            float _minFront_1 = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 20);
            float _minFront_2 = *std::min_element(msg->ranges.begin()+340, msg->ranges.begin() + 360);
            // get the minimum value of the front side of the robot
            _minFront = std::min(_minFront_1, _minFront_2);
            // get 70-90 degrees for the left side of the robot
            _minLeft = *std::min_element(msg->ranges.begin() + 70, msg->ranges.begin() + 90);
            // get 260-280 degrees for the right side of the robot
            _minRight = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 280);
            RCLCPP_INFO(this->get_logger(), "Front: %f, Left: %f, Right: %f", _minFront, _minLeft, _minRight);
        }
            
        void determineState()
        {
            switch (_state)
            {
                case Direction::MOVING_FORWARD:
                    if (_minFront > _maxStopDistance && _minLeft > _maxStopDistance && _minRight > _maxStopDistance)
                    {
                        _state = Direction::STOP_MOVING;
                        break;
                    }
                    else
                    {
                        if (_minFront < _obstacleDistance)
                        {
                            if (_minRight > _minLeft)
                            {
                                _state = Direction::MOVING_RIGHT;
                            }
                            else
                            {
                                _state = Direction::MOVING_LEFT;
                            }
                        }
                        else
                        {
                            _state = Direction::MOVING_FORWARD;
                        }
                        break;
                    }
                case Direction::MOVING_LEFT:
                    if (_minFront > _obstacleDistance)
                    {
                        _state = Direction::MOVING_FORWARD;
                    }
                    break;
                case Direction::MOVING_RIGHT:
                    if (_minFront > _obstacleDistance)
                    {
                        _state = Direction::MOVING_FORWARD;
                    }
                    break;
                
            }
        }

        void publishCmdVel()
        {
            switch (_state)
            {
                case Direction::MOVING_FORWARD:
                    _cmdVel.linear.x = _linearVel;
                    _cmdVel.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "moving forward");
                    break;
                case Direction::MOVING_LEFT:
                    _cmdVel.linear.x = 0.0;
                    _cmdVel.angular.z = _angularVel;
                    RCLCPP_INFO(this->get_logger(), "turning left");
                    break;
                case Direction::MOVING_RIGHT:
                    _cmdVel.linear.x = 0.0;
                    _cmdVel.angular.z = -_angularVel;
                    RCLCPP_INFO(this->get_logger(), "turning left");
                    break;
                case Direction::STOP_MOVING:
                    _cmdVel.linear.x = 0.0;
                    _cmdVel.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "stop moving");
                    break;
            }
            cmdVelPub_->publish(_cmdVel);
        }

        geometry_msgs::msg::Twist _cmdVel;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
        Direction _state;
        float _minFront, _minLeft, _minRight;
        float _obstacleDistance = 1.0;
        
        float _maxStopDistance = 3.5;
        float _linearVel = 0.5;
        float _angularVel = 0.25;


        // float _maxFrontObstacleDistance = 0.5;
        // float _minRightObstacleDistance = 0.5;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarSub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowing>());
    rclcpp::shutdown();
    return 0;
}
