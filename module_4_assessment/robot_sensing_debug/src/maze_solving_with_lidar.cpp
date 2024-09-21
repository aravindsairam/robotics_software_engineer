#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

enum class RobotState {
    MOVING_STRAIGHT,
    TURNING_LEFT,
    TURNING_RIGHT,
    OUT_OF_MAZE
};

class MazeSolving : public rclcpp::Node {
    public:
    MazeSolving() : Node("maze_solving"), state_(RobotState::MOVING_STRAIGHT) {
    
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&MazeSolving::lidarCallback, this, std::placeholders::_1));
    }

    private:
        void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidarMsg) {
            analyseLidarData(lidarMsg);
            determineRobotState();
            publishVelocity();
        }

        void analyseLidarData(const sensor_msgs::msg::LaserScan::SharedPtr lidarMsg) {
            // use std::min_element instead of std::max_element to get the minimum distance to the obstacle
            _rightObstacle = *std::min_element(lidarMsg->ranges.begin() + 260,
                                                    lidarMsg->ranges.begin() + 280);
            // instead of claculating the minimum distance in in one half of the front lidar scan that is 340 to 360
            // calculate the minimum distance in 2 parts that is 350 to 360 and 0 to 10 and take the minimum of both
            float _frontObstacle_1 = *std::min_element(lidarMsg->ranges.begin() + 350,
                                                    lidarMsg->ranges.begin() + 360);
            float _frontObstacle_2 = *std::min_element(lidarMsg->ranges.begin(),
                                                    lidarMsg->ranges.begin() + 10);     
            _frontObstacle = std::min(_frontObstacle_1, _frontObstacle_2);
            _leftObstacle = *std::min_element(lidarMsg->ranges.begin() + 80,
                                                lidarMsg->ranges.begin() + 100);

            RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f, Left: %f",
                        _frontObstacle, _rightObstacle, _leftObstacle);

        }

        void determineRobotState() {
            // all obstacle distance should be greater than the threshold to go out of maze
            if (_frontObstacle > _frontThreshold && _rightObstacle > _frontThreshold &&
                _leftObstacle > _frontThreshold) {
            state_ = RobotState::OUT_OF_MAZE;
            // _frontThreshold should be greater than _frontObstacle
            } else if (_frontObstacle < _frontThreshold) {
            // if left obstacle is less than right obstacle then there is a wall on the left side
            // so turn robot to right side and vice versa
            state_ = _leftObstacle < _rightObstacle ? RobotState::TURNING_RIGHT
                                                    : RobotState::TURNING_LEFT;
            }
            // add else if condition to move straight if front obstacle is greater than the threshold
            else if (_frontObstacle > _frontThreshold){
            state_ = RobotState::MOVING_STRAIGHT;
            }
        }

        void publishVelocity() {
            // _angularVel variable should be used instead of constants
            // _linearVel variable should be used instead of constants 
            switch (state_) {
            case RobotState::MOVING_STRAIGHT:
            // velocity y is given instead of x
            // moving straight should have 0 angular velocity
            _command.linear.x = _linearVel;
            _command.angular.z = 0.0f;
            break;
            case RobotState::TURNING_LEFT:
            // turning left should have 0 linear velocity
            _command.linear.x = 0.0f;
            _command.angular.z = _angularVel;
            break;
            case RobotState::TURNING_RIGHT:
            // turning right should have 0 linear velocity
            _command.linear.x = 0.0f;
            _command.angular.z = -_angularVel;
            break;
            case RobotState::OUT_OF_MAZE:
            // out of maze should have 0 linear and angular velocity
            _command.linear.x = 0.0f;
            _command.angular.z = 0.0f;
            break;
            }

            publisher_->publish(_command);
        }

    // reduce the threshold to 1.0f as it detects front obstacles from a long distance
    // reduce velocities to safe and smooth turn and not to overshoot the turn
    float _frontThreshold = 1.5f;
    float _angularVel = 0.5f;
    float _linearVel = 1.0f;
    float _rightObstacle, _frontObstacle, _leftObstacle;
    RobotState state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    // make the _command variable a member variable
    geometry_msgs::msg::Twist _command;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolving>());
    rclcpp::shutdown();
    return 0;
}

