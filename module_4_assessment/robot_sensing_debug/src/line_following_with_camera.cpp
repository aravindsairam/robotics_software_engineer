// Purpose:
// - Receives Image messages, applies Canny edge_ detection, calculates the error_ between _midPoint and center,
//   and publishes Twist messages to control robot motion.
// - Demonstrates image processing and motion control integration in a ROS2 environment.
// Author: Robotisim

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraSubscriber : public rclcpp::Node {
    public:
    CameraSubscriber()
    : Node("camera_subscriber"), _angularVel(0.3) {
        
        this->declare_parameter<int>("lower_threshold", 200);
        this->declare_parameter<int>("upper_threshold", 250);
        
        publisher_ = this->createpublisher_<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->createsubscription_<sensor_msgs::msg::Image>(
                        "/camera/image_raw", 10,
                        std::bind(&CameraSubscriber::cameraCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\n------ Node Started -----\n");
    }

    private:
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr cameraMsg) {
        getProcessedImage(cameraMsg);
        getMidPoints();
        adjustRobotMotion();
        visualize();
    }

    void getProcessedImage(const sensor_msgs::msg::Image::SharedPtr cameraMsg) {

        cv_bridge::CvImagePtr cvPtr_;
        cvPtr_ = cv_bridge::toCvCopy(cameraMsg, "bgr8");
        
        cv::Mat grayImage_, cannyImage_;
        cv::cvtColor(cvPtr_->image, grayImage_, cv::COLOR_BGR2GRAY);

        int upperThreshold = this->get_parameter("upper_threshold").as_int();
        int lowerThreshold = this->get_parameter("lower_threshold").as_int();
        cv::Canny(grayImage_, cannyImage_, lowerThreshold, upperThreshold);

        // Process Canny image to find the line's _midPoint
        int row_ = 150, column_ = 0;
        _roi = cannyImage_(cv::Range(row_, row_ + 240), cv::Range(column_, column_ + 640));
    
    }

    void getMidPoints() {
        std::vector<int> edge_;
        for (int i = 0; i < 640; ++i) {
            if (_roi.at<uchar>(160, i) == 255) {
                edge_.push_back(i);
            }
        }

        if (!edge_.empty()) {
        int midArea_ = edge_.back() - edge_.front();
        int _midPoint = edge_.front() + midArea_ / 2;
        int _robotMidPoint = 640 / 2;
        }
    }

    void adjustRobotMotion() {
        // Calculate error_ and adjust robot's direction
        double error_ = _robotMidPoint - _midPoint;
        _velocityMsg.linear.x = 0.1;
        if (error_ < 0) {
            _velocityMsg.angular.z = -_angularVel;
        } else {
            _velocityMsg.angular.z = _angularVel;
        }
        else
        {
            _velocityMsg.linear.x = 0.0;
            _velocityMsg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Stop moving");
        }

        publisher_->publish(_velocityMsg);
    }

    void visualize() {
        // Visualization
        cv::circle(_roi, cv::Point(_midPoint, 160), 2, cv::Scalar(255, 255, 255), -1);
        cv::circle(_roi, cv::Point(_robotMidPoint, 160), 5, cv::Scalar(255, 255, 255), -1);
        cv::imshow("Image", _roi);
        cv::waitKey(1);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    double _angularVel;
    cv::Mat _roi;
    int _midPoint, _robotMidPoint;
    geometry_msgs::msg::Twist __velocityMsg;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}
