#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ctello.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
//#include "image_transport/image_transport.hpp"
//#include "image_transport/camera_publisher.hpp"

class CameraPublisher : public rclcpp::Node {

public:

    CameraPublisher(std::shared_ptr<ctello::Tello> drone);

private:
    std::shared_ptr<ctello::Tello> drone;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;

    //std::shared_ptr<image_transport::ImageTransport> imageTransport;
    //std::shared_ptr<image_transport::CameraPublisher> cameraPublisher;

    rclcpp::TimerBase::SharedPtr executionTimer;
    rclcpp::Time lastImageUpdateTime;
    cv::VideoCapture capture;

    void update();
};
