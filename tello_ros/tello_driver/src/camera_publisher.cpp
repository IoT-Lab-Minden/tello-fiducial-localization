#include "tello_driver/camera_publisher.h"
#include "tello_driver/ros_params.h"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>

const char* const DRONE_STREAM_URL{"udp://0.0.0.0:11111"};

CameraPublisher::CameraPublisher(std::shared_ptr<ctello::Tello> drone) : Node("tello_camera") {
    this->drone = drone;
    
    // Declare and get ROS parameters
    std::string fps, resolution;
    int bitrate;
    bool downvision;
    parse_param(this, PARAM_FPS, fps); // Available options: high, middle, low
    parse_param(this, PARAM_BITRATE, bitrate); // Available options: 0-5, (auto, 1Mbps, 2Mbps, 3Mbps, 4Mbps, 5Mbps)
    parse_param(this, PARAM_RESOLUTION, resolution); // Available options: high(720p), low(480p)
    parse_param(this, PARAM_DOWNVISION, downvision);

    // Create camera publishers
    cameraInfoPublisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 5);
    imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 5);

    /*std::shared_ptr<rclcpp::Node> nodePtr = std::shared_ptr<rclcpp::Node>(this);
    imageTransport = std::make_shared<image_transport::ImageTransport>(nodePtr);
    image_transport::CameraPublisher pub = imageTransport->advertiseCamera("/camera/image_raw", queueSize);
    cameraPublisher = std::shared_ptr<image_transport::CameraPublisher>(&pub);*/

    // Set video parameters on the drone
    drone->SendCommand("setfps " + fps);
    drone->SendCommand("setbitrate " + std::to_string(bitrate));
    drone->SendCommand("setresolution " + resolution);
    drone->SendCommand(std::string("downvision ") + (downvision ? "1" : "0"));

    // Activate video streaming
    std::string response = drone->SendCommand("streamon");
    if (response.compare("ok") == 0) {
        // Start video capture
        capture = cv::VideoCapture{DRONE_STREAM_URL, cv::CAP_FFMPEG};
        RCLCPP_INFO(this->get_logger(), "Started camera stream");
        
        // Using double the fps rate to increase responsiveness and prevent
        // high lag
        int fpsValue = ((fps == "low") ? 5 : fps == "middle" ? 15 : 30) * 2;
        // timer
        executionTimer =
            this->create_wall_timer(std::chrono::duration<double>(1.0 / fpsValue), std::bind(&CameraPublisher::update, this));
    }
}

void CameraPublisher::update() {
    cv::Mat frame;
    capture >> frame;

    // If a new video frame was received
    if (!frame.empty()) {
        // Create image msg
        sensor_msgs::msg::Image::SharedPtr imageMsg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                .toImageMsg();

        rclcpp::Time time = this->now();
        imageMsg->header.stamp = time;
        imageMsg->header.frame_id = "camera"; 

        // Create camera info msg
        std::shared_ptr<sensor_msgs::msg::CameraInfo> infoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        infoMsg->header.stamp = time;
        infoMsg->header.frame_id = "camera"; 
        infoMsg->height = imageMsg->height;
        infoMsg->width = imageMsg->width;
        infoMsg->d = {-0.03366830679825901, 0.11374215407583504, 0.0017597037780148648, -0.0016602920515487545, 0.0};
        infoMsg->k = {922.0093085068482, 0.0, 473.626322114718, 0.0, 917.5657673473347, 382.37862261737814, 0.0, 0.0, 1.0};
        infoMsg->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        infoMsg->p = {926.5885620117188, 0.0, 471.94181410584133, 0.0, 0.0, 922.6842651367188, 382.57961252178575, 0.0, 0.0, 0.0, 1.0, 0.0};

        // Publish messages
        // TODO: Replace manual publishing with CameraPublisher
        // (Currently crashes when trying to publish an image using ImageTransport)
        imagePublisher->publish(*imageMsg.get());
        cameraInfoPublisher->publish(*infoMsg.get());

        /*std::cout << "Sending frame" << std::endl;
        try {
            cameraPublisher->publish(*imageMsg.get(), *infoMsg.get());
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
        std::cout << "Sending finished" << std::endl;*/

        lastImageUpdateTime = this->now();
    } else {
        if (this->now().seconds() - lastImageUpdateTime.seconds() > 5.0f) {
            RCLCPP_INFO(this->get_logger(), "No camera image received in the last five seconds");
            lastImageUpdateTime = this->now();
        }
    }
}
