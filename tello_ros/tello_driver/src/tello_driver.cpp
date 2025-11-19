#include "tello_driver/tello_driver.h"
#include "tello_driver/ros_params.h"
#include <cstdio>
#include <signal.h>

TelloDriver::TelloDriver() : Node("tello_driver") {

    initParameters();

    RCLCPP_INFO(this->get_logger(), "Finding tello ...");

    // Connect to drone
    drone = std::make_shared<ctello::Tello>();
    if (!(droneInitialised = drone->Bind())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to drone");
    }
    RCLCPP_INFO(this->get_logger(), "Entered SDK mode");

    // Publish some basic information about the drone
    std::string responseSn = drone->SendCommand("sn?");
    RCLCPP_INFO(this->get_logger(), "Serial Number: %s", responseSn.c_str());
    std::string responseSdk = drone->SendCommand("sdk?");
    RCLCPP_INFO(this->get_logger(), "Tello SDK: %s", responseSdk.c_str());
    std::string responseWifi = drone->SendCommand("wifi?");
    RCLCPP_INFO(this->get_logger(), "Wi-Fi Signal: %s", responseWifi.c_str());
    std::string responseBat = drone->SendCommand("battery?");
    RCLCPP_INFO(this->get_logger(), "Battery: %s%", responseBat.c_str());

    statePublisher = std::make_shared<StatePublisher>(drone);
    executor.add_node(statePublisher);
    movementController = std::make_shared<MovementController>(drone);
    executor.add_node(movementController);
    if (publishCamera) {
        cameraPublisher = std::make_shared<CameraPublisher>(drone);
        executor.add_node(cameraPublisher);
    }
}

void TelloDriver::initParameters() {

    this->declare_parameter(PARAM_PUBLISH_CAMERA, false);

    publishCamera = this->get_parameter(PARAM_PUBLISH_CAMERA).get_parameter_value().get<bool>();
}

void TelloDriver::run() {
    executor.spin();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TelloDriver> driver = std::make_shared<TelloDriver>();

    driver->run();

    rclcpp::shutdown();
    return 0;
}
