#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <vector>
#include "ctello.h"
#include "tello_driver/state_publisher.h"
#include "tello_driver/camera_publisher.h"
#include "tello_driver/movement_controller.h"
#include <thread>

class TelloDriver : public rclcpp::Node {

public:
    TelloDriver();
    void run();

private:

    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<ctello::Tello> drone;
    bool droneInitialised;

    bool publishCamera;
    std::shared_ptr<CameraPublisher> cameraPublisher;
    std::shared_ptr<StatePublisher> statePublisher;
    std::shared_ptr<MovementController> movementController;

    std::string tfMap;
    std::string tfDrone;

    void initParameters();
};
