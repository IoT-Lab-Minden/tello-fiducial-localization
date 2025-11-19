#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tello_msgs/msg/flight_state.hpp"
#include "tello_msgs/msg/drone_state_stamped.hpp"
#include "move_base_msgs/action/move_base.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ctello.h"
#include "tello_driver/navigation.h"
#include <thread>
#include "tello_driver/movement_state.h"

class MovementController : public rclcpp::Node{

public:
    MovementController(std::shared_ptr<ctello::Tello> drone);

private:
    std::shared_ptr<ctello::Tello> drone;

    rclcpp::TimerBase::SharedPtr idleTimer;
    rclcpp::Time lastCmdTime;

    rclcpp::CallbackGroup::SharedPtr emergencyCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr navigationCallbackGroup;

    std::shared_ptr<Navigation> navigation;

    // Movement subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr movePathSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr moveToSubscriber;
    // Currently using internal uav position because the tracking is not good enough
    Eigen::Vector3d internalUavPosition;

    // Drone state subscriber
    rclcpp::Subscription<tello_msgs::msg::DroneStateStamped>::SharedPtr droneStateSubscriber;

    // Command subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr landSubscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stopSubscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergencySubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motorSubscriber;

    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr tempSubscriber;

    rclcpp_action::Server<move_base_msgs::action::MoveBase>::SharedPtr moveToActionServer;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> currentGoal;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    rclcpp::Publisher<tello_msgs::msg::FlightState>::SharedPtr flightStatePublisher;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Time lastImuTime;
    tf2::Vector3 position;

    MovementState movementState;
    bool flying;
    bool hovering;

    float height;
    int currentMotorRunningTime;
    bool motorOn;

    builtin_interfaces::msg::Time takeoffTime;
    builtin_interfaces::msg::Time lastFlyingTime;
    builtin_interfaces::msg::Time lastMovingTime;
    builtin_interfaces::msg::Time currentTime;

    float movementSpeed; // cm/s
    float maxHeight; // in cm
    double planningUpdateRate;
    bool autoCoolingEnabled;
    double coolingStartTemp;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void movePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void moveToCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void droneStateCallback(const tello_msgs::msg::DroneStateStamped::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void takeoffCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void landCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void stopCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void emergencyCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void motorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void tempCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle);
    void handleAccepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle);

    void takeoff();
    void land(bool force);

    void idleRoutine(void);

    float getTimeDifferenceNs(builtin_interfaces::msg::Time previous, builtin_interfaces::msg::Time next);
};
