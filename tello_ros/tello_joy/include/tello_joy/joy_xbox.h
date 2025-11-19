#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"


class JoyXbox : public rclcpp::Node {

public:
    JoyXbox();

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

private:

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoffPublisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr landPublisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stopPublisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergencyPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motorPublisher;

    std::vector<int> previousButtonPresses;
    bool motorOn;
};
