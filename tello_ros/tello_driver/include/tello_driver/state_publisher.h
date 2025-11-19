#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tello_msgs/msg/height_stamped.hpp"
#include "tello_msgs/msg/barometer_stamped.hpp"
#include "tello_msgs/msg/drone_state_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <map>
#include <string>
#include "ctello.h"

template<class T>
class StatePublisherBase {
public:
    void init(rclcpp::Node* node, std::string topicName, int queueSize);
    virtual void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) = 0;

protected:
    typename rclcpp::Publisher<T>::SharedPtr publisher;
    rclcpp::Node* node;
};

template<class T>
void StatePublisherBase<T>::init(rclcpp::Node* node, std::string topicName, int queueSize) {
    publisher = node->create_publisher<T>(topicName, queueSize);
    this->node = node;
}

class BarometerPublisher: public StatePublisherBase<tello_msgs::msg::BarometerStamped> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class BatteryPublisher: public StatePublisherBase<sensor_msgs::msg::BatteryState> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class DroneStatePublisher: public StatePublisherBase<tello_msgs::msg::DroneStateStamped> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class HeightPublisher: public StatePublisherBase<tello_msgs::msg::HeightStamped> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class ImuPublisher: public StatePublisherBase<sensor_msgs::msg::Imu> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class TemperatureLowPublisher: public StatePublisherBase<sensor_msgs::msg::Temperature> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class TemperatureHighPublisher: public StatePublisherBase<sensor_msgs::msg::Temperature> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class TimePublisher: public StatePublisherBase<sensor_msgs::msg::TimeReference> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class ToFPublisher: public StatePublisherBase<sensor_msgs::msg::Range> {
public:
    void publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state);
};

class StatePublisher : public rclcpp::Node {
public:
    StatePublisher(std::shared_ptr<ctello::Tello> drone);

private:
    std::shared_ptr<ctello::Tello> drone;

    BarometerPublisher barometerPublisher;
    BatteryPublisher batteryPublisher;
    DroneStatePublisher droneStatePublisher;
    HeightPublisher heightPublisher;
    ImuPublisher imuPublisher;
    TemperatureHighPublisher temperatureHighPublisher;
    TemperatureLowPublisher temperatureLowPublisher;
    TimePublisher timePublisher;
    ToFPublisher tofPublisher;

    rclcpp::TimerBase::SharedPtr executionTimer;
    rclcpp::Time lastStateUpdateTime;

    void update();
};
