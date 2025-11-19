#include "tello_driver/state_publisher.h"
#include "tello_driver/state.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>

StatePublisher::StatePublisher(std::shared_ptr<ctello::Tello> drone) : Node("state_publisher") {
    this->drone = drone;

    barometerPublisher.init(this, "baro", 1);
    batteryPublisher.init(this, "battery", 1);
    droneStatePublisher.init(this, "drone_state", 1);
    heightPublisher.init(this, "height", 1);
    imuPublisher.init(this, "imu", 1);
    temperatureHighPublisher.init(this, "temperature", 1);
    temperatureLowPublisher.init(this, "temperature_low", 1);
    timePublisher.init(this, "time", 1);
    tofPublisher.init(this, "tof", 1);

    executionTimer =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / (10.0*2)), std::bind(&StatePublisher::update, this));
}

void StatePublisher::update() {
    std::optional<std::string> state = drone->GetState();
    rclcpp::Time time = this->now();

    // Call all state publishers
    if (state.has_value()) {
        // Parse drone state
        std::string stateString = state.value();
        std::map<std::string, double> state;
        parseStateString(stateString, state);

        // Call all state publishers
        barometerPublisher.publish(time, state);
        batteryPublisher.publish(time, state);
        droneStatePublisher.publish(time, state);
        heightPublisher.publish(time, state);
        imuPublisher.publish(time, state);
        temperatureHighPublisher.publish(time, state);
        temperatureLowPublisher.publish(time, state);
        timePublisher.publish(time, state);
        tofPublisher.publish(time, state);

        lastStateUpdateTime = time;
    } else {
        if (this->now().seconds() - lastStateUpdateTime.seconds() > 5.0f) {
            RCLCPP_INFO(this->get_logger(), "No state update received in the last five seconds");
            lastStateUpdateTime = this->now();
        }
    }
}

void BarometerPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    tello_msgs::msg::BarometerStamped msg;
    msg.header.stamp = time;

    msg.baro = state[BAROMETER];

    publisher->publish(msg);
}

void BatteryPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = time;

    msg.voltage = 3.8; // Setting to nominal voltage
    msg.temperature = nan("");
    msg.current = nan("");
    msg.charge = nan("");
    msg.capacity = nan("");
    msg.design_capacity = 1.1;
    msg.percentage = state[BATTERY];
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING; // Assuming that the drone can't be charged while being used
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    // cell_voltage and cell_temperature are not set because they are unknown
    msg.location = "battery_slot";
    msg.serial_number = "GB1-1100mAh-3.8V";

    static int consoleCounter = 0;
    if (consoleCounter % 10 == 0 && state[BATTERY] < 20.0) {
        RCLCPP_INFO(node->get_logger(), "[Battery Warning] Less than 20% battery left");
    }
    consoleCounter++;

    publisher->publish(msg);
}

void DroneStatePublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    tello_msgs::msg::DroneStateStamped msg;
    msg.header.stamp = time;

    msg.pitch = state[PITCH];
    msg.roll = state[ROLL];
    msg.yaw = state[YAW];
    msg.vgx = state[VELOCITY_X];
    msg.vgy = state[VELOCITY_Y];
    msg.vgz = state[VELOCITY_Z];
    msg.templ = state[TEMP_LOW];
    msg.temph = state[TEMP_HIGH];
    msg.tof = state[TOF];
    msg.h = state[HEIGHT];
    msg.bat = state[BATTERY];
    msg.baro = state[BAROMETER];
    msg.time = state[TIME];
    msg.agx = state[ACCELERATION_X];
    msg.agy = state[ACCELERATION_Y];
    msg.agz = state[ACCELERATION_Z];

    publisher->publish(msg);
}

void HeightPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    tello_msgs::msg::HeightStamped msg;
    msg.header.stamp = time;

    msg.height = state[HEIGHT];

    publisher->publish(msg);
}

void ImuPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = time;
    
    double roll_rad = state[ROLL] * M_PI / 180.0;
    double pitch_rad = state[PITCH] * M_PI/ 180.0;
    double yaw_rad = state[YAW] * M_PI/ 180.0;

    tf2::Quaternion tf_quaternion, tf_quaternion_corrected;
    tf_quaternion.setRPY(roll_rad, pitch_rad, yaw_rad);
    // The rotation needs to be transformed into the correct coordinate frame
    // Drone:
    //   x - forward? (See velocity x note down below)
    //   y - right
    //   z - down
    // Ros:
    //   x - forward
    //   y - left
    //   z - up
    tf2::Quaternion tf_correction;
    // Needs to be rotated by 180° around the x-axis
    tf_correction.setRotation(tf2::Vector3(1, 0, 0), M_PI);
    tf_quaternion_corrected = tf_correction * tf_quaternion;
    msg.orientation = tf2::toMsg(tf_quaternion_corrected);
    msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    geometry_msgs::msg::Vector3 velocity;
    // Using velocity vector for linear velocity instead of angular velocity as this is what we get from the drone
    velocity.x = state[VELOCITY_X] / 10 * -1; // Why is the inversion necessary?
    velocity.y = state[VELOCITY_Y] / 10;
    velocity.z = state[VELOCITY_Z] / 10;
    msg.angular_velocity = velocity;
    // Setting first element to -1 to indicate that there are no velocity estimates
    msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    // create a rotation matrix using roll, pitch and yaw
    tf2::Matrix3x3 rot_matrix;
    rot_matrix.setRotation(tf_quaternion);

    // define the gravity vector in the inertial frame
    tf2::Vector3 g_i(0, 0, -9.81);

    // multiply the rotation matrix by the gravity vector to get the gravity vector in the body frame
    tf2::Vector3 g_b = rot_matrix.inverse() * g_i;

    tf2::Vector3 acceleration_measured(state[ACCELERATION_X],
                                       state[ACCELERATION_Y],
                                       state[ACCELERATION_Z]);
    acceleration_measured = (acceleration_measured * 9.81) / 1000;
    // Calculate relative acceleration and divide by 100 to convert cm/^2 to m/s^2
    tf2::Vector3 acceleration_relative = (acceleration_measured - g_b)/* / 100*/;

    if (acceleration_relative.x() < 0.2f && acceleration_relative.x() > -0.2f) acceleration_relative.setX(0.0);
    if (acceleration_relative.y() < 0.2f && acceleration_relative.y() > -0.2f) acceleration_relative.setY(0.0);
    if (acceleration_relative.z() < 0.2f && acceleration_relative.z() > -0.2f) acceleration_relative.setZ(0.0);

    geometry_msgs::msg::Vector3 acceleration;
    acceleration.x = acceleration_relative.x();
    acceleration.y = acceleration_relative.y();
    acceleration.z = acceleration_relative.z();
    msg.linear_acceleration = acceleration;
    msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    publisher->publish(msg);
}

void TemperatureLowPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = time;
    msg.header.frame_id = ""; // TODO: Set to location of temperature sensor
    msg.temperature = state[TEMP_LOW];
    publisher->publish(msg);
}

void TemperatureHighPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = time;
    msg.header.frame_id = ""; // TODO: Set to location of temperature sensor
    msg.temperature = state[TEMP_HIGH];
    publisher->publish(msg);
}

void TimePublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    sensor_msgs::msg::TimeReference msg;
    msg.header.stamp = time;
    msg.time_ref.sec = (int) state[TIME];
    msg.time_ref.nanosec = (int) ((state[TIME] - msg.time_ref.sec) * 1000000000); // TODO: Always zero?
    msg.source = "drone_motor";

    publisher->publish(msg);
}

void ToFPublisher::publish(builtin_interfaces::msg::Time time, std::map<std::string, double> state) {
    sensor_msgs::msg::Range msg;
    msg.header.stamp = time;
    msg.header.frame_id = ""; // TODO: Set to location of range sensor

    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 0.0872665; // TODO: unknown/estimation (is 3d infrared module), current value: 5°
    msg.min_range = 0.3; // based on vision positioning system (actual values lower than 0.3 are always 0.1)
    msg.max_range = 10.0; // based on vision positioning system
    msg.range = state[TOF] / 100;

    if (msg.range >= msg.min_range && msg.range <= msg.max_range) {
        publisher->publish(msg);
    }

}
