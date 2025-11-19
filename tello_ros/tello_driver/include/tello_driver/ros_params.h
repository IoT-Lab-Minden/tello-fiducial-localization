#pragma once

#include "rclcpp/rclcpp.hpp"

#define PARAM_PUBLISH_CAMERA "camera.enabled"
#define PARAM_FPS "camera.fps"
#define PARAM_BITRATE "camera.bitrate"
#define PARAM_RESOLUTION "camera.resolution"
#define PARAM_DOWNVISION "camera.downvision"

#define PARAM_MOVEMENT_SPEED "movement.speed"
#define PARAM_MAX_HEIGHT "movement.max_height"

#define PARAM_AUTO_COOLING "auto_cooling.enabled"
#define PARAM_COOLING_START_TEMP "auto_cooling.start_temperature"

#define PARAM_PLANNING_UPDATE_RATE "planning.main_update_rate"
#define PARAM_PLANNING_AUTO_EXECTION "planning.auto_execution"

template <class T>
bool parse_param(rclcpp::Node * node, const std::string &param_name, T &param_dest) {
    node->declare_parameter(param_name);
    if (!node->get_parameter(param_name, param_dest)) {
        RCLCPP_ERROR(node->get_logger(), "Could not load param '%s'", param_name.c_str());
        return false;
    } else {
        RCLCPP_INFO_STREAM(node->get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
    }
    return true;
}
