/* BSD 3-Clause License
* 
* Copyright (c) 2020, Multi-robot Systems (MRS) group at Czech Technical University in Prague
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* Source: https://github.com/tiiuae/navigation
*/

#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <deque>
#include <future>
#include <mutex>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <tello_driver/astar_planner.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tello_msgs/msg/navigation_diagnostics.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "move_base_msgs/action/move_base.hpp"
#include "tf2_ros/transform_listener.h"
#include "tello_driver/movement_state.h"

enum status_t
{
    IDLE = 0,
    PLANNING,
    COMMANDING,
    MOVING
};

enum waypoint_status_t
{
    EMPTY = 0,
    ONGOING,
    REACHED,
    UNREACHABLE
};

const std::string STATUS_STRING[] = {"IDLE", "PLANNING", "COMMANDING", "MOVING"};
const std::string WAYPOINT_STATUS_STRING[] = {"EMPTY", "ONGOING", "REACHED", "UNREACHABLE"};

/* class Navigation //{ */
class Navigation {
public:
    Navigation(rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr navigationCallbackGroup);

    void setUavPos(float x, float y);
    void setDroneStatus(MovementState state, bool flying, bool hovering);
    void hover();
    void setGoal(std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle);
    void setWaypoints(nav_msgs::msg::Path::SharedPtr path);

private:
    rclcpp::Node *node_;
    rclcpp::TimerBase::SharedPtr executionTimer;
    rclcpp::CallbackGroup::SharedPtr octomapCallbackGroup;

    // internal variables
    std::string parent_frame_;
    int         replanning_counter_ = 0;

    MovementState movementState_;
    bool flying_  = false;
    bool hovering_ = false;

    bool goal_reached_    = false;
    bool hover_requested_ = false;

    Eigen::Vector4d                  internal_uav_pos_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle_;
    Eigen::Vector4d                  current_goal_;
    Eigen::Vector4d                  last_goal_;
    std::mutex                       planning_tree_mutex_;
    std::mutex                       control_diagnostics_mutex_;
    std::mutex                       goal_mutex;
    std::shared_ptr<octomap::ColorOcTree> planning_tree_;
    status_t                         status_ = IDLE;
    waypoint_status_t                waypoint_status_ = EMPTY;

    std::vector<Eigen::Vector4d> waypoint_out_buffer_;
    size_t                       current_waypoint_id_;

    // params
    double euclidean_distance_cutoff_;
    double safe_obstacle_distance_;
    double navigation_tolerance_;
    bool   unknown_is_occupied_;
    double min_altitude_;
    double max_altitude_;
    double max_goal_distance_;
    double distance_penalty_;
    double greedy_penalty_;
    double planning_tree_resolution_;
    double max_waypoint_distance_;
    double max_yaw_step_;
    double planning_timeout_;
    int    replanning_limit_;
    double replanning_distance_;
    bool   auto_execution_;

    int   diagnostics_received_counter_ = 0;

    // Transform listener
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    // Other subscribers
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr  octomap_subscriber_;

    // moveTo publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr moveto_publisher_;

    // diagnostic publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                    path_publisher_;
    rclcpp::Publisher<tello_msgs::msg::NavigationDiagnostics>::SharedPtr diagnostics_publisher_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr             planningTreePublisher_;

    // subscriber callbacks
    void octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg);

    std::vector<Eigen::Vector4d> resamplePath(const std::vector<octomap::point3d> &waypoints, const double start_yaw, const double end_yaw);

    std::shared_ptr<octomap::ColorOcTree> createPlanningTree(std::shared_ptr<octomap::ColorOcTree> tree, double resolution);

    void publishDiagnostics();
    void publishPath(std::vector<Eigen::Vector4d> waypoints);

    void navigationRoutine(void);
};
//}
