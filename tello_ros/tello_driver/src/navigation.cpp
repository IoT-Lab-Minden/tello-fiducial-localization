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

#include "tello_driver/navigation.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tello_driver/ros_params.h"

using namespace std::placeholders;

double getYaw(const geometry_msgs::msg::Quaternion &q) {
    return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

octomap::point3d toPoint3d(const Eigen::Vector4d &vec) {
    octomap::point3d p;
    p.x() = vec.x();
    p.y() = vec.y();
    p.z() = vec.z();
    return p;
}

/* constructor //{ */
Navigation::Navigation(rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr navigationCallbackGroup) {
    node_ = node;

    /* parse params from config file //{ */
    bool loaded_successfully = true;
    loaded_successfully &= parse_param(node_, "planning.euclidean_distance_cutoff", euclidean_distance_cutoff_);
    loaded_successfully &= parse_param(node_, "planning.safe_obstacle_distance", safe_obstacle_distance_);
    loaded_successfully &= parse_param(node_, "planning.unknown_is_occupied", unknown_is_occupied_);
    loaded_successfully &= parse_param(node_, "planning.navigation_tolerance", navigation_tolerance_);
    loaded_successfully &= parse_param(node_, "planning.min_altitude", min_altitude_);
    loaded_successfully &= parse_param(node_, "planning.max_altitude", max_altitude_);
    loaded_successfully &= parse_param(node_, "planning.max_goal_distance", max_goal_distance_);
    loaded_successfully &= parse_param(node_, "planning.distance_penalty", distance_penalty_);
    loaded_successfully &= parse_param(node_, "planning.greedy_penalty", greedy_penalty_);
    loaded_successfully &= parse_param(node_, "planning.planning_tree_resolution", planning_tree_resolution_);
    loaded_successfully &= parse_param(node_, "planning.max_waypoint_distance", max_waypoint_distance_);
    loaded_successfully &= parse_param(node_, "planning.max_yaw_step", max_yaw_step_);
    loaded_successfully &= parse_param(node_, "planning.planning_timeout", planning_timeout_);
    loaded_successfully &= parse_param(node_, "planning.replanning_limit", replanning_limit_);
    loaded_successfully &= parse_param(node_, "planning.replanning_distance", replanning_distance_);
    loaded_successfully &= parse_param(node_, "planning.auto_execution", auto_execution_);

    if (!loaded_successfully) {
        const std::string str = "Could not load all non-optional navigation parameters. Shutting down.";
        RCLCPP_ERROR(node_->get_logger(), str.c_str());
        rclcpp::shutdown();
        return;
    }

    current_waypoint_id_ = 0;

    rclcpp::SubscriptionOptions navigationOptions;
    navigationOptions.callback_group = navigationCallbackGroup;

    octomapCallbackGroup = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions octomapOptions;
    octomapOptions.callback_group = octomapCallbackGroup;

    // Transform Listener
    tfBuffer = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // Other subscribers
    octomap_subscriber_ = node_->create_subscription<octomap_msgs::msg::Octomap>("map", 1, std::bind(&Navigation::octomapCallback, this, _1), octomapOptions);

    // moveTo publisher
    moveto_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("move_to", 1);

    // diagnostic publishers
    path_publisher_        = node_->create_publisher<nav_msgs::msg::Path>("path", 1);
    diagnostics_publisher_ = node_->create_publisher<tello_msgs::msg::NavigationDiagnostics>("diagnostics", 5);
    planningTreePublisher_ = node_->create_publisher<octomap_msgs::msg::Octomap>("planning_tree", 1);

    if (max_waypoint_distance_ <= 0) {
        max_waypoint_distance_ = replanning_distance_;
    }

    executionTimer =
        node_->create_wall_timer(std::chrono::duration<double>(1.0 / 10), std::bind(&Navigation::navigationRoutine, this), navigationCallbackGroup);
}
//}

/* octomapCallback //{ */
void Navigation::octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg) {
    // Setting planning_tree_mutex_ here already to make sure that the newest octomap is used
    // when a new goal is received
    std::scoped_lock lock(planning_tree_mutex_);

    RCLCPP_INFO(node_->get_logger(), "Getting octomap");
    parent_frame_    = msg->header.frame_id;

    auto time_start_planning_tree = std::chrono::high_resolution_clock::now();
    auto treePtr = octomap_msgs::fullMsgToMap(*msg);

    if (!treePtr) {
        RCLCPP_WARN(node_->get_logger(), "Octomap message is empty!");
    } else {
        std::shared_ptr<octomap::ColorOcTree> octree = std::shared_ptr<octomap::ColorOcTree>(dynamic_cast<octomap::ColorOcTree *>(treePtr));
        std::shared_ptr<octomap::ColorOcTree> planning_tree = createPlanningTree(octree, planning_tree_resolution_);

        planning_tree_ = planning_tree;
        RCLCPP_INFO(node_->get_logger(), "The planning tree took %.2fs to create",
            std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start_planning_tree).count());
    }
}
//}

void Navigation::setUavPos(float x, float y) {
    internal_uav_pos_ = Eigen::Vector4d(x, y, 0, 0);
}

/* setDroneStatus //{ */
void Navigation::setDroneStatus(MovementState state, bool flying, bool hovering) {
    std::scoped_lock lock(control_diagnostics_mutex_);
    movementState_ = state;
    flying_ = flying;
    hovering_ = hovering;
    // TODO: Replace goal_reached_ = msg->mission_finished; // Currently only using bool hovering to determine if goal was reached

    // Using a counter instead of boolean to account for delay
    diagnostics_received_counter_++;
}
//}

/* setGoal //{*/
void Navigation::setGoal(std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle) {
    if (status_ != IDLE) {
        RCLCPP_ERROR(node_->get_logger(), "Goto rejected, vehicle not IDLE");
        return;
    }

    std::scoped_lock lock(goal_mutex);
    RCLCPP_INFO(node_->get_logger(), "Received goal");
    goal_handle_ = goal_handle;
    status_ = PLANNING;
}
//}

/* hover //{ */
void Navigation::hover() {
    if (status_ == IDLE) {
        RCLCPP_WARN(node_->get_logger(), "Hover not necessary, vehicle is IDLE");
        return;
    }

    // TODO: Is it here that it is ensured that the drone does not drift?
    status_ = IDLE;
    waypoint_status_ = EMPTY;
    if (goal_handle_->is_executing()) {
        goal_handle_->canceled(std::make_shared<move_base_msgs::action::MoveBase::Result>());
    }
    goal_handle_ = nullptr;
    RCLCPP_INFO(node_->get_logger(), "Navigation stopped. Hovering");
}
//}

/* setWaypoints //{ */
void Navigation::setWaypoints(nav_msgs::msg::Path::SharedPtr path) {
    if (status_ != IDLE) {
        RCLCPP_ERROR(node_->get_logger(), "Path rejected, vehicle not IDLE");
        return;
    }

    // TODO: Currently assumes that the frame is always map
    std::vector<Eigen::Vector4d> waypoints;
    for (const auto &w : path->poses) {
        Eigen::Vector4d waypoint(
            w.pose.position.x,
            w.pose.position.y,
            w.pose.position.z,
            getYaw(w.pose.orientation)
        );

        waypoints.push_back(waypoint);
    }

    RCLCPP_INFO(node_->get_logger(), "Sending %ld waypoints to the control interface:", waypoints.size());
    for (auto &w : waypoints) {
        RCLCPP_INFO(node_->get_logger(), "        %.2f, %.2f, %.2f, %.2f", w.x(), w.y(), w.z(), w.w());
    }

    waypoint_out_buffer_ = waypoints;
    // The first waypoint should always be the initial position,
    // therefore the first goal is waypoint 1
    current_waypoint_id_ = 1;

    status_ = COMMANDING;
    waypoint_status_ = ONGOING;
    RCLCPP_INFO(node_->get_logger(), "Started path navigation");
}
//}

/* navigationRoutine //{ */
void Navigation::navigationRoutine(void) {

    std::scoped_lock lock(goal_mutex);
    if (auto_execution_) {
        if (!goal_handle_) return;
        if (goal_handle_->is_canceling()){
            goal_handle_->canceled(std::make_shared<move_base_msgs::action::MoveBase::Result>());
            status_ = IDLE;
        }
        if (!goal_handle_->is_executing()) return;
    }

    switch (status_) {

    /* IDLE //{ */
    // Just does nothing (or ensures that there is no drift?)
    case IDLE: {
        replanning_counter_ = 0;
        break;
    }
    //}

    /* PLANNING //{ */
    // Planning stage where the path is planned
    case PLANNING: {

        // Get UAV position
        // TODO: Currently using internal position from movement_controller
        geometry_msgs::msg::TransformStamped tfUavPos;
        try {
            tfUavPos = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(node_->get_logger(), "Failed to get transform from %s to %s: %s",
                "map", "base_link", ex.what());

            goal_handle_->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
            return;
        }
        geometry_msgs::msg::TransformStamped tfOdomPos;
        try {
            tfOdomPos = tfBuffer->lookupTransform("map", "odom", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(node_->get_logger(), "Failed to get transform from %s to %s: %s",
                "map", "odom", ex.what());

            goal_handle_->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
            return;
        }
        /*Eigen::Vector4d uav_pos(tfOdomPos.transform.translation.x + internal_uav_pos_.x(),
                                tfOdomPos.transform.translation.y + internal_uav_pos_.y(),
                                tfUavPos.transform.translation.z,
                                getYaw(tfUavPos.transform.rotation));*/
        Eigen::Vector4d uav_pos(tfUavPos.transform.translation.x,
                                tfUavPos.transform.translation.y,
                                tfUavPos.transform.translation.z,
                                getYaw(tfUavPos.transform.rotation));

        if (!flying_) {
            // Add offset when not flying to ensure
            // that planning is done with takeoff already executed
            uav_pos.z() += 0.7;
        }

        waypoint_out_buffer_.clear();

        if (goal_handle_ == nullptr || !goal_handle_->is_executing()) {
            RCLCPP_WARN(node_->get_logger(), "No navigation goals available. Switching to IDLE");
            status_ = IDLE;
            goal_handle_ = nullptr;
            break;
        }

        if (replanning_counter_ >= replanning_limit_) {
            RCLCPP_ERROR(node_->get_logger(),
                        "No waypoint produced after %d repeated attempts. "
                        "Please provide a new waypoint", replanning_counter_);
            goal_handle_->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
            status_ = IDLE;
            waypoint_status_ = UNREACHABLE;
            goal_handle_ = nullptr;
            break;
        }

        std::pair<std::vector<octomap::point3d>, PlanningResult> waypoints;

        geometry_msgs::msg::Pose poseMsg = goal_handle_->get_goal()->target_pose.pose;
        geometry_msgs::msg::TransformStamped tfgoalPos;

        // Use lookupTransform to automatically convert goal pose to the correct frame
        // TODO: Update to use internal uav pos
        try {
            tfgoalPos = tfBuffer->lookupTransform("map", goal_handle_->get_goal()->target_pose.header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(node_->get_logger(), "Failed to get transform from %s to %s: %s",
                goal_handle_->get_goal()->target_pose.header.frame_id.c_str(), "base_link", ex.what());

            goal_handle_->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
            status_ = IDLE;
            waypoint_status_ = UNREACHABLE;
            goal_handle_ = nullptr;
            break;
        }

        last_goal_    = current_goal_;
        current_goal_ = Eigen::Vector4d(tfgoalPos.transform.translation.x + poseMsg.position.x,
                                        tfgoalPos.transform.translation.y + poseMsg.position.y,
                                        tfgoalPos.transform.translation.z + poseMsg.position.z,
                                        getYaw(poseMsg.orientation));

        RCLCPP_INFO(node_->get_logger(), "Waypoint [%.2f, %.2f, %.2f, %.2f] set as a next goal",
                                        current_goal_[0], current_goal_[1], current_goal_[2], current_goal_[3]);
        RCLCPP_INFO(node_->get_logger(), "Current position [%.2f, %.2f, %.2f, %.2f]", 
                                        uav_pos[0], uav_pos[1], uav_pos[2], uav_pos[3]);

        octomap::point3d planning_start = toPoint3d(uav_pos);
        octomap::point3d planning_goal  = toPoint3d(current_goal_);

        {
            std::scoped_lock lock(planning_tree_mutex_);

            if (planning_tree_ == NULL || planning_tree_->size() < 1) {
                RCLCPP_WARN(node_->get_logger(), "Octomap is NULL or empty! Abort planning and swiching to IDLE");
                goal_handle_->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
                status_ = IDLE;
                waypoint_status_ = EMPTY;
                break;
            }

            AstarPlanner planner = AstarPlanner(node_, safe_obstacle_distance_, euclidean_distance_cutoff_, planning_tree_resolution_,
                distance_penalty_, greedy_penalty_, min_altitude_, max_altitude_, planning_timeout_, max_waypoint_distance_, unknown_is_occupied_);

            waypoints = planner.findPath(planning_start, planning_goal, planning_tree_, planning_timeout_);
        }

        RCLCPP_INFO(node_->get_logger(), "Planner returned %ld waypoints, before resampling:", waypoints.first.size());

        for (auto &w :waypoints.first) {
            RCLCPP_INFO(node_->get_logger(), "        %.2f, %.2f, %.2f", w.x(), w.y(), w.z());
        }

        /* GOAL_REACHED //{ */
        if (waypoints.second == GOAL_REACHED) {
            RCLCPP_INFO(node_->get_logger(), "Goal reached");
            waypoint_status_ = REACHED;
            status_ = IDLE;
            goal_handle_->succeed(std::make_shared<move_base_msgs::action::MoveBase::Result>());

            break;
        }
        //}

        /* COMPLETE //{ */
        if (waypoints.second == COMPLETE) {
            replanning_counter_ = 0;
        }
        //}

        /* INCOMPLETE //{ */
        if (waypoints.second == INCOMPLETE) {

            if (waypoints.first.size() < 2) {
                RCLCPP_WARN(node_->get_logger(), "Path not found");
                replanning_counter_++;
                break;
            }

            Eigen::Vector3d w_start;
            w_start.x() = waypoints.first.front().x();
            w_start.y() = waypoints.first.front().y();
            w_start.z() = waypoints.first.front().z();

            Eigen::Vector3d w_end;
            w_end.x() = waypoints.first.back().x();
            w_end.y() = waypoints.first.back().y();
            w_end.z() = waypoints.first.back().z();

            double path_start_end_dist = (w_end - w_start).norm();

            if (path_start_end_dist < 1.1 * planning_tree_resolution_) {
                RCLCPP_WARN(node_->get_logger(), "Path too short");
                replanning_counter_++;
                break;
            }
        }
        //}

        /* GOAL_IN_OBSTACLE //{ */
        if (waypoints.second == GOAL_IN_OBSTACLE) {
            replanning_counter_ = 0;
            RCLCPP_WARN(node_->get_logger(), "Goal [%.2f, %.2f, %.2f, %.2f] is inside an inflated obstacle", current_goal_[0],
                        current_goal_[1], current_goal_[2], current_goal_[3]);
        }
        //}

        /* FAILURE //{ */
        if (waypoints.second == FAILURE) {
            RCLCPP_WARN(node_->get_logger(), "Path to goal not found");
            replanning_counter_++;
            break;
        }
        //}

        /* resample path and add yaw //{ */
        std::vector<Eigen::Vector4d> resampled = resamplePath(waypoints.first, uav_pos.w(), current_goal_.w());

        bool output_current_goal = waypoints.second == COMPLETE;
        for (auto &w : resampled) {
            if ((w.head<3>() - uav_pos.head<3>()).norm() <= replanning_distance_) {
                waypoint_out_buffer_.push_back(w);
            } else {
                RCLCPP_INFO(node_->get_logger(), "Path exceeding replanning distance");
                output_current_goal = false;
                break;
            }
        }

        if (output_current_goal) {
            // Reposition last waypoint if the distance between it and the actual goal is less than the planning tree resolution
            Eigen::Vector3d goal(current_goal_.x(), current_goal_.y(), current_goal_.z());
            Eigen::Vector3d last_waypoint(waypoint_out_buffer_.back().x(), waypoint_out_buffer_.back().y(), waypoint_out_buffer_.back().z());
            if ((goal-last_waypoint).norm() > planning_tree_resolution_) {
                waypoint_out_buffer_.push_back(current_goal_);
            } else {
                waypoint_out_buffer_.back() = current_goal_;
            }
        }
        //}

        current_waypoint_id_ = 0;

        // Remove first goal if it is to close to the start position
        // Assuming that there won't be any collisions
        Eigen::Vector3d current_pos(uav_pos[0], uav_pos[1], uav_pos[2]);
        Eigen::Vector3d first_goal(waypoint_out_buffer_.front().x(), waypoint_out_buffer_.front().y(), waypoint_out_buffer_.front().z());
        // TODO: Do I have to check for < 0.2 on every axis instead of distance?
        // TODO: Sometimes the first and second waypoint are the same
        if ((first_goal - current_pos).norm() < 0.2) {
            waypoint_out_buffer_.front() = uav_pos;
            current_waypoint_id_ = 1;
        }

        std::shared_ptr<move_base_msgs::action::MoveBase::Feedback> feedback = 
            std::make_shared<move_base_msgs::action::MoveBase::Feedback>();
        feedback->base_position.header.frame_id = parent_frame_;
        feedback->base_position.pose.position.x = uav_pos[0];
        feedback->base_position.pose.position.y = uav_pos[1];
        feedback->base_position.pose.position.z = uav_pos[2];
        goal_handle_->publish_feedback(feedback);

        if (auto_execution_) {
            status_ = COMMANDING;
            waypoint_status_ = ONGOING;

            RCLCPP_INFO(node_->get_logger(), "Sending %ld waypoints to the control interface:", waypoint_out_buffer_.size());
        } else {
            status_ = IDLE;
            RCLCPP_INFO(node_->get_logger(), "Generated %ld waypoints:", waypoint_out_buffer_.size());
            goal_handle_->succeed(std::make_shared<move_base_msgs::action::MoveBase::Result>());
        }
        for (auto &w : waypoint_out_buffer_) {
            RCLCPP_INFO(node_->get_logger(), "        %.2f, %.2f, %.2f, %.2f", w.x(), w.y(), w.z(), w.w());
        }
        publishPath(waypoint_out_buffer_);

        break;
    }
    //}

    /* COMMANDING //{ */
    // Send waypoints to the drone
    case COMMANDING: {

        if (waypoint_out_buffer_.size() < 1) {
            RCLCPP_WARN(node_->get_logger(), "No waypoints in the output buffer. Replanning");
            replanning_counter_++;
            status_ = PLANNING;
            break;
        }

        RCLCPP_INFO(node_->get_logger(), "Sending waypoint: %.2f, %.2f, %.2f, %.2f",
            waypoint_out_buffer_[current_waypoint_id_][0],
            waypoint_out_buffer_[current_waypoint_id_][1],
            waypoint_out_buffer_[current_waypoint_id_][2],
            waypoint_out_buffer_[current_waypoint_id_][3]);

        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.frame_id = parent_frame_;
        poseMsg.pose.position.x = waypoint_out_buffer_[current_waypoint_id_][0];
        poseMsg.pose.position.y = waypoint_out_buffer_[current_waypoint_id_][1];
        poseMsg.pose.position.z = waypoint_out_buffer_[current_waypoint_id_][2];
        tf2::Quaternion tf_quaternion;
        tf_quaternion.setRPY(0, 0, waypoint_out_buffer_[current_waypoint_id_][3]);
        poseMsg.pose.orientation = tf2::toMsg(tf_quaternion);
        moveto_publisher_->publish(poseMsg);

        status_            = MOVING;
        {
            std::scoped_lock lock(control_diagnostics_mutex_);
            diagnostics_received_counter_ = 0;
        }
        break;
    }
        //}

    /* MOVING //{ */
    // Checking while drone is moving if goal was reached
    case MOVING: {

        replanning_counter_ = 0;
        {
            // Check if diagnostics received after commanding
            std::scoped_lock lock(control_diagnostics_mutex_);
            if (diagnostics_received_counter_ < 10) {
                RCLCPP_WARN(node_->get_logger(), "Control diagnostics not received after commanding, skipping loop");
            } else {
                if ((hovering_ /*&& goal_reached_*/)) { // TODO: When not using moveTo, goal_reached needs to be reinserted to detect finished movements
                    std::shared_ptr<move_base_msgs::action::MoveBase::Feedback> feedback = 
                        std::make_shared<move_base_msgs::action::MoveBase::Feedback>();
                    feedback->base_position.header.frame_id = parent_frame_;
                    feedback->base_position.pose.position.x = waypoint_out_buffer_[current_waypoint_id_][0];
                    feedback->base_position.pose.position.y = waypoint_out_buffer_[current_waypoint_id_][1];
                    feedback->base_position.pose.position.z = waypoint_out_buffer_[current_waypoint_id_][2];
                    if (auto_execution_)
                        goal_handle_->publish_feedback(feedback);

                    current_waypoint_id_++;
                    if (current_waypoint_id_ >= waypoint_out_buffer_.size()) {
                        RCLCPP_INFO(node_->get_logger(), "Final destination reached");
                        status_ = IDLE;
                        if (auto_execution_)
                            goal_handle_->succeed(std::make_shared<move_base_msgs::action::MoveBase::Result>());
                    } else {
                        RCLCPP_INFO(node_->get_logger(), "End of current segment reached");
                        status_ = COMMANDING;
                    }
                } else if (!flying_) {
                    status_ = IDLE;
                    goal_handle_->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
                }
            }
        }
        break;
    }
        //}
    }

    publishDiagnostics();
}
//}

/* resamplePath //{ */
/*
* Insert rotation and adjust waypoint position when too far apart
*/
std::vector<Eigen::Vector4d> Navigation::resamplePath(const std::vector<octomap::point3d> &waypoints, const double start_yaw, const double end_yaw) {
    std::vector<Eigen::Vector4d> ret;

    for (auto &w : waypoints) {
        ret.push_back(Eigen::Vector4d(w.x(), w.y(), w.z(), 0));
    }

    //ret.back().w() = end_yaw;

    return ret;
}
//}

/* createPlanningTree() //{ */

std::shared_ptr<octomap::ColorOcTree> Navigation::createPlanningTree(std::shared_ptr<octomap::ColorOcTree> tree, double resolution) {

    double x, y, z;

    tree->getMetricMin(x, y, z);
    octomap::point3d metric_min(x, y, z);

    tree->getMetricMax(x, y, z);
    octomap::point3d metric_max(x, y, z);

    // TODO: Does pruning make a difference?
    tree->prune();

    // TODO: There seems to be some memory not being freed when using DynamicEDT
    DynamicEDTOctomapBase<octomap::ColorOcTree> edf(euclidean_distance_cutoff_, tree.get(), metric_min, metric_max, unknown_is_occupied_);
    edf.update();

    std::shared_ptr<octomap::ColorOcTree> planning_tree = std::make_shared<octomap::ColorOcTree>(resolution);

    tree->expand();

    double treeRes = tree->getResolution();
    // TODO: What is the nicer way to do this?
    int diff = 0;
    while(treeRes < planning_tree_resolution_) {
        treeRes *= 2;
        diff++;
    }
    int depth = 16 - diff;

    float distance;
    for (auto it = tree->begin(depth); it != tree->end(); it++) {
        octomap::ColorOcTreeNode* node = tree->search(it.getKey());
        if (node && tree->isNodeOccupied(node)) {
            planning_tree->setNodeValue(it.getCoordinate(), TreeValue::OCCUPIED, true);  // obstacle
            planning_tree->setNodeColor(it.getKey(), 0, 0, 0);
        } else if ((distance = edf.getDistance(it.getCoordinate())) <= safe_obstacle_distance_) {
            planning_tree->setNodeValue(it.getCoordinate(), TreeValue::OCCUPIED, true);  // close to obstacle
            int colorValue = distance * 255 / safe_obstacle_distance_;
            planning_tree->setNodeColor(it.getKey(), colorValue, colorValue, colorValue);
        } else {
            planning_tree->setNodeValue(it.getCoordinate(), TreeValue::FREE, true);  // free and safe
            planning_tree->setNodeColor(it.getKey(), 255, 255, 255);
        }
    }
    planning_tree->updateInnerOccupancy();

    octomap_msgs::msg::Octomap octomapMsg;
    octomap_msgs::fullMapToMsg(*planning_tree, octomapMsg);
    octomapMsg.header.frame_id = "map";
    octomapMsg.header.stamp = node_->now();
    planningTreePublisher_->publish(octomapMsg);

    return planning_tree;
}
//}

/* publishDiagnostics //{ */
// TODO: Use diagnostic_msgs::msg::DiagnosticArray instead?
void Navigation::publishDiagnostics() {
    tello_msgs::msg::NavigationDiagnostics msg;
    msg.header.stamp        = node_->get_clock()->now();
    msg.header.frame_id     = parent_frame_;
    msg.state               = STATUS_STRING[status_];
    msg.current_waypoint_status = WAYPOINT_STATUS_STRING[waypoint_status_];
    msg.current_nav_goal[0] = current_goal_.x();
    msg.current_nav_goal[1] = current_goal_.y();
    msg.current_nav_goal[2] = current_goal_.z();
    msg.last_nav_goal[0]    = last_goal_.x();
    msg.last_nav_goal[1]    = last_goal_.y();
    msg.last_nav_goal[2]    = last_goal_.z();
    diagnostics_publisher_->publish(msg);
}
//}

/* publishPath //{ */
void Navigation::publishPath(std::vector<Eigen::Vector4d> waypoints) {
    nav_msgs::msg::Path msg;
    msg.header.stamp    = node_->get_clock()->now();
    msg.header.frame_id = parent_frame_;
    for (const auto &w : waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = parent_frame_;
        pose.pose.position.x        = w.x();
        pose.pose.position.y        = w.y();
        pose.pose.position.z        = w.z();

        tf2::Quaternion tf_quaternion;
        tf_quaternion.setRPY(0, 0, w.w());
        pose.pose.orientation = tf2::toMsg(tf_quaternion);
        msg.poses.push_back(pose);
    }
    path_publisher_->publish(msg);
}
//}
