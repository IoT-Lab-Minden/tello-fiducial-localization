#include "tello_driver/movement_controller.h"
#include "tello_driver/state.h"
#include "tello_driver/ros_params.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

MovementController::MovementController(std::shared_ptr<ctello::Tello> drone)
: Node("movement_controller"), drone(drone), movementState(LANDED) {

    // Declare and get ROS parameters
    parse_param(this, PARAM_MOVEMENT_SPEED, movementSpeed);
    parse_param(this, PARAM_MAX_HEIGHT, maxHeight);
    parse_param(this, PARAM_PLANNING_UPDATE_RATE, planningUpdateRate);
    parse_param(this, PARAM_AUTO_COOLING, autoCoolingEnabled);
    parse_param(this, PARAM_COOLING_START_TEMP, coolingStartTemp);
    
    // Set drone default movement speed
    std::ostringstream cmdStream;
    cmdStream << "speed " << movementSpeed*100;
    drone->SendCommand(cmdStream.str());

    emergencyCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    navigationCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions emergencyOptions, navigationOptions;
    emergencyOptions.callback_group = emergencyCallbackGroup;
    navigationOptions.callback_group = navigationCallbackGroup;

    // Create all necessary subscribers and publishers
    cmdVelSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
        std::bind(&MovementController::cmdVelCallback, this, std::placeholders::_1), navigationOptions);
    movePathSubscriber = this->create_subscription<nav_msgs::msg::Path>("move_path", 10,
        std::bind(&MovementController::movePathCallback, this, std::placeholders::_1), navigationOptions);
    moveToSubscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("move_to", 10,
        std::bind(&MovementController::moveToCallback, this, std::placeholders::_1), navigationOptions);
    droneStateSubscriber = this->create_subscription<tello_msgs::msg::DroneStateStamped>("drone_state", 1,
        std::bind(&MovementController::droneStateCallback, this, std::placeholders::_1));
    takeoffSubscriber = this->create_subscription<std_msgs::msg::Empty>("takeoff", 1,
        std::bind(&MovementController::takeoffCallback, this, std::placeholders::_1), navigationOptions);
    landSubscriber = this->create_subscription<std_msgs::msg::Empty>("land", 1,
        std::bind(&MovementController::landCallback, this, std::placeholders::_1));
    stopSubscriber = this->create_subscription<std_msgs::msg::Empty>("stop", 1,
        std::bind(&MovementController::stopCallback, this, std::placeholders::_1));
    emergencySubscriber = this->create_subscription<std_msgs::msg::Empty>("emergency", 1,
        std::bind(&MovementController::emergencyCallback, this, std::placeholders::_1), emergencyOptions);
    motorSubscriber = this->create_subscription<std_msgs::msg::Bool>("motor", 1,
        std::bind(&MovementController::motorCallback, this, std::placeholders::_1));
    imuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1,
        std::bind(&MovementController::imuCallback, this, std::placeholders::_1));

    if (autoCoolingEnabled)
        tempSubscriber = this->create_subscription<sensor_msgs::msg::Temperature>("temperature", 1,
            std::bind(&MovementController::tempCallback, this, std::placeholders::_1));

    moveToActionServer = rclcpp_action::create_server<move_base_msgs::action::MoveBase>(
        this,
        "move_base",
        std::bind(&MovementController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MovementController::handleCancel, this, std::placeholders::_1),
        std::bind(&MovementController::handleAccepted, this, std::placeholders::_1), rcl_action_server_get_default_options(), navigationCallbackGroup);
    // Setup transform listener
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    flightStatePublisher = this->create_publisher<tello_msgs::msg::FlightState>("flight_state", 1);

    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    navigation = std::make_shared<Navigation>(this, navigationCallbackGroup);

    idleTimer =
        this->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&MovementController::idleRoutine, this));
}

void MovementController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Don't accept movement commands to not interrupt the landing process
    if (movementState == LANDING) return;

    // Make sure the drone takes off before sending the first movement command
    if (!flying) {
        // Make sure that there is at least some movement demanded
        float movement = msg->linear.x + msg->linear.y + msg->linear.z + msg->angular.z;
        if (movement < 0.01) return;

        takeoff();
        return;
    }

    // It does not make sense to send movement commands when the drone is not flying
    if (movementState != FLYING) return;

    // Calculate the exact speeds
    int roll = int(msg->linear.y*-100*movementSpeed);
    int pitch = int(msg->linear.x*100*movementSpeed);
    int throttle = int(msg->linear.z*100*movementSpeed);
    int yaw = int(msg->angular.z*-100);

    // And send them to the drone
    std::ostringstream rcCmd;
    rcCmd << "rc " << roll << " " << pitch << " "
        << throttle << " " << yaw;
    std::string cmd = rcCmd.str();
    drone->SendCommandNoResponse(cmd);
    if (abs(roll + pitch + yaw + throttle) > 0.01) {
        lastMovingTime = this->now();
        hovering = false; // Directly setting hovering here so that there is no delay
    }
    lastCmdTime = this->now();

    RCLCPP_INFO(this->get_logger(), "CMD: RC %d %d %d %d", roll, pitch, yaw, throttle);
}

void MovementController::movePathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // It does not make sense to send movement commands when the drone is not flying
    if (movementState != FLYING) return;

    navigation->setWaypoints(msg);
}

void MovementController::moveToCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // It does not make sense to send movement commands when the drone is not flying
    if (movementState != FLYING) return;

    geometry_msgs::msg::TransformStamped tfMsg;
    try {
        tfMsg = tfBuffer->lookupTransform(msg->header.frame_id, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform from %s to %s: %s",
            msg->header.frame_id.c_str(), "base_link", ex.what());
        return;
    }

    /*
    // TODO: Remove this when localisation works correctly
    geometry_msgs::msg::TransformStamped tfMsgOdom;
    try {
        tfMsgOdom = tfBuffer->lookupTransform("map", "odom", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform from %s to %s: %s",
            "map", "odom", ex.what());
        return;
    }

    // Using internal drone position for x and y for now
    // Only x and y because they are tracked relative
    tfMsg.transform.translation.x = tfMsgOdom.transform.translation.x + internalUavPosition.x();
    tfMsg.transform.translation.y = tfMsgOdom.transform.translation.y + internalUavPosition.y();*/

    Eigen::Vector3d goal(
        msg->pose.position.x - tfMsg.transform.translation.x,
        msg->pose.position.y - tfMsg.transform.translation.y,
        msg->pose.position.z - tfMsg.transform.translation.z);

    RCLCPP_INFO(this->get_logger(), "Current position: %.2f, %.2f, %.2f", tfMsg.transform.translation.x,
        tfMsg.transform.translation.y, tfMsg.transform.translation.z);
    //RCLCPP_INFO(this->get_logger(), "Current internal position: %.2f, %.2f", internalUavPosition.x(),
    //    internalUavPosition.y());
    RCLCPP_INFO(this->get_logger(), "Absolute goal: %.2f, %.2f, %.2f", msg->pose.position.x,
        msg->pose.position.y, msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Relative goal: %.2f, %.2f, %.2f", goal[0], goal[1], goal[2]);

    // The drone requires at least a minimum distance of 20 cm
    if (abs(goal[0]) < 0.2 &&
        abs(goal[1]) < 0.2 &&
        abs(goal[2]) < 0.2) {
        RCLCPP_INFO(this->get_logger(), "moveTo cmd rejected: position lower than minimum");
        return;
    }

    // And a maximum distance of 5 meters
    if (abs(goal[0]) > 5.0 ||
        abs(goal[1]) > 5.0 ||
        abs(goal[2]) > 5.0) {
        RCLCPP_INFO(this->get_logger(), "moveTo cmd rejected: position higher than maximum");
        return;
    }

    // Requires relative positions
    // TODO: Currently ignoring orientation
    std::ostringstream rcCmd;
    rcCmd << "go " << int(goal[0]*100) << " " << int(goal[1]*100) << " "
        << int(goal[2]*100) << " " << int(movementSpeed*100);
    RCLCPP_INFO(this->get_logger(), "CMD: %s", rcCmd.str().c_str());
    drone->SendCommand(rcCmd.str());
    lastMovingTime = this->now();
    hovering = false; // Directly setting hovering here so that there is no delay
    lastCmdTime = this->now();
    internalUavPosition += Eigen::Vector3d(goal[0], goal[1], goal[2]);
}

rclcpp_action::GoalResponse MovementController::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal) {

    if (movementState != FLYING && this->get_parameter(PARAM_PLANNING_AUTO_EXECTION).as_bool()) {
        RCLCPP_INFO(this->get_logger(), "moveTo action rejected: Drone not flying");

        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Accept when no other goal is active
    // This currently assumes that handleAccepted is called directly after handleGoal
    if (!currentGoal || !currentGoal->is_active()) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Only accept a new goal while executing another goal when both share the same frame_id
    // TODO: Check if handleAccepted is automatically called after finishing previous goal
    if (currentGoal->get_goal()->target_pose.header.frame_id == goal->target_pose.header.frame_id) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    }

    // Reject everything else
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse MovementController::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle) {
    
    // Accept canceling every goal when requested
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MovementController::handleAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handle) {

    // Set current goal
    currentGoal = goal_handle;

    geometry_msgs::msg::TransformStamped tfMsg;
    try {
        tfMsg = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform from %s to %s: %s",
            "map", "base_link", ex.what());
        return;
    }

    // TODO: Remove this, ignore internal uav pos
    //navigation->setUavPos(internalUavPosition.x(), internalUavPosition.y());

    navigation->setGoal(goal_handle);
    /*geometry_msgs::msg::TransformStamped tfMsg;

    // Use lookupTransform to automatically convert it to the correct frame
    try {
        tfMsg = tfBuffer->lookupTransform(goal_handle->get_goal()->target_pose.header.frame_id, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform from %s to %s: %s",
            goal_handle->get_goal()->target_pose.header.frame_id.c_str(), "base_link", ex.what());

        goal_handle->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
        return;
    }

    // Determine translation and rotation values
    tf2::Vector3 positionDroneCentered(
        goal_handle->get_goal()->target_pose.pose.position.x - tfMsg.transform.translation.x,
        goal_handle->get_goal()->target_pose.pose.position.y - tfMsg.transform.translation.y,
        goal_handle->get_goal()->target_pose.pose.position.z - tfMsg.transform.translation.z
    );
    tf2::Quaternion desiredRotation, currentDroneRotation;
    tf2::fromMsg(goal_handle->get_goal()->target_pose.pose.orientation, desiredRotation);
    tf2::fromMsg(tfMsg.transform.rotation, currentDroneRotation);

    tf2::Vector3 relativePosition = tf2::quatRotate(currentDroneRotation, positionDroneCentered);
    tf2::Quaternion relativeRotation = desiredRotation * currentDroneRotation.inverse(); // TODO: Check if the inverse of currentDroneRotation is correct

    // The drone requires at least a minimum distance of 20 cm
    if (abs(relativePosition.x()) < 0.2 &&
        abs(relativePosition.y()) < 0.2 &&
        abs(relativePosition.z()) < 0.2) {
        RCLCPP_INFO(this->get_logger(), "moveTo cmd rejected: position lower than minimum requirements");
        goal_handle->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
        return;
    }

    // And a maximum distance of 5 meters
    if (abs(relativePosition.x()) > 5.0 ||
        abs(relativePosition.y()) > 5.0 ||
        abs(relativePosition.z()) > 5.0) {
        RCLCPP_INFO(this->get_logger(), "moveTo cmd rejected: position higher than maximum");
        goal_handle->abort(std::make_shared<move_base_msgs::action::MoveBase::Result>());
        return;
    }

    // Send movement command
    std::ostringstream rcCmd;
    rcCmd << "go " <<relativePosition.x()*100 << " "
        << relativePosition.y()*100 << " "
        << relativePosition.z()*100 << " "
        << movementSpeed*100;
    drone->SendCommand(rcCmd.str());
    RCLCPP_INFO(this->get_logger(), "CMD: %s", rcCmd.str().c_str());

    // TODO: Rotate drone after finishing go cmd*/
}

void MovementController::takeoffCallback(const std_msgs::msg::Empty::SharedPtr) {
    if (movementState != LANDED) return;

    takeoff();
}

void MovementController::landCallback(const std_msgs::msg::Empty::SharedPtr) {
    land(true);
}

void MovementController::stopCallback(const std_msgs::msg::Empty::SharedPtr) {
    if (movementState != LANDED) return;

    // Stop moving drone and start hovering
    navigation->hover();
    drone->SendCommand("stop");
    lastCmdTime = this->now();
    RCLCPP_INFO(this->get_logger(), "CMD: STOP");
}

void MovementController::emergencyCallback(const std_msgs::msg::Empty::SharedPtr) {
    // Go into emergency mode. Requires complete restart of drone afterwards
    drone->SendCommandNoResponse("emergency");
    flying = false;
    movementState = EMERGENCY;
    RCLCPP_INFO(this->get_logger(), "CMD: EMERGENCY");
}

void MovementController::motorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (movementState != LANDED) return;

    // Motor commands are used to start the rotors too cool the drone when not flying
    if (msg->data) {
        drone->SendCommand("motoron");
        RCLCPP_INFO(this->get_logger(), "CMD: MOTORON");
        motorOn = true;
    } else {
        drone->SendCommand("motoroff");
        RCLCPP_INFO(this->get_logger(), "CMD: MOTOROFF");
        motorOn = false;
    }

    lastCmdTime = this->now();
}

void MovementController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Vector3 velocity(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);
    double deltaTime = 0;
    if (lastImuTime.get_clock_type() == rclcpp::Time(msg->header.stamp).get_clock_type())
        deltaTime = (rclcpp::Time(msg->header.stamp) - lastImuTime).seconds();

    position += velocity * deltaTime;

    // Publish transform msg
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    //transform.transform.translation.x = position.x();
    //transform.transform.translation.y = position.y();
    // Using height state info here because it is a more accurate value even though its resolution is dm
    //transform.transform.translation.z = height / 100; // Convert from cm to m
    //transform.transform.rotation = msg->orientation;

    tfBroadcaster->sendTransform(transform);

    geometry_msgs::msg::TransformStamped orientation;
    orientation.header.stamp = msg->header.stamp;
    orientation.header.frame_id = "map";
    orientation.child_frame_id = "drone_orientation";
    orientation.transform.rotation = msg->orientation;
    tfBroadcaster->sendTransform(orientation);

    // Publish odom msg
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    // Using height state info here because it is a more accurate value even though its resolution is dm
    odom.pose.pose.position.z = height / 100; // Convert from cm to m
    odom.pose.pose.orientation = msg->orientation;

    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();

    // TODO: Correctly set angular velocities
    // (Maybe already calculate them in ImuPublisher?)
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    odomPublisher->publish(odom);

    lastImuTime = rclcpp::Time(msg->header.stamp);
}

void MovementController::tempCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    if (!motorOn && msg->temperature >=coolingStartTemp) {
        drone->SendCommand("motoron");
        RCLCPP_INFO(this->get_logger(), "Started auto cooling");
        motorOn = true;
    }
}

void MovementController::droneStateCallback(const tello_msgs::msg::DroneStateStamped::SharedPtr msg) {
    height = msg->h;
    currentTime = msg->header.stamp;

    // Give the drone some time to finish taking off before going into flight state
    if (movementState == TAKINGOFF) {
        // TODO: Determine better wait time
        // For rc commands 2 seconds seem to be fine
        // But go commands require the visual positioning system of the drone
        // which takes longer to initialize (> 8sec?)
        if (getTimeDifferenceNs(takeoffTime, msg->header.stamp) > 2.0f){
            movementState = FLYING;
        }
    }


    // Update flight state when flying
    if (movementState == FLYING || movementState == LANDING) {
        if (getTimeDifferenceNs(lastFlyingTime, msg->header.stamp) > 1.2f) { // Needs to be larger than 1 second
            bool flying_prev = flying;
            flying = (currentMotorRunningTime != msg->time);
            if (flying) {
                currentMotorRunningTime = msg->time;
                if (!flying_prev && flying)
                    RCLCPP_INFO(this->get_logger(), "Now flying. Ready to receive movement commands");
            } else {
                hovering = false;
                movementState = LANDED;
                motorOn = false;
            }
            lastFlyingTime = msg->header.stamp;

            // Make sure that when we want to land the drone
            // it will actually land
            if (movementState == LANDING && flying) {
                land(true);
            }
        }

        // Check if the drone is hovering
        if (movementState == FLYING) {
            if (abs(msg->vgx) > 0.0 || abs(msg->vgy) > 0.0 || abs(msg->vgz) > 0.0) {
                lastMovingTime = this->now();
                hovering = false;
            } else if (getTimeDifferenceNs(lastMovingTime, msg->header.stamp) > 0.5f) {
                hovering = true;
            }
        }
    }

    // Prevent drone from flying too high
    if (flying && msg->h > maxHeight && movementState != LANDING) {
        land(false);
    }

    // Print some information about the current state once every second
    static int consoleCounter = 0;
    if (consoleCounter % 5 == 0)
        RCLCPP_INFO(this->get_logger(), "motor: %d state: %d flying: %s hovering: %s height: %f battery: %f",
            currentMotorRunningTime, movementState, flying ? "true" : "false", hovering ? "true" : "false", height, msg->bat);
    consoleCounter++;

    // Publish flight state for nodes that want to know if movement commands will be executed
    tello_msgs::msg::FlightState flightStateMsg;
    flightStateMsg.state = movementState;
    flightStateMsg.flying = flying;
    flightStateMsg.hovering = hovering;
    flightStatePublisher->publish(flightStateMsg);
    navigation->setDroneStatus(movementState, flying, hovering);
}

void MovementController::takeoff() {
    if (movementState != LANDED) return;

    lastCmdTime = this->now();
    drone->SendCommand("takeoff");
    takeoffTime = currentTime;
    movementState = TAKINGOFF;
    RCLCPP_INFO(this->get_logger(), "CMD: TAKEOFF");
}

void MovementController::land(bool force) {
    if (!force && movementState != FLYING) return;
    
    movementState = LANDING;
    drone->SendCommand("land");
    RCLCPP_INFO(this->get_logger(), "CMD: LAND");
}

void MovementController::idleRoutine() {
    // Prevent tello drone from auto landing (doest it after 10? sec not receiving a cmd)
    if (movementState == FLYING) {
        // TODO: How to solve for GO cmds taking longer than 8 sec?
        if ((this->now()-lastCmdTime).seconds() > 8.0) {
            drone->SendCommandNoResponse("RC 0 0 0 0");
            lastCmdTime = this->now();
            RCLCPP_INFO(this->get_logger(), "Preventing auto land");
        }
    }
}

float MovementController::getTimeDifferenceNs(builtin_interfaces::msg::Time prev,
                                          builtin_interfaces::msg::Time next) {

    rclcpp::Time prevTime(prev.sec, prev.nanosec);
    rclcpp::Time nextTime(next.sec, next.nanosec);
    rclcpp::Duration duration = nextTime - prevTime;
    return duration.seconds();
}
