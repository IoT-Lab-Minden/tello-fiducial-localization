#include "tello_joy/joy_logitech.h"

JoyLogitech::JoyLogitech()
: Node("joy_logitech") {

    motorOn = false;

    joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1,
        std::bind(&JoyLogitech::joyCallback, this, std::placeholders::_1));

    takeoffPublisher = this->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
    landPublisher = this->create_publisher<std_msgs::msg::Empty>("land", 10);
    stopPublisher = this->create_publisher<std_msgs::msg::Empty>("stop", 10);
    emergencyPublisher = this->create_publisher<std_msgs::msg::Empty>("emergency", 10);
    motorPublisher = this->create_publisher<std_msgs::msg::Bool>("motor", 10);
}

void JoyLogitech::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    /**
     * Commands:
     *   LT + RT: Emergency stop
     *   RB + Y: Takeoff
     *   RB + X: Motor on
     *   RB + B: Stop
     *   RB + A: Land
    */
    if (msg->buttons[6] && msg->buttons[7]) {
        std_msgs::msg::Empty msg;
        emergencyPublisher->publish(msg);
    } else if (msg->buttons[5]) {
        if (!previousButtonPresses[3] && msg->buttons[3]) {
            std_msgs::msg::Empty msg;
            takeoffPublisher->publish(msg);
        } else if (!previousButtonPresses[2] && msg->buttons[2]) {
            std_msgs::msg::Empty msg;
            stopPublisher->publish(msg);
        } else if (!previousButtonPresses[1] && msg->buttons[1]) {
            std_msgs::msg::Empty msg;
            landPublisher->publish(msg);
        } else if (!previousButtonPresses[0] && msg->buttons[0]) {
            motorOn = !motorOn;
            std_msgs::msg::Bool msg;
            msg.data = motorOn;
            motorPublisher->publish(msg);
        }
    }
    previousButtonPresses = msg->buttons;
} 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<JoyLogitech> joy = std::make_shared<JoyLogitech>();

    rclcpp::spin(joy);

    rclcpp::shutdown();
    return 0;
}
