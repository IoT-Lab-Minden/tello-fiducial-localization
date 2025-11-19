
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"

class FiducialLocalisation3D : public rclcpp::Node {
private:
    rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr ft_sub;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;

    void fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg);

    fiducial_msgs::msg::FiducialTransform::SharedPtr getBestTransform(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg);

public:
    FiducialLocalisation3D();
};


fiducial_msgs::msg::FiducialTransform::SharedPtr FiducialLocalisation3D::getBestTransform(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {
    fiducial_msgs::msg::FiducialTransform::SharedPtr best_ft;

    // Determine best fiducial transform of the current image based on the object error
    for (size_t i = 0; i < msg->transforms.size(); i++) {
        fiducial_msgs::msg::FiducialTransform ft = msg->transforms[i];

        if (best_ft == nullptr || best_ft->object_error > ft.object_error) {
            best_ft = std::make_shared<fiducial_msgs::msg::FiducialTransform>(ft);
        }
    }
    
    return best_ft;
}

void FiducialLocalisation3D::fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "Received %d fiducials", msg->transforms.size());

    if (msg->transforms.size() <= 0) return;

    // Get best fiducial transform for uav pose calculation
    fiducial_msgs::msg::FiducialTransform::SharedPtr best_ft = getBestTransform(msg);

    // Lookup all required transforms
    geometry_msgs::msg::TransformStamped fiducialPoseMsg;
    geometry_msgs::msg::TransformStamped odomCameraPoseMsg;
    try {
        fiducialPoseMsg = tfBuffer->lookupTransform("map", "fiducial_" + std::to_string(best_ft->fiducial_id), rclcpp::Time(msg->header.stamp));
        odomCameraPoseMsg = tfBuffer->lookupTransform("odom", "camera", rclcpp::Time(0)); // TODO:
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform: %s", ex.what());
        return;
    }

    // Convert them to tf2::Transform objects
    tf2::Transform fiducialMapPose, odomCameraPose, cameraFiducialPose;
    tf2::fromMsg(fiducialPoseMsg.transform, fiducialMapPose);
    tf2::fromMsg(odomCameraPoseMsg.transform, odomCameraPose);
    tf2::fromMsg(best_ft->transform, cameraFiducialPose);

    tf2::Transform fidPoseFromOdom = odomCameraPose * cameraFiducialPose;
    tf2::Transform fidPoseFromMap = fiducialMapPose;
    tf2::Transform mapToOdom = fidPoseFromMap * fidPoseFromOdom.inverse();

    // Create uav tf msg
    geometry_msgs::msg::TransformStamped uavPoseMsg;
    uavPoseMsg.header.stamp = msg->header.stamp;
    uavPoseMsg.header.frame_id = "map";
    uavPoseMsg.child_frame_id = "odom";
    uavPoseMsg.transform = tf2::toMsg(mapToOdom);

    // And publish it
    tfBroadcaster->sendTransform(uavPoseMsg);
    RCLCPP_INFO(this->get_logger(), "Updated transform based on fiducial %d", best_ft->fiducial_id);
}

FiducialLocalisation3D::FiducialLocalisation3D()
 : rclcpp::Node("fiducial_localisation_3d") 
{
    ft_sub = this->create_subscription<fiducial_msgs::msg::FiducialTransformArray>("/aruco_detect/fiducial_transforms", 1,
                     std::bind(&FiducialLocalisation3D::fiducialCallback, this, std::placeholders::_1));

    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    tfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FiducialLocalisation3D>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}