
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
    tf2::Transform getTransform(fiducial_msgs::msg::FiducialTransform::SharedPtr ft, rclcpp::Time time);

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

tf2::Transform FiducialLocalisation3D::getTransform(fiducial_msgs::msg::FiducialTransform::SharedPtr ft, rclcpp::Time time) {

    // Lookup all required transforms
    geometry_msgs::msg::TransformStamped fiducialPoseMsg;
    geometry_msgs::msg::TransformStamped odomCameraPoseMsg;

        try {
            fiducialPoseMsg = tfBuffer->lookupTransform("map", "fiducial_" + std::to_string(ft->fiducial_id), time);
            odomCameraPoseMsg = tfBuffer->lookupTransform("odom", "camera", rclcpp::Time(0)); // TODO:
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "Failed to get transform: %s", ex.what());
            return tf2::Transform();
        }

    // Convert them to tf2::Transform objects
    tf2::Transform fiducialMapPose, odomCameraPose, cameraFiducialPose;
    tf2::fromMsg(fiducialPoseMsg.transform, fiducialMapPose);
    tf2::fromMsg(odomCameraPoseMsg.transform, odomCameraPose);
    tf2::fromMsg(ft->transform, cameraFiducialPose);

    tf2::Transform fidPoseFromOdom = odomCameraPose * cameraFiducialPose;
    tf2::Transform fidPoseFromMap = fiducialMapPose;
    tf2::Transform mapToOdom = fidPoseFromMap * fidPoseFromOdom.inverse();

    return mapToOdom;
}

void FiducialLocalisation3D::fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "Received %d fiducials", msg->transforms.size());

    if (msg->transforms.size() <= 0) return;

    // Get best fiducial transform for uav pose calculation
    fiducial_msgs::msg::FiducialTransform::SharedPtr best_ft = getBestTransform(msg);

    tf2::Transform mapToOdom = getTransform(best_ft, rclcpp::Time(msg->header.stamp));

    std::vector<tf2::Transform> tfs;
    std::vector<int> ids;
    std::vector<float> error_values;
    for (size_t i = 0; i < msg->transforms.size(); i++) {
        tfs.push_back(getTransform(std::make_shared<fiducial_msgs::msg::FiducialTransform>(msg->transforms[i]), rclcpp::Time(msg->header.stamp)));
        ids.push_back(msg->transforms[i].fiducial_id);
        error_values.push_back(msg->transforms[i].object_error);
    }

    float errors_summed = 0;
    for (float e : error_values) { errors_summed += 1 - e;}

    tf2::Vector3 pos(0, 0, 0);
    for (size_t i = 0; i < tfs.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "    pos: %f %f %f", tfs[i].getOrigin().x(), tfs[i].getOrigin().y(), tfs[i].getOrigin().z());
        //pos += ((1-error_values[i]) / errors_summed) * tfs[i].getOrigin();
        pos += (1.0 / tfs.size()) * tfs[i].getOrigin();
    }
    RCLCPP_INFO(this->get_logger(), "POS: %f %f %f", pos.x(), pos.y(), pos.z());

    float x, y, z, w;
    for (size_t i = 0; i < tfs.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "    rot: %f %f %f %f", tfs[i].getRotation().x(), tfs[i].getRotation().y(), tfs[i].getRotation().z(), tfs[i].getRotation().w());
        if (tfs[i].getRotation().w() >= 0) {
            //rot += ((1-error_values[i]) / errors_summed) * tf2::tf2Vector4(tfs[i].getRotation().x(), tfs[i].getRotation().y(), tfs[i].getRotation().z(), tfs[i].getRotation().w());
            x += ((1-error_values[i]) / errors_summed) * tfs[i].getRotation().x();
            y += ((1-error_values[i]) / errors_summed) * tfs[i].getRotation().y();
            z += ((1-error_values[i]) / errors_summed) * tfs[i].getRotation().z();
            w += ((1-error_values[i]) / errors_summed) * tfs[i].getRotation().w();
        }/* else {
            //rot += ((1-error_values[i]) / errors_summed) * tf2::tf2Vector4(-1*tfs[i].getRotation().x(), -1*tfs[i].getRotation().y(), -1*tfs[i].getRotation().z(), -1*tfs[i].getRotation().w());
            x += ((1-error_values[i]) / errors_summed) * -1*tfs[i].getRotation().x();
            y += ((1-error_values[i]) / errors_summed) * -1*tfs[i].getRotation().y();
            z += ((1-error_values[i]) / errors_summed) * -1*tfs[i].getRotation().z();
            w += ((1-error_values[i]) / errors_summed) * -1*tfs[i].getRotation().w();
        }*/
        RCLCPP_INFO(this->get_logger(), "   ROT: %f %f %f %f", x, y, z, w);
    }
    tf2::Quaternion quat(x, y, z, w);
    quat.normalize();
    RCLCPP_INFO(this->get_logger(), "ROT: %f %f %f %f", quat.x(), quat.y(), quat.z(), quat.w());

    tf2::Transform transform(quat.normalized(), pos);

    /*for (size_t i = 0; i < tfs.size(); i++) {

        // Create uav tf msg
        geometry_msgs::msg::TransformStamped uavPoseMsg;
        uavPoseMsg.header.stamp = msg->header.stamp;
        uavPoseMsg.header.frame_id = "map";
        uavPoseMsg.child_frame_id = std::string("odom_") + std::to_string(ids[i]);
        uavPoseMsg.transform = tf2::toMsg(tfs[i]);

        // And publish it
        tfBroadcaster->sendTransform(uavPoseMsg);
    }*/

    // Create uav tf msg
    geometry_msgs::msg::TransformStamped uavPoseMsg;
    uavPoseMsg.header.stamp = msg->header.stamp;
    uavPoseMsg.header.frame_id = "map";
    uavPoseMsg.child_frame_id = "odom";
    //uavPoseMsg.transform = tf2::toMsg(mapToOdom);
    uavPoseMsg.transform = tf2::toMsg(transform);

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