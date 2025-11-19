#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>
#include "fiducial_msgs/srv/save_markers.hpp"
#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"

#include <vector>

#include <iostream>
#include <fstream>

struct EvalData {
    geometry_msgs::msg::TransformStamped ipadTf;
    geometry_msgs::msg::TransformStamped droneTf;
    geometry_msgs::msg::TransformStamped fiducialTf;
};

class EvalDataRecorder : public rclcpp::Node {

public:
    EvalDataRecorder();

private:
    std::vector<EvalData> data;

    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr ft_sub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr triggerService;
    rclcpp::Service<fiducial_msgs::srv::SaveMarkers>::SharedPtr saveService;

    geometry_msgs::msg::TransformStamped best_fiducial;

    void addEvalPose(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);

    void save(const fiducial_msgs::srv::SaveMarkers::Request::SharedPtr request,
                        fiducial_msgs::srv::SaveMarkers::Response::SharedPtr response);

    void fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg);

    fiducial_msgs::msg::FiducialTransform::SharedPtr getBestTransform(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg);

};

void EvalDataRecorder::addEvalPose(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                         std_srvs::srv::Trigger::Response::SharedPtr response) {
    
    geometry_msgs::msg::TransformStamped ipadTf, droneTf;
    try {
        ipadTf = tfBuffer->lookupTransform("map", "ipad_camera", tf2::TimePointZero);
        droneTf = tfBuffer->lookupTransform("map", "eval_pose", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transforms: %s", ex.what());
        response->success = false;
        response->message = "Failed to get transforms";
        return;
    }

    data.push_back(EvalData{ipadTf, droneTf, best_fiducial});

    RCLCPP_INFO(this->get_logger(), "Recorded pose");

    response->success = true;
}

void EvalDataRecorder::save(const fiducial_msgs::srv::SaveMarkers::Request::SharedPtr request,
                                         fiducial_msgs::srv::SaveMarkers::Response::SharedPtr response) {
    
    if (data.size() <= 0) {
        RCLCPP_INFO(this->get_logger(), "No markers detected to save");
        response->success = false;
        return;
    }

    std::ofstream markerFile(request->path, std::ios::out | std::ios::trunc);
    if (!markerFile.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Failed to open save file");
        response->success = false;
        return;
    }

    markerFile << "id,x,y,z,dx,dy,dz,dw,x_true,y_true,z_true,dx_true,dy_true,dz_true,dw_true,fx,fy,fz,fdx,fdy,fdz,fdw" << std::endl;

    int i = 1;
    for (auto const& m : data) {
        markerFile << i << ',';
        markerFile << m.droneTf.transform.translation.x << ',' << m.droneTf.transform.translation.y << ','
                    << m.droneTf.transform.translation.z << ',';
        markerFile << m.droneTf.transform.rotation.x << ',' << m.droneTf.transform.rotation.y << ','
                    << m.droneTf.transform.rotation.z << ',' << m.droneTf.transform.rotation.w << ',';
        markerFile << m.ipadTf.transform.translation.x << ',' << m.ipadTf.transform.translation.y << ','
                    << m.ipadTf.transform.translation.z << ',';
        markerFile << m.ipadTf.transform.rotation.x << ',' << m.ipadTf.transform.rotation.y << ','
                    << m.ipadTf.transform.rotation.z << ',' << m.ipadTf.transform.rotation.w << ',';
        markerFile << m.fiducialTf.transform.translation.x << ',' << m.fiducialTf.transform.translation.y << ','
                    << m.fiducialTf.transform.translation.z << ',';
        markerFile << m.fiducialTf.transform.rotation.x << ',' << m.fiducialTf.transform.rotation.y << ','
                    << m.fiducialTf.transform.rotation.z << ',' << m.fiducialTf.transform.rotation.w;
        markerFile << std::endl;
        i++;
    }

    markerFile.close();

    RCLCPP_INFO(this->get_logger(), "Saved to file %s", request->path.c_str());

    response->success = true;
}

void EvalDataRecorder::fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {

    if (msg->transforms.size() <= 0) return;

    // Get best fiducial transform for uav pose calculation
    fiducial_msgs::msg::FiducialTransform::SharedPtr best_ft = getBestTransform(msg);

    try {
        best_fiducial = tfBuffer->lookupTransform("ipad_camera", "fiducial_" + std::to_string(best_ft->fiducial_id), rclcpp::Time(msg->header.stamp));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform: %s", ex.what());
        return;
    }
}

fiducial_msgs::msg::FiducialTransform::SharedPtr EvalDataRecorder::getBestTransform(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {
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

EvalDataRecorder::EvalDataRecorder()
: Node("eval_data_recorder") {
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    triggerService = this->create_service<std_srvs::srv::Trigger>("/add_eval_pose",
        std::bind(&EvalDataRecorder::addEvalPose, this, std::placeholders::_1, std::placeholders::_2));

    saveService = this->create_service<fiducial_msgs::srv::SaveMarkers>("/save_eval_poses",
        std::bind(&EvalDataRecorder::save, this, std::placeholders::_1, std::placeholders::_2));
}

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<EvalDataRecorder>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
