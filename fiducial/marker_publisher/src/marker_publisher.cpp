#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <map>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>

struct FiducialMarker {
    int id;
    tf2::Transform transform;
};

class MarkerPublisher : public rclcpp::Node {

public:
    MarkerPublisher();

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    rclcpp::TimerBase::SharedPtr tf_timer;
    rclcpp::TimerBase::SharedPtr marker_timer;

    std::map<int, FiducialMarker> markers;

    float marker_len;

    void publishTransforms();
    void publishMarkers();

    bool loadMarkers(std::string path);

};

void MarkerPublisher::publishTransforms() {
    if (markers.size() <= 0) return;

    for (auto const& m : markers) {
        geometry_msgs::msg::TransformStamped ts;
        ts.transform = tf2::toMsg(m.second.transform);
        ts.header.frame_id = "map";
        ts.header.stamp = this->now();
        ts.child_frame_id = "fiducial_" + std::to_string(m.second.id);
        tfBroadcaster->sendTransform(ts);
    }
}

void MarkerPublisher::publishMarkers() {
    for (auto const& m : markers) {
        // Flattened cube
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = marker_len;
        marker.scale.y = marker_len;
        marker.scale.z = marker_len / 10;
        marker.color.g = marker.color.a = 1.0f;
        marker.color.r = marker.color.b = 0.0f;
        marker.id = m.second.id;
        marker.ns = "fiducial";
        marker.header.frame_id = "fiducial_" + std::to_string(m.second.id);
        markerPub->publish(marker);

        // Text
        visualization_msgs::msg::Marker text;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.header.frame_id = "fiducial_" + std::to_string(m.second.id);
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
        text.id = m.second.id;
        text.scale.x = text.scale.y = text.scale.z = 0.1;
        text.pose.position.z = marker_len + 0.1;
        text.ns = "text";
        text.text = std::to_string(m.second.id);
        markerPub->publish(text);
    }
}

bool MarkerPublisher::loadMarkers(std::string path) {
    RCLCPP_INFO(this->get_logger(), "Loading markers from file %s", path.c_str());
    std::ifstream markerFile(path);

    if (!markerFile.is_open()) return false;

    std::string line;
    std::getline(markerFile, line); // Ignore header

    while (std::getline(markerFile, line)) {
        std::stringstream lineStream(line);
        std::string segment;
        std::vector<std::string> segmentList;

        while(std::getline(lineStream, segment, ',')) {
            segmentList.push_back(segment);
        }

        int id = std::stoi(segmentList[0]);
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(std::stod(segmentList[1]),
                                         std::stod(segmentList[2]),
                                         std::stod(segmentList[3])));
        tf2::Quaternion quaternion(tf2::Vector3(std::stod(segmentList[4]),
                                                std::stod(segmentList[5]),
                                                std::stod(segmentList[6])),
                                   std::stod(segmentList[7]));
        transform.setRotation(quaternion);

        //transform = tf2::Transform(tf2::Quaternion(0.5, -0.5, 0.5, -0.5)) * transform;

        FiducialMarker marker = {id, transform};
        markers.insert({id, marker});
    }

    markerFile.close();

    RCLCPP_INFO(this->get_logger(), "Successfully loaded %d markers", markers.size());

    return true;
}

MarkerPublisher::MarkerPublisher() : Node("marker_publisher") {

    double transform_publish_rate = this->declare_parameter("transform_publish_rate", 1.0);
    double marker_publish_rate = this->declare_parameter("marker_publish_rate", 1.0);
    marker_len = this->declare_parameter("marker_length", 0.135);

    std::string file_path = this->declare_parameter("file", "");

    if (loadMarkers(file_path)) {
        tfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        markerPub = this->create_publisher<visualization_msgs::msg::Marker>("/markers", markers.size() * 2);

        tf_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / transform_publish_rate)),
            std::bind(&MarkerPublisher::publishTransforms, this));

        marker_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / marker_publish_rate)),
            std::bind(&MarkerPublisher::publishMarkers, this));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load markers");
        rclcpp::shutdown();
        exit(-1);
    }
}

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<MarkerPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

