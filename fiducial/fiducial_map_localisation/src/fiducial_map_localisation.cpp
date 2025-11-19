#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cv_bridge/cv_bridge.h>
#include "fiducial_msgs/srv/save_markers.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <map>

#include <iostream>
#include <fstream>

struct FiducialMarker {
    int id;
    tf2::Transform transform;
    double error;
};

class FiducialMapLocalisation : public rclcpp::Node
{
private:

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub;
    std::shared_ptr<image_transport::ImageTransport> img_trans;
    image_transport::Publisher image_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    image_transport::Subscriber img_sub;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;
    rclcpp::Service<fiducial_msgs::srv::SaveMarkers>::SharedPtr saveMarkersService;
    rclcpp::TimerBase::SharedPtr timer;

    std::map<int, FiducialMarker> markers;

    bool publish_images;

    int dicno;
    double fiducial_len;

    bool haveCamInfo;
    int frameNum;
    std::vector <std::vector <cv::Point2f> > corners;
    std::vector <int> ids;
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    std::map<int, double> fiducialLens;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    void estimatePoseSingleMarkers(float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                                   std::vector<double>& reprojectionError);


    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void publishTransforms();
    void publishMarker(FiducialMarker &fid);

    void saveMarkers(const fiducial_msgs::srv::SaveMarkers::Request::SharedPtr request,
                        fiducial_msgs::srv::SaveMarkers::Response::SharedPtr response);

  public:
    FiducialMapLocalisation();
};


/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, std::vector<cv::Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// estimate reprojection error
static double getReprojectionError(const std::vector<cv::Point3f> &objectPoints,
                            const std::vector<cv::Point2f> &imagePoints,
                            const cv::Mat &cameraMatrix, const cv::Mat  &distCoeffs,
                            const cv::Vec3d &rvec, const cv::Vec3d &tvec) {

    std::vector<cv::Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

void FiducialMapLocalisation::estimatePoseSingleMarkers(float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                                std::vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    std::vector<cv::Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
       double fiducialSize = markerLength;

       std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
       if (it != fiducialLens.end()) {
          fiducialSize = it->second;
       }

       getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
       cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                    rvecs[i], tvecs[i]);

       reprojectionError[i] =
          getReprojectionError(markerObjPoints, corners[i],
                               cameraMatrix, distCoeffs,
                               rvecs[i], tvecs[i]);
    }
}

void FiducialMapLocalisation::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->k != std::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->k[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->d[i];
        }

        haveCamInfo = true;
        RCLCPP_INFO(this->get_logger(), "Got camera intrinsics");
    } else {
        RCLCPP_WARN(this->get_logger(), "%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void FiducialMapLocalisation::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {

    RCLCPP_INFO(this->get_logger(), "Got image");
    std::vector <cv::Vec3d>  rvecs, tvecs;
    frameNum++;
        
    if (!haveCamInfo) {
        if (frameNum > 5) {
            RCLCPP_ERROR(this->get_logger(),"No camera intrinsics");
        }
        return;
    }

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        RCLCPP_INFO(this->get_logger(), "Detected %d markers", (int)ids.size());

        if (publish_images) {
            if(ids.size() > 0) {
                cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            }
        }

        std::vector <double>reprojectionError;
        estimatePoseSingleMarkers((float)fiducial_len,
                                    cameraMatrix, distortionCoeffs,
                                    rvecs, tvecs,
                                    reprojectionError);

        for (size_t i=0; i<ids.size(); i++) {
            if (publish_images) {
                cv::aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);
            }

            RCLCPP_INFO(this->get_logger(), "Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                        tvecs[i][0], tvecs[i][1], tvecs[i][2],
                        rvecs[i][0], rvecs[i][1], rvecs[i][2]);

            double angle = cv::norm(rvecs[i]);
            cv::Vec3d axis = rvecs[i] / angle;
            RCLCPP_INFO(this->get_logger(), "angle %f axis %f %f %f",
                        angle, axis[0], axis[1], axis[2]);

            // Convert image_error (in pixels) to object_error (in meters)
            double object_error =
                (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                (norm(tvecs[i]) / fiducial_len);

            tf2::Quaternion q;
            tf2::Transform tf;
            tf.setOrigin(tf2::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
            q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
            tf.setRotation(q);

            // TODO: This code is here twice
            geometry_msgs::msg::TransformStamped cameraPoseMsg;
            try {
                cameraPoseMsg = tfBuffer->lookupTransform("map", "ipad_camera", rclcpp::Time(msg->header.stamp));
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Failed to get transform: %s", ex.what());
                return;
            }
            
            tf2::Transform cameraPose, markerPose;
            tf2::fromMsg(cameraPoseMsg.transform, cameraPose);

            markerPose = cameraPose * tf;

            // Update marker or insert if it does not exist
            if (markers.count(ids[i])) {
                // Update marker transform
                // Only updates when the new object_error is less than the current one
                if (markers[ids[i]].error > object_error) {
                    try {
                        
                        markers[ids[i]].transform = markerPose;
                        markers[ids[i]].error = object_error;

                        publishMarker(markers[ids[i]]);

                    } catch (const tf2::TransformException &ex) {
                        RCLCPP_INFO(this->get_logger(), "Failed to get transform: %s", ex.what());
                    }
                }
            } else {
                FiducialMarker marker = {ids[i], markerPose, object_error};
                markers.insert({ids[i], marker});
                publishMarker(markers[ids[i]]);
            }
        }

        if (publish_images) {
	        image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch(cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
    }
}

void FiducialMapLocalisation::publishTransforms() {
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

void FiducialMapLocalisation::publishMarker(FiducialMarker &fid) {
    // Flattened cube
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    toMsg(fid.transform, marker.pose);

    marker.scale.x = fiducial_len;
    marker.scale.y = fiducial_len;
    marker.scale.z = fiducial_len / 10;
    marker.color.g = marker.color.a = 1.0f;
    marker.color.r = marker.color.b = 0.0f;
    marker.id = fid.id;
    marker.ns = "fiducial";
    marker.header.frame_id = "map";
    markerPub->publish(marker);

    // Text
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.header.frame_id = "map";
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.id = fid.id;
    text.scale.x = text.scale.y = text.scale.z = 0.1;
    text.pose.position.x = marker.pose.position.x;
    text.pose.position.y = marker.pose.position.y;
    text.pose.position.z = marker.pose.position.z;
    text.pose.position.z += (fiducial_len / 2.0) + 0.1;
    text.ns = "text";
    text.text = std::to_string(fid.id);
    markerPub->publish(text);
}

void FiducialMapLocalisation::saveMarkers(const fiducial_msgs::srv::SaveMarkers::Request::SharedPtr request,
                                         fiducial_msgs::srv::SaveMarkers::Response::SharedPtr response) {
    
    if (markers.size() <= 0) {
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

    markerFile << "id,x,y,z,rx,ry,rz,a" << std::endl;

    for (auto const& m : markers) {
        markerFile << m.second.id << ',';
        markerFile << m.second.transform.getOrigin().x() << ',' << m.second.transform.getOrigin().y() << ','
                    << m.second.transform.getOrigin().z() << ',';
        markerFile << m.second.transform.getRotation().getAxis().x() << ',' << m.second.transform.getRotation().getAxis().y() << ','
                    << m.second.transform.getRotation().getAxis().z() << ',' << m.second.transform.getRotation().getAngle();
        markerFile << std::endl;
    }

    markerFile.close();

    response->success = true;
}

FiducialMapLocalisation::FiducialMapLocalisation()
: rclcpp::Node("fiducial_map_localisation") {

    detectorParams = new cv::aruco::DetectorParameters();
    fiducial_len = this->declare_parameter("fiducial_len", 0.135);
    dicno = this->declare_parameter("dictionary", 7);
    publish_images = this->declare_parameter("publish_images", true);

    dictionary = cv::aruco::getPredefinedDictionary(dicno);

    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    tfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    rclcpp::QoS img_qos(1);
    img_sub = image_transport::create_subscription(this, "/camera/image_raw",
                           std::bind(&FiducialMapLocalisation::imageCallback, this, std::placeholders::_1),
                           "raw", img_qos.get_rmw_qos_profile());

    saveMarkersService = this->create_service<fiducial_msgs::srv::SaveMarkers>("/save_markers",
        std::bind(&FiducialMapLocalisation::saveMarkers, this, std::placeholders::_1, std::placeholders::_2));

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    caminfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 1,
                     std::bind(&FiducialMapLocalisation::camInfoCallback, this, std::placeholders::_1));
    image_pub = image_transport::create_publisher(this, "/marker_images", img_qos.get_rmw_qos_profile());
    markerPub = this->create_publisher<visualization_msgs::msg::Marker>("/markers", 100);

    timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / 1.0)),
        std::bind(&FiducialMapLocalisation::publishTransforms, this));

    /* adaptiveThreshConstant */
    auto adaptiveThreshConstantDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshConstantDescription.name = "Constant for adaptive thresholding before finding contours";
    auto adaptiveThreshConstantRange = rcl_interfaces::msg::FloatingPointRange{};
    adaptiveThreshConstantRange.from_value = 0;
    adaptiveThreshConstantRange.to_value = std::numeric_limits<double>::infinity();
    adaptiveThreshConstantDescription.floating_point_range = {adaptiveThreshConstantRange};
    detectorParams->adaptiveThreshConstant = this->declare_parameter("adaptiveThreshConstant", 7.0, adaptiveThreshConstantDescription);

    /* adaptiveThreshWinSizeMax */
    auto adaptiveThreshWinSizeMaxDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshWinSizeMaxDescription.name = "Maximum window size for adaptive thresholding before finding contours";
    auto adaptiveThreshWinSizeMaxRange = rcl_interfaces::msg::IntegerRange();
    adaptiveThreshWinSizeMaxRange.from_value = 1;
    adaptiveThreshWinSizeMaxRange.to_value = std::numeric_limits<int>::max();
    adaptiveThreshWinSizeMaxDescription.integer_range = {adaptiveThreshWinSizeMaxRange};
    detectorParams->adaptiveThreshWinSizeMax = this->declare_parameter("adaptiveThreshWinSizeMax", 53, adaptiveThreshWinSizeMaxDescription); /* default 23 */

    /* adaptiveThreshWinSizeMin */
    auto adaptiveThreshWinSizeMinDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshWinSizeMinDescription.name = "Minimum window size for adaptive thresholding before finding contours";
    auto adaptiveThreshWinSizeMinRange = rcl_interfaces::msg::IntegerRange();
    adaptiveThreshWinSizeMinRange.from_value = 1;
    adaptiveThreshWinSizeMinRange.to_value = std::numeric_limits<int>::max();
    adaptiveThreshWinSizeMinDescription.integer_range = {adaptiveThreshWinSizeMinRange};
    detectorParams->adaptiveThreshWinSizeMin = this->declare_parameter("adaptiveThreshWinSizeMin", 3, adaptiveThreshWinSizeMinDescription);

    /* adaptiveThreshWinSizeStep */
    auto adaptiveThreshWinSizeStepDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshWinSizeStepDescription.name = "Increments from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax during the thresholding";
    auto adaptiveThreshWinSizeStepRange = rcl_interfaces::msg::IntegerRange();
    adaptiveThreshWinSizeStepRange.from_value = 1;
    adaptiveThreshWinSizeStepRange.to_value = std::numeric_limits<int>::max();
    adaptiveThreshWinSizeStepDescription.integer_range = {adaptiveThreshWinSizeStepRange};
    detectorParams->adaptiveThreshWinSizeStep = this->declare_parameter("adaptiveThreshWinSizeStep", 4, adaptiveThreshWinSizeStepDescription); /* default 10 */

    /* cornerRefinementMaxIterations */
    auto cornerRefinementMaxIterationsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    cornerRefinementMaxIterationsDescription.name = "Maximum number of iterations for stop criteria of the corner refinement process";
    auto cornerRefinementMaxIterationsRange = rcl_interfaces::msg::IntegerRange();
    cornerRefinementMaxIterationsRange.from_value = 1;
    cornerRefinementMaxIterationsRange.to_value = std::numeric_limits<int>::max();
    cornerRefinementMaxIterationsDescription.integer_range = {cornerRefinementMaxIterationsRange};
    detectorParams->cornerRefinementMaxIterations = this->declare_parameter("cornerRefinementMaxIterations", 30, cornerRefinementMaxIterationsDescription);

    /* cornerRefinementMinAccuracy */
    auto cornerRefinementMinAccuracyDescription = rcl_interfaces::msg::ParameterDescriptor{};
    cornerRefinementMinAccuracyDescription.name = "Minimum error for the stop criteria of the corner refinement process";
    auto cornerRefinementMinAccuracyRange = rcl_interfaces::msg::FloatingPointRange();
    cornerRefinementMinAccuracyRange.from_value = 0.0;
    cornerRefinementMinAccuracyRange.to_value = 1.0;
    cornerRefinementMinAccuracyDescription.floating_point_range = {cornerRefinementMinAccuracyRange};
    detectorParams->cornerRefinementMinAccuracy = this->declare_parameter("cornerRefinementMinAccuracy", 0.01, cornerRefinementMinAccuracyDescription); /* default 0.1 */

    /* cornerRefinementWinSize */
    auto cornerRefinementWinSizeDescription = rcl_interfaces::msg::ParameterDescriptor{};
    cornerRefinementWinSizeDescription.name = "Window size for the corner refinement process (in pixels)";
    auto cornerRefinementWinSizeRange = rcl_interfaces::msg::IntegerRange();
    cornerRefinementWinSizeRange.from_value = 1;
    cornerRefinementWinSizeRange.to_value = std::numeric_limits<int>::max();
    cornerRefinementWinSizeDescription.integer_range = {cornerRefinementWinSizeRange};
    detectorParams->cornerRefinementWinSize = this->declare_parameter("cornerRefinementWinSize", 5, cornerRefinementWinSizeDescription);

#if CV_MINOR_VERSION==2 and CV_MAJOR_VERSION==3
    detectorParams->doCornerRefinement = this->declare_parameter("doCornerRefinement", true); /* default false */
#else
    bool doCornerRefinement = true;

    auto doCornerRefinementDescription = rcl_interfaces::msg::ParameterDescriptor{};
    doCornerRefinementDescription.name = "Whether to do subpixel corner refinement";
    doCornerRefinement = this->declare_parameter("doCornerRefinement", true, doCornerRefinementDescription);
    if (doCornerRefinement) {
       bool cornerRefinementSubpix = true;
       auto cornerRefinementSubpixDescription = rcl_interfaces::msg::ParameterDescriptor{};
       cornerRefinementSubpixDescription.name = "Whether to do subpixel corner refinement (true) or contour (false)";
       cornerRefinementSubpix = this->declare_parameter("cornerRefinementSubpix", true, cornerRefinementSubpixDescription);
       if (cornerRefinementSubpix) {
         detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif

    /* errorCorrectionRate */
    auto errorCorrectionRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    errorCorrectionRateDescription.name = "Error correction rate respect to the maximum error correction capability for each dictionary";
    auto errorCorrectionRateRange = rcl_interfaces::msg::FloatingPointRange();
    errorCorrectionRateRange.from_value = 0.0;
    errorCorrectionRateRange.to_value = 1.0;
    errorCorrectionRateDescription.floating_point_range = {errorCorrectionRateRange};
    detectorParams->errorCorrectionRate = this->declare_parameter("errorCorrectionRate", 0.6, errorCorrectionRateDescription);

    /* minCornerDistanceRate */
    auto minCornerDistanceRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minCornerDistanceRateDescription.name = "Minimum distance between corners for detected markers relative to its perimeter";
    auto minCornerDistanceRateRange = rcl_interfaces::msg::FloatingPointRange();
    minCornerDistanceRateRange.from_value = 0;
    minCornerDistanceRateRange.to_value = std::numeric_limits<double>::infinity();
    minCornerDistanceRateDescription.floating_point_range = {minCornerDistanceRateRange};
    detectorParams->minCornerDistanceRate = this->declare_parameter("minCornerDistanceRate", 0.05, minCornerDistanceRateDescription);

    /* markerBorderBits */
    auto markerBorderBitsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    markerBorderBitsDescription.name = "Number of bits of the marker border, i.e. marker border width";
    auto markerBorderBitsRange = rcl_interfaces::msg::IntegerRange();
    markerBorderBitsRange.from_value = 0;
    markerBorderBitsRange.to_value = std::numeric_limits<int>::max();
    markerBorderBitsDescription.integer_range = {markerBorderBitsRange};
    detectorParams->markerBorderBits = this->declare_parameter("markerBorderBits", 1, markerBorderBitsDescription);

    /* maxErroneousBitsInBorderRate */
    auto maxErroneousBitsInBorderRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    maxErroneousBitsInBorderRateDescription.name = "Maximum number of accepted erroneous bits in the border (i.e. number of allowed white bits in the border)";
    auto maxErroneousBitsInBorderRateRange = rcl_interfaces::msg::FloatingPointRange();
    maxErroneousBitsInBorderRateRange.from_value = 0.0;
    maxErroneousBitsInBorderRateRange.to_value = 1.0;
    maxErroneousBitsInBorderRateDescription.floating_point_range = {maxErroneousBitsInBorderRateRange};
    detectorParams->maxErroneousBitsInBorderRate = this->declare_parameter("maxErroneousBitsInBorderRate", 0.04, maxErroneousBitsInBorderRateDescription);

    /* minDistanceToBorder */
    auto minDistanceToBorderDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minDistanceToBorderDescription.name = "Minimum distance of any corner to the image border for detected markers (in pixels)";
    auto minDistanceToBorderRange = rcl_interfaces::msg::IntegerRange();
    minDistanceToBorderRange.from_value = 0;
    minDistanceToBorderRange.to_value = std::numeric_limits<int>::max();
    minDistanceToBorderDescription.integer_range = {minDistanceToBorderRange};
    detectorParams->minDistanceToBorder = this->declare_parameter("minDistanceToBorder", 3, minDistanceToBorderDescription);

    /* minMarkerDistanceRate */
    auto minMarkerDistanceRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minMarkerDistanceRateDescription.name = "Minimum mean distance beetween two marker corners to be considered similar, so that the smaller one is removed. The rate is relative to the smaller perimeter of the two markers";
    auto minMarkerDistanceRateRange = rcl_interfaces::msg::FloatingPointRange();
    minMarkerDistanceRateRange.from_value = 0.0;
    minMarkerDistanceRateRange.to_value = 1.0;
    minMarkerDistanceRateDescription.floating_point_range = {minMarkerDistanceRateRange};
    detectorParams->minMarkerDistanceRate = this->declare_parameter("minMarkerDistanceRate", 0.05, minMarkerDistanceRateDescription);

    /* minMarkerPerimeterRate */
    auto minMarkerPerimeterRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minMarkerPerimeterRateDescription.name = "Determine minumum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image";
    auto minMarkerPerimeterRateRange = rcl_interfaces::msg::FloatingPointRange();
    minMarkerPerimeterRateRange.from_value = 0.0;
    minMarkerPerimeterRateRange.to_value = 1.0;
    minMarkerPerimeterRateDescription.floating_point_range = {minMarkerPerimeterRateRange};
    detectorParams->minMarkerPerimeterRate = this->declare_parameter("minMarkerPerimeterRate", 0.1, minMarkerPerimeterRateDescription); /* default 0.3 */

    /* maxMarkerPerimeterRate */
    auto maxMarkerPerimeterRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    maxMarkerPerimeterRateDescription.name = "Determine maximum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image";
    auto maxMarkerPerimeterRateRange = rcl_interfaces::msg::IntegerRange();
    maxMarkerPerimeterRateRange.from_value = 0.0;
    maxMarkerPerimeterRateRange.to_value = 1.0;
    maxMarkerPerimeterRateDescription.integer_range = {maxMarkerPerimeterRateRange};
    detectorParams->maxMarkerPerimeterRate = this->declare_parameter("maxMarkerPerimeterRate", 4.0, maxMarkerPerimeterRateDescription);

    /* minOtsuStdDev */
    auto minOtsuStdDevDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minOtsuStdDevDescription.name = "Minimum standard deviation in pixels values during the decodification step to apply Otsu thresholding (otherwise, all the bits are set to 0 or 1 depending on mean higher than 128 or not)";
    auto minOtsuStdDevRange = rcl_interfaces::msg::FloatingPointRange();
    minOtsuStdDevRange.from_value = 0.0;
    minOtsuStdDevRange.to_value = std::numeric_limits<double>::infinity();
    minOtsuStdDevDescription.floating_point_range = {minOtsuStdDevRange};
    detectorParams->minOtsuStdDev = this->declare_parameter("minOtsuStdDev", 5.0, minOtsuStdDevDescription);

    /* perspectiveRemoveIgnoredMarginPerCell */
    auto perspectiveRemoveIgnoredMarginPerCellDescription = rcl_interfaces::msg::ParameterDescriptor{};
    perspectiveRemoveIgnoredMarginPerCellDescription.name = "Width of the margin of pixels on each cell not considered for the determination of the cell bit. Represents the rate respect to the total size of the cell, i.e. perpectiveRemovePixelPerCell";
    auto perspectiveRemoveIgnoredMarginPerCellRange = rcl_interfaces::msg::FloatingPointRange();
    perspectiveRemoveIgnoredMarginPerCellRange.from_value = 0.0;
    perspectiveRemoveIgnoredMarginPerCellRange.to_value = 1.0;
    perspectiveRemoveIgnoredMarginPerCellDescription.floating_point_range = {perspectiveRemoveIgnoredMarginPerCellRange};
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = this->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.13, perspectiveRemoveIgnoredMarginPerCellDescription);

    /* perspectiveRemovePixelPerCell */
    auto perspectiveRemovePixelPerCellDescription = rcl_interfaces::msg::ParameterDescriptor{};
    perspectiveRemovePixelPerCellDescription.name = "Number of bits (per dimension) for each cell of the marker when removing the perspective";
    auto perspectiveRemovePixelPerCellRange = rcl_interfaces::msg::IntegerRange();
    perspectiveRemovePixelPerCellRange.from_value = 1;
    perspectiveRemovePixelPerCellRange.to_value = std::numeric_limits<int>::max();
    perspectiveRemovePixelPerCellDescription.integer_range = {perspectiveRemovePixelPerCellRange};
    detectorParams->perspectiveRemovePixelPerCell = this->declare_parameter("perspectiveRemovePixelPerCell", 8, perspectiveRemovePixelPerCellDescription);

    /* polygonalApproxAccuracyRate */
    auto polygonalApproxAccuracyRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    polygonalApproxAccuracyRateDescription.name = "Width of the margin of pixels on each cell not considered for the determination of the cell bit. Represents the rate respect to the total size of the cell, i.e. perpectiveRemovePixelPerCell";
    auto polygonalApproxAccuracyRateRange = rcl_interfaces::msg::FloatingPointRange();
    polygonalApproxAccuracyRateRange.from_value = 0.0;
    polygonalApproxAccuracyRateRange.to_value = 1.0;
    polygonalApproxAccuracyRateDescription.floating_point_range = {polygonalApproxAccuracyRateRange};
    detectorParams->polygonalApproxAccuracyRate = this->declare_parameter("polygonalApproxAccuracyRate", 0.01, polygonalApproxAccuracyRateDescription); /* default 0.05 */

    RCLCPP_INFO(this->get_logger(), "Initialization successful");
}

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<FiducialMapLocalisation>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
