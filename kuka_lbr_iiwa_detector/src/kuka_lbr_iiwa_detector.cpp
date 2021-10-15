#include "kuka_lbr_iiwa_detector/kuka_lbr_iiwa_detector.h"

MarkerDetector::MarkerDetector()
: m_cameraParamsFile { "/config/head_camera.yaml" },
  m_detectorParamsFile { "/config/detector_parameters.yaml" },
  m_package_path { ros::package::getPath("kuka_lbr_iiwa_detector") },
  m_cameraTopicName { "/lbr_iiwa_14_r820/ee_camera/image_raw" },
  m_imageTopicName { "/lbr_iiwa_14_r820/ee_camera/image_with_markers" },
  m_markersTopicName { "/detected_markers" },
  m_dict { cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100) }
{
    //Topics to publish
    m_pub_pose = m_nh.advertise<geometry_msgs::Pose>(m_markersTopicName, 1);
    m_pub_image = m_nh.advertise<sensor_msgs::Image>(m_imageTopicName, 1);

    //Topic to subscribe
    readDetectorParams(m_package_path + m_detectorParamsFile);
    readCameraParams(m_package_path + m_cameraParamsFile);

    m_sub = m_nh.subscribe(m_cameraTopicName, 1, &MarkerDetector::callback, this);
}

MarkerDetector::~MarkerDetector() {}

void MarkerDetector::readCameraParams(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["camera_matrix"] >> m_cameraMatrix;
    fs["distortion_coefficients"] >> m_distCoeffs;
}

void MarkerDetector::readDetectorParams(const std::string &filename)
{
    m_detectorParams = cv::aruco::DetectorParameters::create();
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["adaptiveThreshWinSizeMin"] >> m_detectorParams->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> m_detectorParams->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> m_detectorParams->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> m_detectorParams->adaptiveThreshConstant;    
    fs["minMarkerPerimeterRate"] >> m_detectorParams->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> m_detectorParams->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> m_detectorParams->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> m_detectorParams->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> m_detectorParams->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> m_detectorParams->minMarkerDistanceRate;    
    fs["cornerRefinementWinSize"] >> m_detectorParams->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> m_detectorParams->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> m_detectorParams->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> m_detectorParams->markerBorderBits;

    fs["perspectiveRemovePixelPerCell"] >> m_detectorParams->perspectiveRemovePixelPerCell;    
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> m_detectorParams->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> m_detectorParams->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> m_detectorParams->minOtsuStdDev;
    fs["errorCorrectionRate"] >> m_detectorParams->errorCorrectionRate;

    m_detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

geometry_msgs::Quaternion MarkerDetector::rotMatToQuat(const cv::Mat &rot) const
{
    tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                           rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                           rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

    tf2::Transform tf2_transform(tf2_rot, tf2::Vector3());
    geometry_msgs::Pose pose_msg;
    tf2::toMsg(tf2_transform, pose_msg);
    return pose_msg.orientation;
}

void MarkerDetector::callback(const sensor_msgs::Image::ConstPtr &img) const
{
    geometry_msgs::Pose marker_pose;
    geometry_msgs::Point marker_point;
    geometry_msgs::Quaternion marker_quat;

    sensor_msgs::Image image_msg;

    auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

    cv::Mat rotationMatrix;

    cv::Mat imageCopy;
    (cv_ptr->image).copyTo(imageCopy);
  
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::aruco::detectMarkers(imageCopy, 
                              m_dict, 
                              markerCorners, 
                              markerIds, 
                              m_detectorParams);

    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, 
                                         m_cameraMatrix, m_distCoeffs, 
                                         rvecs, tvecs);

    // Draw axis for each marker
    for (auto i { 0 }; i < markerIds.size(); i++)
    {
        cv::aruco::drawAxis(imageCopy, m_cameraMatrix, m_distCoeffs, 
                            rvecs[i], tvecs[i], 0.05);
    }

    // Take first marker
    if (!markerIds.empty())
    {
        marker_point.x = tvecs[0][0];
        marker_point.y = tvecs[0][1];
        marker_point.z = tvecs[0][2];

        cv::Rodrigues(rvecs[0], rotationMatrix);

        marker_pose.position = marker_point;
        marker_pose.orientation = rotMatToQuat(rotationMatrix);

        m_pub_pose.publish(marker_pose);
    }

    cv_ptr->image = imageCopy;
    (*cv_ptr).toImageMsg(image_msg);
    m_pub_image.publish(image_msg);
} 

