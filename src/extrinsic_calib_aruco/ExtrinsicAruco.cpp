/*
 * aruco_extrinsic.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: nearlab
 */



#include "utils/Conversions.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ExtrinsicAruco.hpp>
#include <image_transport/image_transport.h>

using namespace std;

ArucoExtrinsic::ArucoExtrinsic(string node_name)
    : n(node_name)
{
    it = new image_transport::ImageTransport(n);
    GetROSParameterValues();

}



//-----------------------------------------------------------------------------------
// GetROSParameterValues
//-----------------------------------------------------------------------------------

void ArucoExtrinsic::GetROSParameterValues() {
    bool all_required_params_found = true;

    n.param<double>("frequency", ros_freq, 25);

    // load the intrinsic calibration file
    std::string cam_intrinsic_calibration_file_path;
    if (n.getParam("cam_intrinsic_calibration_file_path", cam_intrinsic_calibration_file_path)) {
        ReadCameraParameters(cam_intrinsic_calibration_file_path);
    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("cam_intrinsic_calibration_file_path").c_str());
    }




    std::string image_transport_namespace;
    if (n.getParam("image_transport_namespace", image_transport_namespace)) {

        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<sensor_msgs::Image>( image_transport_namespace, ros::Duration(5))) {
            ROS_ERROR("Topic '%s' is not publishing.", image_transport_namespace.c_str());
            all_required_params_found = false;
        }
        else
            ROS_INFO("Reading camera images from transport '%s'", image_transport_namespace.c_str());

    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("image_transport_namespace").c_str());
        all_required_params_found = false;
    }

    // register image transport subscriber
    sub = it->subscribe(
            image_transport_namespace, 1, &ArucoExtrinsic::CameraImageCallback, this);



    // Load the description of the aruco board from the parameters
    if (!n.getParam("aruco_board_w", Board.Width)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_board_w").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_board_h", Board.Height)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_board_h").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_marker_length_in_meters", Board.MarkerLength)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_marker_length_in_meters").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_marker_separation_in_meters", Board.MarkerSeparation)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_marker_separation_in_meters").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_dictionary_id", Board.DictionaryID)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_dictionary_id").c_str());
    	all_required_params_found = false;
    }


    if (!all_required_params_found)
    	throw std::runtime_error("ERROR: some required topics are not set");



    // advertise publishers
    std::string board_to_cam_pose_topic_name;
	if (!n.getParam("board_to_cam_pose_topic_name", board_to_cam_pose_topic_name))
		board_to_cam_pose_topic_name = "board_to_camera";

    pub_board_to_cam_pose = n.advertise<geometry_msgs::PoseStamped>(board_to_cam_pose_topic_name, 1, 0);
	ROS_INFO("Publishing board to camera pose on '%s'", n.resolveName(board_to_cam_pose_topic_name).c_str());

}

void ArucoExtrinsic::ReadCameraParameters(std::string file_path) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
	ROS_INFO_STREAM("Reading camera intrinsic data from: " << file_path);

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> Camera.camMatrix;
    fs["distortion_coefficients"] >> Camera.distCoeffs;

    // check if we got osomething
    if(Camera.distCoeffs.empty()){
    	ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
    	throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(Camera.camMatrix.empty()){
    	ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
    	throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }


}



void ArucoExtrinsic::CameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try
  {
    image_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}



cv::Mat& ArucoExtrinsic::Image(ros::Duration timeout) {
    ros::Rate loop_rate(1);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_msg.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout whilst waiting for a new image from the image topic. "
                "Is the camera still publishing ?");
        }
    }

    return image_msg;
}





