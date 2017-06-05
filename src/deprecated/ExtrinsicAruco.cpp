/*
 * aruco_extrinsic.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: nearlab
 */


#include "ExtrinsicAruco.hpp"
#include "utils/Conversions.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pwd.h>

using namespace std;

ArucoExtrinsic::ArucoExtrinsic(string node_name)
    : n(node_name)
{
    it = new image_transport::ImageTransport(n);
    GetROSParameterValues();

}



//-----------------------------------------------------------------------------------
// SetupROS
//-----------------------------------------------------------------------------------

void ArucoExtrinsic::GetROSParameterValues() {
    bool all_required_params_found = true;

    n.param<bool>("show_image", show_image, false);
    board.draw_axes = show_image;

    // ------- load the intrinsic calibration files
    // get home directory
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;

    std::string cam_name;
    if (n.getParam("cam_name", cam_name)) {
        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/") << cam_name << "_intrinsics.xml";
        ReadCameraParameters(path.str(), cam_intrinsics);
    } else {
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.xml",
                n.resolveName("cam_name").c_str());
        all_required_params_found = false;
    }



    std::string image_transport_namespace;
    if (n.getParam("image_transport_namespace", image_transport_namespace)) {

        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<sensor_msgs::Image>( image_transport_namespace, ros::Duration(5))) {
            ROS_ERROR("Topic '%s' is not publishing.",
                      image_transport_namespace.c_str());
            all_required_params_found = false;
        }
        else
            ROS_INFO("[SUBSCRIBERS] Images will be read from topic %s",
                     image_transport_namespace.c_str());


    } else {
        ROS_ERROR("%s Parameter '%s' is required.",
                  ros::this_node::getName().c_str(), n.resolveName("image_transport_namespace").c_str());
        all_required_params_found = false;
    }

    // register image transport subscriber
    sub = it->subscribe(
            image_transport_namespace, 1, &ArucoExtrinsic::CameraImageCallback, this);



    // Load the description of the aruco board from the parameters
    if (!n.getParam("aruco_board_w", board.Width)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_board_w").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_board_h", board.Height)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_board_h").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_marker_length_in_meters", board.MarkerLength)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_marker_length_in_meters").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_marker_separation_in_meters", board.MarkerSeparation)){
    	ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_marker_separation_in_meters").c_str());
    	all_required_params_found = false;
    }
    if (!n.getParam("aruco_dictionary_id", board.DictionaryID)){
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
	ROS_INFO("Publishing board to camera pose on '%s'",
     n.resolveName(board_to_cam_pose_topic_name).c_str());

}

void ArucoExtrinsic::ReadCameraParameters(const std::string file_path,
                                           CameraIntrinsics & camera) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> camera.camMatrix;
    fs["distortion_coefficients"] >> camera.distCoeffs;

    // check if we got osomething
    if(camera.distCoeffs.empty()){
        ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(camera.camMatrix.empty()){
        ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }


}

void ArucoExtrinsic::CameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try
  {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;
      new_image = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat& ArucoExtrinsic::Image(ros::Duration timeout) {
    ros::Rate loop_rate(1);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout whilst waiting for a new image from the image topic. "
                "Is the camera still publishing ?");
        }
    }

    return image;
}





