//
// Created by nima on 01/06/17.
//

#include <ros/ros.h>
#include "ArmToWorldCalibration.h"
#include <pwd.h>
#include <opencv2/core/persistence.hpp>

void ReadCamParams(std::string cam_name, cv::Mat &cam_mat, cv::Mat &dist_mat);

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "arm_to_world_calibration");
    ros::NodeHandle n(ros::this_node::getName());

    // ------- load the intrinsic calibration files
    cv::Mat cam_mat, dist_mat;
    std::string cam_name;
    if (n.getParam("cam_name", cam_name)) {
        ReadCamParams(cam_name, cam_mat, dist_mat);
    } else
        ROS_ERROR("Parameter '%s' is required. Place the intrinsic calibration "
                          "file of each camera in ~/.ros/camera_info/ named "
                          "as  <cam_name>_intrinsics.xml",
                  n.resolveName("cam_name").c_str());


    // a topic name is required for the images
    std::string cam_name_space;
    n.param<std::string>("camera_img_topic", cam_name_space,
                         "/camera/image_raw");

    std::stringstream cam_pose_namespace;
    cam_pose_namespace << std::string("/") << cam_name
               << "/world_to_camera_transform";

    std::string arm_pose_namespace;
    n.param<std::string>("arm_pose_namespace", arm_pose_namespace,
                         "/dvrk/PSM1/");

    // putting the calibration point on the corners of the board squares
    // the parameter can be set directly, unless there is the global
    // /calibrations/board_params
    double calib_points_distance = 0.01;
    std::vector<float> board_params = std::vector<float>(5, 0.0);

    if(!n.getParam("calib_points_distance", calib_points_distance)){
        if(n.getParam("/calibrations/board_params", board_params))
            calib_points_distance = board_params[3];

    };

    int num_calib_points;
    n.param<int>("number_of_calibration_points", num_calib_points, 6);


    KDL::Frame arm_to_world_frame;
    ArmToWorldCalibration AWC;
    AWC.DoCalibration(cam_name_space,
                      cam_pose_namespace.str(),
                      arm_pose_namespace,
                      cam_mat,
                      dist_mat,
                      num_calib_points,
                      calib_points_distance,
                      arm_to_world_frame);


}


void ReadCamParams(std::string cam_name, cv::Mat &cam_mat, cv::Mat &dist_mat){

    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;
    std::stringstream path;
    path << std::string(home_dir) << "/.ros/camera_info/" << cam_name << "_intrinsics.yaml";


    cv::FileStorage fs(path.str(), cv::FileStorage::READ);
    ROS_INFO("Reading camera intrinsic data from: '%s'", path.str().c_str());
    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> cam_mat;
    fs["distortion_coefficients"] >> dist_mat;

    // check if we got osomething
    if(cam_mat.empty()){
        ROS_ERROR("distortion_coefficients was not found in '%s' ",
                  path.str ().c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(dist_mat.empty()){
        ROS_ERROR("camera_matrix was not found in '%s' ", path.str().c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }

}
