//
// Created by nima on 02/11/17.
//

#include "AugmentedCamera.h"
#include <pwd.h>
#include <custom_conversions/Conversions.h>

AugmentedCamera::AugmentedCamera(ros::NodeHandlePtr n, image_transport::ImageTransport *it,
                   const std::string cam_name, const std::string ns) {

    // AR camera
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;
    std::stringstream path;
    path << std::string(home_dir) << std::string("/.ros/camera_info/")
         << cam_name << "_intrinsics.yaml";
    ReadCameraParameters(path.str());

    // --------------------Images
    // image subscriber
    img_topic = "/"+cam_name+ "/image_raw";
    if(ns!="")
        img_topic = "/"+ns+"/"+cam_name+ "/image_raw";;

    sub_image = it->subscribe(img_topic, 1, &AugmentedCamera::ImageCallback, this);


    // ------------------ CAM POSE
    // we first try to read the poses as parameters and later update the
    // poses if new messages are arrived on the topics
    std::vector<double> temp_vec = std::vector<double>( 7, 0.0);
    n->getParam("/calibrations/world_frame_to_"+cam_name+"_frame", temp_vec);

    // now we set up the subscribers
    sub_pose = n->subscribe("/"+cam_name+ "/world_to_camera_transform",
                            1, &AugmentedCamera::PoseCallback, this);
}

//------------------------------------------------------------------------------
void AugmentedCamera::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_from_ros = cv_bridge::toCvCopy(msg, "rgb8")->image;
        new_image= true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

//------------------------------------------------------------------------------
void AugmentedCamera::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    new_cam_pose = true;
    tf::poseMsgToKDL(msg->pose, world_to_cam_pose);
}

void
AugmentedCamera::GetIntrinsicParams(double &fx, double &fy, double &cx, double &cy) {

    fx = camera_matrix.at<double>(0, 0);
    fy = camera_matrix.at<double>(1, 1);
    cx = camera_matrix.at<double>(0, 2);
    cy = camera_matrix.at<double>(1, 2);
}
//------------------------------------------------------------------------------
bool AugmentedCamera::IsPoseNew() {
    if(new_cam_pose) {
        new_cam_pose = false;
        return true;
    }
    return false;
}
//------------------------------------------------------------------------------
void AugmentedCamera::ReadCameraParameters(const std::string file_path) {

    cv::FileStorage fs(file_path, cv::FileStorage::READ);

    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");
    fs["camera_matrix"] >> camera_matrix;

    // check if we got something
    if(camera_matrix.empty()){
        ROS_ERROR("camera_matrix not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }

    fs["distortion_coefficients"] >> camera_distortion;
    if(camera_distortion.empty()){
        ROS_ERROR("distortion_coefficients  not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
}

//------------------------------------------------------------------------------
void AugmentedCamera::LockAndGetImage(cv::Mat &image) {
    ros::Rate loop_rate(2);
    ros::Time timeout_time = ros::Time::now() + ros::Duration(1);

    while(ros::ok() && image_from_ros.empty()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (ros::Time::now() > timeout_time)
            ROS_WARN_STREAM(("Timeout: No Image on."+img_topic+
                             " Trying again...").c_str());
    }
    image_from_ros.copyTo(image);

    new_image = false;
}

bool AugmentedCamera::IsImageNew() {
    if(new_image) {
        new_image = false;
        return true;
    }
    return false;
}
