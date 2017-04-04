//
// Created by charm on 2/21/17.
//

#ifndef TELEOP_VISION_OVERLAYGRAPHICS_H
#define TELEOP_VISION_OVERLAYGRAPHICS_H

//#include <GLFW/glfw3.h>
//#include <GL/glu.h>
#include "ros/ros.h"

#include "opencv2/highgui/highgui.hpp"
#include <kdl_conversions/kdl_msg.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>

// service
#include "teleop_vision/CalculateStereoCamsTransfromFromTopics.h"

#include "utils/Drawings.h"


class OverlayGraphics {

private:

    // Reads parameters and sets up subscribers and publishers
    void SetupROS();

    // reads the intrinsic camera parameters
    void ReadCameraParameters(const std::string file_path,
                              CameraIntrinsics & camera_intrins);

    void PublishOverlayImpl(image_transport::Publisher & pub, const cv::Mat & img);

public:

    OverlayGraphics(std::string node_name, int width, int height);

    // CALLBACKS
    void ImageLeftCallback(const sensor_msgs::ImageConstPtr &msg);
    void ImageRightCallback(const sensor_msgs::ImageConstPtr &msg);

    // The camera poses. Note that this actually defines the pose of the
    // task coordinate frame in camera coordinate frame
    void LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void RightCamPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // Tool poses in task coordinate frame (task-space).
    void Tool1PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void Tool2PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // Receives ac path from the ac geometry node as an array of poses.
    // Currently only the positions are used.
    void ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg);
    void ACPoseDesiredLeftCallback(const geometry_msgs::PoseStampedConstPtr & msg);
    void ACPoseDesiredRightCallback(const geometry_msgs::PoseStampedConstPtr & msg);

    // Locking call to retrieve the images
    cv::Mat& ImageLeft(ros::Duration timeout = ros::Duration(1));
    cv::Mat& ImageRight(ros::Duration timeout = ros::Duration(1));

    void PublishOverlayRight(const cv::Mat & img);
    void PublishOverlayLeft(const cv::Mat & img);

    const bool IsROSOverlayEnabled() const { return use_ros_overlay; }

public:

    ros::NodeHandle n;
    std::vector<cv::Point3d> ac_path;

    cv::Mat image_msg;
    CameraIntrinsics cam_intrinsics[2];
    KDL::Frame pose_cam_l;
    KDL::Frame pose_cam_r;
    KDL::Frame pose_tool1;
    KDL::Frame pose_tool2;
    KDL::Frame taskspace_to_psm1_tr;
    KDL::Frame taskspace_to_psm2_tr;
    KDL::Frame left_cam_to_right_cam_tr;

    KDL::Frame pose_desired_l;
    KDL::Frame pose_desired_r;

    cv::Vec3d cam_rvec_l, cam_tvec_l;
    cv::Vec3d cam_rvec_r, cam_tvec_r;
    cv::Mat image_left_;
    cv::Mat image_right_;
    bool new_right_image = false;
    bool new_left_image = false;
    ros::ServiceClient stereo_tr_calc_client;
    teleop_vision::CalculateStereoCamsTransfromFromTopics stereo_tr_srv;

private:
    int image_width_;
    int image_height_;
    int num_cam_pose_publishers;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_subscribers[2];

    bool use_ros_overlay = false;
    image_transport::Publisher overlay_image_left;
    image_transport::Publisher overlay_image_right;

    ros::Subscriber camera_pose_subscriber_left;
    ros::Subscriber camera_pose_subscriber_right;

    ros::Subscriber subscriber_pose_psm1;
    ros::Subscriber subscriber_pose__sub;

    ros::Subscriber subscriber_ac_path;
    ros::Subscriber ac_pose_desired_right_subscriber;
    ros::Subscriber ac_pose_desired_left_subscriber;
};

#endif //TELEOP_VISION_OVERLAYGRAPHICS_H


namespace VisualUtils{

    void SwitchFullScreen(const std::string window_name);

}

