//
// Created by charm on 2/21/17.
//

#ifndef TELEOP_VISION_ACOVERLAY_H
#define TELEOP_VISION_ACOVERLAY_H

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
#include <sensor_msgs/Joy.h>

// service
#include "teleop_vision/CalculateStereoCamsTransfromFromTopics.h"

// including Drawings.h just for the CameraIntrinsics type. fix.
#include "Drawings.h"


class ACOverlay {

private:

    // Reads parameters and sets up subscribers and publishers
    void SetupROS();

    // reads the intrinsic camera parameters
    void ReadCameraParameters(const std::string file_path,
                              CameraIntrinsics &camera_intrins);

public:

    ACOverlay(std::string node_name, int width, int height);

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
    //    void ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg);
    void ACPoseDesiredLeftCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void ACPoseDesiredRightCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // foot switch used to select the ac path
    void FootSwitchCallback(const sensor_msgs::Joy &msg);

    // Locking call to retrieve the images
    cv::Mat &ImageLeft(ros::Duration timeout = ros::Duration(1));

    cv::Mat &ImageRight(ros::Duration timeout = ros::Duration(1));

    // Publisher for the ac path in use
    ros::Publisher publisher_ac_path;

    //overlay image publishers
    image_transport::Publisher publisher_overlayed[2];


public:
    // IN ALL CODE 0 is Left, 1 is Right
    // ----------------------------------

    ros::NodeHandle n;
    std::vector<cv::Point3d> ac_path;
    bool foot_switch_pressed = false;

    cv::Mat image_msg;
    CameraIntrinsics cam_intrinsics[2];
    KDL::Frame pose_cam_l;
    KDL::Frame pose_cam_r;
    KDL::Frame pose_tool1;
    KDL::Frame pose_tool2;
    KDL::Frame taskspace_to_psm1_tr;
    KDL::Frame taskspace_to_psm2_tr;
    KDL::Frame left_cam_to_right_cam_tr;

    KDL::Frame pose_desired[2];

    cv::Vec3d cam_rvec[2];
    cv::Vec3d cam_tvec[2];
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

    image_transport::Subscriber subscriber_image_left;
    image_transport::Subscriber subscriber_image_right;
    ros::Subscriber subscriber_camera_pose_left;
    ros::Subscriber subscriber_camera_pose_right;

    ros::Subscriber subscriber_pose_psm1;
    ros::Subscriber subscriber_pose__sub;
    ros::Subscriber subscriber_ac_pose_desired_right;
    ros::Subscriber subscriber_ac_pose_desired_left;

    //ros::Subscriber subscriber_ac_path;
    ros::Subscriber subscriber_foot_pedal_clutch;


};

#endif //TELEOP_VISION_ACOVERLAY_H


namespace VisualUtils {

    void SwitchFullScreen(const std::string window_name);

}

namespace SimpleACs {

    void GenerateXYCircle(const KDL::Vector center, const double radius, const int num_points,
                          std::vector<cv::Point3d> &ac_path);
}

namespace MultiplePathsTask {
    enum class Status {
        Ready, ACSelected, Finished
    };


    size_t FindClosestTarget(const KDL::Vector tool_current_position,
                             const std::vector<cv::Point3d> targets);

    void GeneratePathPoints(const KDL::Vector tool_current_position,
                            const cv::Point3d targets, std::vector<cv::Point3d> &ac_path);

    void DrawAllPaths(cv::InputOutputArray image,
                      const CameraIntrinsics &cam_intrinsics,
                      const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                      const KDL::Vector &tooltip_pos,
                      std::vector<cv::Point3d> targets,
                      const size_t &selected_index,
                      const cv::Scalar &color_selected,
                      const cv::Scalar &color_others);
}

void  VecPoint3dToPoseArray(std::vector<cv::Point3d> vec, geometry_msgs::PoseArray & out) ;