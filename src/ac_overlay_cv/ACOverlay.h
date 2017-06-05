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
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

// service
//#include "atar/CalculateStereoCamsTransfromFromTopics.h"
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

    // Tool poses in task coordinate frame (taskspace).
    void Tool1PoseCurrentCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Tool2PoseCurrentCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
//    void Tool1TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
//    void Tool2TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Receives ac path from the ac geometry node as an array of poses.
    // Currently only the positions are used.
    //    void ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg);


    // foot switch used to select the ac path
    void FootSwitchCallback(const sensor_msgs::Joy &msg);

    // Locking call to retrieve the images
    cv::Mat &ImageLeft(ros::Duration timeout = ros::Duration(1));

    cv::Mat &ImageRight(ros::Duration timeout = ros::Duration(1));

    // Publisher for the ac path in use
    ros::Publisher publisher_ac_path;

    //overlay image publishers
    image_transport::Publisher publisher_overlayed[2];
    image_transport::Publisher publisher_stereo_overlayed;

    // converts the desired poses to geometry pose messages and published them
    void PublishDesiredPose();


public:
    // IN ALL CODE 0 is Left Cam, 1 is Right cam
    // ----------------------------------

    ros::NodeHandle n;
    double desired_pose_update_freq;
    std::vector<cv::Point3d> ac_path;
    bool foot_switch_pressed = false;

    cv::Mat image_msg;
    CameraIntrinsics cam_intrinsics[2];
    KDL::Frame pose_cam[2];
    KDL::Frame pose_current_tool[2];
//    KDL::Twist twist_tool_current[2];

    KDL::Frame slave_frame_to_task_frame[2];

    KDL::Frame left_cam_to_right_cam_tr;

    KDL::Frame pose_desired_tool[2];

    // used for the ring task
    KDL::Vector ac_path_tangent_current[2];

    cv::Vec3d cam_rvec[2];
    cv::Vec3d cam_tvec[2];
    cv::Mat image_left;
    cv::Mat image_right;
    bool new_right_image = false;
    bool new_left_image = false;
    ros::ServiceClient stereo_tr_calc_client;
    //atar::CalculateStereoCamsTransfromFromTopics stereo_tr_srv;

private:

    int n_arms;

    int image_width;
    int image_height;
    int num_cam_pose_publishers;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_subscribers[2];


    image_transport::Subscriber subscriber_image_left;
    image_transport::Subscriber subscriber_image_right;
    ros::Subscriber subscriber_camera_pose_left;
    ros::Subscriber subscriber_camera_pose_right;


    ros::Subscriber *subscriber_tool_current_pose;
    ros::Publisher *publisher_tool_pose_desired;
    //    // Current twists Not used for now
    //    ros::Subscriber *subscriber_twist_current_tool;
    //    ros::Publisher *publisher_twist_current_tool;

    // two function pointers for slave pose callbacks
    void (ACOverlay::*pose_current_tool_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave twist callbacks
    void (ACOverlay::*twist_current_tool_callbacks[2])
            (const geometry_msgs::TwistStamped::ConstPtr &msg);

    //ros::Subscriber subscriber_ac_path;
    ros::Subscriber subscriber_foot_pedal_clutch;


};

#endif //TELEOP_VISION_ACOVERLAY_H


namespace VisualUtils {

    void SwitchFullScreen(const std::string window_name);

}

namespace CricleACTask {

    void GenerateXYCircle(const KDL::Vector center, const double radius, const int num_points,
                          std::vector<cv::Point3d> &ac_path);

}

namespace MultiplePathsTask {
    enum class Status {
        Ready, ACSelected, SubTaskFinished, TaskFinished
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

namespace RingTask{

    // in the ring task we want the ring's normal vector to be in along the direction of the path normal
    // this leaves the rotation around the normal unconstrained. Also the direction of the tangent is
    // not important, so we find the smallest rotation that takes the ring's normal vector from its
    // current one to that of the path's tangent vector
    // Assuming that the normal to the ring is the x vector of current_orientation.
    KDL::Rotation CalculateDesiredOrientation(const KDL::Vector ac_path_tangent_current,
                                              const KDL::Rotation current_orientation);
}

// other functions to be sorted later
void  VecPoint3dToPoseArray(std::vector<cv::Point3d> vec, geometry_msgs::PoseArray & out) ;

void ClosestPointOnACPathAndItsTangent(const KDL::Vector tool_current_position,
                                       const std::vector<cv::Point3d> &ac_path,
                                       KDL::Vector &tool_desired_position,
                                       KDL::Vector &tangent_vector);
