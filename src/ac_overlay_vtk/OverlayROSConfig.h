//
// Created by nima on 4/17/17.
//

#ifndef TELEOP_VISION_OVERLAYROSCONFIG_H
#define TELEOP_VISION_OVERLAYROSCONFIG_H

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
#include <std_msgs/Char.h>

//#include "utils/Drawings.h"
#include "active_constraints/ActiveConstraintParameters.h"
#include "teleop_vision/TaskState.h"
#include "BuzzWireTask.h"


class OverlayROSConfig {


public:

    OverlayROSConfig(std::string node_name, int width, int height);

    // Locking call to retrieve the images
    void LockAndGetImages(ros::Duration timeout, cv::Mat images[]);

    // return true if both images are newly received and copy them in imgs
    bool GetNewImages(cv::Mat imgs[]) ;

    //overlay image publishers
    image_transport::Publisher publisher_overlayed[2];
    image_transport::Publisher publisher_stereo_overlayed;

    // publishes the active constraint parameters
    void PublishACtiveConstraintParameters(
            const active_constraints::ActiveConstraintParameters &);

    // publishes the active constraint parameters
    void PublishTaskState(teleop_vision::TaskState msg);

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

    // foot switch used to select the ac path
    void FootSwitchCallback(const sensor_msgs::Joy &msg);

    // this topic is used to control the task state from the recording node
    // during the acquisitions.
    void RecordingEventsCallback(const std_msgs::CharConstPtr &msg);


private:

    // Reads parameters and sets up subscribers and publishers
    void SetupROS();

    // reads the intrinsic camera parameters
    void ReadCameraParameters(const std::string file_path,
                              cv::Mat &camera_matrix,
                              cv::Mat &camera_distortion);

public:
    // IN ALL CODE 0 is Left Cam, 1 is Right cam
    // ----------------------------------

    ros::NodeHandle n;
    double desired_pose_update_freq;
    std::vector<cv::Point3d> ac_path;
    bool foot_switch_pressed = false;
    bool with_guidance;
    cv::Mat camera_matrix[2];
    cv::Mat camera_distortion[2];
    KDL::Frame pose_cam[2];
    KDL::Frame pose_current_tool[2];

    KDL::Frame slave_frame_to_task_frame[2];

    KDL::Frame left_cam_to_right_cam_tr;

    cv::Vec3d cam_rvec[2];
    cv::Vec3d cam_tvec[2];

    bool new_right_image = false;
    bool new_left_image = false;

    BuzzWireTask *buzz_task;

private:

    int n_arms;
    cv::Mat image_from_ros[2];

    char recording_event;
    bool new_recording_event;
    bool show_reference_frames;

    int image_width;
    int image_height;
    int num_cam_pose_publishers;

    image_transport::ImageTransport *it;
    image_transport::Subscriber image_subscribers[2];

    image_transport::Subscriber subscriber_image_left;
    image_transport::Subscriber subscriber_image_right;
    ros::Subscriber subscriber_camera_pose_left;
    ros::Subscriber subscriber_camera_pose_right;
    ros::Subscriber subscriber_foot_pedal_clutch;
    ros::Subscriber subscriber_recording_events;

    ros::Subscriber * subscriber_tool_current_pose;
    ros::Publisher * publisher_tool_pose_desired;
    ros::Publisher * publisher_ac_params;
    ros::Publisher publisher_task_state;

    // two function pointers for slave pose callbacks
    void (OverlayROSConfig::*pose_current_tool_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);


};


namespace VisualUtils {

    void SwitchFullScreen(const std::string window_name);

}

#endif //TELEOP_VISION_OVERLAYROSCONFIG_H
