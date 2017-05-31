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
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include <boost/thread/thread.hpp>
#include "VTKTask.h"

class OverlayROSConfig {


public:

    OverlayROSConfig(std::string node_name);

    // Locking call to retrieve the images
    void LockAndGetImages(ros::Duration timeout, cv::Mat images[]);

    // return true if both images are newly received and copy them in imgs
    bool GetNewImages(cv::Mat imgs[]);

    // returns the poses of the cameras. Intended to be used once at
    // initializations to load the poses provided as parameters. The poses
    // can be updated and accessed by the GetNewCameraPoses method in the loop
    void GetCameraPoses(cv::Vec3d cam_rvec[2], cv::Vec3d cam_tvec[2]);

    // If the poses of the cameras are published, this method will return
    // true when any of the cam poses are updated. If left or right pose is
    // missing it will be found transforming the other available pose with the
    // left_cam_to_right_cam_tr. This transformation is calculated once in a
    // locking call in this method.
    bool GetNewCameraPoses(cv::Vec3d cam_rvec[2], cv::Vec3d cam_tvec[2]);

    //overlay image publishers
    image_transport::Publisher publisher_overlayed[2];
    image_transport::Publisher publisher_stereo_overlayed;

    // publishes the active constraint parameters
    void PublishACtiveConstraintParameters(
            const custom_msgs::ActiveConstraintParameters &);

    // publishes the active constraint parameters
    void PublishTaskState(custom_msgs::TaskState msg);

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

    // stop the running haptic thread (if any), destruct the previous task
    // (if any) and start a new task and thread.
    void StartTask(const uint task_id);

    // stop the running haptic thread and destruct the  task object
    void StopTask();

private:

    // Reads parameters and sets up subscribers and publishers
    void SetupROS();

    // reads the intrinsic camera parameters
    void ReadCameraParameters(const std::string file_path,
                              cv::Mat &camera_matrix,
                              cv::Mat &camera_distortion);

    boost::thread haptics_thread;

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
    bool cam_poses_provided_as_params;
    KDL::Frame slave_frame_to_task_frame[2];

    KDL::Frame left_cam_to_right_cam_tr;

    cv::Vec3d cam_rvec[2];
    cv::Vec3d cam_tvec[2];

    bool new_image[2] = {false, false};
    bool new_cam_pose[2] = {false, false};;
    bool found_left_cam_to_right_cam_tr = false;
    VTKTask *task_ptr;

private:

    int n_arms;
    cv::Mat image_from_ros[2];

    char recording_event;
    bool new_recording_event;
    bool show_reference_frames;

    int image_width;
    int image_height;

    std::string stl_files_dir;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_subscribers[2];

    image_transport::Subscriber subscriber_image_left;
    image_transport::Subscriber subscriber_image_right;
    ros::Subscriber sub_cam_pose_left;
    ros::Subscriber sub_cam_pose_right;
    ros::Subscriber sub_foot_pedal_clutch;
    ros::Subscriber subscriber_recording_events;

    ros::Subscriber * subtool_current_pose;
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
