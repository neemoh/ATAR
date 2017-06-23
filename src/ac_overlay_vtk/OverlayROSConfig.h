//
// Created by nima on 4/17/17.
//

#ifndef TELEOP_VISION_OVERLAYROSCONFIG_H
#define TELEOP_VISION_OVERLAYROSCONFIG_H

#include "ros/ros.h"
#include <boost/thread/thread.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <kdl_conversions/kdl_msg.h>
#include <image_transport/image_transport.h>
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include "VTKTask.h"
#include "Rendering.h"
#include <mutex>

class OverlayROSConfig {


public:

    OverlayROSConfig(std::string node_name);


    bool UpdateWorld();

private:
    std::mutex m;

    // stop the running haptic thread (if any), destruct the previous task
    // (if any) and start a new task and thread.
    void HandleTaskEvent();

    // start a new task and thread.
    void StartTask(const uint task_id);

    // stop the running haptic thread and destruct the  task object
    void DeleteTask();

    // Locking call to retrieve the images
    void LockAndGetImages(ros::Duration timeout, cv::Mat images[]);

    // return true if both images are newly received and copy them in imgs
    bool GetNewImages( cv::Mat images[]);

    // If the poses of the cameras are published, this method will return
    // true when any of the cam poses are updated. If left or right pose is
    // missing it will be found transforming the other available pose with the
    // left_cam_to_right_cam_tr. This transformation is calculated once in a
    // locking call in this method.
    bool GetNewCameraPoses(cv::Vec3d cam_rvec[2], cv::Vec3d cam_tvec[2]);

    // publishes the active constraint parameters
    void PublishACtiveConstraintParameters(
            const custom_msgs::ActiveConstraintParameters &);

    // publishes the active constraint parameters
    void PublishTaskState(custom_msgs::TaskState msg);

    void DoArmToWorldFrameCalibration(const uint arm_id);

    void StartArmToWorldFrameCalibration(const uint arm_id);

    void Cleanup();

    void PublishRenderedImages();

    // Reads parameters and sets up subscribers and publishers
    void SetupROS();

    // reads the intrinsic camera parameters
    void ReadCameraParameters(const std::string file_path,
                              cv::Mat &camera_matrix,
                              cv::Mat &camera_distortion);

public:
    // -------------------------------------------------------------------------
    // ROS CALLBACKS
    void ImageLeftCallback(const sensor_msgs::ImageConstPtr &msg);

    void ImageRightCallback(const sensor_msgs::ImageConstPtr &msg);

    // The camera poses. Note that this actually defines the pose of the
    // task coordinate frame in camera coordinate frame
    void LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void RightCamPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // Tool poses in task coordinate frame (taskspace).
    void Tool1PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Tool2PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Tool gripper callbacks
    void Tool1GripperCurrentCallback(const std_msgs::Float32::ConstPtr &msg);

    void Tool2GripperCurrentCallback(const std_msgs::Float32::ConstPtr &msg);

    // foot switch used to select the ac path
    void FootSwitchCallback(const sensor_msgs::JoyConstPtr &msg);

    // this topic is used to control the task state from the recording node
    // during the acquisitions.
    void ControlEventsCallback(const std_msgs::Int8ConstPtr &msg);


private:

    VTKTask *task_ptr;

    Rendering * graphics;

    boost::thread haptics_thread;

    // IN ALL CODE 0 is Left Cam, 1 is Right cam
    // ----------------------------------

    ros::NodeHandle n;
    double desired_pose_update_freq;
    int n_arms;
    bool publish_overlayed_images       = false;
    bool one_window_mode                = false;
    bool with_shadows                   = false;
    bool offScreen_rendering            = false;
    bool show_reference_frames          = false;
    bool new_task_event                 = false;

    std::string mesh_files_dir;

    bool foot_switch_pressed = false;
    bool with_guidance;
    cv::Mat camera_matrix[2];
    cv::Mat camera_distortion[2];
    KDL::Frame pose_cam[2];
    KDL::Frame pose_current_tool[2];
    double gripper_current[2];
    bool cam_poses_provided_as_params;
    KDL::Frame slave_frame_to_world_frame[2];
    KDL::Frame left_cam_to_right_cam_tr;
    cv::Vec3d cam_rvec[2];
    cv::Vec3d cam_tvec[2];
    bool new_image[2] = {false, false};
    bool new_cam_pose[2] = {false, false};;
    bool found_left_cam_to_right_cam_tr = false;


    cv::Mat image_from_ros[2];
    uint running_task_id;
    std::string cv_window_names[2];
    int8_t control_event;

    image_transport::ImageTransport *it;
    image_transport::Subscriber image_subscribers[2];

    image_transport::Subscriber subscriber_image_left;
    image_transport::Subscriber subscriber_image_right;
    ros::Subscriber sub_cam_pose_left;
    ros::Subscriber sub_cam_pose_right;
    ros::Subscriber sub_foot_pedal_clutch;
    ros::Subscriber subscriber_control_events;

    ros::Subscriber * subtool_current_pose;
    ros::Subscriber * subtool_current_gripper;
    ros::Publisher * publisher_tool_pose_desired;
    ros::Publisher * publisher_ac_params;
    ros::Publisher publisher_task_state;

    //overlay image publishers
    image_transport::Publisher publisher_overlayed[2];
    image_transport::Publisher publisher_stereo_overlayed;

    // two function pointers for slave pose callbacks
    void (OverlayROSConfig::*pose_current_tool_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave gripper callbacks
    void (OverlayROSConfig::*gripper_callbacks[2])
            (const std_msgs::Float32::ConstPtr &msg);


};


#endif //TELEOP_VISION_OVERLAYROSCONFIG_H
