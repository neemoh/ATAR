//
// Created by nima on 4/17/17.
//

#ifndef TELEOP_VISION_OVERLAYROSCONFIG_H
#define TELEOP_VISION_OVERLAYROSCONFIG_H

// related headers
#include "SimTask.h"
#include "Rendering.h"
#include <boost/thread/thread.hpp>
#include <mutex>
// ros and opencv
#include "ros/ros.h"
#include <kdl_conversions/kdl_msg.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
// ros messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"



class ARCore {
public:

    ARCore(std::string node_name);

    bool UpdateWorld();

private:
    std::mutex m;

    // Reads parameters and sets up subscribers and publishers
    void SetupROSandGetParameters();

    // reads required parameters and initializes the graphics
    void SetupGraphics();

    // stop the running haptic thread (if any), destruct the previous task
    // (if any) and start a new task and thread.
    void HandleTaskEvent();

    // start a new task and thread.
    void StartTask(const uint task_id);

    // stop the running haptic thread and destruct the  task object
    void DeleteTask();

    // If the poses of the cameras are published, this method will return
    // true when any of the cam poses are updated. If left or right pose is
    // missing it will be found transforming the other available pose with the
    // left_cam_to_right_cam_tr. This transformation is calculated once in a
    // locking call in this method.
    bool GetNewCameraPoses(cv::Vec3d cam_rvec[2], cv::Vec3d cam_tvec[2]);

    // publishes the active constraint parameters
    void PublishTaskState(custom_msgs::TaskState msg){
        publisher_task_state.publish(msg);
    };

    void DoArmToWorldFrameCalibration(const uint arm_id);

    void StartArmToWorldFrameCalibration(const uint arm_id);

    void Cleanup();

    void PublishRenderedImages();

    // reads the intrinsic camera parameters
    void ReadCameraParameters(const std::string file_path,
                              cv::Mat &camera_matrix,
                              cv::Mat &camera_distortion);

public:
    // -------------------------------------------------------------------------
    // ROS CALLBACKS

    // this topic is used to control the task state from the recording node
    // during the acquisitions.
    void ControlEventsCallback(const std_msgs::Int8ConstPtr &msg);

private:

    SimTask *task_ptr;

    Rendering * graphics;

    boost::thread haptics_thread;

    // IN ALL CODE 0 is Left Cam, 1 is Right cam
    // ----------------------------------

    ros::NodeHandlePtr n;

    double   haptic_loop_rate;
    int n_arms;
    bool ar_mode                        = false;
    bool publish_overlayed_images       = false;
    bool one_window_mode                = false;
    bool new_task_event                 = false;
    bool show_reference_frames          = false;

    bool with_guidance;

    cv::Mat camera_matrix[2];
    cv::Mat camera_distortion[2];
    KDL::Frame pose_cam[2];
    KDL::Frame slave_frame_to_world_frame[2];
    KDL::Frame left_cam_to_right_cam_tr;

    //// estimate left to right cam trans
    //uint left_cam_to_right_cam_tr_loop_count = 0;
    //KDL::Vector left_cam_to_right_cam_tr_sum_pos;

    cv::Vec3d cam_rvec_curr[2];
    cv::Vec3d cam_tvec_curr[2];
    cv::Vec3d cam_rvec_avg[2];
    cv::Vec3d cam_tvec_avg[2];
    bool new_image[2] = {false, false};
    bool new_cam_pose[2] = {false, false};;

    cv::Mat image_from_ros[2];
    uint running_task_id;
    std::string cv_window_names[2];
    int8_t control_event;

    ros::Subscriber sub_cam_pose_left;
    ros::Subscriber sub_cam_pose_right;
    ros::Subscriber subscriber_control_events;

    ros::Subscriber * subtool_current_pose;
    ros::Subscriber * subtool_current_gripper;
    ros::Publisher * publisher_tool_pose_desired;
    ros::Publisher publisher_task_state;

    //overlay image publishers
    image_transport::Publisher publisher_overlayed[2];
    image_transport::Publisher publisher_stereo_overlayed;


};

void SwitchFullScreenCV(const std::string window_name);

#endif //TELEOP_VISION_OVERLAYROSCONFIG_H
