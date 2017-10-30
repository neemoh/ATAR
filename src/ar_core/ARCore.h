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
#include <std_msgs/Int8.h>
// ros and opencv
#include "ros/ros.h"
// ros messages




class ARCore {
public:

    ARCore(std::string node_name);

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

//    // If the poses of the cameras are published, this method will return
//    // true when any of the cam poses are updated. If left or right pose is
//    // missing it will be found transforming the other available pose with the
//    // left_cam_to_right_cam_tr. This transformation is calculated once in a
//    // locking call in this method.
//    bool GetNewCameraPoses(cv::Vec3d cam_rvec[2], cv::Vec3d cam_tvec[2]);


    void Cleanup();

public:
    // -------------------------------------------------------------------------
    // ROS CALLBACKS

    // this topic is used to control the task state from the recording node
    // during the acquisitions.
    void ControlEventsCallback(const std_msgs::Int8ConstPtr &msg);

private:

    SimTask *task_ptr;

    boost::thread haptics_thread;

    ros::NodeHandlePtr n;

    bool new_task_event                 = false;

    uint running_task_id;
    int8_t control_event;

    ros::Subscriber sub_cam_pose_left;
    ros::Subscriber sub_cam_pose_right;
    ros::Subscriber subscriber_control_events;

};


#endif //TELEOP_VISION_OVERLAYROSCONFIG_H
