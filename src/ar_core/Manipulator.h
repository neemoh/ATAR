//
// Created by nima on 16/10/17.
//

#ifndef ATAR_MANIPULATOR_H
#define ATAR_MANIPULATOR_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include "ARCamera.h"

class Manipulator {

public:
    Manipulator(ros::NodeHandlePtr n,
                const std::string topic_ns,
                const std::string pose_topic,
                const std::string gripper_topic,
                ARCamera *cam_ptr,
                const std::string twist_topic = "");

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void GripperCallback(const std_msgs::Float64ConstPtr &msg);

    void TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);

    void GetPoseLocal(KDL::Frame& pose){pose = pose_local;};

    void GetPoseWorld(KDL::Frame& pose){pose = pose_world;};

    void GetGripper(double& gripper){gripper = gripper_angle;};

    void GetTwistLocal(KDL::Twist& twist){twist = twist_local;};

    void GetTwistWorld(KDL::Twist& twist){twist = twist_world;};

    void DoArmToWorldFrameCalibration();

private:

    KDL::Frame pose_local;
    KDL::Frame pose_world;
    KDL::Frame local_to_world_frame_tr;
    KDL::Rotation local_to_image_frame_tr;

    KDL::Twist twist_local;
    KDL::Twist twist_world;
    double gripper_angle;

    ARCamera * cam_ptr;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_gripper;
    ros::Subscriber sub_twist;
};


#endif //ATAR_MANIPULATOR_H
