//
// Created by nima on 16/10/17.
//

#ifndef ATAR_MANIPULATOR_H
#define ATAR_MANIPULATOR_H

// This class interfaces with a manipulator by subscribing to its pose and
// twist topics and transforming them to the world reference frame. This
// local to world transformation is found differently in VR and AR cases. The
// common element in both cases is that we need to know the pose of the
// camera with respect to the world (camera_to_world_frame_tr). This has to
// be set from outside of the class by passing the pointer of this Manipulator
// object to a Rendering object through the SetManipulatorInterestedInCamPose
// method (check the DemoTask). After doing that the camera pose will be
// communicated to the manipulator every time it changes. Now about the rest
// of the kinematics chain:
//
// - VR: The manipulator is interfacing with a master device. Here the
// local_to_world_frame_tr is calculated as:
// local_to_world_frame_tr.M = camera_to_world_frame_tr.M *local_to_image_frame_rot;
// where local_to_image_frame_rot is the tr from the base of the master
// device to the image frame (i.e. the image you see in the display, i.e. the
// camera!). THe image frame is opencv style: X axis is left to right, y is
// top to bottom and so z is perpendicular into the image. Note that we are
// only interested in the rotation from the master base to the image. This tr
// is set by setting a parameter "/calibrations"+arm_ns+"_frame_to_image_frame"
// with 4 elements representing the quaternion rotation. Check the
// params_calibrations_ar.yaml file to see examples of this.
//
// - AR: In the augmented reality case we are interfacing with a slave arm
// that is seen in the camera images. Here we need to find the transformation
// from the slave to the world frame by performing a calibration procedure.
// TO BE COMPLETED

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>

class Manipulator {

public:
    Manipulator(std::string topic_ns,
                std::string pose_topic,
                std::string gripper_topic,
                std::string twist_topic = "",
                KDL::Frame initial_pose=KDL::Frame());

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void GripperCallback(const std_msgs::Float32ConstPtr &msg);

    void TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);

    void GetPoseLocal(KDL::Frame& pose){pose = pose_local;};

    void GetPoseWorld(KDL::Frame& pose){pose = pose_world;};

    KDL::Frame GetPoseWorld(){return pose_world;};

    void GetGripper(double& gripper){gripper = gripper_angle;};

    double GetGripper(){return gripper_angle;};

    void GetTwistLocal(KDL::Twist& twist){twist = twist_local;};

    void GetTwistWorld(KDL::Twist& twist){twist = twist_world;};

    void DoArmToWorldFrameCalibration();

    void SetWorldToCamTr(const KDL::Frame &in);  // needed for AR

    KDL::Frame GetWorldToLocalTr(){return local_to_world_frame_tr.Inverse();};


private:
    void CalibrationThread();

private:
    std::string arm_ns;

    ros::NodeHandlePtr n; // made it a member just for the calibration method

    boost::thread calibration_thread;

    KDL::Frame pose_local;
    KDL::Frame pose_world;
    KDL::Frame local_to_world_frame_tr;
    KDL::Frame camera_to_world_frame_tr;  //i.e. camera to world
    KDL::Rotation local_to_image_frame_rot;

    KDL::Twist twist_local;
    KDL::Twist twist_world;
    double gripper_angle;

    ros::Subscriber sub_pose;
    ros::Subscriber sub_gripper;
    ros::Subscriber sub_twist;
};


#endif //ATAR_MANIPULATOR_H
