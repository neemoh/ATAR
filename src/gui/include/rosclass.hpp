#ifndef ROSCLASS_HPP
#define ROSCLASS_HPP


#include <vector>
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>

#include <opencv2/core.hpp>
#include <QtCore>

#include <custom_msgs/ActiveConstraintParameters.h>
#include <custom_msgs/TaskState.h>

class RosObj : public QThread
{
public:

    RosObj(QObject *parent, std::string node_name);
    ~RosObj();

    void RingPoseCurrentCallback(const geometry_msgs::PoseConstPtr &msg);
    void RingPoseDesiredCallback(const geometry_msgs::PoseConstPtr &msg);

    void Slave0CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void Slave1CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void Slave0DesiredPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void Slave1DesiredPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void Master0CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void Master1CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void Slave0TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void Slave1TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);

    void Master0JointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
    void Master1JointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    void Master1GripperCallback(const std_msgs::Float32::ConstPtr &msg);
    void Master2GripperCallback(const std_msgs::Float32::ConstPtr &msg);

    void Master0TwistCallback(const geometry_msgs::TwistConstPtr &msg);
    void Master1TwistCallback(const geometry_msgs::TwistConstPtr &msg);

    void Master0WrenchCallback(const geometry_msgs::WrenchConstPtr &msg);
    void Master1WrenchCallback(const geometry_msgs::WrenchConstPtr &msg);

    void Master1ACParamsCallback(
            const custom_msgs::ActiveConstraintParametersConstPtr & msg);
    void Master0ACParamsCallback(
            const custom_msgs::ActiveConstraintParametersConstPtr & msg);

    void TaskSTateCallback(const custom_msgs::TaskStateConstPtr  &msg);

    void CoagFootSwitchCallback(const sensor_msgs::Joy &msg);

    void ClutchFootSwitchCallback(const sensor_msgs::Joy &msg);


    void OnCameraImageMessage(const sensor_msgs::ImageConstPtr &msg);
    void SetStateLabel(int in ){ state_label = in;  }

    void OpenRecordingFile(std::string filename);
    void CloseRecordingFile();
    void StartRecording() {recording = true;}
    void PauseRecording() {recording = false;}
    bool IsRecording() {return recording;}

    // get the number of repetition. Used to be sent from the task world, now
    // is set and updated  locally.
    uint GetRepetitionNumber() {
        //return task_state.number_of_repetition;
        return repetition_num;}

    void SetRepetitionNumber(uint in) {
         repetition_num = in;}

    void ResetCurrentAcquisition(){repetition_data->clear();};


    void run();


    /**
     * Get the latest image from the input.
     *
     * If no image is available this call will block until an image is received. Otherwise this will
     * simply return #RosObj::Image. In practice the only time where #RosObj::Image might be empty
     * is just after the start of the program before the first frame has arrived or if the camera
     * gets disconnected.
     *
     * @param timeout The maximum amount of time to wait for the image
     * @return A reference to the image.
     */
    cv::Mat& Image(ros::Duration timeout = ros::Duration(1));


    void CleanUpAndQuit();

private:
    void GetROSParameterValues();
    int n_arms;
    int state_label;
    bool recording;
    bool new_task_state_msg;
    std::ofstream reporting_file;
    uint repetition_num=1;
    // two function pointers for each master/slave related topic

    void (RosObj::*slave_pose_current_callbacks[2])
            (const geometry_msgs::PoseStampedConstPtr &msg);

    void (RosObj::*master_pose_current_callbacks[2])
            (const geometry_msgs::PoseStampedConstPtr &msg);

    void (RosObj::*slave_pose_desired_callbacks[2])
            (const geometry_msgs::PoseStampedConstPtr &msg);

    void (RosObj::*slave_twist_callbacks[2])
            (const geometry_msgs::TwistStampedConstPtr &msg);

    void (RosObj::*master_joint_state_callbacks[2])
            (const sensor_msgs::JointStateConstPtr &msg);

    void (RosObj::*gripper_callbacks[2])
            (const std_msgs::Float32::ConstPtr &msg);

    void (RosObj::*master_twist_callbacks[2])
            (const geometry_msgs::TwistConstPtr &msg);

    void (RosObj::*master_wrench_callbacks[2])
            (const geometry_msgs::Wrench::ConstPtr &msg);

    void (RosObj::*master_ac_params_callbacks[2])
            (const custom_msgs::ActiveConstraintParametersConstPtr &msg);



public:

    ros::NodeHandle n;
    ros::Rate * ros_rate;
    double ros_freq;

    std::string node_name;
    std::string slave_names[2];
    std::string master_names[2];
    cv::Mat image_msg;

    geometry_msgs::Pose ring_pose_current;
    geometry_msgs::Pose ring_pose_desired;

    // NOTE: 0 stands for left and 1 for right;

    // current and desired cartesian poses of the slaves
    geometry_msgs::Pose slave_current_pose[2];

    geometry_msgs::Pose slave_desired_pose[2];

    geometry_msgs::Pose master_current_pose[2];

    // joint states
    sensor_msgs::JointState master_joint_state[2];

    // gripper
    double gripper_position[2];

    // twists
    geometry_msgs::Twist slave_twist[2];

    geometry_msgs::Twist master_twist[2];

    // wrenches
    geometry_msgs::Wrench master_wrench[2];

    // task state
    custom_msgs::TaskState task_state;

    // left and right master active constraints parameters
    custom_msgs::ActiveConstraintParameters ac_params[2];

    // foot pedals;
    bool clutch_pedal_pressed;
    bool coag_pedal_pressed;

    std::vector<double> task_frame_to_slave_frame[2];
    geometry_msgs::Pose cam_pose;

    // SUBSCRIBERS
    ros::Subscriber subscriber_ring_pose_current;
    ros::Subscriber subscriber_ring_pose_desired;

    ros::Subscriber * subscriber_slave_pose_current;
    ros::Subscriber * subscriber_slave_pose_desired;
    ros::Subscriber * subscriber_master_pose_current;

    ros::Subscriber * subscriber_master_joint_state;

    ros::Subscriber * subscriber_master_current_gripper;

    ros::Subscriber * subscriber_master_twist;
    ros::Subscriber * subscriber_slave_twist;
    ros::Subscriber * subscriber_master_wrench;

    ros::Subscriber * subscriber_ac_params;

    ros::Subscriber subscriber_task_state;

    ros::Subscriber subscriber_foot_pedal_coag;
    ros::Subscriber subscriber_foot_pedal_clutch;

    ros::Publisher publisher_control_events;

private:
    std::vector< std::vector<double> > * repetition_data;
};



#endif // ROSCLASS_HPP
