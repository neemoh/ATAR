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

#include "src/gui/SteadyHandPerfEval.h"

class RosBridge : public QThread
{
public:

    RosBridge(QObject *parent, std::string node_name);
    ~RosBridge();

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

    void TaskSTateCallback(const custom_msgs::TaskStateConstPtr  &msg);

    void CoagFootSwitchCallback(const sensor_msgs::Joy &msg);

    void ClutchFootSwitchCallback(const sensor_msgs::Joy &msg);

//    void OnCameraImageMessage(const sensor_msgs::ImageConstPtr &msg);

    void SetStateLabel(int in ){ state_label = in;  }

    void OpenRecordingFile(std::string filename);
    void CloseRecordingFile();
    void StartRecording(
        const int in,
        const double last_session_perf,
        const double last_last_session_perf
    );

    void PauseRecording() {recording = false;}
    void ContinueRecording() {recording = true;}

    bool IsRecording() {return recording;}

    // get the number of repetition. Used to be sent from the task world, now
    // is set and updated  locally.
    uint GetRepetitionNumber() {
        //return task_state.number_of_repetition;
        return repetition_num;}

    void SetRepetitionNumber(uint in) {
         repetition_num = in;}

    void ResetCurrentAcquisition();

    void ResetTask();

    void run();

    void CleanUpAndQuit();

    void SetHapticsMode(int state){haptics_mode = state;};

    void OverrideACActivation(const double act);

    void GetPerformanceHistory(std::vector<double> &perf_hist){
        perf_hist = perf_history;
    };

private:
    int n_arms;
    int state_label;
    bool recording;
    bool new_task_state_msg;
    int haptics_mode = 0;
    std::ofstream reporting_file;
    uint repetition_num = 1;

    std::vector<double> VectorizeData();

    SteadyHandPerfEval *perf_eval=0;
    std::vector<double> perf_history;

    double assistance_activation;
    // this comes from the last session performance of the same user
    double performance_initial_value;

private:

    void InitializeAdaptiveAssistance(
        const uint n_session,
        const double performance_initial_value,
        const double last_last_session_perf
    );

    void PublishACActivation(const double &activation);

    void DumpDataToFileAndClearBuffer();

    void GetROSParameterValues();


    // two function pointers for each master/slave related topic

    void (RosBridge::*slave_pose_current_callbacks[2])
            (const geometry_msgs::PoseStampedConstPtr &msg);

    void (RosBridge::*master_pose_current_callbacks[2])
            (const geometry_msgs::PoseStampedConstPtr &msg);

    void (RosBridge::*slave_pose_desired_callbacks[2])
            (const geometry_msgs::PoseStampedConstPtr &msg);

    void (RosBridge::*slave_twist_callbacks[2])
            (const geometry_msgs::TwistStampedConstPtr &msg);

    void (RosBridge::*master_joint_state_callbacks[2])
            (const sensor_msgs::JointStateConstPtr &msg);

    void (RosBridge::*gripper_callbacks[2])
            (const std_msgs::Float32::ConstPtr &msg);

    void (RosBridge::*master_twist_callbacks[2])
            (const geometry_msgs::TwistConstPtr &msg);

    void (RosBridge::*master_wrench_callbacks[2])
            (const geometry_msgs::Wrench::ConstPtr &msg);


public:

    custom_msgs::ActiveConstraintParameters ac_parameters[2];

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
    custom_msgs::TaskState task_state, last_task_state;

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

    ros::Subscriber subscriber_task_state;

    ros::Subscriber subscriber_foot_pedal_coag;
    ros::Subscriber subscriber_foot_pedal_clutch;

    ros::Publisher publisher_control_events;
    ros::Publisher * publisher_ac_params;

private:
    std::vector< std::vector<double> > * ongoing_acq_buffer;
};



#endif // ROSCLASS_HPP
