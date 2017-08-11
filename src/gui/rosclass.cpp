//
// Created by nearlab on 14/12/16.
//

#include <stdexcept>

#include <cv_bridge/cv_bridge.h>

#include "rosclass.hpp"

#include <custom_msgs/ActiveConstraintParameters.h>
#include "custom_msgs/TaskState.h"
#include <std_msgs/Int8.h>


RosObj::RosObj(QObject *parent, std::string node_name)
        : n(node_name), ros_freq(0), node_name(node_name),
          state_label(0), recording(false), new_task_state_msg(false)
{

    slave_pose_current_callbacks[0] = &RosObj::Slave0CurrentPoseCallback;
    slave_pose_current_callbacks[1] = &RosObj::Slave1CurrentPoseCallback;

    slave_pose_desired_callbacks[0] = &RosObj::Slave0DesiredPoseCallback;
    slave_pose_desired_callbacks[1] = &RosObj::Slave1DesiredPoseCallback;

    master_pose_current_callbacks[0] = &RosObj::Master0CurrentPoseCallback;
    master_pose_current_callbacks[1] = &RosObj::Master1CurrentPoseCallback;

    slave_twist_callbacks[0] = &RosObj::Slave0TwistCallback;
    slave_twist_callbacks[1] = &RosObj::Slave1TwistCallback;

    master_joint_state_callbacks[0] = &RosObj::Master0JointStateCallback;
    master_joint_state_callbacks[1] = &RosObj::Master1JointStateCallback;

    gripper_callbacks[0] = &RosObj::Master1GripperCallback;
    gripper_callbacks[1] = &RosObj::Master2GripperCallback;

    master_twist_callbacks[0] = &RosObj::Master0TwistCallback;
    master_twist_callbacks[1] = &RosObj::Master1TwistCallback;

    master_wrench_callbacks[0] = &RosObj::Master0WrenchCallback;
    master_wrench_callbacks[1] = &RosObj::Master1WrenchCallback;

    master_ac_params_callbacks[0] = &RosObj::Master0ACParamsCallback;
    master_ac_params_callbacks[1] = &RosObj::Master1ACParamsCallback;

    task_frame_to_slave_frame[0] = std::vector<double>(7, 0.0);
    task_frame_to_slave_frame[1] = std::vector<double>(7, 0.0);

    GetROSParameterValues();
}


RosObj::~RosObj(){
    //    delete task_frame_to_slave_frame;;
    qDebug() << "In rosobj dsstructor";
    ros::shutdown();
    qDebug() << "rosobj dsstructor end";

}

//-----------------------------------------------------------------------------------
// GetROSParameterValues
//-----------------------------------------------------------------------------------

void RosObj::GetROSParameterValues() {

    // spinning frequency. THe recording happens at the freq of task_state
    ros_rate = new ros::Rate(100);

    n.param<int>("number_of_arms", n_arms, 1);
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

    if (n_arms < 1)
        ROS_ERROR("Number of arms must be at least 1.");

    subscriber_slave_pose_current = new ros::Subscriber[n_arms];
    subscriber_master_pose_current = new ros::Subscriber[n_arms];
    subscriber_slave_pose_desired = new ros::Subscriber[n_arms];
    subscriber_slave_twist = new ros::Subscriber[n_arms];
    subscriber_master_joint_state = new ros::Subscriber[n_arms];
    subscriber_master_current_gripper = new ros::Subscriber[n_arms];
    subscriber_master_twist = new ros::Subscriber[n_arms];
    subscriber_master_wrench = new ros::Subscriber[n_arms];
    subscriber_ac_params = new ros::Subscriber[n_arms];

    // arm names
    for (int n_arm = 0; n_arm < n_arms; n_arm++) {

        //getting the name of the arms
        std::stringstream param_name;
        param_name << std::string("slave_") << n_arm + 1 << "_name";
        std::cout << param_name.str()<< std::endl;

        n.getParam(param_name.str(),slave_names[n_arm]);

        param_name.str("");
        param_name << std::string("master_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), master_names[n_arm]);


        //        // publishers
        //        param_name.str("");
        //        param_name << std::string("/dvrk/") << master_names[n_arm]
        //                      << "/set_wrench_body";
        //        publisher_wrench[n_arm] = n.advertise<geometry_msgs::Wrench>(
        //                    param_name.str().c_str(), 1);
        //        ROS_INFO("Will publish on %s", param_name.str().c_str());

        // subscribers
        param_name.str("");
        param_name << std::string("/")
                   << slave_names[n_arm]<< "/tool_pose_desired";
        subscriber_slave_pose_desired[n_arm] = n.subscribe(
                param_name.str(), 1,
                slave_pose_desired_callbacks[n_arm],
                this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());

        // the current pose of the slaves
        param_name.str("");
        param_name << std::string("/dvrk/") << slave_names[n_arm]
                   << "/position_cartesian_current";
        subscriber_slave_pose_current[n_arm] = n.subscribe(param_name.str(), 1,
                                                           slave_pose_current_callbacks[n_arm],
                                                           this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());


        // the current pose of the masters
        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/position_cartesian_current";
        subscriber_master_pose_current[n_arm] = n.subscribe(param_name.str(), 1,
                                                            master_pose_current_callbacks[n_arm],
                                                            this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());


        //  master's joint state
        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/state_joint_current";
        subscriber_master_joint_state[n_arm]
                = n.subscribe(param_name.str(), 1
                , master_joint_state_callbacks[n_arm], this);

        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());

        //  master's gripper
        param_name.str("");
        param_name << std::string("/dvrk/") <<master_names[n_arm]
                   << "/gripper_position_current";
        subscriber_master_current_gripper[n_arm] =
                n.subscribe(param_name.str(), 1, gripper_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        //  master's twist
        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/twist_body_current";
        subscriber_master_twist[n_arm] = n.subscribe(param_name.str(),
                                                     1,
                                                     master_twist_callbacks[n_arm],
                                                     this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());


        //  Slave's twist
        param_name.str("");
        param_name << std::string("/dvrk/") << slave_names[n_arm]
                   << "/twist_body_current";
        subscriber_slave_twist[n_arm] = n.subscribe(param_name.str(),
                                                    1,
                                                    slave_twist_callbacks[n_arm],
                                                    this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());

        //master's wrench
        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/set_wrench_body";
        subscriber_master_wrench[n_arm] = n.subscribe(param_name.str(),
                                                      1,
                                                      master_wrench_callbacks[n_arm],
                                                      this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/") << master_names[n_arm]
                   << "/active_constraint_param";
        subscriber_ac_params[n_arm] = n.subscribe(param_name.str(), 1,
                                                  master_ac_params_callbacks[n_arm],
                                                  this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());


//        // the transformation from the coordinate frame of the slave (RCM) to the task coordinate
//        // frame.
//        param_name.str("");
//        param_name << (std::string) "/calibrations/world_frame_to_"
//                   << slave_names[n_arm] << "_frame";
//        if (!n.getParam(param_name.str(), task_frame_to_slave_frame[n_arm]))
//            ROS_WARN("Parameter %s was not found.", param_name.str().c_str());


    }

    // common subscriber
    subscriber_foot_pedal_coag = n.subscribe("/dvrk/footpedals/coag", 1,
                                             &RosObj::CoagFootSwitchCallback,
                                             this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /dvrk/footpedals/coag");

    subscriber_foot_pedal_clutch = n.subscribe("/dvrk/footpedals/clutch", 1,
                                               &RosObj::ClutchFootSwitchCallback,
                                               this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /dvrk/footpedals/clutch");


    subscriber_task_state = n.subscribe("/task_state", 1,
                                        &RosObj::TaskSTateCallback,
                                        this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /task_state");

    //publisher
    publisher_control_events = n.advertise<std_msgs::Int8>
            ("/control_events", 1);
    ROS_INFO("Will publish on /control_events");


}




//-----------------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------------

// PSMs current pose
void RosObj::Slave0CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    slave_current_pose[0].position = msg->pose.position;
    slave_current_pose[0].orientation = msg->pose.orientation;
}

void RosObj::Slave1CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    slave_current_pose[1].position = msg->pose.position;
    slave_current_pose[1].orientation = msg->pose.orientation;
}

// PSMs Desired pose
void RosObj::Slave0DesiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    slave_desired_pose[0].position = msg->pose.position;
    slave_desired_pose[0].orientation = msg->pose.orientation;
}

void RosObj::Slave1DesiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    slave_desired_pose[1].position = msg->pose.position;
    slave_desired_pose[1].orientation = msg->pose.orientation;
}

// MTMs current pose
void RosObj::Master0CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    master_current_pose[0].position = msg->pose.position;
    master_current_pose[0].orientation = msg->pose.orientation;
}

void RosObj::Master1CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    master_current_pose[1].position = msg->pose.position;
    master_current_pose[1].orientation = msg->pose.orientation;
}


// Slave Twist
void RosObj::Slave0TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    slave_twist[0].linear = msg->twist.linear;
    slave_twist[0].linear = msg->twist.linear;
}

void RosObj::Slave1TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    slave_twist[1].linear = msg->twist.linear;
    slave_twist[1].linear = msg->twist.linear;
}


// MTMs  joint state
void RosObj::Master0JointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
    master_joint_state[0].position = msg->position;
    master_joint_state[0].velocity = msg->velocity;
    master_joint_state[0].effort = msg->effort;
}

void RosObj::Master1JointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
    master_joint_state[1].position = msg->position;
    master_joint_state[1].velocity = msg->velocity;
    master_joint_state[1].effort = msg->effort;
}

// -----------------------------------------------------------------------------
// Reading the gripper positions
void RosObj::Master1GripperCallback(
        const std_msgs::Float32::ConstPtr &msg) {
    gripper_position[0] =  msg->data;
}

void RosObj::Master2GripperCallback(
        const std_msgs::Float32::ConstPtr &msg) {
    gripper_position[1] =  msg->data;

}

// MTMs Twist
void RosObj::Master0TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    master_twist[0].linear = msg->twist.linear;
    master_twist[0].linear = msg->twist.linear;
}

void RosObj::Master1TwistCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    master_twist[1].linear = msg->twist.linear;
    master_twist[1].linear = msg->twist.linear;
}


// MTMs Wrench
void RosObj::Master0WrenchCallback(const geometry_msgs::WrenchConstPtr &msg) {
    master_wrench[0].force= msg->force;
    master_wrench[0].torque= msg->torque;
}

void RosObj::Master1WrenchCallback(const geometry_msgs::WrenchConstPtr &msg) {
    master_wrench[1].force= msg->force;
    master_wrench[1].torque= msg->torque;
}


// MTMs AC params
void RosObj::Master0ACParamsCallback(
        const custom_msgs::ActiveConstraintParametersConstPtr & msg){
    ac_params[0].active = msg->active;
    ac_params[0].method = msg->method;
    ac_params[0].angular_damping_coeff = msg->angular_damping_coeff;
    ac_params[0].angular_elastic_coeff = msg->angular_elastic_coeff;
    ac_params[0].linear_damping_coeff = msg->linear_damping_coeff;
    ac_params[0].linear_elastic_coeff = msg->linear_elastic_coeff;
    ac_params[0].max_force = msg->max_force;
    ac_params[0].max_torque = msg->max_torque;
}


void RosObj::Master1ACParamsCallback(
        const custom_msgs::ActiveConstraintParametersConstPtr & msg){
    ac_params[1].active = msg->active;
    ac_params[1].method = msg->method;
    ac_params[1].angular_damping_coeff = msg->angular_damping_coeff;
    ac_params[1].angular_elastic_coeff = msg->angular_elastic_coeff;
    ac_params[1].linear_damping_coeff = msg->linear_damping_coeff;
    ac_params[1].linear_elastic_coeff = msg->linear_elastic_coeff;
    ac_params[1].max_force = msg->max_force;
    ac_params[1].max_torque = msg->max_torque;
}

// task state
void RosObj::TaskSTateCallback(const custom_msgs::TaskStateConstPtr &msg){
    task_state.task_name = msg->task_name;
    task_state.number_of_repetition = msg->number_of_repetition;
    task_state.task_state = msg->task_state;
    task_state.time_stamp = msg->time_stamp;
    task_state.error_field_1 = msg->error_field_1;
    task_state.error_field_2 = msg->error_field_2;
    new_task_state_msg = true;
}

// foot pedals
void RosObj::CoagFootSwitchCallback(const sensor_msgs::Joy &msg){
    clutch_pedal_pressed = (bool) msg.buttons[0];
}

void RosObj::ClutchFootSwitchCallback(const sensor_msgs::Joy &msg){
    coag_pedal_pressed = (bool) msg.buttons[0];
}


void RosObj::OnCameraImageMessage(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_msg = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

cv::Mat& RosObj::Image(ros::Duration timeout) {
    ros::Rate loop_rate(100);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_msg.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_ERROR("Timeout whilst waiting for a new image from the image topic. "
                              "Is the camera still publishing ?");
        }
    }

    return image_msg;
}


void RosObj::OpenRecordingFile(std::string filename){

    reporting_file.open(filename);
    //first line is the name of the arms
    reporting_file
            << (std::string)"slave_names[0]: " << slave_names[0]
            << ", slave_names[1]: " << slave_names[1]
            << ", master_names[0]: " << master_names[0]
            << ", master_names[1]: " << master_names[1]
            << std::endl;

    reporting_file
            << "number_of_repetition"
            << ", task_state"
            << ", time_stamp"
            << ", error_field_1"
            << ", error_field_2"
            << ", clutch_pedal_pressed"
            << ", coag_pedal_pressed" ;

    for (int i = 0; i < n_arms; ++i) {

        reporting_file
                << ", slave_current_pose.position.x"
                << ", slave_current_pose.position.y"
                << ", slave_current_pose.position.z"
                << ", slave_current_pose.orientation.x"
                << ", slave_current_pose.orientation.y"
                << ", slave_current_pose.orientation.z"
                << ", slave_current_pose.orientation.w"

                << ", slave_desired_pose.position.x"
                << ", slave_desired_pose.position.y"
                << ", slave_desired_pose.position.z"
                << ", slave_desired_pose.orientation.x"
                << ", slave_desired_pose.orientation.y"
                << ", slave_desired_pose.orientation.z"
                << ", slave_desired_pose.orientation.w"

                << ", master_current_pose.position.x"
                << ", master_current_pose.position.y"
                << ", master_current_pose.position.z"
                << ", master_current_pose.orientation.x"
                << ", master_current_pose.orientation.y"
                << ", master_current_pose.orientation.z"
                << ", master_current_pose.orientation.w"

                << ", master_joint_state.position[0]"
                << ", master_joint_state.position[1]"
                << ", master_joint_state.position[2]"
                << ", master_joint_state.position[3]"
                << ", master_joint_state.position[4]"
                << ", master_joint_state.position[5]"
                << ", master_joint_state.position[6]"
                << ", master_joint_state.position[7]"

                << ", gripper_position"

                << ", slave_twist.linear.x"
                << ", slave_twist.linear.y"
                << ", slave_twist.linear.z"
                << ", slave_twist.angular.x"
                << ", slave_twist.angular.y"
                << ", slave_twist.angular.z"

                << ", master_twist.linear.x"
                << ", master_twist.linear.y"
                << ", master_twist.linear.z"
                << ", master_twist.angular.x"
                << ", master_twist.angular.y"
                << ", master_twist.angular.z"

                << ", master_wrench.force.x"
                << ", master_wrench.force.y"
                << ", master_wrench.force.z"
                << ", master_wrench.torque.x"
                << ", master_wrench.torque.y"
                << ", master_wrench.torque.z"

                << ",ac_params.active"
                << ",ac_params.method"
                << ",ac_params.angular_damping_coeff"
                << ",ac_params.angular_elastic_coeff"
                << ",ac_params.linear_damping_coeff"
                << ",ac_params.linear_elastic_coeff"
                << ",ac_params.max_force"
                << ",ac_params.max_torque";

    }

    reporting_file << std::endl;
}



void RosObj::run(){

    while(ros::ok() ){
        ros::Time begin = ros::Time::now();

        // decided to record the data only when the task state is 1.
//        if(recording && new_task_state_msg && task_state.task_state==1 ){
        if(recording ){

            new_task_state_msg = false;

            //seconds = psm1_joint_state.header.stamp.nsec;
            //	std::cout << "sec=" << secs << "  nsec= " << nsecs << std::endl;

            std::vector<double> data;

            data.push_back(double(task_state.number_of_repetition));
            data.push_back(double(task_state.task_state));
            data.push_back(task_state.time_stamp);
            data.push_back(task_state.error_field_1);
            data.push_back(task_state.error_field_2);
            data.push_back(double(clutch_pedal_pressed));
            data.push_back(double(coag_pedal_pressed));

            for(uint j=0; j<n_arms; j++){

                data.push_back(slave_current_pose[j].position.x);
                data.push_back(slave_current_pose[j].position.y);
                data.push_back(slave_current_pose[j].position.z);
                data.push_back(slave_current_pose[j].orientation.x);
                data.push_back(slave_current_pose[j].orientation.y);
                data.push_back(slave_current_pose[j].orientation.z);
                data.push_back(slave_current_pose[j].orientation.w);

                data.push_back(slave_desired_pose[j].position.x);
                data.push_back(slave_desired_pose[j].position.y);
                data.push_back(slave_desired_pose[j].position.z);
                data.push_back(slave_desired_pose[j].orientation.x);
                data.push_back(slave_desired_pose[j].orientation.y);
                data.push_back(slave_desired_pose[j].orientation.z);
                data.push_back(slave_desired_pose[j].orientation.w);

                data.push_back(master_current_pose[j].position.x);
                data.push_back(master_current_pose[j].position.y);
                data.push_back(master_current_pose[j].position.z);
                data.push_back(master_current_pose[j].orientation.x);
                data.push_back(master_current_pose[j].orientation.y);
                data.push_back(master_current_pose[j].orientation.z);
                data.push_back(master_current_pose[j].orientation.w);

                data.push_back(master_joint_state[j].position[0]);
                data.push_back(master_joint_state[j].position[1]);
                data.push_back(master_joint_state[j].position[2]);
                data.push_back(master_joint_state[j].position[3]);
                data.push_back(master_joint_state[j].position[4]);
                data.push_back(master_joint_state[j].position[5]);
                data.push_back(master_joint_state[j].position[6]);
                data.push_back(master_joint_state[j].position[7]);

                data.push_back(gripper_position[j]);

                data.push_back(slave_twist[j].linear.x);
                data.push_back(slave_twist[j].linear.y);
                data.push_back(slave_twist[j].linear.z);
                data.push_back(slave_twist[j].angular.x);
                data.push_back(slave_twist[j].angular.y);
                data.push_back(slave_twist[j].angular.z);

                data.push_back(master_twist[j].linear.x);
                data.push_back(master_twist[j].linear.y);
                data.push_back(master_twist[j].linear.z);
                data.push_back(master_twist[j].angular.x);
                data.push_back(master_twist[j].angular.y);
                data.push_back(master_twist[j].angular.z);

                data.push_back(master_wrench[j].force.x);
                data.push_back(master_wrench[j].force.y);
                data.push_back(master_wrench[j].force.z);
                data.push_back(master_wrench[j].torque.x);
                data.push_back(master_wrench[j].torque.y);
                data.push_back(master_wrench[j].torque.z);

                data.push_back(double(ac_params[j].active));
                data.push_back(double(ac_params[j].method));
                data.push_back(ac_params[j].angular_damping_coeff);
                data.push_back(ac_params[j].angular_elastic_coeff);
                data.push_back(ac_params[j].linear_damping_coeff);
                data.push_back(ac_params[j].linear_elastic_coeff);
                data.push_back(ac_params[j].max_force);
                data.push_back(ac_params[j].max_torque);
            }

            for(int i= 0; i<data.size()-1; i++){
                reporting_file << data[i] << ", ";
            }
            reporting_file << data.back() << std::endl;
        }

        ros::spinOnce();
        ros_rate->sleep();
    }
}


void RosObj::CloseRecordingFile(){

    reporting_file.close();
    qDebug() << "Closed reporting file";


}


void RosObj::CleanUpAndQuit(){

    //TODO cleanup
    ros::shutdown();

    delete [] subscriber_slave_pose_current;
    delete [] subscriber_master_pose_current;
    delete [] subscriber_slave_pose_desired;
    delete [] subscriber_slave_twist ;
    delete [] subscriber_master_joint_state ;
    delete [] subscriber_master_twist ;
    delete [] subscriber_master_wrench;
    delete [] subscriber_ac_params;
    delete ros_rate;

    quit();

}





