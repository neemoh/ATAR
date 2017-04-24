//
// Created by nearlab on 14/12/16.
//

#include <stdexcept>

#include <cv_bridge/cv_bridge.h>

#include "rosclass.hpp"

using namespace std;

RosObj::RosObj(QObject *parent, string node_name)
  : n(node_name), ros_freq(0), node_name(node_name),
    state_label(0), recording(false)
{
  GetROSParameterValues();
}


//-----------------------------------------------------------------------------------
// GetROSParameterValues
//-----------------------------------------------------------------------------------

void RosObj::GetROSParameterValues() {
  bool all_required_params_found = true;

  n.param<double>("rate", ros_freq, 100);

  std::string psm1_cartesian_pose_topic_name_param;
  if (!n.getParam("/psm1_cartesian_pose_topic_name", psm1_cartesian_pose_topic_name_param)) {
    ROS_ERROR("Parameter /psm1_cartesian_pose_topic_name is required.");
    all_required_params_found = false;
  }

  std::string psm2_cartesian_pose_topic_name_param;
  if (!n.getParam("/psm2_cartesian_pose_topic_name", psm2_cartesian_pose_topic_name_param))  {
    ROS_ERROR("/Parameter /psm2_cartesian_pose_topic_name is required.");
    all_required_params_found = false;
  }

  std::string psm1_joint_state_topic_name_param;
  if (!n.getParam("/psm1_joint_state_topic_name", psm1_joint_state_topic_name_param)) {
    ROS_ERROR("Parameter /psm1_joint_state_topic_name is required.");
    all_required_params_found = false;
  }

  std::string psm2_joint_state_topic_name_param;
  if (!n.getParam("/psm2_joint_state_topic_name", psm2_joint_state_topic_name_param)) {
    ROS_ERROR("Parameter /psm2_joint_state_topic_name is required.");
    all_required_params_found = false;
  }

  std::string pedal_topic_name_param;
  if (!n.getParam("/pedal_topic_name", pedal_topic_name_param)) {
    ROS_ERROR("Parameter /pedal_topic_name is required.");
    all_required_params_found = false;
  }

  if (!all_required_params_found)
    throw std::runtime_error("ERROR: some required topics are not set");


  psm1_cart_subscriber = n.subscribe(psm1_cartesian_pose_topic_name_param, 10, &RosObj::OnPsm1CartesianPoseMessage, this);
  psm2_cart_subscriber = n.subscribe(psm2_cartesian_pose_topic_name_param, 10, &RosObj::OnPsm2CartesianPoseMessage, this);
  psm1_joint_subscriber = n.subscribe(psm1_joint_state_topic_name_param, 10, &RosObj::OnPsm1JointStateMessage, this);
  psm2_joint_subscriber = n.subscribe(psm2_joint_state_topic_name_param, 10, &RosObj::OnPsm2JointStateMessage, this);

  ros_rate = new ros::Rate(ros_freq);

}




//-----------------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------------

void RosObj::OnPsm1CartesianPoseMessage(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  psm1_pose.position = msg->pose.position;
  psm1_pose.orientation = msg->pose.orientation;
}

void RosObj::OnPsm2CartesianPoseMessage(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  psm2_pose.position = msg->pose.position;
  psm2_pose.orientation = msg->pose.orientation;
}

void RosObj::OnPsm1JointStateMessage(const sensor_msgs::JointState::ConstPtr &msg) {
  psm1_joint_state.position = msg->position;
  psm1_joint_state.velocity = msg->velocity;
}

void RosObj::OnPsm2JointStateMessage(const sensor_msgs::JointState::ConstPtr &msg) {
  psm2_joint_state.position = msg->position;
  psm2_joint_state.velocity = msg->velocity;
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


void RosObj::run(){

  while(ros::ok()){
    ros::Time begin = ros::Time::now();


    if(recording){
      ros::Duration past = ros::Time::now() - begin;
      double secs = past.toSec();
      double nsecs = past.toNSec();

      //seconds = psm1_joint_state.header.stamp.nsec;
      //	std::cout << "sec=" << secs << "  nsec= " << nsecs << std::endl;
      std::vector<double> data{secs, nsecs,

            double(state_label),
            psm1_pose.position.x, psm1_pose.position.y
            ,psm1_pose.position.z,  psm1_pose.orientation.x
            ,psm1_pose.orientation.y,  psm1_pose.orientation.z
            ,psm1_pose.orientation.w
            ,psm2_pose.position.x,  psm2_pose.position.y
            ,psm2_pose.position.z,  psm2_pose.orientation.x
            ,psm2_pose.orientation.y , psm2_pose.orientation.z
            ,psm2_pose.orientation.w
            ,psm1_joint_state.position[0]
            ,psm1_joint_state.position[1]
            ,psm1_joint_state.position[2]
            ,psm1_joint_state.position[3]
            ,psm1_joint_state.position[4]
            ,psm1_joint_state.position[5]
            ,psm1_joint_state.position[6]
            ,psm1_joint_state.velocity[0]
            ,psm1_joint_state.velocity[1]
            ,psm1_joint_state.velocity[2]
            ,psm1_joint_state.velocity[3]
            ,psm1_joint_state.velocity[4]
            ,psm1_joint_state.velocity[5]
            ,psm1_joint_state.velocity[6]
            ,psm2_joint_state.position[0]
            ,psm2_joint_state.position[1]
            ,psm2_joint_state.position[2]
            ,psm2_joint_state.position[3]
            ,psm2_joint_state.position[4]
            ,psm2_joint_state.position[5]
            ,psm2_joint_state.position[6]
            ,psm2_joint_state.velocity[0]
            ,psm2_joint_state.velocity[1]
            ,psm2_joint_state.velocity[2]
            ,psm2_joint_state.velocity[3]
            ,psm2_joint_state.velocity[4]
            ,psm2_joint_state.velocity[5]
            ,psm2_joint_state.velocity[6]
                              };

      for(int i= 0; i<data.size()-1; i++){
        reporting_file << data[i] << ", ";
      }
      reporting_file << data.back() << std::endl;

    }

    ros::spinOnce();
    ros_rate->sleep();
  }

}


void RosObj::OpenRecordingFile(std::string filename){

  reporting_file.open(filename);
  reporting_file
      << "state_label"
      << ", psm1_pose.position.x" << ", psm1_pose.position.y"
      << ", psm1_pose.position.z" << ", psm1_pose.orientation.x "
      << ", psm1_pose.orientation.y" << ", psm1_pose.orientation.z "
      << ", psm1_pose.orientation.w"
      << ", psm2_pose.position.x" << ", psm2_pose.position.y "
      << ", psm2_pose.position.z" << ", psm2_pose.orientation.x "
      << ", psm2_pose.orientation.y" << ", psm2_pose.orientation.z "
      << ", psm2_pose.orientation.w"
      <<",psm1_joint_state.position[0]"
     <<", psm1_joint_state.position[1]"
    <<", psm1_joint_state.position[2]"
   <<", psm1_joint_state.position[3]"
  <<", psm1_joint_state.position[4]"
  <<", psm1_joint_state.position[5]"
  <<", psm1_joint_state.position[6]"
  <<", psm1_joint_state.velocity[0]"
  <<", psm1_joint_state.velocity[1]"
  <<", psm1_joint_state.velocity[2]"
  <<", psm1_joint_state.velocity[3]"
  <<", psm1_joint_state.velocity[4]"
  <<", psm1_joint_state.velocity[5]"
  <<", psm1_joint_state.velocity[6]"
  <<",psm2_joint_state.position[0]"
  <<", psm2_joint_state.position[1]"
  <<", psm2_joint_state.position[2]"
  <<", psm2_joint_state.position[3]"
  <<", psm2_joint_state.position[4]"
  <<", psm2_joint_state.position[5]"
  <<", psm2_joint_state.position[6]"
  <<", psm2_joint_state.velocity[0]"
  <<", psm2_joint_state.velocity[1]"
  <<", psm2_joint_state.velocity[2]"
  <<", psm2_joint_state.velocity[3]"
  <<", psm2_joint_state.velocity[4]"
  <<", psm2_joint_state.velocity[5]"
  <<", psm2_joint_state.velocity[6]"<< std::endl;

}

void RosObj::CloseRecordingFile(){

  reporting_file.close();
  qDebug() << "Closed reporting file";


}



int RosObj::AddRegistrationPoint(){

  int num_registeration_points = 6;

  if(registration_points.size() < num_registeration_points){
    // taking 5 points for psm1

    std::vector<double> point{psm1_pose.position.x, psm1_pose.position.y, psm1_pose.position.z};
    registration_points.push_back(point);
  }
  else if(registration_points.size() < (2*num_registeration_points)){

    std::vector<double> point{psm2_pose.position.x, psm2_pose.position.y, psm2_pose.position.z};
    registration_points.push_back(point);
  }


  // check if we got all points
  if(registration_points.size() == (2*num_registeration_points)){

    // write the values in a file
    std::ofstream registration_file;
    registration_file.open("registration_pints.csv");
    // write a header
    registration_file << "position.x, position.y, position.z #num registeration points per arm= " <<
                         num_registeration_points << std::endl;

    for(int i=0; i< registration_points.size(); i++){

      registration_file
          << registration_points[i][0]
          << ", " << registration_points[i][1]
          << ", " << registration_points[i][2] << std::endl;

    }
    registration_file.close();

  }

  return registration_points.size();

}



void RosObj::ResetRegistrationPoints(){

  registration_points.clear();
  qDebug() << "registration_points size "<<registration_points.size();
}




