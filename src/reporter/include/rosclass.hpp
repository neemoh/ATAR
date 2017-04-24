#ifndef ROSCLASS_HPP
#define ROSCLASS_HPP


#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <opencv2/core.hpp>
#include <QtCore>
#include <fstream>
#include <iostream>

class RosObj : public QThread
{
public:

  RosObj(QObject *parent, std::string node_name);

  void OnPsm1CartesianPoseMessage(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void OnPsm2CartesianPoseMessage(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void OnPsm1JointStateMessage(const sensor_msgs::JointState::ConstPtr &msg);
  void OnPsm2JointStateMessage(const sensor_msgs::JointState::ConstPtr &msg);

  void OnCameraImageMessage(const sensor_msgs::ImageConstPtr &msg);
  void SetStateLabel(int in ){ state_label = in;  }

  void OpenRecordingFile(std::string filename);
  void CloseRecordingFile();
  void StartRecording() {recording = true;}
  void StopRecording() {recording = false;}

  int AddRegistrationPoint();
  void ResetRegistrationPoints();

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

private:
  void GetROSParameterValues();
  int state_label;
  bool recording;
  std::ofstream reporting_file;
  std::vector<std::vector<double>> registration_points;
public:

  ros::NodeHandle n;
  ros::Rate * ros_rate;
  double ros_freq;

  std::string node_name;

  cv::Mat image_msg;
  geometry_msgs::Pose psm1_pose;
  geometry_msgs::Pose psm2_pose;
  sensor_msgs::JointState psm1_joint_state;
  sensor_msgs::JointState psm2_joint_state;

  std::vector<double> board_to_robot_tr;
  geometry_msgs::Pose cam_pose;

  ros::Subscriber psm1_cart_subscriber;
  ros::Subscriber psm1_joint_subscriber;
  ros::Subscriber psm2_joint_subscriber;
  ros::Subscriber psm2_cart_subscriber;

  ros::Publisher pub_cam_to_robot_pose;
  ros::Publisher pub_board_to_robot_pose;

};



#endif // ROSCLASS_HPP
