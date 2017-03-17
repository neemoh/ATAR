//
// Created by charm on 3/3/17.
//

#include "ros/ros.h"
#include "teleop_vision/CalculateStereoCamsTransfromFromTopics.h"
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

bool GetTransform(
        teleop_vision::CalculateStereoCamsTransfromFromTopics::Request  &req,
        teleop_vision::CalculateStereoCamsTransfromFromTopics::Response &res){
    //ros::NodeHandle n;
    ROS_INFO("Service calculate_stereo_cams_transofrm_from_topics was called");

    auto pose_cam_1 =
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>
                    (req.cam_1_pose_topic_name, ros::Duration(1));
    auto pose_cam_2 =
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>
                    (req.cam_2_pose_topic_name, ros::Duration(1));

    if(!pose_cam_1){
        ROS_WARN("No data is being publish on %s", req.cam_1_pose_topic_name.c_str());
        return false;
    }
    else if (!pose_cam_2){
        ROS_WARN("No data is being publish on %s", req.cam_2_pose_topic_name.c_str());
        return false;
    }
    std::cout << "Received Cam 1 Pose: \n" << pose_cam_1->pose <<std::endl;
    std::cout << "Received Cam 2 Pose: \n" << pose_cam_2->pose <<std::endl;

    // Converting to frames
    KDL::Frame frame_cam_1, frame_cam_2;
    tf::poseMsgToKDL(pose_cam_1->pose, frame_cam_1);
    tf::poseMsgToKDL(pose_cam_2->pose, frame_cam_2);

    // Find the transform
    geometry_msgs::Pose pose_out;
    tf::poseKDLToMsg((frame_cam_2 * frame_cam_1.Inverse()), pose_out);
    std::cout << "Calculated cam1 to cam2 transform as: \n" << pose_out <<std::endl;

    // send back
    res.cam_1_to_cam_2_pose = pose_out;


//    // Testing. To be removed
//    KDL::Vector my_vec(1, 0.0,0.0);
//    KDL::Vector my_vec1, my_vec2, my_vec2_t;
//
//    my_vec1 = frame_cam_1* my_vec;
//    my_vec2 = frame_cam_2* my_vec;
//    my_vec2_t = (frame_cam_2 * frame_cam_1.Inverse()) * my_vec1;
//
//    std::cout << "my_vec1 " <<  my_vec1[0] << "  "<<  my_vec1[1] << "  "
//              <<  my_vec1[2] << "  "<< std::endl;
//    std::cout << "my_vec2 " <<  my_vec2[0] << "  "<<  my_vec2[1] << "  "
//              <<  my_vec2[2] << "  "<< std::endl;
//    std::cout << "my_vec2_t " <<  my_vec2_t[0] << "  "<<  my_vec2_t[1] << "  "
//              <<  my_vec2_t[2] << "  "<< std::endl;

    return true;

};



int main(int argc, char **argv){

    ros::init(argc, argv, "calculate_stereo_cams_transform_from_topics");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService(
            "calculate_stereo_cams_transform_from_topics", GetTransform);
    ROS_INFO("Detected!");
    ros::spin();

    return 0;
}
