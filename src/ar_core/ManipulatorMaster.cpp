//
// Created by nima on 16/10/17.
//

#include <kdl_conversions/kdl_msg.h>
#include <custom_conversions/Conversions.h>
#include "ManipulatorMaster.h"

ManipulatorMaster::ManipulatorMaster(ros::NodeHandle *n,
                                     const std::string arm_ns,
                                     const std::string pose_topic,
                                     const std::string gripper_topic,
                                     const KDL::Frame *world_to_camera_tr,
                                     const std::string twist_topic) {

    //--------------------------------------------------------------------------
    // Define subscribers
    sub_pose = n->subscribe(arm_ns+pose_topic, 1,
                           &ManipulatorMaster::PoseCallback, this);

    sub_gripper = n->subscribe(arm_ns+gripper_topic, 1,
                        &ManipulatorMaster::GripperCallback, this);

    if(twist_topic!="")
        sub_twist = n->subscribe(arm_ns+twist_topic, 1,
                                  &ManipulatorMaster::TwistCallback, this);

    //--------------------------------------------------------------------------
    // Get the transformation to the display and calculate the tr to the
    // world frame
    std::string param = "/calibrations"+arm_ns+"_frame_to_image_frame";
    std::vector<double> vect_temp = std::vector<double>(7, 0.0);
    KDL::Frame local_to_image_frame_tr;

    if (n->getParam(param, vect_temp)){
        conversions::VectorToKDLFrame(vect_temp, local_to_image_frame_tr);
        // make sure the translation part is zero
        local_to_image_frame_tr.p = KDL::Vector(0.0, 0.0,0.0);

        if(world_to_camera_tr!=NULL)
            local_to_world_frame_tr = (*world_to_camera_tr).Inverse() *
                local_to_image_frame_tr;
    } else
        ROS_WARN("Parameter %s was not found.", param.c_str());
}

void
ManipulatorMaster::PoseCallback(const geometry_msgs::PoseStampedConstPtr
                                &msg) {

    tf::poseMsgToKDL(msg->pose, pose_local);
    pose_world =  local_to_world_frame_tr * pose_local;

}

void ManipulatorMaster::GripperCallback(const std_msgs::Float32ConstPtr &msg) {
    gripper_angle = msg->data;
}


void
ManipulatorMaster::TwistCallback(const geometry_msgs::TwistStampedConstPtr
                                 &msg) {

    tf::twistMsgToKDL(msg->twist, twist_local);
    twist_world = local_to_world_frame_tr * twist_local;
}
