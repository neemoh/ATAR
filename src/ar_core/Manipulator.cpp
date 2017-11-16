//
// Created by nima on 16/10/17.
//

#include <kdl_conversions/kdl_msg.h>
#include <custom_conversions/Conversions.h>
#include "Manipulator.h"
#include "ManipulatorToWorldCalibration.h"
#include <src/arm_to_world_calibration/ArmToWorldCalibration.h>

Manipulator::Manipulator(
                         const std::string arm_ns,
                         const std::string pose_topic,
                         const std::string gripper_topic,
                         const std::string twist_topic,
                         KDL::Frame initial_pose)
        :
        n(ros::NodeHandlePtr(new ros::NodeHandle("~"))),
        pose_world(initial_pose),
        arm_ns(arm_ns)
{

    //--------------------------------------------------------------------------
    // Define subscribers
    sub_pose = n->subscribe(arm_ns+pose_topic, 1,
                            &Manipulator::PoseCallback, this);

    sub_gripper = n->subscribe(arm_ns+gripper_topic, 1,
                               &Manipulator::GripperCallback, this);

    if(!twist_topic.empty())
        sub_twist = n->subscribe(arm_ns+twist_topic, 1,
                                 &Manipulator::TwistCallback, this);

    //--------------------------------------------------------------------------
    // Get the transformation to the display and calculate the tr to the
    // world frame
    std::string param = "/calibrations"+arm_ns+"_frame_to_image_frame";
    std::vector<double> vect_temp = std::vector<double>(4, 0.0);

    if (n->getParam(param, vect_temp)){
        conversions::QuatVectorToKDLRot(vect_temp, local_to_image_frame_rot);
    }
    else
        ROS_WARN("Parameter %s was not found.", param.c_str());

}

// -----------------------------------------------------------------------------
void Manipulator::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {

    tf::poseMsgToKDL(msg->pose, pose_local);
    pose_world =  local_to_world_frame_tr * pose_local;

}

// -----------------------------------------------------------------------------
void Manipulator::GripperCallback(const std_msgs::Float32ConstPtr &msg) {
    gripper_angle = msg->data;
}


// -----------------------------------------------------------------------------
void Manipulator::TwistCallback(const geometry_msgs::TwistStampedConstPtr
                                &msg) {

    tf::twistMsgToKDL(msg->twist, twist_local);
    twist_world = local_to_world_frame_tr * twist_local;
}

// -----------------------------------------------------------------------------
void Manipulator::SetWorldToCamTr(const KDL::Frame &in) {

    camera_to_world_frame_tr = in.Inverse();
    local_to_world_frame_tr.M = camera_to_world_frame_tr.M *
                                local_to_image_frame_rot;
}


// -----------------------------------------------------------------------------
void Manipulator::DoArmToWorldFrameCalibration() {
    ROS_INFO("Starting AMnipulator to world calibration thread.");

    // bind the haptics thread
    calibration_thread = boost::thread(
            boost::bind(&Manipulator::CalibrationThread, this));
}


void Manipulator::CalibrationThread(){

    ManipulatorToWorldCalibration cal(this);
    KDL::Frame world_to_local_tr;
    if(cal.DoCalibration(world_to_local_tr)){

        // set ros param
        std::string param =
                "/calibrations/world_frame_to_"+arm_ns+"_frame";

        std::vector<double> vec7(7, 0.0);
        conversions::KDLFrameToVector(world_to_local_tr, vec7);
        n->setParam(param, vec7);

        // set output
        local_to_world_frame_tr = world_to_local_tr.Inverse();
    }
    
    // not really needed..
    calibration_thread.interrupt();


}

