//
// Created by nima on 16/10/17.
//

#include <kdl_conversions/kdl_msg.h>
#include <custom_conversions/Conversions.h>
#include "Manipulator.h"
#include "ManipulatorToWorldCalibration.h"
#include <src/arm_to_world_calibration/ArmToWorldCalibration.h>

Manipulator::Manipulator(
        const std::string arm_name,
        const std::string pose_topic,
        const std::string gripper_topic,
        const std::string twist_topic,
        KDL::Frame initial_pose)
        :
        n(ros::NodeHandlePtr(new ros::NodeHandle("~"))),
        pose_world(initial_pose),
        arm_name(arm_name)
{

    //--------------------------------------------------------------------------
    // Define subscribers
    sub_pose = n->subscribe(pose_topic, 1,
                            &Manipulator::PoseCallback, this);

    if(!gripper_topic.empty())
        sub_gripper = n->subscribe(gripper_topic, 1,
                                   &Manipulator::GripperCallback, this);

    if(!twist_topic.empty())
        sub_twist = n->subscribe(twist_topic, 1,
                                 &Manipulator::TwistCallback, this);

    //--------------------------------------------------------------------------
    // Get the transformation to the display and calculate the tr to the
    // world frame
    std::string master_mode_param =
            "/calibrations/"+arm_name+"_frame_to_image_frame";
    std::vector<double> vect_master = std::vector<double>(4, 0.0);

    std::string slave_mode_param =
            "/calibrations/world_frame_to_"+arm_name+"_frame";
    std::vector<double> vec_slave = std::vector<double>(7, 0.0);

    if (n->getParam(master_mode_param, vect_master)) {
        conversions::QuatVectorToKDLRot(vect_master, local_to_image_frame_rot);
        master_mode = true;
    }
    else if(n->getParam(slave_mode_param, vec_slave))
    {
        conversions::PoseVectorToKDLFrame(vec_slave, local_to_world_frame_tr);
        local_to_world_frame_tr = local_to_world_frame_tr.Inverse();
        master_mode = false;
    }
    else
        ROS_WARN("Neither %s nor %s parameter was found. Manipulator may not "
                         "be correctly calibrated.",
                 master_mode_param.c_str(), slave_mode_param.c_str());

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
    if(master_mode) {
        camera_to_world_frame_tr = in.Inverse();
        local_to_world_frame_tr.M = camera_to_world_frame_tr.M *
                                    local_to_image_frame_rot;
    } else
        ROS_WARN_ONCE("SetWorldToCamTr is needed when manipulator is in master"
                              " mode.");
}


// -----------------------------------------------------------------------------
void Manipulator::DoArmToWorldFrameCalibration() {

    if(!master_mode) {
        ROS_INFO("Starting manipulator to world calibration thread.");

        // bind the haptics thread
        calibration_thread = boost::thread(
                boost::bind(&Manipulator::CalibrationThread, this));
    } else
        ROS_WARN("Manipulator to world calibration is used "
                         "when manipulator is in slave mode.");

}


void Manipulator::CalibrationThread(){

    ManipulatorToWorldCalibration cal(this);
    KDL::Frame world_to_local_tr;
    if(cal.DoCalibration(world_to_local_tr)){

        // set ros param
        std::string param =
                "/calibrations/world_frame_to_"+arm_name+"_frame";

        std::vector<double> vec7(7, 0.0);
        conversions::KDLFrameToVector(world_to_local_tr, vec7);
        n->setParam(param, vec7);

        // set output
        local_to_world_frame_tr = world_to_local_tr.Inverse();
    }

    // not really needed..
    calibration_thread.interrupt();


}

