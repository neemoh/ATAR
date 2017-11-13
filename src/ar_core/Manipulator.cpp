//
// Created by nima on 16/10/17.
//

#include <kdl_conversions/kdl_msg.h>
#include <custom_conversions/Conversions.h>
#include "Manipulator.h"
#include <src/arm_to_world_calibration/ArmToWorldCalibration.h>

Manipulator::Manipulator(ros::NodeHandlePtr nh,
                         const std::string arm_ns,
                         const std::string pose_topic,
                         const std::string gripper_topic,
                         const std::string twist_topic,
                         KDL::Frame initial_pose)
        :
        n(nh),
        pose_world(initial_pose)
{

    //--------------------------------------------------------------------------
    // Define subscribers
    sub_pose = n->subscribe(arm_ns+pose_topic, 1,
                            &Manipulator::PoseCallback, this);

    sub_gripper = n->subscribe(arm_ns+gripper_topic, 1,
                               &Manipulator::GripperCallback, this);

    if(twist_topic!="")
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

    // TODO: OLD IMPLEMENTATION. NEEDS TO BE FIXED
    //// REMOVE THIS LINE
    int arm_id = 0;
    /////

    std::string cam_name;
    n->getParam("left_cam_name", cam_name);

    cv::Mat camera_matrix[2];
    cv::Mat camera_distortion[2];

    //getting the name of the arms
    std::stringstream param_name;
    std::string slave_name;

    param_name << std::string("slave_") << arm_id + 1 << "_name";
    n->getParam(param_name.str(), slave_name);

    std::stringstream arm_pose_namespace;
    arm_pose_namespace << std::string("/dvrk/") <<slave_name
                       << "/position_cartesian_current";

    std::stringstream cam_pose_namespace;
    std::string left_cam_name;
    n->getParam("left_cam_name", left_cam_name);
    cam_pose_namespace << std::string("/") << left_cam_name
                       << "/world_to_camera_transform";

    std::string cam_image_name_space ;
    n->getParam("left_image_topic_name", cam_image_name_space);

    // putting the calibration point on the corners of the board squares
    // the parameter can be set directly, unless there is the global
    // /calibrations/board_params
    double calib_points_distance = 0.01;
    std::vector<float> board_params = std::vector<float>(5, 0.0);

    if(!n->getParam("calib_points_distance", calib_points_distance)){
        if(n->getParam("/calibrations/board_params", board_params))
            calib_points_distance = board_params[3];
    };

    int num_calib_points;
    n->param<int>("number_of_calibration_points", num_calib_points, 6);

    ArmToWorldCalibration AWC;
    KDL::Frame world_to_arm_frame;
    std::vector<double> calib_point_center
            = {board_params[1]/2 * calib_points_distance
                    , board_params[2]/2 * calib_points_distance};

    if(AWC.DoCalibration(
            cam_image_name_space, cam_pose_namespace.str(),
            arm_pose_namespace.str(),
            camera_matrix[0], camera_distortion[0], (uint) num_calib_points,
            calib_points_distance, calib_point_center, world_to_arm_frame
    )){

        // set ros param
        param_name.str("");
        param_name << std::string("/calibrations/world_frame_to_") <<
                   slave_name << "_frame";

        std::vector<double> vec7(7, 0.0);
        conversions::KDLFrameToVector(world_to_arm_frame, vec7);
        n->setParam(param_name.str(), vec7);

        // set output
        local_to_world_frame_tr = world_to_arm_frame.Inverse();
    }

}

