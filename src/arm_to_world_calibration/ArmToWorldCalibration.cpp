//
// Created by nima on 01/06/17.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <custom_conversions/Conversions.h>
#include <opencv2/highgui/highgui_c.h>
#include "ArmToWorldCalibration.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <kdl_conversions/kdl_msg.h>


// -----------------------------------------------------------------------------
//
bool ArmToWorldCalibration::DoCalibration(const std::string img_topic_namespace,
                                          const std::string cam_pose_topic_namespace,
                                          const std::string arm_pose_topic_namespace,
                                          const cv::Mat camera_matrix,
                                          const cv::Mat dist_coeffs,
                                          const uint num_calib_points,
                                          const double calib_points_distance,
                                          KDL::Frame & result) {

    cam_mat = camera_matrix;
    dist_mat = dist_coeffs;
    window_name = std::string("Arm to world calibration") + arm_pose_topic_namespace;

    // --------------------------------Set ros up ------------------------------
    ros::NodeHandle n("IntrinsicCalibrationCharuco");
    ros::Rate loop_rate = ros::Rate(50);

    // Image subsciber
    image_transport::ImageTransport it = image_transport::ImageTransport(n);
    image_transport::Subscriber sub =
            it.subscribe(img_topic_namespace, 1,
                         &ArmToWorldCalibration::CameraImageCallback, this);
    ROS_INFO("IntrinsicCalibrationCharuco subscribed to %s", img_topic_namespace
            .c_str());

    // Cam pose subsciber
    ros::Subscriber camera_pose_subscriber =
            n.subscribe(cam_pose_topic_namespace, 10,
                        &ArmToWorldCalibration::CameraPoseCallback, this);

    // Arm pose subsciber
    ros::Subscriber arm_pose_subscriber =
            n.subscribe(arm_pose_topic_namespace, 10,
                        &ArmToWorldCalibration::ArmPoseCallback, this);

    // -------------------------------------------------------------------------
    // Definition of target points for calib2
    for (uint i=0; i<num_calib_points; i++){
        if(i<num_calib_points/3)
            calib_points_in_world_frame.push_back(
                    Eigen::Vector3d((1+i)*(calib_points_distance),
                                    calib_points_distance,
                                    0.0));
        if(i>num_calib_points/3-1 && i<2*num_calib_points/3)
            calib_points_in_world_frame.push_back(
                    Eigen::Vector3d((i+1-num_calib_points/3)*(calib_points_distance),
                                    2*calib_points_distance,
                                    0.0));
        if(i>-1+2*num_calib_points/3)
            calib_points_in_world_frame.push_back(
                    Eigen::Vector3d((i+1-2*num_calib_points/3)*(calib_points_distance),
                                    3*calib_points_distance,
                                    0.0));
    }
    // -------------------------------------------------------------------------

    while(ros::ok() && !exit ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // -------------------------------------------------------------------------

    if(calibration_done){
        result = world_to_arm_tr;
        cvDestroyWindow(window_name.c_str());
        return true;
    }

    cvDestroyWindow(window_name.c_str());
    return false;
}


// -----------------------------------------------------------------------------
//
void ArmToWorldCalibration::ArmPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // save the pose in a kdl frame
    tf::poseMsgToKDL(msg->pose, arm_pose_in_robot_frame);

}


// -----------------------------------------------------------------------------
//
void ArmToWorldCalibration::CameraPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // converting to frame and rvec/tvec
    tf::poseMsgToKDL(msg->pose, world_to_cam_tr);
    conversions::KDLFrameToRvectvec(world_to_cam_tr,
                                    cam_rvec,
                                    cam_tvec);
}


// -----------------------------------------------------------------------------
//
void ArmToWorldCalibration::CameraImageCallback(
        const sensor_msgs::ImageConstPtr &msg) {


    cv::Mat image, imageCopy;

    try{
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    image.copyTo(imageCopy);

    PutDrawings(imageCopy);

    cv::imshow(window_name, imageCopy);
    char key = (char) cv::waitKey(1);
    if (key == 27)
        exit = true;
    if (key == 'c' && !calibration_done) {

        // save the position of the end effector
        calib_points_in_arm_frame.push_back( Eigen::Vector3d(
                arm_pose_in_robot_frame.p[0],
                arm_pose_in_robot_frame.p[1],
                arm_pose_in_robot_frame.p[2]));

        if (calib_points_in_arm_frame.size() == calib_points_in_world_frame.size()){
            world_to_arm_tr = CalculateTransformation
                    (calib_points_in_world_frame, calib_points_in_arm_frame);
            calibration_done = true;
        }
    }
}


// -----------------------------------------------------------------------------
//
KDL::Frame ArmToWorldCalibration::CalculateTransformation(
        std::vector<Eigen::Vector3d> points_in_frame_1,
        std::vector<Eigen::Vector3d> points_in_frame_2) {

    if( points_in_frame_1.size()!= points_in_frame_2.size())
        throw std::runtime_error("Num of points don't match.");

    // points_in_frame_2_mat: measured=arm
    Eigen::Matrix<double, 3, 6> points_in_frame_2_mat;
    Eigen::Matrix<double, 3, 6> points_in_frame_1_mat;

    for (uint i = 0; i < points_in_frame_1.size(); i++) {
        points_in_frame_1_mat.col(i) = points_in_frame_1[i];
        points_in_frame_2_mat.col(i) = points_in_frame_2[i];
    }

    auto temp = Eigen::umeyama(points_in_frame_1_mat, points_in_frame_2_mat, 0);

    KDL::Frame task_frame_to_robot_frame;
    // converting Eigen::Matrix into KDL::Frame
    cv::Matx33d rot_mat;
    for (uint i = 0; i < 3; i++) {
        rot_mat(i, 0) = temp(i, 0);
        rot_mat(i, 1) = temp(i, 1);
        rot_mat(i, 2) = temp(i, 2);
        task_frame_to_robot_frame.p[i] = temp(i, 3);
    }
    conversions::Matx33dToKdlRot(rot_mat, task_frame_to_robot_frame.M);
    ROS_INFO_STREAM(std::string("-- World To PSM Transformation Calculated: \n")
                            << temp << std::endl);

    return task_frame_to_robot_frame;
}


// -----------------------------------------------------------------------------
//
void ArmToWorldCalibration::PutDrawings(cv::Mat img) {

    std::string instructions;
    if(!calibration_done) {

        // -------------------- draw  calibration points------------------------
        std::vector<cv::Point3d> calib_points_in_world_frame_cv;

        for (uint i = 0; i < calib_points_in_world_frame.size(); i++) {
            calib_points_in_world_frame_cv.push_back(cv::Point3d(
                    calib_points_in_world_frame[i][0],
                    calib_points_in_world_frame[i][1],
                    calib_points_in_world_frame[i][2]));
        }

        std::vector<cv::Point2d> calib_points_screen;
        cv::projectPoints(calib_points_in_world_frame_cv,
                          cam_rvec, cam_tvec,
                          cam_mat, dist_mat, calib_points_screen);

        for (uint i = 0; i < calib_points_in_world_frame.size(); i++) {
            cv::circle(img, calib_points_screen[i], 6, cv::Scalar(255, 255, 0),
                       1, CV_AA);
        }

        // draw the current target with a different colou
        cv::circle(img, calib_points_screen[calib_points_in_arm_frame.size()],
                   6, cv::Scalar(0, 0, 255), 1, CV_AA);
        // --------------------------------------------------------------------

        instructions = "Point at the red point with the tooltip, then press "
                "'c'. Press 'Esc' to exit";


        std::stringstream tool_pos_msg;
        tool_pos_msg <<"Tool point at x: "<< arm_pose_in_robot_frame.p[0]
                     << ", y: "<< arm_pose_in_robot_frame.p[1]
                     << ", z:" << arm_pose_in_robot_frame.p[2];

        cv::putText(img, tool_pos_msg.str(), cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 0), 2);
    }
    else { // if(!calibration_done)
        instructions = "Calibration finished. Press 'f' to exit";

        // ---------------------- draw the tool tip frame ----------------------
        KDL::Frame arm_pose_world_frame = world_to_arm_tr.Inverse() *
                                          arm_pose_in_robot_frame;

        DrawCoordinateFrameInTaskSpace( img, cam_mat, dist_mat,
                                        arm_pose_world_frame,
                                        cam_rvec, cam_tvec, 0.01);

    }

    // draw the instructions
    cv::putText(img, instructions, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 0), 2);

    // draw the coordinate frame of the board
    DrawCoordinateFrameInTaskSpace(img, cam_mat, dist_mat, KDL::Frame(),
                                   cam_rvec, cam_tvec, 0.02);

}


// -----------------------------------------------------------------------------
//
void ArmToWorldCalibration::DrawCoordinateFrameInTaskSpace(
        const cv::InputOutputArray &image, const cv::Mat cam_mat,
        const cv::Mat dist_mat,
        const KDL::Frame frame,
        const cv::Vec3d &rvec, const cv::Vec3d &tvec,
        float length){

    CV_Assert(image.getMat().total() != 0 &&
              (image.getMat().channels() == 1 || image.getMat().channels() == 3));
    CV_Assert(length > 0);

    // project axis points
    KDL::Vector x_axis = frame.p +  length * frame.M.UnitX();
    KDL::Vector y_axis = frame.p +  length * frame.M.UnitY();
    KDL::Vector z_axis = frame.p +  length * frame.M.UnitZ();

    std::vector<cv::Point3f> axisPoints {
            cv::Point3f( (float)frame.p[0],  (float)frame.p[1],  (float)frame.p[2]),
            cv::Point3f( (float)x_axis[0],  (float)x_axis[1],   (float)x_axis[2]),
            cv::Point3f( (float)y_axis[0],  (float)y_axis[1],   (float)y_axis[2]),
            cv::Point3f( (float)z_axis[0],  (float)z_axis[1],   (float)z_axis[2]),
    };


    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cam_mat,
                      dist_mat, imagePoints);

    // draw axis lines
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 200), 2, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 200, 0), 2, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(200, 0, 0), 2, CV_AA);
}
