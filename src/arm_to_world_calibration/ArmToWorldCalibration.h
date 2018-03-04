//
// Created by nima on 01/06/17.
//

#ifndef ATAR_ARMTOWORLDCALIBRATION_H
#define ATAR_ARMTOWORLDCALIBRATION_H

#include <iostream>
#include <kdl/frames.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

class ArmToWorldCalibration {

public:
    ArmToWorldCalibration() {};

    bool DoCalibration(
        const std::string img_topic_namespace,
        const std::string cam_pose_topic_namespace,
        const std::string arm_pose_topic_namespace,
        const cv::Mat camera_matrix,
        const cv::Mat dist_coeffs,
        const uint num_calib_points,
        const double calib_points_distance,
        const std::vector<double> calib_points_position_center,
        KDL::Frame &result
    );
private:
    void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void ArmPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void CameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    KDL::Frame CalculateTransformation(
            std::vector<Eigen::Vector3d> points_in_frame_1,
            std::vector<Eigen::Vector3d> points_in_frame_2);

    void PutDrawings(cv::Mat img);

    void DrawCoordinateFrameInTaskSpace(
            const cv::InputOutputArray &image,
            const cv::Mat cam_mat, const cv::Mat dist_mat,
            const KDL::Frame frame,
            const cv::Vec3d &rvec, const cv::Vec3d &tvec,
            float length);

private:

    bool calibration_done = false;
    bool exit = false;

    std::string window_name;
    cv::Vec3d cam_tvec, cam_rvec;
    cv::Mat cam_mat, dist_mat;

    KDL::Frame arm_pose_in_robot_frame;
    KDL::Frame world_to_cam_tr;
    KDL::Frame world_to_arm_tr;

    std::vector< Eigen::Vector3d> calib_points_in_arm_frame;
    std::vector< Eigen::Vector3d> calib_points_in_world_frame;

};


#endif //ATAR_ARMTOWORLDCALIBRATION_H
