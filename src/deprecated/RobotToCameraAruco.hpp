/*
 * Calibrator.h
 *
 *  Created on: Dec 13, 2016
 *      Author: nima
 */

#ifndef SRC_UTILS_CALIBBOARDROBOT_HPP_
#define SRC_UTILS_CALIBBOARDROBOT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <tf_conversions/tf_kdl.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>


#include "src/ac_overlay_cv/Drawings.h"
#include "utils/Conversions.hpp"

class RobotToCameraAruco {
public:
    RobotToCameraAruco(std::string node_name);

    void DrawToolTarget(cv::String &instructions, cv::Mat img);

    void Calib2DrawTarget(cv::String &instructions, cv::Mat img);
    void Calib2SaveCalibrationPoint();
    void Calib2CalculateTransformation();

    void Calib1DrawCalibrationAxis(cv::String &instructions, cv::Mat img);
    void Calib1SaveCalibrationPoint();
    void Calib1CalculateTransformation(const std::vector<Eigen::Vector3d> axis_points,
                                       KDL::Frame &transformation);

    void SetROSParameters();

    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    FindAxis(Eigen::MatrixXd axis_points);

    void Reset();

    bool IsCalibrated() { return calib_finished; }

    // ROS Callbacks
    void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void LeftCameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void RightCameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr
                                &msg);

    cv::Mat &Image(ros::Duration timeout);

    // Eddy Andre addition
    const double length_x = 0.0782, length_y = 0.0514;
    const double  aruco_marker_length_in_meters = 0.023, aruco_marker_separation_in_meters = 0.0;
    std::vector< Eigen::Vector3d> meas_points;
    //std::vector< Eigen::Vector3d> points_on_camera;
    Eigen::Matrix<double, 3, 6> target_mat;
    Eigen::Matrix<double, 3, 6> meas_points_mat;
    Eigen::Matrix4d camera_to_robot;
    //


public:
    CameraIntrinsics camera_intrinsics;
    int cam_id; // 0 for left, 1 for right
    double ros_freq = 0.0;
    KDL::Frame task_frame_to_cam_frame[2];
    KDL::Frame tool_pose_in_robot_frame;
    KDL::Frame tool_pose_in_task_frame;
    KDL::Frame task_frame_to_robot_frame;
    bool task_frame_to_robot_frame_param_present;

    cv::Vec3d task_frame_to_cam_rvec[2], task_frame_to_cam_tvec[2];

private:
    void SetupROS();

    void ReadCameraParameters(const std::string file_path,
                              CameraIntrinsics & camera);

    // MUST BE EVEN NUMBER. The number of points recorded for calibration. default value of 5 is used
    // if no parameter is found on the ros server.
    int num_calib_points = 6;

    // the length of the axis line in [m] shown on the image
    double visual_axis_length = 0.1;

    std::vector<Eigen::Vector3d> measured_points;

    bool calib_finished;

    ros::NodeHandle n;
    ros::Subscriber left_camera_pose_subscriber;
    ros::Subscriber right_camera_pose_subscriber;
    ros::Subscriber robot_pose_subscriber;

    image_transport::Subscriber camera_image_subscriber;
    image_transport::ImageTransport *it;


    // Eddy Andre addition
    //
    //
    //
    std::vector<cv::Point3d> target;
    std::vector<cv::Point2d> calib_points_screen;
    //
    //
    //
    std::vector<cv::Point3d> calib_points_in_board_frame;

    cv::Mat image_msg;

};

///////// Defining the templates

// taken from https://gist.github.com/ialhashim/0a2554076a6cf32831ca
template <class Vector3>
std::pair<Vector3, Vector3> FitLineToPoints(const std::vector<Vector3> &c) {
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix<typename Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic>
            centers(num_atoms, 3);
    for (size_t i = 0; i < num_atoms; ++i)
        centers.row(i) = c[i];

    Vector3 origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Vector3 axis = eig.eigenvectors().col(2).normalized();

    return std::make_pair(origin, axis);
}

template <class Vector3>
std::pair<Vector3, Vector3> FitPlaneToPoints(const std::vector<Vector3> &c) {
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix<typename Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic>
            coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i)
        coord.col(i) = c[i];

    // calculate centroid
    Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(),
                     coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0);
    coord.row(1).array() -= centroid(1);
    coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    Vector3 plane_normal = svd.matrixU().rightCols(1);
    return std::make_pair(centroid, plane_normal);
}

// operator overload to print out vectors
std::ostream &operator<<(std::ostream &out, const std::vector<double> &vect);

#endif /* SRC_UTILS_CALIBBOARDROBOT_HPP_ */
