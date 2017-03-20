/*
 * Calibrator.h
 *
 *  Created on: Dec 13, 2016
 *      Author: nima
 */

#ifndef SRC_UTILS_CALIBBOARDROBOT_HPP_
#define SRC_UTILS_CALIBBOARDROBOT_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf_conversions/tf_kdl.h>
#include <image_transport/image_transport.h>

struct CameraIntrinsics {
    cv::Mat camMatrix;
    cv::Mat distCoeffs;
};

class RobotToCameraAruco {
public:
    RobotToCameraAruco(std::string node_name);

    /**
     * Draw a circle representing the next tool target
     *
     * @param instructions [out] A message to write on the screen
     * @param img [out] The current back-buffer on which to draw the target
     */
    void DrawToolTarget(cv::String &instructions, cv::Mat img);

    void SaveCalibrationPoint(geometry_msgs::Pose::_position_type position);

    void MakeTr(std::vector<cv::Point3d> axisPoints, cv::Matx33d &_rotm,
                cv::Vec3d &br_tvec);

    void GetTr();

    void CalculateTransformationN(std::vector<Eigen::Vector3d> axis_points,
                                  cv::Matx33d &rotm, cv::Vec3d &br_tvec);

    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    FindAxis(const Eigen::MatrixXd &axis_points);

    //    template<typename Vector3>
    //    std::pair<Vector3, Vector3> FitLineToPoints(const
    //    std::vector<Vector3> & c);
    //
    ////    template<class Vector3>
    ////    std::pair<Vector3, Vector3> FitPlaneToPoints(const
    /// std::vector<Vector3> & c);
    //

    void DrawCalibrationAxis(cv::String &instructions, cv::Mat img);
    void Reset();

    bool IsCalibrated() { return calib_finished; }

    // ROS Callbacks
    void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void CameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    cv::Mat &Image(ros::Duration timeout);

public:
    CameraIntrinsics camera_intrinsics;
    double ros_freq = 0.0;
    geometry_msgs::PoseStamped camera_pose;
    geometry_msgs::PoseStamped robot_pose;
    KDL::Frame board_to_cam_frame;
    KDL::Frame board_to_robot_frame;

    cv::Vec3d board_to_cam_rvec, board_to_cam_tvec;

private:
    void GetROSParameterValues();

    void ReadCameraParameters(std::string file_path);

    // the number of points recorded per axis. default value of 5 is used
    // if no parameter is found on the ros server.
    int num_points_per_axis = 5;

    // the length of the axis line in [m] shown on the image
    double visual_axis_length = 0.1;

    ros::NodeHandle n;
    ros::Subscriber camera_pose_subscriber;
    ros::Subscriber robot_pose_subscriber;

    image_transport::Subscriber camera_image_subscriber;
    image_transport::ImageTransport *it;

    std::vector<cv::Point3d> axis_points_Old;
    std::vector<Eigen::Vector3d> measured_points;

    bool calib_finished;
    std::vector<cv::Point3d> calib_points_in_board_frame;

    cv::Mat image_msg;
    cv::Vec3d calib_tvec;
    cv::Matx33d calib_rotm;
    std::vector<cv::Point2d> calib_points_screen;
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
