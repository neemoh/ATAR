//
// Created by nima on 16/11/17.
//

#ifndef ATAR_MANIPULATORTOWORLDCALIBRATION_H
#define ATAR_MANIPULATORTOWORLDCALIBRATION_H


#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Manipulator.h"
#include "AugmentedCamera.h"

class ManipulatorToWorldCalibration {
public:
    explicit ManipulatorToWorldCalibration(Manipulator *manip);

    ~ManipulatorToWorldCalibration();

    bool DoCalibration(KDL::Frame & result);

    void PutDrawings(cv::Mat img, bool calibration_done,
                     KDL::Frame manip_pose_loc);

    void DrawCoordinateFrameInTaskSpace(
            const cv::InputOutputArray &image,
            KDL::Frame frame,
            const cv::Vec3d &rvec, const cv::Vec3d &tvec,
            float length);

    KDL::Frame CalculateTransformation(
            std::vector<Eigen::Vector3d> points_in_frame_1,
            std::vector<Eigen::Vector3d> points_in_frame_2);

private:
    AugmentedCamera * ar_camera;
    Manipulator * manipulator;
    image_transport::ImageTransport *it;

    std::vector< Eigen::Vector3d> calib_points_in_world_frame;
    std::vector< Eigen::Vector3d> calib_points_in_arm_frame;

    KDL::Frame world_to_arm_tr;

    cv::Mat cam_matrix;
    cv::Mat cam_distortation;


};


#endif //ATAR_MANIPULATORTOWORLDCALIBRATION_H
