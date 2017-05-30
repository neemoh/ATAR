//
// Created by nima on 29/05/17.
//

#ifndef ATAR_INTRINSICCALIBRATIONCHARUCO_H
#define ATAR_INTRINSICCALIBRATIONCHARUCO_H

#include <iostream>
#include <vector>
#include <opencv2/aruco/charuco.hpp>
#include <sensor_msgs/Image.h>

class IntrinsicCalibrationCharuco {
public:
    IntrinsicCalibrationCharuco(std::string img_topic_namespace,
                                std::vector<float> charuco_board_param);


    bool DoCalibration(std::string outputFile,
                       double &repError,
                       cv::Mat &cameraMatrix,
                       cv::Mat &distCoeffs);
    void CameraImageCallback(const
                             sensor_msgs::ImageConstPtr &msg);

private:
    bool saveCameraParams(const std::string
                                 &filename, cv::Size imageSize, float
                                 aspectRatio, int flags,
                                 const cv::Mat &cameraMatrix, const cv::Mat
                                 &distCoeffs, double totalAvgErr);
private:
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    bool finished_capturing = false;

    std::vector< std::vector< std::vector< cv::Point2f > > > allCorners;
    std::vector< std::vector< int > > allIds;
    std::vector< cv::Mat > allImgs;
    cv::Size imgSize;


};


#endif //ATAR_INTRINSICCALIBRATIONCHARUCO_H
