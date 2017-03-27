//
// Created by charm on 3/19/17.
//

#ifndef TELEOP_VISION_DRAWINGS_H
#define TELEOP_VISION_DRAWINGS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <kdl/frames.hpp>

struct CameraIntrinsics {
    cv::Mat camMatrix;
    cv::Mat distCoeffs;
};


namespace DrawingsCV{
    // Uses opencv line function to draw a cube in task coordinate frame
    // rvec is the orientation of the task coordinate frame in camera frame
    // tvec is the the position of the task coordinate frame in camera frame
    void DrawCubeCV(cv::InputOutputArray image,
                    const CameraIntrinsics &cam_intrinsics,
                    const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                    const cv::Point3d &origin, const cv::Point3d &whd,
                    const cv::Scalar &color);

    // Draws a circle at the tool tip. Position is in task coordinate
    // frame (task-space).
    // rvec is the orientation of the task coordinate frame in camera frame
    // tvec is the the position of the task coordinate frame in camera frame
    void DrawToolTipCV(cv::InputOutputArray image,
                       const CameraIntrinsics &cam_intrinsics,
                       const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                       KDL::Vector position,
                       const cv::Scalar color);

    // Uses opencv circle to draw a point at each ac path position point.
    // rvec is the orientation of the task coordinate frame in camera frame
    // tvec is the the position of the task coordinate frame in camera frame
    void DrawACPathCV(cv::InputOutputArray image,
                      const std::vector<cv::Point3d> &ac_path,
                      const CameraIntrinsics &cam_intrinsics,
                      const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                      const cv::Scalar color);

}


#endif //TELEOP_VISION_DRAWINGS_H
