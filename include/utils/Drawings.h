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
    /*!
    * \briefUses opencv line function to draw a cube in task coordinate frame
    *
    * \param rvec is the orientation of the task coordinate frame in camera frame
    * \param tvec is the the position of the task coordinate frame in camera frame
    */
    void DrawCube(cv::InputOutputArray image,
                  const CameraIntrinsics &cam_intrinsics,
                  const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                  const cv::Point3d &origin, const cv::Point3d &whd,
                  const cv::Scalar &color);

    /*!
    * \brief Draws a circle at the tool tip. Position is in task coordinate
    * frame (task-space).
    * \param  rvec is the orientation of the task coordinate frame in camera frame
    * \param tvec is the the position of the task coordinate frame in camera frame
     */
    void DrawPoint(cv::InputOutputArray image,
                   const CameraIntrinsics &cam_intrinsics,
                   const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                   KDL::Vector position,
                   const cv::Scalar color);


    /*!
    * \brief Uses opencv circle to draw a point at each element of the input vector.
     *
     * \param rvec is the orientation of the task coordinate frame in camera frame
     * \param tvec is the the position of the task coordinate frame in camera frame
     */
    void DrawPoint3dVector(cv::InputOutputArray image,
                           const std::vector<cv::Point3d> &points,
                           const CameraIntrinsics &cam_intrinsics,
                           const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                           const cv::Scalar color);

    void DrawLineFrom2KDLPoints(cv::InputOutputArray image,
                                const CameraIntrinsics &cam_intrinsics,
                                const cv::Vec3d &rvec,
                                const cv::Vec3d &tvec,
                                KDL::Vector current,
                                KDL::Vector desired,
                                const cv::Scalar color);

    /*!
     * \brief Uses opencv line to draw a coordinate frame seen from task coordinate frame
     *
     * \param frame  the frame expressed in task coordinate frame
     * \param rvec is the orientation of the task coordinate frame in camera frame
     * \param tvec is the the position of the task coordinate frame in camera frame
     */
    void DrawCoordinateFrameInTaskSpace(
            const cv::InputOutputArray &image, const CameraIntrinsics &cam_intrinsics,
            const KDL::Frame frame,
            const cv::Vec3d &rvec, const cv::Vec3d &tvec,
            float length);
}


#endif //TELEOP_VISION_DRAWINGS_H
