//
// Created by charm on 3/19/17.
//

#include <iostream>
#include "Drawings.h"


void DrawingsCV::DrawCube(cv::InputOutputArray image,
                          const CameraIntrinsics &cam_intrinsics,
                          const cv::Vec3d &rvec,
                          const cv::Vec3d &tvec,
                          const cv::Point3d &origin,
                          const cv::Point3d &whd,
                          const cv::Scalar &color){

    CV_Assert(image.getMat().total() != 0 &&
              (image.getMat().channels() == 1 || image.getMat().channels() == 3));

    // project axis points
    std::vector<cv::Point3f> axisPoints;

    axisPoints.push_back(cv::Point3d(origin.x, origin.y + whd.y, origin.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y + whd.y, origin.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y, origin.z));
    axisPoints.push_back(cv::Point3d(origin.x, origin.y, origin.z));

    axisPoints.push_back(cv::Point3d(origin.x, origin.y + whd.y, origin.z + whd.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y + whd.y, origin.z + whd.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y, origin.z + whd.z));
    axisPoints.push_back(cv::Point3d(origin.x, origin.y, origin.z + whd.z));

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec,
                      cam_intrinsics.camMatrix, cam_intrinsics.distCoeffs, imagePoints);

    // draw axis lines
    int points[7] = {0, 1, 2, 4, 5, 6};
    for (int i = 0; i < 6; i++) {
        cv::line(image, imagePoints[points[i]], imagePoints[points[i] + 1],
                 color, 2, CV_AA);
    }
    for (int i = 0; i < 4; i++) {
        line(image, imagePoints[i], imagePoints[i + 4], color, 2, CV_AA);
    }
    line(image, imagePoints[3], imagePoints[0], color, 2, CV_AA);
    line(image, imagePoints[7], imagePoints[4], color, 2, CV_AA);

}


void DrawingsCV::DrawPoint(cv::InputOutputArray image,
                           const CameraIntrinsics &cam_intrinsics,
                           const cv::Vec3d &rvec,
                           const cv::Vec3d &tvec,
                           KDL::Vector position,
                           const cv::Scalar color) {

    std::vector<cv::Point3d> toolPoint3d_vec_crf;
    std::vector<cv::Point2d> toolPoint2d;
    toolPoint3d_vec_crf.push_back(cv::Point3d(position[0] , position[1] , position[2]));

    projectPoints(toolPoint3d_vec_crf, rvec, tvec, cam_intrinsics.camMatrix,
                  cam_intrinsics.distCoeffs, toolPoint2d);
    circle(image, toolPoint2d[0], 5, color, -1);

}



void DrawingsCV::DrawACPath(cv::InputOutputArray image,
                            const std::vector<cv::Point3d> &ac_path,
                            const CameraIntrinsics &cam_intrinsics,
                            const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                            const cv::Scalar color){
    if(ac_path.size() > 0) {

        std::vector<cv::Point2d> ac_points_2d;
        projectPoints(ac_path, rvec, tvec, cam_intrinsics.camMatrix,
                      cam_intrinsics.distCoeffs, ac_points_2d);

        for (int i = 0; i < ac_path.size(); ++i) {
            circle(image, ac_points_2d[i], 2, color, -1);
        }
    }
}


void DrawingsCV::DrawCurrentToDesiredLine(cv::InputOutputArray image,
                                          const CameraIntrinsics &cam_intrinsics,
                                          const cv::Vec3d &rvec,
                                          const cv::Vec3d &tvec,
                                          KDL::Vector current,
                                          KDL::Vector desired,
                                          const cv::Scalar color) {

    std::vector<cv::Point3d> points_3d;
    std::vector<cv::Point2d> points_2d;
    points_3d.push_back(cv::Point3d(current[0] , current[1] , current[2]));
    points_3d.push_back(cv::Point3d(desired[0] , desired[1] , desired[2]));

    projectPoints(points_3d, rvec, tvec, cam_intrinsics.camMatrix,
                  cam_intrinsics.distCoeffs, points_2d);
    line(image, points_2d[0], points_2d[1], color, 2, CV_AA);

    circle(image, points_2d[0], 5, color, -1);
    circle(image, points_2d[1], 5, color, -1);

}
