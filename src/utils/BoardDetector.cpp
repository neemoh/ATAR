/*
 * BoardDetector.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: nima
 */

#include <ros/ros.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <utils/Colors.hpp>
#include "utils/BoardDetector.hpp"

//-----------------------------------------------------------------------------------
// BOARD DETECTOR CLASS METHODS
//-----------------------------------------------------------------------------------

BoardDetector::BoardDetector(ArucoBoard board, CameraIntrinsics camera, double n_avg)
        : board(board),
          n_avg_steps(n_avg),
          camera_(camera)
{
    aruco_.DetectorParams = cv::aruco::DetectorParameters::create();
    aruco_.DetectorParams->doCornerRefinement = true;

    aruco_.Dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME(board.DictionaryID));

    aruco_.Gridboard = cv::aruco::GridBoard::create(
            board.Height, board.Width, board.MarkerLength, board.MarkerSeparation,
            aruco_.Dictionary
    );
    aruco_.Board = aruco_.Gridboard.staticCast<cv::aruco::Board>();
}


void BoardDetector::Detect(cv::InputOutputArray &image) {

    cv::Vec3d rotation_current, translation_current;

    if (image.empty())
        ROS_ERROR("Empty image received. Is an image source present?");

    // detect markers
    cv::aruco::detectMarkers(
            image, aruco_.Dictionary, aruco_.DetectedCorners, aruco_.DetectedMarkerIds,
            aruco_.DetectorParams, aruco_.RejectedCorners);

    // Should we try to find the other markers ?
    if (aruco_.RefindStrategy)
        cv::aruco::refineDetectedMarkers(
                image, aruco_.Board, aruco_.DetectedCorners, aruco_.DetectedMarkerIds,
                aruco_.RejectedCorners, camera_.camMatrix, camera_.distCoeffs
        );

    // if we have found some markers on the image we can estimate the pose
    if (aruco_.DetectedMarkerIds.size() > 0) {

        try{
            // "The returned transformation is the one that transforms points from
            // the board coordinate system to the camera coordinate system."
            auto num_markers_used = cv::aruco::estimatePoseBoard(
                    aruco_.DetectedCorners, aruco_.DetectedMarkerIds, aruco_.Board,
                    camera_.camMatrix, camera_.distCoeffs,
                    rotation_current, translation_current);
            board_detected_ = (num_markers_used > 0);
        }
        catch (const cv::Exception& e){
            ROS_ERROR("Something went wrong in cv::aruco::estimatePoseBoard");
        }

        // average approximation of the orientation to fix the oscillation of the axes
        if (n_avg_steps == 0.0){
            //no averaging
            rvec = rotation_current;
            tvec = translation_current;
        } else {
            rvec -= rvec / n_avg_steps;
            rvec += rotation_current / n_avg_steps;

            // averaging the origin position
            tvec -= tvec / n_avg_steps;
            tvec += translation_current / n_avg_steps;
        }


        // wait for the initialization of the averaging (4 times the avg window should be enough)
        if (frame_counter_ < n_avg_steps * 4){
            frame_counter_++;
            board_detected_ = false;
        }


    } else
        board_detected_ = false;

}


void BoardDetector::DrawAxis(cv::InputOutputArray &image) {
    if (board_detected_)
        drawAxisAntiAliased(image, camera_.camMatrix, camera_.distCoeffs, rvec, tvec, 0.01);
}


void BoardDetector::drawAxisAntiAliased(
        const cv::_InputOutputArray &_image, const cv::_InputArray &camera_Matrix,
        const cv::_InputArray &_distCoeffs, const cv::_InputArray &_rvec, const cv::_InputArray &_tvec,
        float length)
{
    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert(length > 0);

    // project axis points
    std::vector<cv::Point3f> axisPoints {
            cv::Point3f( 0,  0,  0),
            cv::Point3f( 1,  0,  0) * length,
            cv::Point3f( 0,  1,  0) * length,
            cv::Point3f( 0,  0,  1) * length
    };

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, _rvec, _tvec, camera_Matrix, _distCoeffs, imagePoints);

    // draw axis lines
    cv::line(_image, imagePoints[0], imagePoints[1], Colors::Blue, 2, CV_AA);
    cv::line(_image, imagePoints[0], imagePoints[2], Colors::Green, 2, CV_AA);
    cv::line(_image, imagePoints[0], imagePoints[3], Colors::Red, 2, CV_AA);
}


void BoardDetector::DrawDetectedMarkers(cv::InputOutputArray &image) {
    if (aruco_.DetectedMarkerIds.size() > 0)
        cv::aruco::drawDetectedMarkers(image, aruco_.DetectedCorners, aruco_.DetectedMarkerIds);
}


void BoardDetector::DetectBoardAndDrawAxis(cv::InputOutputArray &image) {

    if (image.empty())
        throw std::runtime_error("Error: BoardDetector::DetectBoardAndDrawAxis received an empty image.");

    Detect(image);

    if (board_detected_ && board.draw_axes) {
        DrawAxis(image);
        DrawDetectedMarkers(image);
    }
}
