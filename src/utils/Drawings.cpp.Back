//
// Created by nearlab on 14/12/16.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "kdl/frames.hpp"

#include "utils/Colors.hpp"
#include "utils/Drawings.hpp"
#include "utils/Conversions.hpp"

using namespace std;
using namespace cv;

Drawings::Drawings(
    const Mat _camMatrix,
    const Mat _distCoeffs,
    const KDL::Frame _br_frame,
    const double _m_to_px)
{
    camMatrix = _camMatrix;
    distCoeffs = _distCoeffs;

    board_to_robot_transl = Point3d(_br_frame.p[0], _br_frame.p[1], _br_frame.p[2]);
    conversions::KDLRotToMatx33d(_br_frame.M, board_to_robot_rotm);

    m_to_px = _m_to_px;
    new_task = true;
    notification_counter = 0;
}


void Drawings::drawSquare(InputOutputArray _image) {

    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));

    // project axis points
    sq_points_in_board.push_back(
        Point3f(sq_center.x - sq_dims.x / 2, sq_center.y - sq_dims.y / 2, sq_center.z));
    sq_points_in_board.push_back(
        Point3f(sq_center.x + sq_dims.x / 2, sq_center.y - sq_dims.y / 2, sq_center.z));
    sq_points_in_board.push_back(
        Point3f(sq_center.x + sq_dims.x / 2, sq_center.y + sq_dims.y / 2, sq_center.z));
    sq_points_in_board.push_back(
        Point3f(sq_center.x - sq_dims.x / 2, sq_center.y + sq_dims.y / 2, sq_center.z));

    vector<Point2d> imagePoints;
    projectPoints(sq_points_in_board, board_to_cam_rvec, board_to_cam_tvec, camMatrix, distCoeffs, imagePoints);

    // draw axis lines
    for (int i = 0; i < 3; i++) {
        line(_image, imagePoints[i], imagePoints[i + 1], Scalar(200, 100, 10), 1);
    }
    line(_image, imagePoints[3], imagePoints[0], Scalar(200, 100, 10), 1);

}


void Drawings::drawToolTip(InputOutputArray _image, position_t position, const Scalar _color) {

    //	Point3d board_to_robot_transl = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
    Point3d toolPoint3d_rrf = Point3d(position.x, position.y, position.z);

    // taking the robot tool tip from the robot ref frame to board ref frame and convert to pixles from meters
    Point3d temp = board_to_robot_rotm.t() * (toolPoint3d_rrf - board_to_robot_transl);
    Point3d toolPoint3d_crf = Point3d(temp.x * m_to_px, temp.y * m_to_px, temp.z * m_to_px);

    vector<Point3d> toolPoint3d_vec_crf;
    vector<Point2d> toolPoint2d;
    toolPoint3d_vec_crf.push_back(toolPoint3d_crf);

    projectPoints(toolPoint3d_vec_crf, board_to_cam_rvec, board_to_cam_tvec, camMatrix, distCoeffs, toolPoint2d);
    circle(_image, toolPoint2d[0], 2, _color, -1);

}


//--------------------------------------------------------------------------------------------
// Get cloud points of the ac
//--------------------------------------------------------------------------------------------
void Drawings::transfromToBoard(const geometry_msgs::Pose &_tool_curr_in_slvrf,
    const geometry_msgs::Pose &_tool_dest_in_slvrf) {

    //	Point3d board_to_robot_transl = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
    Point3d toolPoint3d_slvrf = Point3d(_tool_curr_in_slvrf.position.x,
        _tool_curr_in_slvrf.position.y, _tool_curr_in_slvrf.position.z);
    Point3d desPoint3d_slvrf = Point3d(_tool_dest_in_slvrf.position.x,
        _tool_dest_in_slvrf.position.y, _tool_dest_in_slvrf.position.z);

    // taking the robot tool tip from the robot ref frame to board ref frame and convert to pixels from meters
    Point3d temp = board_to_robot_rotm.t() * (toolPoint3d_slvrf - board_to_robot_transl);
    curr_tool_boardrf = Point3d(temp.x * m_to_px, temp.y * m_to_px, temp.z * m_to_px);

    temp = board_to_robot_rotm.t() * (desPoint3d_slvrf - board_to_robot_transl);
    dest_tool_boardrf = Point3d(temp.x * m_to_px, temp.y * m_to_px, temp.z * m_to_px);

}


//--------------------------------------------------------------------------------------------
// setTransientNotification
//--------------------------------------------------------------------------------------------
void Drawings::setTransientNotification(cv::String _msg, double _origin_x, double _origin_y,
    unsigned int _duration_counts) {

    textOrigin = Point(_origin_x, _origin_y);
    transient_notification_msg = _msg;
    notification_counter = _duration_counts;


}


//--------------------------------------------------------------------------------------------
// showTransientNotification
//--------------------------------------------------------------------------------------------
void Drawings::showTransientNotification(InputOutputArray _image) {


    if (notification_counter > 0) {
        putText(_image, transient_notification_msg, textOrigin, 1, 1, Colors::Blue);
        notification_counter--;
    }
}
