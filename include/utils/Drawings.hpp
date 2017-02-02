//
// Created by nearlab on 14/12/16.
//

#ifndef TELEOP_VISION_DRAWTOOLS_HPP
#define TELEOP_VISION_DRAWTOOLS_HPP

#include <vector>

#include <opencv2/core.hpp>

#include <geometry_msgs/Pose.h>
#include "utils/Conversions.hpp"
#include "kdl/frames.hpp"



class Drawings {

    typedef geometry_msgs::Pose::_position_type position_t;

public:
    Drawings(
        const cv::Mat _camMatrix,
        const cv::Mat _distCoeffs,
        const KDL::Frame _br_frame,
        const double m_to_px);

    void setSquare(cv::Point3d _center, cv::Point2d _dims) {
        sq_center = _center;
        sq_dims = _dims;
    };

    void drawSquare(cv::InputOutputArray _image);

    void drawToolTip(cv::InputOutputArray _image, position_t position, const cv::Scalar _color);

    void update_cam_2_board_ref(cv::Vec3d _bc_rvec, cv::Vec3d _bc_tvec) {
        board_to_cam_rvec = _bc_rvec;
        board_to_cam_tvec = _bc_tvec;
    }

    void update_board_to_robot_frame(KDL::Frame br_frame){
    	board_to_robot_transl = cv::Point3d(br_frame.p[0], br_frame.p[1], br_frame.p[2]);
    	conversions::kdlRotToMatx33d(br_frame.M, board_to_robot_rotm);
    }

    // transform and save to curr_tool_boardrf and dest_tool_boardrf
    void transfromToBoard(const geometry_msgs::Pose &_tool_curr_in_slvrf,
        const geometry_msgs::Pose &_tool_dest_in_slvrf);

    // save the position and the color of the users position
    // color is from blue to red showing the amount of penetration in z.
    void recordUsersPath(bool _is_tool_in_touch);

    // draw the points with the colors recorded as the user path
    void drawUsersPath(cv::InputOutputArray _image);

    // resets the points and the colors saved as the user path
    void resetUsersPath();

    // set a notification that will be shown for _duration_counts counts.
    void setTransientNotification(cv::String _msg, double _origin_x, double _origin_y,
        unsigned int _duration_counts);

    // show the transient notification
    void showTransientNotification(cv::InputOutputArray _image);

    // alert the user about the worksapce limit with a message
    void showWorkspaceAlert(cv::InputOutputArray _image);

    bool isToolInTouch(double _depth);

public:

    cv::Mat camMatrix;
    cv::Mat distCoeffs;

    cv::Vec3d board_to_cam_rvec;
    cv::Vec3d board_to_cam_tvec;

    cv::Point3d curr_tool_boardrf;
    cv::Point3d dest_tool_boardrf;

    std::vector <cv::Point3d> users_path;
    std::vector <cv::Scalar> users_path_colors;
    bool new_task;
    cv::Point3d board_to_robot_transl;
    cv::Matx33d board_to_robot_rotm;

    double m_to_px;
    // draw the AC
    cv::Point3d sq_center;
    cv::Point2d sq_dims;
    std::vector<cv::Point3d> sq_points_in_board;
    std::vector <cv::Point3d> ac_points_in_board;
    std::vector <cv::Point3d> ac_points_in_robot;

    // messaging
    unsigned int notification_counter;
    cv::String transient_notification_msg;
    cv::Point textOrigin;
};


#endif //TELEOP_VISION_DRAWTOOLS_HPP
