/*
* calib_board_robot.hpp
*
*  Created on: Jun 9, 2016
*      Author: nima
*/

#ifndef TELEOP_VISION_SRC_BOARD_AC_HPP_
#define TELEOP_VISION_SRC_BOARD_AC_HPP_

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "kdl/frames.hpp"
#include "geometry_msgs/Pose.h"
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>


using namespace std;
using namespace cv;


const Scalar RED(50, 0, 255), GREEN1(30, 220, 10), GREEN2(80, 150, 10), BLUE1(220, 100, 20), BLUE2(
    200, 130, 20), CYAN(255, 255, 0), ORANGE(65, 64, 255);

//-----------------------------------------------------------------------------------
// BOARD DETECtOR CLASS
//-----------------------------------------------------------------------------------

class boardDetector {
public:
    boardDetector(
        int _markersX,
        int _markersY,
        float _markerLength,
        float _markerSeparation,
        int _dictionaryId,
        Mat &_camMatrix,
        Mat &_distCoeffs,
        double _n_avg);

    void detect(Vec3d &_rvec, Vec3d &_tvec);

    void drawAxis();

    void drawDetectedMarkers();

    static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);


public:
    int markersX;
    int markersY;
    float markerLength;
    float markerSeparation;
    int dictionaryId;
    float axisLength;
    bool refindStrategy;
    int markersOfBoardDetected;
    bool ready;

    // number of averaging points to prevent board frame oscillation
    double n_avg;
    unsigned int init_counter;

    Mat image;
    Mat camMatrix, distCoeffs;
    vector<int> ids;
    vector<vector<Point2f> > corners, rejected;
    Vec3d rvec, tvec;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::GridBoard> gridboard;
    Ptr<aruco::Board> board;
};



//-----------------------------------------------------------------------------------
// ROSOBJ CLASS
//-----------------------------------------------------------------------------------

class rosObj {
public:

    rosObj(int argc, char *argv[], string n_name);

    void init();

    void toolCurrCallback(const geometry_msgs::Pose::ConstPtr &msg);

    void toolDestCallback(const geometry_msgs::Pose::ConstPtr &msg);

    void eventsCallback(const std_msgs::Char::ConstPtr &msg);

public:
    bool all_good;
    bool new_event;
    bool msg_ws_alert;
    bool acqusition_running;

    std::string cam_data_path_param;
    std::string tool_curr_topic_name_param;
    std::string tool_dest_topic_name_param;
    std::string events_topic_name_param;
    std::string node_name;
    ros::NodeHandle n;
    double freq_ros;
    int camId_param;
    std_msgs::Char acquisition_event;
//	ros::Subscriber sub_robot;
    geometry_msgs::Pose tool_curr_pose_in_slvrf;
    geometry_msgs::Pose tool_dest_pose_in_slvrf;
    sensor_msgs::PointCloud2 ac_point_cloud;
    vector<double> board_to_robot_tr_param;
    int markersX_param;
    int markersY_param;
    float markerLength_px_param;
    float markerSeparation_px_param;
    int dictionaryId_param;
    float markerlength_m_param;
};


//-----------------------------------------------------------------------------------
// CALIBBOARDROBOT CLASS
//-----------------------------------------------------------------------------------
class calibBoardRobot {

public:

    calibBoardRobot() {
        calib_status = 0;
        message = "Point at point 1 and press 'space' to start calibration";
    };

    bool updateMe(string &msg, cv::Mat img);

    void setCalibpoint(double x, double y, double z);

    void make_tr(vector<Point3d> axisPoints, Matx33d &_rotm, Vec3d &br_tvec);

    void get_tr(Matx33d &_rotm, Vec3d &br_tvec);

    void setVisualPoints(vector<Point2d> in) { visual_calib_points = in; };

    void reset();

private:
    vector<Point3d> axisPoints;
    bool calib_status;
    string message;
    Vec3d calib_tvec;
    Matx33d calib_rotm;
    vector<Point2d> visual_calib_points;
};




//-----------------------------------------------------------------------------------
// AC CLASS
//-----------------------------------------------------------------------------------

class drawings {

public:
    drawings(
        const Mat _camMatrix,
        const Mat _distCoeffs,
        const KDL::Frame _br_frame,
        const double m_to_px);

//	ac_square(Point3d _center, Point2d _dims) {	center = _center;
//	dims = _dims;};

    void setSquare(Point3d _center, Point2d _dims) {
        sq_center = _center;
        sq_dims = _dims;
    };


    // drawing a simple rectangle
    void create3dCurve(double _center_x, double _center_y, unsigned int n_points);

    // create an ellipse save in ac_points_in_board and convert and save in ac_points_in_robot
    void
    createEllipse(double _center_x, double _center_y, double a, double b, unsigned int n_points);

    // create an ellipse save in ac_points_in_board and convert and save in ac_points_in_robot
    void createSine(double _center_x, double _center_y, double a, double b, unsigned int n_points);

    // create an ellipse save in ac_points_in_board and convert and save in ac_points_in_robot
    void createHat(double _center_x, double _center_y, double a, double b, unsigned int n_points);

    void drawSquare(InputOutputArray _image);

    void drawToolTip(InputOutputArray _image, double _x, double _y, double _z, const Scalar _color);

    // showing the projection of the current tool pose to the desired point
    void drawGuidingLines(InputOutputArray _image, const geometry_msgs::Pose &tool,
        const geometry_msgs::Pose &desired);

    void drawACPoints(InputOutputArray _image, bool _is_tool_in_touch);

    void update_cam_2_board_ref(Vec3d _bc_rvec, Vec3d _bc_tvec) {
        bc_rvec = _bc_rvec;
        bc_tvec = _bc_tvec;
    }

    // generate a pointCloud2 message from the ac_points_in_robot
    void getACCloud(sensor_msgs::PointCloud2 &cloud_out);

    // transform and save to curr_tool_boardrf and dest_tool_boardrf
    void transfromToBoard(const geometry_msgs::Pose &_tool_curr_in_slvrf,
        const geometry_msgs::Pose &_tool_dest_in_slvrf);

    // save the position and the color of the users position
    // color is from blue to red showing the amount of penetration in z.
    void recordUsersPath(bool _is_tool_in_touch);

    // draw the points with the colors recorded as the user path
    void drawUsersPath(InputOutputArray _image);

    // resets the points and the colors saved as the user path
    void resetUsersPath();

    // set a notification that will be shown for _duration_counts counts.
    void setTransientNotification(string _msg, double _origin_x, double _origin_y,
        unsigned int _duration_counts);

    // show the transient notification
    void showTransientNotification(InputOutputArray _image);

    // alert the user about the worksapce limit with a message
    void showWorkspaceAlert(InputOutputArray _image);

    bool isToolInTouch(double _depth);

public:

    Mat camMatrix;
    Mat distCoeffs;

    Vec3d bc_rvec;
    Vec3d bc_tvec;

    Point3d curr_tool_boardrf;
    Point3d dest_tool_boardrf;

    vector<Point3d> users_path;
    vector<Scalar> users_path_colors;
    bool new_task;
//	KDL::Frame br_frame;
    Point3d br_trans;
    Matx33d br_rotm;

    double m_to_px;
    // draw the AC
    Point3d sq_center;
    Point2d sq_dims;
    vector<Point3d> sq_points_in_board;
    vector<Point3d> ac_points_in_board;
    vector<Point3d> ac_points_in_robot;


    // messaging
    unsigned int notification_counter;
    string transient_notification_msg;
    Point textOrigin;

};

namespace conversions {

// some self explanetory conversions!
    void rvecTokdlRot(const cv::Vec3d _rvec, KDL::Rotation &_kdl);

    void rvectvecToKdlFrame(const cv::Vec3d _rvec, const cv::Vec3d _tvec, KDL::Frame &_kdl);

    void matx33dToKdlRot(const cv::Matx33d _mat, KDL::Rotation &_kdl);

    void kdlRotToMatx33d(const KDL::Rotation _kdl, cv::Matx33d &_mat);

    void poseMsgToVector(const geometry_msgs::Pose in_pose, vector<double> &out_vec);

    void vectorToPoseMsg(const vector<double> in_vec, geometry_msgs::Pose &out_pose);


};

//-----------------------------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------------------------

// drawing a simple box frame
void drawCube(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
    InputArray _rvec, InputArray _tvec);


// operator overload to print out vectors
ostream &operator<<(ostream &out, const vector<double> &vect);


#endif /* TELEOP_VISION_SRC_BOARD_AC_HPP_ */
