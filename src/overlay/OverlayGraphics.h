//
// Created by charm on 2/21/17.
//

#ifndef TELEOP_VISION_OVERLAYGRAPHICS_H
#define TELEOP_VISION_OVERLAYGRAPHICS_H

//#include <GLFW/glfw3.h>
//#include <GL/glu.h>
#include "ros/ros.h"

#include "opencv2/highgui/highgui.hpp"
#include <kdl_conversions/kdl_msg.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>


typedef struct
{
    unsigned char buttons[8];
    float x;
    float y;
    float z;
}TabletInfo;

struct CameraDistortion {
    cv::Mat camMatrix;
    cv::Mat distCoeffs;
};


class OverlayGraphics {
public:

    OverlayGraphics(std::string node_name, int width, int height);


//    void drawBoundinBox(
//            float x_min, float x_max,
//            float y_min, float y_max,
//            float z_min, float z_max);
//
//    void drawElipsoid(
//            float x_min, float x_max,
//            float y_min, float y_max,
//            float z_min, float z_max);
//
//    void InitGL(int w, int h);
//
//    void Render(GLFWwindow* window,  TabletInfo tablet_info,
//                std::vector<cv::Point2f> safety_area);
//
//    void RenderSide(
//            GLFWwindow* window,
//            KDL::Frame &cameraPose,
//            cv::Mat &cameraMatrix,
//            unsigned char* buffer,
//            int x, int width, int height,
//            GLuint texId,
//            std::vector<cv::Point2f> safety_area);

    // CALLBACKS
    void ImageLeftCallback(const sensor_msgs::ImageConstPtr &msg);
    void ImageRightCallback(const sensor_msgs::ImageConstPtr &msg);
    void LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void PSM1PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void PSM2PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg);

    cv::Mat& ImageLeft(ros::Duration timeout = ros::Duration(1));
    cv::Mat& ImageRight(ros::Duration timeout = ros::Duration(1));

    void DrawCube(cv::InputOutputArray image, const CameraDistortion &cam_intrinsics,
                  const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    // position is in task reference frame
    void DrawToolTip(cv::InputOutputArray image,
                     const CameraDistortion &cam_intrinsics,
                     const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                     KDL::Vector position,
                     const cv::Scalar color);

    void DrawACPath(cv::InputOutputArray image,
                    const CameraDistortion &cam_intrinsics,
                    const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                    const cv::Scalar color);

private:

    void GetROSParameterValues();

    void ReadCameraParameters(std::string file_path);

public:
    //GLuint texIdL;
    //GLuint texIdR;
    unsigned char* bufferL_;
    unsigned char* bufferR_;

    //in-class initialization
    ros::NodeHandle n;
    double ros_freq = 0.0;
    std::vector<cv::Point3d> ac_path;

    cv::Mat image_msg;
    CameraDistortion Camera;
    KDL::Frame pose_cam_l;
    KDL::Frame pose_cam_r;
    KDL::Frame pose_psm1;
    KDL::Frame pose_psm2;
    KDL::Frame taskspace_to_psm1_tr;
    KDL::Frame taskspace_to_psm2_tr;

//    ros::Publisher pub_board_to_cam_pose;

    cv::Vec3d cam_rvec_l, cam_tvec_l;
    cv::Mat image_left_;
    cv::Mat image_right_;

private:
    int image_width_;
    int image_height_;

    image_transport::ImageTransport *it;
    image_transport::Subscriber image_subscriber_left;
    image_transport::Subscriber image_subscriber_right;
    ros::Subscriber camera_pose_subscriber_left;
    ros::Subscriber camera_pose_subscriber_right;
    ros::Subscriber psm1_pose_sub;
    ros::Subscriber psm2_pose_sub;

    ros::Subscriber ac_path_subscriber;
    KDL::Frame camera_pose_left;
    KDL::Frame camera_pose_right;
    cv::Mat camera_matrix_l;
    cv::Mat camera_matrix_r;



};

void fromMattoImageA(
        const cv::Mat& img_in,
        sensor_msgs::Image* img_out);
#endif //TELEOP_VISION_OVERLAYGRAPHICS_H


namespace VisualUtils{

    void SwitchFullScreen(const std::string window_name);

}
