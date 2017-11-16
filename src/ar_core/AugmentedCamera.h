//
// Created by nima on 02/11/17.
//

#ifndef ATAR_ARCAMERA_H
#define ATAR_ARCAMERA_H

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/aruco/charuco.hpp>

class AugmentedCamera {
public:

    AugmentedCamera(image_transport::ImageTransport *it=NULL,
             const std::string cam_name="", const std::string ns="");

    // callbacks
    void ImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void GetIntrinsicParams(double& fx, double &fy, double &cx, double &cy);

    // returns the camera intrinsic matrices
    void GetIntrinsicMatrices(cv::Mat &cam_mat, cv::Mat &dist_mat);

    // returns true if there is a new pose available and will write it on &pose
    bool GetNewWorldToCamTr(KDL::Frame &pose);

    // this returns the pose of the cam regardless of it being new or not
    KDL::Frame GetWorldToCamTr(){return world_to_cam_tr;};

    bool IsImageNew();

    // Tt is important to copy the image to prevent seg fault due to
    // subscriber thread and vtk rendering thread accessing the image at the
    // same time

    cv::Mat LockAndGetImage();

    cv::Mat GetImage(){return image;};

private:

    void ReadCameraParameters(std::string file_path);

    bool DetectCharucoBoardPose(KDL::Frame &pose, cv::Mat image);

private:
    std::string                 img_topic;
    cv::Mat                     image;
    bool                        new_image = false;
    bool                        new_pose_from_sub = false;
    KDL::Frame                  world_to_cam_tr;
    bool                        is_pose_from_subscriber =true;

    cv::Mat                     camera_matrix;
    cv::Mat                     camera_distortion;

    // pose estimation
    cv::Ptr<cv::aruco::CharucoBoard>   charuco_board;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    image_transport::Subscriber sub_image;
    ros::Subscriber             sub_pose;
};


#endif //ATAR_ARCAMERA_H
