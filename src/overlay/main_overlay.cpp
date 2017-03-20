#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "Overlay.h"
#include <std_msgs/Float32.h>
//#include "Drawings.h"
//#include <iostream>
//#include <GLFW/glfw3.h>
//#include <GL/glu.h>





int main(int argc, char **argv)
{

    ros::init(argc, argv, "overlay");


    OverlayGraphics og (ros::this_node::getName(),720, 576);

    ros::Rate loop_rate(og.ros_freq);

    std::string left_window_name = "Overlay Left";
    std::string right_window_name = "Overlay Right";
    // Create the window in which to render the video feed
    cvNamedWindow(left_window_name.c_str(),CV_WINDOW_NORMAL);
    cvNamedWindow(right_window_name.c_str(),CV_WINDOW_NORMAL);
    cv::Mat left_image, right_image;

    while (ros::ok())
    {
        // --------------------------------------------------------------------------------------
        // keyboard commands


        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(left_window_name);
        else if(key == 't'){
            if (og.stereo_tr_calc_client.call(og.stereo_tr_srv)) {
                //
                std::vector<double> pose_vec_out(7, 0.0);
                conversions::PoseMsgToVector(
                        og.stereo_tr_srv.response.cam_1_to_cam_2_pose, pose_vec_out);
                og.n.setParam("left_cam_to_right_cam_transform", pose_vec_out);
                ROS_INFO_STREAM(
                        std::string("Set parameter ") << og.n.resolveName("left_cam_to_right_cam_transform")
                                         << " as\n"
                                         << og.stereo_tr_srv.response.cam_1_to_cam_2_pose);
            }
            else
                ROS_ERROR("Failed to call service stereo_tr_calc_client");
        }



        // --------------------------------------------------------------------------------------
        // Draw things LEFT
        og.ImageLeft(ros::Duration(1)).copyTo(left_image);
        og.ImageRight(ros::Duration(1)).copyTo(right_image);

        DrawingsCV::DrawCubeCV(left_image, og.cam_intrinsics[0], og.cam_rvec_l, og.cam_tvec_l,
                      cv::Point3d(0, 0, 0),
                      cv::Point3d(0.0128, 0.0128, 0.04),
                      cv::Scalar(200, 100, 10));

        DrawingsCV::DrawCubeCV(left_image, og.cam_intrinsics[0], og.cam_rvec_l, og.cam_tvec_l,
                      cv::Point3d(6 * 0.0128, 3.6 * 0.0128, 0),
                      cv::Point3d(0.0128, 0.0128, 0.04),
                      cv::Scalar(100, 100, 200));

        DrawingsCV::DrawToolTipCV(left_image, og.cam_intrinsics[0], og.cam_rvec_l, og.cam_tvec_l,
                         og.pose_tool2.p, cv::Scalar(100, 50, 200));


        DrawingsCV::DrawACPathCV(og.ImageLeft(), og.ac_path, og.cam_intrinsics[0], og.cam_rvec_l,
                                 og.cam_tvec_l, cv::Scalar(100, 50, 200));

        cv::imshow("Overlay Left", og.ImageLeft());

        cv::imshow(left_window_name, left_image);


        // Draw things RIGHT
        DrawingsCV::DrawCubeCV(right_image, og.cam_intrinsics[1], og.cam_rvec_r, og.cam_tvec_r,
                      cv::Point3d(0, 0, 0),
                      cv::Point3d(0.0128, 0.0128, 0.04),
                      cv::Scalar(200, 100, 10));

        DrawingsCV::DrawCubeCV(right_image, og.cam_intrinsics[1], og.cam_rvec_r, og.cam_tvec_r,
                      cv::Point3d(6 * 0.0128, 3.6 * 0.0128, 0),
                      cv::Point3d(0.0128, 0.0128, 0.04),
                      cv::Scalar(100, 100, 200));

        DrawingsCV::DrawToolTipCV(right_image, og.cam_intrinsics[1], og.cam_rvec_r, og.cam_tvec_r,
                         og.pose_tool2.p, cv::Scalar(100, 50, 200));

        DrawingsCV::DrawACPathCV(og.ImageLeft(), og.ac_path, og.cam_intrinsics[1], og.cam_rvec_r,
                                 og.cam_tvec_r, cv::Scalar(100, 50, 200));

        cv::imshow(right_window_name, right_image);


        ros::spinOnce();
        loop_rate.sleep();
    }

//	glfwDestroyWindow(window);
//	glfwTerminate();
    return 0;
}





