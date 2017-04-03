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

    // frequency of the generated images is based on the received images
    // loop_rate is the frequency of spinning and checking for new messages
    ros::Rate loop_rate(200);

    std::string left_window_name = "Overlay Left";
    std::string right_window_name = "Overlay Right";
    // Create the window in which to render the video feed
    cvNamedWindow(left_window_name.c_str(),CV_WINDOW_NORMAL);
    cvNamedWindow(right_window_name.c_str(),CV_WINDOW_NORMAL);
    cv::Mat left_image, right_image;

    cv::Scalar color_desired_point(200, 100, 10);
    cv::Scalar color_ac_path(100, 100, 200);

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

        if(og.new_left_image && og.new_right_image) {

            og.ImageLeft(ros::Duration(1)).copyTo(left_image);
            og.ImageRight(ros::Duration(1)).copyTo(right_image);

            // --------------------------------------------------------------------------------------
            // Draw things LEFT

            DrawingsCV::DrawCube(left_image, og.cam_intrinsics[0],
                                 og.cam_rvec_l, og.cam_tvec_l,
                                 cv::Point3d(0, 4 * 0.0128, 0),
                                 cv::Point3d(0.0128, 0.0128, 0.04),
                                 cv::Scalar(200, 100, 10));

            DrawingsCV::DrawCube(left_image, og.cam_intrinsics[0],
                                 og.cam_rvec_l, og.cam_tvec_l,
                                 cv::Point3d(6 * 0.0128, 4 * 0.0128, 0),
                                 cv::Point3d(0.0128, 0.0128, 0.04),
                                 cv::Scalar(10, 100, 200));

            DrawingsCV::DrawPoint(left_image, og.cam_intrinsics[0],
                                  og.cam_rvec_l, og.cam_tvec_l,
                                  og.pose_tool2.p,
                                  cv::Scalar(100, 50, 200));


            DrawingsCV::DrawACPath(left_image, og.ac_path,
                                   og.cam_intrinsics[0], og.cam_rvec_l,
                                   og.cam_tvec_l, color_ac_path);

            DrawingsCV::DrawCurrentToDesiredLine(left_image, og.cam_intrinsics[0],
                                                 og.cam_rvec_l, og.cam_tvec_l,
                                                 og.pose_tool2.p,
                                                 og.pose_desired_r.p,
                                                 color_desired_point);
            // draw desired point right tool
            DrawingsCV::DrawPoint(left_image, og.cam_intrinsics[0],
                                  og.cam_rvec_l, og.cam_tvec_l,
                                  og.pose_desired_r.p,
                                  color_desired_point);

            cv::imshow("Overlay Left", og.ImageLeft());

            cv::imshow(left_window_name, left_image);

            // --------------------------------------------------------------------------------------
            //  Draw things RIGHT
            DrawingsCV::DrawCube(right_image, og.cam_intrinsics[1],
                                 og.cam_rvec_r, og.cam_tvec_r,
                                 cv::Point3d(0, 4 * 0.0128, 0),
                                 cv::Point3d(0.0128, 0.0128, 0.04),
                                 cv::Scalar(200, 100, 10));

            DrawingsCV::DrawCube(right_image, og.cam_intrinsics[1],
                                 og.cam_rvec_r, og.cam_tvec_r,
                                 cv::Point3d(6 * 0.0128, 4 * 0.0128, 0),
                                 cv::Point3d(0.0128, 0.0128, 0.04),
                                 cv::Scalar(10, 100, 200));

            DrawingsCV::DrawPoint(right_image, og.cam_intrinsics[1],
                                  og.cam_rvec_r, og.cam_tvec_r,
                                  og.pose_tool2.p,
                                  cv::Scalar(10, 50, 100));


            DrawingsCV::DrawCurrentToDesiredLine(right_image, og.cam_intrinsics[1],
                                                 og.cam_rvec_r, og.cam_tvec_r,
                                                 og.pose_tool2.p,
                                                 og.pose_desired_r.p,
                                                 color_desired_point);

            DrawingsCV::DrawACPath(right_image, og.ac_path,
                                     og.cam_intrinsics[1], og.cam_rvec_r,
                                     og.cam_tvec_r, color_ac_path);

            // draw desired point right tool
            DrawingsCV::DrawPoint(right_image, og.cam_intrinsics[1],
                                  og.cam_rvec_r, og.cam_tvec_r,
                                  og.pose_desired_r.p,
                                  color_desired_point);

            // Publish the overlays to ROS
            if (og.IsROSOverlayEnabled()) {
                og.PublishOverlayLeft(left_image);
                og.PublishOverlayRight(right_image);
            }

            cv::imshow(right_window_name, right_image);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

//	glfwDestroyWindow(window);
//	glfwTerminate();
    return 0;
}





