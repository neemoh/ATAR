//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "OverlayROSConfig.h"
#include <std_msgs/Float32.h>
#include "CalibratedCamera.h"
#include "BuzzWireTask.h"



enum class Tasks {None, CricleAC, MultiplePaths};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ac_overlay");
    OverlayROSConfig rc (ros::this_node::getName(),720, 576);

    // frequency of the generated images is based on the received images
    // loop_rate is the frequency of spinning and checking for new messages
    // and evaluating and publishing the desired pose according to the active
    // constraint
    ros::Rate loop_rate(rc.desired_pose_update_freq);

    // Create the window for the video feed
    std::string cv_window_name = "Augmented Stereo images";
    cvNamedWindow(cv_window_name.c_str() ,CV_WINDOW_NORMAL);

    cv::Mat cam_images[2];
    cv::Mat augmented_stereo_image;

    std::vector<cv::Point3d> ac_path_in_use;

    std::stringstream ui_instructions;
    ui_instructions <<"Test. " ;


    rc.ImageLeft(ros::Duration(1)).copyTo(cam_images[0]);
    rc.ImageRight(ros::Duration(1)).copyTo(cam_images[1]);

    Rendering graphics;
    graphics.SetWorldToCameraTransform(rc.cam_rvec, rc.cam_tvec);
    cv::Matx33d cam_matrices[2];
    cam_matrices[0]= rc.cam_intrinsics[0].camMatrix;
    cam_matrices[1]= rc.cam_intrinsics[1].camMatrix;
    graphics.SetCameraIntrinsics(cam_matrices);
    graphics.ConfigureBackgroundImage(cam_images);
    graphics.SetEnableBackgroundImage(true);

    graphics.Render();

    BuzzWireTask btask(0.004, 0.0005, rc.show_reference_frames); // ring_radius and wire_radius, show frames


    graphics.AddActorsToScene(btask.GetActors());

    while (ros::ok())
    {

        // --------------------------------------------------------------------------------------
        // keyboard commands

        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(cv_window_name);
        else if (key == '1'){
            ROS_INFO("Task 1 Selected");
                 }

        btask.SetCurrentToolPose(rc.pose_current_tool[0]);

        if(rc.new_left_image && rc.new_right_image) {

            rc.ImageLeft(ros::Duration(1)).copyTo(cam_images[0]);
            rc.ImageRight(ros::Duration(1)).copyTo(cam_images[1]);


            // print instructions
            for (int i = 0; i < 2; ++i)
                cv::putText(cam_images[i], ui_instructions.str(),
                            cv::Point(50, 50), 0, 0.8, cv::Scalar(20, 150, 20), 2);

            // update the moving actors
            btask.UpdateActors();

            // update the camera images and view angle (in case window changes size)
            graphics.UpdateBackgroundImage(cam_images);
            graphics.UpdateViewAngleForActualWindowSize();

            // Render!
            graphics.Render();

            // Copy the rendered image to memory, show it and publish it.
            graphics.GetRenderedImage(augmented_stereo_image);
            cv::imshow(cv_window_name, augmented_stereo_image);
            rc.publisher_stereo_overlayed.publish(cv_bridge::CvImage(std_msgs::Header(),
                                                                     "bgr8", augmented_stereo_image).toImageMsg());

        } // if new image

        // updating the desired pose happens at the higher frequency
        KDL::Frame pose_desired_tool[2];
        pose_desired_tool[0] = btask.GetDesiredToolPose();
        rc.PublishDesiredPose(pose_desired_tool);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



