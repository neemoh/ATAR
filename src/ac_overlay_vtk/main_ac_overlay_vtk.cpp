//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "OverlayROSConfig.h"
#include <std_msgs/Float32.h>

#include <boost/thread/thread.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ac_overlay");
    OverlayROSConfig rc (ros::this_node::getName(),720, 576);

    //    ros::Rate loop_rate(rc.desired_pose_update_freq);

    // Create the window for the video feed
    std::string cv_window_name = "Augmented Stereo images";
    cvNamedWindow(cv_window_name.c_str() ,CV_WINDOW_NORMAL);

    std::stringstream ui_instructions;
    ui_instructions <<"Test. " ;

    cv::Mat augmented_stereo_image;
    cv::Mat cam_images[2];
    rc.LockAndGetImages(ros::Duration(1), cam_images);

    Rendering graphics;

    graphics.SetWorldToCameraTransform(rc.cam_rvec, rc.cam_tvec);
    graphics.SetCameraIntrinsics(rc.camera_matrix);
    graphics.ConfigureBackgroundImage(cam_images);
    graphics.SetEnableBackgroundImage(true);
    graphics.Render();

    // Add the task actors to the graphics
    graphics.AddActorsToScene(rc.buzz_task->GetActors());

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

        if(rc.GetNewImages(cam_images)) {

            // Time performance debug
            //ros::Time start =ros::Time::now();

            //            // print instructions
            //            for (int i = 0; i < 2; ++i)
            //                cv::putText(cam_images[i], ui_instructions.str(),
            //                            cv::Point(50, 50), 0, 0.8, cv::Scalar(20, 150, 20), 2);

            // update the moving actors
            rc.buzz_task->UpdateActors();

            // update the camera images and view angle (in case window changes size)
            graphics.UpdateBackgroundImage(cam_images);
            graphics.UpdateViewAngleForActualWindowSize();

            // Render!
            graphics.Render();

            // Copy the rendered image to memory, show it and/or publish it.
            graphics.GetRenderedImage(augmented_stereo_image);
            cv::imshow(cv_window_name, augmented_stereo_image);
            rc.publisher_stereo_overlayed.publish(
                    cv_bridge::CvImage(std_msgs::Header(),
                                       "bgr8", augmented_stereo_image).toImageMsg());

            // publish the active constraint parameters if needed
            if(rc.buzz_task->IsACParamChanged()) {
                rc.PublishACtiveConstraintParameters(
                        rc.buzz_task->GetACParameters());
            }
            // publish the task state
            rc.PublishTaskState(rc.buzz_task->GetTaskStateMsg());

            // check time performance
            // std::cout <<  "it took: " << (ros::Time::now() - start).toNSec() /1000000 << std::endl;

        } // if new image

        // copying the render buffer back from gpu takes a very long time,
        // creating a bottle neck that reduced the update rate to 25 Hz
        // already. So no sleep is needed foe now.
        //        loop_rate.sleep();
    }

    return 0;
}



