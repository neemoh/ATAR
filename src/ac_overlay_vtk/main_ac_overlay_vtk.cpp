//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "OverlayROSConfig.h"
#include <std_msgs/Float32.h>
#include "Rendering.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ac_overlay");
    OverlayROSConfig rc (ros::this_node::getName());

    // Create the window for the video feed
    std::string cv_window_name = "Augmented Stereo images";
    cvNamedWindow(cv_window_name.c_str() ,CV_WINDOW_NORMAL);

    std::stringstream ui_instructions;
    ui_instructions <<"Test. " ;

    cv::Mat augmented_stereo_image;
    cv::Mat cam_images[2];
    rc.LockAndGetImages(ros::Duration(1), cam_images);

    Rendering graphics;

    // in case camera poses are set as parameters
    cv::Vec3d cam_rvec[2], cam_tvec[2];
    rc.GetCameraPoses(cam_rvec, cam_tvec);
    graphics.SetWorldToCameraTransform(cam_rvec, cam_tvec);

    // set the intrinsics and configure the background image
    graphics.SetCameraIntrinsics(rc.camera_matrix);
    graphics.ConfigureBackgroundImage(cam_images);
    graphics.SetEnableBackgroundImage(true);
    graphics.Render();

    bool task_is_running = 0;

    while (ros::ok())
    {
        // --------------------------------------------------------------------------------------
        // Update cam poses if needed
        if(rc.GetNewCameraPoses(cam_rvec, cam_tvec))
            graphics.SetWorldToCameraTransform(cam_rvec, cam_tvec);

        // --------------------------------------------------------------------------------------
        // keyboard commands

        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(cv_window_name);

        else if (key == '1' || key == '2'){
            int task_id = key - '0';
            ROS_INFO("Task %d Selected", task_id);
            if(task_is_running) {
                graphics.RemoveAllActorsFromScene();
                rc.StopTask();
            }

            rc.StartTask((uint)task_id);
            // Add the task actors to the graphics
            graphics.AddActorsToScene(rc.task_ptr->GetActors());
            task_is_running = true;
        }


        if(rc.GetNewImages(cam_images)) {

            // Time performance debug
            //ros::Time start =ros::Time::now();

            //            // print instructions
            //            for (int i = 0; i < 2; ++i)
            //                cv::putText(cam_images[i], ui_instructions.str(),
            //                            cv::Point(50, 50), 0, 0.8, cv::Scalar(20, 150, 20), 2);

            // update the moving actors
            if(task_is_running)
                rc.task_ptr->UpdateActors();

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
            if(task_is_running) {
                if (rc.task_ptr->IsACParamChanged()) {
                    rc.PublishACtiveConstraintParameters(
                            rc.task_ptr->GetACParameters());
                }
                // publish the task state
                rc.PublishTaskState(rc.task_ptr->GetTaskStateMsg());
            }
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



