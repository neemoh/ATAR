//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "custom_conversions/Conversions.h"
#include "OverlayROSConfig.h"
#include <std_msgs/Float32.h>
#include <ode/ode.h>
#include "Rendering.h"

dWorldID World;

dJointGroupID contactgroup;

int main(int argc, char **argv)
{
    World = dWorldCreate();
    contactgroup = dJointGroupCreate(0);

    ros::init(argc, argv, "ac_overlay");
    OverlayROSConfig rc (ros::this_node::getName());
    uint num_windows = 1;

    // Create the window for the video feed
    std::string cv_window_names[2] = {"Augmented Left", "Augmented Right"};

    if(num_windows==1)
        cvNamedWindow(cv_window_names[0].c_str() ,CV_WINDOW_NORMAL);
    else if(num_windows==2){
        cvNamedWindow(cv_window_names[0].c_str() ,CV_WINDOW_NORMAL);
        cvNamedWindow(cv_window_names[1].c_str() ,CV_WINDOW_NORMAL);
    }

    std::stringstream ui_instructions;
    ui_instructions <<"Test. " ;

    cv::Mat augmented_images[2];
    cv::Mat cam_images[2];
    rc.LockAndGetImages(ros::Duration(1), cam_images);

    Rendering graphics(num_windows);

    // in case camera poses are set as parameters
    cv::Vec3d cam_rvec[2], cam_tvec[2];
    rc.GetCameraPoses(cam_rvec, cam_tvec);
    graphics.SetWorldToCameraTransform(cam_rvec, cam_tvec);

    // set the intrinsics and configure the background image
    graphics.SetCameraIntrinsics(rc.camera_matrix);
    graphics.ConfigureBackgroundImage(cam_images);
    graphics.SetEnableBackgroundImage(true);
    graphics.Render();

    // task_id 0 means no running task
    uint task_id = 0;
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // --------------------------------------------------------------------------------------
        // Update cam poses if needed
        if(rc.GetNewCameraPoses(cam_rvec, cam_tvec))
            graphics.SetWorldToCameraTransform(cam_rvec, cam_tvec);

        // --------------------------------------------------------------------------------------
        // keyboard commands

        char key = (char)cv::waitKey(1);
        if (key == 27) {// Esc
            graphics.RemoveAllActorsFromScene();
            rc.Cleanup();
            break;

        }
        else if (key == 'a'){
            if(task_id>0) {
                // if a task is running first stop it
                graphics.RemoveAllActorsFromScene();
                rc.DeleteTask();
                // then do the calibration
                rc.DoArmToWorldFrameCalibration(0);
                // run the task again
                rc.StartTask((uint)task_id);
                graphics.AddActorsToScene(rc.task_ptr->GetActors());
            }
            else // if no task is running just do the calibration
                rc.DoArmToWorldFrameCalibration(0);
        }
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(cv_window_names[0]);


        else if (key == '1' || key == '2'|| key == '3' || key == '4'){
            ROS_INFO("Task %d Selected", task_id);
            if(task_id) {
                graphics.RemoveAllActorsFromScene();
                rc.DeleteTask();
            }
            task_id = uint(key - '0');

            rc.StartTask((uint)task_id);
            // Add the task actors to the graphics
            graphics.AddActorsToScene(rc.task_ptr->GetActors());

        }


        if(rc.GetNewImages(cam_images)) {

            // Time performance debug
            //ros::Time start =ros::Time::now();

            // update the moving actors
            if(task_id)
                rc.task_ptr->UpdateActors();

            // update the camera images and view angle (in case window changes size)
            graphics.UpdateBackgroundImage(cam_images);
            graphics.UpdateCameraViewForActualWindowSize();

            // Render!
            graphics.Render();

            // Copy the rendered image to memory, show it and/or publish it.
            if(rc.publish_overlayed_images) {
                graphics.GetRenderedImage(augmented_images);
                if(num_windows ==1){
                    cv::imshow(cv_window_names[0], augmented_images[0]);
                    rc.publisher_stereo_overlayed.publish(
                            cv_bridge::CvImage(std_msgs::Header(),
                                               "bgr8",
                                               augmented_images[0])
                                    .toImageMsg());

                }
                else if(num_windows ==2){
                    for (int i = 0; i < 2; ++i) {
                        cv::imshow(cv_window_names[i], augmented_images[i]);
                        rc.publisher_overlayed[i].publish(
                                cv_bridge::CvImage(std_msgs::Header(),
                                                   "bgr8",
                                                   augmented_images[i])
                                        .toImageMsg());
                    }
                }

            }
            // publish the active constraint parameters if needed
            if(task_id) {
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
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}



