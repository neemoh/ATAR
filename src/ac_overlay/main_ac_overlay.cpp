#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "ACOverlay.h"
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

    cv::Scalar color_ac_path_selected(200, 100, 10);
    cv::Scalar color_ac_path(100, 100, 200);


    std::vector<cv::Point3d> targets;
    targets.push_back(cv::Point3d(0.01,  0.01, 0.0));
    targets.push_back(cv::Point3d(0.01,  0.02, 0.01));
    targets.push_back(cv::Point3d(0.015, 0.025, 0.0));
    targets.push_back(cv::Point3d(0.035, 0.03, 0.0));
    targets.push_back(cv::Point3d(0.04,  0.02, 0.02));

    MultiplePathsTask::Status task_state = MultiplePathsTask::Status::Ready;

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


        og.ImageLeft(ros::Duration(1)).copyTo(left_image);
        og.ImageRight(ros::Duration(1)).copyTo(right_image);



        // task -------------------------------

        size_t selected_ac = 0;
        if(task_state == MultiplePathsTask::Status::Ready){

            // find the closest destination
            selected_ac = MultiplePathsTask::FindClosestTarget(og.pose_tool2.p, targets);

            // draw the AC paths left
            MultiplePathsTask::DrawAllPaths(left_image, og.cam_intrinsics[0],
                                            og.cam_rvec_l, og.cam_tvec_l, og.pose_tool2.p,
                                            targets, selected_ac,
                                            color_ac_path_selected, color_ac_path);
            // draw the AC paths right
            MultiplePathsTask::DrawAllPaths(left_image, og.cam_intrinsics[1],
                                            og.cam_rvec_r, og.cam_tvec_r, og.pose_tool2.p,
                                            targets, selected_ac,
                                            color_ac_path_selected, color_ac_path);

            //if foot switch pressed select the current ac
            if(og.foot_switch_pressed == 1)
                task_state = MultiplePathsTask::Status::ACSelected;
        }

        if(task_state == MultiplePathsTask::Status::ACSelected){

            // draw only the selected ac path
            // make a one element vector so that we can use the same DrawAllPaths function
            std::vector<cv::Point3d> target_selected;
            target_selected.push_back(targets[selected_ac]);

            // draw left
            MultiplePathsTask::DrawAllPaths(left_image, og.cam_intrinsics[0],
                                            og.cam_rvec_l, og.cam_tvec_l,
                                            og.pose_tool2.p, target_selected, 0,
                                            color_ac_path_selected, color_ac_path);
            // draw right
            MultiplePathsTask::DrawAllPaths(left_image, og.cam_intrinsics[1],
                                            og.cam_rvec_r, og.cam_tvec_r,
                                            og.pose_tool2.p, target_selected, 0,
                                            color_ac_path_selected, color_ac_path);

            // criterion to call the singel tasj finished is if we are close enough to the target
            KDL::Vector dist_target(target_selected[0].x - og.pose_tool2.p[0],
                                    target_selected[0].y - og.pose_tool2.p[1],
                                    target_selected[0].z - og.pose_tool2.p[2]);
            if(dist_target.Norm() < 0.003){
                // flag the end of the subtask
                task_state = MultiplePathsTask::Status::Finished;
                //remove the reached target
                targets.erase(targets.begin()+selected_ac);
            }
        }

        if (task_state == MultiplePathsTask::Status::Finished){

            // criterion to call the singel tasj finished is if we are close enough to the target
            KDL::Vector dist_target(targets[0].x - og.pose_tool2.p[0],
                                    targets[0].y - og.pose_tool2.p[1],
                                    targets[0].z - og.pose_tool2.p[2]);
            if(dist_target.Norm() > 0.05){
                // flag the end of the subtask
                task_state = MultiplePathsTask::Status::Finished;
                //remove the reached target
                targets.erase(targets.begin()+selected_ac);
            }
        }







            // --------------------------------------------------------------------------------------
        // Draw things LEFT


        DrawingsCV::DrawCubeCV(left_image, og.cam_intrinsics[0], og.cam_rvec_l, og.cam_tvec_l,
                      cv::Point3d(0, 0, 0),
                      cv::Point3d(0.0128, 0.0128, 0.04),
                      cv::Scalar(200, 100, 10));



        // Draw things RIGHT
        DrawingsCV::DrawCubeCV(right_image, og.cam_intrinsics[1], og.cam_rvec_r, og.cam_tvec_r,
                      cv::Point3d(0, 0, 0),
                      cv::Point3d(0.0128, 0.0128, 0.04),
                      cv::Scalar(200, 100, 10));



        cv::imshow(left_window_name, left_image);
        cv::imshow(right_window_name, right_image);


        ros::spinOnce();
        loop_rate.sleep();
    }

//	glfwDestroyWindow(window);
//	glfwTerminate();
    return 0;
}





