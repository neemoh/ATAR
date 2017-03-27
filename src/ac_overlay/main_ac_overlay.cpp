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



enum class Tasks {None, CricleAC, MultiplePaths};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "overlay");
    ACOverlay ao (ros::this_node::getName(),720, 576);

    // frequency of the generated images is based on the received images
    // loop_rate is the frequency of spinning and checking for new messages
    ros::Rate loop_rate(200);
    std::string window_name[2];

    window_name[0] = "Overlay Left";
    window_name[1] = "Overlay Right";
    // Create the window in which to render the video feed
    cvNamedWindow(window_name[0].c_str(),CV_WINDOW_NORMAL);
    cvNamedWindow(window_name[1].c_str(),CV_WINDOW_NORMAL);
    //cv::Mat left_image, right_image;
    cv::Mat cam_images[2];

    cv::Scalar color_ac_path_selected(200, 100, 10);
    cv::Scalar color_ac_path(100, 100, 200);
    cv::Scalar color_ac_desired_point(200, 50, 90);

// tasks
    Tasks selected_task = Tasks::None;

    std::vector<cv::Point3d> targets;
    targets.push_back(cv::Point3d(0.020,  0.05, 0.0));
    targets.push_back(cv::Point3d(0.025,  0.06, 0.01));
    targets.push_back(cv::Point3d(0.030,  0.025, 0.0));
    targets.push_back(cv::Point3d(0.035,  0.035, 0.0));
    targets.push_back(cv::Point3d(0.050,  0.06, 0.02));
    targets.push_back(cv::Point3d(0.030,  0.07, 0.02));

    MultiplePathsTask::Status task_state = MultiplePathsTask::Status::Ready;

    std::vector<cv::Point3d> ac_path_in_use;


    while (ros::ok())
    {
        // --------------------------------------------------------------------------------------
        // keyboard commands

        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(window_name[0]);
        else if(key == 't'){
            if (ao.stereo_tr_calc_client.call(ao.stereo_tr_srv)) {
                //
                std::vector<double> pose_vec_out(7, 0.0);
                conversions::PoseMsgToVector(
                        ao.stereo_tr_srv.response.cam_1_to_cam_2_pose, pose_vec_out);
                ao.n.setParam("left_cam_to_right_cam_transform", pose_vec_out);
                ROS_INFO_STREAM(
                        std::string("Set parameter ") << ao.n.resolveName("left_cam_to_right_cam_transform")
                                                      << " as\n"
                                                      << ao.stereo_tr_srv.response.cam_1_to_cam_2_pose);
            }
            else
                ROS_ERROR("Failed to call service stereo_tr_calc_client");
        }
        else if (key == '1'){
            ROS_INFO("Task 1 Selected");
            selected_task = Tasks::CricleAC;
            SimpleACs::GenerateXYCircle(KDL::Vector(0.045, 0.03, 0.0), 0.025, 200, ac_path_in_use);
        }
        else if (key == '2'){
            ROS_INFO("Task 2 Selected");
            selected_task = Tasks::MultiplePaths;
            task_state = MultiplePathsTask::Status::Ready;
        }

        if(ao.new_left_image && ao.new_right_image) {

            ao.ImageLeft(ros::Duration(1)).copyTo(cam_images[0]);
            ao.ImageRight(ros::Duration(1)).copyTo(cam_images[1]);


            // task -------------------------------

            if (selected_task == Tasks::MultiplePaths) {
                size_t selected_ac = 0;
                if (task_state == MultiplePathsTask::Status::Ready) {

                    // find the closest destination
                    selected_ac = MultiplePathsTask::FindClosestTarget(ao.pose_tool2.p, targets);

                    // draw the AC paths
                    for (int i = 0; i < 2; ++i) {
                        MultiplePathsTask::DrawAllPaths(cam_images[i], ao.cam_intrinsics[i],
                                                        ao.cam_rvec[i], ao.cam_tvec[i], ao.pose_tool2.p,
                                                        targets, selected_ac,
                                                        color_ac_path_selected, color_ac_path);
                    }

                    //if foot switch pressed select the current ac
                    if (ao.foot_switch_pressed == 1) {
                        task_state = MultiplePathsTask::Status::ACSelected;
                        // generate points of the selected ac path
                        MultiplePathsTask::GeneratePathPoints(ao.pose_tool2.p, targets[selected_ac],
                                                              ac_path_in_use);
                    }
                }

                if (task_state == MultiplePathsTask::Status::ACSelected) {

                    // draw only the selected ac path
                    // draw
                    for (int i = 0; i < 2; ++i) {
                        DrawingsCV::DrawACPath(cam_images[i], ac_path_in_use,
                                               ao.cam_intrinsics[i], ao.cam_rvec[i],
                                               ao.cam_tvec[i], color_ac_path_selected);

                        DrawingsCV::DrawCurrentToDesiredLine(cam_images[i], ao.cam_intrinsics[i],
                                                             ao.cam_rvec[i], ao.cam_tvec[i],
                                                             ao.pose_tool2.p,
                                                             ao.pose_desired[1].p,
                                                             color_ac_desired_point);
                    }

                    // criterion to call the singel tasj finished is if we are close enough to the target
                    KDL::Vector dist_target(targets[selected_ac].x - ao.pose_tool2.p[0],
                                            targets[selected_ac].y - ao.pose_tool2.p[1],
                                            targets[selected_ac].z - ao.pose_tool2.p[2]);

                    if (dist_target.Norm() < 0.01) {
                        // flag the end of the subtask
                        task_state = MultiplePathsTask::Status::Finished;
                        //remove the reached target
                        targets.erase(targets.begin() + selected_ac);
                        std::cout <<"Target Reached" <<  std::endl;

                    }
                }

                if (task_state == MultiplePathsTask::Status::Finished) {

                    // criterion to call the singel task finished is if we are close enough to the target
                    KDL::Vector dist_target(targets[0].x - ao.pose_tool2.p[0],
                                            targets[0].y - ao.pose_tool2.p[1],
                                            targets[0].z - ao.pose_tool2.p[2]);
                    std::cout <<"dist_target.Norm()" << dist_target.Norm() <<  std::endl;

                    if (dist_target.Norm() > 0.05) {
                        // flag the end of the subtask
                        task_state = MultiplePathsTask::Status::Ready;
                        //remove the reached target
                        targets.erase(targets.begin() + selected_ac);
                    }
                }
            } else if (selected_task == Tasks::CricleAC) {

                for (int i = 0; i < 2; ++i) {

                    DrawingsCV::DrawACPath(cam_images[i], ac_path_in_use,
                                           ao.cam_intrinsics[i],
                                           ao.cam_rvec[i], ao.cam_tvec[i], color_ac_path);

                    DrawingsCV::DrawCurrentToDesiredLine(cam_images[i], ao.cam_intrinsics[i],
                                                         ao.cam_rvec[i], ao.cam_tvec[i],
                                                         ao.pose_tool2.p,
                                                         ao.pose_desired[1].p,
                                                         color_ac_desired_point);
                }

            }



            // --------------------------------------------------------------------------------------
            // Draw things LEFT
//            for (int i = 0; i < 2; ++i) {
//                DrawingsCV::DrawCube(cam_images[i], ao.cam_intrinsics[i], ao.cam_rvec[i],
//                                     ao.cam_tvec[i],
//                                     cv::Point3d(0, 0, 0),
//                                     cv::Point3d(0.0128, 0.0128, 0.04),
//                                     cv::Scalar(200, 100, 10));
//            }

            for (int j = 0; j <2 ; ++j) {
                cv::imshow(window_name[j], cam_images[j]);
                ao.publisher_overlayed[j].publish(
                        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_images[j]).toImageMsg());

            }

            if (selected_task == Tasks::CricleAC || task_state == MultiplePathsTask::Status::ACSelected) {
                geometry_msgs::PoseArray out;
                VecPoint3dToPoseArray(ac_path_in_use, out);
                out.header.stamp = ros::Time::now();
                out.header.frame_id = "/task_space";
                ao.publisher_ac_path.publish(out);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

//	glfwDestroyWindow(window);
//	glfwTerminate();
    return 0;
}



