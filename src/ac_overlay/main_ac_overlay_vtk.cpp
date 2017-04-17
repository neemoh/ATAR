//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "ACOverlay.h"
#include <std_msgs/Float32.h>
#include "CalibratedCamera.h"



#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkImageActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkSTLReader.h>
#include <vtkProperty.h>
#include <vtkParametricTorus.h>
#include <vtkParametricFunctionSource.h>
#include "Rendering.h"



enum class Tasks {None, CricleAC, MultiplePaths};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ac_overlay");
    ACOverlay ao (ros::this_node::getName(),720, 576);

    // frequency of the generated images is based on the received images
    // loop_rate is the frequency of spinning and checking for new messages
    // and evaluating and publishing the desired pose according to the active
    // constraint
    ros::Rate loop_rate(ao.desired_pose_update_freq);

    // Create the window for the video feed
    std::string cv_window_name = "Augmented Stereo images";
    cvNamedWindow(cv_window_name.c_str() ,CV_WINDOW_NORMAL);

    cv::Mat cam_images[2];
    cv::Mat augmented_stereo_image;

    cv::Scalar color_ac_path_selected(200, 100, 10);
    cv::Scalar color_ac_path(100, 100, 200);
    cv::Scalar color_ac_desired_point(200, 50, 90);

    // task2 targets
    Tasks selected_task = Tasks::None;

    std::vector<cv::Point3d> targets_original;
    double h = 0.0265;
    double w = 0.025;
    targets_original.push_back(cv::Point3d(0.013      ,  0.028, 0.026));
    targets_original.push_back(cv::Point3d(0.013 + 1*h,  0.028, 0.026));
    targets_original.push_back(cv::Point3d(0.013 + 2*h,  0.028, 0.026));
    targets_original.push_back(cv::Point3d(0.013      ,  0.028 + 1*w, 0.026));
    targets_original.push_back(cv::Point3d(0.013+ 1*h ,  0.028 + 1*w, 0.026));
    targets_original.push_back(cv::Point3d(0.011+ 2*h ,  0.028 + 1*w, 0.026));

    std::vector<cv::Point3d> targets = targets_original;
    MultiplePathsTask::Status task_state = MultiplePathsTask::Status::Ready;
    std::vector<cv::Point3d> ac_path_in_use;
    size_t selected_ac = 0;

    std::stringstream ui_instructions;
    ui_instructions <<"Press 1 for task1 and 2 for task2. " ;


    // ----------------------------------------------------------------------------------------------
    //                              VTK test
    // ----------------------------------------------------------------------------------------------

    ao.ImageLeft(ros::Duration(1)).copyTo(cam_images[0]);
    ao.ImageRight(ros::Duration(1)).copyTo(cam_images[1]);

    Rendering graphics;


    graphics.SetWorldToCameraTransform(ao.cam_rvec, ao.cam_tvec);
    cv::Matx33d cam_matrices[2];
    cam_matrices[0]= ao.cam_intrinsics[0].camMatrix;
    cam_matrices[1]= ao.cam_intrinsics[1].camMatrix;
    graphics.SetCameraIntrinsics(cam_matrices);
    graphics.ConfigureBackgroundImage(cam_images);
    graphics.SetEnableBackgroundImage(true);

    graphics.Render();


    // Create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource=
            vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(0.003);
    sphereSource->SetThetaResolution(20);
    sphereSource->SetPhiResolution(20);
    // Create a mesh_mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    int counter = 0;
    sphereActor->SetPosition(0.012, 0.00, 0.0);
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(0.8, 0.2, 0.4);

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(0.0, 0, 0.0);
    transform->RotateX(0);
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes = vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkAxesActor> tool_frame_axes = vtkSmartPointer<vtkAxesActor>::New();


    vtkSmartPointer<vtkParametricTorus> parametricObject = vtkSmartPointer<vtkParametricTorus>::New();
    parametricObject->SetCrossSectionRadius(0.2);
    double rad =     parametricObject->GetCrossSectionRadius();
    std::cout << "rad " << rad << std::endl;

    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource =
            vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> ring_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_mapper->SetInputConnection(parametricFunctionSource->GetOutputPort());
    // Create an actor for the contours
    vtkSmartPointer<vtkActor> ring_actor =
            vtkSmartPointer<vtkActor>::New();
    ring_actor->SetMapper(ring_mapper);
    ring_actor->SetScale(0.005);
    ring_actor->GetProperty()->SetColor(0.2, 0.4, 0.4);

    // The task_coordinate_axes are positioned with a user transform
    task_coordinate_axes->SetUserTransform(transform);
    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    tool_frame_axes->SetUserTransform(transform);
    tool_frame_axes->SetXAxisLabelText("");
    tool_frame_axes->SetYAxisLabelText("");
    tool_frame_axes->SetZAxisLabelText("");
    tool_frame_axes->SetTotalLength(0.007, 0.007, 0.007);
    tool_frame_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    std::string inputFilename = "/home/charm/Desktop/cads/task1_first_mq.stl";

    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(inputFilename.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mesh_mapper->SetInputConnection(reader->GetOutputPort());
    std::cout << "reader: " << mesh_mapper->GetLength() << std::endl;

    vtkSmartPointer<vtkActor> mesh_actor = vtkSmartPointer<vtkActor>::New();
    mesh_actor->SetMapper(mesh_mapper);
    mesh_actor->SetPosition(0.02, 0.03, 0.0);
    mesh_actor->SetScale(0.0005);
    mesh_actor->SetOrientation(90, 0.0 , 90.0);
    mesh_actor->GetProperty()->SetColor(0.7, 0.5, 0.2);
    mesh_actor->GetProperty()->SetSpecular(0.8);

    graphics.AddActorToScene(task_coordinate_axes);
    graphics.AddActorToScene(tool_frame_axes);
    graphics.AddActorToScene(sphereActor);
    graphics.AddActorToScene(mesh_actor);
    graphics.AddActorToScene(ring_actor);


    while (ros::ok())
    {

        // --------------------------------------------------------------------------------------
        // keyboard commands

        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(cv_window_name);
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
            ac_path_in_use.clear();
            CricleACTask::GenerateXYCircle(KDL::Vector(0.045, 0.04, 0.04), 0.025, 200, ac_path_in_use);
        }
        else if (key == '2'){
            ROS_INFO("Task 2 Selected");
            selected_task = Tasks::MultiplePaths;
            task_state = MultiplePathsTask::Status::Ready;
        }
        else if (key == '0'){
            ROS_INFO("No task Selected");
            selected_task = Tasks::None;
        }


        if(ao.new_left_image && ao.new_right_image) {

            ao.ImageLeft(ros::Duration(1)).copyTo(cam_images[0]);
            ao.ImageRight(ros::Duration(1)).copyTo(cam_images[1]);

            // task -------------------------------

            if (selected_task == Tasks::MultiplePaths) {
                if (task_state == MultiplePathsTask::Status::Ready) {
                    //change the instructions
                    ui_instructions.str(std::string());
                    ui_instructions <<"Press the scene_camera pedal to select the closest target (blue). "
                                    << targets.size()<<" to go.";
                    // find the closest destination
                    selected_ac = MultiplePathsTask::FindClosestTarget(ao.pose_current_tool[0].p, targets);

                    // draw the AC paths
                    for (int i = 0; i < 2; ++i) {
                        MultiplePathsTask::DrawAllPaths(cam_images[i], ao.cam_intrinsics[i],
                                                        ao.cam_rvec[i], ao.cam_tvec[i], ao.pose_current_tool[0].p,
                                                        targets, selected_ac,
                                                        color_ac_path_selected, color_ac_path);
                    }

                    //if foot switch pressed select the current ac
                    if (ao.foot_switch_pressed == 1) {
                        // generate points of the selected ac path
                        MultiplePathsTask::GeneratePathPoints(ao.pose_current_tool[0].p, targets[selected_ac],
                                                              ac_path_in_use);
                        std::cout <<"Selected target "<< selected_ac << std::endl;
                        task_state = MultiplePathsTask::Status::ACSelected;
                        //change the instructions
                        ui_instructions.str(std::string());
                        ui_instructions <<"Active constraint activated. Approach the target. ";
                    }
                }

                if (task_state == MultiplePathsTask::Status::ACSelected) {

                    // draw only the selected ac path
                    // draw
                    for (int i = 0; i < 2; ++i) {
                        DrawingsCV::DrawPoint3dVector(cam_images[i],
                                                      ac_path_in_use,
                                                      ao.cam_intrinsics[i],
                                                      ao.cam_rvec[i],
                                                      ao.cam_tvec[i],
                                                      color_ac_path_selected);
                        // draw target point
                        DrawingsCV::DrawPoint(cam_images[i],
                                              ao.cam_intrinsics[i], ao.cam_rvec[i],
                                              ao.cam_tvec[i],
                                              KDL::Vector(targets[selected_ac].x,
                                                          targets[selected_ac].y,
                                                          targets[selected_ac].z),
                                              color_ac_path_selected);

                        DrawingsCV::DrawLineFrom2KDLPoints(cam_images[i],
                                                           ao.cam_intrinsics[i],
                                                           ao.cam_rvec[i],
                                                           ao.cam_tvec[i],
                                                           ao.pose_current_tool[0].p,
                                                           ao.pose_desired_tool[0].p,
                                                           color_ac_desired_point);
                    }

                    // criterion to call the single task finished is if we are close enough to the target
                    KDL::Vector dist_target(targets[selected_ac].x - ao.pose_current_tool[0].p[0],
                                            targets[selected_ac].y - ao.pose_current_tool[0].p[1],
                                            targets[selected_ac].z - ao.pose_current_tool[0].p[2]);

                    if (dist_target.Norm() < 0.003) {
                        // flag the end of the subtask
                        task_state = MultiplePathsTask::Status::SubTaskFinished;
                        //remove the reached target
                        targets.erase(targets.begin() + selected_ac);
                        std::cout <<"Target "<<selected_ac <<" Reached. "
                                  << targets.size()<<" more to go." << std::endl;
                        //change the instructions
                        ui_instructions.str(std::string());
                        ui_instructions <<"Target reached. Elevate the tool (away from the board). ";
                    }
                }

                if (task_state == MultiplePathsTask::Status::SubTaskFinished) {


                    // when task finishes the user should go back to a homeing distance, just to get away from teh board.
                    if (std::fabs(targets[0].z - ao.pose_current_tool[0].p[2]) > 0.035) {
                        // flag the end of the subtask
                        task_state = MultiplePathsTask::Status::Ready;
                        //change the instructions
                        ui_instructions.str(std::string());
                        ui_instructions <<"Press the scene_camera pedal to select the closest target (blue line). "
                                        << targets.size()<<" targets to go.";
                    }
                    if(targets.size()==0){
                        ui_instructions.str(std::string());
                        ui_instructions <<"Press 1 for task1 and 2 for task2. ";
                        // refill the targets
                        targets = targets_original;
                    }

                }

            } else if (selected_task == Tasks::CricleAC) {

                for (int i = 0; i < 2; ++i) {

                    // Draw the ac path points
                    DrawingsCV::DrawPoint3dVector(cam_images[i], ac_path_in_use,
                                                  ao.cam_intrinsics[i],
                                                  ao.cam_rvec[i],
                                                  ao.cam_tvec[i], color_ac_path);
                    // Draw a line to the closest point
                    DrawingsCV::DrawLineFrom2KDLPoints(cam_images[i],
                                                       ao.cam_intrinsics[i],
                                                       ao.cam_rvec[i],
                                                       ao.cam_tvec[i],
                                                       ao.pose_current_tool[0].p,
                                                       ao.pose_desired_tool[0].p,
                                                       color_ac_desired_point);
                }
                // change instructions
                ui_instructions.str(std::string());
                ui_instructions <<"A simple circular active constraints.";
            }


            for (int j = 0; j <2 ; ++j) {
//                // draw the coordinate frame of the board
//                DrawingsCV::DrawCoordinateFrameInTaskSpace(cam_images[j], ao.cam_intrinsics[j],
//                                                           KDL::Frame(),
//                                                           ao.cam_rvec[j], ao.cam_tvec[j], 0.01);
//                // draw the end-effector ref frame
//                DrawingsCV::DrawCoordinateFrameInTaskSpace(cam_images[j], ao.cam_intrinsics[j],
//                                                           ao.pose_current_tool[0],
//                                                           ao.cam_rvec[j], ao.cam_tvec[j], 0.01);
//                //draw the desired pose frame
                DrawingsCV::DrawCoordinateFrameInTaskSpace(cam_images[j], ao.cam_intrinsics[j],
                                                           ao.pose_desired_tool[0],
                                                           ao.cam_rvec[j], ao.cam_tvec[j], 0.01);;
//                //draw the  tangent
//                DrawingsCV::DrawLineFrom2KDLPoints(cam_images[j],
//                                                   ao.cam_intrinsics[j],
//                                                   ao.cam_rvec[j],
//                                                   ao.cam_tvec[j],
//                                                   ao.pose_desired_tool[0].p,
//                                                   ao.pose_desired_tool[0].p + 0.01*ao.ac_path_tangent_current[0],
//                                                   color_ac_path_selected);

            }

            // print instructions
            for (int i = 0; i < 2; ++i)
                cv::putText(cam_images[i], ui_instructions.str(),
                            cv::Point(50, 50), 0, 0.8, cv::Scalar(20, 150, 20), 2);



            // publishing the ac path not needed anymore
            //            if (selected_task == Tasks::CricleAC || task_state == MultiplePathsTask::Status::ACSelected) {
            //                geometry_msgs::PoseArray out;
            //                VecPoint3dToPoseArray(ac_path_in_use, out);
            //                out.header.stamp = ros::Time::now();
            //                out.header.frame_id = "/task_space";
            //                ao.publisher_ac_path.publish(out);
            //            }

            // ----------------------------------------------------------------------------------------------
            //                              VTK test
            // ----------------------------------------------------------------------------------------------
            counter++;
            sphereActor->SetPosition(0.012 + 0.05 * sin(double(counter)/100*M_PI), 0.00, 0.0);
            sphereActor->Modified();

            vtkSmartPointer<vtkMatrix4x4> vtk_matrix =vtkSmartPointer<vtkMatrix4x4>::New();
            VTKConversions::KDLFrameToVTKMatrix(ao.pose_current_tool[0], vtk_matrix);
            tool_frame_axes->SetUserMatrix(vtk_matrix);

            ring_actor->SetUserMatrix(vtk_matrix);
            ring_actor->SetOrientation(90, 0, 0);
            
            graphics.UpdateBackgroundImage(cam_images);
            graphics.UpdateViewAngleForActualWindowSize();

            graphics.Render();

            graphics.GetRenderedImage(augmented_stereo_image);
            cv::imshow(cv_window_name, augmented_stereo_image);

            ao.publisher_stereo_overlayed.publish(cv_bridge::CvImage(std_msgs::Header(),
                                       "bgr8", augmented_stereo_image).toImageMsg());


        } // if new image

        // updating the desired pose happens at the higher frequency
        if(ac_path_in_use.size()>0){

            ClosestPointOnACPathAndItsTangent(ao.pose_current_tool[0].p,
                                              ac_path_in_use,
                                              ao.pose_desired_tool[0].p,
                                              ao.ac_path_tangent_current[0]);
            if (selected_task == Tasks::CricleAC){
                ao.pose_desired_tool[0].M = RingTask::CalculateDesiredOrientation(ao.ac_path_tangent_current[0],
                                                                                  ao.pose_current_tool[0].M);

            }

            ao.PublishDesiredPose();
        }


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



