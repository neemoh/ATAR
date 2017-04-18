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


#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkImageActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkSTLReader.h>
#include <vtkProperty.h>
#include <vtkParametricTorus.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLocator.h>
#include <vtkTransformPolyDataFilter.h>
#include "Rendering.h"
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>

#include <vtkPointData.h>

#include <vtkXMLPolyDataReader.h>



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
    size_t selected_ac = 0;

    std::stringstream ui_instructions;
    ui_instructions <<"Press 1 for task1 and 2 for task2. " ;


    // ----------------------------------------------------------------------------------------------
    //                              VTK test
    // ----------------------------------------------------------------------------------------------

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

    BuzzWireTask btask(0.005, 0.002);


    graphics.AddActorsToScene(btask.GetActors());

    KDL::Vector desired_normal;

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
            if (rc.stereo_tr_calc_client.call(rc.stereo_tr_srv)) {
                //
                std::vector<double> pose_vec_out(7, 0.0);
                conversions::PoseMsgToVector(
                        rc.stereo_tr_srv.response.cam_1_to_cam_2_pose, pose_vec_out);
                rc.n.setParam("left_cam_to_right_cam_transform", pose_vec_out);
                ROS_INFO_STREAM(
                        std::string("Set parameter ") << rc.n.resolveName("left_cam_to_right_cam_transform")
                                                      << " as\n"
                                                      << rc.stereo_tr_srv.response.cam_1_to_cam_2_pose);
            }
            else
                ROS_ERROR("Failed to call service stereo_tr_calc_client");
        }
        else if (key == '1'){
            ROS_INFO("Task 1 Selected");
                 }
        else if (key == '2'){
            ROS_INFO("Task 2 Selected");
             }
        else if (key == '0'){
            ROS_INFO("No task Selected");
        }

        btask.SetCurrentToolPose(rc.pose_current_tool[0]);

        if(rc.new_left_image && rc.new_right_image) {

            rc.ImageLeft(ros::Duration(1)).copyTo(cam_images[0]);
            rc.ImageRight(ros::Duration(1)).copyTo(cam_images[1]);


            // print instructions
            for (int i = 0; i < 2; ++i)
                cv::putText(cam_images[i], ui_instructions.str(),
                            cv::Point(50, 50), 0, 0.8, cv::Scalar(20, 150, 20), 2);

//            for (int j = 0; j <2 ; ++j) {
//
//                //draw the desired pose frame
//                DrawingsCV::DrawCoordinateFrameInTaskSpace(cam_images[j], rc.cam_intrinsics[j],
//                                                           rc.pose_desired_tool[0],
//                                                           rc.cam_rvec[j], rc.cam_tvec[j], 0.005);;
////                //draw the  tangent
////                DrawingsCV::DrawLineFrom2KDLPoints(cam_images[j],
////                                                   ao.cam_intrinsics[j],
////                                                   ao.cam_rvec[j],
////                                                   ao.cam_tvec[j],
////                                                   ao.pose_desired_tool[0].p,
////                                                   ao.pose_desired_tool[0].p + 0.01*ao.ac_path_tangent_current[0],
////                                                   color_ac_path_selected);
//
//            }

            // publishing the ac path not needed anymore
            //            if (selected_task == Tasks::CricleAC || task_state == MultiplePathsTask::Status::ACSelected) {
            //                geometry_msgs::PoseArray out;
            //                VecPoint3dToPoseArray(ac_path_in_use, out);
            //                out.header.stamp = ros::Time::now();
            //                out.header.frame_id = "/task_space";
            //                rc.publisher_ac_path.publish(out);
            //            }

            // ----------------------------------------------------------------------------------------------
            //                              VTK test
            // ----------------------------------------------------------------------------------------------


            btask.UpdateActors();


            graphics.UpdateBackgroundImage(cam_images);
            graphics.UpdateViewAngleForActualWindowSize();

//            point_ids = polydata->GetCell(cell_id)->GetPointIds();

//            std::cout << polydata->GetCell(cell_id)->GetPointIds()<< std::endl;

//            double *point0
//                    = polydata->GetCellData()->GetNormals()->GetTuple(point_ids->GetId(0));
//            std::cout << "\nPoint 0 "<< point_ids->GetId(0) << std::endl;
//            printf("0x -> %f",point0[0]);
//            printf(" 0y -> %f",point0[1]);
//            printf(" 0z -> %f\n",point0[2]);
//            double *point1=
//                    polydata->GetCellData()->GetNormals()->GetTuple(point_ids->GetId(1));
//            std::cout << "\nPoint 1 "<< point_ids->GetId(1) << std::endl;
//            printf("1x -> %f",point1[0]);
//            printf(" 1y -> %f",point1[1]);
//            printf(" 1z -> %f\n",point1[2]);
//            double *point2=
//                    polydata->GetCellData()->GetNormals()->GetTuple(point_ids->GetId(2));
//            std::cout << "\nPoint 2 "<< point_ids->GetId(2) << std::endl;
//            printf("2x -> %f",point2[0]);
//            printf(" 2y -> %f",point2[1]);
//            printf(" 2z -> %f\n",point2[2]);


//            std::cout << "Coordinates of closest point: " << closest_point[0] << " " << closest_point[1] << " " << closest_point[2] << std::endl;
//            std::cout << "Squared distance to closest point: " << closestPointDist2 << std::endl;

            graphics.Render();

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



