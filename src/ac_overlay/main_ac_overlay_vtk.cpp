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


    // --------------------------------------------------
    // Sphere
    vtkSmartPointer<vtkSphereSource> sphereSource=
            vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(0.003);
    sphereSource->SetThetaResolution(20);
    sphereSource->SetPhiResolution(20);
    sphereSource->Update();

    // to transform the data
    vtkSmartPointer<vtkTransform> sphere_translation =
            vtkSmartPointer<vtkTransform>::New();
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(sphereSource->GetOutputPort());
    transformFilter->SetTransform(sphere_translation);
    transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    int counter = 0;
    sphereActor->SetPosition(0.012, 0.00, 0.0);
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(0.8, 0.2, 0.4);


    // --------------------------------------------------
    // RING
    vtkSmartPointer<vtkParametricTorus> parametricObject = vtkSmartPointer<vtkParametricTorus>::New();
    double ring_cross_section_radius = 0.001;
    double ring_radius = 0.007;
    double ring_scale = 0.006;
    parametricObject->SetCrossSectionRadius(ring_cross_section_radius/ ring_scale);
    parametricObject->SetRingRadius(ring_radius/ ring_scale);
    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource =
            vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();


    // to transform the data
    vtkSmartPointer<vtkTransform> ring_local_transform =
            vtkSmartPointer<vtkTransform>::New();
    vtkSmartPointer<vtkTransformPolyDataFilter> ring_local_transform_filter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    ring_local_transform_filter->SetInputConnection(parametricFunctionSource->GetOutputPort());
    ring_local_transform->RotateX(90);
    ring_local_transform->Translate(0.0, ring_radius/ring_scale, 0.0);

    ring_local_transform_filter->SetTransform(ring_local_transform);
    ring_local_transform_filter->Update();

    vtkSmartPointer<vtkPolyDataMapper> ring_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_mapper->SetInputConnection(ring_local_transform_filter->GetOutputPort());

    // Create an line_actor for the contours
    vtkSmartPointer<vtkActor> ring_actor =
            vtkSmartPointer<vtkActor>::New();
    ring_actor->SetMapper(ring_mapper);
    ring_actor->SetScale(ring_scale);
    ring_actor->GetProperty()->SetColor(0.2, 0.4, 0.4);

    // --------------------------------------------------
    // FRAMES

    vtkSmartPointer<vtkAxesActor> task_coordinate_axes = vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkAxesActor> tool_current_frame_axes = vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkAxesActor> tool_desired_frame_axes = vtkSmartPointer<vtkAxesActor>::New();
    // The task_coordinate_axes are positioned with a user transform
//    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
//    transform->Translate(0.0, 0, 0.0);
//    transform->RotateX(0);
//    task_coordinate_axes->SetUserTransform(transform);
    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    tool_current_frame_axes->SetXAxisLabelText("");
    tool_current_frame_axes->SetYAxisLabelText("");
    tool_current_frame_axes->SetZAxisLabelText("");
    tool_current_frame_axes->SetTotalLength(0.007, 0.007, 0.007);
    tool_current_frame_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    tool_desired_frame_axes->SetXAxisLabelText("");
    tool_desired_frame_axes->SetYAxisLabelText("");
    tool_desired_frame_axes->SetZAxisLabelText("");
    tool_desired_frame_axes->SetTotalLength(0.007, 0.007, 0.007);
    tool_desired_frame_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    // --------------------------------------------------
    // MESH
    std::string inputFilename = "/home/charm/Desktop/cads/task1_first_mq.STL";
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(inputFilename.c_str());
    reader->Update();
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->DeepCopy(reader->GetOutput());

    // Genreate Normals
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator->SetInputData(polydata);
    normalGenerator->ComputePointNormalsOff();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();
    /*
    // Optional settings
    normalGenerator->SetFeatureAngle(0.1);
    normalGenerator->SetSplitting(1);
    normalGenerator->SetConsistency(0);
    normalGenerator->SetAutoOrientNormals(0);
    normalGenerator->SetComputePointNormals(1);
    normalGenerator->SetComputeCellNormals(0);
    normalGenerator->SetFlipNormals(0);
    normalGenerator->SetNonManifoldTraversal(1);
    */
    polydata = normalGenerator->GetOutput();
//    vtkSmartPointer<vtkDataArray>  cellNormals = vtkSmartPointer<vtkDataArray>::New();
//    cellNormals = polydata->GetCellData()->GetNormals();

    // transform
    vtkSmartPointer<vtkTransform> mesh_transform = vtkSmartPointer<vtkTransform>::New();
    mesh_transform->Translate(0.04, 0.05, 0.0);
    mesh_transform->RotateX(90);
    mesh_transform->RotateY(90);

    vtkSmartPointer<vtkTransformPolyDataFilter> mesh_transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    mesh_transformFilter->SetInputConnection(reader->GetOutputPort());
    mesh_transformFilter->SetTransform(mesh_transform);
    mesh_transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mesh_mapper->SetInputConnection(mesh_transformFilter->GetOutputPort());
//    std::cout << "reader: " << mesh_mapper->GetLength() << std::endl;
    vtkSmartPointer<vtkActor> mesh_actor = vtkSmartPointer<vtkActor>::New();
    mesh_actor->SetMapper(mesh_mapper);
//    mesh_actor->SetPosition(0.00, 0.00, 0.0);
//    mesh_actor->SetScale(0.0005);
//    mesh_actor->SetOrientation(90, 0.0 , 90.0);
    mesh_actor->GetProperty()->SetColor(0.7, 0.5, 0.2);
    mesh_actor->GetProperty()->SetSpecular(0.8);
    // --------------------------------------------------
    // CLOSEST POINT
    // Create the tree
    vtkSmartPointer<vtkCellLocator> cellLocator =
            vtkSmartPointer<vtkCellLocator>::New();
//    cellLocator->SetDataSet(sphereActor->GetMapper()->GetInput());
    cellLocator->SetDataSet(mesh_transformFilter->GetOutput());
    cellLocator->BuildLocator();

    double tool_point[3] = {0.0, 0.0, 0.0};
    //Find the closest points to TestPoint
    double closest_point[3];//the coordinates of the closest point will be returned here
    double closestPointDist2; //the squared distance to the closest point will be returned here
    vtkIdType cell_id; //the cell id of the cell containing the closest point will be returned here
    int subId; //this is rarely used (in triangle strips only, I believe)

    // ------------------------------------
    // closest point line
    vtkSmartPointer<vtkLineSource> lineSource =
            vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(tool_point);
    lineSource->SetPoint2(closest_point);
    lineSource->Update();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> line_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    line_mapper->SetInputConnection(lineSource->GetOutputPort());
    vtkSmartPointer<vtkActor> line_actor =
            vtkSmartPointer<vtkActor>::New();
    line_actor->SetMapper(line_mapper);
    line_actor->GetProperty()->SetLineWidth(4);


    graphics.AddActorToScene(task_coordinate_axes);
    graphics.AddActorToScene(tool_current_frame_axes);
    graphics.AddActorToScene(tool_desired_frame_axes);
    graphics.AddActorToScene(sphereActor);
    graphics.AddActorToScene(mesh_actor);
    graphics.AddActorToScene(ring_actor);
    graphics.AddActorToScene(line_actor);

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


            vtkSmartPointer<vtkMatrix4x4> tool_current_vtkmatrix =vtkSmartPointer<vtkMatrix4x4>::New();
            VTKConversions::KDLFrameToVTKMatrix(rc.pose_current_tool[0], tool_current_vtkmatrix);
            tool_current_frame_axes->SetUserMatrix(tool_current_vtkmatrix);

            ring_actor->SetUserMatrix(tool_current_vtkmatrix);

//            ring_actor->SetOrientation(90, 0, 0);

            vtkSmartPointer<vtkMatrix4x4> tool_desired_vtkmatrix =vtkSmartPointer<vtkMatrix4x4>::New();
            VTKConversions::KDLFrameToVTKMatrix(rc.pose_desired_tool[0], tool_desired_vtkmatrix);
            tool_desired_frame_axes->SetUserMatrix(tool_desired_vtkmatrix);

            
            graphics.UpdateBackgroundImage(cam_images);
            graphics.UpdateViewAngleForActualWindowSize();

            counter++;
//            sphereActor->SetPosition(0.012 + 0.05 * sin(double(counter)/100*M_PI), 0.00, 0.0);
//            sphereActor->Modified();
            double dx = 0.05 * sin(double(counter)/100*M_PI);
            sphere_translation->Translate(dx/100, 0.00, 0.0);
            tool_point[0] = rc.pose_current_tool[0].p[0];
            tool_point[1] = rc.pose_current_tool[0].p[1];
            tool_point[2] = rc.pose_current_tool[0].p[2];

//            transformFilter->SetTransform(sphere_translation);
//            transformFilter->Update();
//            sphereActor->Modified();
            cellLocator->Update();
            cellLocator->FindClosestPoint(tool_point, closest_point, cell_id, subId, closestPointDist2);
            vtkSmartPointer<vtkIdList> point_ids  = vtkSmartPointer<vtkIdList>::New();



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

            lineSource->SetPoint1(tool_point);
            lineSource->SetPoint2(closest_point);

//            std::cout << "Coordinates of closest point: " << closest_point[0] << " " << closest_point[1] << " " << closest_point[2] << std::endl;
//            std::cout << "Squared distance to closest point: " << closestPointDist2 << std::endl;

            graphics.Render();

            graphics.GetRenderedImage(augmented_stereo_image);
            cv::imshow(cv_window_name, augmented_stereo_image);

            rc.publisher_stereo_overlayed.publish(cv_bridge::CvImage(std_msgs::Header(),
                                                                     "bgr8", augmented_stereo_image).toImageMsg());


        } // if new image

        // updating the desired pose happens at the higher frequency

        rc.pose_desired_tool[0].p = KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);
        // find desired orientation
        desired_normal = rc.pose_desired_tool[0].p - rc.pose_current_tool[0].p;

        rc.pose_desired_tool[0].M = CalculateDesiredOrientation(desired_normal,
                                                                rc.pose_current_tool[0].M);
        rc.PublishDesiredPose();



        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



