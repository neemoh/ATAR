#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "ACOverlay.h"
#include <std_msgs/Float32.h>
#include "bardCalibratedCamera.h"




#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkImageActor.h>
#include <vtkSuperquadricSource.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkCamera.h>
#include <vtkImageImport.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include "Rendering.h"
//#include "Drawings.h"
//#include <iostream>
//#include <GLFW/glfw3.h>
//#include <GL/glu.h>

bool fromIpl2Vtk( cv::Mat _src, vtkSmartPointer<vtkImageImport> importer, vtkImageData* _dest )
{
    assert( _src.data != NULL );


    if ( _dest )
    {
        importer->SetOutput( _dest );
    }
    importer->SetDataSpacing( 1, 1, 1 );
    importer->SetDataOrigin( 0, 0, 0 );
    importer->SetWholeExtent(   0, _src.size().width-1, 0,
                                _src.size().height-1, 0, 0 );
    importer->SetDataExtentToWholeExtent();
    importer->SetDataScalarTypeToUnsignedChar();
    importer->SetNumberOfScalarComponents( _src.channels() );
    importer->SetImportVoidPointer( _src.data );
    importer->Update();
    return true;
}


void SetCamExtrinsics(const cv::Vec3d &cam_rvec, const cv::Vec3d &cam_tvec,
                      vtkSmartPointer<vtkCamera> camera){

    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(cam_rvec, rotationMatrix);

    vtkSmartPointer<vtkMatrix4x4> m_WorldToCameraTransform =
            vtkSmartPointer<vtkMatrix4x4>::New();
    m_WorldToCameraTransform->Identity();

// Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_WorldToCameraTransform->SetElement(i, j, rotationMatrix.at<double>(i, j));
        }
        m_WorldToCameraTransform->SetElement(i, 3, cam_tvec[i]);
    }


    vtkSmartPointer<vtkMatrix4x4> m_CameraToWorldTransform=
            vtkSmartPointer<vtkMatrix4x4>::New();
    m_CameraToWorldTransform->DeepCopy(m_WorldToCameraTransform);
    m_CameraToWorldTransform->Invert();

    // This implies a right handed coordinate system.
    // By default, assume camera position is at origin, looking down the world z-axis.
    double origin1[4]     = {0, 0,    0,    1};
    double focalPoint[4] = {0, 0,   1, 1};
    double viewUp[4]     = {0, -1, 0,    1};

    m_CameraToWorldTransform->MultiplyPoint(origin1, origin1);
    m_CameraToWorldTransform->MultiplyPoint(focalPoint, focalPoint);
    m_CameraToWorldTransform->MultiplyPoint(viewUp, viewUp);
    viewUp[0] = viewUp[0] - origin1[0];
    viewUp[1] = viewUp[1] - origin1[1];
    viewUp[2] = viewUp[2] - origin1[2];

    camera->SetPosition(origin1[0], origin1[1], origin1[2]);
    camera->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
    camera->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
//    camera.SetClippingRange(1, 5000);
//
//    camera->SetFocalPoint(0, 0, 0);
//    camera->SetPosition(0.0, 0.0, 0.1 );
    std::cout << "Camera position = " << origin1[0] << ", " << origin1[1] << ", " << origin1[2] << std::endl;
}


void SetImageCameraToFaceImage(vtkSmartPointer<vtkCamera> camera, vtkImageData* ImageData,
                               vtkRenderWindow* renderWindow)
{

    int    windowSize[2];
    windowSize[0] = renderWindow->GetSize()[0];
    windowSize[1] = renderWindow->GetSize()[1];

    int    imageSize[3];
    ImageData->GetDimensions(imageSize);

    double spacing[3];
    ImageData->GetSpacing(spacing);

    double origin[3];
    ImageData->GetOrigin(origin);

    double clippingRange[2];
    clippingRange[0] = 1;
    clippingRange[1] = 100000;

    double distanceAlongX = ( spacing[0] * (imageSize[0] - 1) ) / 2.0;
    double vectorAlongX[3] = {1, 0, 0};
    vectorAlongX[0] = distanceAlongX;

    double distanceAlongY = ( spacing[1] * (imageSize[1] - 1) ) / 2.0;
    double vectorAlongY[3] = {0, 1, 0};
    vectorAlongY[1] = distanceAlongY;

    double distanceToFocalPoint = -1000;
    double vectorAlongZ[3] = {0, 0, 1};
    vectorAlongZ[2] = distanceToFocalPoint;

    double viewUpScaleFactor = 1.0e9;
    if ( true )
    {
        viewUpScaleFactor *= -1;
    }

    double focalPoint[3] = {0, 0, 1};
    for ( unsigned int i = 0; i < 3; ++i)
    {
        focalPoint[i] = origin[i] + vectorAlongX[i] + vectorAlongY[i];
    }

    double position[3] = {0, 0, 0};
    position[0] = focalPoint[0] + vectorAlongZ[0];
    position[1] = focalPoint[1] + vectorAlongZ[1];
    position[2] = focalPoint[2] + vectorAlongZ[2];

    double viewUp[3] = {0, 1, 0};
    viewUp[0] = vectorAlongY[0] * viewUpScaleFactor;
    viewUp[1] = vectorAlongY[1] * viewUpScaleFactor;
    viewUp[2] = vectorAlongY[2] * viewUpScaleFactor;

    double imageWidth = imageSize[0]*spacing[0];
    double imageHeight = imageSize[1]*spacing[1];

    double widthRatio = imageWidth / windowSize[0];
    double heightRatio = imageHeight / windowSize[1];

    double scale = 1;
    if (widthRatio > heightRatio)
    {
        scale = 0.5*imageWidth*((double)windowSize[1]/(double)windowSize[0]);
    }
    else
    {
        scale = 0.5*imageHeight;
    }

    camera->SetPosition(position);
    camera->SetFocalPoint(focalPoint);
    camera->SetViewUp(viewUp);
    camera->SetParallelProjection(true);
    camera->SetParallelScale(scale);
    camera->SetClippingRange(clippingRange);
}




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

    Rendering *rend= new Rendering;

    rend->SetWorldToCameraTransform(ao.cam_rvec[0], ao.cam_tvec[0]);
    rend->SetCameraIntrinsics(ao.cam_intrinsics->camMatrix);
    rend->SetupBackgroundImage(cam_images[0]);
    rend->SetEnableImage(true);

//    vtkSmartPointer<vtkImageData> imageData =
//            vtkSmartPointer<vtkImageData>::New();
//    vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New();
//
//    fromIpl2Vtk(cam_images[0], importer, imageData);
//
//// Create an image actor to display the image
//    vtkSmartPointer<vtkImageActor> imageActor =
//            vtkSmartPointer<vtkImageActor>::New();
//    imageActor->SetInputData(imageData);

//    // Create a renderer to display the image in the background
//    vtkSmartPointer<vtkRenderer> backgroundRenderer =
//            vtkSmartPointer<vtkRenderer>::New();

    rend->renderWindow->Render();

    // Create a superquadric
    vtkSmartPointer<vtkSphereSource> sphereSource=
            vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(0.03);
    sphereSource->SetThetaResolution(20);
    sphereSource->SetPhiResolution(20);

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetPosition(0.012, 0.00, 0.0);
    sphereActor->SetMapper(sphereMapper);

//    vtkSmartPointer<vtkRenderer> sceneRenderer = vtkSmartPointer<vtkRenderer>::New();
//

//
//    // Set up the render window and renderers such that there is
//    // a background layer and a foreground layer
//    backgroundRenderer->SetLayer(0);
//    backgroundRenderer->InteractiveOff();
//    sceneRenderer->SetLayer(1);
//    sceneRenderer->InteractiveOff();
//    renderWindow->SetNumberOfLayers(2);
//    renderWindow->AddRenderer(backgroundRenderer);
//    renderWindow->AddRenderer(sceneRenderer);

    // ----------------------------------------------------------------------------------------------
    //                              VTK test


//    vtkSmartPointer<vtkCamera> scene_camera =
//            vtkSmartPointer<vtkCamera>::New();
//    SetCamExtrinsics(ao.cam_rvec[0], ao.cam_tvec[0], scene_camera);
//
//
//
//    sceneRenderer->SetActiveCamera(scene_camera);
////    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
////            vtkSmartPointer<vtkRenderWindowInteractor>::New();
////    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Add actors to the renderers
//    sceneRenderer->AddActor(sphereActor);
    rend->AddActorToScene(sphereActor);
//    backgroundRenderer->AddActor(imageActor);

    vtkSmartPointer<vtkTransform> transform =
            vtkSmartPointer<vtkTransform>::New();
    transform->Translate(0.0, 0, 0.0);
    transform->RotateX(0);
    vtkSmartPointer<vtkAxesActor> axes =
            vtkSmartPointer<vtkAxesActor>::New();

    // The axes are positioned with a user transform
    axes->SetUserTransform(transform);
    axes->SetXAxisLabelText("");
    axes->SetYAxisLabelText("");
    axes->SetZAxisLabelText("");
//    axes->SetCylinderRadius(0.002);
//    axes->SetConeRadius(0.04);
    axes->SetTotalLength(0.01, 0.01, 0.01);

//    sceneRenderer->AddActor(axes);
    rend->AddActorToScene(axes);


    // sizes
    int img_height = cam_images[0].rows;
    int img_width = cam_images[0].cols;
    int window_height = img_height;
    int window_width = img_width;

    rend->renderWindow->SetSize(window_width, window_height);

    // Render once to figure out where the background scene_camera will be
//    rend->renderWindow->Render();

    // Set up the background scene_camera to fill the renderer with the image

//    vtkSmartPointer<vtkCamera> background_camera = vtkSmartPointer<vtkCamera>::New();
//    backgroundRenderer->SetActiveCamera(background_camera);
////    background_camera->ParallelProjectionOn();
//
//    SetImageCameraToFaceImage(background_camera, imageData, renderWindow);

    // Render again to set the correct view
//    rend->renderWindow->Render();



    // Screenshot
//    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
//            vtkSmartPointer<vtkWindowToImageFilter>::New();
//    windowToImageFilter->SetInput(renderWindow);
//    windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
//    windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
//    windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
//    windowToImageFilter->Update();
//
//    vtkSmartPointer<vtkPNGWriter> writer =
//            vtkSmartPointer<vtkPNGWriter>::New();
//    writer->SetFileName("screenshot2.png");
//    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
//    writer->Write();

//    renderWindow->Render();
//    renderer->ResetCamera();
//    renderWindow->Render();
//    renderWindowInteractor->Start();

//-----------------------------------------------------      VTK TO CV
//    vtkImageData* image = windowToImageFilter->GetOutput();
//// Check number of components.
//    const int numComponents =  image->GetNumberOfScalarComponents(); // 3
//
//// Construct the OpenCv Mat
//    int dims[3];
//    image->GetDimensions(dims);
//    cv::Mat openCVImage(dims[0], dims[1], CV_8UC4, image->GetScalarPointer()); // Unsigned int, 4 channels
//
//    cvtColor(openCVImage, openCVImage, CV_BGRA2GRAY);
//
//// Flip because of different origins between vtk and OpenCV
//    cv::flip(openCVImage,openCVImage, 0);
// -------------------------------------------------------------------------









//    renderWindow->Render();

//    eye = (-R'T);
//    center = eye + R'(0,0,1)
//    up = -R[0][1]


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
                // draw the coordinate frame of the board
                DrawingsCV::DrawCoordinateFrameInTaskSpace(cam_images[j], ao.cam_intrinsics[j],
                                                           KDL::Frame(),
                                                           ao.cam_rvec[j], ao.cam_tvec[j], 0.01);
                // draw the end-effector ref frame
                DrawingsCV::DrawCoordinateFrameInTaskSpace(cam_images[j], ao.cam_intrinsics[j],
                                                           ao.pose_current_tool[0],
                                                           ao.cam_rvec[j], ao.cam_tvec[j], 0.01);
                //draw the desired pose frame
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


            for (int j = 0; j <2 ; ++j) {
                cv::imshow(window_name[j], cam_images[j]);
                ao.publisher_overlayed[j].publish(
                        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_images[j]).toImageMsg());

            }

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

////            cvCvtColor(cam_images[0].data, im, CV_BGRA2RGB);
//            cv::Mat _src;
////    cv::flip(src, _src, 0);
//            cv::cvtColor(cam_images[0], _src, cv::COLOR_BGR2RGB);
//            importer->SetImportVoidPointer( _src.data );
//
//            importer->Update();
////            fromIpl2Vtk(_src, importerimageData);
//
//            imageActor->SetInputData(imageData);
//            SetImageCameraToFaceImage(background_camera, imageData, renderWindow);

            rend->UpdateBackgroundImage(cam_images[0]);
            rend->UpdateWindowSizeRelatedViews();

//            double focalLengthY = ao.cam_intrinsics[0].camMatrix.at<double>(1,1);
//            int *size = renderWindow->GetSize();
//            int window_width = size[0];
//            int window_height = size[1];
//            int img_height = cam_images[0].rows;
//
//            if( window_height != img_height )
//            {
//                double factor = static_cast<double>(window_height)/static_cast<double>(img_height);
//                focalLengthY = ao.cam_intrinsics[0].camMatrix.at<double>(1,1) * factor;
//            }
//
//            double view_angle = 2 * atan( ( window_height / 2 ) / focalLengthY ) * 180 / M_PI;
//            scene_camera->SetViewAngle(view_angle);

            rend->renderWindow->Render();

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

    delete(rend);
//	glfwDestroyWindow(window);
//	glfwTerminate();
    return 0;
}



