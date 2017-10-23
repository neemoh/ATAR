//
// Created by nima on 4/12/17.
//

#include "CalibratedCamera.h"
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>
#include <ros/ros.h> // added only for ros_debug
#include <pwd.h>
#include <custom_conversions/Conversions.h>

//
//
//#ifndef VTK_IMPLEMENT_MESA_CXX
//vtkStandardNewMacro(CalibratedCamera);
//#endif

//----------------------------------------------------------------------------
CalibratedCamera::CalibratedCamera(ros::NodeHandle *n,
                                   const std::string cam_name)
        : intrinsic_matrix(NULL)
        , image_width_(640)
        , image_height_(480)
        , fx_(1)
        , fy_(1)
        , cx_(0)
        , cy_(0)
{
    // if there is no name we assume the camera is not augmented reality type
    bool is_ar = false;
    if(cam_name!="")
        is_ar = true;

    intrinsic_matrix = vtkMatrix4x4::New();
    intrinsic_matrix->Identity();

    image_importer_ = vtkSmartPointer<vtkImageImport>::New();
    image_actor_ = vtkSmartPointer<vtkImageActor>::New();
    camera_image_ = vtkSmartPointer<vtkImageData>::New();

    camera_virtual = vtkSmartPointer<vtkCamera>::New();
    camera_real = NULL;

    if(is_ar)
        camera_real = vtkSmartPointer<vtkCamera>::New();


    //--------------- SET INTRINSICS
    if(is_ar) {
        // AR camera
        struct passwd *pw = getpwuid(getuid());
        const char *home_dir = pw->pw_dir;
        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << cam_name << "_intrinsics.yaml";
        ReadCameraParameters(path.str());
    }else{
        // NOT AR. TODO:  SET THE INTRINSICS
    }

    if(is_ar) {
        // --------------------GET IMAGE IF AR
        // image subscriber
        std::string left_image_topic_name = "/camera/left/image_color";;
        if (n->getParam("left_image_topic_name", left_image_topic_name))
            ROS_DEBUG(
                    "[SUBSCRIBERS] Left cam images from '%s'",
                    left_image_topic_name.c_str());
        sub_image = it->subscribe(
                left_image_topic_name, 1, &CalibratedCamera::ImageCallback, this);
    }
    // ------------------ CAM POSE

    // we first try to read the poses as parameters and later update the
    // poses if new messages are arrived on the topics
    std::vector<double> temp_vec = std::vector<double>( 7, 0.0);
    // left cam pose as parameter
    if (n->getParam("/calibrations/world_frame_to_"+cam_name+"_frame",
                    temp_vec)) {
        KDL::Frame pose_cam;
        conversions::VectorToKDLFrame(temp_vec, pose_cam);
        conversions::KDLFrameToRvectvec(pose_cam, cam_rvec_, cam_tvec_);
    }

    // now we set up the subscribers
    std::stringstream topic_name;
    topic_name << std::string("/") << cam_name
               << "/world_to_camera_transform";
    ROS_DEBUG("[SUBSCRIBERS] Left came pose from '%s'",
              topic_name.str().c_str());
    sub_pose = n->subscribe(
            topic_name.str(), 1, &CalibratedCamera::PoseCallback, this);

    // in AR mode we read real camera images and show them as the background
    // of our rendering
    if (is_ar){
        cv::Mat cam_image;
        LockAndGetImage(cam_image);
        ConfigureBackgroundImage(cam_image);
    }


    is_initialized = true;







}



////----------------------------------------------------------------------------
//void CalibratedCamera::SetIntrinsicParameters(const double& fx, const double& fy,
//                                              const double& cx, const double& cy)
//{
//    fx_ = fx;
//    fy_ = fy;
//    cx_ = cx;
//    cy_ = cy;
//
//    ROS_DEBUG_STREAM( std::string("Camera Matrix: fx= ") <<  fx_ << ", fy= " <<  fy_
//              << ", cx= " <<  cx_ << ", cy= " <<  cy_ );
//}




void CalibratedCamera::UpdateVirtualView(const double &window_width,
                                         const double &window_height) {

    //When window aspect ratio is different than that of the image we need to
    // take that into account
    double image_aspect_ratio =  image_width_ / image_height_;
    double window_aspect_ratio =  window_width / window_height;

    double window_resize_factor;
    if(window_aspect_ratio >= image_aspect_ratio)
        window_resize_factor = window_height / image_height_;
    else
        window_resize_factor = window_width / image_width_;

    // calculate the view angle and set it.
    double view_angle = 2*atan((window_height/2)
                               / (window_resize_factor*fy_) ) * 180/M_PI;
    camera_virtual->SetViewAngle(view_angle);

    // convert the principal point to window center
    double wc_x = -2 * window_resize_factor*(cx_ - image_width_/2)
                  / window_width;
    double wc_y =  2 * window_resize_factor*(cy_ - image_height_/2)
                   / window_height;
    camera_virtual->SetWindowCenter(wc_x, wc_y);

    camera_virtual->Modified();
}

void CalibratedCamera::SetCemraToFaceImage(const int *window_size,
                                           const int imageSize[], const double spacing[],
                                           const double origin[]) {


    double clippingRange[2];
    clippingRange[0] = 1;
    clippingRange[1] = 100000;

    double distanceAlongX = (spacing[0] * (imageSize[0] - 1)) / 2.0;
    double vectorAlongX[3] = {1, 0, 0};
    vectorAlongX[0] = distanceAlongX;

    double distanceAlongY = (spacing[1] * (imageSize[1] - 1)) / 2.0;
    double vectorAlongY[3] = {0, 1, 0};
    vectorAlongY[1] = distanceAlongY;

    double distanceToFocalPoint = -1000;
    double vectorAlongZ[3] = {0, 0, 1};
    vectorAlongZ[2] = distanceToFocalPoint;

    double viewUpScaleFactor = 1.0e9;
    if (true) {
        viewUpScaleFactor *= -1;
    }

    double focalPoint[3] = {0, 0, 1};
    for (unsigned int i = 0; i < 3; ++i) {
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

    double imageWidth = imageSize[0] * spacing[0];
    double imageHeight = imageSize[1] * spacing[1];

    double widthRatio = imageWidth / window_size[0];
    double heightRatio = imageHeight / window_size[1];

    double scale;
    if (widthRatio > heightRatio) {
        scale = 0.5 * imageWidth *
                ((double) window_size[1] / (double)(window_size[0]));
    } else {
        scale = 0.5 * imageHeight;
    }

    camera_real->SetPosition(position);
    camera_real->SetFocalPoint(focalPoint);
    camera_real->SetViewUp(viewUp);
    camera_real->SetParallelProjection(true);
    camera_real->SetParallelScale(scale);
    camera_real->SetClippingRange(clippingRange);

}


// -----------------------------------------------------------------------------
void CalibratedCamera::ReadCameraParameters(const std::string file_path) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);


    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");
    cv::Mat camera_matrix;
    cv::Mat camera_distortion;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> camera_distortion;

    // check if we got something
    if(camera_matrix.empty()){
        ROS_ERROR("distortion_coefficients not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(camera_distortion.empty()){
        ROS_ERROR("camera_matrix not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }

    fx_ = camera_matrix.at<double>(0, 0);
    fy_ = camera_matrix.at<double>(1, 1);
    cx_ = camera_matrix.at<double>(0, 2);
    cy_ = camera_matrix.at<double>(1, 2);

    ROS_DEBUG_STREAM(std::string("Camera Matrix: fx= ")
                             <<  fx_ << ", fy= " <<  fy_ << ", cx= "<<  cx_
                             << ", cy= " <<  cy_ );
}

void CalibratedCamera::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_from_ros = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_image= true;
        if(is_initialized)
            UpdateBackgroundImage(image_from_ros);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CalibratedCamera::PoseCallback(
        const geometry_msgs::PoseStampedConstPtr & msg)
{
    KDL::Frame world_to_cam_pose;
    new_cam_pose = true;
    tf::poseMsgToKDL(msg->pose, world_to_cam_pose);
    conversions::KDLFrameToRvectvec(world_to_cam_pose, cam_rvec_, cam_tvec_);

    SetWorldToCameraTransform(cam_rvec_, cam_tvec_);
}


//------------------------------------------------------------------------------
void CalibratedCamera::SetWorldToCameraTransform(const cv::Vec3d cam_rvec,
                                          const cv::Vec3d cam_tvec) {


        cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
        cv::Rodrigues(cam_rvec, rotationMatrix);

        vtkSmartPointer<vtkMatrix4x4>
                world_to_camera_transform =
                vtkSmartPointer<vtkMatrix4x4>::New();
        world_to_camera_transform->Identity();

        // Convert to VTK matrix.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                world_to_camera_transform->SetElement(
                        i, j, rotationMatrix.at<double>(
                                i, j
                        ));
            }
            world_to_camera_transform->SetElement(i, 3, cam_tvec[i]);
        }

        vtkSmartPointer<vtkMatrix4x4> camera_to_world_transform =
                vtkSmartPointer<vtkMatrix4x4>::New();
        camera_to_world_transform->Identity();

        camera_to_world_transform->DeepCopy(world_to_camera_transform);

        camera_to_world_transform->Invert();

    double origin[4]     = {0,  0,   0, 1};
    double focalPoint[4] = {0,  0,   1, 1};
    double viewUp[4]     = {0, -1,   0, 1};

    camera_to_world_transform->MultiplyPoint(origin, origin);
    camera_to_world_transform->MultiplyPoint(focalPoint, focalPoint);
    camera_to_world_transform->MultiplyPoint(viewUp, viewUp);
    viewUp[0] = viewUp[0] - origin[0];
    viewUp[1] = viewUp[1] - origin[1];
    viewUp[2] = viewUp[2] - origin[2];

    camera_virtual->SetPosition(origin[0], origin[1], origin[2]);
    camera_virtual->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
    camera_virtual->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
    camera_virtual->SetClippingRange(0.05, 10);
    camera_virtual->Modified();



    //lights[0]->SetPosition(
    //    camera_to_world_transform_[0]->Element[0][3],
    //    camera_to_world_transform_[0]->Element[1][3],
    //    camera_to_world_transform_[0]->Element[2][3]
    //);


}

void CalibratedCamera::SetRealCameraToFaceImage(const int *window_size) {
    int imageSize[3];
    image_importer_->GetOutput()->GetDimensions(imageSize);

    double spacing[3];
    image_importer_->GetOutput()->GetSpacing(spacing);

    double origin[3];
    image_importer_->GetOutput()->GetOrigin(origin);

    SetCemraToFaceImage(window_size, imageSize,
                                     spacing, origin);
}

void CalibratedCamera::UpdateBackgroundImage(cv::Mat img) {

    for (int i = 0; i < 2; ++i) {
//    cv::flip(src, _src, 0);
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        image_importer_->SetImportVoidPointer( img.data );
        image_importer_->Modified();
        image_importer_->Update();
    }
}

void CalibratedCamera::ConfigureBackgroundImage(cv::Mat img) {

    assert( img.data != NULL );

    image_width_ = img.size().width;
    image_height_ =  img.size().height;

    if (camera_image_) {
        image_importer_->SetOutput(camera_image_);
    }
    image_importer_->SetDataSpacing(1, 1, 1);
    image_importer_->SetDataOrigin(0, 0, 0);
    image_importer_->SetWholeExtent(0, (int)image_width_ - 1, 0,
                                    (int)image_height_ - 1, 0, 0);
    image_importer_->SetDataExtentToWholeExtent();
    image_importer_->SetDataScalarTypeToUnsignedChar();
    image_importer_->SetNumberOfScalarComponents(img.channels());
    image_importer_->SetImportVoidPointer(img.data);
    image_importer_->Update();

    image_actor_->SetInputData(camera_image_);

}

void
CalibratedCamera::LockAndGetImage(cv::Mat &images) {


    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + ros::Duration(1);

    while(image_from_ros.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time)
            ROS_WARN("Timeout: No new left Image. Trying again...");
    }
    image_from_ros.copyTo(images);

    new_image = false;


}

bool CalibratedCamera::IsImageNew() {
    if(new_image){
        new_image = false;
        return true;
    }
    return false;
}
