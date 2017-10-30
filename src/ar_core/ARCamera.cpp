//
// Created by nima on 4/12/17.
//

#include "ARCamera.h"
#include <pwd.h>
#include <custom_conversions/Conversions.h>


//----------------------------------------------------------------------------
ARCamera::ARCamera(ros::NodeHandlePtr n, image_transport::ImageTransport *it,
                   const std::string cam_name)
        :
        intrinsic_matrix(NULL)
        , image_width_(640)
        , image_height_(480)
        , fx_(240)
        , fy_(320)
        , cx_(0)
        , cy_(0)
{
    // if there is no name and image transport we assume the camera is not
    // augmented reality type
    bool is_ar = true;
    if(cam_name=="" || it==NULL)
        is_ar = false;

    intrinsic_matrix = vtkMatrix4x4::New();
    intrinsic_matrix->Identity();

    camera_virtual = vtkSmartPointer<vtkCamera>::New();

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

        image_importer_ = vtkSmartPointer<vtkImageImport>::New();
        image_actor_ = vtkSmartPointer<vtkImageActor>::New();
        camera_image_ = vtkSmartPointer<vtkImageData>::New();
        camera_real = vtkSmartPointer<vtkCamera>::New();

        // --------------------Images
        // image subscriber
        std::string img_topic = "/"+cam_name+ "/image_raw";;
        sub_image = it->subscribe(img_topic, 1, &ARCamera::ImageCallback, this);

        // in AR mode we read real camera images and show them as the background
        // of our rendering
        cv::Mat cam_image;
        LockAndGetImage(cam_image, img_topic);
        ConfigureBackgroundImage(cam_image);

        // ------------------ CAM POSE
        // we first try to read the poses as parameters and later update the
        // poses if new messages are arrived on the topics
        std::vector<double> temp_vec = std::vector<double>( 7, 0.0);
        if (n->getParam("/calibrations/world_frame_to_"+cam_name+"_frame", temp_vec))
            conversions::PoseVectorToKDLFrame(temp_vec, world_to_cam_pose);

        SetCameraPose();

        // now we set up the subscribers
        sub_pose = n->subscribe("/"+cam_name+ "/world_to_camera_transform",
                                1, &ARCamera::PoseCallback, this);

    }

    is_initialized = true;

}



////----------------------------------------------------------------------------
//void ARCamera::SetIntrinsicParameters(const double& fx, const double& fy,
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




void ARCamera::UpdateVirtualView(const double &window_width,
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

void ARCamera::SetCemraToFaceImage(const int *window_size,
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

    double vectorAlongZ[3] = {0, 0, -1000};

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
void ARCamera::ReadCameraParameters(const std::string file_path) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);


    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");
    cv::Mat camera_matrix;
    fs["camera_matrix"] >> camera_matrix;

    //    cv::Mat camera_distortion;
    //    fs["distortion_coefficients"] >> camera_distortion;

    // check if we got something
    if(camera_matrix.empty()){
        ROS_ERROR("camera_matrix not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    //    if(camera_distortion.empty()){
    //        ROS_ERROR("distortion_coefficients  not found in '%s' ", file_path.c_str());
    //        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    //    }

    fx_ = camera_matrix.at<double>(0, 0);
    fy_ = camera_matrix.at<double>(1, 1);
    cx_ = camera_matrix.at<double>(0, 2);
    cy_ = camera_matrix.at<double>(1, 2);

    ROS_DEBUG_STREAM(std::string("Camera Matrix: fx= ")
                             <<  fx_ << ", fy= " <<  fy_ << ", cx= "<<  cx_
                             << ", cy= " <<  cy_ );
}

void ARCamera::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_from_ros = cv_bridge::toCvCopy(msg, "rgb8")->image;
        new_image= true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ARCamera::PoseCallback(
        const geometry_msgs::PoseStampedConstPtr & msg)
{
    new_cam_pose = true;
    tf::poseMsgToKDL(msg->pose, world_to_cam_pose);

}


//------------------------------------------------------------------------------
void ARCamera::SetCameraPose() {

    KDL::Frame cam_to_world_pose = world_to_cam_pose.Inverse();

    KDL::Vector origin = cam_to_world_pose.p;
    KDL::Vector focal_point = cam_to_world_pose * KDL::Vector(0, 0, 1);
    KDL::Vector view_up = cam_to_world_pose.M * KDL::Vector(0, -1, 0);

    camera_virtual->SetPosition(origin[0], origin[1], origin[2]);
    camera_virtual->SetFocalPoint(focal_point[0], focal_point[1], focal_point[2]);
    camera_virtual->SetViewUp(view_up[0], view_up[1], view_up[2]);
    camera_virtual->SetClippingRange(0.05, 10);
    camera_virtual->Modified();
}

void ARCamera::UpdateBackgroundImage(const int *window_size) {

    SetCameraPose();

    if(is_initialized && !image_from_ros.empty()) {
        image_from_ros.copyTo(img);
        //    cv::flip(src, _src, 0);
        image_importer_->SetImportVoidPointer(img.data);
        image_importer_->Modified();
        image_importer_->Update();

        int imageSize[3];
        image_importer_->GetOutput()->GetDimensions(imageSize);

        double spacing[3];
        image_importer_->GetOutput()->GetSpacing(spacing);

        double origin[3];
        image_importer_->GetOutput()->GetOrigin(origin);

        SetCemraToFaceImage(window_size, imageSize, spacing, origin);
    }
}

void ARCamera::ConfigureBackgroundImage(cv::Mat img) {

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
ARCamera::LockAndGetImage(cv::Mat &images, std::string img_topic) {

    ros::Rate loop_rate(2);
    ros::Time timeout_time = ros::Time::now() + ros::Duration(1);

    while(ros::ok() && image_from_ros.empty()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (ros::Time::now() > timeout_time)
            ROS_WARN_STREAM(("Timeout: No Image on."+img_topic+
                             " Trying again...").c_str());
    }
    image_from_ros.copyTo(images);

    new_image = false;
}

bool ARCamera::IsImageNew() {
    if(new_image){
        new_image = false;
        return true;
    }
    return false;
}

void
ARCamera::SetWorldToCamTf(const KDL::Frame & frame) {
    world_to_cam_pose = frame;
    SetCameraPose();
}

//ARCamera::ARCamera(const ARCamera &) {
//
//}
