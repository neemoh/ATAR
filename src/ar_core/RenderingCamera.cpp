//
// Created by nima on 4/12/17.
//

#include "RenderingCamera.h"
#include <custom_conversions/Conversions.h>


//----------------------------------------------------------------------------
RenderingCamera::RenderingCamera(const std::vector<int> view_resolution,
                                 image_transport::ImageTransport *it,
                                 const std::string cam_name, const std::string ns)
        :
        intrinsic_matrix(vtkMatrix4x4::New())
        ,camera_virtual(vtkSmartPointer<vtkCamera>::New())
        , image_width_(view_resolution[0])
        , image_height_(view_resolution[1])
        , fx_(1000)
        , fy_(1000)
        , cx_(view_resolution[0]/2)
        , cy_(view_resolution[1]/2)
{

    ros::NodeHandle n("~");

    // if there is no name and image transport we assume the camera is not
    // augmented reality type
    if(!cam_name.empty() && it!= nullptr)
        is_ar = true;

    intrinsic_matrix->Identity();

    // set a default cam pose
    SetWorldToCamTf(KDL::Frame(KDL::Rotation::EulerZYZ(-85*M_PI/180 ,
                                                       110*M_PI/180,
                                                       -75*M_PI/180),
                               KDL::Vector(0.05, -0.0, 0.35)));
    if(is_ar) {
        image_importer_ = vtkSmartPointer<vtkImageImport>::New();
        image_actor_ = vtkSmartPointer<vtkImageActor>::New();
        camera_image_ = vtkSmartPointer<vtkImageData>::New();
        camera_real = vtkSmartPointer<vtkCamera>::New();

        ar_camera = new AugmentedCamera( it, cam_name, ns);

        ar_camera->GetIntrinsicParams(fx_, fy_, cx_, cy_);

        // in AR mode we read real camera images and show them as the background
        // of our rendering
        ConfigureBackgroundImage(ar_camera->LockAndGetImage());
    }

    // this flag is to make sure nothing goes wrong if some refreshes the
    // camera from outside before it is initialized
    is_initialized = true;

}


//------------------------------------------------------------------------------
void RenderingCamera::SetPtrManipulatorInterestedInCamPose(Manipulator *in) {

    interested_manipulators.push_back(in);

    //we need to update once already, because the pose may have been already
    // set and stay constant
    UpdateCamPoseFollowers(world_to_cam_tr);
}


void RenderingCamera::RefreshCamera(const int *view_size_in_window){

    // update the virtual view according to window size
    UpdateVirtualView(view_size_in_window);

    if(is_ar)// update the background image according to view size
        UpdateBackgroundImage(view_size_in_window);

    // update the pose if needed
    if(ar_camera!= nullptr)
        if(ar_camera->GetNewWorldToCamTr(world_to_cam_tr))
            SetWorldToCamTf(world_to_cam_tr);

}



//------------------------------------------------------------------------------
void RenderingCamera::ConfigureBackgroundImage(cv::Mat img) {

    assert( img.data != nullptr );

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


//------------------------------------------------------------------------------
void RenderingCamera::UpdateVirtualView(const int *window_size) {

    double  window_width = window_size[0];
    double window_height = window_size[1];

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


//------------------------------------------------------------------------------
void RenderingCamera::UpdateBackgroundImage(const int *window_size) {

    cv::Mat img;
    if(ar_camera->IsImageNew())
        img = ar_camera->GetImage();
    if(is_initialized && !img.empty()) {
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

        SetCameraToFaceImage(window_size, imageSize, spacing, origin);
    }

}


//------------------------------------------------------------------------------
void RenderingCamera::SetCameraToFaceImage(const int *window_size,
                                           const int *imageSize, const double *spacing,
                                           const double *orig) {

    KDL::Vector origin(orig[0], orig[1], orig[2]);

    double distance_along_x = (spacing[0] * (imageSize[0] - 1)) / 2.0;
    KDL::Vector vector_along_x(distance_along_x, 0, 0);

    double distance_along_y = (spacing[1] * (imageSize[1] - 1)) / 2.0;
    KDL::Vector vector_along_y(0, distance_along_y, 0);

    auto focal_point = origin + vector_along_x + vector_along_y;

    KDL::Vector vector_along_z(0, 0, -1000);
    auto position = focal_point + vector_along_z;

    double viewup_scale_factor = -1.0e9;
    auto view_up = viewup_scale_factor*vector_along_y;

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

    camera_real->SetPosition(position[0], position[1], position[2]);
    camera_real->SetFocalPoint(focal_point[0],focal_point[1],focal_point[2]);
    camera_real->SetViewUp(view_up[0], view_up[1], view_up[2]);
    camera_real->SetParallelProjection(true);
    camera_real->SetParallelScale(scale);

    double clipping_range[2] = {1, 10000};
    camera_real->SetClippingRange(clipping_range);
}



//------------------------------------------------------------------------------
void RenderingCamera::SetWorldToCamTf(const KDL::Frame & in) {

    world_to_cam_tr = in;

    // update the manipulators who are following the camera's pose
    UpdateCamPoseFollowers(world_to_cam_tr);

    KDL::Frame cam_to_world_pose = world_to_cam_tr.Inverse();

    KDL::Vector origin =        cam_to_world_pose.p;
    KDL::Vector focal_point =   cam_to_world_pose * KDL::Vector(0, 0, 1);
    KDL::Vector view_up =       cam_to_world_pose.M * KDL::Vector(0, -1, 0);

    camera_virtual->SetPosition(origin[0], origin[1], origin[2]);
    camera_virtual->SetFocalPoint(focal_point[0], focal_point[1], focal_point[2]);
    camera_virtual->SetViewUp(view_up[0], view_up[1], view_up[2]);
    camera_virtual->SetClippingRange(0.05, 10);
    camera_virtual->Modified();
}


//------------------------------------------------------------------------------
void RenderingCamera::UpdateCamPoseFollowers(const KDL::Frame &pose) {
    for (auto &im : interested_manipulators) {
        im->SetWorldToCamTr(pose);
    }
}



