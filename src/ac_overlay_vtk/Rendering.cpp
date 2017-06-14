//
// Created by nima on 4/12/17.
//
#include "Rendering.h"

#include <vtkCamera.h>
#include <vtkImageData.h>
#include <opencv2/calib3d.hpp>
#include <vtkImageActor.h>
#include <vtkObjectFactory.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vtk-6.2/vtkFrameBufferObject.h>
#include <vtk-6.2/vtkOpenGLRenderer.h>
#include <vtk-6.2/vtkFrameBufferObject2.h>

Rendering::Rendering(uint num_windows)
        : num_render_windows(num_windows)
//        : background_renderer_(NULL)
//        , scene_renderer_(NULL)
//        , background_renderer_(NULL)
//        , scene_renderer_(NULL)
//        , camera_to_world_transform_(NULL)
{
    // make sure the number of windows are alright
    if(num_render_windows <1) num_render_windows =1;
    else if(num_render_windows >2) num_render_windows = 2;

    double view_port[2][4] = {{0.0, 0.0, 0.5, 1.0}, {0.5, 0.0, 1.0, 1.0}};

    render_window_[0] = vtkSmartPointer<vtkRenderWindow>::New();
    if(num_render_windows==2)
        render_window_[1] = vtkSmartPointer<vtkRenderWindow>::New();

    for (int i = 0; i < 2; ++i) {

        image_importer_[i] = vtkSmartPointer<vtkImageImport>::New();
        image_actor_[i] = vtkSmartPointer<vtkImageActor>::New();
        camera_image_[i]   = vtkSmartPointer<vtkImageData>::New();

        background_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
        background_renderer_[i]->InteractiveOff();
        background_renderer_[i]->SetLayer(0);

        background_camera_[i] = vtkSmartPointer<CalibratedCamera>::New();
        background_renderer_[i]->SetActiveCamera(background_camera_[i]);

        scene_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
        scene_renderer_[i]->InteractiveOff();
        scene_renderer_[i]->SetLayer(1);

        if(num_render_windows==1){
            background_renderer_[i]->SetViewport(view_port[i]);
            scene_renderer_[i]->SetViewport(view_port[i]);
        }

        scene_camera_[i] = vtkSmartPointer<CalibratedCamera>::New();
        scene_renderer_[i]->SetActiveCamera(scene_camera_[i]);

        camera_to_world_transform_[i] = vtkSmartPointer<vtkMatrix4x4>::New();
        camera_to_world_transform_[i]->Identity();


        int j=0;
        if(num_render_windows==2)
            j=i;
        render_window_[j]->SetNumberOfLayers(2);
        render_window_[j]->AddRenderer(background_renderer_[i]);
        render_window_[j]->AddRenderer(scene_renderer_[i]);

        window_to_image_filter_[j] =
                vtkSmartPointer<vtkWindowToImageFilter>::New();
        window_to_image_filter_[j]->SetInput(render_window_[j]);
        //    window_to_image_filter_->SetInputBufferTypeToRGBA(); //record  he
        // alpha (transparency) channel for future use
        window_to_image_filter_[j]->ReadFrontBufferOff(); // read from the
        // back buffer
        // important for getting high update rate (If needed, images can be shown
        // with opencv)
        //    render_window_->SetOffScreenRendering(1);

    }



}


//------------------------------------------------------------------------------
Rendering::~Rendering()
{

    for (int j = 0; j < num_render_windows; ++j) {
        render_window_[j]->RemoveRenderer(background_renderer_[j]);
        render_window_[j]->RemoveRenderer(scene_renderer_[j]);
    }


}


//------------------------------------------------------------------------------
void Rendering::SetWorldToCameraTransform(const cv::Vec3d cam_rvec[], const cv::Vec3d cam_tvec[]) {

    for (int k = 0; k < 2; ++k) {

        cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
        cv::Rodrigues(cam_rvec[k], rotationMatrix);

        vtkSmartPointer<vtkMatrix4x4> world_to_camera_transform =vtkSmartPointer<vtkMatrix4x4>::New();
        world_to_camera_transform->Identity();

        // Convert to VTK matrix.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                world_to_camera_transform->SetElement(i, j, rotationMatrix.at<double>(i, j));
            }
            world_to_camera_transform->SetElement(i, 3, cam_tvec[k][i]);
        }

        camera_to_world_transform_[k]->DeepCopy(world_to_camera_transform);

        camera_to_world_transform_[k]->Invert();

        scene_camera_[k]->SetExtrinsicParameters(camera_to_world_transform_[k]);
    }

}


//------------------------------------------------------------------------------
void Rendering::SetEnableBackgroundImage(bool isEnabled)
{
    for (int i = 0; i < 2; ++i) {
        if (isEnabled)
        {
            if(background_renderer_[i]->GetActors()->GetNumberOfItems() == 0)
                background_renderer_[i]->AddActor(image_actor_[i]);
        }
        else
        {
            if(background_renderer_[i]->GetActors()->GetNumberOfItems() > 0)
                background_renderer_[i]->RemoveActor(image_actor_[i]);
        }
    }
}


//------------------------------------------------------------------------------
void Rendering::SetCameraIntrinsics(const cv::Mat intrinsics[])
{
    for (int i = 0; i < 2; ++i) {
//    m_Intrinsics = intrinsics;
        background_camera_[i]->SetIntrinsicParameters(intrinsics[i].at<double>(0, 0),
                                                      intrinsics[i].at<double>(1, 1),
                                                      intrinsics[i].at<double>(0, 2),
                                                      intrinsics[i].at<double>(1, 2));
//    background_camera_->SetUseCalibratedCamera(true);
        scene_camera_[i]->SetIntrinsicParameters(intrinsics[i].at<double>(0, 0),
                                                 intrinsics[i].at<double>(1, 1),
                                                 intrinsics[i].at<double>(0, 2),
                                                 intrinsics[i].at<double>(1, 2));
//    scene_camera_->SetUseCalibratedCamera(true);
    }
}


//------------------------------------------------------------------------------
void
Rendering::SetImageCameraToFaceImage(const int id, const int *window_size) {

    int imageSize[3];
    image_importer_[id]->GetOutput()->GetDimensions(imageSize);

    double spacing[3];
    image_importer_[id]->GetOutput()->GetSpacing(spacing);

    double origin[3];
    image_importer_[id]->GetOutput()->GetOrigin(origin);

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

    double widthRatio = imageWidth / (window_size[0]/(3-
                                                     num_render_windows));
    double heightRatio = imageHeight / window_size[1];

    double scale;
    if (widthRatio > heightRatio) {
        scale = 0.5 * imageWidth *
                ((double) window_size[1] / (double)(window_size[0]/(3-
                                                                  num_render_windows)));
    } else {
        scale = 0.5 * imageHeight;
    }

    background_camera_[id]->SetPosition(position);
    background_camera_[id]->SetFocalPoint(focalPoint);
    background_camera_[id]->SetViewUp(viewUp);
    background_camera_[id]->SetParallelProjection(true);
    background_camera_[id]->SetParallelScale(scale);
    background_camera_[id]->SetClippingRange(clippingRange);

}


//------------------------------------------------------------------------------
void Rendering::UpdateBackgroundImage(cv::Mat  img[]) {

    for (int i = 0; i < 2; ++i) {
//    cv::flip(src, _src, 0);
        cv::cvtColor(img[i], img[i], cv::COLOR_BGR2RGB);
        image_importer_[i]->SetImportVoidPointer( img[i].data );
        image_importer_[i]->Modified();
        image_importer_[i]->Update();
    }
}


//------------------------------------------------------------------------------
void Rendering::UpdateCameraViewForActualWindowSize() {

    for (int i = 0; i < 2; ++i) {

        int k = 0;
        if(num_render_windows==2)
            k=i;
        int *window_size = render_window_[k]->GetActualSize();

        // update each windows view
        scene_camera_[i]->UpdateView(window_size[0] / (3 - num_render_windows),
                                     window_size[1]);

        // update the background image for each camera
        SetImageCameraToFaceImage(i, window_size);
    }
}


//------------------------------------------------------------------------------
void Rendering::ConfigureBackgroundImage(cv::Mat *img) {

    int image_width = img[0].size().width;
    int image_height = img[0].size().height;

    // if one window the width is double
    for (int j = 0; j < num_render_windows; ++j) {
        render_window_[j]->SetSize((3-num_render_windows) * image_width,
                                   image_height);
    }


    for (int i = 0; i < 2; ++i) {
        assert( img[i].data != NULL );

        scene_camera_[i]->SetCameraImageSize(image_width, image_height);
        background_camera_[i]->SetCameraImageSize(image_width, image_height);

        if (camera_image_[i]) {
            image_importer_[i]->SetOutput(camera_image_[i]);
        }
        image_importer_[i]->SetDataSpacing(1, 1, 1);
        image_importer_[i]->SetDataOrigin(0, 0, 0);
        image_importer_[i]->SetWholeExtent(0, image_width - 1, 0,
                                           image_height - 1, 0, 0);
        image_importer_[i]->SetDataExtentToWholeExtent();
        image_importer_[i]->SetDataScalarTypeToUnsignedChar();
        image_importer_[i]->SetNumberOfScalarComponents(img[i].channels());
        image_importer_[i]->SetImportVoidPointer(img[i].data);
        image_importer_[i]->Update();

        image_actor_[i]->SetInputData(camera_image_[i]);
    }


}


//------------------------------------------------------------------------------
void Rendering::AddActorToScene(vtkSmartPointer<vtkProp> actor) {

    scene_renderer_[0]->AddActor(actor);
    scene_renderer_[1]->AddActor(actor);

}



void
Rendering::AddActorsToScene(std::vector<vtkSmartPointer<vtkProp> > actors) {

    for (int i = 0; i <actors.size() ; ++i) {
        scene_renderer_[0]->AddActor(actors[i]);
        scene_renderer_[1]->AddActor(actors[i]);
    }
//    scene_renderer_[0]->AddViewProp( actors[actors.size()-1] );
//    scene_renderer_[1]->AddViewProp( actors[actors.size()-1] );

}


//------------------------------------------------------------------------------
void Rendering::Render() {

//    scene_renderer_->Modified();
//    background_renderer_->Modified();
//    render_window_->Modified();
    for (int i = 0; i < num_render_windows; ++i) {
        render_window_[i]->Render();
    }

}


//------------------------------------------------------------------------------
void Rendering::GetRenderedImage(cv::Mat *images) {

    // TODO: REWRITE FOR 2-WINDOW CASE (writes on the same image for now)

    for (int i = 0; i < num_render_windows; ++i) {

        window_to_image_filter_[i]->Modified();
        vtkImageData *image = window_to_image_filter_[i]->GetOutput();
        window_to_image_filter_[i]->Update();

        // copy to cv Mat
        int dims[3];
        image->GetDimensions(dims);

//    std::cout << " dims[0] " << dims[0] << " dims[1] " << dims[1] << " "
//            "dims[2] " << dims[2] <<std::endl;
        if (dims[0] > 0) {
            cv::Mat openCVImage(dims[1], dims[0], CV_8UC3,
                                image->GetScalarPointer()); // Unsigned int, 4 channels
            // convert to bgr
            cv::cvtColor(openCVImage, images[i], cv::COLOR_RGB2BGR);

            // Flip because of different origins between vtk and OpenCV
            cv::flip(images[i], images[i], 0);
        }
    }
}

void Rendering::RemoveAllActorsFromScene() {

    scene_renderer_[0]->RemoveAllViewProps();
    scene_renderer_[1]->RemoveAllViewProps();
}


//------------------------------------------------------------------------------
void VTKConversions::AxisAngleToVTKMatrix(const cv::Vec3d cam_rvec,
                                          const cv::Vec3d cam_tvec,
                                          vtkSmartPointer<vtkMatrix4x4>  out) {


    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(cam_rvec, rotationMatrix);

    out->Identity();

    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out->SetElement(i, j, rotationMatrix.at<double>(i, j));
        }
        out->SetElement(i, 3, cam_tvec[i]);
    }


}


//------------------------------------------------------------------------------
void ::VTKConversions::KDLFrameToVTKMatrix(const KDL::Frame in,
                                           vtkSmartPointer<vtkMatrix4x4> out) {
    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out->SetElement(i, j, in.M(i,j));
        }
        out->SetElement(i, 3, in.p[i]);
    }

}


//------------------------------------------------------------------------------
void VTKConversions::VTKMatrixToKDLFrame(const vtkSmartPointer<vtkMatrix4x4> in,
                                         KDL::Frame & out) {
    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.M(i,j) = in->GetElement(i, j);
        }
        out.p[i] = in->GetElement(i, 3);
    }

}
