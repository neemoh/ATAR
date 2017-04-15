//
// Created by nima on 4/12/17.
//
#include "Rendering.h"

#include <vtkCamera.h>
#include <vtkImageData.h>
#include <opencv-3.2.0-dev/opencv2/calib3d.hpp>
#include <vtkImageActor.h>
#include <vtkObjectFactory.h>
#include <opencv-3.2.0-dev/opencv2/imgproc.hpp>



//----------------------------------------------------------------------------
Rendering::Rendering()
//        : background_renderer_(NULL)
//        , scene_renderer(NULL)
//        , background_renderer_(NULL)
//        , scene_renderer(NULL)
//        , camera_to_world_transform_(NULL)
{

    render_window = vtkSmartPointer<vtkRenderWindow>::New();
    render_window->SetNumberOfLayers(2);

    double view_port[2][4] = {{0.0, 0.0, 0.5, 1.0}, {0.5, 0.0, 1.0, 1.0}};


    for (int i = 0; i < 2; ++i) {

        image_importer_[i] = vtkSmartPointer<vtkImageImport>::New();
        image_actor_[i] = vtkSmartPointer<vtkImageActor>::New();
        camera_image[i]   = vtkSmartPointer<vtkImageData>::New();

        background_renderer_[i] = vtkSmartPointer<vtkRenderer>::New();
        background_renderer_[i]->InteractiveOff();
        background_renderer_[i]->SetBackground(0, 0, 0);
        background_renderer_[i]->SetLayer(0);

        scene_renderer[i] = vtkSmartPointer<vtkRenderer>::New();
        scene_renderer[i]->InteractiveOff();
        scene_renderer[i]->SetLayer(1);
        scene_renderer[i]->SetViewport(view_port[i]);

        scene_camera_[i] = vtkSmartPointer<CalibratedCamera>::New();
        scene_renderer[i]->SetActiveCamera(scene_camera_[i]);

        background_camera_[i] = vtkSmartPointer<CalibratedCamera>::New();
        background_renderer_[i]->SetActiveCamera(background_camera_[i]);
        background_renderer_[i]->SetViewport(view_port[i]);

        camera_to_world_transform_[i] = vtkSmartPointer<vtkMatrix4x4>::New();
        camera_to_world_transform_[i]->Identity();


        render_window->AddRenderer(background_renderer_[i]);
        render_window->AddRenderer(scene_renderer[i]);

    }
    render_window->SetOffScreenRendering(1);

//    renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    renderWindowInteractor->SetRenderWindow(render_window);

    window_to_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    window_to_image_filter->SetInput(render_window);
    window_to_image_filter->SetInputBufferTypeToRGB(); //record the alpha (transparency) channel for future use
    window_to_image_filter->ReadFrontBufferOff(); // read from the back buffer


}

//-----------------------------------------------------------------------------
Rendering::~Rendering()
{

        for (int j = 0; j < 2; ++j) {
            render_window->RemoveRenderer(background_renderer_[j]);
            render_window->RemoveRenderer(scene_renderer[j]);
        }


}

//----------------------------------------------------------------------------
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


//-----------------------------------------------------------------------------
void Rendering::SetEnableImage(bool isEnabled)
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


//----------------------------------------------------------------------------
void Rendering::SetCameraIntrinsics(const cv::Matx33d intrinsics[])
{
    for (int i = 0; i < 2; ++i) {
//    m_Intrinsics = intrinsics;
        background_camera_[i]->SetIntrinsicParameters(intrinsics[i](0, 0),
                                                      intrinsics[i](1, 1),
                                                      intrinsics[i](0, 2),
                                                      intrinsics[i](1, 2));
//    background_camera_->SetUseCalibratedCamera(true);
        scene_camera_[i]->SetIntrinsicParameters(intrinsics[i](0, 0),
                                                 intrinsics[i](1, 1),
                                                 intrinsics[i](0, 2),
                                                 intrinsics[i](1, 2));
//    scene_camera_->SetUseCalibratedCamera(true);
    }
}


//----------------------------------------------------------------------------------
void Rendering::SetImageCameraToFaceImage(const int id) {

        int *windowSize = render_window->GetSize();

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

        double widthRatio = imageWidth / (windowSize[0]/2);
        double heightRatio = imageHeight / windowSize[1];

        double scale = 1;
        if (widthRatio > heightRatio) {
            scale = 0.5 * imageWidth *
                    ((double) windowSize[1] / (double)(windowSize[0]/2));
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

//----------------------------------------------------------------------------
void Rendering::UpdateBackgroundImage(cv::Mat  img[]) {

    cv::Mat _src;
    for (int i = 0; i < 2; ++i) {
//    cv::flip(src, _src, 0);
        cv::cvtColor(img[i], _src, cv::COLOR_BGR2RGB);
        image_importer_[i]->SetImportVoidPointer( _src.data );
        image_importer_[i]->Modified();
        image_importer_[i]->Update();
    }
}

//----------------------------------------------------------------------------
void Rendering::UpdateViewAngleForActualWindowSize() {
    for (int i = 0; i < 2; ++i) {
        SetImageCameraToFaceImage(i);
        int *window_size = render_window->GetActualSize();
        scene_camera_[i]->UpdateViewAngle(window_size[0]/2, window_size[1]);
    }
}

//----------------------------------------------------------------------------
void Rendering::SetupBackgroundImage(cv::Mat img[]) {

    for (int i = 0; i < 2; ++i) {
        assert( img[i].data != NULL );

        scene_camera_[i]->SetCameraImageSize(img[i].size().width, img[i].size().height);
        background_camera_[i]->SetCameraImageSize(img[i].size().width,
                                               img[i].size().height);

        if (camera_image[i]) {
            image_importer_[i]->SetOutput(camera_image[i]);
        }
        image_importer_[i]->SetDataSpacing(1, 1, 1);
        image_importer_[i]->SetDataOrigin(0, 0, 0);
        image_importer_[i]->SetWholeExtent(0, img[i].size().width - 1, 0,
                                        img[i].size().height - 1, 0, 0);
        image_importer_[i]->SetDataExtentToWholeExtent();
        image_importer_[i]->SetDataScalarTypeToUnsignedChar();
        image_importer_[i]->SetNumberOfScalarComponents(img[i].channels());
        image_importer_[i]->SetImportVoidPointer(img[i].data);
        image_importer_[i]->Update();

        image_actor_[i]->SetInputData(camera_image[i]);

        render_window->SetSize(2 * img[i].size().width, img[i].size().height);
    }

}

//----------------------------------------------------------------------------
void Rendering::AddActorToScene(vtkSmartPointer<vtkProp> actor) {

    scene_renderer[0]->AddActor(actor);
    scene_renderer[1]->AddActor(actor);

}

//----------------------------------------------------------------------------
void Rendering::Render() {

//    scene_renderer->Modified();
//    background_renderer_->Modified();
//    render_window->Modified();

//    render_window[0]->Render();
    render_window->Render();

}

//----------------------------------------------------------------------------
void Rendering::GetRenderedImage(cv::Mat &img) {

    window_to_image_filter->Modified();
    vtkImageData* image = window_to_image_filter->GetOutput();
    window_to_image_filter->Update();

    // copy to cv Mat
    int dims[3];
    image->GetDimensions(dims);

    if(dims[0]>0) {
        cv::Mat openCVImage(dims[1], dims[0], CV_8UC3,
                            image->GetScalarPointer()); // Unsigned int, 4 channels
        // convert to bgr
        cv::cvtColor(openCVImage, img, cv::COLOR_RGB2BGR);

        // Flip because of different origins between vtk and OpenCV
        cv::flip(img, img, 0);
    }
}
