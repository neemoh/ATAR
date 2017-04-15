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
        : background_renderer_(NULL)
        , scene_renderer(NULL)
        , camera_to_world_transform(NULL)
{

    image_importer_ = vtkSmartPointer<vtkImageImport>::New();
    image_actor_ = vtkSmartPointer<vtkImageActor>::New();
    render_window = vtkSmartPointer<vtkRenderWindow>::New();
    camera_image   = vtkSmartPointer<vtkImageData>::New();

    background_renderer_ = vtkSmartPointer<vtkRenderer>::New();
    background_renderer_->InteractiveOff();
    background_renderer_->SetBackground(0, 0, 0);
    background_renderer_->SetLayer(0);

    scene_renderer = vtkSmartPointer<vtkRenderer>::New();
    scene_renderer->InteractiveOff();
    scene_renderer->SetLayer(1);

    scene_camera_ = vtkSmartPointer<CalibratedCamera>::New();
    scene_renderer->SetActiveCamera(scene_camera_);

    background_camera_ = vtkSmartPointer<CalibratedCamera>::New();
    background_renderer_->SetActiveCamera(background_camera_);


    camera_to_world_transform = vtkSmartPointer<vtkMatrix4x4>::New();
    camera_to_world_transform->Identity();

    render_window->SetNumberOfLayers(2);
    render_window->AddRenderer(background_renderer_);
    render_window->AddRenderer(scene_renderer);

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
    render_window->RemoveRenderer(background_renderer_);
    render_window->RemoveRenderer(scene_renderer);
}

//----------------------------------------------------------------------------
void Rendering::SetWorldToCameraTransform(const cv::Vec3d &cam_rvec, const cv::Vec3d &cam_tvec) {

    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(cam_rvec, rotationMatrix);

    vtkSmartPointer<vtkMatrix4x4> world_to_camera_transform =vtkSmartPointer<vtkMatrix4x4>::New();
    world_to_camera_transform->Identity();

    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            world_to_camera_transform->SetElement(i, j, rotationMatrix.at<double>(i, j));
        }
        world_to_camera_transform->SetElement(i, 3, cam_tvec[i]);
    }

    camera_to_world_transform->DeepCopy(world_to_camera_transform);

    camera_to_world_transform->Invert();

    scene_camera_->SetExtrinsicParameters(camera_to_world_transform);
}


//-----------------------------------------------------------------------------
void Rendering::SetEnableImage(bool isEnabled)
{
    if (isEnabled)
    {
        if (background_renderer_->GetActors()->GetNumberOfItems() == 0)
        {
            background_renderer_->AddActor(image_actor_);
        }
    }
    else
    {
        if (background_renderer_->GetActors()->GetNumberOfItems() > 0)
        {
            background_renderer_->RemoveActor(image_actor_);
        }
    }
}


//----------------------------------------------------------------------------
void Rendering::SetCameraIntrinsics(const cv::Matx33d& intrinsics)
{
//    m_Intrinsics = intrinsics;
    background_camera_->SetIntrinsicParameters(intrinsics(0,0), intrinsics(1,1), intrinsics(0,2), intrinsics(1,2));
//    background_camera_->SetUseCalibratedCamera(true);
    scene_camera_->SetIntrinsicParameters(intrinsics(0,0), intrinsics(1,1), intrinsics(0,2), intrinsics(1,2));
//    scene_camera_->SetUseCalibratedCamera(true);
}


//----------------------------------------------------------------------------------
void Rendering::SetImageCameraToFaceImage()
{


    int *windowSize = render_window->GetSize();

    int    imageSize[3];
    image_importer_->GetOutput()->GetDimensions(imageSize);

    double spacing[3];
    image_importer_->GetOutput()->GetSpacing(spacing);

    double origin[3];
    image_importer_->GetOutput()->GetOrigin(origin);

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

    background_camera_->SetPosition(position);
    background_camera_->SetFocalPoint(focalPoint);
    background_camera_->SetViewUp(viewUp);
    background_camera_->SetParallelProjection(true);
    background_camera_->SetParallelScale(scale);
    background_camera_->SetClippingRange(clippingRange);
}

//----------------------------------------------------------------------------
void Rendering::UpdateBackgroundImage(cv::Mat & img) {

    cv::Mat _src;
//    cv::flip(src, _src, 0);
    cv::cvtColor(img, _src, cv::COLOR_BGR2RGB);
    image_importer_->SetImportVoidPointer( _src.data );
    image_importer_->Modified();
    image_importer_->Update();

}

//----------------------------------------------------------------------------
void Rendering::UpdateViewAngleForActualWindowSize() {

    SetImageCameraToFaceImage();
    int * window_size = render_window->GetActualSize();

    scene_camera_->UpdateViewAngle(window_size[0], window_size[1]);
}

//----------------------------------------------------------------------------
void Rendering::SetupBackgroundImage(cv::Mat &img) {
    assert( img.data != NULL );

    scene_camera_->SetCameraImageSize(img.size().width, img.size().height);
    background_camera_->SetCameraImageSize(img.size().width, img.size().height);

    if ( camera_image )
    {
        image_importer_->SetOutput( camera_image );
    }
    image_importer_->SetDataSpacing( 1, 1, 1 );
    image_importer_->SetDataOrigin( 0, 0, 0 );
    image_importer_->SetWholeExtent(   0, img.size().width-1, 0,
                                     img.size().height-1, 0, 0 );
    image_importer_->SetDataExtentToWholeExtent();
    image_importer_->SetDataScalarTypeToUnsignedChar();
    image_importer_->SetNumberOfScalarComponents( img.channels() );
    image_importer_->SetImportVoidPointer( img.data );
    image_importer_->Update();

    image_actor_->SetInputData(camera_image);

    render_window->SetSize(img.size().width, img.size().height);

}

//----------------------------------------------------------------------------
void Rendering::AddActorToScene(vtkSmartPointer<vtkProp> actor) {

    scene_renderer->AddActor(actor);

}

//----------------------------------------------------------------------------
void Rendering::Render() {

//    scene_renderer->Modified();
//    background_renderer_->Modified();
//    render_window->Modified();

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
