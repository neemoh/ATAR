//
// Created by charm on 4/12/17.
//
#include "Rendering.h"

#include <vtkCamera.h>
#include <vtkImageData.h>
#include <opencv-3.2.0-dev/opencv2/calib3d.hpp>
#include <vtkImageActor.h>
#include <vtkObjectFactory.h>
#include <opencv-3.2.0-dev/opencv2/imgproc.hpp>



Rendering::Rendering()
        : backgroundRenderer(NULL)
        , sceneRenderer(NULL)
        , WorldToCameraTransform(NULL)
        , CameraToWorldTransform(NULL)
{

    //  m_Timer = new QTimer();
    //  m_Timer->setInterval(40);
    //  connect(m_Timer, SIGNAL(timeout()), this, SLOT(OnTimerTriggered()));

    ImageImporter = vtkSmartPointer<vtkImageImport>::New();
    ImageActor = vtkSmartPointer<vtkImageActor>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    camera_image   = vtkSmartPointer<vtkImageData>::New();

    backgroundRenderer = vtkSmartPointer<vtkRenderer>::New();
    backgroundRenderer->InteractiveOff();
    backgroundRenderer->SetBackground(0, 0, 0);
    backgroundRenderer->SetLayer(0);

    sceneRenderer = vtkSmartPointer<vtkRenderer>::New();
//    sceneRenderer->InteractiveOff();
    sceneRenderer->SetLayer(1);

    SceneCamera = vtkSmartPointer<CalibratedCamera>::New();
//    SceneCamera->SetUseCalibratedCamera(false);
    sceneRenderer->SetActiveCamera(SceneCamera);

    BackgroundCamera = vtkSmartPointer<CalibratedCamera>::New();
//    BackgroundCamera->SetUseCalibratedCamera(false);
    backgroundRenderer->SetActiveCamera(BackgroundCamera);

    WorldToCameraTransform = vtkSmartPointer<vtkMatrix4x4>::New();
    WorldToCameraTransform->Identity();

    CameraToWorldTransform = vtkSmartPointer<vtkMatrix4x4>::New();
    CameraToWorldTransform->Identity();

    renderWindow->SetNumberOfLayers(2);
    renderWindow->AddRenderer(backgroundRenderer);
    renderWindow->AddRenderer(sceneRenderer);

    windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->SetInputBufferTypeToRGB(); //record the alpha (transparency) channel for future use
    windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
}

//-----------------------------------------------------------------------------
Rendering::~Rendering()
{
    renderWindow->RemoveRenderer(backgroundRenderer);
    renderWindow->RemoveRenderer(sceneRenderer);
}

void Rendering::SetWorldToCameraTransform(const cv::Vec3d &cam_rvec, const cv::Vec3d &cam_tvec) {

    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(cam_rvec, rotationMatrix);

    WorldToCameraTransform->Identity();

// Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            WorldToCameraTransform->SetElement(i, j, rotationMatrix.at<double>(i, j));
        }
        WorldToCameraTransform->SetElement(i, 3, cam_tvec[i]);
    }

    CameraToWorldTransform->DeepCopy(WorldToCameraTransform);

    CameraToWorldTransform->Invert();

    SceneCamera->SetExtrinsicParameters(CameraToWorldTransform);
}


//-----------------------------------------------------------------------------
void Rendering::SetEnableImage(bool isEnabled)
{
    if (isEnabled)
    {
        if (backgroundRenderer->GetActors()->GetNumberOfItems() == 0)
        {
            backgroundRenderer->AddActor(ImageActor);
        }
    }
    else
    {
        if (backgroundRenderer->GetActors()->GetNumberOfItems() > 0)
        {
            backgroundRenderer->RemoveActor(ImageActor);
        }
    }
//    this->UpdateLayers();
}

//bool Rendering::fromIpl2Vtk( cv::Mat _src, vtkImageData* _dest )
//{
//    assert( _src.data != NULL );
//
//
//    if ( _dest )
//    {
//        ImageImporter->SetOutput( camera_image );
//    }
//    ImageImporter->SetDataSpacing( 1, 1, 1 );
//    ImageImporter->SetDataOrigin( 0, 0, 0 );
//    ImageImporter->SetWholeExtent(   0, _src.size().width-1, 0,
//                                     _src.size().height-1, 0, 0 );
//    ImageImporter->SetDataExtentToWholeExtent();
//    ImageImporter->SetDataScalarTypeToUnsignedChar();
//    ImageImporter->SetNumberOfScalarComponents( _src.channels() );
//    ImageImporter->SetImportVoidPointer( _src.data );
//    ImageImporter->Update();
//    return true;
//}


void Rendering::SetCameraIntrinsics(const cv::Matx33d& intrinsics)
{
//    m_Intrinsics = intrinsics;
    BackgroundCamera->SetIntrinsicParameters(intrinsics(0,0), intrinsics(1,1), intrinsics(0,2), intrinsics(1,2));
    BackgroundCamera->SetUseCalibratedCamera(true);
    SceneCamera->SetIntrinsicParameters(intrinsics(0,0), intrinsics(1,1), intrinsics(0,2), intrinsics(1,2));
    SceneCamera->SetUseCalibratedCamera(true);
}


//----------------------------------------------------------------------------------

void Rendering::SetImageCameraToFaceImage()
{


    int *windowSize = renderWindow->GetSize();

    int    imageSize[3];
    ImageImporter->GetOutput()->GetDimensions(imageSize);

    double spacing[3];
    ImageImporter->GetOutput()->GetSpacing(spacing);

    double origin[3];
    ImageImporter->GetOutput()->GetOrigin(origin);

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

    BackgroundCamera->SetPosition(position);
    BackgroundCamera->SetFocalPoint(focalPoint);
    BackgroundCamera->SetViewUp(viewUp);
    BackgroundCamera->SetParallelProjection(true);
    BackgroundCamera->SetParallelScale(scale);
    BackgroundCamera->SetClippingRange(clippingRange);
}

void Rendering::UpdateBackgroundImage(cv::Mat & img) {

    cv::Mat _src;
//    cv::flip(src, _src, 0);
    cv::cvtColor(img, _src, cv::COLOR_BGR2RGB);
    ImageImporter->SetImportVoidPointer( _src.data );
    ImageImporter->Update();
//    ImageActor->SetInputData(camera_image);

    renderWindow->Render();
}

void Rendering::UpdateWindowSizeRelatedViews() {

    SetImageCameraToFaceImage();
    SceneCamera->UpdateViewAngle();
}

void Rendering::SetupBackgroundImage(cv::Mat &img) {
    assert( img.data != NULL );

    SceneCamera->SetCalibratedImageSize(img.size().width, img.size().height);
    BackgroundCamera->SetCalibratedImageSize(img.size().width, img.size().height);

    if ( camera_image )
    {
        ImageImporter->SetOutput( camera_image );
    }
    ImageImporter->SetDataSpacing( 1, 1, 1 );
    ImageImporter->SetDataOrigin( 0, 0, 0 );
    ImageImporter->SetWholeExtent(   0, img.size().width-1, 0,
                                     img.size().height-1, 0, 0 );
    ImageImporter->SetDataExtentToWholeExtent();
    ImageImporter->SetDataScalarTypeToUnsignedChar();
    ImageImporter->SetNumberOfScalarComponents( img.channels() );
    ImageImporter->SetImportVoidPointer( img.data );
    ImageImporter->Update();

    ImageActor->SetInputData(camera_image);

    renderWindow->SetSize(img.size().width, img.size().height);

}

void Rendering::AddActorToScene(vtkSmartPointer<vtkProp> actor) {

    sceneRenderer->AddActor(actor);

}

void Rendering::Render() {

    renderWindow->Render();

}

void Rendering::GetRenderedImage(cv::Mat &img) {

    windowToImageFilter->Modified();
    vtkImageData* image = windowToImageFilter->GetOutput();
    windowToImageFilter->Update();

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
