//
// Created by nima on 4/12/17.
//

#include "CalibratedCamera.h"
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>

//
//
//#ifndef VTK_IMPLEMENT_MESA_CXX
//vtkStandardNewMacro(CalibratedCamera);
//#endif

//----------------------------------------------------------------------------
CalibratedCamera::CalibratedCamera()
        : intrinsic_matrix(NULL)
        , image_width_(512)
        , image_height_(512)
        , fx_(1)
        , fy_(1)
        , cx_(0)
        , cy_(0)
{
    intrinsic_matrix = vtkMatrix4x4::New();
    intrinsic_matrix->Identity();
    camera = vtkSmartPointer<vtkCamera>::New();

}


//----------------------------------------------------------------------------
void CalibratedCamera::SetCameraImageSize(const int &width, const int &height)
{
    image_width_ = width;
    image_height_ = height;
    camera->Modified();
}


//----------------------------------------------------------------------------
void CalibratedCamera::SetIntrinsicParameters(const double& fx, const double& fy,
                                              const double& cx, const double& cy)
{
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;

    std::cout << "fx_: " <<  fx_ << " fy_: " <<  fy_
              << " cx_: " <<  cx_ << " cy_: " <<  cy_ << std::endl;
}


//----------------------------------------------------------------------------
void CalibratedCamera::SetExtrinsicParameters(vtkSmartPointer<vtkMatrix4x4> matrix)
{

    double origin[4]     = {0,  0,   0, 1};
    double focalPoint[4] = {0,  0,   1, 1};
    double viewUp[4]     = {0, -1,   0, 1};

    matrix->MultiplyPoint(origin, origin);
    matrix->MultiplyPoint(focalPoint, focalPoint);
    matrix->MultiplyPoint(viewUp, viewUp);
    viewUp[0] = viewUp[0] - origin[0];
    viewUp[1] = viewUp[1] - origin[1];
    viewUp[2] = viewUp[2] - origin[2];

    camera->SetPosition(origin[0], origin[1], origin[2]);
    camera->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
    camera->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
    camera->SetClippingRange(0.05, 10);

    camera->Modified();
}



void CalibratedCamera::UpdateView(const double &window_width,
                                  const double &window_height) {

    //When window aspect ratio is different than that of the image we need to
    // take that into account
    double image_aspect_ratio =  image_width_ / image_height_;
    double window_aspect_ratio =  window_width / window_height;

    double window_resize_factor;
    if(window_aspect_ratio > image_aspect_ratio)
        window_resize_factor = window_height / image_height_;
    else
        window_resize_factor = window_width / image_width_;

    // calculate the view angle and set it.
    double view_angle = 2*atan((window_height/2)
                        / (window_resize_factor*fy_) ) * 180/M_PI;
    camera->SetViewAngle(view_angle);

    // convert the principal point to window center
    double wc_x = -2 * window_resize_factor*(cx_ - image_width_/2)
                  / window_width;
    double wc_y =  2 * window_resize_factor*(cy_ - image_height_/2)
                   / window_height;
    camera->SetWindowCenter(wc_x, wc_y);

    camera->Modified();
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

    camera->SetPosition(position);
    camera->SetFocalPoint(focalPoint);
    camera->SetViewUp(viewUp);
    camera->SetParallelProjection(true);
    camera->SetParallelScale(scale);
    camera->SetClippingRange(clippingRange);

}


