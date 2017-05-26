//
// Created by nima on 4/12/17.
//

#include "CalibratedCamera.h"
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>



#ifndef VTK_IMPLEMENT_MESA_CXX
vtkStandardNewMacro(CalibratedCamera);
#endif

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
}


//----------------------------------------------------------------------------
void CalibratedCamera::SetCameraImageSize(const int &width, const int &height)
{
    image_width_ = width;
    image_height_ = height;
    this->Modified();
}


////----------------------------------------------------------------------------
//void CalibratedCamera::SetActualWindowSize(const int& width, const int& height)
//{
//    m_WindowWidthInPixels = width;
//    m_WindowHeightInPixels = height;
//    this->Modified();
//}


//----------------------------------------------------------------------------
void CalibratedCamera::SetIntrinsicParameters(const double& fx, const double& fy,
                                              const double& cx, const double& cy)
{
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    this->Modified();
}


//----------------------------------------------------------------------------
void CalibratedCamera::SetExtrinsicParameters(vtkSmartPointer<vtkMatrix4x4> matrix)
{

    double origin[4]     = {0, 0,    0,    1};
    double focalPoint[4] = {0, 0,   1, 1};
    double viewUp[4]     = {0, -1, 0,    1};

    matrix->MultiplyPoint(origin, origin);
    matrix->MultiplyPoint(focalPoint, focalPoint);
    matrix->MultiplyPoint(viewUp, viewUp);
    viewUp[0] = viewUp[0] - origin[0];
    viewUp[1] = viewUp[1] - origin[1];
    viewUp[2] = viewUp[2] - origin[2];

    this->SetPosition(origin[0], origin[1], origin[2]);
    this->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
    this->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
//  this->SetClippingRange(1, 5000);

//    std::cout << "Camera position = " << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;
    this->Modified();
}



void CalibratedCamera::UpdateViewAngle(const int& window_width, const int& window_height) {

    double focalLengthY = fy_;
    if( window_height != image_height_ )
    {
        double factor = static_cast<double>(window_height)/static_cast<double>(image_height_);
        focalLengthY = fy_ * factor;
    }

    double view_angle = 2 * atan( ( window_height / 2 ) / focalLengthY ) * 180 / M_PI;
    this->SetViewAngle(view_angle);
    this->Modified();
    //Todo take into account window width

}


