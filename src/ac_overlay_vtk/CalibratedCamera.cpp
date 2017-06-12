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

    this->SetPosition(origin[0], origin[1], origin[2]);
    this->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
    this->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
    //  this->SetClippingRange(1, 5000);

    //    std::cout << "Camera position = " << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;
    this->Modified();
}



void CalibratedCamera::UpdateView(const int &window_width,
                                  const int &window_height) {

    // calculate the view angle and set it.
    double window_factor = window_height/image_height_;

    double view_angle = 2*atan((window_height/2) / (window_factor*fy_))
                        * 180/M_PI;
    this->SetViewAngle(view_angle);

    // convert the principal point to window center
    double wc_x = -2 * (window_factor*cx_ - double(window_width)/2)
                  / window_width;
    double wc_y =  2 * (window_factor*cy_ - double(window_height)/2)
                  / window_height;
    this->SetWindowCenter(wc_x, wc_y);

    this->Modified();
}


