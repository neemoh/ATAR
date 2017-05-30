//
// Created by nima on 4/12/17.
//

#ifndef bardCalibratedCamera_h
#define bardCalibratedCamera_h

#include <vtkOpenGLCamera.h>
#include <vtkRenderer.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkSetGet.h>

/**
 * \class CalibratedCamera
 * \brief This is a Subclass of vtkCamera augmented with intrinsic and extrinsic
 * camera information.
 * Changing the size of the rendering window is taken into consideration to get
 * a correct rendering view (still need to add).
 */

class CalibratedCamera : public vtkOpenGLCamera
{
public:
    static CalibratedCamera *New();
    vtkTypeMacro(CalibratedCamera, vtkOpenGLCamera);


    /**
     * \brief Set the size of the camera image (real world camera)in pixels.
     * This is the size of the image at the time of intrinsic calibration.
     * \param width in pixels.
     * \param height in pixels.
     */
    void SetCameraImageSize(const int &width, const int &height);

    //    /**
    //     * \brief Set the window size currently used
    //     * \param width in pixels.
    //     * \param height in pixels.
    //     */
    //    void SetActualWindowSize(const int& width, const int& height);

    /**
     * \brief Set the intrinsic parameters from camera calibration.
     */
    void SetIntrinsicParameters(const double& fx, const double& fy,
                                const double& cx, const double& cy);

    /**
    * \brief Update the view angle of the virtual Camera according to window size
     * Note that the windows is the opengl window here,
    */
    void UpdateViewAngle(const int& width, const int& height);

    /**
     * \brief Sets the pose of the camera with respect to world (task frame)
     */
    void SetExtrinsicParameters(vtkSmartPointer<vtkMatrix4x4> matrix);

protected:

    CalibratedCamera();
    ~CalibratedCamera() {}

private:

    CalibratedCamera(const CalibratedCamera&);  // Purposefully not implemented.
    void operator=(const CalibratedCamera&);  // Purposefully not implemented.

    vtkSmartPointer<vtkMatrix4x4> intrinsic_matrix;
    //    int m_WindowWidthInPixels;
    //    int m_WindowHeightInPixels;
    int image_width_;
    int image_height_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
};


#endif
