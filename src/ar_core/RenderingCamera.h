//
// Created by nima on 4/12/17.
//

#ifndef RENDERINGCAMERA_h
#define RENDERINGCAMERA_h

#include "src/ar_core/Manipulator.h"
#include "src/ar_core/AugmentedCamera.h"

#include <vtkImageImport.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkOpenGLCamera.h>
#include <vtkRenderer.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkSetGet.h>
/**
 * \class RenderingCamera
 * \brief This is a Subclass of vtkCamera augmented with intrinsic and extrinsic
 * camera information.
 * Changing the size of the rendering window is taken into consideration to get
 * a correct rendering view (still need to add).
 */

class RenderingCamera
{
public:

    RenderingCamera(ros::NodeHandlePtr n,
                 image_transport::ImageTransport *it=NULL,
                 const std::string cam_name="", const std::string ns="");

    //    RenderingCamera(const RenderingCamera&);

    ~RenderingCamera() {delete ar_camera;};

    KDL::Frame GetWorldToCamTr(){return ar_camera->GetWorldToCamTr();};

    void SetWorldToCamTf(const KDL::Frame & frame);

    void SetPtrManipulatorInterestedInCamPose(Manipulator* in);

    void RefreshCamera(const int *window_size);

private:

    RenderingCamera(const RenderingCamera&);  // Purposefully not implemented.

    void operator=(const RenderingCamera&);  // Purposefully not implemented.


    void ConfigureBackgroundImage(cv::Mat img);

    /**
    * \brief Update the view angle of the virtual Camera according to window size
     * Note that the windows is the opengl window here,
    */
    void UpdateVirtualView(const int *window_size);

    // Set up the background scene_camera to fill the renderer with the image

    void UpdateBackgroundImage(const int *window_size);

    void SetCameraToFaceImage(const int *window_siz,
                              const int *imageSize, const double *spacing,
                              const double *origin);

    void UpdateCamPoseFollowers();

public:

    vtkSmartPointer<vtkCamera>              camera_virtual;
    vtkSmartPointer<vtkCamera>              camera_real;
    vtkSmartPointer<vtkImageActor>          image_actor_;

private:

    AugmentedCamera*                   ar_camera;
    bool                        is_initialized = false;
    std::vector<Manipulator*>   interested_manipulators;

    vtkSmartPointer<vtkImageImport>         image_importer_;
    vtkSmartPointer<vtkImageData>           camera_image_;
    vtkSmartPointer<vtkMatrix4x4>           intrinsic_matrix;

    double image_width_;
    double image_height_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;


};


#endif //RENDERINGCAMERA