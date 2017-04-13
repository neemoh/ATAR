//
// Created by nima on 4/12/17.
//

#ifndef TELEOP_VISION_RENDERING_H
#define TELEOP_VISION_RENDERING_H
#include "opencv2/core/core.hpp"
#include "CalibratedCamera.h"
#include <vtkImageImport.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkImageActor.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>

/**
 * \class Rendering
 * \brief Rendering using opengl and vtk
 */

class Rendering {
public:

//    vtkTypeMacro(Rendering, vtkRenderWindow);
//    static Rendering *New();

    Rendering();
    ~Rendering();

    void SetWorldToCameraTransform(const cv::Vec3d &cam_rvec, const cv::Vec3d &cam_tvec);

    void SetEnableImage(bool isEnabled);

    void SetupBackgroundImage(cv::Mat &);

    void UpdateBackgroundImage(cv::Mat &);

    void UpdateViewAngleForActualWindowSize();

    void SetCameraIntrinsics(const cv::Matx33d& intrinsics);

    void AddActorToScene(vtkSmartPointer<vtkProp> actor);

    void Render();

    void GetRenderedImage(cv::Mat & img);


private:


    // Set up the background scene_camera to fill the renderer with the image
    void SetImageCameraToFaceImage();


    vtkSmartPointer<CalibratedCamera>    BackgroundCamera;
    vtkSmartPointer<CalibratedCamera>    SceneCamera;
    vtkSmartPointer<vtkImageImport>      ImageImporter;
    vtkSmartPointer<vtkImageActor>       ImageActor;
    vtkSmartPointer<vtkRenderer>         backgroundRenderer;
    vtkSmartPointer<vtkRenderer>         sceneRenderer;
    vtkSmartPointer<vtkMatrix4x4>        WorldToCameraTransform;
    vtkSmartPointer<vtkMatrix4x4>        CameraToWorldTransform;
    vtkSmartPointer<vtkImageData>        camera_image;
    vtkSmartPointer<vtkRenderWindow>     renderWindow;
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter ;
};


#endif //TELEOP_VISION_RENDERING_H
