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
#include <vtkRenderWindowInteractor.h>

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

    void SetWorldToCameraTransform(const cv::Vec3d cam_rvec[], const cv::Vec3d cam_tvec[]);

    void SetEnableImage(bool isEnabled);

    void SetupBackgroundImage(cv::Mat []);

    void UpdateBackgroundImage(cv::Mat []);

    void UpdateViewAngleForActualWindowSize();

    void SetCameraIntrinsics(const cv::Matx33d intrinsics[]);

    void AddActorToScene(vtkSmartPointer<vtkProp> actor);

    void Render();

    void GetRenderedImage(cv::Mat & img);


private:


    // Set up the background scene_camera to fill the renderer with the image
    void SetImageCameraToFaceImage(const int id);

    //cameras
    vtkSmartPointer<CalibratedCamera>    background_camera_[2];
    vtkSmartPointer<CalibratedCamera>    scene_camera_[2];
    // renderer
    vtkSmartPointer<vtkRenderer>         background_renderer_[2];
    vtkSmartPointer<vtkRenderer>         scene_renderer[2];
    // image importing
    vtkSmartPointer<vtkImageImport>      image_importer_[2];
    vtkSmartPointer<vtkImageActor>       image_actor_[2];
    vtkSmartPointer<vtkImageData>        camera_image[2];
    // transforms
    vtkSmartPointer<vtkMatrix4x4>        camera_to_world_transform_[2];
    // windows
    vtkSmartPointer<vtkRenderWindow>     render_window;
    //    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    // reading images back
    vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter ;

};


#endif //TELEOP_VISION_RENDERING_H
