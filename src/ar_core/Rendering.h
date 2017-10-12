//
// Created by nima on 4/12/17.
//

#ifndef ATAR_RENDERING_H
#define ATAR_RENDERING_H

#include <opencv2/opencv.hpp>
#include "CalibratedCamera.h"
#include <kdl/frames.hpp>

#include <vtkImageImport.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkImageActor.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOpenGLRenderer.h>
#include <vtkLightActor.h>
#include <vtkFrameBufferObject.h>
#include <vtkImageData.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkCamera.h>

#include <vtkCameraPass.h>
#include <vtkLightsPass.h>
#include <vtkSequencePass.h>
#include <vtkOpaquePass.h>
#include <vtkDepthPeelingPass.h>
#include <vtkTranslucentPass.h>
#include <vtkVolumetricPass.h>
#include <vtkOverlayPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkInformation.h>
#include <vtkProperty.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <assert.h>
#include <vtkFrustumSource.h>


/**
 * \class Rendering
 * \brief Rendering using opengl and vtk
 *
 * scene_renderer_ renders the virtual objects
 * background_renderer_ renders the images captured by the real camera
 *
 */

class Rendering {
public:

    //    vtkTypeMacro(Rendering, vtkRenderWindow);
    //    static Rendering *New();

    Rendering(bool AR_mode, uint num_windows, bool with_shaodws,
                  bool offScreen_rendering,
                  std::vector<int> window_position);

    ~Rendering();

    void SetWorldToCameraTransform(const cv::Vec3d cam_rvec[], const cv::Vec3d cam_tvec[]);

    void SetEnableBackgroundImage(bool isEnabled);

    void ConfigureBackgroundImage(cv::Mat *);

    void UpdateBackgroundImage(cv::Mat []);

    void UpdateCameraViewForActualWindowSize();

    void SetCameraIntrinsics(const cv::Mat intrinsics[]);

    void AddActorToScene(vtkSmartPointer<vtkProp> actor);

    void AddActorsToScene(std::vector< vtkSmartPointer<vtkProp> > actors);

    void RemoveAllActorsFromScene();

    void Render();

    void GetRenderedImage(cv::Mat *images);

    void ToggleFullScreen();

private:

    void AddShadowPass(vtkSmartPointer<vtkOpenGLRenderer>);

    // Set up the background scene_camera to fill the renderer with the image
    void SetImageCameraToFaceImage(const int id, const int *window_size);

private:
    int num_render_windows_;
    bool with_shadows_;
    bool ar_mode_;
    //cameras
    CalibratedCamera  *                     background_camera_[2];
    CalibratedCamera  *                     scene_camera_[3];

    vtkSmartPointer<vtkLight>               lights[2];
    // renderer
    vtkSmartPointer<vtkOpenGLRenderer>      background_renderer_[3];
    vtkSmartPointer<vtkOpenGLRenderer>      scene_renderer_[3];
    // image importing
    vtkSmartPointer<vtkImageImport>         image_importer_[2];
    vtkSmartPointer<vtkImageActor>          image_actor_[2];
    vtkSmartPointer<vtkImageData>           camera_image_[2];
    // transforms

    // windows
    vtkSmartPointer<vtkRenderWindow>        render_window_[3];
    //    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    // reading images back
    vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter_[2] ;

};


#endif //ATAR_RENDERING_H
