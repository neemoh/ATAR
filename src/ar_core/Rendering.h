//
// Created by nima on 4/12/17.
//

#ifndef ATAR_RENDERING_H
#define ATAR_RENDERING_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "RenderingCamera.h"
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

    Rendering(std::vector<int> view_resolution={640,480},
              bool ar_mode=false,
              int num_views =1,
              bool one_window_per_view=false,
              bool borders_off=false,
              std::vector<int> window_positions={100,50,740,50,1380,50});

    ~Rendering();

    void SetEnableBackgroundImage(bool isEnabled);

    void UpdateCameraViewForActualWindowSize();

    void AddActorsToScene(std::vector< vtkSmartPointer<vtkProp> > actors);

    void AddActorToScene(vtkSmartPointer<vtkProp> actor);

    void RemoveAllActorsFromScene();

    void Render();

    bool AreImagesNew();

    void GetRenderedImage(cv::Mat *images);

    void ToggleFullScreen();

    void SetManipulatorInterestedInCamPose(Manipulator*);

    void SetMainCameraPose(const KDL::Frame& pose);

    KDL::Frame GetMainCameraPose() {return cameras[0]->GetWorldToCamTr();};

private:

    void GetCameraNames(int num_views, std::string cam_names[], std::string
    &ns);

    void AddShadowPass(vtkSmartPointer<vtkOpenGLRenderer>);

    void SetupLights();

    void PublishRenderedImages();


private:
    int n_windows;
    int n_views;
    bool with_shadows_;
    bool ar_mode_;
    bool publish_overlayed_images_;

    // transfroms between cameras used if cam 1 or both cam 1 and cam2 exist
    // and when the SetMainCameraPose method is called externally
    KDL::Frame                              cam0_to_cam1_tr;
    KDL::Frame                              cam0_to_cam2_tr;

    //cameras
    RenderingCamera *                       cameras [3];
    image_transport::ImageTransport *       it;

    vtkSmartPointer<vtkLight>               lights[2];

    // renderer
    vtkSmartPointer<vtkOpenGLRenderer>      background_renderer_[3];
    vtkSmartPointer<vtkOpenGLRenderer>      scene_renderer_[3];

    // windows
    vtkSmartPointer<vtkRenderWindow>        render_window_[3];

    // reading images back
    vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter_[3] ;

    //overlay image publishers (SLOW)
    image_transport::Publisher              publisher_stereo_overlayed;
};


#endif //ATAR_RENDERING_H
