//
// Created by nima on 4/12/17.
//

#ifndef ATAR_RENDERING_H
#define ATAR_RENDERING_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ARCamera.h"
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

    Rendering(ros::NodeHandlePtr n);

    ~Rendering();

    void SetEnableBackgroundImage(bool isEnabled);

    void UpdateCameraViewForActualWindowSize();

    void AddActorsToScene(std::vector< vtkSmartPointer<vtkProp> > actors);

    void RemoveAllActorsFromScene();

    void Render();

    bool AreImagesNew();

    void GetRenderedImage(cv::Mat *images);

    void ToggleFullScreen();

private:

    void AddShadowPass(vtkSmartPointer<vtkOpenGLRenderer>);

    void SetupLights();

private:
    int num_render_windows_;
    bool with_shadows_;
    bool ar_mode_;

    //cameras
    ARCamera *                              cameras [3];
    image_transport::ImageTransport *       it;

    vtkSmartPointer<vtkLight>               lights[2];

    // renderer
    vtkSmartPointer<vtkOpenGLRenderer>      background_renderer_[3];
    vtkSmartPointer<vtkOpenGLRenderer>      scene_renderer_[3];

    // windows
    vtkSmartPointer<vtkRenderWindow>        render_window_[3];

    // reading images back
    vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter_[2] ;

};


#endif //ATAR_RENDERING_H
