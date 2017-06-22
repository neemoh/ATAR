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
#include <kdl/frames.hpp>
#include <vtkOpenGLRenderer.h>
#include <vtkLightActor.h>
#include <vtkFrameBufferObject.h>
#include <vtkImageSinusoidSource.h>
#include <vtkImageData.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>
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
#include <vtkConeSource.h>
#include <vtkPlaneSource.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkInformation.h>
#include <vtkProperty.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <assert.h>
#include <vtkMath.h>
#include <vtkFrustumSource.h>
#include <vtkPlanes.h>
#include <vtkActorCollection.h>
#include <vtkPolyDataNormals.h>

/**
 * \class Rendering
 * \brief Rendering using opengl and vtk
 */

class Rendering {
public:

    //    vtkTypeMacro(Rendering, vtkRenderWindow);
    //    static Rendering *New();

    Rendering(uint num_windows, bool with_shaodws);
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

    void SetFullScreen();

private:

    void AddShadowPass(vtkSmartPointer<vtkOpenGLRenderer>);

    // Set up the background scene_camera to fill the renderer with the image
    void SetImageCameraToFaceImage(const int id, const int *window_size);

private:
    int num_render_windows_;
    bool with_shadows_;

    //cameras
    CalibratedCamera  *                     background_camera_[2];
    CalibratedCamera  *                     scene_camera_[2];

    vtkSmartPointer<vtkLight>               lights[2];
    // renderer
    vtkSmartPointer<vtkOpenGLRenderer>      background_renderer_[2];
    vtkSmartPointer<vtkOpenGLRenderer>      scene_renderer_[2];
    // image importing
    vtkSmartPointer<vtkImageImport>         image_importer_[2];
    vtkSmartPointer<vtkImageActor>          image_actor_[2];
    vtkSmartPointer<vtkImageData>           camera_image_[2];
    // transforms
    vtkSmartPointer<vtkMatrix4x4>           camera_to_world_transform_[2];
    // windows
    vtkSmartPointer<vtkRenderWindow>        render_window_[2];
    //    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    // reading images back
    vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter_[2] ;

};

namespace VTKConversions{

    void AxisAngleToVTKMatrix (const cv::Vec3d cam_rvec, const cv::Vec3d cam_tvec,
                               vtkSmartPointer<vtkMatrix4x4> out);

    void KDLFrameToVTKMatrix (const KDL::Frame in,
                               vtkSmartPointer<vtkMatrix4x4> out);

    void VTKMatrixToKDLFrame(const vtkSmartPointer<vtkMatrix4x4> in,
                                               KDL::Frame  & out);

}
#endif //TELEOP_VISION_RENDERING_H
