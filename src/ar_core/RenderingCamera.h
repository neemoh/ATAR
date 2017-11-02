//
// Created by nima on 4/12/17.
//

#ifndef bardCalibratedCamera_h
#define bardCalibratedCamera_h

#include "src/ar_core/Manipulator.h"

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>

#include <vtkImageImport.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
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

class RenderingCamera
{
public:

    RenderingCamera(ros::NodeHandlePtr n,
                 image_transport::ImageTransport *it=NULL,
                 const std::string cam_name="", const std::string ns="");

    //    RenderingCamera(const RenderingCamera&);

    ~RenderingCamera() {}

    bool IsImageNew();

    void ImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    KDL::Frame GetWorldToCamTr(){return world_to_cam_pose;};

    void SetWorldToCamTf(const KDL::Frame & frame);

    void SetPtrManipulatorInterestedInCamPose(Manipulator* in);

    void RefreshCamera(const int *window_size);

private:

    RenderingCamera(const RenderingCamera&);  // Purposefully not implemented.

    void operator=(const RenderingCamera&);  // Purposefully not implemented.

    void ReadCameraParameters(const std::string file_path);

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

    void LockAndGetImage(cv::Mat &image, std::string cam_name);



    void UpdateCamPoseFollowers();

public:

    vtkSmartPointer<vtkCamera>              camera_virtual;
    vtkSmartPointer<vtkCamera>              camera_real;
    vtkSmartPointer<vtkImageActor>          image_actor_;

private:

    cv::Mat                     image_from_ros;
    bool                        new_image = false;
    bool                        new_cam_pose = false;
    bool                        is_initialized = false;
    std::vector<Manipulator*>   interested_manipulators;

    KDL::Frame                  world_to_cam_pose;
    cv::Mat                     img;

    image_transport::Subscriber sub_image;
    ros::Subscriber             sub_pose;

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


#endif
