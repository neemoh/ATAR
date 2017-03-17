#ifndef OSAFLEA3CAMERA_H
#define OSAFLEA3CAMERA_H
// ============================================================================
#include "flycapture/FlyCapture2.h"
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <boost/thread/mutex.hpp>

#include "iostream"
// ============================================================================
using namespace FlyCapture2;
// ============================================================================
class Fla3Camera
{
public:
    // Constructor
    Fla3Camera();

    // Destructor
    ~Fla3Camera();

    void Init();
    uint GetNumberOfCameras();
    void ConnectCamera(uint a_cameraNum);
    void DisconnectCamera(uint a_cameraNum);
    void StartCameraCapture(uint a_cameraNum);
    void StopCameraCapture(uint a_cameraNum);
//    void GetImageFromCamera(int a_cameraNum, unsigned char **a_data, int *a_rowSize, int *a_colSize);
    void GetImageFromCamera(uint a_cameraNum, cv::Mat &image_out, int *a_rowSize, int *a_colSize);
    void grabImage(uint cam_num, sensor_msgs::Image &image);
    sensor_msgs::ImagePtr ResizeImage(const sensor_msgs::Image &image_in);
    void HandleError(const std::string &prefix, const FlyCapture2::Error &error);
    std::vector<uint32_t> getAttachedCameras();

    void GetAndSaveImageFromCamera(int a_cameraNum, char* a_filename);
    void SetCameraProperty(int a_cameraNum, char* a_configFile);
    double GetFrameRate(int a_cameraNum);

private:
    bool *p_osaFlea3CameraConnect;
    bool *capture_running;
    uint num_cams;
    BusManager bus_manager;
    PGRGuid *p_PGIdentity;
    Camera *cameras;
    boost::mutex mutex_;
    uint DetermineNumberOfCameras();
};

#endif
// ============================================================================
