#include "Fla3Camera.h"
#include <sensor_msgs/fill_image.h>

//#include <stdexcept>

// ============================================================================
// ----------------------------------------------------------------------------
Fla3Camera::Fla3Camera()
{
}

// ----------------------------------------------------------------------------
Fla3Camera::~Fla3Camera()
{
    delete[] p_osaFlea3CameraConnect;
    delete[] capture_running;
    delete[] p_PGIdentity;
    delete[] cameras;
}



// ----------------------------------------------------------------------------
void Fla3Camera::Init()
{
    num_cams = DetermineNumberOfCameras();
    printf("Number of cameras is %d\n", num_cams);

    // Initialize the variable that keep track of camera connect and start
    p_osaFlea3CameraConnect = new bool[num_cams];
    capture_running = new bool[num_cams];
    p_PGIdentity = new PGRGuid[num_cams];
    cameras = new Camera[num_cams];
    for (int i = 0 ; i < num_cams; i++)
    {
        p_osaFlea3CameraConnect[i] = false;
        capture_running[i] = false;
    }

    
}

// ----------------------------------------------------------------------------
uint Fla3Camera::GetNumberOfCameras()
{
    return num_cams;
}

// ----------------------------------------------------------------------------
void Fla3Camera::ConnectCamera(uint a_cameraNum)
{
    if ((a_cameraNum +1) > num_cams)
        throw std::runtime_error("Trying to connect to an invalid camera!");

    if (!p_osaFlea3CameraConnect[a_cameraNum])
    {
        Error t_error;
        t_error = bus_manager.GetCameraFromIndex(a_cameraNum, &(p_PGIdentity[a_cameraNum]));
        if (t_error != PGRERROR_OK)
        {
            throw std::runtime_error("Error: Fla3Camera::ConnectCamera::GetCameraFromIndex");
        }
        else
        {
            // Connect to a camera
            t_error = cameras[a_cameraNum].Connect(&(p_PGIdentity[a_cameraNum]));
            if (t_error != PGRERROR_OK)
            {
                throw std::runtime_error("Error: Fla3Camera::ConnectCamera::Connect");
            }
            else
            {
                // Set the camera to be connected
                p_osaFlea3CameraConnect[a_cameraNum] = true;
                printf("camera_intrinsics %d connected!\n", a_cameraNum);
            }
        }
    }
    FlyCapture2::PixelFormat a;
}

// ----------------------------------------------------------------------------
void Fla3Camera::DisconnectCamera(uint a_cameraNum)
{
    // Disconnect the camera
    Error t_error;
    t_error = cameras[a_cameraNum].Disconnect();
    if (t_error != PGRERROR_OK)
        throw std::runtime_error("Error: Fla3Camera::DisconnectCamera");
    else// Set the camera to be disconnected
        p_osaFlea3CameraConnect[a_cameraNum] = false;
    std::cout << "camera_intrinsics " << a_cameraNum << " disconnected." << std::endl;

}

// ----------------------------------------------------------------------------
uint Fla3Camera::DetermineNumberOfCameras()
{
    uint t_numCameras;
    Error t_error = bus_manager.GetNumOfCameras(&t_numCameras);
    if (t_error != PGRERROR_OK)
            throw std::runtime_error("Error: Fla3Camera::DetermineNumberOfCameras");

    return t_numCameras;
}

// ----------------------------------------------------------------------------
void Fla3Camera::StartCameraCapture(uint a_cameraNum)
{
    Error t_error;

    if ((a_cameraNum+1) > num_cams)
        throw std::runtime_error("Trying to start an invalid camera!");
    else if (!p_osaFlea3CameraConnect[a_cameraNum])
        throw std::runtime_error("Trying to start a camera that is not connected!");

    // Start capturing images
    t_error = cameras[a_cameraNum].StartCapture();
    if (t_error != PGRERROR_OK)
        throw std::runtime_error("Error: Fla3Camera::StartCameraCapture");
    else
    {
        capture_running[a_cameraNum] = true;
        printf("camera_intrinsics %d started!\n", a_cameraNum);
    }

}

// ----------------------------------------------------------------------------
void Fla3Camera::StopCameraCapture(uint a_cameraNum)
{
    Error error;

    if ((a_cameraNum+1) > num_cams)
        throw std::runtime_error("Trying to stop an invalid camera!");

    else if (!p_osaFlea3CameraConnect[a_cameraNum])
        throw std::runtime_error("Trying to stop a camera that is not connected!");

    // Stop capturing images
    error = cameras[a_cameraNum].StopCapture();
    HandleError("Fla3Camera::StopCameraCapture Failed to stop capturing", error);
    capture_running[a_cameraNum] = false;
    std::cout << "camera_intrinsics " << a_cameraNum << " stopped." << std::endl;


}

// ----------------------------------------------------------------------------
//void Fla3Camera::GetImageFromCamera(int a_cameraNum, unsigned char** a_data, int *a_rowSize, int *a_colSize)
void Fla3Camera::GetImageFromCamera(uint a_cameraNum, cv::Mat &image_out, int *a_rowSize, int *a_colSize)
{
    if ((a_cameraNum+1) > num_cams)
        throw std::runtime_error("Trying to get the image from an invalid camera!");
    else if (!p_osaFlea3CameraConnect[a_cameraNum])
        throw std::runtime_error("Trying to get the image from a camera that is not connected!");
    else if (!capture_running[a_cameraNum])
        throw std::runtime_error("Trying to get the image from a camera that is not started!");


    FlyCapture2::Error t_error;
    FlyCapture2::Image t_rawImage;

    // Retrieve an image
    t_error = cameras[a_cameraNum].RetrieveBuffer( &t_rawImage );
    if (t_error != PGRERROR_OK)
    {
        throw std::runtime_error("Error: Unable to get image from camera");
    }

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    t_rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );

    // Create a converted image
    FlyCapture2::Image rgbImage;

    // Convert the raw image
    t_error = t_rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
    if (t_error != PGRERROR_OK)
    {
        throw std::runtime_error("Error: Unable to convert image from camera");
    }

    // Copy the image from to the data buffer
    // unsigned char *t_data = t_convertedImage.GetData();
//    rgbImage.GetDataSize();

    // Copy FlyCapture2 image into OpenCV struct
//    if ((*a_data) == NULL){
//        *a_data = new unsigned char[rgbImage.GetDataSize()];
//        std::cout << "(*a_data) == NULL " << std::endl;
//
//    }
//    memcpy( *a_data,
//            rgbImage.GetData(),
//            rgbImage.GetDataSize());


    // convert to OpenCV Mat
    unsigned int rowBytes = rgbImage.GetReceivedDataSize()/rgbImage.GetRows();
//    cv::Mat image_temp = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
    cv::Mat image_temp = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

    cv::imshow("Aruco extrinsic", image_temp);
    cv::waitKey(1);
    *a_rowSize = (int)rows;
    *a_colSize = (int)cols;
}

// ----------------------------------------------------------------------------
void Fla3Camera::GetAndSaveImageFromCamera(int a_cameraNum, char* a_filename)
{
    if ((a_cameraNum+1) > num_cams)
    {
        throw std::runtime_error("Trying to get the image from an invalid camera!");
    }
    else if (!p_osaFlea3CameraConnect[a_cameraNum] )
    {
        throw std::runtime_error("Trying to get the image from a camera that is not connected!");
    }
    else if (!capture_running[a_cameraNum])
    {
        throw std::runtime_error("Trying to get the image from a camera that is not started!");
    }

    Error t_error;
    FlyCapture2::Image t_rawImage;

    // Retrieve an image
    t_error = cameras[a_cameraNum].RetrieveBuffer( &t_rawImage );
    if (t_error != PGRERROR_OK)
    {
        throw std::runtime_error("Error: Unable to get image from camera");
    }

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    t_rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );

    // Create a converted image
    FlyCapture2::Image t_convertedImage;

    // Convert the raw image
    t_error = t_rawImage.Convert( PIXEL_FORMAT_RGB8, &t_convertedImage );
    if (t_error != PGRERROR_OK)
        throw std::runtime_error("Error: Unable to convert image from camera");

    // Save the image. If a file format is not passed in, then the file
    // extension is parsed to attempt to determine the file format.
    t_error = t_convertedImage.Save(a_filename);
    if (t_error != PGRERROR_OK)
    {
        throw std::runtime_error("Error: Unable to save image from camera");
    }
}

// ----------------------------------------------------------------------------
void Fla3Camera::SetCameraProperty(int a_cameraNum, char* a_configFile)
{

}

// ----------------------------------------------------------------------------
double Fla3Camera::GetFrameRate(int a_cameraNum)
{
    Error t_error;

    if ((a_cameraNum+1) > num_cams)
    {
        throw std::runtime_error("Trying to grab the frame rate of an invalid camera!");
    }
    else if (!p_osaFlea3CameraConnect[a_cameraNum])
    {
        throw std::runtime_error("Trying to grab the frame rate of a camera that is not connected!");
    }
    else if (!capture_running[a_cameraNum])
    {
        throw std::runtime_error("Trying to grab the frame rate of a camera that is not started!");
    }

    // Retrieve frame rate property
    Property t_frmRate;

    t_frmRate.type = FRAME_RATE;
    t_error = cameras[a_cameraNum].GetProperty( &t_frmRate );
    if (t_error != PGRERROR_OK)
    {
        throw std::runtime_error("Error: Fla3Camera::GetFrameRate");
    }
    else
    {
        return (double)t_frmRate.absValue;
    }
}

// ----------------------------------------------------------------------------



void Fla3Camera::grabImage(uint cam_num, sensor_msgs::Image &image)
{
    boost::mutex::scoped_lock scopedLock(mutex_);
    if(cameras[cam_num].IsConnected() && capture_running[cam_num])
    {

        // Make a FlyCapture2::image to hold the buffer returned by the camera.
        Image rawImage;
        // Retrieve an image
        Error error = cameras[cam_num].RetrieveBuffer(&rawImage);
        HandleError("PointGreyCamera::grabImage Failed to retrieve buffer", error);

        // Set header timestamp as embedded for now
        TimeStamp embeddedTime = rawImage.GetTimeStamp();
        image.header.stamp.sec = embeddedTime.seconds;
        image.header.stamp.nsec = 1000 * embeddedTime.microSeconds;

        // Check the bits per pixel.
        auto bitsPerPixel = rawImage.GetBitsPerPixel();

        // Set the image encoding
        std::string imageEncoding =  sensor_msgs::image_encodings::BAYER_RGGB8;

        fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
//        image.header.frame_id = frame_id;
    }
    else if(cameras[cam_num].IsConnected())
    {
        throw std::runtime_error("PointGreyCamera::grabImage: camera_intrinsics is currently not running.  Please start the capture.");
    }
    else
    {
        throw std::runtime_error("PointGreyCamera::grabImage not connected!");
    }
}


void Fla3Camera::HandleError(const std::string &prefix, const FlyCapture2::Error &error)
{
    if(error == PGRERROR_TIMEOUT)
    {
        throw std::runtime_error("PointGreyCamera: Failed to retrieve buffer within timeout.");
    }
    else if(error != PGRERROR_OK)     // If there is actually an error (PGRERROR_OK means the function worked as intended...)
    {
        std::string start(" | FlyCapture2::ErrorType ");
        std::stringstream out;
        out << error.GetType();
        std::string desc(error.GetDescription());
        throw std::runtime_error(prefix + start + out.str() + " " + desc);
    }
}


std::vector<uint32_t> Fla3Camera::getAttachedCameras()
{
    std::vector<uint32_t> serials;
    Error error = bus_manager.GetNumOfCameras(&num_cams);
    HandleError("PFla3Camera::getAttachedCameras: Could not get number of cameras", error);
    for(unsigned int i = 0; i < num_cams; i++)
    {
        unsigned int this_serial;
        error = bus_manager.GetCameraSerialNumberFromIndex(i, &this_serial);
        HandleError("PointGreyCamera::getAttachedCameras: Could not get get serial number from index", error);
        serials.push_back(this_serial);
        std::cout << "Found camera " << i << " with serial number: "<< this_serial<< std::endl;
    }
    return serials;
}