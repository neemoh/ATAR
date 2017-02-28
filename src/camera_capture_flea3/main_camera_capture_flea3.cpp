
#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "Fla3Camera.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}


int main(int argc, char *argv[]){

    uint left_cam_serial_num =14150439;

    // camera_intrinsics related stuff
    Fla3Camera cameras;
    cameras.Init();

//    uint num_cams = cameras.GetNumberOfCameras();
    std::vector<uint32_t> cam_serial_nums = cameras.getAttachedCameras();

    uint num_cams = (uint)cam_serial_nums.size();

    for (uint j = 0; j <num_cams ; ++j) {
        cameras.ConnectCamera(j);
        cameras.StartCameraCapture(j);
    }

    unsigned char **imageData;
    imageData = new unsigned char*[num_cams];
    *imageData = NULL;

//    unsigned char* imageData[num_cams];
    int *p_rowSize;
    int *p_colSize;
    p_rowSize = new int[num_cams];
    p_colSize = new int[num_cams];


    // Initialize ROS variables

    // Override SIGINT handler

    ros::init(argc, argv, "flea3", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    ros::NodeHandle nh(ros::this_node::getName());
    double ros_freq;
    nh.param<double>("frequency", ros_freq, 30);

    ros::Rate loop_rate(ros_freq);

    // Get ros parameters
    std::string left_image_topic_name;
    if (!nh.getParam("left_image_topic_name", left_image_topic_name))
        left_image_topic_name = "/left_cam";

    std::string right_image_topic_name;
    if (!nh.getParam("right_image_topic_name", right_image_topic_name))
        right_image_topic_name = "/right_cam";

    //determine left and right cams
    std::string img_topic_names[num_cams];

    if(num_cams>0){
        if(cam_serial_nums[0]==left_cam_serial_num){
            img_topic_names[0] = nh.resolveName(left_image_topic_name);
            if(num_cams>1)
                img_topic_names[1] = nh.resolveName(right_image_topic_name);
        }
        else{
            img_topic_names[0] = nh.resolveName(right_image_topic_name);
            if(num_cams>1)
                img_topic_names[1] = nh.resolveName(left_image_topic_name);
        }
    }

    sensor_msgs::Image image[num_cams];
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub[num_cams];
    for (int l = 0; l < num_cams; ++l) {
       pub[l] = it.advertise(img_topic_names[l], 1);
    }

    while(!g_request_shutdown){

        for (uint i = 0; i < num_cams; i++) {
            cameras.grabImage(i, image[i]);
            pub[i].publish(image[i]);
        }

        loop_rate.sleep();
    }

    ROS_INFO("Shutting down.");
    for (uint k = 0; k <num_cams ; ++k) {
        cameras.StopCameraCapture(k);
        cameras.DisconnectCamera(k);
    }

    delete imageData;
    ros::shutdown();
	return 0;
}




//
//// =========================================================================================
//void GetImageLeft(vctUCharVec &a_image) const
//{
//    a_image.SetSize(p_rowSize[1] * p_colSize[1] * 3);
//
//    /*
//    for (int i = 0; i < p_rowSize[0]*p_colSize[0]*3; i++)
//    {
//        a_image[i] = imageData[0][i];
//    }
//    */
//
//    memcpy(a_image.Pointer(0), imageData[1], p_rowSize[1]*p_colSize[1]*3);
//
//    //CMN_LOG_CLASS_INIT_VERBOSE << "image Left"  << std::endl;
//}
//
//// =========================================================================================
//void GetImageRight(vctUCharVec &a_image) const
//{
//    a_image.SetSize(p_rowSize[0] * p_colSize[0] * 3);
//
//    /*
//    for (int i = 0; i < p_rowSize[1]*p_colSize[1]*3; i++)
//    {
//        a_image[i] = imageData[1][i];
//    }
//    */
//
//    memcpy(a_image.Pointer(0), imageData[0], p_rowSize[0]*p_colSize[0]*3);
//
//    //CMN_LOG_CLASS_INIT_VERBOSE << "image Right"  << std::endl;
//}
//
//// =========================================================================================
//void GetRowSizeLeft(int &a_size) const
//{
//    a_size = p_rowSize[0];
//}
//
//// =========================================================================================
//void GetColSizeLeft(int &a_size) const
//{
//    a_size = p_colSize[0];
//}
//
//// =========================================================================================
//void GetRowSizeRight(int &a_size) const
//{
//    a_size = p_rowSize[1];
//}
//
//// =========================================================================================
//void GetColSizeRight(int &a_size) const
//{
//    a_size = p_colSize[1];
//}