//
// Created by charm on 3/27/17.
//

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float32.h>

#include "opencv2/highgui/highgui.hpp"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>

cv::Mat image[2];
cv::Mat stereo_image;
bool new_right_image = false;
bool new_left_image = false;
bool new_stereo_image = false;

void ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg);
void ImageRightCallback(const sensor_msgs::ImageConstPtr& msg);
void ImageStereoCallback(const sensor_msgs::ImageConstPtr& msg);
void SwitchFullScreen(const std::string window_name);

int main(int argc, char **argv)
{

    int num_topics =0;
    std::string left_image_topic_name;
    std::string right_image_topic_name;
    std::string stereo_image_topic_name;

    if ( argc < 2 )
    {
        std::cout << "Input argument: Topic name(s). \n"
                "Provide one name if the topic contains a side by side image of left and right cameras.\n"
                "Provide two if left and right images are published on separate topics." << std::endl;
        return EXIT_FAILURE;
    }
    else if (argc == 2){
        num_topics = 1;
        stereo_image_topic_name = argv[1];
    }
    else if (argc == 3){
        num_topics = 2;
        left_image_topic_name = argv[1];
        right_image_topic_name = argv[2];
    }

    ros::init(argc, argv, "stereo_view");
    ros::NodeHandle nh(ros::this_node::getName());

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_subscribers[2];

    if (num_topics == 1) {
        //--------
        // stereo image subscriber
        ROS_INFO("[SUBSCRIBERS] Both camera images will be read from topic '%s'",
                 left_image_topic_name.c_str());
        image_subscribers[0] = it.subscribe(
                stereo_image_topic_name, 1, ImageStereoCallback);

    }

    if (num_topics == 2) {
        //--------
        // Left image subscriber
        if (nh.getParam("left_image_topic_name", left_image_topic_name))
            ROS_INFO(
                    "%s [SUBSCRIBERS] Left camera images will be read from topic '%s'",
                    ros::this_node::getName().c_str(),
                    left_image_topic_name.c_str());
        image_subscribers[0] = it.subscribe(
                left_image_topic_name, 1, ImageLeftCallback);

        //--------
        // Left image subscriber.
        if (nh.getParam("right_image_topic_name", right_image_topic_name))
            ROS_INFO(
                    "%s [SUBSCRIBERS] Right camera images will be read from topic '%s'",
                    ros::this_node::getName().c_str(),
                    right_image_topic_name.c_str());
        image_subscribers[1] = it.subscribe(
                right_image_topic_name, 1, ImageRightCallback);

    }
    // frequency of the generated images is based on the received images
    // loop_rate is the frequency of spinning and checking for new messages
    ros::Rate loop_rate(200);

    std::string window_name[2];

    window_name[0] = "Overlay  Left";
    window_name[1] = "Overlay  Right";
    // Create the window for the video feed
    cvNamedWindow(window_name[0].c_str(),CV_WINDOW_NORMAL);
    cvNamedWindow(window_name[1].c_str(),CV_WINDOW_NORMAL);


    while (ros::ok())
    {
        // --------------------------------------------------------------------------------------
        // keyboard commands


        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
        {
            SwitchFullScreen(window_name[0]);
            SwitchFullScreen(window_name[1]);
        }



        if(new_left_image && new_right_image || new_stereo_image) {

            if(num_topics==1){
                int image_width = stereo_image.cols;
                int image_height = stereo_image.rows;

                image[0] = cv::Mat(stereo_image, cv::Rect(0, 0, image_width/2, image_height));
                image[1] = cv::Mat(stereo_image, cv::Rect(image_width/2, 0, image_width/2, image_height));

            }

            for (int j = 0; j <2 ; ++j) {
                cv::imshow(window_name[j], image[j]);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}





void ImageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image[0] = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_right_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image[1] = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_left_image = true;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageStereoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        stereo_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_stereo_image = true;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void SwitchFullScreen(const std::string window_name) {

    if (cvGetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN) ==
        CV_WINDOW_NORMAL)
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    else
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

}