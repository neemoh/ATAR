//
// Created by nima on 24/05/17.
//
#include <iostream>
#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>


namespace {
    const char* about = "Capture and publisher images from usb cameras";
    const char* keys  =
                    "{id1        |       | camera 1 id }"
                    "{id2        |       | camera 2 id }";
}


int main(int argc, char *argv[]){

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);


    std::vector<int> cam_ids;
    std::vector<std::string> window_names;

    std::cout << "argc= " << argc << std::endl;
    if(argc == 3){
        cam_ids.push_back(parser.get<int>("id1"));
        std::cout << "Using uSB cam with id " << cam_ids[0] << std::endl;

        cam_ids.push_back(parser.get<int>("id2"));
        std::cout << "Using uSB cam with id " << cam_ids[1] << std::endl;
    }
    else if(argc == 2) {
        cam_ids.push_back(parser.get<int>("id1"));
    }
    else{
        parser.printMessage();
        return 0;
    }


    for (int i = 0; i < cam_ids.size(); ++i) {
        std::stringstream win_name;
        win_name << "cam " << cam_ids[i];
        window_names.push_back(win_name.str());
    }


    ros::init(argc, argv, "usb_cam_publisher");
    std::string ros_node_name = ros::this_node::getName();
    ros::NodeHandle n(ros_node_name);


    ros::Rate loop_rate(1);

    cv::VideoCapture inputVideo[cam_ids.size()];
    int waitTime;


    for (int i = 0; i < cam_ids.size(); ++i) {
        inputVideo[i].open(cam_ids[i]);
    }



    while(ros::ok() ){

        for (int i = 0; i < cam_ids.size(); ++i) {
            inputVideo[i].grab();
            cv::Mat image;
            inputVideo[i].retrieve(image);

            imshow(window_names[i], image);
            loop_rate.sleep();


        }

        char key = (char)cv::waitKey(1);
        if(key == 27) break;


        ros::spinOnce();
        loop_rate.sleep();
    }


}
