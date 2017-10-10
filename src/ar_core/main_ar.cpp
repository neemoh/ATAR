//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "ARCore.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "ar_core");
    ARCore acore (ros::this_node::getName());

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        if(!acore.UpdateWorld())
            break;
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

