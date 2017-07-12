//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "custom_conversions/Conversions.h"
#include "OverlayROSConfig.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "ac_overlay");
    OverlayROSConfig rc (ros::this_node::getName());

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        if(!rc.UpdateWorld())
            break;
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}



