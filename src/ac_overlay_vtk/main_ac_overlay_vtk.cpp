//
// Created by nima on 4/13/17.
//
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "custom_conversions/Conversions.h"
#include "OverlayROSConfig.h"
#include <ode/ode.h>

dWorldID World;

dJointGroupID contactgroup;

int main(int argc, char **argv)
{
    World = dWorldCreate();
    contactgroup = dJointGroupCreate(0);

    ros::init(argc, argv, "ac_overlay");
    OverlayROSConfig rc (ros::this_node::getName());

    ros::Rate loop_rate(30);

    while (ros::ok())
    {

        if(!rc.UpdateWorld())
            break;

        // copying the render buffer back from gpu takes a very long time,
        // creating a bottle neck that reduced the update rate to 25 Hz
        // already. So no sleep is needed foe now.
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}



