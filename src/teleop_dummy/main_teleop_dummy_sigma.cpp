//
// Created by nima on 12/10/17.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <src/ar_core/ControlEvents.h>

// This node simulates the slaves of the dvrk in a teleop mode and controls the
// behavior of the master console to mock that of the dvrk teleoperation mode.
// At the moment the dvrk does not publish the foot pedals status if the
// dvrk-console is not run in teleop mode. That's why we run the dvrk-console
// in a normal teleop mode, but Home the arms through this node (instead of
// the user interface) so that we can power up only the masters and not the
// slaves that are not needed here.



// ------------------------------------- global variables ---------------------------
int buttons[2];
bool new_buttons_msg;
bool new_master_pose;
KDL::Frame master_pose;
// ------------------------------------- callback functions ---------------------------
void ButtonsCallback(const sensor_msgs::JoyConstPtr & msg){
    for (int i = 0; i < msg->buttons.size(); ++i) {
        buttons[i] = msg->buttons[i];
    }
    new_buttons_msg = true;
}

void MasterPoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg){
    geometry_msgs::Pose pose = msg->pose;
    tf::poseMsgToKDL(msg->pose, master_pose);
    new_master_pose = true;

}



void ControlEventsCallback(const std_msgs::Int8ConstPtr
                           &msg) {

    int8_t control_event = msg->data;
    ROS_DEBUG("Received control event %d", control_event);

    switch(control_event){
        case CE_HOME_MASTERS:
            break;

        default:
            break;
    }

}


// ------------------------------------- Main ---------------------------
int main(int argc, char * argv[]) {

    ros::init(argc, argv, "teleop_dummy");
    ros::NodeHandle n(ros::this_node::getName());

    if( ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();

    // ------------------------------------- Buttons---------------------------
    ros::Subscriber sub_clutch_clutch = n.subscribe(
            "/sigma7/sigma0/buttons", 1, ButtonsCallback);


    // ------------ MATERS POSE
    ros::Subscriber sub_master_1_current_pose =  n.subscribe("/sigma/right/pose",
                                                             1, MasterPoseCurrentCallback);

    // ------------ SLAVE PUBLISH POSE

    std::string pose_topic = std::string("/sigma_slave/position_cartesian_current");
    ros::Publisher pub_slave_pose
            = n.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);

    // ------------ subscribe to control events that come from the GUI
    ros::Subscriber sub_control_events = n.subscribe("/atar/control_events", 1,
                                                     ControlEventsCallback);

    double scaling = 0.2;
    n.getParam("scaling", scaling);
    ROS_INFO(" Master to slave position scaling: %f", scaling);

    // spinning freq, publishing freq will be according to the freq of master poses received
    ros::Rate loop_rate(1000);

    KDL::Vector master_position_at_clutch_instance;
    KDL::Vector slave_position_at_clutch_instance;
    KDL::Frame slave_pose;

    // get initial tool position
    std::vector<double> init_tool_position[2]= {{0., 0., 0.} , {0., 0., 0.}};
    n.getParam("initial_tool_1_position", init_tool_position[0]);
    n.getParam("initial_tool_2_position", init_tool_position[1]);
    slave_pose.p = KDL::Vector(init_tool_position[0][0],
                               init_tool_position[0][1],
                               init_tool_position[0][2]);
    slave_pose.p = KDL::Vector(init_tool_position[1][0],
                               init_tool_position[1][1],
                               init_tool_position[1][2]);

    while(ros::ok()){

        if(new_buttons_msg){
            new_buttons_msg = false;
            master_position_at_clutch_instance = master_pose.p;
            slave_position_at_clutch_instance = slave_pose.p;
        }

        // Incremental slave position
        if(new_master_pose) {
            new_master_pose = false;

            // if operator present increment the slave position
            if(buttons[1]==1){
                slave_pose.p = slave_position_at_clutch_instance +
                               scaling * (master_pose.p - master_position_at_clutch_instance);
                slave_pose.M = master_pose.M;
            }

            //publish pose
            geometry_msgs::PoseStamped pose;
            tf::poseKDLToMsg( slave_pose, pose.pose);
            pub_slave_pose.publish(pose);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
