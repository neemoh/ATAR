//
// Created by nima on 19/07/17.
//
#include <iosfwd>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/String.h>

// This node simulates the slaves of the dvrk in a teleop mode and controls the
// behavior of the master console to mock that of the dvrk teleoperation mode.
// At the moment the dvrk does not publish the foot pedals status if the
// dvrk-console is not run in teleop mode. That's why we run the dvrk-console
// in a normal teleop mode, but Home the arms through this node (instead of
// the user interface) so that we can power up only the masters and not the
// slaves that are not needed here.



// ------------------------------------- global variables ---------------------------
bool clutch_pressed;
bool coag_pressed;
bool new_coag_msg;
bool new_clutch_msg;
bool new_master_pose[2];
KDL::Frame master_pose[2];
std::string master_state[2];

// ------------------------------------- callback functions ---------------------------
void ClutchCallback(const sensor_msgs::JoyConstPtr & msg){
    clutch_pressed = (bool)msg->buttons[0];
    new_clutch_msg = true;
}

void CoagCallback(const sensor_msgs::JoyConstPtr & msg){
    coag_pressed = (bool)msg->buttons[0];
    new_coag_msg = true;
}

void Master1PoseCurrentCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg){
    geometry_msgs::Pose pose = msg->pose;
    tf::poseMsgToKDL(msg->pose, master_pose[0]);
    new_master_pose[0] = true;

}
void Master2PoseCurrentCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg){
    geometry_msgs::Pose pose = msg->pose;
    tf::poseMsgToKDL(msg->pose, master_pose[1]);
    new_master_pose[1] = true;
}

void Master1StateCallback(
    const std_msgs::String::ConstPtr &msg){
    master_state[0] = msg->data;
    ROS_DEBUG( "Master 1 says: %s" , master_state[0].data());

}

void Master2StateCallback(
    const std_msgs::String::ConstPtr &msg){
    master_state[1] = msg->data;
    ROS_DEBUG( "Master 2 says: %s" , master_state[1].data());

}

// ------------------------------------- Main ---------------------------
int main(int argc, char * argv[]) {

    ros::init(argc, argv, "teleop_dummy");
    std::string ros_node_name = ros::this_node::getName();
    ros::NodeHandle n(ros_node_name);

    if( ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();

    // ------------------------------------- Clutches---------------------------
    ros::Subscriber sub_clutch_clutch = n.subscribe(
        "/dvrk/footpedals/clutch", 1, ClutchCallback);
    ROS_INFO("[SUBSCRIBERS] Subscribed to /dvrk/footpedals/clutch");

    ros::Subscriber sub_coag_clutch = n.subscribe(
        "/dvrk/footpedals/coag", 1, CoagCallback);
    ROS_INFO("[SUBSCRIBERS] Subscribed to /dvrk/footpedals/coag");

    // ------------------------------------- ARMS---------------------------
    std::string slave_names[2];
    std::string master_names[2];
    int num_arms = 0;

    if (n.getParam("slave_1_name", slave_names[0]))
        num_arms = 1;
    if (n.getParam("slave_2_name", slave_names[1]))
        num_arms = 2;

    if(num_arms==0)
        return 0;

    n.getParam("master_1_name", master_names[0]);
    n.getParam("master_2_name", master_names[1]);

    // ------------ MATERS POSE
    std::stringstream param_name;
    param_name << std::string("/dvrk/") << master_names[0]
               << "/position_cartesian_current";
    ros::Subscriber sub_master_1_current_pose =  n.subscribe(param_name.str(), 1, Master1PoseCurrentCallback);
    ROS_INFO("[SUBSCRIBERS] Subscribed to %s", param_name.str().c_str());

    param_name.str("");
    param_name << std::string("/dvrk/") << master_names[1]
               << "/position_cartesian_current";
    ros::Subscriber sub_master_2_current_pose =  n.subscribe(param_name.str(), 1, Master2PoseCurrentCallback);
    ROS_INFO("[SUBSCRIBERS] Subscribed to %s", param_name.str().c_str());

    // ------------ MATERS GET STATE
    param_name.str("");
    param_name << std::string("/dvrk/") << master_names[0]
               << "/robot_state";
    ros::Subscriber sub_master_1_state =  n.subscribe(param_name.str(), 1, Master1StateCallback);
    ROS_INFO("[SUBSCRIBERS] Subscribed to %s", param_name.str().c_str());

    param_name.str("");
    param_name << std::string("/dvrk/") << master_names[1]
               << "/robot_state";
    ros::Subscriber sub_master_2_state =  n.subscribe(param_name.str(), 1, Master2StateCallback);
    ROS_INFO("[SUBSCRIBERS] Subscribed to %s", param_name.str().c_str());


    // ------------ MATERS SET STATE
    param_name.str("");
    param_name << std::string("/dvrk/") << master_names[0]
               << "/set_robot_state";
    ros::Publisher pub_master_1_state = n.advertise<std_msgs::String>(param_name.str(), 2, true);
    ROS_INFO("[PUBLISHERS] Will publish to %s", param_name.str().c_str());

    param_name.str("");
    param_name << std::string("/dvrk/") << master_names[1]
               << "/set_robot_state";
    ros::Publisher pub_master_2_state = n.advertise<std_msgs::String>(param_name.str(), 2);
    ROS_INFO("[PUBLISHERS] Will publish to %s", param_name.str().c_str());


    // ------------ SLAVE PUBLISH POSE
    param_name.str("");
    param_name << std::string("/dvrk/") << slave_names[0]
               << "/position_cartesian_current";
    ros::Publisher pub_slave_1_pose = n.advertise<geometry_msgs::PoseStamped>(param_name.str(), 2);
    ROS_INFO("[PUBLISHERS] Will publish to %s", param_name.str().c_str());

    param_name.str("");
    param_name << std::string("/dvrk/") << slave_names[1]
               << "/position_cartesian_current";
    ros::Publisher pub_slave_2_pose = n.advertise<geometry_msgs::PoseStamped>(param_name.str(), 2);
    ROS_INFO("[PUBLISHERS] Will publish to %s", param_name.str().c_str());

    double scaling = 0.2;
    n.getParam("scaling", scaling);
    ROS_INFO(" Master to slave position scaling: %f", scaling);

    // spinning freq, publishing freq will be according to the freq of master poses received
    ros::Rate loop_rate(1000);


    ros::Rate loop_rate_slow(1);
    ros::spinOnce();

    loop_rate_slow.sleep();
    ros::spinOnce();

    std_msgs::String string_msg;
    if(master_state[0].data() != std::string("DVRK_READY")) {
        string_msg.data = "Home";
        pub_master_1_state.publish(string_msg);
        ROS_INFO( "Attempting to Home %s", master_names[0].c_str());
    } else
        ROS_INFO( "%s is alreade Homed.", master_names[0].c_str());

    // second master arm
    if(num_arms==2) {
        if (master_state[1].data() != std::string("DVRK_READY")) {
            string_msg.data = "Home";
            pub_master_2_state.publish(string_msg);
            ROS_INFO( "Attempting to Home %s", master_names[1].c_str());
        } else
            ROS_INFO( "%s is alreade Homed.", master_names[1].c_str());
    }

    KDL::Vector master_position_at_clutch_instance[2];
    KDL::Vector slave_position_at_clutch_instance[2];
    KDL::Frame slave_pose[2];

    while(ros::ok()){


        if(new_coag_msg || new_clutch_msg){
            new_coag_msg = false;

            if(coag_pressed && !clutch_pressed){
                string_msg.data = "DVRK_EFFORT_CARTESIAN";
                pub_master_1_state.publish(string_msg);
                master_position_at_clutch_instance[0] = master_pose[0].p;
                slave_position_at_clutch_instance[0] = slave_pose[0].p;
                if(num_arms==2){
                    pub_master_2_state.publish(string_msg);
                    master_position_at_clutch_instance[1] = master_pose[1].p;
                    slave_position_at_clutch_instance[1] = slave_pose[1].p;
                }
            }

            if(!coag_pressed){
                string_msg.data = "DVRK_POSITION_CARTESIAN";
                pub_master_1_state.publish(string_msg);
                if(num_arms==2)
                    pub_master_2_state.publish(string_msg);
            }
        }
        if(new_clutch_msg){
            new_clutch_msg = false;

            if(coag_pressed&& clutch_pressed){
                string_msg.data = "DVRK_CLUTCH";
                pub_master_1_state.publish(string_msg);
                if(num_arms==2)
                    pub_master_2_state.publish(string_msg);
            }
        }

        // incremental slave position

        if(new_master_pose[0]) {
            new_master_pose[0] = false;

            // if operator present increment the slave position
            if(coag_pressed && !clutch_pressed){
                slave_pose[0].p = slave_position_at_clutch_instance[0] +
                    scaling * (master_pose[0].p - master_position_at_clutch_instance[0]);
                slave_pose[0].M = master_pose[0].M;
            }

            //publish pose
            geometry_msgs::PoseStamped pose;
            tf::poseKDLToMsg( slave_pose[0], pose.pose);
            pub_slave_1_pose.publish(pose);
        }

        if(num_arms==2) {
            if (new_master_pose[1]) {
                new_master_pose[1] = false;

                // if operator present increment the slave position
                if (coag_pressed && !clutch_pressed) {
                    slave_pose[1].p = slave_position_at_clutch_instance[1] +
                        scaling * (master_pose[1].p
                            - master_position_at_clutch_instance[1]);
                    slave_pose[1].M = master_pose[1].M;
                }
                //publish pose
                geometry_msgs::PoseStamped pose;
                tf::poseKDLToMsg(slave_pose[1], pose.pose);
                pub_slave_2_pose.publish(pose);

            }
        }


        ros::spinOnce();
        loop_rate.sleep();


    }

}
