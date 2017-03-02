/*
 * main_aruco_extrinsic.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: nearlab
 */

#include <ExtrinsicAruco.hpp>
#include <opencv2/highgui.hpp>
#include <tf_conversions/tf_kdl.h>
#include "utils/Conversions.hpp"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char *argv[]) {

    std::string ros_node_name("extrinsic_aruco");

    ros::init(argc, argv, ros_node_name);
    ArucoExtrinsic ae(ros_node_name);

    KDL::Frame board_to_cam_frame;

    //-----------------------------------------------------------------------------------
    // Construct the board detector object
    //-----------------------------------------------------------------------------------
    BoardDetector board_detector(ae.Board, ae.Camera, 2);

    // Create the window in which to render the video feed
    // cvNamedWindow("Aruco extrinsic", CV_WINDOW_NORMAL);
    //    cvSetWindowProperty("teleop", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    // This is the image that will be rendered at every loop. We'll be drawing the hints on it.
    cv::Mat back_buffer;

    ros::Rate loop_rate(ae.ros_freq);

    while (ros::ok()) {

        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();

    	// DETECT BOARD
        board_detector.DetectBoardAndDrawAxis(ae.Image());
        conversions::RvecTvecToKDLFrame(board_detector.rvec,
                                        board_detector.tvec, board_to_cam_frame);

    	// draw results
    	board_detector.image.copyTo(back_buffer);

    	if (board_detector.Ready()) {

    		geometry_msgs::PoseStamped board_to_cam_msg;
    		// convert pixel to meters
    		//cam_to_robot.p = cam_to_robot.p / drawings.m_to_px;
    		tf::poseKDLToMsg(board_to_cam_frame, board_to_cam_msg.pose);

    		ae.pub_board_to_cam_pose.publish(board_to_cam_msg);
    	}

        if(ae.show_image)
            cv::imshow("Aruco extrinsic", back_buffer);

    	loop_rate.sleep();
    	ros::spinOnce();

    }

    ROS_INFO("Ending Session...\n");
    ros::shutdown();

    return 0;
}


// operator overload to print out vectors
std::ostream &operator<<(std::ostream &out, const std::vector<double> &vect) {
	for (unsigned int iter = 0; iter < vect.size(); ++iter) {
		out << "[" << iter << "]: " << vect.at(iter) << "\t";
	}

    return out;
}
