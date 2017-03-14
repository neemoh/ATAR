/*
 * main_robot_to_camera_aruco.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: nearlab
 */

#include "RobotToCameraAruco.hpp"
#include "utils/Colors.hpp"
#include "utils/Drawings.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Enum used to track the progress of the calibration procedure.
 */
enum class CalibProgress { Idle, Ready, Calibrating, Finished };

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "robot_to_camera_aruco");

    // get name in case asigned externally


    RobotToCameraAruco r(ros::this_node::getName());

//     //	Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 1);
//     std::vector<Eigen::Vector3d> A = {Eigen::Vector3d(0.0100, -0.0010, 0.100),
//                                       Eigen::Vector3d(0.0200, 0.0010, 0.101),
//                                       Eigen::Vector3d(0.0400, 0.0000, 0.100),
//                                       Eigen::Vector3d(0.000, 0.0200, 0.1010),
//                                       Eigen::Vector3d(-0.001, 0.040, 0.1010),
//                                       Eigen::Vector3d(0.001, 0.0500, 0.1000)};
//
//     cv::Matx33d rotm_;
//     cv::Vec3d br_tvec_;
//
//     r.CalculateTransformationN(A, rotm_, br_tvec_);
//
//     std::cout << "\n rotm_ \n" << rotm_ << std::endl;
//     std::cout << "\n br_tvec_ \n" << br_tvec_ << std::endl;

    //-----------------------------------------------------------------------------------
    // Construct the drawing object
    //-----------------------------------------------------------------------------------
    Drawings drawings(
            r.camera_intrinsics.camMatrix, r.camera_intrinsics.distCoeffs,
            r.board_to_robot_frame,
            //        r.board.MarkerLength_px / r.board.MarkerLength);
            1);

    //	drawings.setSquare(Point3d(500,350,0),  Point2d(500,100));
    //	drawings.createEllipse(525,350,220,150,400);

    // Create the window in which to render the video feed
    cvNamedWindow("PSM to board Calib",CV_WINDOW_NORMAL);


    // This is the image that will be rendered at every loop. We'll be drawing
    // the hints on it.
    cv::Mat back_buffer;
    cv::String instructions = "Waiting ...";

    ros::Rate loop_rate(r.ros_freq);

    CalibProgress progress = CalibProgress::Ready;

    while (ros::ok()) {

        back_buffer = r.Image(ros::Duration(1));

        // todo add a way of checking if the board is seen
        //        if (progress == CalibProgress::Idle) {
        //            if (boardDetector.Ready()) {
        //		progress = CalibProgress::Ready;
        //            }
        //            continue;
        //        }

        instructions = "Press k to begin calibration.";
        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();

        if (key == 'k') { // Start calibration
            r.Reset();
            progress = CalibProgress::Calibrating;
        }
        if (key == 'f') { //full screen
            cvSetWindowProperty("PSM to board Calib", CV_WND_PROP_FULLSCREEN,
                                CV_WINDOW_FULLSCREEN);
        }
        if (progress == CalibProgress::Calibrating) {

            //			r.DrawToolTarget(instructions,
            // back_buffer);
            r.DrawCalibrationAxis(instructions, back_buffer);

            if (key == ' ') {
                // save the position of the end effector
                r.SaveCalibrationPoint(r.robot_pose.pose.position);

                // was adding the point enough to complete the calibration?
                if (r.IsCalibrated()) {
                    progress = CalibProgress::Finished;

                    // get the calibration data
                    r.GetTr();
                    drawings.update_board_to_robot_frame(
                            r.board_to_robot_frame);
                }
            }
        }

        if (progress == CalibProgress::Finished) {
            instructions = "Calibration finished. Press 'Esc' to exit";

            //            if (boardDetector.Ready()) {
            drawings.update_cam_2_board_ref(r.board_to_cam_rvec,
                                            r.board_to_cam_tvec);

            // draw the tool tip point
            drawings.drawToolTip(back_buffer, r.robot_pose.pose.position,
                                 Colors::Redish);

            //-----------------------------------------------------------------------------------
            // publish camera to robot pose
            //-----------------------------------------------------------------------------------
            //                KDL::Frame cam_to_robot = board_to_psm_frame *
            //                board_to_cam_frame.Inverse();
            //
            //                geometry_msgs::Pose cr_pose_msg;
            //                // convert pixel to meters
            //                //cam_to_robot.p = cam_to_robot.p /
            //                drawings.m_to_px;
            //                tf::poseKDLToMsg(cam_to_robot, cr_pose_msg);
            //
            //                r.pub_board_to_robot_pose.publish(cr_pose_msg);
            //                r.pub_board_to_robot_pose.publish(board_to_psm_pose_msg);
            //            }
        }

        drawings.showTransientNotification(back_buffer);

        cv::Point textOrigin(10, 20);
        auto text_color = (!r.IsCalibrated()) ? Colors::Red : Colors::Green;
        cv::putText(back_buffer, instructions, textOrigin, 1, 1, text_color);

        cv::imshow("PSM to board Calib", back_buffer);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Ending Session...\n");
    ros::shutdown();

    return 0;
}
