/*
 * main_robot_to_camera_aruco.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: nearlab
 */

#include "RobotToCameraAruco.hpp"
#include "utils/Colors.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Enum used to track the progress of the calibration procedure.
 */
enum class CalibProgress { Ready, Method1Calibrating, Method2Calibrating, Finished };

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "arm_to_world_calibration");


    // get name in case asigned externally
    RobotToCameraAruco r(ros::this_node::getName());

    // Create the window in which to render the video feed
    cvNamedWindow(ros::this_node::getName().c_str(), CV_WINDOW_NORMAL);

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
        //            if (boardDetector.Detected()) {
        //		progress = CalibProgress::Detected;
        //            }
        //            continue;
        //        }

        instructions = "Press 1 or 2 to begin calibration.";
        char key = (char) cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();

        if (key == '1') { // Start calibration
            r.Reset();
            progress = CalibProgress::Method1Calibrating;
        }
        if (key == '2') { // Start calibration
            r.Reset();
            progress = CalibProgress::Method2Calibrating;
        }
        if (key == 'f') { //full screen
            cvSetWindowProperty(
                    "PSM to board Calib", CV_WND_PROP_FULLSCREEN,
                    CV_WINDOW_FULLSCREEN);
        }

        if (progress == CalibProgress::Method1Calibrating) {

            //			r.DrawToolTarget(instructions,
            // back_buffer);
            r.Calib1DrawCalibrationAxis(instructions, back_buffer);

            if (key == ' ') {
                // save the position of the end effector
                r.Calib1SaveCalibrationPoint();

                // was adding the point enough to complete the calibration?
                if (r.IsCalibrated()) {
                    progress = CalibProgress::Finished;

                    // set the calibration data
                    r.SetROSParameters();
                }
            }
        }


        if (progress == CalibProgress::Method2Calibrating) {


            r.Calib2DrawTarget(instructions, back_buffer);

            if (key == 's') {

                r.Calib2SaveCalibrationPoint();

                // was adding the point enough to complete the calibration?
                if (r.IsCalibrated()) {
                    progress = CalibProgress::Finished;

                    // set the calibration data
                    r.SetROSParameters();
                }
            }
        }

        if (progress == CalibProgress::Finished) {
            instructions = "Calibration finished. Press 'Esc' to exit";
        }

        if (progress == CalibProgress::Finished || r.task_frame_to_robot_frame_param_present) {

            // take the tool tip to task space
            r.tool_pose_in_task_frame = r.task_frame_to_robot_frame.Inverse() *
                                        r.tool_pose_in_robot_frame;

            DrawingsCV::DrawCoordinateFrameInTaskSpace(
                    back_buffer, r.camera_intrinsics,
                    r.tool_pose_in_task_frame,
                    r.task_frame_to_cam_rvec[r.cam_id],
                    r.task_frame_to_cam_tvec[r.cam_id], 0.01);
        }

        // draw the coordinate frame of the board
        DrawingsCV::DrawCoordinateFrameInTaskSpace(
                back_buffer, r.camera_intrinsics,
                KDL::Frame(),
                r.task_frame_to_cam_rvec[r.cam_id],
                r.task_frame_to_cam_tvec[r.cam_id], 0.01);
        cv::Point textOrigin(10, 20);
        auto text_color = (!r.IsCalibrated()) ? Colors::Red : Colors::Green;
        cv::putText(back_buffer, instructions, textOrigin, 1, 1, text_color);

        cv::imshow(ros::this_node::getName().c_str(), back_buffer);

        ros::spinOnce();
        loop_rate.sleep();



    }
    ROS_INFO("Ending Session...\n");
    ros::shutdown();

    return 0;
}
