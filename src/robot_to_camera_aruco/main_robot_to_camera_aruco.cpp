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
enum class CalibProgress { Ready, Method1Calibrating, Method2Calibrating, Finished };       // Added MidStep

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "robot_to_camera_aruco");

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
        char key = (char)cv::waitKey(1);
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
            cvSetWindowProperty("PSM to board Calib", CV_WND_PROP_FULLSCREEN,
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
                    r.SetTaskFrameToRobotFrameParam();

                }
            }
        }


        if (progress == CalibProgress::Method2Calibrating){
            // computing camera_to_robot_tranform using chain rule (rTk=rTb*bTk with r=robot, b=board, k=camera)
            // auto camera_to_robot = r.task_frame_to_robot_frame * r.board_to_cam_frame.Inverse();
            // cv::Vec3d rvec, tvec;
            // conversions::kdlFrameToRvectvec(camera_to_robot, rvec, tvec);

            r.Calib2DrawTarget(instructions, back_buffer);

            if (key == 's') {

                r.Calib2SaveCalibrationPoint();

                // was adding the point enough to complete the calibration?
                if (r.IsCalibrated()) {

                    progress = CalibProgress::Finished;

                    // ROS_INFO_STREAM("  -> chain rule: \n" << rvec << std::endl << tvec << std::endl);
                    // drawings.update_camera_to_robot(camera_to_robot);
                    // ROS_INFO_STREAM("  -> quaternion matching: \n" << camera_to_robot << std::endl);

                }
            }
        }

        if (progress == CalibProgress::Finished) {
            instructions = "Calibration finished. Press 'Esc' to exit";

            // take the tool tip to task space
            r.tool_pose_in_task_frame = r.task_frame_to_robot_frame.Inverse() *
                    r.tool_pose_in_robot_frame;

            DrawingsCV::DrawCoordinateFrameInTaskSpace(back_buffer, r.camera_intrinsics,
                                                       r.tool_pose_in_task_frame,
                                                       r.task_frame_to_cam_rvec,
                                                       r.task_frame_to_cam_tvec, 0.01);


        }

//        drawings.showTransientNotification(back_buffer);
        // draw the coordinate frame of the board
        DrawingsCV::DrawCoordinateFrameInTaskSpace(back_buffer, r.camera_intrinsics,
                                                   KDL::Frame(),
                                                   r.task_frame_to_cam_rvec,
                                                   r.task_frame_to_cam_tvec, 0.01);
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
