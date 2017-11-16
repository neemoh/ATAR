//
// Created by nima on 16/11/17.
//

#include <image_transport/image_transport.h>
#include <custom_conversions/Conversions.h>
#include "ManipulatorToWorldCalibration.h"

ManipulatorToWorldCalibration::ManipulatorToWorldCalibration(
        Manipulator *manip)
        :manipulator(manip)
{

    ros::NodeHandle n("~");

    it =  new image_transport::ImageTransport(n);

    std::string ns;
    n.getParam("cams_namespace", ns);

    std::string cam_name;
    n.getParam("cam_0_name", cam_name);

    ar_camera = new AugmentedCamera( it, cam_name, ns);

    // get the intrinsics
    ar_camera->GetIntrinsicMatrices(cam_matrix, cam_distortation);


    // putting the calibration point on the corners of the board squares
    // the parameter can be set directly, unless there is the global
    // /calibrations/board_params
    double calib_points_distance = 0.01;
    std::vector<float> board_params = std::vector<float>(5, 0.0);
    n.getParam("/calibrations/board_params", board_params);
    calib_points_distance = board_params[3];

    int num_calib_points = 6;
    std::vector<double> calib_point_center
            = {board_params[1]/2 * calib_points_distance
                    , board_params[2]/2 * calib_points_distance};
    // -------------------------------------------------------------------------
    // define calibration points in rows of 3 points centered around
    // calib_points_position_center

    int rows = 3;
    int cols = num_calib_points/rows + int((num_calib_points%rows)>0);
    for (uint i=0; i<num_calib_points; i++) {
        calib_points_in_world_frame.push_back(
                Eigen::Vector3d(
                        calib_point_center[0] +
                        (1-cols/2 + i/rows) * calib_points_distance
                        ,calib_point_center[1] +
                         (-1 + double(i%rows)) * calib_points_distance,
                        0.0) );
    }

}


// -----------------------------------------------------------------------------
//
void ManipulatorToWorldCalibration::PutDrawings(cv::Mat img
        , bool calibration_done, KDL::Frame manip_pose_loc) {

    std::string instructions;
    cv::Vec3d cam_tvec, cam_rvec;

    if(!calibration_done) {

        conversions::KDLFrameToRvectvec(ar_camera->GetWorldToCamTr(),
                                        cam_rvec,
                                        cam_tvec);
        // -------------------- draw  calibration points------------------------
        std::vector<cv::Point3d> calib_points_in_world_frame_cv;

        for (uint i = 0; i < calib_points_in_world_frame.size(); i++) {
            calib_points_in_world_frame_cv.emplace_back(
                    calib_points_in_world_frame[i][0],
                    calib_points_in_world_frame[i][1],
                    calib_points_in_world_frame[i][2]);
        }

        std::vector<cv::Point2d> calib_points_screen;
        cv::projectPoints(calib_points_in_world_frame_cv,
                          cam_rvec, cam_tvec,
                          cam_matrix, cam_distortation, calib_points_screen);

        for (uint i = 0; i < calib_points_in_world_frame.size(); i++) {
            cv::circle(img, calib_points_screen[i], 6, cv::Scalar(255, 255, 0),
                       2, CV_AA);
        }

        // draw the current target with a different colou
        cv::circle(img, calib_points_screen[calib_points_in_arm_frame.size()],
                   6, cv::Scalar(0, 0, 255), 2, CV_AA);
        // --------------------------------------------------------------------

        instructions = "Point at the red point with the tooltip, then press "
                "'c'. Press 'Esc' to exit";


        std::stringstream tool_pos_msg;
        tool_pos_msg <<"Tool point at x: "<< manip_pose_loc.p[0]
                     << ", y: "<< manip_pose_loc.p[1]
                     << ", z:" << manip_pose_loc.p[2];

        cv::putText(img, tool_pos_msg.str(), cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 0), 2);
    }
    else { // if(!calibration_done)
        instructions = "Calibration finished. Press 'Esc' to exit";

        // ---------------------- draw the tool tip frame ----------------------
        KDL::Frame arm_pose_world_frame = world_to_arm_tr.Inverse() *
                                          manip_pose_loc;

        DrawCoordinateFrameInTaskSpace( img, arm_pose_world_frame,
                                        cam_rvec, cam_tvec, 0.01);

    }

    // draw the instructions
    cv::putText(img, instructions, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 0), 2);

    // draw the coordinate frame of the board
    DrawCoordinateFrameInTaskSpace(img, KDL::Frame(),
                                   cam_rvec, cam_tvec, 0.02);

}


// -----------------------------------------------------------------------------
//
void ManipulatorToWorldCalibration::DrawCoordinateFrameInTaskSpace(
        const cv::InputOutputArray &image,
        const KDL::Frame frame,
        const cv::Vec3d &rvec, const cv::Vec3d &tvec,
        float length){

    CV_Assert(image.getMat().total() != 0 &&
              (image.getMat().channels() == 1 || image.getMat().channels() == 3));
    CV_Assert(length > 0);

    // project axis points
    KDL::Vector x_axis = frame.p +  length * frame.M.UnitX();
    KDL::Vector y_axis = frame.p +  length * frame.M.UnitY();
    KDL::Vector z_axis = frame.p +  length * frame.M.UnitZ();

    std::vector<cv::Point3f> axisPoints {
            cv::Point3f( (float)frame.p[0],  (float)frame.p[1],  (float)frame.p[2]),
            cv::Point3f( (float)x_axis[0],  (float)x_axis[1],   (float)x_axis[2]),
            cv::Point3f( (float)y_axis[0],  (float)y_axis[1],   (float)y_axis[2]),
            cv::Point3f( (float)z_axis[0],  (float)z_axis[1],   (float)z_axis[2]),
    };


    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cam_matrix,
                      cam_distortation, imagePoints);

    // draw axis lines
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 200), 2, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 200, 0), 2, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(200, 0, 0), 2, CV_AA);
}

ManipulatorToWorldCalibration::~ManipulatorToWorldCalibration() {

    delete ar_camera;
    delete it;
}

bool ManipulatorToWorldCalibration::DoCalibration(KDL::Frame & result) {

    bool exit = false;
    bool calibration_done = false;
    cv::Mat image;
    ros::Rate rate(30);

    while(ros::ok() && !exit) {

        cv::cvtColor(ar_camera->GetImage(), image, cv::COLOR_RGB2BGR);

        // get the manipulator pose
        KDL::Frame manip_pose_loc;
        manipulator->GetPoseLocal(manip_pose_loc);

        PutDrawings(image, calibration_done, manip_pose_loc);

        cv::imshow("Manipulator to world calibration", image);


        auto key = (char) cv::waitKey(1);

        if (key == 27)
            exit = true;
        if (key == 'c' && !calibration_done) {

            // save the position of the end effector
            calib_points_in_arm_frame.emplace_back(
                    manip_pose_loc.p[0],
                    manip_pose_loc.p[1],
                    manip_pose_loc.p[2]);

            if (calib_points_in_arm_frame.size() ==
                calib_points_in_world_frame.size()) {

                world_to_arm_tr = CalculateTransformation
                        (calib_points_in_world_frame,
                         calib_points_in_arm_frame);
                calibration_done = true;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    if(calibration_done){
        result = world_to_arm_tr;
        cvDestroyWindow("Manipulator to world calibration");
        return true;
    }

    cvDestroyWindow("Manipulator to world calibration");
    return false;
}


// -----------------------------------------------------------------------------
//
KDL::Frame ManipulatorToWorldCalibration::CalculateTransformation(
        std::vector<Eigen::Vector3d> points_in_frame_1,
        std::vector<Eigen::Vector3d> points_in_frame_2) {

    if( points_in_frame_1.size()!= points_in_frame_2.size())
        throw std::runtime_error("Num of points don't match.");

    // points_in_frame_2_mat: measured=arm
    Eigen::Matrix<double, 3, 6> points_in_frame_2_mat;
    Eigen::Matrix<double, 3, 6> points_in_frame_1_mat;

    for (uint i = 0; i < points_in_frame_1.size(); i++) {
        points_in_frame_1_mat.col(i) = points_in_frame_1[i];
        points_in_frame_2_mat.col(i) = points_in_frame_2[i];
    }

    auto temp = Eigen::umeyama(points_in_frame_1_mat, points_in_frame_2_mat, 0);

    KDL::Frame task_frame_to_robot_frame;
    // converting Eigen::Matrix into KDL::Frame
    cv::Matx33d rot_mat;
    for (uint i = 0; i < 3; i++) {
        rot_mat(i, 0) = temp(i, 0);
        rot_mat(i, 1) = temp(i, 1);
        rot_mat(i, 2) = temp(i, 2);
        task_frame_to_robot_frame.p[i] = temp(i, 3);
    }
    conversions::Matx33dToKdlRot(rot_mat, task_frame_to_robot_frame.M);
    ROS_INFO_STREAM(std::string("-- World To PSM Transformation Calculated: \n")
                            << temp << std::endl);

    return task_frame_to_robot_frame;
}