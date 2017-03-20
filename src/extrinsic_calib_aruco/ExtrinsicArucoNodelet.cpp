//
// Created by nima on 27/02/17.
//

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "ExtrinsicAruco.hpp"
#include <opencv2/highgui.hpp>
#include <tf_conversions/tf_kdl.h>
#include "utils/Conversions.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <memory>
#include <cv_bridge/cv_bridge.h>

namespace teleop_vision {

    class ExtrinsicArucoNodelet : public nodelet::Nodelet {

        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber sub_;
        ros::Publisher pub_board_to_cam_pose_;
        //in-class initialization
        ArucoBoard board;
        BoardDetector *board_detector;
        cv::Mat image_msg;
        CameraIntrinsics camera_intrinsics;

    public:
        ExtrinsicArucoNodelet();

    private:
        virtual void onInit();

        void ImageCallback(
                const sensor_msgs::ImageConstPtr &msg);

    private:

        void GetROSParameterValues(ros::NodeHandle &nh);

        void ReadCameraParameters(std::string file_path);


    private:


    };

    ExtrinsicArucoNodelet::ExtrinsicArucoNodelet() {

    }

    void ExtrinsicArucoNodelet::onInit() {
        ros::NodeHandle &nh = getNodeHandle();
        ros::NodeHandle &private_nh = getPrivateNodeHandle();
        GetROSParameterValues(private_nh);

        board_detector = new BoardDetector(board, camera_intrinsics, 1);

    }


    void ExtrinsicArucoNodelet::ImageCallback(
            const sensor_msgs::ImageConstPtr &msg) {

        image_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;
        KDL::Frame board_to_cam_frame;

        //      char key = (char)cv::waitKey(1);
//        if (key == 27) // Esc
//            ros::shutdown();

        // DETECT BOARD
        if(!image_msg.empty()) {
            board_detector->DetectBoardAndDrawAxis(image_msg);
            conversions::RvecTvecToKDLFrame(board_detector->rvec,
                                            board_detector->tvec,
                                            board_to_cam_frame);


            if (board_detector->Detected()) {

                geometry_msgs::PoseStamped board_to_cam_msg;
                // convert pixel to meters
                //cam_to_robot.p = cam_to_robot.p / drawings.m_to_px;
                tf::poseKDLToMsg(board_to_cam_frame, board_to_cam_msg.pose);

                pub_board_to_cam_pose_.publish(board_to_cam_msg);
            }

          //  if (board.draw_axes)
               // cv::imshow("Aruco extrinsic", image);

        }

    }
//
//    std::string ros_node_name("extrinsic_aruco");
//
//    ros::init(argc, argv, ros_node_name);
//    ArucoExtrinsic ae(ros_node_name);
//
//    KDL::Frame board_to_cam_frame;
//
//    //-----------------------------------------------------------------------------------
//    // Construct the board detector object
//    //-----------------------------------------------------------------------------------
//    BoardDetector board_detector(ae.board, ae.camera_intrinsics, 2);
//
//    // Create the window in which to render the video feed
//    // cvNamedWindow("Aruco extrinsic", CV_WINDOW_NORMAL);
//    //    cvSetWindowProperty("teleop", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
//
//    // This is the image that will be rendered at every loop. We'll be drawing the hints on it.
//    cv::Mat back_buffer;
//
//    ros::Rate loop_rate(ae.ros_freq);


    void ExtrinsicArucoNodelet::GetROSParameterValues(ros::NodeHandle &n) {
        bool all_required_params_found = true;

        n.param<bool>("draw_axes", board.draw_axes, false);
        // load the intrinsic calibration file
        std::string cam_intrinsic_calibration_file_path;
        if (n.getParam("cam_intrinsic_calibration_file_path", cam_intrinsic_calibration_file_path)) {
            ReadCameraParameters(cam_intrinsic_calibration_file_path);
        } else {
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("cam_intrinsic_calibration_file_path").c_str());
        }




        std::string image_transport_namespace;
        if (n.getParam("image_transport_namespace", image_transport_namespace)) {

            // if the topic name is found, check if something is being published on it
            ROS_INFO("Will read camera images from transport '%s'", image_transport_namespace.c_str());

        } else {
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("image_transport_namespace").c_str());
            all_required_params_found = false;
        }

        it_.reset(new image_transport::ImageTransport(n));
        sub_ = it_->subscribe(image_transport_namespace, 1,
                              &ExtrinsicArucoNodelet::ImageCallback, this);


        // Load the description of the aruco board from the parameters
        if (!n.getParam("aruco_board_w", board.Width)){
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_board_w").c_str());
            all_required_params_found = false;
        }
        if (!n.getParam("aruco_board_h", board.Height)){
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_board_h").c_str());
            all_required_params_found = false;
        }
        if (!n.getParam("aruco_marker_length_in_meters", board.MarkerLength)){
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_marker_length_in_meters").c_str());
            all_required_params_found = false;
        }
        if (!n.getParam("aruco_marker_separation_in_meters", board.MarkerSeparation)){
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_marker_separation_in_meters").c_str());
            all_required_params_found = false;
        }
        if (!n.getParam("aruco_dictionary_id", board.DictionaryID)){
            ROS_ERROR("Parameter '%s' is required.", n.resolveName("aruco_dictionary_id").c_str());
            all_required_params_found = false;
        }


        pub_board_to_cam_pose_ = n.advertise<geometry_msgs::PoseStamped>(
                "board_to_camera", 1, 0);
        ROS_INFO("Will publish board to camera pose as '%s'",
                 n.resolveName("board_to_camera").c_str());

        if (!all_required_params_found)
            throw std::runtime_error("ERROR: some required topics are not set");


    }

    void ExtrinsicArucoNodelet::ReadCameraParameters(std::string file_path) {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        ROS_INFO("Reading camera intrinsic data from: '%s'" , file_path.c_str());

        if (!fs.isOpened())
            throw std::runtime_error("Unable to read the camera parameters file.");

        fs["camera_matrix"] >> camera_intrinsics.camMatrix;
        fs["distortion_coefficients"] >> camera_intrinsics.distCoeffs;

        // check if we got osomething
        if(camera_intrinsics.distCoeffs.empty()){
            ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
            throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
        }
        if(camera_intrinsics.camMatrix.empty()){
            ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
            throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
        }
    }


} //namespace teleop_vision

#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(teleop_vision::ExtrinsicArucoNodelet, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS(teleop_vision, ExtrinsicArucoNodelet,
                        teleop_vision::ExtrinsicArucoNodelet, nodelet::Nodelet);