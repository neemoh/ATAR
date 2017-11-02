//
// Created by nima on 21/05/17.
//



#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <custom_conversions/Conversions.h>
#include <pwd.h>
#include "src/intrinsic_calib/IntrinsicCalibrationCharuco.h"

using namespace std;
using namespace cv;


cv::Mat image_msg;
bool new_image = false;

void ImgCallback(const sensor_msgs::ImageConstPtr &msg);

cv::Mat &Image(ros::Duration timeout);

static bool readCameraParameters(string filename, Mat &camMatrix,
                                 Mat &distCoeffs);

bool DetectCharucoBoardPose(cv::Mat &image,
                            Ptr<aruco::CharucoBoard> charucoboard,
                            Ptr<aruco::Dictionary> dictionary,
                            const Mat &camMatrix, const Mat &distCoeffs,
                            Vec3d &rvec, Vec3d &tvec);

bool EstimatePoseCharucoBoard(InputArray _charucoCorners,
                              InputArray _charucoIds,
                              const Ptr<aruco::CharucoBoard> &_board,
                              InputArray _cameraMatrix, InputArray _distCoeffs,
                              OutputArray _rvec, OutputArray _tvec);



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int main(int argc, char *argv[]) {

    ros::init(argc, argv, "extrinsic_charuco");
    std::string ros_node_name = ros::this_node::getName();

    ros::NodeHandle n(ros_node_name);
    ros::Rate loop_rate = ros::Rate(200);


    std::stringstream instruction_msg;

    //----------- Read camera parameters
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;

    std::stringstream cam_intrinsics_path;
    std::string cam_name;
    n.param<std::string>("camera_name", cam_name, "camera");
    cam_intrinsics_path << std::string(home_dir) << "/.ros/camera_info/"
                        << cam_name << "_intrinsics.yaml";
    Mat cam_matrix, dist_coeffs;
    if(!readCameraParameters(cam_intrinsics_path.str(), cam_matrix, dist_coeffs))
        instruction_msg << std::string(
                "Did not find the intrinsic calibration data in ") <<
                        cam_intrinsics_path.str() <<
                        " Press C to perform intrinsic calibration.";

    //----------- Read boardparameters
    // board_params comprises:
    // [dictionary_id, board_w, board_h,
    // square_length_in_meters, marker_length_in_meters]
    std::vector<float> board_params = std::vector<float>(5, 0.0);
    if(!n.getParam("board_params", board_params))
    {
        if(!n.getParam("/calibrations/board_params", board_params))
            ROS_ERROR("Ros parameter board_param is required. board_param="
                              "[dictionary_id, board_w, board_h, "
                              "square_length_in_meters, marker_length_in_meters]");
    }


    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME
                                                   (board_params[0]));
    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(board_params[1], board_params[2],
                                        board_params[3], board_params[4],
                                        dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();
    float axisLength = 0.5f * ((float)min(board_params[1], board_params[2]) *
                               (board_params[3]));
    //-----------

    //----------- ROS pub and sub
    // advertise publishers
    std::stringstream pose_topic_name;
    pose_topic_name << std::string("/")
                    << cam_name << "/world_to_camera_transform";

    ros::Publisher publisher_pose =n.advertise<geometry_msgs::PoseStamped>
            (pose_topic_name.str(), 1, 0);
    ROS_INFO("Publishing board to camera pose on '%s'",
             pose_topic_name.str().c_str());

    std::string img_topic = "/"+cam_name+ "/image_raw";
    std::string cams_ns;
    if(n.getParam("cams_namespace", cams_ns) && cams_ns!="")
        img_topic = "/"+cams_ns+"/"+cam_name+ "/image_raw";


    // if the topic name is found, check if something is being published on it
    if (!ros::topic::waitForMessage<sensor_msgs::Image>(
            img_topic, ros::Duration(5)))
        ROS_WARN("Topic '%s' is not publishing.", img_topic.c_str());
    image_transport::ImageTransport it = image_transport::ImageTransport(n);
    // register image transport subscriber
    image_transport::Subscriber sub = it.subscribe(
            img_topic, 1, &ImgCallback);
    //-----------

    bool show_image;
    n.param<bool>("show_image", show_image, true);

    //----------- Intrinsic Calibration
    IntrinsicCalibrationCharuco * IC_ptr;
    //-----------

    while(ros::ok() ){
        if(new_image) {

            new_image = false;
            Mat imageCopy, image;
            image = Image(ros::Duration(1));
            image.copyTo(imageCopy);

            Vec3d rvec, tvec;
            bool valid_pose = DetectCharucoBoardPose(image, charucoboard,
                                                     dictionary, cam_matrix,
                                                     dist_coeffs, rvec, tvec);

            if (valid_pose) {
                // draw the axes
                aruco::drawAxis(imageCopy, cam_matrix, dist_coeffs, rvec, tvec,
                                axisLength);

                // publish the pose
                geometry_msgs::PoseStamped board_to_cam_msg;
                conversions::RvecTvecToPoseMsg(rvec, tvec, board_to_cam_msg.pose);
                publisher_pose.publish(board_to_cam_msg);

            }

            if(show_image) {
                cv::putText(
                        imageCopy, instruction_msg.str(),
                        cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 0, 0), 2
                );
                imshow(("extrinsic charuco " + cam_name).c_str(), imageCopy);
            }

            char key = (char) waitKey(1);
            if (key == 27) break;

            else if(key == 'c'){

                // get home directory

                IC_ptr = new IntrinsicCalibrationCharuco(img_topic, board_params);
                double intrinsic_calib_err;
                if(IC_ptr->DoCalibration(cam_intrinsics_path.str(),
                                         intrinsic_calib_err,
                                         cam_matrix,
                                         dist_coeffs))
                    instruction_msg.str("");
                else
                    instruction_msg.str("Intrinsic Calibration failed. Please"
                                                " repeat.");
                delete IC_ptr;
            }

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


//------------------------------------------------------------------------------
void ImgCallback(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }
}

//------------------------------------------------------------------------------
cv::Mat &Image(ros::Duration timeout) {
    ros::Rate loop_rate(1);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while (image_msg.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new Image received.");
        }
    }

    return image_msg;
}

//------------------------------------------------------------------------------
static bool readCameraParameters(string file_path,
                                 Mat &camera_matrix, Mat &camera_distortion) {
    FileStorage fs(file_path, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> camera_distortion;
    // check if we got something
    if(camera_matrix.empty()){
        ROS_WARN("distortion_coefficients not found in '%s' ", file_path.c_str());
        return false;

    }
    if(camera_distortion.empty()){
        ROS_WARN("camera_matrix not found in '%s' ", file_path.c_str());
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool DetectCharucoBoardPose(cv::Mat &image,
                            Ptr<aruco::CharucoBoard> charucoboard,
                            Ptr<aruco::Dictionary> dictionary,
                            const Mat &camMatrix, const Mat &distCoeffs,
                            Vec3d &rvec, Vec3d &tvec
){

    vector<int> markerIds, charucoIds;
    vector<vector<Point2f> > markerCorners, rejectedMarkers;
    vector<Point2f> charucoCorners;


    Ptr<aruco::DetectorParameters> detector_params =
            aruco::DetectorParameters::create();
    detector_params->doCornerRefinement = true;

    // detect markers
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds,
                         detector_params,
                         rejectedMarkers);

//    // refind strategy to detect more markers
//    if (refindStrategy)
//        aruco::refineDetectedMarkers(image, board, markerCorners,
//                                     markerIds, rejectedMarkers,
//                                     camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (markerIds.size() > 0)
        interpolatedCorners =
                aruco::interpolateCornersCharuco(markerCorners,
                                                 markerIds, image,
                                                 charucoboard,
                                                 charucoCorners,
                                                 charucoIds, camMatrix,
                                                 distCoeffs);

    // estimate charuco board pose
    bool validPose = false;
    if (camMatrix.total() != 0)
        validPose = EstimatePoseCharucoBoard(charucoCorners,
                                             charucoIds,
                                             charucoboard,
                                             camMatrix,
                                             distCoeffs, rvec,
                                             tvec);


//    double currentTime =
//            ((double) getTickCount() - tick) / getTickFrequency();
//    totalTime += currentTime;
//    totalIterations++;
//    if (totalIterations % 30 == 0) {
//        cout << "Detection Time = " << currentTime * 1000 << " ms "
//             << "(Mean = " << 1000 * totalTime / double(totalIterations)
//             << " ms)" << " validPose: " << validPose << " tvec: "
//             << tvec[0] << " " << tvec[1] << " " << tvec[2] << endl;
//    }

//
//    cv::Mat imageCopy;
//    // draw results
//    image.copyTo(imageCopy);
//    if (markerIds.size() > 0) {
//        aruco::drawDetectedMarkers(imageCopy, markerCorners);
//    }
//
//
//    if (interpolatedCorners > 0) {
//        Scalar color;
//        color = Scalar(255, 0, 255);
//        aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners,
//                                          charucoIds, color);
//    }
//
//    imshow("out", imageCopy);

    return validPose;

}


//------------------------------------------------------------------------------
bool EstimatePoseCharucoBoard(InputArray _charucoCorners, InputArray _charucoIds,
                              const Ptr<aruco::CharucoBoard> &_board,
                              InputArray _cameraMatrix, InputArray _distCoeffs,
                              OutputArray _rvec, OutputArray _tvec) {

    CV_Assert((_charucoCorners.getMat().total() ==
               _charucoIds.getMat().total()));

    // need, at least, 4 corners
    if(_charucoIds.getMat().total() < 4) return false;

    vector< Point3f > objPoints;
    objPoints.reserve(_charucoIds.getMat().total());
    for(unsigned int i = 0; i < _charucoIds.getMat().total(); i++) {
        int currId = _charucoIds.getMat().at< int >(i);
        CV_Assert(currId >= 0 && currId < (int)_board->chessboardCorners.size());
        objPoints.push_back(_board->chessboardCorners[currId]);
    }

    // points need to be in different lines, check if detected points are enough
    //if(!_arePointsEnoughForPoseEstimation(objPoints)) return false;
    solvePnP(objPoints, _charucoCorners,
             _cameraMatrix, _distCoeffs, _rvec, _tvec);

    return true;
}
