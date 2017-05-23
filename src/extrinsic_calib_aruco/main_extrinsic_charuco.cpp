//
// Created by nima on 21/05/17.
//



#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <opencv-3.2.0-dev/opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <include/utils/Conversions.hpp>

using namespace std;
using namespace cv;


cv::Mat image_msg;
bool new_image = false;

void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

cv::Mat &Image(ros::Duration timeout);

static bool readCameraParameters(string filename, Mat &camMatrix,
                                 Mat &distCoeffs);

bool DetectCharucoBoardPose(cv::Mat &image,
                            Ptr<aruco::CharucoBoard> charucoboard,
                            Ptr<aruco::Dictionary> dictionary,
                            const Mat &camMatrix, const Mat &distCoeffs,
                            Vec3d &rvec, Vec3d &tvec);

std::string GetCameraTopicName(ros::NodeHandle &n);

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


    //----------- Read camera parameters
    std::string cam_intrinsic_file;
    if (!n.getParam("cam_intrinsic_file", cam_intrinsic_file))
        ROS_ERROR("Parameter '%s' is required.",
                  n.resolveName("cam_intrinsic_file").c_str());

    Mat camMatrix, distCoeffs;
    readCameraParameters(cam_intrinsic_file, camMatrix, distCoeffs);
    //-----------

    //----------- Read boardparameters
    std::vector<float> board_params = std::vector<float>(5, 0.0);
    if(!n.getParam("board_params", board_params))
        ROS_ERROR("Ros parameter board_param is required. board_param="
                          "[dictionary_id, board_w, board_h, "
                          "square_length_in_meters, marker_length_in_meters]");

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
    std::string world_to_cam_pose_topic_name;
    if (!n.getParam("world_to_cam_pose_topic_name", world_to_cam_pose_topic_name))
        world_to_cam_pose_topic_name = "world_to_camera";

    ros::Publisher pub_board_to_cam_pose =
            n.advertise<geometry_msgs::Pose>(world_to_cam_pose_topic_name, 1, 0);
    ROS_INFO("Publishing board to camera pose on '%s'",
             n.resolveName(world_to_cam_pose_topic_name).c_str());

    std::string image_transport_namespace =
            GetCameraTopicName(n);
    image_transport::ImageTransport it = image_transport::ImageTransport(n);
    // register image transport subscriber
    image_transport::Subscriber sub = it.subscribe(
            image_transport_namespace, 1, &CameraImageCallback);
    //-----------


    //while(inputVideo.grab()) {
    while(ros::ok() ){
        if(new_image) {

            new_image = false;
            Mat imageCopy, image;
            image = Image(ros::Duration(1));
            image.copyTo(imageCopy);

            Vec3d rvec, tvec;
            bool valid_pose = DetectCharucoBoardPose(image, charucoboard,
                                                     dictionary, camMatrix,
                                                     distCoeffs, rvec, tvec);

            if (valid_pose) {
                // draw the axes
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec,
                                axisLength);

                // publish the pose
                geometry_msgs::Pose board_to_cam_msg;
                conversions::RvecTvecToPoseMsg(rvec, tvec, board_to_cam_msg);
                pub_board_to_cam_pose.publish(board_to_cam_msg);

            }

            imshow("out", imageCopy);
            char key = (char) waitKey(1);
            if (key == 27) break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


//------------------------------------------------------------------------------
void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
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
static bool readCameraParameters(string filename,
                                 Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
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
std::string GetCameraTopicName(ros::NodeHandle &n){

    std::string image_transport_namespace;

    if (n.getParam("image_transport_namespace", image_transport_namespace)) {
        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<sensor_msgs::Image>(
                image_transport_namespace, ros::Duration(5))) {
            ROS_ERROR("Topic '%s' is not publishing.",
                      image_transport_namespace.c_str());
        }
        else
            ROS_INFO("[SUBSCRIBERS] Images will be read from topic %s",
                     image_transport_namespace.c_str());
    } else {
        ROS_ERROR("%s Parameter '%s' is required.",
                  ros::this_node::getName().c_str(),
                  n.resolveName("image_transport_namespace").c_str());
    }
    return  image_transport_namespace;

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