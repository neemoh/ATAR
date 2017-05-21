//
// Created by nima on 21/05/17.
//



#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
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


namespace {
    const char* about = "Pose estimation using a ChArUco board";
    const char* keys  =
            "{w        |       | Number of squares in X direction }"
                    "{h        |       | Number of squares in Y direction }"
                    "{sl       |       | Square side length (in meters) }"
                    "{ml       |       | Marker side length (in meters) }"
                    "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
                    "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
                    "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
                    "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
                    "{c        |       | Output file with calibrated camera parameters }"
                    "{v        |       | Input from video file, if ommited, input comes from camera }"
                    "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
                    "{dp       |       | File of marker detector parameters }"
                    "{rs       |       | Apply refind strategy }"
                    "{r        |       | show rejected candidates too }";
}

bool EstimatePoseCharucoBoard(InputArray _charucoCorners, InputArray _charucoIds,
                              const Ptr<aruco::CharucoBoard> &_board, InputArray _cameraMatrix, InputArray _distCoeffs,
                              OutputArray _rvec, OutputArray _tvec) {

    CV_Assert((_charucoCorners.getMat().total() == _charucoIds.getMat().total()));

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
    std::cout << "0:  " << objPoints[0] << std::endl;
    std::cout << "1:  " << objPoints[1] << std::endl;
    solvePnP(objPoints, _charucoCorners, _cameraMatrix, _distCoeffs, _rvec, _tvec);

    return true;
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

cv::Mat image_msg;
bool new_image = false;

void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


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
/**

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
 */

/**
 */
int main(int argc, char *argv[]) {
//    CommandLineParser parser(argc, argv, keys);
//    parser.about(about);

//    if(argc < 6) {
//        parser.printMessage();
//        return 0;
//    }

    int squares_w ,squares_h;
    float square_length , marker_length;
    int dictionary_id, cam_id;
    bool showRejected;
    bool refindStrategy;
    std::string cam_intrinsic_file;

    Ptr<aruco::DetectorParameters> detector_params = aruco::DetectorParameters::create();
    detector_params->doCornerRefinement = true;

    ros::init(argc, argv, "extrinsic_aruco");
    std::string ros_node_name = ros::this_node::getName();

    ros::NodeHandle n(ros_node_name);
    ros::Rate loop_rate = ros::Rate(200);

    // Load the description of the aruco board from the parameters
    if (!n.getParam("board_w", squares_w))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("board_w").c_str());
    if (!n.getParam("board_h", squares_h))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("board_h").c_str());
    if (!n.getParam("square_length_in_meters", square_length))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("marker_length_in_meters").c_str());
    if (!n.getParam("marker_length_in_meters", marker_length))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("marker_length_in_meters").c_str());
    if (!n.getParam("dictionary_id", dictionary_id))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("dictionary_id").c_str());
    if (!n.getParam("cam_id", cam_id))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("cam_id").c_str());
    if (!n.getParam("cam_intrinsic_file", cam_intrinsic_file))
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("cam_intrinsic_file").c_str());

    // advertise publishers
    std::string world_to_cam_pose_topic_name;
    if (!n.getParam("world_to_cam_pose_topic_name", world_to_cam_pose_topic_name))
        world_to_cam_pose_topic_name = "world_to_camera";

    ros::Publisher pub_board_to_cam_pose = n.advertise<geometry_msgs::PoseStamped>(world_to_cam_pose_topic_name, 1, 0);
    ROS_INFO("Publishing board to camera pose on '%s'",
             n.resolveName(world_to_cam_pose_topic_name).c_str());

    std::string image_transport_namespace;
    if (n.getParam("image_transport_namespace", image_transport_namespace)) {
        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<sensor_msgs::Image>( image_transport_namespace, ros::Duration(5))) {
            ROS_ERROR("Topic '%s' is not publishing.",
                      image_transport_namespace.c_str());
        }
        else
            ROS_INFO("[SUBSCRIBERS] Images will be read from topic %s",
                     image_transport_namespace.c_str());
    } else {
        ROS_ERROR("%s Parameter '%s' is required.",
                  ros::this_node::getName().c_str(), n.resolveName("image_transport_namespace").c_str());
    }
    image_transport::ImageTransport it = image_transport::ImageTransport(n);
    // register image transport subscriber
    image_transport::Subscriber sub = it.subscribe(
            image_transport_namespace, 1, &CameraImageCallback);



    Mat camMatrix, distCoeffs;
    bool readOk = readCameraParameters(cam_intrinsic_file, camMatrix, distCoeffs);

//    String video;
//    if(parser.has("v")) {
//        video = parser.get<String>("v");
//    }
//
//    Mat camMatrix, distCoeffs;
//    if(parser.has("c")) {
//        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
//        if(!readOk) {
//            cerr << "Invalid camera file" << endl;
//            return 0;
//        }
//    }
//
//    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
//    if(parser.has("dp")) {
//        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
//        if(!readOk) {
//            cerr << "Invalid detector parameters file" << endl;
//            return 0;
//        }
//    }
//
//    if(!parser.check()) {
//        parser.printErrors();
//        return 0;loop_rate
//    }

    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

    VideoCapture inputVideo;
    int waitTime;
        inputVideo.open(cam_id);
        waitTime = 10;


    float axisLength = 0.5f * ((float)min(squares_w, squares_h) * (square_length));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(squares_w, squares_h, square_length, marker_length, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;

    //while(inputVideo.grab()) {
    while(ros::ok() ){
        if(new_image) {
            new_image = false;
            Mat imageCopy, image;
            //inputVideo.retrieve(image);
            image = Image(ros::Duration(1));

            double tick = (double) getTickCount();
            Vec3d rvec, tvec;
            vector<int> markerIds, charucoIds;
            vector<vector<Point2f> > markerCorners, rejectedMarkers;
            vector<Point2f> charucoCorners;

            // detect markers
            aruco::detectMarkers(image, dictionary, markerCorners, markerIds,
                                 detector_params,
                                 rejectedMarkers);

            // refind strategy to detect more markers
            if (refindStrategy)
                aruco::refineDetectedMarkers(image, board, markerCorners,
                                             markerIds, rejectedMarkers,
                                             camMatrix, distCoeffs);

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
                validPose = aruco::estimatePoseCharucoBoard(charucoCorners,
                                                            charucoIds,
                                                            charucoboard,
                                                            camMatrix,
                                                            distCoeffs, rvec,
                                                            tvec);


            double currentTime =
                    ((double) getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if (totalIterations % 30 == 0) {
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                     << "(Mean = " << 1000 * totalTime / double(totalIterations)
                     << " ms)" << " validPose: " << validPose << " tvec: "
                     << tvec[0] << " " << tvec[1] << " " << tvec[2] << endl;
            }



            // draw results
            image.copyTo(imageCopy);
            if (markerIds.size() > 0) {
                aruco::drawDetectedMarkers(imageCopy, markerCorners);
            }

            if (showRejected && rejectedMarkers.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, rejectedMarkers,
                                           noArray(), Scalar(100, 0, 255));

            if (interpolatedCorners > 0) {
                Scalar color;
                color = Scalar(255, 0, 255);
                aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners,
                                                  charucoIds, color);
            }

            if (validPose) {
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec,
                                axisLength);

                KDL::Frame temp_frame;
                conversions::RvecTvecToKDLFrame(rvec,
                                                tvec,
                                                temp_frame);
                geometry_msgs::PoseStamped board_to_cam_msg;

                tf::poseKDLToMsg(temp_frame, board_to_cam_msg.pose);

                pub_board_to_cam_pose.publish(board_to_cam_msg);

            }

            imshow("out", imageCopy);
            char key = (char) waitKey(waitTime);
            if (key == 27) break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
