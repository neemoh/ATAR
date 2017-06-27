//
// Created by nima on 29/05/17.
//

#include <opencv-3.2.0-dev/opencv2/opencv.hpp>
#include "IntrinsicCalibrationCharuco.h"
#include <ros/ros.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


IntrinsicCalibrationCharuco::IntrinsicCalibrationCharuco(
        std::string img_topic_namespace,
        std::vector<float> charuco_board_params) :
        image_topic_ns(img_topic_namespace)
{

    if(charuco_board_params.size()!=5)
        throw std::runtime_error("charuco_board_params must have 5 elements");

    dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME(charuco_board_params[0]));

    // create charuco board object with params:
    // int squaresX,
    // int squaresY,
    // float squareLength,
    // float markerLength,
    // const Ptr<Dictionary> &dictionary);

    charuco_board = cv::aruco::CharucoBoard::create(
            (int)charuco_board_params[1],
            (int)charuco_board_params[2],
            charuco_board_params[3],
            charuco_board_params[4],
            dictionary);
    detector_params =
            cv::aruco::DetectorParameters::create();
    detector_params->doCornerRefinement = true;

}

bool IntrinsicCalibrationCharuco::DoCalibration(std::string outputFile,
                                                double &repError,
                                                cv::Mat &cameraMatrix,
                                                cv::Mat &distCoeffs) {



    ros::NodeHandle n("IntrinsicCalibrationCharuco");
    ros::Rate loop_rate = ros::Rate(50);
    image_transport::ImageTransport it = image_transport::ImageTransport(n);
    image_transport::Subscriber sub = it.subscribe(image_topic_ns, 1,
                                                   &IntrinsicCalibrationCharuco::CameraImageCallback, this);
    ROS_INFO("IntrinsicCalibrationCharuco subscribed to %s", image_topic_ns
            .c_str());
    std::string window_name = "Intrinsic calibration";
    // -----------------------------------------------------------------------//

    while(ros::ok() && !finished_capturing ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // -----------------------------------------------------------------------//

    if(allIds.size() < 10 || allImgs.size() < 15) {
        ROS_WARN("Not enough captures for calibration. Take at least 15 "
                          "frames");
        cvDestroyWindow(window_name.c_str());
        return false;
    }

    //    cv::Mat cameraMatrix, distCoeffs;
    std::vector< cv::Mat > rvecs, tvecs;
    //    double repError;
    float aspectRatio = 1;

    // prepare data for calibration
    std::vector< std::vector< cv::Point2f > > allCornersConcatenated;
    std::vector< int > allIdsConcatenated;
    std::vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    int calibrationFlags = 0;
    cv::Ptr<cv::aruco::Board> board = charuco_board
            .staticCast<cv::aruco::Board>();

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = cv::aruco::calibrateCameraAruco(
            allCornersConcatenated, allIdsConcatenated,
            markerCounterPerFrame, board, imgSize, cameraMatrix,
            distCoeffs, cv::noArray(), cv::noArray
                    (), calibrationFlags);

    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    std::vector< cv::Mat > allCharucoCorners;
    std::vector< cv::Mat > allCharucoIds;
    std::vector< cv::Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);


    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        cv::Mat currentCharucoCorners, currentCharucoIds;
        cv::aruco::interpolateCornersCharuco(
                allCorners[i], allIds[i],
                allImgs[i], charuco_board,
                currentCharucoCorners, currentCharucoIds, cameraMatrix,
                distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if(allCharucoCorners.size() < 4) {
        ROS_ERROR("Not enough corners for calibration");
        cvDestroyWindow(window_name.c_str());
        return false;
    }

    // calibrate camera using charuco
    repError =
            cv::aruco::calibrateCameraCharuco(
                    allCharucoCorners, allCharucoIds,
                    charuco_board, imgSize,
                    cameraMatrix, distCoeffs, rvecs, tvecs,
                    calibrationFlags);

    bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio,
                                    calibrationFlags, cameraMatrix,
                                    distCoeffs, repError);
    if(!saveOk) {
        ROS_ERROR("Cannot save output file");
        cvDestroyWindow(window_name.c_str());
        return false;
    }

    std::cout << "Rep Error: " << repError << std::endl;
    std::cout << "Rep Error Aruco: " << arucoRepErr << std::endl;
    std::cout << "Calibration saved to " << outputFile << std::endl;

    cvDestroyWindow(window_name.c_str());
    return true;
}



void IntrinsicCalibrationCharuco::CameraImageCallback(
        const sensor_msgs::ImageConstPtr &msg) {

    // generally not a good idea to do things that may take a while, in the
    // callback, but let's bet on the low refresh rate of images and do it
    // anyways!

    cv::Mat image, imageCopy;

    try {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;

    // detect markers
    cv::aruco::detectMarkers(
        image, dictionary, corners, ids,
        detector_params, rejected
    );

    // refind strategy to detect more markers
    //            if (refindStrategy)
    //                cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

    // interpolate charuco corners
    cv::Mat currentCharucoCorners, currentCharucoIds;
    if (ids.size() > 0)
        cv::aruco::interpolateCornersCharuco(
            corners, ids, image, charuco_board,
            currentCharucoCorners,
            currentCharucoIds
        );

    // draw results
    image.copyTo(imageCopy);
    if (ids.size() > 0)
        cv::aruco::drawDetectedMarkers(imageCopy, corners);

    if (currentCharucoCorners.total() > 0)
        cv::aruco::drawDetectedCornersCharuco(
            imageCopy, currentCharucoCorners, currentCharucoIds
        );

    cv::putText(
        imageCopy, "Press 'c' to add current frame. 'f' to finish and "
            "calibrate.",
        cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(
            255,
            0, 0
        ), 2
    );

    cv::imshow("Intrinsic calibration", imageCopy);
    char key = (char) cv::waitKey(1);
    if (key == 'f')
        finished_capturing = true;

    cv::Size size;
    size = charuco_board->getChessboardSize();

    if (key == 'c') {
        if (ids.size() >= (size.height * size.width / 2)) {
            std::cout << "Frame " << allImgs.size() + 1 << " captured and found "
                      << ids.size() << " markers" << std::endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();
        } else
            std::cout << "Not enough markers detected: "
                      << ids.size() << std::endl;
    }
}




bool IntrinsicCalibrationCharuco::saveCameraParams(
        const std::string
        &filename, cv::Size imageSize, float
        aspectRatio, int flags,
        const cv::Mat &cameraMatrix, const cv::Mat
        &distCoeffs, double totalAvgErr) {

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

//    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;
//
//    if(flags != 0) {
//        sprintf(buf, "flags: %s%s%s%s",
//                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
//                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
//                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
//                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
//    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}
