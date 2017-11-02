//
// Created by nima on 02/11/17.
//

#include "AugmentedCamera.h"
#include <pwd.h>
#include <custom_conversions/Conversions.h>

AugmentedCamera::AugmentedCamera(ros::NodeHandlePtr n, image_transport::ImageTransport *it,
                                 const std::string cam_name, const std::string ns) {

    // AR camera
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;
    std::stringstream path;
    path << std::string(home_dir) << std::string("/.ros/camera_info/")
         << cam_name << "_intrinsics.yaml";
    ReadCameraParameters(path.str());

    // --------------------Images
    // image subscriber
    img_topic = "/"+cam_name+ "/image_raw";
    if(ns!="")
        img_topic = "/"+ns+"/"+cam_name+ "/image_raw";

    sub_image = it->subscribe(img_topic, 1, &AugmentedCamera::ImageCallback, this);


    // ------------------------------------------- CAM POSE
    // we first try to read the poses as parameters and later update the
    // poses if new messages are arrived on the topics

    // --------------- 1- it can be set as a parameter
    std::vector<double> temp_vec = std::vector<double>( 7, 0.0);
    n->getParam("/calibrations/world_frame_to_"+cam_name+"_frame", temp_vec);

    // --------------- 2- it can be read from a topic

    std::string pose_topic_name = "/"+cam_name+ "/world_to_camera_transform";
    if(ns!="")
        pose_topic_name = "/"+ns+"/"+cam_name+ "/world_to_camera_transform";

    // is something being published?
    if (ros::topic::waitForMessage<geometry_msgs::PoseStamped>
            (pose_topic_name, ros::Duration(1))){
        // now we set up the subscribers
        sub_pose = n->subscribe(pose_topic_name, 1,
                                &AugmentedCamera::PoseCallback, this);
    }
    else{
        is_pose_from_subscriber = false;

        // --------------- 3- it can be estimated directly
        ROS_INFO("Camera pose is not published on topic '%s'. Pose will be "
                         "estimated.", pose_topic_name.c_str());

        // Read boardparameters
        // board_params comprises:
        // [dictionary_id, board_w, board_h,
        // square_length_in_meters, marker_length_in_meters]
        std::vector<float> board_params = std::vector<float>(5, 0.0);
        if(!n->getParam("board_params", board_params))
        {
            if(!n->getParam("/calibrations/board_params", board_params))
                ROS_ERROR("Ros parameter board_param is required. board_param="
                                  "[dictionary_id, board_w, board_h, "
                                  "square_length_in_meters, marker_length_in_meters]");
        }

        dictionary =
                cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME
                                                           (board_params[0]));
        // create charuco board object
        charuco_board = cv::aruco::CharucoBoard::create((int)board_params[1],
                                                        (int)board_params[2],
                                                        board_params[3],
                                                        board_params[4],
                                                        dictionary);
    }
}

//------------------------------------------------------------------------------
void AugmentedCamera::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try
    {
        image_from_ros = cv_bridge::toCvCopy(msg, "rgb8")->image;
        new_image= true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

//------------------------------------------------------------------------------
void AugmentedCamera::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    new_pose_from_sub = true;
    tf::poseMsgToKDL(msg->pose, world_to_cam_pose);
}

void
AugmentedCamera::GetIntrinsicParams(double &fx, double &fy, double &cx, double &cy) {

    fx = camera_matrix.at<double>(0, 0);
    fy = camera_matrix.at<double>(1, 1);
    cx = camera_matrix.at<double>(0, 2);
    cy = camera_matrix.at<double>(1, 2);
}

//------------------------------------------------------------------------------
void AugmentedCamera::ReadCameraParameters(const std::string file_path) {

    cv::FileStorage fs(file_path, cv::FileStorage::READ);

    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");
    fs["camera_matrix"] >> camera_matrix;

    // check if we got something
    if(camera_matrix.empty()){
        ROS_ERROR("camera_matrix not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }

    fs["distortion_coefficients"] >> camera_distortion;
    if(camera_distortion.empty()){
        ROS_ERROR("distortion_coefficients  not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
}

//------------------------------------------------------------------------------
void AugmentedCamera::LockAndGetImage(cv::Mat &image) {
    ros::Rate loop_rate(2);
    ros::Time timeout_time = ros::Time::now() + ros::Duration(1);

    while(ros::ok() && image_from_ros.empty()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (ros::Time::now() > timeout_time)
            ROS_WARN_STREAM(("Timeout: No Image on."+img_topic+
                             " Trying again...").c_str());
    }
    image_from_ros.copyTo(image);

    new_image = false;
}

bool AugmentedCamera::IsImageNew() {
    if(new_image) {
        new_image = false;
        return true;
    }
    return false;
}

cv::Mat AugmentedCamera::GetImage() {
    // it is important to copy the image to prevent seg fault due to
    // subscriber thread and vtk rendering thread accessing the image at the
    // same time
    cv::Mat out;
    image_from_ros.copyTo(out);
    return out;
}

bool AugmentedCamera::GetNewWorldToCamTr(KDL::Frame &pose) {

    // if pose comes from the subscriber
    if(new_pose_from_sub){
        pose = world_to_cam_pose;
        new_pose_from_sub = false;
        return true;
    }
    else if(!is_pose_from_subscriber && !image_from_ros.empty() ){
        cv::Mat img;
        image_from_ros.copyTo(img);
        if(DetectCharucoBoardPose(world_to_cam_pose, img)) {
            pose = world_to_cam_pose;
            return true;
        }
    }

    return false;
}

bool AugmentedCamera::DetectCharucoBoardPose(KDL::Frame &pose, cv::Mat image) {

    std::vector<int> marker_ids, charuco_ids;
    std::vector<std::vector<cv::Point2f> > marker_corners, rejected_markers;
    std::vector<cv::Point2f> charuco_corners;


    cv::Ptr<cv::aruco::DetectorParameters> detector_params =
            cv::aruco::DetectorParameters::create();
    detector_params->doCornerRefinement = true;

    // detect markers
    cv::aruco::detectMarkers(image, dictionary, marker_corners,
                             marker_ids, detector_params, rejected_markers);

    //    // refind strategy to detect more markers
    //    if (refindStrategy)
    //        aruco::refineDetectedMarkers(image, board, markerCorners,
    //                                     marker_ids, rejectedMarkers,
    //                                     camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (marker_ids.size() > 0)
        interpolatedCorners =
                cv::aruco::interpolateCornersCharuco(marker_corners,
                                                     marker_ids, image,
                                                     charuco_board,
                                                     charuco_corners,
                                                     charuco_ids, camera_matrix,
                                                     camera_distortion);

    // estimate charuco board pose
    CV_Assert((charuco_corners.size() ==charuco_ids.size()));

    // need, at least, 4 corners
    if(charuco_ids.size() < 4)
        return false;

    std::vector< cv::Point3f > objPoints;
    objPoints.reserve(charuco_ids.size());

    for(unsigned int i = 0; i < charuco_ids.size(); i++) {
        int currId = charuco_ids[i];
        CV_Assert(currId >= 0
                  && currId < (int)charuco_board->chessboardCorners.size());
        objPoints.push_back(charuco_board->chessboardCorners[currId]);
    }

    // points need to be in different lines, check if detected points are enough
    cv::Vec3d rvec, tvec;
    solvePnP(objPoints, charuco_corners, camera_matrix, camera_distortion,
             rvec, tvec);

    conversions::RvecTvecToKDLFrame(rvec, tvec, pose);
    return true;
}


