//
// Created by charm on 2/21/17.
//

#include "ACOverlay.h"
#include <utils/Conversions.hpp>
#include <pwd.h>
#include "opencv2/calib3d/calib3d.hpp"

ACOverlay::ACOverlay(std::string node_name, int width, int height)
        : n(node_name), image_width(width), image_height(height)
{

//    bufferL_ = new unsigned char[image_width*image_height*4];
//    bufferR_ = new unsigned char[image_width*image_height*4];

    it = new image_transport::ImageTransport(n);
    SetupROS();


}



void ACOverlay::ReadCameraParameters(const std::string file_path,
                                           CameraIntrinsics & camera) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> camera.camMatrix;
    fs["distortion_coefficients"] >> camera.distCoeffs;

    // check if we got osomething
    if(camera.distCoeffs.empty()){
        ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(camera.camMatrix.empty()){
        ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }


}



//-----------------------------------------------------------------------------------
// SetupROS
//-----------------------------------------------------------------------------------

void ACOverlay::SetupROS() {

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    bool all_required_params_found = true;

    // ------- load the intrinsic calibration files
    // get home directory
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;

    std::string left_cam_name;
    if (n.getParam("left_cam_name", left_cam_name)) {

        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << left_cam_name << "_intrinsics.xml";
        ReadCameraParameters(path.str(), cam_intrinsics[0]);
    } else
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.xml",
                n.resolveName("left_cam_name").c_str());

    std::string right_cam_name;
    if (n.getParam("right_cam_name", right_cam_name)) {
        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << right_cam_name << "_intrinsics.xml";
        ReadCameraParameters(path.str(), cam_intrinsics[1]);
    } else
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.xml",
                n.resolveName("right_cam_name").c_str());


    //--------
    // Left image subscriber
    std::string left_image_topic_name = "/camera/left/image_color";;
    if (n.getParam("left_image_topic_name", left_image_topic_name))
        ROS_INFO(
                "[SUBSCRIBERS] Left camera images will be read from topic '%s'",
                left_image_topic_name.c_str());
    image_subscribers[0] = it->subscribe(
            left_image_topic_name, 1, &ACOverlay::ImageLeftCallback,
            this);

    //--------
    // Left image subscriber.
    std::string right_image_topic_name = "/camera/right/image_color";
    if (n.getParam("right_image_topic_name", right_image_topic_name))
        ROS_INFO(
                "[SUBSCRIBERS] Right camera images will be read from topic '%s'",
                right_image_topic_name.c_str());
    image_subscribers[1] = it->subscribe(
            right_image_topic_name, 1, &ACOverlay::ImageRightCallback,
            this);

    //--------

    //--------
    // We need to know the pose of the cameras with respect to the task-space coordinates.
    // If the camera or the markers move the pose should be estimated by a node and here we
    // subscribe to that topic. If on the other hand no cam/marker motion is involved the
    // fixed pose is read as a static parameter.
    //
    // when num_cam_pose_publishers ==0 : everything fixed. no pose is being published we
    // read the pose of left cam and derive the pose of the right one using the
    // left_to_right transform.
    //
    // when num_cam_pose_publishers ==1 : left cam pose is published and right one is derived
    // using the left_to_right transform.
    //
    // when num_cam_pose_publishers ==2 : both poses are published.


    n.param("num_cam_pose_publishers", num_cam_pose_publishers, 0);
    // the transformation from left cam coordinate frame to right cam. If this is not
    // available, then both cam poses must be publishing.
    if(num_cam_pose_publishers < 2){

        std::vector<double> left_to_right_cam_transform = std::vector<double>(7, 0.0);
        if(n.getParam("/calibrations/left_cam_frame_to_right_cam_frame", left_to_right_cam_transform))
            conversions::VectorToKDLFrame(left_to_right_cam_transform, left_cam_to_right_cam_tr);
        else
            ROS_ERROR("Expecting %d camera pose publishers. "
                              " Parameter /calibrations/left_cam_frame_to_right_cam_frame is not set. If both of the camera poses are not "
                              "published this parameter is needed.",
                       num_cam_pose_publishers);
    }

    if (num_cam_pose_publishers == 0) {
        // fixed case
        std::vector<double> task_frame_to_left_cam_frame = std::vector<double>(
                7, 0.0);
        if (n.getParam("/calibrations/task_frame_to_left_cam_frame",
                       task_frame_to_left_cam_frame)) {
            conversions::VectorToKDLFrame(task_frame_to_left_cam_frame,
                                          pose_cam[0]);
            conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec[0], cam_tvec[0]);
        } else
            ROS_ERROR("Expecting 0 camera pose publishers. Parameter "
                              "/calibrations/task_frame_to_left_cam_frame is not set.");

        std::vector<double> task_frame_to_right_cam_frame = std::vector<double>(
                7, 0.0);
        if (n.getParam("/calibrations/task_frame_to_right_cam_frame",
                       task_frame_to_right_cam_frame)) {
            conversions::VectorToKDLFrame(task_frame_to_right_cam_frame,
                                          pose_cam[1]);
            conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec[1], cam_tvec[1]);
        } else
            ROS_ERROR("[SUBSCRIBERS] Expecting 0 camera pose publishers. "
                              "Parameter /calibrations/task_frame_to_right_cam_frame is not set.");

    }
    else if (num_cam_pose_publishers == 1 || num_cam_pose_publishers == 2){
        // If a parameter is set, use that
        // Left image pose subscriber. Get the topic name parameter and make sure it is being published
        std::string left_cam_pose_topic_name;
        if (n.getParam("left_cam_pose_topic_name", left_cam_pose_topic_name)) {
            // if the topic name is found, check if something is being published on it
            if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                    left_cam_pose_topic_name, ros::Duration(4))) {
                ROS_WARN("Topic '%s' is not publishing.",
                         n.resolveName(left_cam_pose_topic_name).c_str());
            } else
                ROS_INFO(
                        "[SUBSCRIBERS] Left camera pose will be read from topic '%s'",
                        n.resolveName(left_cam_pose_topic_name).c_str());
        } else {
            ROS_ERROR("Parameter '%s' is required.",
                      n.resolveName("left_cam_pose_topic_name").c_str());
            all_required_params_found = false;
        }
        subscriber_camera_pose_left = n.subscribe(
                left_cam_pose_topic_name, 1, &ACOverlay::LeftCamPoseCallback,
                this);


        if(num_cam_pose_publishers == 2){
            // Right camera pose will be needed only if the fixed transformation between right and
            // left camera is not provided as a parameter.
            // Right image pose subscriber. Get the topic name parameter and make sure it is being published
            std::string right_cam_pose_topic_name;
            if (n.getParam("right_cam_pose_topic_name", right_cam_pose_topic_name)) {
                // if the topic name is found, check if something is being published on it
                if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                        right_cam_pose_topic_name, ros::Duration(4)))
                    ROS_WARN("%s parameter was not provided and topic '%s' is not being published yet.",
                             n.resolveName("right_cam_pose_topic_name").c_str(),
                             right_cam_pose_topic_name.c_str());
                else
                    ROS_INFO("[SUBSCRIBERS] Right camera pose will be read from topic '%s'",

                             n.resolveName(right_cam_pose_topic_name).c_str());
            } else {
                ROS_ERROR("Since right_cam_pose_topic_name parameter was not provided"
                                  "parameter '%s' is required.", n.resolveName("right_cam_pose_topic_name").c_str());
                all_required_params_found = false;
            }
            subscriber_camera_pose_right = n.subscribe(
                    right_cam_pose_topic_name, 1, &ACOverlay::RightCamPoseCallback, this);

            //----------
            // to find the transformation between the cameras we defined a simple service.
            stereo_tr_calc_client =
                    n.serviceClient<teleop_vision::CalculateStereoCamsTransfromFromTopics>
                            ("/calculate_stereo_cams_transform_from_topics");
            stereo_tr_srv.request.cam_1_pose_topic_name = left_cam_pose_topic_name;
            stereo_tr_srv.request.cam_2_pose_topic_name = right_cam_pose_topic_name;

        }
    }
    if (!all_required_params_found)
        throw std::runtime_error("ERROR: some required topics are not set");


    //--------
    // task_frame_to_PSM1_frame
    std::vector<double> taskspace_to_psm1_vec = std::vector<double>(7, 0.0);
    if(!n.getParam("/calibrations/task_frame_to_PSM1_frame", taskspace_to_psm1_vec))
        ROS_WARN("Parameter /task_frame_to_PSM1_frame is not set. This parameter is required"
                         "if PSM1 is used.");
    conversions::VectorToKDLFrame(taskspace_to_psm1_vec, task_frame_to_PSM1_frame);

    //--------
    // task_frame_to_PSM2_frame
    std::vector<double> taskspace_to_psm2_vec = std::vector<double>(7, 0.0);
    if(!n.getParam("/calibrations/task_frame_to_PSM2_frame", taskspace_to_psm2_vec))
        ROS_WARN("Parameter /task_frame_to_PSM2_frame is not set. This parameter is required"
                         "if PSM2 is used.");
    conversions::VectorToKDLFrame(taskspace_to_psm2_vec, task_frame_to_PSM2_frame);


    //--------
    // PSM1 pose subscriber.
    std::string psm1_pose_topic_name = "/dvrk/PSM1/position_cartesian_current";
    subscriber_pose_psm1 =
            n.subscribe(psm1_pose_topic_name, 2,
                        &ACOverlay::Tool1PoseCallback, this);
    ROS_INFO("[SUBSCRIBERS] Tool 1 pose will be read from topic '%s'",
             psm1_pose_topic_name.c_str());

    //--------
    // PSM2 pose subscriber.
    std::string psm2_pose_topic_name = "/dvrk/PSM2/position_cartesian_current";
    subscriber_pose__sub =
            n.subscribe(psm2_pose_topic_name, 2,
                        &ACOverlay::Tool2PoseCallback, this);
    ROS_INFO("[SUBSCRIBERS] Tool 2 pose will be read from topic '%s'",

             psm2_pose_topic_name.c_str());



    //--------
    //subscriber_ac_path = n.subscribe("/ac_path", 1, &ACOverlay::ACPathCallback, this);

    std::string topic_name = "/ac_path";
    publisher_ac_path = n.advertise<geometry_msgs::PoseArray>(topic_name, 1 );
    ROS_INFO("Will publish on %s",
             topic_name.c_str());

    subscriber_foot_pedal_clutch = n.subscribe("/dvrk/footpedals/camera", 1,
                                               &ACOverlay::FootSwitchCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /dvrk/footpedals/camera");

    subscriber_ac_pose_desired_right = n.subscribe("/PSM1/tool_pose_desired", 1,
                                                   &ACOverlay::ACPoseDesiredRightCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /PSM1/tool_pose_desired");

    subscriber_ac_pose_desired_left = n.subscribe("/PSM2/tool_pose_desired", 1,
                                                  &ACOverlay::ACPoseDesiredRightCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /PSM2/tool_pose_desired");

    // publishers for the overlayed images
    publisher_overlayed[0] = it->advertise("left/image_color", 1);
    publisher_overlayed[1] = it->advertise("right/image_color", 1);

    // advertise publishers
//    std::string board_to_cam_pose_topic_name;
//    if (!n.getParam("board_to_cam_pose_topic_name", board_to_cam_pose_topic_name))
//        board_to_cam_pose_topic_name = "board_to_camera";
//
//    pub_board_to_cam_pose = n.advertise<geometry_msgs::PoseStamped>(board_to_cam_pose_topic_name, 1, 0);
//    ROS_INFO("Publishing board to camera pose on '%s'", n.resolveName(board_to_cam_pose_topic_name).c_str());

}



void ACOverlay::ImageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_right = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_right_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ACOverlay::ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_left = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_left_image = true;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



void ACOverlay::LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{


    tf::poseMsgToKDL(msg->pose, pose_cam[0]);
    conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec[0], cam_tvec[0]);

    // If we don't have the transforms between the cameras
    if(num_cam_pose_publishers==1) {
        pose_cam[1] = left_cam_to_right_cam_tr * pose_cam[0] ;
        conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec[1], cam_tvec[1]);
    }
}

void ACOverlay::RightCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam[1]);
    conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec[1], cam_tvec[1]);
}

void ACOverlay::Tool1PoseCallback(
        const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Convert message to KDL
    KDL::Frame frame_temp;
    tf::poseMsgToKDL(msg->pose, frame_temp);

    // take the robot end-effector pose to the task space coordinate frame
    pose_tool1 = task_frame_to_PSM1_frame.Inverse() * frame_temp;

}

void ACOverlay::Tool2PoseCallback(
        const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Convert message to KDL
    KDL::Frame frame_temp;
    tf::poseMsgToKDL(msg->pose, frame_temp);

    // take the robot end-effector pose to the task space coordinate frame
    pose_tool2 = task_frame_to_PSM2_frame.Inverse() * frame_temp;
}

//void ACOverlay::ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg){
//
//    for (int n_point = 0; n_point < msg->poses.size(); ++n_point) {
//        ac_path.push_back(cv::Point3d(msg->poses[n_point].position.x,
//                                      msg->poses[n_point].position.y,
//                                      msg->poses[n_point].position.z));
//    }
//}

void ACOverlay::ACPoseDesiredLeftCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    // Convert message to KDL
    tf::poseMsgToKDL(msg->pose, pose_desired[0]);
}

void ACOverlay::ACPoseDesiredRightCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    // Convert message to KDL
    tf::poseMsgToKDL(msg->pose, pose_desired[1]);
}

void ACOverlay::FootSwitchCallback(const sensor_msgs::Joy & msg){
    foot_switch_pressed = (bool)msg.buttons[0];
}

cv::Mat& ACOverlay::ImageLeft(ros::Duration timeout) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_left.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time)
            ROS_WARN("Timeout: No new left Image.");
    }
    new_left_image = false;
    return image_left;
}



cv::Mat& ACOverlay::ImageRight(ros::Duration timeout) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_right.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new right Image.");
        }
    }
    new_right_image = false;
    return image_right;
}



size_t MultiplePathsTask::FindClosestTarget(const KDL::Vector tool_current_position,
                                             const std::vector<cv::Point3d> targets){
    double min_d = 100000; // something large
    size_t i_min = 0;

    for(size_t i=0; i<targets.size(); i++){
        double dx = tool_current_position[0] - targets[i].x;
        double dy = tool_current_position[1] - targets[i].y;
        double dz = tool_current_position[2] - targets[i].z;
        double norm2 = dx*dx + dy*dy + dz*dz;
        if(norm2 < min_d){
            min_d = norm2;
            i_min = i;
        }

    }
    return i_min;

}

void MultiplePathsTask::GeneratePathPoints(const KDL::Vector current_position,
                        const cv::Point3d target, std::vector<cv::Point3d> & ac_path){
    ac_path.clear();
    // simple linear interpolation
    double n_points_per_meter =2000;
    KDL::Vector to_target = KDL::Vector(target.x - current_position[0],
                                        target.y - current_position[1],
                                        target.z - current_position[2]);
    int n_points = int(to_target.Norm() * n_points_per_meter);

    for (int i = 0; i < n_points ; ++i) {
        KDL::Vector temp = current_position + double(i)/double(n_points) * to_target;
        ac_path.push_back(cv::Point3d(temp[0], temp[1], temp[2]));
    }


}
void MultiplePathsTask::DrawAllPaths(cv::InputOutputArray image,
                                    const CameraIntrinsics &cam_intrinsics,
                                    const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                                    const KDL::Vector &tooltip_pos,
                                    std::vector<cv::Point3d> targets,
                                    const size_t &selected_index,
                                    const cv::Scalar &color_selected,
                                    const cv::Scalar &color_others){

    // add the tooltip to the targets to project all the points together
    targets.push_back(cv::Point3d(tooltip_pos[0], tooltip_pos[1],tooltip_pos[2]));

    // project the points to 2d
    std::vector<cv::Point2d> targets_2d;
    projectPoints(targets, rvec, tvec, cam_intrinsics.camMatrix, cam_intrinsics.distCoeffs,
                  targets_2d);

    // get back the tooltip (for easy reading of the code)
    cv::Point2d tooltip_2d = targets_2d.back();
    targets_2d.pop_back();

    // draw lines and target points
    for (int i = 0; i <targets_2d.size() ; ++i) {

        if(i==selected_index) {
            line(image, targets_2d[i], tooltip_2d, color_selected, 2, CV_AA);
            circle(image, targets_2d[i], 5, color_selected, -1);
        }
        else{
            line(image, targets_2d[i], tooltip_2d, color_others, 2, CV_AA);
            circle(image, targets_2d[i], 5, color_others, -1);
        }

    }


}


void VisualUtils::SwitchFullScreen(const std::string window_name) {

    if (cvGetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN) ==
        CV_WINDOW_NORMAL)
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    else
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

}


void SimpleACs::GenerateXYCircle(const KDL::Vector center, const double radius, const int num_points,
                                          std::vector<cv::Point3d> & ac_path){
    cv::Point3d point;
    for (int n_point = 0; n_point <num_points ; ++n_point) {
        double angle = 2* M_PI*(double)n_point / (double)(num_points);
        point.z = center[2];
        point.x = center[0] + radius * cos(angle);
        point.y = center[1] + radius * sin(angle);
        ac_path.push_back(point);
    }
}

void  VecPoint3dToPoseArray(std::vector<cv::Point3d> vec, geometry_msgs::PoseArray & out) {

    geometry_msgs::Pose point;

    for (int i = 0; i < vec.size(); ++i) {
        point.position.x = vec[i].x;
        point.position.y = vec[i].y;
        point.position.z = vec[i].z;
        out.poses.push_back(point);
    }
}
