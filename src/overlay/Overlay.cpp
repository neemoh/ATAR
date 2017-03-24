//
// Created by charm on 2/21/17.
//

#include "Overlay.h"
#include <utils/Conversions.hpp>
#include <pwd.h>
#include "opencv2/calib3d/calib3d.hpp"

OverlayGraphics::OverlayGraphics(std::string node_name, int width, int height)
        : n(node_name), image_width_(width), image_height_(height)

{

//    bufferL_ = new unsigned char[image_width_*image_height_*4];
//    bufferR_ = new unsigned char[image_width_*image_height_*4];
    it = new image_transport::ImageTransport(n);
    SetupROS();


}



void OverlayGraphics::ReadCameraParameters(const std::string file_path,
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

void OverlayGraphics::SetupROS() {

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
                "%s Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.xml",
                ros::this_node::getName().c_str(),
                n.resolveName("left_cam_name").c_str());

    std::string right_cam_name;
    if (n.getParam("right_cam_name", right_cam_name)) {
        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << right_cam_name << "_intrinsics.xml";
        ReadCameraParameters(path.str(), cam_intrinsics[1]);
    } else
        ROS_ERROR(
                "%s Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.xml",
                ros::this_node::getName().c_str(),
                n.resolveName("right_cam_name").c_str());


    //--------
    // Left image subscriber
    std::string left_image_topic_name = "/camera/left/image_color";;
    if (n.getParam("left_image_topic_name", left_image_topic_name))
        ROS_INFO(
                "%s [SUBSCRIBERS] Left camera images will be read from topic '%s'",
                ros::this_node::getName().c_str(),
                left_image_topic_name.c_str());
    image_subscribers[0] = it->subscribe(
            left_image_topic_name, 1, &OverlayGraphics::ImageLeftCallback,
            this);

    //--------
    // Left image subscriber.
    std::string right_image_topic_name = "/camera/right/image_color";
    if (n.getParam("right_image_topic_name", right_image_topic_name))
        ROS_INFO(
                "%s [SUBSCRIBERS] Right camera images will be read from topic '%s'",
                ros::this_node::getName().c_str(),
                right_image_topic_name.c_str());
    image_subscribers[1] = it->subscribe(
            right_image_topic_name, 1, &OverlayGraphics::ImageRightCallback,
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
        if(n.getParam("/left_cam_to_right_cam_transform", left_to_right_cam_transform))
            conversions::VectorToKDLFrame(left_to_right_cam_transform, left_cam_to_right_cam_tr);
        else
            ROS_ERROR("%s Expecting %d camera pose publishers. "
                              " Parameter /left_cam_to_right_cam_transform is not set. If both of the camera poses are not "
                              "published this parameter is needed.",
                      ros::this_node::getName().c_str(), num_cam_pose_publishers);
    }

    if (num_cam_pose_publishers == 0) {
        // fixed case
        std::vector<double> taskspace_to_left_cam_transform = std::vector<double>(
                7, 0.0);
        if (n.getParam("/taskspace_to_left_cam_transform",
                       taskspace_to_left_cam_transform)) {
            conversions::VectorToKDLFrame(taskspace_to_left_cam_transform,
                                          pose_cam_l);
            conversions::KDLFrameToRvectvec(pose_cam_l, cam_rvec_l, cam_tvec_l);
        } else
            ROS_ERROR("%s Expecting 0 camera pose publishers. Parameter "
                              "/taskspace_to_left_cam_transform is not set.",
                      ros::this_node::getName().c_str());

        std::vector<double> taskspace_to_right_cam_transform = std::vector<double>(
                7, 0.0);
        if (n.getParam("/taskspace_to_right_cam_transform",
                       taskspace_to_right_cam_transform)) {
            conversions::VectorToKDLFrame(taskspace_to_right_cam_transform,
                                          pose_cam_r);
            conversions::KDLFrameToRvectvec(pose_cam_r, cam_rvec_r, cam_tvec_r);
        } else
            ROS_ERROR("%s [SUBSCRIBERS] Expecting 0 camera pose publishers. "
                              " Parameter /taskspace_to_right_cam_transform is not set.",
                      ros::this_node::getName().c_str());

    }
    else if (num_cam_pose_publishers == 1 || num_cam_pose_publishers == 2){
        // If a parameter is set, use that
        // Left image pose subscriber. Get the topic name parameter and make sure it is being published
        std::string left_cam_pose_topic_name;
        if (n.getParam("left_cam_pose_topic_name", left_cam_pose_topic_name)) {
            // if the topic name is found, check if something is being published on it
            if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                    left_cam_pose_topic_name, ros::Duration(4))) {
                ROS_WARN("%s: Topic '%s' is not publishing.",
                         ros::this_node::getName().c_str(),
                         n.resolveName(left_cam_pose_topic_name).c_str());
            } else
                ROS_INFO(
                        "%s [SUBSCRIBERS] Left camera pose will be read from topic '%s'",
                        ros::this_node::getName().c_str(),
                        n.resolveName(left_cam_pose_topic_name).c_str());
        } else {
            ROS_ERROR("%s:Parameter '%s' is required.",
                      ros::this_node::getName().c_str(),
                      n.resolveName("left_cam_pose_topic_name").c_str());
            all_required_params_found = false;
        }
        camera_pose_subscriber_left = n.subscribe(
                left_cam_pose_topic_name, 1, &OverlayGraphics::LeftCamPoseCallback,
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
                    ROS_INFO("%s [SUBSCRIBERS] Right camera pose will be read from topic '%s'",
                             ros::this_node::getName().c_str(),
                             n.resolveName(right_cam_pose_topic_name).c_str());
            } else {
                ROS_ERROR("Since right_cam_pose_topic_name parameter was not provided"
                                  "parameter '%s' is required.", n.resolveName("right_cam_pose_topic_name").c_str());
                all_required_params_found = false;
            }
            camera_pose_subscriber_right = n.subscribe(
                    right_cam_pose_topic_name, 1, &OverlayGraphics::RightCamPoseCallback, this);


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
    // PSM1 pose subscriber
    std::vector<double> taskspace_to_psm1_vec = std::vector<double>(7, 0.0);
    if(!n.getParam("/taskspace_to_PSM1_tr", taskspace_to_psm1_vec))
        ROS_WARN("%s Parameter /taskspace_to_PSM1_tr is not set. This parameter is required"
                         "if PSM1 is used.", ros::this_node::getName().c_str());
    conversions::VectorToKDLFrame(taskspace_to_psm1_vec, taskspace_to_psm1_tr);

    //--------
    // PSM! pose subscriber
    std::vector<double> taskspace_to_psm2_vec = std::vector<double>(7, 0.0);
    if(!n.getParam("/taskspace_to_PSM2_tr", taskspace_to_psm2_vec))
        ROS_WARN("%s Parameter /taskspace_to_PSM2_tr is not set. This parameter is required"
                         "if PSM2 is used.",ros::this_node::getName().c_str());
    conversions::VectorToKDLFrame(taskspace_to_psm2_vec, taskspace_to_psm2_tr);

    //--------
    // PSM1 pose subscriber.
    std::string psm1_pose_topic_name = "/dvrk/PSM1/position_cartesian_current";
    psm1_pose_sub =
            n.subscribe(psm1_pose_topic_name, 2,
                        &OverlayGraphics::Tool1PoseCallback, this);
    ROS_INFO("%s [SUBSCRIBERS] Tool 1 pose will be read from topic '%s'",
             ros::this_node::getName().c_str(),
             psm1_pose_topic_name.c_str());

    //--------
    // PSM2 pose subscriber.
    std::string psm2_pose_topic_name = "/dvrk/PSM2/position_cartesian_current";
    psm2_pose_sub =
            n.subscribe(psm2_pose_topic_name, 2,
                        &OverlayGraphics::Tool2PoseCallback, this);
    ROS_INFO("%s [SUBSCRIBERS] Tool 2 pose will be read from topic '%s'",
             ros::this_node::getName().c_str(),
             psm2_pose_topic_name.c_str());


    //--------
    // PSM2 pose subscriber.
    ac_path_subscriber = n.subscribe("/ac_path", 1, &OverlayGraphics::ACPathCallback, this);

    ac_pose_desired_right_subscriber = n.subscribe("/PSM2/tool_pose_desired", 1,
                                             &OverlayGraphics::ACPoseDesiredRightCallback, this);
    ac_pose_desired_left_subscriber = n.subscribe("/PSM1/tool_pose_desired", 1,
                                             &OverlayGraphics::ACPoseDesiredRightCallback, this);

    // advertise publishers
//    std::string board_to_cam_pose_topic_name;
//    if (!n.getParam("board_to_cam_pose_topic_name", board_to_cam_pose_topic_name))
//        board_to_cam_pose_topic_name = "board_to_camera";
//
//    pub_board_to_cam_pose = n.advertise<geometry_msgs::PoseStamped>(board_to_cam_pose_topic_name, 1, 0);
//    ROS_INFO("Publishing board to camera pose on '%s'", n.resolveName(board_to_cam_pose_topic_name).c_str());

}



void OverlayGraphics::ImageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_right_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_right_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void OverlayGraphics::ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_left_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_left_image = true;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void OverlayGraphics::LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam_l);
    conversions::KDLFrameToRvectvec(pose_cam_l, cam_rvec_l, cam_tvec_l);

    // If we don't have the transforms between the cameras
    if(num_cam_pose_publishers==1) {
        pose_cam_r = left_cam_to_right_cam_tr * pose_cam_l ;
        conversions::KDLFrameToRvectvec(pose_cam_r, cam_rvec_r, cam_tvec_r);
    }
}

void OverlayGraphics::RightCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam_r);
    conversions::KDLFrameToRvectvec(pose_cam_r, cam_rvec_r, cam_tvec_r);

}

void OverlayGraphics::Tool1PoseCallback(
        const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Convert message to KDL
    KDL::Frame frame_temp;
    tf::poseMsgToKDL(msg->pose, frame_temp);

    // take the robot end-effector pose to the task space coordinate frame
    pose_tool1 = taskspace_to_psm1_tr.Inverse() * frame_temp;

}

void OverlayGraphics::Tool2PoseCallback(
        const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Convert message to KDL
    KDL::Frame frame_temp;
    tf::poseMsgToKDL(msg->pose, frame_temp);

    // take the robot end-effector pose to the task space coordinate frame
    pose_tool2.p = taskspace_to_psm2_tr.M.Inverse() * (frame_temp.p - taskspace_to_psm2_tr.p);
//    pose_tool2 = taskspace_to_psm2_tr.Inverse() * frame_temp;

}

void OverlayGraphics::ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg){

    for (int n_point = 0; n_point < msg->poses.size(); ++n_point) {
        ac_path.push_back(cv::Point3d(msg->poses[n_point].position.x,
                                      msg->poses[n_point].position.y,
                                      msg->poses[n_point].position.z));
    }
    std::cout << "Received AC_path of " << ac_path.size()<<" points. First point is at "<<
              ac_path[0].x << ", " <<  ac_path[0].y<< ", " << ac_path[0].z<<
                                                           std::endl;
}


void OverlayGraphics::ACPoseDesiredLeftCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    // Convert message to KDL
    tf::poseMsgToKDL(msg->pose, pose_desired_l);
}

void OverlayGraphics::ACPoseDesiredRightCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    // Convert message to KDL
    tf::poseMsgToKDL(msg->pose, pose_desired_r);
}


cv::Mat& OverlayGraphics::ImageLeft(ros::Duration timeout) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_left_.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time)
            ROS_WARN("Timeout: No new left Image.");
    }
    new_left_image = false;
    return image_left_;
}



cv::Mat& OverlayGraphics::ImageRight(ros::Duration timeout) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_right_.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new right Image.");
        }
    }
    new_right_image = false;
    return image_right_;
}



void VisualUtils::SwitchFullScreen(const std::string window_name) {

    if (cvGetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN) ==
        CV_WINDOW_NORMAL)
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    else
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

}