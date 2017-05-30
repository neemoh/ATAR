//
// Created by charm on 4/17/17.
//

#include "OverlayROSConfig.h"

#include <utils/Conversions.hpp>
#include <pwd.h>
#include "BuzzWireTask.h"
#include "KidneyTask.h"

OverlayROSConfig::OverlayROSConfig(std::string node_name)
        : n(node_name)
{

    // assign the callback functions
    pose_current_tool_callbacks[0] = &OverlayROSConfig::Tool1PoseCurrentCallback;
    pose_current_tool_callbacks[1] = &OverlayROSConfig::Tool2PoseCurrentCallback;
    //    twist_current_tool_callbacks[0] = &OverlayROSConfig::Tool1TwistCallback;
    //    twist_current_tool_callbacks[1] = &OverlayROSConfig::Tool2TwistCallback;

    it = new image_transport::ImageTransport(n);

    SetupROS();



}



void OverlayROSConfig::ReadCameraParameters(const std::string file_path,
                                            cv::Mat &camera_matrix,
                                            cv::Mat &camera_distortion) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> camera_distortion;

    // check if we got something
    if(camera_matrix.empty()){
        ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(camera_distortion.empty()){
        ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }


}



//-----------------------------------------------------------------------------------
// SetupROS
//-----------------------------------------------------------------------------------

void OverlayROSConfig::SetupROS() {

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    bool all_required_params_found = true;
    // Loop frequency
    n.param<double>("desired_pose_update_frequency", desired_pose_update_freq, 100);
    ROS_INFO("The desired pose will be updated at '%f'", desired_pose_update_freq);

    n.param<bool>("enable_guidance", with_guidance, true);
    ROS_INFO("Starting the BuzzWire task with guidance: %s",
             with_guidance ? "true" : "false");


    if (n.getParam("stl_files_dir", stl_files_dir)) {
        ROS_INFO("stl files will be loaded from: %s", stl_files_dir.c_str());
    } else
        ROS_ERROR(
                "Parameter '%s' is required. ",
                n.resolveName("stl_files_dir").c_str());

    // ------------------------------------- IMAGES -----------------------------------------

    // ------- load the intrinsic calibration files
    // get home directory
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;

    std::string left_cam_name;
    if (n.getParam("left_cam_name", left_cam_name)) {

        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << left_cam_name << "_intrinsics.yaml";
        ReadCameraParameters(path.str(), camera_matrix[0],
                             camera_distortion[0]);
    } else
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.yaml",
                n.resolveName("left_cam_name").c_str());

    std::string right_cam_name;
    if (n.getParam("right_cam_name", right_cam_name)) {
        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << right_cam_name << "_intrinsics.yaml";
        ReadCameraParameters(path.str(), camera_matrix[1],
                             camera_distortion[1]);
    } else
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.yaml",
                n.resolveName("right_cam_name").c_str());


    //--------
    // Left image subscriber
    std::string left_image_topic_name = "/camera/left/image_color";;
    if (n.getParam("left_image_topic_name", left_image_topic_name))
        ROS_INFO(
                "[SUBSCRIBERS] Left camera images will be read from topic '%s'",
                left_image_topic_name.c_str());
    image_subscribers[0] = it->subscribe(
            left_image_topic_name, 1, &OverlayROSConfig::ImageLeftCallback,
            this);

    //--------
    // Left image subscriber.
    std::string right_image_topic_name = "/camera/right/image_color";
    if (n.getParam("right_image_topic_name", right_image_topic_name))
        ROS_INFO(
                "[SUBSCRIBERS] Right camera images will be read from topic '%s'",
                right_image_topic_name.c_str());
    image_subscribers[1] = it->subscribe(
            right_image_topic_name, 1, &OverlayROSConfig::ImageRightCallback,
            this);

    //-------- KEPT FOR THE OLD OVERLAY NODE TO WORK THE NEW NODE HAS JUST ONE PUBLISHER
    // publishers for the overlayed images
    publisher_overlayed[0] = it->advertise("left/image_color", 1);
    publisher_overlayed[1] = it->advertise("right/image_color", 1);

    publisher_stereo_overlayed = it->advertise("stereo/image_color", 1);

    // ------------------------------------- CAM POSES -----------------------------------------

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
        std::stringstream left_cam_pose_topic_name;
        left_cam_pose_topic_name << std::string("/")
                                 << left_cam_name << "/world_to_camera_transform";
        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                left_cam_pose_topic_name.str(), ros::Duration(4))) {
            ROS_WARN("Topic '%s' is not publishing.",
                     left_cam_pose_topic_name.str().c_str());
        } else
            ROS_INFO(
                    "[SUBSCRIBERS] Left camera pose will be read from topic '%s'",
                    left_cam_pose_topic_name.str().c_str());

        subscriber_camera_pose_left = n.subscribe(
                left_cam_pose_topic_name.str(), 1,
                &OverlayROSConfig::LeftCamPoseCallback,
                this);


        if(num_cam_pose_publishers == 2){
            // Right camera pose will be needed only if the fixed transformation between right and
            // left camera is not provided as a parameter.
            // Right image pose subscriber. Get the topic name parameter and make sure it is being published
            std::stringstream right_cam_pose_topic_name;

            right_cam_pose_topic_name << std::string("/")
                                      << left_cam_name << "/world_to_camera_transform";
            // if the topic name is found, check if something is being published on it
            if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                    right_cam_pose_topic_name.str(), ros::Duration(4)))
                ROS_WARN("Topic '%s' is not being published yet.",
                         right_cam_pose_topic_name.str().c_str());
            else
                ROS_INFO("[SUBSCRIBERS] Right camera pose will be read from topic '%s'",
                         right_cam_pose_topic_name.str().c_str());

            subscriber_camera_pose_right = n.subscribe(
                    right_cam_pose_topic_name.str(), 1, &OverlayROSConfig::RightCamPoseCallback, this);


        }
    }


    // ------------------------------------- Clutches---------------------------------------

    subscriber_foot_pedal_clutch = n.subscribe("/dvrk/footpedals/camera", 1,
                                               &OverlayROSConfig::FootSwitchCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /dvrk/footpedals/camera");




    // ------------------------------------- TOOLS -----------------------------------------
    n.param<int>("number_of_arms", n_arms, 1);
    if(n_arms<1 || n_arms>2)
        throw std::runtime_error("number_of_arms must be 1 or 2");
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

//    publisher_tool_pose_desired = new ros::Publisher[(uint)n_arms];
    subscriber_tool_current_pose = new ros::Subscriber[(uint)n_arms];
    publisher_ac_params = new ros::Publisher[(uint)n_arms];

    //    // Twists not needed for now
    //    publisher_twist_current_tool = new ros::Publisher[(uint)n_arms];
    //    subscriber_twist_current_tool = new ros::Subscriber[(uint)n_arms];

    std::string slave_names[n_arms];
    std::string master_names[n_arms];
    //std::string master_names[n_arms];
    std::string check_topic_name;

    for(int n_arm = 0; n_arm<n_arms; n_arm++){

        //getting the name of the arms
        std::stringstream param_name;
        param_name << std::string("slave_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), slave_names[n_arm]);


        param_name.str("");
        param_name << std::string("master_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), master_names[n_arm]);

        // the current pose of the tools (slaves)
        param_name.str("");
        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/position_cartesian_current";
        subscriber_tool_current_pose[n_arm] = n.subscribe(param_name.str(), 1,
                                                          pose_current_tool_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());
        // we will later check to see if something is publishing on the current slave pose
        check_topic_name = param_name.str();

//        // The desired point according to the active constraint geometry definition
//        param_name.str("");
//        param_name << std::string("/")<< slave_names[n_arm] << "/tool_pose_desired";
//        publisher_tool_pose_desired[n_arm] = n.advertise<geometry_msgs::PoseStamped>(
//                param_name.str().c_str(), 1 );
//        ROS_INFO("Will publish on %s", param_name.str().c_str());

        // Publishing the active constraint parameters that may change during the task
        param_name.str("");
        param_name << std::string("/")<< master_names[n_arm] << "/active_constraint_param";
        publisher_ac_params[n_arm] = n.advertise<custom_msgs::ActiveConstraintParameters>(
                param_name.str().c_str(), 1 );
        ROS_INFO("Will publish on %s", param_name.str().c_str());

        // current twists Not used for now
        //        param_name.str("");
        //        param_name << std::string("/") << slave_names[n_arm] << "/twist_tool_current";
        //        publisher_twist_current_tool[n_arm] = n.advertise<geometry_msgs::TwistStamped>(
        //                param_name.str().c_str(), 1 );
        //        ROS_INFO("Will publish on %s", param_name.str().c_str());
        //
        //
        //        param_name.str("");
        //        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/twist_body_current";
        //        subscriber_twist_current_tool[n_arm] = n.subscribe(param_name.str(), 1,
        //                                                           twist_current_tool_callbacks[n_arm], this);
        //        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        // the transformation from the coordinate frame of the slave (RCM) to the task coordinate
        // frame.
        param_name.str("");
        param_name << (std::string)"/calibrations/task_frame_to_" << slave_names[n_arm] << "_frame";
        std::vector<double> vect_temp = std::vector<double>(7, 0.0);
        if(n.getParam(param_name.str(), vect_temp)){
            conversions::VectorToKDLFrame(vect_temp, slave_frame_to_task_frame[n_arm]);
            // param is from task to RCM, we want the inverse
            slave_frame_to_task_frame[n_arm] = slave_frame_to_task_frame[n_arm].Inverse();
        }
        else
            ROS_ERROR("Parameter %s is needed.", param_name.str().c_str());
    }

    // Publisher for the task state
    std::string task_state_topic_name = "/task_state";
    publisher_task_state = n.advertise<custom_msgs::TaskState>(
            task_state_topic_name.c_str(), 1);
    ROS_INFO("Will publish on %s", task_state_topic_name.c_str());

    subscriber_recording_events = n.subscribe(
            "/recording_events", 1, &OverlayROSConfig::RecordingEventsCallback,
            this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /task_state");

    n.param<bool>("show_reference_frames", show_reference_frames, true);

    // advertise publishers
    //    std::string board_to_cam_pose_topic_name;
    //    if (!n.getParam("board_to_cam_pose_topic_name", board_to_cam_pose_topic_name))
    //        board_to_cam_pose_topic_name = "board_to_camera";
    //
    //    pub_board_to_cam_pose = n.advertise<geometry_msgs::PoseStamped>(board_to_cam_pose_topic_name, 1, 0);
    //    ROS_INFO("Publishing board to camera pose on '%s'", n.resolveName(board_to_cam_pose_topic_name).c_str());
    if (!all_required_params_found)
        throw std::runtime_error("ERROR: some required topics are not set");
}



void OverlayROSConfig::ImageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_from_ros[1] = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_right_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void OverlayROSConfig::ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_from_ros[0] = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_left_image = true;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void OverlayROSConfig::LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam[0]);
    conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec[0], cam_tvec[0]);

    // If we don't have the transforms between the cameras
    if(num_cam_pose_publishers==1) {
        pose_cam[1] = left_cam_to_right_cam_tr * pose_cam[0] ;
        conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec[1], cam_tvec[1]);
    }
}


void OverlayROSConfig::RightCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam[1]);
    conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec[1], cam_tvec[1]);
}




// Reading the pose of the slaves and take them to task space

void OverlayROSConfig::Tool1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    pose_current_tool[0] =  slave_frame_to_task_frame[0] * frame;

}

void OverlayROSConfig::Tool2PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    pose_current_tool[1] =  slave_frame_to_task_frame[1] * frame;
}


void OverlayROSConfig::FootSwitchCallback(const sensor_msgs::Joy & msg){
    foot_switch_pressed = (bool)msg.buttons[0];
}

bool OverlayROSConfig::GetNewImages( cv::Mat images[]) {

    if(new_left_image && new_right_image) {
        image_from_ros[0].copyTo(images[0]);
        image_from_ros[1].copyTo(images[1]);
        new_left_image = false;
        new_right_image = false;
        return true;
    }

    return false;

}

void OverlayROSConfig::LockAndGetImages(ros::Duration timeout, cv::Mat images[]) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_from_ros[0].empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time)
            ROS_WARN("Timeout: No new left Image. Trying again...");
    }
    image_from_ros[0].copyTo(images[0]);

    new_left_image = false;

    while(image_from_ros[1].empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new right Image. Trying again...");
        }
    }
    image_from_ros[1].copyTo(images[1]);

    new_right_image = false;

}


void OverlayROSConfig::PublishACtiveConstraintParameters(
        const custom_msgs::ActiveConstraintParameters &ac_params) {

    publisher_ac_params[0].publish(ac_params);
    if(n_arms==2)
        publisher_ac_params[0].publish(ac_params);

}

void OverlayROSConfig::PublishTaskState(custom_msgs::TaskState msg) {
    publisher_task_state.publish(msg);

}

void OverlayROSConfig::RecordingEventsCallback(const std_msgs::CharConstPtr
                                               &msg) {

    recording_event = msg->data;
    new_recording_event = true;

    switch(msg->data){
        case 'r':
            task_ptr->ResetTask();
            break;

        case 'd':
            task_ptr->ResetCurrentAcquisition();
            break;

        default:
            break;
    }

}

void OverlayROSConfig::StartTask(const uint task_id) {

    // create the task
    if(task_id ==1){

        // allocate anew dynamic task
        std::cout << "Starting new task. "<< std::endl;
        task_ptr   = new BuzzWireTask(stl_files_dir, show_reference_frames,
                                      (bool) (n_arms - 1), with_guidance);
        // assign the tool pose pointers
        ros::spinOnce();
        task_ptr->SetCurrentToolPosePointer(pose_current_tool[0], 0);
        task_ptr->SetCurrentToolPosePointer(pose_current_tool[1], 1);

        // bind the haptics thread
        haptics_thread = boost::thread(boost::bind(
                &VTKTask::FindAndPublishDesiredToolPose, task_ptr));

    }
    else if(task_id ==2){

        std::cout << "Starting new task. "<< std::endl;
        task_ptr   = new KidneyTask(stl_files_dir, false,
                                    (bool) (n_arms - 1), with_guidance);
        // assign the tool pose pointers
        ros::spinOnce();
        task_ptr->SetCurrentToolPosePointer(pose_current_tool[0], 0);
        task_ptr->SetCurrentToolPosePointer(pose_current_tool[1], 1);

        // bind the haptics thread
        haptics_thread = boost::thread(boost::bind(
                &VTKTask::FindAndPublishDesiredToolPose, task_ptr));

    }

}

void OverlayROSConfig::StopTask() {

    std::cout << "interrupting haptics thread. "<< std::endl;
    haptics_thread.interrupt();
    ros::Rate sleep(50);
    sleep.sleep();
    delete task_ptr;
}


void VisualUtils::SwitchFullScreen(const std::string window_name) {

    if (cvGetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN) ==
        CV_WINDOW_NORMAL)
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    else
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

}


