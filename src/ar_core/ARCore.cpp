//
// Created by charm on 4/17/17.
//

#include "ARCore.h"
#include <custom_conversions/Conversions.h>
#include <pwd.h>
#include <src/arm_to_world_calibration/ArmToWorldCalibration.h>
#include "src/ar_core/tasks/TaskBuzzWire.h"
#include "src/ar_core/tasks/TaskKidney.h"
#include "src/ar_core/tasks/TaskDeformable.h"
#include "src/ar_core/tasks/TaskBulletTest.h"
#include "ControlEvents.h"
#include "src/ar_core/tasks/TaskNeedle.h"
#include "src/ar_core/tasks/TaskHook.h"
#include "src/ar_core/tasks/TaskSteadyHand.h"

// -----------------------------------------------------------------------------
ARCore::ARCore(std::string node_name)
    : n(node_name), running_task_id(0), task_ptr(NULL)
{

    // assign the callback functions
    pose_current_tool_callbacks[0] = &ARCore::Tool1PoseCurrentCallback;
    pose_current_tool_callbacks[1] = &ARCore::Tool2PoseCurrentCallback;
    gripper_callbacks[0] = &ARCore::Tool1GripperCurrentCallback;
    gripper_callbacks[1] = &ARCore::Tool2GripperCurrentCallback;

    it = new image_transport::ImageTransport(n);

    SetupROSandGetParameters();

    SetupGraphics();

}

//------------------------------------------------------------------------------
void ARCore::SetupROSandGetParameters() {

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    bool all_required_params_found = true;

    // Loop frequency
    n.param<double>("desired_pose_update_frequency",
                    haptic_loop_rate, 100);

    n.param<bool>("enable_guidance", with_guidance, true);
    ROS_INFO("Starting the BuzzWire task with guidance: %s",
             with_guidance ? "true" : "false");


    if (n.getParam("mesh_files_dir", mesh_files_dir)) {
        ROS_INFO("stl files will be loaded from: %s", mesh_files_dir.c_str());
    } else {
        ROS_ERROR(
            "Parameter '%s' is required. ",
            n.resolveName("mesh_files_dir").c_str());
        all_required_params_found = false;
    }

    // ---------------------------- CAM INTRINSICS  ----------------------------
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
    } else {
        ROS_ERROR(
            "Parameter '%s' is required. Place the intrinsic calibration "
                "file of each camera in ~/.ros/camera_info/ named as "
                "<cam_name>_intrinsics.yaml",
            n.resolveName("left_cam_name").c_str());
        all_required_params_found = false;
    }
    std::string right_cam_name;
    if (n.getParam("right_cam_name", right_cam_name)) {
        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << right_cam_name << "_intrinsics.yaml";
        ReadCameraParameters(path.str(), camera_matrix[1],
                             camera_distortion[1]);
    } else {
        ROS_ERROR(
            "Parameter '%s' is required. Place the intrinsic calibration "
                "file of each camera in ~/.ros/camera_info/ named as "
                "<cam_name>_intrinsics.yaml",
            n.resolveName("right_cam_name").c_str());
        all_required_params_found = false;
    }

    // ------------------------------------- IMAGES ----------------------------
    // Left image subscriber
    std::string left_image_topic_name = "/camera/left/image_color";;
    if (n.getParam("left_image_topic_name", left_image_topic_name))
        ROS_INFO(
            "[SUBSCRIBERS] Left cam images from '%s'",
            left_image_topic_name.c_str());
    image_subscribers[0] = it->subscribe(
        left_image_topic_name, 1, &ARCore::ImageLeftCallback,
        this);

    //--------
    // Left image subscriber.
    std::string right_image_topic_name = "/camera/right/image_color";
    if (n.getParam("right_image_topic_name", right_image_topic_name))
        ROS_INFO(
            "[SUBSCRIBERS] Right cam images from '%s'",
            right_image_topic_name.c_str());
    image_subscribers[1] = it->subscribe(
        right_image_topic_name, 1, &ARCore::ImageRightCallback,
        this);

    // KEPT FOR THE OLD OVERLAY NODE TO WORK THE NEW NODE HAS JUST ONE PUBLISHER
    // publishers for the overlayed images
    publisher_overlayed[0] = it->advertise("left/image_color", 1);
    publisher_overlayed[1] = it->advertise("right/image_color", 1);

    publisher_stereo_overlayed = it->advertise("stereo/image_color", 1);


    // -------------------------------- CAM POSES ------------------------------
    // We need to know the pose of the cameras with respect to the world frame
    // coordinates. If the camera or the markers move the pose should be
    // estimated by a node and here we subscribe to that topic. If on the
    // other hand no cam/marker motion is involved the fixed pose of the left
    // camera is read as a static parameter and the right one is calculated
    // from the fixed tr hard coded here. If you are not using the dvrk
    // endoscopic camera you need to estimate the left to right cam transform
    // yourself and put it here:
    std::vector<double> l_r_cams = {-0.00538475, 0.000299458, -0.000948875,
        0.0016753, -0.00112252, -0.00358978, 0.999992};
    conversions::VectorToKDLFrame(l_r_cams, left_cam_to_right_cam_tr);

    // we first try to read the poses as parameters and later update the
    // poses if new messages are arrived on the topics
    std::vector<double> temp_vec = std::vector<double>( 7, 0.0);
    // left cam pose as parameter
    if (n.getParam("/calibrations/world_frame_to_left_cam_frame", temp_vec)) {
        conversions::VectorToKDLFrame(temp_vec, pose_cam[0]);
        conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec_curr[0], cam_tvec_curr[0]);
        cam_tvec_avg[0] = cam_tvec_curr[0];
        cam_rvec_avg[0] = cam_rvec_curr[0];

        // right cam
        pose_cam[1] = left_cam_to_right_cam_tr * pose_cam[0];
        conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec_curr[1],
                                        cam_tvec_curr[1]);
        cam_tvec_avg[1] = cam_tvec_curr[1];
        cam_rvec_avg[1] = cam_rvec_curr[1];

        new_cam_pose[0] = true;
        new_cam_pose[1] = true;
    }

    // now we set up the subscribers
    std::stringstream topic_name;
    topic_name << std::string("/") << left_cam_name
               << "/world_to_camera_transform";
    ROS_INFO("[SUBSCRIBERS] Left came pose from '%s'",
             topic_name.str().c_str());
    sub_cam_pose_left = n.subscribe(
        topic_name.str(), 1, &ARCore::LeftCamPoseCallback, this);

    topic_name.str("");
    topic_name << std::string("/") << right_cam_name
               << "/world_to_camera_transform";
    // if the topic name is found, check if something is being published on it
    ROS_INFO("[SUBSCRIBERS] Right cam pose from '%s'",
             topic_name.str().c_str());
    sub_cam_pose_right = n.subscribe(
        topic_name.str(), 1, &ARCore::RightCamPoseCallback, this);


    // ------------------------------------- Clutches---------------------------
    sub_foot_pedal_clutch = n.subscribe( "/dvrk/footpedals/camera", 1,
                                         &ARCore::FootSwitchCallback, this);
    ROS_INFO("[SUBSCRIBERS] /dvrk/footpedals/camera");

    // ------------------------------------- TOOLS -----------------------------
    n.param<int>("number_of_arms", n_arms, 1);
    if(n_arms<1 || n_arms>2)
        throw std::runtime_error("number_of_arms must be 1 or 2");
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

    //    publisher_tool_pose_desired = new ros::Publisher[(uint)n_arms];
    subtool_current_pose = new ros::Subscriber[(uint)n_arms];
    subtool_current_gripper = new ros::Subscriber[(uint)n_arms];
//    publisher_ac_params = new ros::Publisher[(uint)n_arms];


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
        param_name << std::string("/dvrk/") <<slave_names[n_arm]
                   << "/position_cartesian_current";
        subtool_current_pose[n_arm] =
            n.subscribe(param_name.str(), 1,
                        pose_current_tool_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] %s", param_name.str().c_str());
        // we will later check to see if something is publishing on the
        // current slave pose
        check_topic_name = param_name.str();

        // the current pose of the tools (slaves)
        param_name.str("");
        param_name << std::string("/dvrk/") <<master_names[n_arm]
                   << "/gripper_position_current";
        subtool_current_gripper[n_arm] =
            n.subscribe(param_name.str(), 1, gripper_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] %s", param_name.str().c_str());
        // we will later check to see if something is publishing on the
        // current slave pose
        check_topic_name = param_name.str();

        // Publishing the active constraint parameters that may change during
        // the task
//        param_name.str("");
//        param_name << std::string("/atar/")<< master_names[n_arm]
//                   << "/active_constraint_param";
//        publisher_ac_params[n_arm] =
//            n.advertise<custom_msgs::ActiveConstraintParameters>(
//                param_name.str().c_str(), 1 );
//        ROS_INFO("Will publish on %s", param_name.str().c_str());

        n.param<bool>("AR_mode", ar_mode, false);
        ROS_INFO("AR mode: %s",
                 ar_mode ? "true" : "false");

        // the transformation from the coordinate frame of the slave (RCM)
        // to the task coordinate frame is needed in AR mode.
        if(ar_mode) {
            param_name.str("");
            param_name << (std::string) "/calibrations/world_frame_to_" <<
                       slave_names[n_arm] << "_frame";
            std::vector<double> vect_temp = std::vector<double>(7, 0.0);
            if (n.getParam(param_name.str(), vect_temp)) {
                conversions::VectorToKDLFrame(vect_temp,
                                              slave_frame_to_world_frame[n_arm]);
                // param is from task to RCM, we want the inverse
                slave_frame_to_world_frame[n_arm] =
                    slave_frame_to_world_frame[n_arm].Inverse();
            } else
                ROS_INFO("Parameter %s was not found. Arm to world calibration "
                             "must be performed.",
                         param_name.str().c_str());
        }
        else{
            // in VR mode the slaves are the masters, that is, we don't use
            // the slaves. Instead we read the pose of the masters and to
            // register the motions correctly we need the transform from the
            // base of the masters to the console displays (i.e. to camera
            // images).
            param_name.str("");
            param_name << (std::string) "/calibrations/" <<
                       slave_names[n_arm] << "_frame_to_image_frame";
            std::vector<double> vect_temp = std::vector<double>(7, 0.0);
            if (n.getParam(param_name.str(), vect_temp)) {
                KDL::Frame mtm_to_image;
                conversions::VectorToKDLFrame(vect_temp, mtm_to_image);

                // param is from task to RCM, we want the inverse
                slave_frame_to_world_frame[n_arm] = mtm_to_image *
                    pose_cam[0];
                //make sure translation is null
                slave_frame_to_world_frame[n_arm].p = KDL::Vector(0.0, 0.0,0.0);

            } else {
                ROS_ERROR("Parameter %s was not found. This is needed in VR "
                              "mode.",
                          param_name.str().c_str());
                all_required_params_found = false;
            }
        }

    }

    // Publisher for the task state
    std::string task_state_topic_name = "/atar/task_state";
    publisher_task_state = n.advertise<custom_msgs::TaskState>(
        task_state_topic_name.c_str(), 1);
    ROS_INFO("Will publish on %s", task_state_topic_name.c_str());

    subscriber_control_events = n.subscribe(
        "/atar/control_events", 1, &ARCore::ControlEventsCallback,
        this);
    ROS_INFO("[SUBSCRIBERS] /control_events");


    if (!all_required_params_found)
        throw std::runtime_error("ERROR: some required parameters are not set");
}


// -----------------------------------------------------------------------------
void ARCore::SetupGraphics() {

    n.param<bool>("publish_overlayed_images", publish_overlayed_images, true);
    ROS_INFO("Rendered Images will be grabbed from gpu and published: %s",
             publish_overlayed_images ? "true" : "false");

    n.param<bool>("one_window_mode", one_window_mode, false);
    ROS_INFO("Rendered Images will be shown in a single window: %s",
             one_window_mode ? "true" : "false");

    bool offScreen_rendering, with_shadows;
    n.param<bool>("with_shadows", with_shadows, false);
    ROS_INFO("Shadows Generation: %s",
             with_shadows ? "true" : "false");

    n.param<bool>("offScreen_rendering", offScreen_rendering, false);
    ROS_INFO("offScreen_rendering: %s",
             offScreen_rendering ? "true" : "false");

    n.param<bool>("show_reference_frames", show_reference_frames, true);

    std::vector<int> windows_position(4, 0);
    n.getParam("windows_position", windows_position);

    // Create the window for the video feed if we publish the images
    if (publish_overlayed_images) {
        if (one_window_mode) {
            cv_window_names[0] = "Augmented Stereo";
            cvNamedWindow(cv_window_names[0].c_str(), CV_WINDOW_NORMAL);

        } else {
            cv_window_names[0] = "Augmented Left";
            cv_window_names[1] = "Augmented Right";
            cvNamedWindow(cv_window_names[0].c_str(), CV_WINDOW_NORMAL);
            cvNamedWindow(cv_window_names[1].c_str(), CV_WINDOW_NORMAL);
        }
    }


    graphics = new Rendering(ar_mode, 2 - (uint) one_window_mode, with_shadows,
                             offScreen_rendering, windows_position);

    // in case camera poses are set as parameters
    graphics->SetWorldToCameraTransform(cam_rvec_curr, cam_tvec_curr);

    // set the intrinsics and configure the background image
    graphics->SetCameraIntrinsics(camera_matrix);

    // in ar mode we read real camera images and show them as the background
    // of our rendering
    if (ar_mode){
        cv::Mat cam_images[2];
        LockAndGetImages(ros::Duration(1), cam_images);
        graphics->ConfigureBackgroundImage(cam_images);
        graphics->SetEnableBackgroundImage(true);
    }

//    graphics->Render();

}
// -----------------------------------------------------------------------------
bool ARCore::UpdateWorld() {

    // -------------------------------------------------------------------------
    // Update cam poses if needed
    cv::Vec3d cam_rvec[2];    cv::Vec3d cam_tvec[2];
    // in AR mode we would like to change the virtual cam poses as the real
    // camera moves. In VR mode this returns the fixed cam poses only once.
    if(GetNewCameraPoses(cam_rvec, cam_tvec))
        graphics->SetWorldToCameraTransform(cam_rvec, cam_tvec);

    // -------------------------------------------------------------------------
    if (control_event == CE_EXIT) {// Esc
        graphics->RemoveAllActorsFromScene();
        Cleanup();
        return false;
    }

    if(new_task_event)
        HandleTaskEvent();

    cv::Mat cam_images[2];
    if(GetNewImages(cam_images) || !ar_mode) {

        // Time performance debug
        //ros::Time start =ros::Time::now();

        // update the moving actors
        if(task_ptr)
            task_ptr->UpdateActors();

        if(ar_mode)
            // update the camera images
            graphics->UpdateBackgroundImage(cam_images);

        // update  view angle (in case window changes size)
        if(ar_mode)
            graphics->UpdateCameraViewForActualWindowSize();

        // Render!
        graphics->Render();

        // arm calibration
        if(control_event== CE_CALIB_ARM1)
            StartArmToWorldFrameCalibration(0);

        // Copy the rendered image to memory, show it and/or publish it.
        if(publish_overlayed_images)
            PublishRenderedImages();

        // publish the active constraint parameters if needed
        if(task_ptr) {
            if (task_ptr->IsACParamChanged()) {
                PublishACtiveConstraintParameters(
                    task_ptr->GetACParameters());
            }
            // publish the task state
            PublishTaskState(task_ptr->GetTaskStateMsg());
        }
        // check time performance
        // std::cout <<  "it took: " <<
        // (ros::Time::now() - start).toNSec() /1000000 << std::endl;

    } // if new image

    // if no task is running we need to spin our self
    if(!task_ptr)
        ros::spinOnce();

    return true;
}

// -----------------------------------------------------------------------------
void ARCore::HandleTaskEvent() {

    if (new_task_event){

        ROS_INFO("Task %d Selected", running_task_id);

        //close tasks if it was already running
        if(task_ptr) {
            graphics->RemoveAllActorsFromScene();
            DeleteTask();
        }

        StartTask(running_task_id);

        if(task_ptr)
            // If task initialized, add the task actors to the graphics
            graphics->AddActorsToScene(task_ptr->GetActors());

        new_task_event = false;

    }

}


// -----------------------------------------------------------------------------
void ARCore::StartTask(const uint task_id) {

    // create the task
    if(task_id ==1){
        // allocate anew dynamic task
        ROS_DEBUG("Starting new TaskKidney. ");
        task_ptr   = new TaskKidney(mesh_files_dir, show_reference_frames,
                                    (bool) (n_arms - 1), with_guidance);
    }
    else if(task_id ==2){

        // getting the names of the slaves
        std::string slave_names[n_arms];
        for(int n_arm = 0; n_arm<n_arms; n_arm++) {

            //getting the name of the arms
            std::stringstream param_name;
            param_name << std::string("slave_") << n_arm + 1 << "_name";
            n.getParam(param_name.str(), slave_names[n_arm]);
        }

        // starting the task
        ROS_DEBUG("Starting new BuzzWireTask task. ");
        task_ptr   = new TaskSteadyHand(
            mesh_files_dir, show_reference_frames, (bool) (n_arms - 1),
            with_guidance, haptic_loop_rate, slave_names, slave_frame_to_world_frame
        );
    }
    else if(task_id ==3){
        ROS_DEBUG("Starting new TaskNeedle. ");
        task_ptr   = new TaskNeedle(mesh_files_dir, show_reference_frames,
                                    (bool) (n_arms - 1), with_guidance);
    }
    else if(task_id ==4){
        ROS_DEBUG("Starting new TaskBulletTest. ");
        task_ptr   = new TaskBulletTest(mesh_files_dir, show_reference_frames,
                                        (bool) (n_arms - 1), with_guidance);
    }
    else if(task_id ==5){
        ROS_DEBUG("Starting new TaskDeformable . ");
        task_ptr   = new TaskDeformable(mesh_files_dir, show_reference_frames,
                                        (bool) (n_arms - 1), with_guidance);

    }
    else if(task_id ==6) {
        ROS_DEBUG("No task assigned yet. ");
        task_ptr   = new TaskHook(mesh_files_dir, show_reference_frames,
                                  (bool) (n_arms - 1), with_guidance);
    }
    else if(task_id ==7) {
        ROS_DEBUG("No task assigned yet. ");

        // getting the names of the slaves
        std::string slave_names[n_arms];
        for (int n_arm = 0; n_arm < n_arms; n_arm++) {

            //getting the name of the arms
            std::stringstream param_name;
            param_name << std::string("slave_") << n_arm + 1 << "_name";
            n.getParam(param_name.str(), slave_names[n_arm]);
        }

        // starting the task
        ROS_DEBUG("Starting new BuzzWireTask task. ");
        task_ptr = new TaskBuzzWire(
            mesh_files_dir, show_reference_frames, (bool) (n_arms - 1),
            with_guidance, haptic_loop_rate, slave_names,
            slave_frame_to_world_frame
        );
    }
    if(task_ptr) {
        // assign the tool pose pointers
        ros::spinOnce();
        task_ptr->SetCurrentToolPosePointer(pose_current_tool[0], 0);
        task_ptr->SetCurrentToolPosePointer(pose_current_tool[1], 1);

        task_ptr->SetCurrentGripperpositionPointer(gripper_current[0], 0);
        task_ptr->SetCurrentGripperpositionPointer(gripper_current[1], 1);

        task_ptr->UpdateActors();

        // bind the haptics thread
        haptics_thread = boost::thread(boost::bind(
            &VTKTask::FindAndPublishDesiredToolPose, task_ptr));
    }
}

// -----------------------------------------------------------------------------
void ARCore::DeleteTask() {

    ROS_DEBUG("Interrupting haptics thread");
    haptics_thread.interrupt();
    ros::Rate sleep(50);
    sleep.sleep();
    delete task_ptr;
    task_ptr = 0;
}


// -----------------------------------------------------------------------------
void ARCore::LockAndGetImages(ros::Duration timeout,
                                        cv::Mat images[]) {

    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_from_ros[0].empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time)
            ROS_WARN("Timeout: No new left Image. Trying again...");
    }
    image_from_ros[0].copyTo(images[0]);

    new_image[0] = false;

    while(image_from_ros[1].empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new right Image. Trying again...");
        }
    }
    image_from_ros[1].copyTo(images[1]);

    new_image[1] = false;

}

// -----------------------------------------------------------------------------
bool ARCore::GetNewImages( cv::Mat images[]) {

    if(new_image[0] && new_image[1]) {
        image_from_ros[0].copyTo(images[0]);
        image_from_ros[1].copyTo(images[1]);
        new_image[0] = false;
        new_image[1] = false;
        return true;
    }

    return false;

}

// -----------------------------------------------------------------------------
bool ARCore::GetNewCameraPoses(cv::Vec3d cam_rvec_out[2],
                                         cv::Vec3d cam_tvec_out[2]) {

    // // estimate left_to_right_cam transform:

    //left_cam_to_right_cam_tr = pose_cam[1] * pose_cam[0].Inverse();
    //double x,y,z,w;
    //left_cam_to_right_cam_tr.M.GetQuaternion(x, y, z, w);
    ////
    //left_cam_to_right_cam_tr_loop_count++;
    //left_cam_to_right_cam_tr_sum_pos+= left_cam_to_right_cam_tr.p;
    //if(left_cam_to_right_cam_tr_loop_count == 200){
    //    left_cam_to_right_cam_tr_sum_pos=
    //        left_cam_to_right_cam_tr_sum_pos / double(left_cam_to_right_cam_tr_loop_count);
    //    std::cout << "estimated laft to right cam tr: {"
    //              << left_cam_to_right_cam_tr_sum_pos.x() << ", "
    //              << left_cam_to_right_cam_tr_sum_pos.y() << ", "
    //              << left_cam_to_right_cam_tr_sum_pos.z() << ", "
    //              << x << ", "
    //              << y << ", "
    //              << z << ", "
    //              << w << "} "
    //              << std::endl;
    //    left_cam_to_right_cam_tr_loop_count = 0;
    //}

    // if one of the poses is not available estimate the other one through
    // the left to right fixed transform
    if (new_cam_pose[0] && !new_cam_pose[1]) {
        KDL::Frame pose_camZZ;
        pose_camZZ = left_cam_to_right_cam_tr * pose_cam[0];
        conversions::KDLFrameToRvectvec(pose_camZZ, cam_rvec_curr[1],
                                        cam_tvec_curr[1]);
    } else if (!new_cam_pose[0] && new_cam_pose[1]) {
        pose_cam[0] = left_cam_to_right_cam_tr.Inverse() * pose_cam[1];
        conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec_curr[0],
                                        cam_tvec_curr[0]);
    }



    double avg_factor;
    n.param<double>("cam_pose_averaging_factor", avg_factor, 0.5);

    // FIXME change to normal averaging with buffer.
    // populate the out values
    if (new_cam_pose[0] || new_cam_pose[1]) {

        for (int k = 0; k < 2; ++k) {
            if (ar_mode) {
                // average the position to prevent small jitter when the board
                // does not have good visibility
                cam_tvec_avg[k] -= avg_factor * cam_tvec_avg[k];
                cam_tvec_avg[k] += avg_factor * cam_tvec_curr[k];

                // not mathematically legal, but should work...
                cam_rvec_avg[k] -= avg_factor * cam_rvec_avg[k];
                cam_rvec_avg[k] += avg_factor * cam_rvec_curr[k];

                // to prevent jitter due to wrong board pose estimation we discard
                // poses that are too different from the last pose
                if ((cv::norm(cam_tvec_avg[k] - cam_tvec_curr[k]) < 0.05)
                    && (cv::norm(cam_rvec_avg[k] - cam_rvec_curr[k]) < 0.1)) {

                    cam_rvec_out[k] = cam_rvec_avg[k];
                    cam_tvec_out[k] = cam_tvec_avg[k];
                    new_cam_pose[k] = false;
//                    if (k == 1)
//                        return true;
                }
            }
            else{ // ar_mode
                // no averaging is needed in VR mode
                cam_rvec_out[k] = cam_rvec_curr[k];
                cam_tvec_out[k] = cam_tvec_curr[k];
                //new_cam_pose[k] = false;
            }
        }
        return true;
    }

    return false;
}


// -----------------------------------------------------------------------------
void ARCore::PublishACtiveConstraintParameters(
    const custom_msgs::ActiveConstraintParameters ac_params[2]) {

//    publisher_ac_params[0].publish(ac_params[0]);
//    if(n_arms==2)
//        publisher_ac_params[1].publish(ac_params[1]);
}

// -----------------------------------------------------------------------------
void ARCore::PublishTaskState(custom_msgs::TaskState msg) {
    publisher_task_state.publish(msg);

}

// -----------------------------------------------------------------------------
void ARCore::DoArmToWorldFrameCalibration(const uint arm_id) {

    //getting the name of the arms
    std::stringstream param_name;
    std::string slave_name;
    param_name << std::string("slave_") << arm_id + 1 << "_name";
    n.getParam(param_name.str(), slave_name);

    std::stringstream arm_pose_namespace;
    arm_pose_namespace << std::string("/dvrk/") <<slave_name
                       << "/position_cartesian_current";

    std::stringstream cam_pose_namespace;
    std::string left_cam_name;
    n.getParam("left_cam_name", left_cam_name);
    cam_pose_namespace << std::string("/") << left_cam_name
                       << "/world_to_camera_transform";

    std::string cam_image_name_space ;
    n.getParam("left_image_topic_name", cam_image_name_space);

    // putting the calibration point on the corners of the board squares
    // the parameter can be set directly, unless there is the global
    // /calibrations/board_params
    double calib_points_distance = 0.01;
    std::vector<float> board_params = std::vector<float>(5, 0.0);

    if(!n.getParam("calib_points_distance", calib_points_distance)){
        if(n.getParam("/calibrations/board_params", board_params))
            calib_points_distance = board_params[3];

    };

    int num_calib_points;
    n.param<int>("number_of_calibration_points", num_calib_points, 6);

    ArmToWorldCalibration AWC;
    KDL::Frame world_to_arm_frame;
    std::vector<double> calib_point_center
        = {board_params[1]/2 * calib_points_distance
           , board_params[2]/2 * calib_points_distance};

    if(AWC.DoCalibration(
        cam_image_name_space, cam_pose_namespace.str(),
        arm_pose_namespace.str(),
        camera_matrix[0], camera_distortion[0], (uint) num_calib_points,
        calib_points_distance, calib_point_center, world_to_arm_frame
    )){

        // set ros param
        param_name.str("");
        param_name << std::string("/calibrations/world_frame_to_") <<
                   slave_name << "_frame";

        std::vector<double> vec7(7, 0.0);
        conversions::KDLFrameToVector(world_to_arm_frame, vec7);
        n.setParam(param_name.str(), vec7);

        // set output
        slave_frame_to_world_frame[arm_id] = world_to_arm_frame.Inverse();
    }

}

// -----------------------------------------------------------------------------
void ARCore::Cleanup() {

    DeleteTask();
    delete graphics;
}

// -----------------------------------------------------------------------------
void ARCore::StartArmToWorldFrameCalibration(const uint arm_id) {

    ROS_INFO("Starting Arm 1 to world calibration.");
    if(running_task_id>0) {
        // if a task is running first stop it
        graphics->RemoveAllActorsFromScene();
        DeleteTask();
        // then do the calibration
        DoArmToWorldFrameCalibration(arm_id);
        // run the task again
        StartTask((uint)running_task_id);
        graphics->AddActorsToScene(task_ptr->GetActors());
    }
    else // if no task is running just do the calibration
        DoArmToWorldFrameCalibration(arm_id);

    control_event = running_task_id;
}

// -----------------------------------------------------------------------------
void ARCore::PublishRenderedImages() {

    cv::Mat augmented_images[2];

    char key = (char)cv::waitKey(1);
    if (key == 27) // Esc
        ros::shutdown();
    else if (key == 'f')  //full screen
        SwitchFullScreenCV(cv_window_names[0]);

    graphics->GetRenderedImage(augmented_images);
    if(one_window_mode){
        cv::imshow(cv_window_names[0], augmented_images[0]);
        publisher_stereo_overlayed.publish(
            cv_bridge::CvImage(std_msgs::Header(),
                               "bgr8",
                               augmented_images[0])
                .toImageMsg());

    }
    else{
        for (int i = 0; i < 2; ++i) {
            cv::imshow(cv_window_names[i], augmented_images[i]);
            publisher_overlayed[i].publish(
                cv_bridge::CvImage(std_msgs::Header(),
                                   "bgr8",
                                   augmented_images[i])
                    .toImageMsg());
        }
        if (key == 'f')  //full screen
            SwitchFullScreenCV(cv_window_names[1]);
    }

}


// -----------------------------------------------------------------------------
void ARCore::ReadCameraParameters(const std::string file_path,
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

// -----------------------------------------------------------------------------
void ARCore::ImageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_from_ros[1] = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_image[1] = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// -----------------------------------------------------------------------------
void ARCore::ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_from_ros[0] = cv_bridge::toCvCopy(msg, "bgr8")->image;
        new_image[0] = true;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// -----------------------------------------------------------------------------
void ARCore::LeftCamPoseCallback(
    const geometry_msgs::PoseStampedConstPtr & msg)
{
    new_cam_pose[0] = true;
    tf::poseMsgToKDL(msg->pose, pose_cam[0]);
    conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec_curr[0], cam_tvec_curr[0]);

}

void ARCore::RightCamPoseCallback(
    const geometry_msgs::PoseStampedConstPtr & msg)
{
    new_cam_pose[1] = true;
    tf::poseMsgToKDL(msg->pose, pose_cam[1]);
    conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec_curr[1], cam_tvec_curr[1]);
}


// Reading the pose of the slaves and take them to task space
// -----------------------------------------------------------------------------
void ARCore::Tool1PoseCurrentCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    pose_current_tool[0] =  slave_frame_to_world_frame[0] * frame;

}

void ARCore::Tool2PoseCurrentCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    pose_current_tool[1] =  slave_frame_to_world_frame[1] * frame;
}

// -----------------------------------------------------------------------------
// Reading the gripper positions
void ARCore::Tool1GripperCurrentCallback(
    const std_msgs::Float32::ConstPtr &msg) {
    gripper_current[0] =  msg->data;
}

void ARCore::Tool2GripperCurrentCallback(
    const std_msgs::Float32::ConstPtr &msg) {
    gripper_current[1] =  msg->data;

}

// -----------------------------------------------------------------------------
void ARCore::FootSwitchCallback(const sensor_msgs::JoyConstPtr & msg){
    foot_switch_pressed = (bool)msg->buttons[0];
}

// -----------------------------------------------------------------------------
void ARCore::ControlEventsCallback(const std_msgs::Int8ConstPtr
                                             &msg) {

    control_event = msg->data;
    ROS_DEBUG("Received control event %d", control_event);

    switch(control_event){
    case CE_RESET_TASK:
        task_ptr->ResetTask();
        break;

    case CE_RESET_ACQUISITION:
        task_ptr->ResetCurrentAcquisition();
        break;

    case CE_PUBLISH_IMGS_ON:
        publish_overlayed_images = true;
        break;

    case CE_PUBLISH_IMGS_OFF:
        publish_overlayed_images = false;
        break;

    case CE_TOGGLE_FULLSCREEN:
        graphics->ToggleFullScreen();
        break;

    case CE_START_TASK1:
        running_task_id = 1;
        new_task_event = true;
        break;

    case CE_START_TASK2:
        running_task_id = 2;
        new_task_event = true;
        break;

    case CE_START_TASK3:
        running_task_id = 3;
        new_task_event = true;
        break;

    case CE_START_TASK4:
        running_task_id = 4;
        new_task_event = true;
        break;

    case CE_START_TASK5:
        running_task_id = 5;
        new_task_event = true;
        break;

    case CE_START_TASK6:
        running_task_id = 6;
        new_task_event = true;
        break;

    case CE_START_TASK7:
        running_task_id = 7;
        new_task_event = true;
        break;

    default:
        break;
    }

}


void SwitchFullScreenCV(const std::string window_name) {

    if (cvGetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN) ==
        CV_WINDOW_NORMAL)
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    else
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

}
