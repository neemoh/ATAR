//
// Created by charm on 4/17/17.
//

#include "ARCore.h"
#include <custom_conversions/Conversions.h>
#include "ControlEvents.h"
// tasks
#include "src/ar_core/tasks/TaskBuzzWire.h"
#include "src/ar_core/tasks/TaskDeformable.h"
#include "src/ar_core/tasks/TaskNeedle.h"
#include "src/ar_core/tasks/TaskRingTransfer.h"
#include "src/ar_core/tasks/TaskSteadyHand.h"
#include "src/ar_core/tasks/TaskDemo.h"

std::string MESH_DIRECTORY;

// -----------------------------------------------------------------------------
ARCore::ARCore(std::string node_name)
        :
        running_task_id(0), task_ptr(NULL)
{

    n = ros::NodeHandlePtr(new ros::NodeHandle(node_name));

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();

    if (n->getParam("mesh_directory", MESH_DIRECTORY)) {
        ROS_INFO("mesh directory: %s", MESH_DIRECTORY.c_str());
    } else
        ROS_ERROR("Parameter '%s' is required. ",
                  n->resolveName("MESH_DIRECTORY").c_str());

    subscriber_control_events = n->subscribe(
            "/atar/control_events", 1, &ARCore::ControlEventsCallback, this);}

//    // -------------------------------- CAM POSES ------------------------------
//    // We need to know the pose of the cameras with respect to the world frame
//    // coordinates. If the camera or the markers move the pose should be
//    // estimated by a node and here we subscribe to that topic. If on the
//    // other hand no cam/marker motion is involved the fixed pose of the left
//    // camera is read as a static parameter and the right one is calculated
//    // from the fixed tr hard coded here. If you are not using the dvrk
//    // endoscopic camera you need to estimate the left to right cam transform
//    // yourself and put it here:
//    std::vector<double> l_r_cams = {-0.00538475, 0.000299458, -0.000948875,
//                                    0.0016753, -0.00112252, -0.00358978, 0.999992};
//    conversions::PoseVectorToKDLFrame(l_r_cams, left_cam_to_right_cam_tr);
//
//    // we first try to read the poses as parameters and later update the
//    // poses if new messages are arrived on the topics
//    std::vector<double> temp_vec = std::vector<double>( 7, 0.0);
//    // left cam pose as parameter
//    if (n->getParam("/calibrations/world_frame_to_left_cam_frame", temp_vec)) {
//        conversions::PoseVectorToKDLFrame(temp_vec, pose_cam[0]);
//        conversions::KDLFrameToRvectvec(pose_cam[0], cam_rvec_curr[0], cam_tvec_curr[0]);
//        cam_tvec_avg[0] = cam_tvec_curr[0];
//        cam_rvec_avg[0] = cam_rvec_curr[0];
//
//        // right cam
//        pose_cam[1] = left_cam_to_right_cam_tr * pose_cam[0];
//        conversions::KDLFrameToRvectvec(pose_cam[1], cam_rvec_curr[1],
//                                        cam_tvec_curr[1]);
//        cam_tvec_avg[1] = cam_tvec_curr[1];
//        cam_rvec_avg[1] = cam_rvec_curr[1];
//
//        new_cam_pose[0] = true;
//        new_cam_pose[1] = true;
//    }



// -----------------------------------------------------------------------------
bool ARCore::UpdateWorld() {

    if (control_event == CE_EXIT) {// Esc
        Cleanup();
        return false;
    }

    if(new_task_event)
        HandleTaskEvent();

    // Time performance debug
    //        ros::Time start =ros::Time::now();

    // update the moving graphics_actors
    if(task_ptr) {
        task_ptr->StepWorld();
        // arm calibration
        if(control_event== CE_CALIB_ARM1)
            task_ptr->StartManipulatorToWorldFrameCalibration(0);
    }

    // check time performance
    //        std::cout <<  "it took: " <<
    //        (ros::Time::now() - start).toNSec() /1000000 << std::endl;


    // if no task is running we need to spin
    if(!task_ptr)
        ros::spinOnce();

    return true;
}

// -----------------------------------------------------------------------------
void ARCore::HandleTaskEvent() {

    if (new_task_event){

        ROS_INFO("Task %d Selected", running_task_id);

        //close tasks if it was already running
        if(task_ptr)
            DeleteTask();

        StartTask(running_task_id);

        new_task_event = false;
    }
}


// -----------------------------------------------------------------------------
void ARCore::StartTask(const uint task_id) {

    // create the task
    if(task_id ==1){
        // allocate anew dynamic task
        ROS_DEBUG("Starting new TestTask task. ");
        task_ptr   = new TaskDemo(n);
    }
    else if(task_id ==2){
        // starting the task
        ROS_DEBUG("Starting new BuzzWireTask task. ");
        task_ptr   = new TaskSteadyHand(n);
    }
    else if(task_id ==3){
        ROS_DEBUG("Starting new TaskNeedle. ");
        task_ptr   = new TaskNeedle(n);
    }
    else if(task_id ==4){
        ROS_DEBUG("Starting new TaskDeformable . ");
        task_ptr   = new TaskDeformable(n);
    }
    else if(task_id ==5){
        ROS_DEBUG("Starting new TaskRingTransfer. ");
        task_ptr   = new TaskRingTransfer(n);

    }
    else if(task_id ==6) {

    }
    else if(task_id ==7) {

    }
    else if(task_id == 8) {

    }
    if(task_ptr) {
        // assign the tool pose pointers
        ros::spinOnce();

        task_ptr->StepWorld();

        // bind the haptics thread
        haptics_thread = boost::thread(
                boost::bind(&SimTask::HapticsThread, task_ptr));
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
void ARCore::Cleanup() {
    DeleteTask();
//    delete graphics;
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
//            publish_overlayed_images = true;
            break;

        case CE_PUBLISH_IMGS_OFF:
//            publish_overlayed_images = false;
            break;

        case CE_TOGGLE_FULLSCREEN:
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

        case CE_START_TASK8:
            running_task_id = 8;
            new_task_event = true;
            break;

        default:
            break;
    }

}


// -----------------------------------------------------------------------------
//bool ARCore::GetNewCameraPoses(cv::Vec3d cam_rvec_out[2],
//                               cv::Vec3d cam_tvec_out[2]) {
//
//    // if one of the poses is not available estimate the other one through
//    // the left to right fixed transform
//    if (new_cam_pose[0] && !new_cam_pose[1]) {
//        pose_cam[1] = left_cam_to_right_cam_tr * pose_cam[0];
//        conversions::KDLFrameToRvectvec(pose_cam[1],
//                                        cam_rvec_curr[1],cam_tvec_curr[1]);
//    }
//    else if (!new_cam_pose[0] && new_cam_pose[1]) {
//        pose_cam[0] = left_cam_to_right_cam_tr.Inverse() * pose_cam[1];
//        conversions::KDLFrameToRvectvec(pose_cam[0],
//                                        cam_rvec_curr[0], cam_tvec_curr[0]);
//    }
//
//
//    double avg_factor;
//    n->param<double>("cam_pose_averaging_factor", avg_factor, 0.5);
//
//    // FIXME change to normal averaging with buffer.
//    // populate the out values
//    if (new_cam_pose[0] || new_cam_pose[1]) {
//
//        for (int k = 0; k < 2; ++k) {
//            if (ar_mode) {
//                // average the position to prevent small jitter when the board
//                // does not have good visibility
//                cam_tvec_avg[k] -= avg_factor * cam_tvec_avg[k];
//                cam_tvec_avg[k] += avg_factor * cam_tvec_curr[k];
//
//                // not mathematically legal, but should work...
//                cam_rvec_avg[k] -= avg_factor * cam_rvec_avg[k];
//                cam_rvec_avg[k] += avg_factor * cam_rvec_curr[k];
//
//                // to prevent jitter due to wrong board pose estimation we discard
//                // poses that are too different from the last pose
//                if ((cv::norm(cam_tvec_avg[k] - cam_tvec_curr[k]) < 0.05)
//                    && (cv::norm(cam_rvec_avg[k] - cam_rvec_curr[k]) < 0.1)) {
//
//                    cam_rvec_out[k] = cam_rvec_avg[k];
//                    cam_tvec_out[k] = cam_tvec_avg[k];
//                    new_cam_pose[k] = false;
////                    if (k == 1)
////                        return true;
//                }
//            }
//            else{ // ar_mode
//                // no averaging is needed in VR mode
//                cam_rvec_out[k] = cam_rvec_curr[k];
//                cam_tvec_out[k] = cam_tvec_curr[k];
//                //new_cam_pose[k] = false;
//            }
//        }
//        return true;
//    }
//
//    return false;
//}