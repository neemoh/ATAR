//
// Created by charm on 4/17/17.
//

#include "TaskHandler.h"
#include <custom_conversions/Conversions.h>
#include <src/ar_core/tasks/TaskDemo2.h>
#include "ControlEvents.h"
// tasks
#include "src/deprecated/TaskBuzzWire.h"
#include "src/ar_core/tasks/TaskDeformable.h"
#include "src/ar_core/tasks/TaskRingTransfer.h"
#include "src/ar_core/tasks/TaskSteadyHand.h"
#include "src/ar_core/tasks/TaskDemo1.h"

std::string RESOURCES_DIRECTORY;

// -----------------------------------------------------------------------------
TaskHandler::TaskHandler(std::string node_name)
        :
        task_ptr(nullptr)
{

    n = ros::NodeHandlePtr(new ros::NodeHandle(node_name));

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();

    if (n->getParam("recources_directory", RESOURCES_DIRECTORY))
        ROS_DEBUG("recources directory: %s", RESOURCES_DIRECTORY.c_str());
    else
        ROS_ERROR("Parameter '%s' is required. ",
                  n->resolveName("recources_directory").c_str());

    subscriber_control_events = n->subscribe(
            "/atar/control_events", 1, &TaskHandler::ControlEventsCallback, this);

    ROS_INFO("Task Handler is ready!");

}


// -----------------------------------------------------------------------------
bool TaskHandler::UpdateWorld() {

    if (control_event == CE_EXIT) {// Esc
        Cleanup();
        return false;
    }

    if(new_task_event)
        HandleTaskEvent();

    // Time performance debug
    ros::Time start =ros::Time::now();

    // update the moving graphics_actors
    if(task_ptr) {
        task_ptr->StepWorld();
        // arm calibration
        if(control_event== CE_CALIB_ARM1)
            task_ptr->StartManipulatorToWorldFrameCalibration(0);
    }

    // check time performance
    double dt = (ros::Time::now() - start).toNSec()/1000000;
    ROS_DEBUG_STREAM_COND((dt>35),"Slow execution! last loop took: " <<  dt);

    // if no task is running we need to spin
    if(!task_ptr)
        ros::spinOnce();

    return true;
}

// -----------------------------------------------------------------------------
void TaskHandler::HandleTaskEvent() {

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
void TaskHandler::StartTask(const uint task_id) {

    // create the task
    if(task_id ==1){
        // allocate anew dynamic task
        ROS_DEBUG("Starting new TestTask task. ");
        task_ptr   = new TaskDemo1(n);
    }
    else if(task_id ==2){
        ROS_DEBUG("Starting new BuzzWireTask task. ");
        task_ptr   = new TaskDemo2(n);
    }
    else if(task_id ==3){
        ROS_DEBUG("Starting new BuzzWireTask task. ");
        task_ptr   = new TaskSteadyHand(n);
    }
    else if(task_id ==4){
        ROS_DEBUG("Starting new TaskRingTransfer. ");
        task_ptr   = new TaskRingTransfer(n);
    }
    else if(task_id ==5){
        ROS_DEBUG("Starting new TaskDeformable . ");
        task_ptr   = new TaskDeformable(n);
    }
    else if(task_id ==6){

    }
    else if(task_id ==7) {

    }
    else if(task_id ==8) {

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
void TaskHandler::DeleteTask() {

    ROS_DEBUG("Interrupting haptics thread");
    haptics_thread.interrupt();
    ros::Rate sleep(50);
    sleep.sleep();
    delete task_ptr;
    task_ptr = nullptr;
}

// -----------------------------------------------------------------------------
void TaskHandler::Cleanup() {
    DeleteTask();
}

// -----------------------------------------------------------------------------
void TaskHandler::ControlEventsCallback(const std_msgs::Int8ConstPtr
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

