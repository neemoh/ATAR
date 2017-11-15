//
// Created by nima on 4/17/17.
//

#ifndef ATAR_TASKHANDLER_H
#define ATAR_TASKHANDLER_H

#include "SimTask.h"
#include "Rendering.h"
#include <boost/thread/thread.hpp>
#include <std_msgs/Int8.h>
#include "ros/ros.h"

// This class loads the tasks. Its UpdateWorld method is called from the
// main and TaskHandler checks for control commands from the gui node.

class TaskHandler {
public:

    TaskHandler(std::string node_name);

    bool UpdateWorld();

    // this topic is used to control the task state from the recording node
    // during the acquisitions.
    void ControlEventsCallback(const std_msgs::Int8ConstPtr &msg);

private:
    // stop the running haptic thread (if any), destruct the previous task
    // (if any) and start a new task and thread.
    void HandleTaskEvent();

    // start a new task and thread.
    void StartTask(uint task_id);

    // stop the running haptic thread and destruct the  task object
    void DeleteTask();

    void Cleanup();

private:

    SimTask *task_ptr;

    boost::thread haptics_thread;


    bool new_task_event = false;

    uint running_task_id;
    int8_t control_event;

    ros::Subscriber subscriber_control_events;

};


#endif //ATAR_TASKHANDLER_H
