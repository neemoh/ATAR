//
// Created by nima on 13/06/17.
//
/**
 * \class TaskDemo1
 * \brief This demo shows how to use a master device to interact with SimObjects
 *
 * **/

#ifndef ATAR_TASKDEMO_H
#define ATAR_TASKDEMO_H

#include "src/ar_core/SimTask.h"
#include "src/ar_core/SimObject.h"


class TaskDemo1 : public SimTask{
public:

    explicit TaskDemo1(ros::NodeHandlePtr n);

    // updates the task logic and the graphics_actors
    void TaskLoop() override;

private:

    // SimObjects that are added to the task will be automatically deleted
    // when the task goes out of the scope.
    SimObject *sphere[6];

};

#endif //ATAR_TASKBULLETt_H
