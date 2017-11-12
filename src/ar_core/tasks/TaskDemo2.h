//
// Created by nima on 12/11/17.
//
// This

#ifndef ATAR_TASKDEMO2_H
#define ATAR_TASKDEMO2_H

#include <src/ar_core/SimTask.h>
#include <src/ar_core/SimForceps.h>

class TaskDemo2 : public SimTask {
public:

    explicit TaskDemo2(ros::NodeHandlePtr n);

    ~TaskDemo2() override;

    // The main loop. Updates physics, graphics and task logic
    void TaskLoop() override;

    // The main loop. Updates physics, graphics and task logic
    void HapticsThread() override;

private:

    Manipulator * master;
    SimForceps * forceps;

};


#endif //ATAR_TASKDEMO2_H
