//
// Created by nima on 16/11/17.
//

#ifndef ATAR_TASKDEMO4_H
#define ATAR_TASKDEMO4_H


#include <src/ar_core/SimTask.h>

class TaskDemo4 : public SimTask {
public:
    TaskDemo4();

    void TaskLoop() override;

    void StartManipulatorToWorldFrameCalibration(const uint arm_id) override
    {
        slave[arm_id]->DoArmToWorldFrameCalibration();
    };


private:
    Manipulator *slave[2];
};


#endif //ATAR_TASKDEMO4_H
