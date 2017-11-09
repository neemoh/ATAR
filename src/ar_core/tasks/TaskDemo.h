//
// Created by nima on 13/06/17.
//

#ifndef ATAR_TASKTEST_H
#define ATAR_TASKTEST_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <src/ar_core/Rendering.h>
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"

#include "src/ar_core/SimTask.h"
#include "src/ar_core/SimObject.h"
#include "src/ar_core/Colors.hpp"
#include "src/ar_core/Forceps.h"
#include "src/ar_core/Manipulator.h"

class TaskDemo : public SimTask{
public:

    explicit TaskDemo(ros::NodeHandlePtr n);

    ~TaskDemo() override;

    // updates the task logic and the graphics_actors
    void TaskLoop() override;

    // a sparate thread that you can use for high refresh rate things like
    // providing haptic feedback
    void HapticsThread() override;

private:

    void StartManipulatorToWorldFrameCalibration(const uint arm_id) override;

private:

    SimObject *sphere[6];
    Forceps * forceps;

    Colors colors;
    Manipulator *master;

};

#endif //ATAR_TASKBULLETt_H
