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

    TaskDemo(ros::NodeHandlePtr n);

    ~TaskDemo();

    // updates the task logic and the graphics_actors
    void StepWorld();

    void StepPhysics();

    void HapticsThread();

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }

    custom_msgs::TaskState GetTaskStateMsg();

    void ResetCurrentAcquisition();

    // resets the number of repetitions and task state;
    void ResetTask();

private:

    void StartManipulatorToWorldFrameCalibration(const uint arm_id);

private:
    // task specific members
    SimObject *sphere[6];
    Forceps * forceps;
    KDL::Frame tool_pose;
    double grip_angle;


private:

        // task template members
    Colors colors;
    ros::Time time_last;

    custom_msgs::ActiveConstraintParameters ac_parameters;

    Manipulator *master;

};

#endif //ATAR_TASKBULLETt_H
