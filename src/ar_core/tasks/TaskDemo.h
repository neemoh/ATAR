//
// Created by nima on 13/06/17.
//

#ifndef ATAR_TASKTEST_H
#define ATAR_TASKTEST_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"

#include "src/ar_core/SimTask.h"
#include "src/ar_core/SimObject.h"
#include "src/ar_core/Colors.hpp"
#include "src/ar_core/Forceps.h"


class TaskDemo : public SimTask{
public:

    TaskDemo(const std::string mesh_files_dir,
            const bool show_ref_frames, const bool num_tools,
            const bool with_guidance);

    ~TaskDemo();

    // updates the task logic and the graphics_actors
    void StepWorld();

    void StepPhysics();

    void HapticsThread();

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }

    // sets the pose of the tools
    void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int tool_id);

    // sets the position of the gripper
    void SetCurrentGripperpositionPointer(double &gripper_position, const int
    tool_id);

    custom_msgs::TaskState GetTaskStateMsg();

    bool IsACParamChanged();

    // returns the ac parameters
    custom_msgs::ActiveConstraintParameters * GetACParameters();

    void ResetCurrentAcquisition();

    // resets the number of repetitions and task state;
    void ResetTask();


    void InitBullet();


private:
    // task specific members
    SimObject *sphere[6];
    Forceps * forceps[2];


private:
    // task template members
    Colors colors;
    std::string mesh_files_dir;

    ros::Time time_last;

    custom_msgs::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl[2];
    KDL::Frame * tool_current_pose_kdl[2];
    double * gripper_position[2];
};

#endif //ATAR_TASKBULLETt_H
