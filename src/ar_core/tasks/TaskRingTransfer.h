//
// Created by nima on 13/06/17.
//

#ifndef ATAR_TASHOOK_H
#define ATAR_TASKHOOK_H


#include "src/ar_core/SimTask.h"
#include "src/ar_core/SimObject.h"
#include "src/ar_core/SimFiveLinkGripper.h"
#include <src/ar_core/Manipulator.h>
#include "src/ar_core/Rendering.h"
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include "src/ar_core/BulletVTKMotionState.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>


class TaskRingTransfer : public SimTask{
public:

    TaskRingTransfer(ros::NodeHandlePtr n);

    ~TaskRingTransfer();

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }

    // updates the task logic and the graphics_actors
    void TaskLoop();

    custom_msgs::TaskState GetTaskStateMsg();

    // resets the number of repetitions and task state;
    void ResetTask();


    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    void ResetCurrentAcquisition();


    /**
    * \brief This is the function that is handled by the desired pose thread.
     * It first reads the current poses of the tools and then finds the
     * desired pose from the mesh.
  *  **/
    void HapticsThread();

private:

    double board_dimensions[3];

    std::vector<std::vector<double>> gripper_link_dims;
    SimFiveLinkGripper * grippers[1];

    SimObject *hook_mesh;

    ros::Time time_last;

    // -------------------------------------------------------------------------
    // graphics
    // for not we use the same type of active constraint for both arms
    custom_msgs::ActiveConstraintParameters ac_parameters;

    Manipulator *master;


};

#endif //ATAR_TASKHOOK_H
