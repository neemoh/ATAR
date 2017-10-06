//
// Created by nima on 25/05/17.
//

#ifndef ATAR_VTKTASK_H
#define ATAR_VTKTASK_H


#include <vtkSmartPointer.h>
#include <vtkProp.h>
#include <vtkPolyDataMapper.h>
#include <vector>
#include <custom_msgs/TaskState.h>
#include <kdl/frames.hpp>
#include <custom_msgs/ActiveConstraintParameters.h>
#include <ros/ros.h>

//n ote about vtkSmartPointer:
// One way to create a VTK object is
//      vtkObject* MyObject = vtkObject::New();
// This method, however, can (and likely will) lead to memory management
// issues at some point or another. You must manually delete the object
//      MyObject->Delete();
// Instead if vtkSmartPointer is used the memory will be released when the
//      object having the pointer goes out of scope:
// vtkSmartPointer<vtkObject> MyObject = vtkSmartPointer<vtkObject>::New();



class VTKTask{
public:
    VTKTask(
            const bool show_ref_frames,
            const bool biman,
            const bool with_guidance,
            const double haptic_loop_rate
        ) :
            show_ref_frames(show_ref_frames),
            bimanual(biman),
            with_guidance(with_guidance),
            haptic_loop_rate(haptic_loop_rate){};

    virtual ~VTKTask(){
    };

    // returns all the task actors to be sent to the rendering part
    virtual std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return actors;
    };

    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    virtual void ResetCurrentAcquisition(){};

    // resets the number of repetitions and task state;
    virtual void ResetTask() {};


    virtual custom_msgs::TaskState GetTaskStateMsg() = 0;

    // updates the task logic and the actors
    virtual void UpdateActors(){};

    // sets the pose of the tools
    virtual void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int
    tool_id) {};

    // sets the position of the gripper
    virtual void SetCurrentGripperpositionPointer(double &gripper_position, const int
    tool_id) {};

    // returns the status of the change of the ac_param
    virtual bool IsACParamChanged() = 0;


    // returns the ac parameters
    virtual custom_msgs::ActiveConstraintParameters * GetACParameters(){};

    virtual void FindAndPublishDesiredToolPose() = 0;

protected:

    bool bimanual;
    bool with_guidance;
    bool show_ref_frames;
    double haptic_loop_rate;
    std::vector<vtkSmartPointer<vtkProp>>           actors;

};



#endif //ATAR_VTKTASK_H
