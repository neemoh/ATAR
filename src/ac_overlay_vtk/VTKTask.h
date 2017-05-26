//
// Created by nima on 25/05/17.
//

#ifndef ATAR_VTKTASK_H
#define ATAR_VTKTASK_H


#include <vtkProp.h>
#include <vtkPolyDataMapper.h>
#include <vector>
#include <custom_msgs/TaskState.h>
#include <kdl/frames.hpp>
#include <active_constraints/ActiveConstraintParameters.h>


class VTKTask{
public:
    VTKTask(const bool show_ref_frames,
            const bool biman,
            const bool with_guidance) :
            show_ref_frames(show_ref_frames),
            bimanual(biman),
            with_guidance(with_guidance){};

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

    // returns the status of the change of the ac_param
    virtual bool IsACParamChanged() = 0;


    // returns the ac parameters
    virtual active_constraints::ActiveConstraintParameters GetACParameters(){};

    virtual void FindAndPublishDesiredToolPose() = 0;

protected:
    bool bimanual;
    bool with_guidance;
    bool show_ref_frames;
    std::vector<vtkSmartPointer<vtkProp>>           actors;

};


#endif //ATAR_VTKTASK_H
