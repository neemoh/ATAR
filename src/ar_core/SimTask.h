//
// Created by nima on 25/05/17.
//

#ifndef ATAR_SIMTASK_H
#define ATAR_SIMTASK_H

#include <ros/ros.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vector>
#include <custom_msgs/TaskState.h>
#include <custom_msgs/ActiveConstraintParameters.h>
#include <kdl/frames.hpp>
#include <btBulletDynamicsCommon.h>


//note about vtkSmartPointer:
// One way to create a VTK object is
//      vtkObject* MyObject = vtkObject::New();
// This method, however, can (and likely will) lead to memory management
// issues at some point or another. You must manually delete the object
//      MyObject->Delete();
// Instead if vtkSmartPointer is used the memory will be released when the
//      object having the pointer goes out of scope:
// vtkSmartPointer<vtkObject> MyObject = vtkSmartPointer<vtkObject>::New();



class SimTask{
public:
    SimTask(ros::NodeHandle *n,
            const double haptic_loop_rate)
            :
            nh(n),
            haptic_loop_rate(haptic_loop_rate){};

    virtual ~SimTask() {};

    // The main loop. Updates physics, graphics and task logic
    virtual void StepWorld() {};

    // steps the physics simulation
    virtual void StepPhysics() {};

    // This is the function that is handled by the haptics thread.
    virtual void HapticsThread() = 0;

    // returns all the task graphics_actors to be sent to the rendering part
    virtual std::vector< vtkSmartPointer <vtkProp> >GetActors() {return graphics_actors;};

    // sets the pose of the tools
    virtual void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int
    tool_id) {};

    // sets the position of the gripper
    virtual void SetCurrentGripperpositionPointer(double &gripper_position, const int
    tool_id) {};

    // returns the status of the task
    virtual custom_msgs::TaskState GetTaskStateMsg() = 0;

    // returns the status of the change of the ac_param
    virtual bool IsACParamChanged() = 0;

    // returns the ac parameters
    virtual custom_msgs::ActiveConstraintParameters * GetACParameters(){};

    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    virtual void ResetCurrentAcquisition(){};

    // resets the number of repetitions and task state;
    virtual void ResetTask() {};

protected:

    ros::NodeHandle * nh;
    double haptic_loop_rate;
    std::vector<vtkSmartPointer<vtkProp>>   graphics_actors;
    btDiscreteDynamicsWorld*                dynamics_world;

    //make sure to re-use collision shapes among rigid bodies whenever possible!
    btSequentialImpulseConstraintSolver* solver;
    btBroadphaseInterface* overlappingPairCache;
    btCollisionDispatcher* dispatcher;
    btDefaultCollisionConfiguration* collisionConfiguration;
};



#endif //ATAR_SIMTASK_H
