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
#include <kdl/frames.hpp>
#include <btBulletDynamicsCommon.h>
#include "Rendering.h"


//note about vtkSmartPointer:
// One way to create a VTK object is
//      vtkObject* MyObject = vtkObject::New();
// This method, however, can (and likely will) lead to memory management
// issues at some point or another. You must manually delete the object
//      MyObject->Delete();
// Instead if vtkSmartPointer is used the memory will be released when the
//      object having the pointer goes out of scope:
// vtkSmartPointer<vtkObject> MyObject = vtkSmartPointer<vtkObject>::New();

extern std::string                      MESH_DIRECTORY;


class SimTask{
public:
    SimTask(ros::NodeHandlePtr n, const double haptic_loop_rate)
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
    virtual std::vector< vtkSmartPointer <vtkProp> >GetActors() {return graphics_actors;};;;

    // returns the status of the task
    virtual custom_msgs::TaskState GetTaskStateMsg() = 0;;

    // minor reset
    virtual void ResetCurrentAcquisition(){};

    // full reset ;
    virtual void ResetTask() {};

    virtual void StartManipulatorToWorldFrameCalibration(const uint arm_id){};


protected:

    ros::NodeHandlePtr                      nh;

    Rendering *                             graphics;

    double                                  haptic_loop_rate;
    std::vector<vtkSmartPointer<vtkProp>>   graphics_actors;
    btDiscreteDynamicsWorld*                dynamics_world;

    //make sure to re-use collision shapes among rigid bodies whenever possible!
    btSequentialImpulseConstraintSolver*    solver;
    btBroadphaseInterface*                  overlappingPairCache;
    btCollisionDispatcher*                  dispatcher;
    btDefaultCollisionConfiguration*        collisionConfiguration;
};



#endif //ATAR_SIMTASK_H
