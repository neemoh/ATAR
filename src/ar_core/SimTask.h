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
#include <kdl/frames.hpp>
#include <btBulletDynamicsCommon.h>
#include "Rendering.h"
#include "SimObject.h"
#include "SimMechanism.h"


extern std::string                      MESH_DIRECTORY;


class SimTask{
public:
    SimTask(ros::NodeHandlePtr n, const double haptic_loop_rate);

    virtual ~SimTask();

    // The main loop. Updates physics, graphics and task logic
    virtual void StepWorld();

    // The main loop. Updates physics, graphics and task logic
    virtual void TaskLoop() =0;

    // This is the function that is handled by the haptics thread.
    virtual void HapticsThread() = 0;

    // minor reset
    virtual void ResetCurrentAcquisition(){};

    // full reset ;
    virtual void ResetTask() {};

    virtual void StartManipulatorToWorldFrameCalibration(const uint arm_id){};

    void AddSimObjectToTask(SimObject* obj);

    void AddSimMechanismToTask(SimMechanism* mech);

private:
    void InitBullet();

    // steps the physics simulation
    virtual void StepPhysics();

protected:

    ros::NodeHandlePtr                      nh;
    ros::Time                               time_last;

    std::unique_ptr<Rendering>              graphics;

    double                                  haptic_loop_rate;
    std::vector<vtkSmartPointer<vtkProp>>   graphics_actors;
    std::vector<SimObject*>                 sim_objs;
    btDiscreteDynamicsWorld *               dynamics_world;

    //make sure to re-use collision shapes among rigid bodies whenever possible!
    std::unique_ptr<btSequentialImpulseConstraintSolver>    solver;
    std::unique_ptr<btBroadphaseInterface>            overlappingPairCache;
    std::unique_ptr<btCollisionDispatcher>            dispatcher;
    std::unique_ptr<btDefaultCollisionConfiguration>  collisionConfiguration;
};



#endif //ATAR_SIMTASK_H
