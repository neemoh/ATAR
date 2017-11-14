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
#include "Colors.hpp"
#include <memory>

extern std::string                      RESOURCES_DIRECTORY;


class SimTask{
public:

    explicit SimTask(ros::NodeHandlePtr n);

    virtual ~SimTask();

    // The main loop. Updates physics, graphics and task logic and it is
    // called from outside
    virtual void StepWorld();

    // This is the function that is handled by the haptics thread and can be
    // used for calculations that need to run at a high frequency.
    // also the ros spinning happens here (since you might have subscribers
    // in the haptics thread) so when you override it don't forget to include
    // ros spinning!
    virtual void HapticsThread();

    // minor reset
    virtual void ResetCurrentAcquisition(){};

    // full reset ;
    virtual void ResetTask() {};

    virtual void StartManipulatorToWorldFrameCalibration(const uint arm_id){};

    void AddSimObjectToTask(SimObject* obj);

    void AddSimMechanismToTask(SimMechanism* mech);

private:

    // This method is called from the StepWorld loop. The idea is to override
    // this in children tasks.
    virtual void TaskLoop() =0;

    // initialize the bullet realted things
    void InitBullet();

    // steps the physics simulation. Can be overridden if needed.
    virtual void StepPhysics();

protected:

    ros::NodeHandlePtr                      nh;
    ros::Time                               time_last;

    std::unique_ptr<Rendering>              graphics;
    Colors                                  colors;

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
