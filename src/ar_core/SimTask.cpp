//
// Created by nearlab on 07/11/17.
//

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "SimTask.h"


SimTask::SimTask()
        :
        nh(ros::NodeHandlePtr(new ros::NodeHandle("~"))),
        time_last(ros::Time::now()),
        graphics(nullptr),
        dynamics_world(nullptr){

    // Initialize Bullet Physics
    InitBullet();
}


// -----------------------------------------------------------------------------
void SimTask::InitBullet() {

    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration =
            std::make_unique<btDefaultCollisionConfiguration>();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher =
            std::make_unique<btCollisionDispatcher>(collisionConfiguration.get());

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = std::make_unique<btDbvtBroadphase>();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = std::make_unique<btSequentialImpulseConstraintSolver>();


    dynamics_world = new btDiscreteDynamicsWorld(dispatcher.get(),
                                                 overlappingPairCache.get(),
                                                 solver.get(),
                                                 collisionConfiguration.get());

    dynamics_world->setGravity(btVector3(0, 0, -10));


    btContactSolverInfo& info = dynamics_world->getSolverInfo();
    //optionally set the m_splitImpulsePenetrationThreshold (only used when m_splitImpulse  is enabled)
    //only enable split impulse position correction when the penetration is
    // deeper than this m_splitImpulsePenetrationThreshold, otherwise use the
    // regular velocity/position constraint coupling (Baumgarte).
    info.m_splitImpulsePenetrationThreshold = -0.02f;
    info.m_numIterations = 15;
    info.m_solverMode = SOLVER_USE_2_FRICTION_DIRECTIONS;


}

// -----------------------------------------------------------------------------
void SimTask::StepWorld() {

    // render
    if(graphics)
        graphics->Render();
    else
        throw std::runtime_error("Oops! It seems that the graphics was "
                                         "not constructed.");

    // step the world
    StepPhysics();

    // call the task loop
    TaskLoop();
}

// -----------------------------------------------------------------------------
void SimTask::StepPhysics() {

    double time_step = (ros::Time::now() - time_last).toSec();
    //std::cout << "time_step: " << time_step << std::endl;

    if(dynamics_world)
        dynamics_world->stepSimulation(btScalar(time_step), 100, 1.f/128.f);
    else
        throw std::runtime_error("Oops! It seems that the dynamics_world was "
                                         "not constructed.");
    time_last = ros::Time::now();

}


// -----------------------------------------------------------------------------
SimTask::~SimTask() {

    ROS_DEBUG("Destructing task. Num SimObjects: %lu", sim_objs.size());

    //remove the rigid bodies from the world
    for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
        dynamics_world->removeCollisionObject(dynamics_world->getCollisionObjectArray()[i]);

    // remove the constraints from the world
    for (int i = dynamics_world->getNumConstraints() - 1; i >= 0; i--)
        dynamics_world->removeConstraint(dynamics_world->getConstraint(i));



        //delete dynamics world
    for(auto i:sim_objs)
        delete i;
    delete dynamics_world;

}

void SimTask::AddSimObjectToTask(SimObject *obj) {

    if(obj) {
        if (obj->GetObjectType() != NOVISUALS) {
            if (graphics)
                graphics->AddActorToScene(obj->GetActor(), obj->IsShadowOn());
            else
                throw std::runtime_error("Oops! It seems that the graphics was "
                                                 "not constructed.");
        }

        if (obj->GetObjectType() != NOPHYSICS) {
            sim_objs.emplace_back(obj);
            dynamics_world->addRigidBody(obj->GetBody());
        }
    }
}

void SimTask::AddSimMechanismToTask(SimMechanism *mech) {

    if(mech) {
        auto sim_objs = mech->GetSimObjects();
        for (auto i: sim_objs)
            AddSimObjectToTask(i);

        auto constraints = mech->GetConstraints();
        for (auto i:constraints)
            dynamics_world->addConstraint(i);
    }

}

void SimTask::HapticsThread() {

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}
