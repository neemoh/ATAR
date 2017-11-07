//
// Created by nearlab on 07/11/17.
//

#include <ros/ros.h>
#include "SimTask.h"


SimTask::SimTask(ros::NodeHandlePtr n, const double haptic_loop_rate)
    :
    nh(n),
        haptic_loop_rate(haptic_loop_rate){ // Initialize Bullet Physics
        InitBullet();
}


void SimTask::InitBullet() {
    
    ///-----initialization_start-----
    
    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();
    
    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();
    
    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;
    
    
    dynamics_world = new btDiscreteDynamicsWorld(dispatcher,
        overlappingPairCache, solver,
        collisionConfiguration);
    
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
