//
// Created by nima on 13/06/17.
//

#include "TaskDemo.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>

TaskDemo::TaskDemo(const std::string mesh_files_dir,
                       const bool show_ref_frames, const bool biman,
                       const bool with_guidance)
    :
    SimTask(show_ref_frames, biman, with_guidance, 0) ,
    mesh_files_dir(mesh_files_dir),
    time_last(ros::Time::now())
{

    InitBullet();

    // -------------------------------------------------------------------------
    // static floor
    // always add a floor under the workspace of your task to prevent objects
    // from falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE,
                                     ObjectType::DYNAMIC, floor_dims);
    dynamics_world->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    // Create a floor
    SimObject *board;
    {
        // define object dimensions
        std::vector<double> board_dimensions = {0.15, 0.10, 0.01};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector( board_dimensions[0] / 3,
                               board_dimensions[1] / 2,
                               -board_dimensions[2]/ 2) );

        // we want a static floor so ObjectType is DYNAMIC and no density is
        // passed (default is zero)
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                              board_dimensions, pose);

        // we can access all the properties of a VTK actor
        board->GetActor()->GetProperty()->SetColor(colors.Gray);

        // we need to add the rigid body to the dynamics workd and the actor
        // to the graphics_actors vector
        dynamics_world->addRigidBody(board->GetBody());
        graphics_actors.push_back(board->GetActor());
    }
    // -------------------------------------------------------------------------
    // Create 6 dynamic spheres
    {
        // define object dimensions
        std::vector<double> sphere_dimensions = {0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.02, 0.05, 0.05 ));

        // Define
        double density = 50000; // kg/m3
        for (int i = 0; i < 6; ++i) {

            pose.p = pose.p+ KDL::Vector(0.0001, 0.0, 0.008);

            sphere[i] = new SimObject(ObjectShape::SPHERE, ObjectType::DYNAMIC,
                                   sphere_dimensions, pose, density);

            // we can access all the properties of a VTK actor
            sphere[i]->GetActor()->GetProperty()->SetColor(colors.BlueDodger);
            // same applies for bullet parameters
            sphere[i]->GetBody()->setFriction(1.5f);
            sphere[i]->GetBody()->setRollingFriction(0.5f);
            sphere[i]->GetBody()->setSpinningFriction(0.5f);

            // we need to add the rigid body to the dynamics workd and the actor
            // to the graphics_actors vector
            dynamics_world->addRigidBody(sphere[i]->GetBody());
            graphics_actors.push_back(sphere[i]->GetActor());

        }

    }

    // -------------------------------------------------------------------------
    // Create 6 dynamic cubes
    // objects can be defined locally too...
    {
        // define object dimensions
        std::vector<double> sphere_dimensions = {0.01, 0.01, 0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.02, 0.04, 0.05 ));

        // Define
        double density = 50000; // kg/m3
        //override the default friction value
        float friction = 0.2;

        for (int i = 0; i < 6; ++i) {

            pose.p = pose.p+ KDL::Vector(0.0001, 0.0, 0.008);

            SimObject cube(ObjectShape::BOX, ObjectType::DYNAMIC,
                                   sphere_dimensions, pose, density, friction);

            // we can access all the properties of a VTK actor
            cube.GetActor()->GetProperty()->SetColor(colors.Orange);

            // we need to add the rigid body to the dynamics workd and the actor
            // to the graphics_actors vector
            dynamics_world->addRigidBody(cube.GetBody());
            graphics_actors.push_back(cube.GetActor());
        }

    }
    // -------------------------------------------------------------------------
    // Create Forceps
    {
        KDL::Frame forceps_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
        forceps_pose.M.DoRotZ(M_PI/2);
        forceps[0] = new Forceps(mesh_files_dir, forceps_pose);

        forceps_pose.p.x(0.07);
        forceps[1] = new Forceps(mesh_files_dir, forceps_pose);

        for (int j = 0; j < 2; ++j) {
            forceps[j]->AddToWorld(dynamics_world);
            forceps[j]->AddToActorsVector(graphics_actors);
        }
    }
}


//------------------------------------------------------------------------------
void TaskDemo::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {
    tool_current_pose_kdl[tool_id] = &tool_pose;
}

//------------------------------------------------------------------------------
void TaskDemo::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskDemo::StepWorld() {

    //--------------------------------
    //use the tool pose for moving the virtual tools ...
    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);
    double grip_angle = 1;
    forceps[0]->SetPoseAndJawAngle(tool_pose, grip_angle);

    //--------------------------------
    // step the world
    StepPhysics();

    // you can access the pose of the objects:
//    ROS_INFO("Sphere0 z: %f",sphere[0]->GetPose().p[2]);

    //--------------------------------
    // Check if the task is completed
}


//------------------------------------------------------------------------------
bool TaskDemo::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters * TaskDemo::GetACParameters() {
    custom_msgs::ActiveConstraintParameters *msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState TaskDemo::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskDemo::ResetTask() {

    // save the measurements and the metrics and reset the initial conditions
    // to start a new repetition of the task
    ROS_INFO("Repetition completed. Resetting the task.");

    // save metrics


}

void TaskDemo::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
}


void TaskDemo::HapticsThread() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
                             ("/PSM1/tool_pose_desired", 10);
    if(bimanual)
        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
                                 ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {

        //        CalculatedDesiredToolPose();

        // publish desired poses
        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {

            // convert to pose message
            geometry_msgs::PoseStamped pose_msg;
            tf::poseKDLToMsg(tool_desired_pose_kdl[n_arm], pose_msg.pose);
            // fill the header
            pose_msg.header.frame_id = "/task_space";
            pose_msg.header.stamp = ros::Time::now();
            // publish
            pub_desired[n_arm].publish(pose_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}





void TaskDemo::InitBullet() {

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

void TaskDemo::StepPhysics() {

    double time_step = (ros::Time::now() - time_last).toSec();
    //std::cout << "time_step: " << time_step << std::endl;
    dynamics_world->stepSimulation(btScalar(time_step), 100, 1.f/128.f);
    time_last = ros::Time::now();

}


TaskDemo::~TaskDemo() {

    ROS_INFO("Destructing Demo task objects: %d",
             dynamics_world->getNumCollisionObjects());
    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamics_world->removeCollisionObject(obj);
        delete obj;
    }

    //delete dynamics world
    delete dynamics_world;

    //delete solver
    delete solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;

}


