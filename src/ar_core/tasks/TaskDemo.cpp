//
// Created by nima on 13/06/17.
//

#include "TaskDemo.h"
#include <custom_conversions/Conversions.h>
#include <boost/thread/thread.hpp>

TaskDemo::TaskDemo(ros::NodeHandlePtr n)
        :
        SimTask(n, 500)

{

    bool ar_mode = false;
    int n_views = 1;
    bool one_window_per_view = false;
    bool borders_off  = false;
    std::vector<int> view_resolution = {920, 640};
    std::vector<int> window_positions={100,50, 800, 50};
    // the only needed argument to construct a Renderer if the nodehandle ptr
    // The rest have default values.
    graphics = std::make_unique<Rendering>(n, view_resolution, ar_mode, n_views,
                             one_window_per_view, borders_off,window_positions);

    // Define a master manipulator
    master = new Manipulator(nh, "/sigma7/sigma0", "/pose", "/gripper_angle");

    // for correct calibration, the master needs the pose of the camera
    graphics->SetManipulatorInterestedInCamPose(master);

   

    // DEFINE OBJECTS
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor under the workspace of your task to prevent objects
    // from falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE, floor_dims);
    AddSimObjectToTask(floor);
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
        AddSimObjectToTask(board);
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
                                      sphere_dimensions, pose, density, 0.1,
                                      "",
                                      RESOURCES_DIRECTORY+"/texture/checker"
                                              ".png");

            // we can access all the properties of a VTK actor
//            sphere[i]->GetActor()->GetProperty()->SetColor(colors.BlueDodger);
            // same applies for bullet parameters
            sphere[i]->GetBody()->setFriction(1.5f);
            sphere[i]->GetBody()->setRollingFriction(0.5f);
            sphere[i]->GetBody()->setSpinningFriction(0.5f);

            // we need to add the rigid body to the dynamics workd and the actor
            // to the graphics_actors vector
            AddSimObjectToTask(sphere[i]);
        }

    }

    // -------------------------------------------------------------------------
    // Create 6 dynamic cubes
    SimObject *cube;
    {
        // define object dimensions
        std::vector<double> sphere_dimensions = {0.01, 0.01, 0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.05, 0.055, 0.05 ));

        // Define
        double density = 50000; // kg/m3
        //override the default friction value
        float friction = 0.2;

        for (int i = 0; i < 6; ++i) {

            pose.p = pose.p+ KDL::Vector(0.0001, 0.0, 0.008);

            cube = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                           sphere_dimensions, pose, density, friction);

            // we can access all the properties of a VTK actor
            cube->GetActor()->GetProperty()->SetColor(colors.Orange);

            // we need to add the rigid body to the dynamics workd and the actor
            // to the graphics_actors vector
            AddSimObjectToTask(cube);
        }

    }
    // -------------------------------------------------------------------------
    // Create SimForceps

    KDL::Frame forceps_init_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
    forceps_init_pose.M.DoRotZ(M_PI/2);
    forceps = new SimForceps(forceps_init_pose);

    AddSimMechanismToTask(forceps);

};

//------------------------------------------------------------------------------
void TaskDemo::TaskLoop() {


    // Read the pose and gripper angle of the master and assign it to our
    // virtual forceps
    forceps->SetPoseAndJawAngle(master->GetPoseWorld(),  master->GetGripper());

    // let's get the pose of the camera and incrementally rotate it for fun!
    KDL::Frame cam_p = graphics->GetMainCameraPose();
    cam_p.M.DoRotZ(-0.005);
    graphics->SetMainCameraPose(cam_p);


    // FYI you can access the pose of the objects:
    //    ROS_INFO("Sphere0 z: %f",sphere[0]->GetPose().p[2]);

}



void TaskDemo::HapticsThread() {

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}



// -----------------------------------------------------------------------------
void TaskDemo::StartManipulatorToWorldFrameCalibration(const uint arm_id) {

    master->DoArmToWorldFrameCalibration();
    // TODO : Update the implementation of this
//    ROS_INFO("Starting Arm 1 to world calibration->");
//    if(running_task_id>0) {
//        // if a task is running first stop it
//        graphics->RemoveAllActorsFromScene();
//        DeleteTask();
//        // then do the calibration
//        DoArmToWorldFrameCalibration(arm_id);
//        // run the task again
//        StartTask((uint)running_task_id);
//        graphics->AddActorsToScene(task_ptr->GetActors());
//    }
//    else // if no task is running just do the calibration
//        DoArmToWorldFrameCalibration(arm_id);
//
//    control_event = (int8_t)running_task_id;
}

TaskDemo::~TaskDemo() {

    ROS_INFO("Destructing TaskDemo.");

    delete master;

}


