//
// Created by nima on 12/11/17.
//
/**
 * \class TaskDemo2
 * \brief This demo shows how to use a master device to interact with SimObjects
 *
 * **/

#include "TaskDemo2.h"
#include <boost/thread/thread.hpp>

//------------------------------------------------------------------------------
TaskDemo2::TaskDemo2(ros::NodeHandlePtr n):
        SimTask(n) {

    bool ar_mode = false;
    int n_views = 1;
    bool one_window_per_view = false;
    bool borders_off  = false;
    std::vector<int> view_resolution = {920, 640};
    std::vector<int> window_positions={300,50};
    // the only needed argument to construct a Renderer if the nodehandle ptr
    // The rest have default values.
    graphics = std::make_unique<Rendering>(n, view_resolution, ar_mode, n_views,
                                           one_window_per_view, borders_off,
                                           window_positions);

    // Define a master manipulator
    master = new Manipulator(nh, "/sigma7/sigma0", "/pose", "/gripper_angle");

    // for correct calibration, the master needs the pose of the camera
    graphics->SetManipulatorInterestedInCamPose(master);

    // DEFINE OBJECTS
    // -------------------------------------------------------------------------
    // Create SimForceps

    KDL::Frame forceps_init_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
    forceps_init_pose.M.DoRotZ(M_PI/2);
    forceps = new SimForceps(forceps_init_pose);

    AddSimMechanismToTask(forceps);
    // -------------------------------------------------------------------------
    // Create a board
    SimObject *board;
    {
        // define object dimensions
        std::vector<double> board_dimensions = {0.15, 0.10, 0.01};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion(0., 0., 0., 1.),
                        KDL::Vector(0.05, 0.05, -0.005));

        // we want a static floor so ObjectType is DYNAMIC and no density is
        // passed (default is zero)
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                              board_dimensions, pose);
        // add to simulation
        AddSimObjectToTask(board);
    }

    // -------------------------------------------------------------------------
    // Create 6 dynamic cubes
    SimObject *cube;

    {
        // define object dimensions
        std::vector<double> dimensions = {0.01, 0.01, 0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.07, 0.055, 0.05 ));

        for (int i = 0; i < 6; ++i) {
            // increment the pose of each object
            pose.p = pose.p+ KDL::Vector(0.0001, 0.0, 0.008);

            // construct the object. Sor Sphere and planes we can have a
            // texture image too!
            cube = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                                 dimensions, pose, 20000);
            // we need to add the SimObject to the task. Without this step
            // the object is not going to be used
            AddSimObjectToTask(cube);
        }
    }

}

//------------------------------------------------------------------------------
TaskDemo2::~TaskDemo2() {
    delete master;
}

//------------------------------------------------------------------------------
void TaskDemo2::TaskLoop() {

    // update the pose of the virtual forceps from the real manipulator
    forceps->SetPoseAndJawAngle(master->GetPoseWorld(),  master->GetGripper());

}

//------------------------------------------------------------------------------
void TaskDemo2::HapticsThread() {

    ros::Rate loop_rate(200);

    // define a publisher for force
    // calculate the force that you want to send at high freq
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}
