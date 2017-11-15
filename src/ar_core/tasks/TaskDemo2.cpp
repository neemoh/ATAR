//
// Created by nima on 12/11/17.
//

#include "TaskDemo2.h"
#include <boost/thread/thread.hpp>

//------------------------------------------------------------------------------
TaskDemo2::TaskDemo2()
{

    // create a rendering object with desired parameters
    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({920, 640}),
            /*ar_mode=*/false,
            /*n_views=*/1,
            /*one_window_per_view=*/false,
            /*borders_off=*/false,
            /*window_positions=*/std::vector<int>({300,50}));

    // Define a master manipulator
    master[0] = new Manipulator("/dvrk/PSM1_DUMMY",
                                "/position_cartesian_current",
                                "/gripper_position_current");

    // for correct calibration, the master needs the pose of the camera
    graphics->SetManipulatorInterestedInCamPose(master[0]);

    // Define a second master manipulator
    master[1] = new Manipulator("/dvrk/PSM2_DUMMY",
                                "/position_cartesian_current",
                                "/gripper_position_current");
    graphics->SetManipulatorInterestedInCamPose(master[1]);

    // DEFINE OBJECTS
    // -------------------------------------------------------------------------
    // Create one SimForceps to be connected to master[0]
    KDL::Frame forceps_init_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
    forceps_init_pose.M.DoRotZ(M_PI/2);
    forceps = new SimForceps(forceps_init_pose);
    // add to simulation
    AddSimMechanismToTask(forceps);

    // --------------------------------------------------------
    // create a sphere and use it as a tool for master[1]
    sphere_tool = new SimObject(ObjectShape::SPHERE, ObjectType::KINEMATIC,
                                std::vector<double>(1,0.005));
    sphere_tool->GetActor()->GetProperty()->SetColor(colors.OrangeRed);

    // --------------------------------------------------------
    // create a floor
    SimObject* plane;
    plane = new SimObject(ObjectShape::PLANE, ObjectType::DYNAMIC,
                          std::vector<double>({0.15,0.10}),
                          KDL::Frame(KDL::Vector(0.075, 0.05, 0.001)), 0, 0.9,
                          RESOURCES_DIRECTORY+"/texture/persian_carpet.jpg");
    // add to simulation
    AddSimObjectToTask(plane);

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
            cube->GetActor()->GetProperty()->SetColor(colors.Coral);

            // we need to add the SimObject to the task. Without this step
            // the object is not going to be used
            AddSimObjectToTask(cube);
        }
    }

}

//------------------------------------------------------------------------------
TaskDemo2::~TaskDemo2() {
    delete master[0];
    delete master[1];
}

//------------------------------------------------------------------------------
void TaskDemo2::TaskLoop() {

    // update the pose of the virtual forceps from the real manipulator
    forceps->SetPoseAndJawAngle(master[0]->GetPoseWorld(),
                                master[0]->GetGripper());

    // update the position of the spherical tool
    sphere_tool->SetKinematicPose(master[1]->GetPoseWorld());
}

//------------------------------------------------------------------------------
void TaskDemo2::HapticsThread() {

    ros::Rate loop_rate(500);

    // for example we can define a publisher for force
    // calculate the force that you want to send at high freq...


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}
