//
// Created by nima on 16/11/17.
//

#include <vtkAxesActor.h>
#include <src/ar_core/SimForceps.h>
#include "TaskDemo4.h"

TaskDemo4::TaskDemo4() {

    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({640, 480}),
            /*ar_mode=*/ true);


    // Define a master manipulator
    slave[0] = new Manipulator("lwr",
                                "/posepublishing",
                                "/gripperAnglepublishing");

    // for correct calibration, the master needs the pose of the camera
//    graphics->SetManipulatorInterestedInCamPose(slave[0]);


    // -------------------------------------------------------------------------
    // Create one SimForceps to be connected to master[0]
    KDL::Frame forceps_init_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
    forceps_init_pose.M.DoRotZ(M_PI/2);
    gripper = new SimGripperLarge(forceps_init_pose);
    // add to simulation
    AddSimMechanismToTask(gripper);


    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::LINE_SHAFT);
    task_coordinate_axes->SetTipType(vtkAxesActor::SPHERE_TIP);
    graphics->AddActorToScene(task_coordinate_axes, false);


    // --------------------------------------------------------
    // create a floor
    SimObject* plane;
    plane = new SimObject(ObjectShape::PLANE, ObjectType::DYNAMIC,
                          std::vector<double>({0.15,0.10}),
                          KDL::Frame(KDL::Vector(0.075, 0.05, 0.001)), 0, 0.9);
    plane->GetActor()->GetProperty()->SetColor(colors.GrayLight);

    // add to simulation
    AddSimObjectToTask(plane);
    // -------------------------------------------------------------------------
    // Create 6 dynamic cubes
    SimObject *cube;
    {
        // define object dimensions
        std::vector<double> dimensions = {0.02, 0.02, 0.008};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.07, 0.055, 0.05 ));

        for (int i = 0; i < 6; ++i) {
            // increment the pose of each object
            pose.p = pose.p+ KDL::Vector(0.001, 0.0, 0.008);

            // construct the object. Sor Sphere and planes we can have a
            // texture image too!
            cube = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                                 dimensions, pose, 20000);
            cube->GetActor()->GetProperty()->SetColor(colors.BlueNavy);

            // we need to add the SimObject to the task. Without this step
            // the object is not going to be used
            AddSimObjectToTask(cube);
        }
    }
}



void TaskDemo4::TaskLoop() {

    // update the pose of the virtual forceps from the real manipulator
    gripper->SetPoseAndJawAngle(slave[0]->GetPoseWorld(),
                                slave[0]->GetGripperAngles());
}
