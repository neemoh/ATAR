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
                                "/posepublishing");

    // for correct calibration, the master needs the pose of the camera
//    graphics->SetManipulatorInterestedInCamPose(slave[0]);


    // -------------------------------------------------------------------------
    // Create one SimForceps to be connected to master[0]
    KDL::Frame forceps_init_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
    forceps_init_pose.M.DoRotZ(M_PI/2);
    forceps = new SimForceps(forceps_init_pose);
    // add to simulation
    AddSimMechanismToTask(forceps);


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
    graphics->AddActorToScene(task_coordinate_axes);
}



void TaskDemo4::TaskLoop() {

    // update the pose of the virtual forceps from the real manipulator
    forceps->SetPoseAndJawAngle(slave[0]->GetPoseWorld(), 0);
}
