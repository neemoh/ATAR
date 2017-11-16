//
// Created by nima on 16/11/17.
//

#include "TaskDemo4.h"

void TaskDemo4::TaskLoop() {

}

TaskDemo4::TaskDemo4() {

    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({640, 480}),
            /*ar_mode=*/ true);


    // Define a master manipulator
    slave[0] = new Manipulator("/dvrk/PSM1_DUMMY",
                                "/position_cartesian_current",
                                "/gripper_position_current");

    // for correct calibration, the master needs the pose of the camera
    graphics->SetManipulatorInterestedInCamPose(slave[0]);

}
