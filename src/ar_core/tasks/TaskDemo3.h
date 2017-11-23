//
// Created by nima on 14/11/17.
//

#ifndef ATAR_TASKDEMO3_H
#define ATAR_TASKDEMO3_H

// This demo shows how to create ar tasks in which 3D objects are overlaid on
// the images of a real camera. Print the charuco_d0_h4_w6_sl200_m10_ml150
// file on a paper and measure the length of the black squares and the marker
// squares with a ruler. Update the last two elements of the board_params
// parameter of the test3_ar.launch file with these measurements. When you run
// the task 4 virtual cubes should be placed on 4 on the corners of the board
// when the board is in the field of the view of the camera with a text mesh
// shown in the middle.

#include <src/ar_core/SimTask.h>

class TaskDemo3: public SimTask {
public:
    explicit TaskDemo3();
};


#endif //ATAR_TASKDEMO3_H
