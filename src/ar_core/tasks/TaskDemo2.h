//
// Created by nima on 12/11/17.
//
/**
 * \class TaskDemo2
 * \brief This demo shows how to use a master device to interact with SimObjects
 *
 * **/

#ifndef ATAR_TASKDEMO2_H
#define ATAR_TASKDEMO2_H

#include <src/ar_core/SimTask.h>
#include <src/ar_core/SimForceps.h>

class TaskDemo2 : public SimTask {
public:

    explicit TaskDemo2();

    ~TaskDemo2() override;

    // In this demo we use the loop to read the poses of our real
    // manipulators and update the poses of our virtual tools
    void TaskLoop() override;

    // This thread can run faster than the main thread and used for purposes
    // like sending information for haptic feedback
    void HapticsThread() override;

private:

    // these objects interfacew with the real manipulators through ros
    Manipulator * master[2];

    // this sphere will represent one of our tools
    SimObject * sphere_tool;

    // this object is a simulated mechanism that will represent one of our
    // tools.
    SimForceps * forceps;

};


#endif //ATAR_TASKDEMO2_H
