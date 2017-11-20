//
// Created by nima on 20/11/17.
//

#ifndef ATAR_TASKACTIVECONSTRAINTDESIGN_H
#define ATAR_TASKACTIVECONSTRAINTDESIGN_H


#include <src/ar_core/SimTask.h>

class TaskActiveConstraintDesign : public SimTask {

public:
    TaskActiveConstraintDesign();

    void TaskLoop() override;

};


#endif //ATAR_TASKACTIVECONSTRAINTDESIGN_H
