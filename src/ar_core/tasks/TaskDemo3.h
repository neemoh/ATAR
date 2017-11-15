//
// Created by nima on 14/11/17.
//

#ifndef ATAR_TASKDEMO3_H
#define ATAR_TASKDEMO3_H


#include <src/ar_core/SimTask.h>

class TaskDemo3: public SimTask {
public:
    explicit TaskDemo3();

    void TaskLoop() override;

};


#endif //ATAR_TASKDEMO3_H
