//
// Created by nima on 20/11/17.
//

#ifndef ATAR_TASKACTIVECONSTRAINTDESIGN_H
#define ATAR_TASKACTIVECONSTRAINTDESIGN_H


#include <src/ar_core/SimTask.h>
#include <vtkCellLocator.h>
#include <vtkLineSource.h>

class TaskActiveConstraintDesign : public SimTask {

public:
    TaskActiveConstraintDesign();

    ~TaskActiveConstraintDesign();

    void TaskLoop() override;


private:
    SimObject *sphere_tool;
    Manipulator * master;
    vtkSmartPointer<vtkLineSource> line;
    vtkSmartPointer<vtkLineSource> closestLine = vtkLineSource::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper;
    vtkSmartPointer<vtkCellLocator> cellLocator;
    bool start = true;
    KDL::Frame mesh_pose;
    KDL::Frame ball_pose;

    double lastPoint[3];
};



#endif //ATAR_TASKACTIVECONSTRAINTDESIGN_H
