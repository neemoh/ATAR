//
// Created by nima on 20/11/17.
//

#ifndef ATAR_TASKACTIVECONSTRAINTDESIGN_H
#define ATAR_TASKACTIVECONSTRAINTDESIGN_H


#include <src/ar_core/SimTask.h>
#include <vtkCellLocator.h>
#include <vtkLineSource.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <src/ar_core/SimDrawPath.h>


class TaskActiveConstraintDesign : public SimTask {

public:
    TaskActiveConstraintDesign();

    ~TaskActiveConstraintDesign() override;

    void TaskLoop() override;
    
    void StartManipulatorToWorldFrameCalibration(const uint arm_id) override
    {
        slave->DoArmToWorldFrameCalibration();
    };
private:

    void FindClosestPointToMesh(double closest_point[]);


private:
    SimObject *sphere_tool;
    Manipulator * master;
    Manipulator * slave;

    int count=0;
    // path
    SimDrawPath path;

    vtkSmartPointer<vtkLineSource> closestLine = vtkLineSource::New();
    vtkSmartPointer<vtkCellLocator> cellLocator;
    bool start = true;
    KDL::Frame mesh_pose;

    KDL::Frame master_initial_pose_for_cam_move;
    KDL::Frame camera_initial_pose_for_cam_move;
    bool first_iteration_cam_move=true;
};



#endif //ATAR_TASKACTIVECONSTRAINTDESIGN_H
