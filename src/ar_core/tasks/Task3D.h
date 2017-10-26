//
// Created by Andrea on 28/07/2017.
//

#ifndef ATAR_TASKBULLETTEST_H
#define ATAR_TASKBULLETTEST_H


#include "src/ar_core/SimTask.h"

#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkImageActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkSTLReader.h>
#include <vtkProperty.h>
#include <vtkParametricTorus.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLocator.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkCornerAnnotation.h>

#include "src/ar_core/Rendering.h"
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include "src/ar_core/BulletVTKMotionState.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <btBulletDynamicsCommon.h>
#include "src/ar_core/SimObject.h"
#include <vtkMinimalStandardRandomSequence.h>
#include <src/ar_core/Manipulator.h>


class Task3D : public SimTask{
public:

    Task3D();

    ~Task3D();

    // returns all the task actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }

    // updates the task logic and the graphics_actors
    void StepWorld();

    custom_msgs::TaskState GetTaskStateMsg();

    // check if the task is finished
    void TaskEvaluation();

    // resets the number of repetitions and task state;
    void ResetTask();


    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    void ResetCurrentAcquisition();


    /**
    * \brief This is the function that is handled by the desired pose thread.
     * It first reads the current poses of the tools and then finds the
     * desired pose from the mesh.
  *  **/
    void HapticsThread();

    void InitBullet();

    void StepPhysics();

    void CheckCrossing();

    void ArrowManager();

    void ExitChecking();


private:
    std::vector<std::array<double, 3> > sphere_positions;

    enum class TaskState: uint8_t {Idle, Entry, Exit};

    // quidditch task
    int rings_number = 4;
    SimObject* ring[4];
    SimObject* hinge_cyl[4];
    KDL::Vector ideal_position[4];
    btHingeConstraint * hinges[4];
    double* radii;
    int target;
    std::vector<double> kine_dim;
    SimObject* arrow;
    std::vector<std::vector<int>> index;
    int path = 0;
    TaskState task_state;
    KDL::Vector distance;
    double threshold=0.005;
    double transf[4];
    KDL::Vector arrow_posit;
    double arrow_x, arrow_y, arrow_z, arrow_w;
    double var;
    KDL::Rotation rot;
    //


    custom_msgs::TaskState task_state_msg;
    bool out[4];
    SimObject* kine_p;
    float height=0.035;
    btDiscreteDynamicsWorld* dynamicsWorld;
    ros::Time time_last;
    double color[3];

    //Metrics
    bool cond=0;
    bool touch=0;
    // Timing
    double time;
    ros::Time begin;
    float axes_dist=0;

    unsigned char n_rep=1;


    KDL::Vector pointer_posit;

    Manipulator *master;

};

#endif //ATAR_TASKBULLETt_H


