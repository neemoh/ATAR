//
// Created by nima on 13/06/17.
//

#ifndef ATAR_TASKNEEDLE_H
#define ATAR_TASKNEEDLE_H


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
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"


class TaskNeedle : public SimTask{
public:

    TaskNeedle(ros::NodeHandlePtr nh);

    ~TaskNeedle();

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }

    // updates the task logic and the graphics_actors
    void TaskLoop();

    custom_msgs::TaskState GetTaskStateMsg();

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

    void StepPhysics();

    void UpdateGripperLinksPose(const KDL::Frame pose,
        const double grip_angle,
        const std::vector<std::vector<double> > gripper_link_dims,
                                SimObject* link_objects[]);


private:
    std::vector<std::array<double, 3> > sphere_positions;
    MyContactResultCallback result;

    double board_dimensions[3];
    std::vector<std::vector<double>> gripper_link_dims;

    SimObject *ring_mesh;
    SimObject *needle_mesh;
    SimObject *board;

    ros::Time time_last;

    btPairCachingGhostObject* ghostObject;
    // -------------------------------------------------------------------------
    // graphics

    // for not we use the same type of active constraint for both arms
    custom_msgs::ActiveConstraintParameters ac_parameters;

    Manipulator *master;


};

#endif //ATAR_TASKNEEDLE_H
