//
// Created by nima on 08/06/17.
//

#ifndef ATAR_ODETASK_H
#define ATAR_ODETASK_H
#include "VTKTask.h"

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

#include "Rendering.h"
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <ode/ode.h>
#include <vtkMinimalStandardRandomSequence.h>



#define GEOMSPERBODY 1
#define MAX_CONTACTS 8
#define NUM_SPHERES 15
#define RAD_SPHERES 0.008
extern  dWorldID World;

extern dJointGroupID contactgroup;

struct MyObject {
    dBodyID Body;  // the dynamics body
    dGeomID Geom[GEOMSPERBODY];  // geometries representing this body
};


class ODETask : public VTKTask{
public:

    ODETask(const std::string stl_files_dir,
               const bool show_ref_frames, const bool num_tools,
               const bool with_guidance);

    ~ODETask();

    // returns all the task actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return actors;
    }
    // sets the pose of the tools
    void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int tool_id);

    // updates the task logic and the actors
    void UpdateActors();

    bool IsACParamChanged();

    // returns the ac parameters
    custom_msgs::ActiveConstraintParameters GetACParameters();

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
    void FindAndPublishDesiredToolPose();

    void InitODE();

    void CloseODE();

    void SimLoopODE();


    void DrawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb,
                      size_t obj_idx);

private:
    MyObject Objct[NUM_SPHERES+1];

    dSpaceID Space;
    std::vector<std::array<double, 3> > sphere_positions;

    std::string stl_files_dir;
    double board_dimensions[3];
    // -------------------------------------------------------------------------
    // graphics

    // for not we use the same type of active constraint for both arms
    custom_msgs::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl[2];
    KDL::Frame *tool_current_pose_kdl[2];

    vtkSmartPointer<vtkActor>                       d_board_actor;
    std::vector< vtkSmartPointer<vtkActor>>         d_sphere_actors;

};
static void nearCallback (void *data, dGeomID o1, dGeomID o2);



#endif //ATAR_ODETASK_H
