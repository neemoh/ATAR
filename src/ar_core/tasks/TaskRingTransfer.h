//
// Created by nima on 13/06/17.
//

#ifndef ATAR_TASHOOK_H
#define ATAR_TASKHOOK_H


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
#include "src/ar_core/FiveLinkGripper.h"
#include <vtkMinimalStandardRandomSequence.h>



class TaskHook : public SimTask{
public:

    TaskHook(const std::string mesh_files_dir,
            const bool show_ref_frames, const bool num_tools,
            const bool with_guidance);

    ~TaskHook();

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }
    // sets the pose of the tools
    void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int tool_id);

    // sets the position of the gripper
    void SetCurrentGripperpositionPointer(double &gripper_position, const int
    tool_id);

    // updates the task logic and the graphics_actors
    void StepWorld();

    bool IsACParamChanged();

    // returns the ac parameters
    custom_msgs::ActiveConstraintParameters * GetACParameters();

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

    void InitBullet();

    void StepPhysics();

private:

    double board_dimensions[3];

    std::vector<std::vector<double>> gripper_link_dims;
    FiveLinkGripper * grippers[1];
    uint counter=0;

    SimObject *hook_mesh;

    ros::Time time_last;

    // -------------------------------------------------------------------------
    // graphics
    // for not we use the same type of active constraint for both arms
    custom_msgs::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl[2];
    KDL::Frame *tool_current_pose_kdl[2];
    double * jaw_position[2];

};

#endif //ATAR_TASKHOOK_H
