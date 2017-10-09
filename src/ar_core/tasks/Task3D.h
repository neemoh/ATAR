//
// Created by Andrea on 28/07/2017.
//

#ifndef ATAR_TASKBULLETTEST_H
#define ATAR_TASKBULLETTEST_H


#include "src/ar_core/VTKTask.h"

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
#include "src/ar_core/BulletVTKObject.h"
#include <vtkMinimalStandardRandomSequence.h>



class Task3D : public VTKTask{
public:

    Task3D(const std::string mesh_files_dir,
                   const bool show_ref_frames, const bool num_tools,
                   const bool with_guidance);

    ~Task3D();

    // returns all the task actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return actors;
    }
    // sets the pose of the tools
    void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int tool_id);

    // sets the position of the gripper
    void SetCurrentGripperpositionPointer(double &gripper_position, const int
    tool_id);

    // updates the task logic and the actors
    void UpdateActors();

    bool IsACParamChanged();

    // returns the ac parameters
    custom_msgs::ActiveConstraintParameters * GetACParameters();

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
    void FindAndPublishDesiredToolPose();

    void InitBullet();

    void StepDynamicsWorld();

    void CheckCrossing();

    void ArrowManager();

    void ExitChecking();


private:
    std::vector<std::array<double, 3> > sphere_positions;

    enum class TaskState: uint8_t {Idle, Entry, Exit};

    // quidditch task
    int rings_number = 4;
    BulletVTKObject* ring[4];
    BulletVTKObject* hinge_cyl[4];
    KDL::Vector ideal_position[4];
    btHingeConstraint * hinges[4];
    double* radii;
    int target;
    std::vector<double> kine_dim;
    BulletVTKObject* arrow;
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
    BulletVTKObject* kine_p;
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

    //keep track of the shapes, we release memory at exit.
    //make sure to re-use collision shapes among rigid bodies whenever possible!
//    btAlignedObjectArray<btCollisionShape*> collisionShapes;
    btSequentialImpulseConstraintSolver* solver;
    btBroadphaseInterface* overlappingPairCache;
    btCollisionDispatcher* dispatcher;
    btDefaultCollisionConfiguration* collisionConfiguration;

    // -------------------------------------------------------------------------
    // graphics

    // for not we use the same type of active constraint for both arms
    custom_msgs::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl[2];
    KDL::Frame *tool_current_pose_kdl[2];
    double *gripper_position[2];
};

#endif //ATAR_TASKBULLETt_H


