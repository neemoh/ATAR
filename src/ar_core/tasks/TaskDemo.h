//
// Created by nima on 13/06/17.
//

#ifndef ATAR_TASKTEST_H
#define ATAR_TASKTEST_H


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
#include "src/ar_core/SimObject.h"
#include <vtkMinimalStandardRandomSequence.h>



class TaskTest : public VTKTask{
public:

    TaskTest(const std::string mesh_files_dir,
            const bool show_ref_frames, const bool num_tools,
            const bool with_guidance);

    ~TaskTest();

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
    void StepWorld();

    bool IsACParamChanged();

    // returns the ac parameters
    custom_msgs::ActiveConstraintParameters * GetACParameters();

    custom_msgs::TaskState GetTaskStateMsg();

    // check if the task is finished
    void EndChecking();

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

    void StepDynamicsWorld();


private:
    std::vector<std::array<double, 3> > sphere_positions;

    double board_dimensions[3];
    double peg_dimensions[3];
    double sides;
    bool out[4];
    ros::Time start_pause;
    SimObject* kine_box;
    SimObject* kine_scoop;
    SimObject* kine_cylinder_1;
    SimObject* peg4;
    SimObject* peg1;
    SimObject* peg2;
    SimObject* peg3;
    SimObject* cubes[4];
    bool count = 0;
    KDL::Frame peg_pose1;
    KDL::Frame peg_pose2;
    KDL::Frame peg_pose3;
    KDL::Frame peg_pose4;

    BulletVTKMotionState* motion_state_;
    std::vector<double> target_pos;
    KDL::Vector previous_point;
    btDiscreteDynamicsWorld* dynamicsWorld;
    ros::Time time_last;

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
    double * gripper_position[2];
//    vtkSmartPointer<vtkActor>                       d_board_actor;
//    std::vector< vtkSmartPointer<vtkActor>>         d_sphere_actors;

    int peg_type=1; // 1 = spheres, 0= cubes


};

#endif //ATAR_TASKBULLETt_H
